// tab5_map.ino - GNSS-following vector map on the M5Stack Tab5 (ESP32-P4).
//
// Pins and GNSS parsing come from the working tab5_gnss_sensors sketch.
//   DIP: TX pos 1, RX pos 1 -> module TX = G7, module RX = G6
//        PPS pos 3 (optional) -> G51
//
// Task layout
//   core 0, prio 5   gnss        UART drain and NMEA parsing
//   core 0, prio 2   loop()      compositing, touch, status overlay
//   core 1, prio 1   tilerender  PMTiles read, inflate, decode, rasterise
//
// The renderer sits alone on core 1 at the lowest priority. A tile takes
// several hundred milliseconds on this part, and must never be able to delay
// the serial drain - which is exactly what the old triple-pumpGnss() dance
// was working around. With the drain in a preemptible task it goes away.
//
// SETUP: copy your extract to the card as /local.pmtiles.

#include <M5Unified.h>
#include <SD.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <WiFi.h>

#include "gnss.h"
#include "mapengine.h"
#include "wifistore.h"
#include "portal.h"

constexpr int      PIN_GNSS_TX = 7;    // module transmits here -> ESP32 RX
constexpr int      PIN_GNSS_RX = 6;    // module listens here   -> ESP32 TX
constexpr int      PIN_PPS     = 51;
constexpr uint32_t GNSS_BAUD   = 38400;

static const char *PMT_PATH = "/local.pmtiles";

// ---- ESP-Hosted (WiFi) wiring ----------------------------------------------
// The P4 has no radio of its own; WiFi comes from an ESP32-C6 companion over
// SDIO. That link is on slot 1 with its own dedicated pins, entirely separate
// from the SD card on slot 0 (GPIO 43/44/39-42) - there is no bus conflict,
// despite the failure mode looking exactly like one.
//
// These are normally supplied by the m5stack_tab5 variant. Setting them here
// as well means the sketch works when built against the generic esp32p4
// board, where those defines are absent and ESP-Hosted has no idea where to
// find the C6.
constexpr int PIN_C6_CLK = 12;
constexpr int PIN_C6_CMD = 13;
constexpr int PIN_C6_D0  = 11;
constexpr int PIN_C6_D1  = 10;
constexpr int PIN_C6_D2  = 9;
constexpr int PIN_C6_D3  = 8;
constexpr int PIN_C6_RST = 15;

// Must run before anything touches the WiFi stack.
static void wifiSetPins() {
    bool ok = WiFi.setPins(PIN_C6_CLK, PIN_C6_CMD, PIN_C6_D0, PIN_C6_D1,
                           PIN_C6_D2, PIN_C6_D3, PIN_C6_RST);
    Serial.printf("wifi: setPins clk=%d cmd=%d d0..d3=%d,%d,%d,%d rst=%d -> %s\n",
                  PIN_C6_CLK, PIN_C6_CMD, PIN_C6_D0, PIN_C6_D1,
                  PIN_C6_D2, PIN_C6_D3, PIN_C6_RST, ok ? "ok" : "FAILED");
}

// Bring the radio up on its own and report what happened. Worth doing as a
// distinct step: if the C6 link is down, the MAC reads as all zeros, which
// distinguishes "no companion chip" from "wrong password" - two failures that
// otherwise look identical from the application side.
static bool wifiRadioUp() {
    wifiSetPins();
    if (!WiFi.mode(WIFI_STA)) { Serial.println("wifi: WiFi.mode(STA) failed"); return false; }
    delay(200);

    String mac = WiFi.macAddress();
    Serial.printf("wifi: station MAC %s\n", mac.c_str());
    if (mac == "00:00:00:00:00:00" || mac.length() == 0) {
        Serial.println("wifi: MAC is zero - ESP-Hosted link to the C6 is not up.");
        Serial.println("      Check the board selection (esp32:esp32:m5stack_tab5)");
        Serial.println("      and that the C6 still has its SDIO WiFi firmware.");
        return false;
    }
    int n = WiFi.scanNetworks();
    Serial.printf("wifi: scan found %d network%s\n", n, n == 1 ? "" : "s");
    for (int i = 0; i < n && i < 8; i++)
        Serial.printf("      %-32s %4d dBm  %s\n",
                      WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                      WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured");
    return true;
}

// Speed-driven zoom with hysteresis. A single threshold makes anything
// hovering near it flip repeatedly, and every flip invalidates all nine tiles.
static const uint8_t Z_FAST = 12, Z_MID = 13, Z_SLOW = 14;
static const uint32_t ZOOM_HOLD_MS = 8000;
static uint32_t g_lastZoomChange = 0;

// Try the stored credential first. The portal only comes up if there is no
// usable credential or the join fails, so a working device never stops to ask.
// Touch-and-hold during the first two seconds forces setup, which is the
// escape hatch for changing networks without reflashing.
static bool connectWifi(uint32_t timeout_ms) {
    WifiCred c;
    if (!wifistore_load(&c)) { Serial.println("wifi: no usable stored credential"); return false; }

    Serial.printf("wifi: joining '%s' using %s\n", c.ssid,
                  c.is_psk ? "derived PSK" : "passphrase");
    WiFi.persistent(false);
    // The stored value is the 64-hex PSK, which the supplicant accepts in
    // place of a passphrase - the passphrase itself was never written down.
    WiFi.begin(c.ssid, c.secret);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) delay(100);

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("wifi: connected, IP %s, RSSI %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        return true;
    }
    Serial.printf("wifi: join failed, status %d\n", (int)WiFi.status());
    WiFi.disconnect(true);
    return false;
}

static bool wantsSetup() {
    uint32_t t0 = millis();
    while (millis() - t0 < 2000) {
        M5.update();
        if (M5.Touch.getCount()) return true;
        delay(20);
    }
    return false;
}

static bool mountSD(const char **bus) {
    if (SD_MMC.begin("/sdcard", false)) { *bus = "SDMMC 4-bit"; return true; }
    if (SD_MMC.begin("/sdcard", true))  { *bus = "SDMMC 1-bit"; return true; }
    if (SD.begin())                     { *bus = "SPI";         return true; }
    return false;
}

static void pickZoom(const GnssFix &fix) {
    if (!gnss_fine(fix)) return;
    uint32_t now = millis();
    if (now - g_lastZoomChange < ZOOM_HOLD_MS) return;

    uint8_t cur = map_zoom(), want;
    double v = fix.speedKmh;
    // Separate up and down thresholds; the dead band between them stops a
    // vehicle sitting near a boundary from thrashing the grid.
    if (v > 60.0)                        want = Z_FAST;
    else if (v > 45.0 && cur == Z_FAST)  want = Z_FAST;
    else if (v > 20.0)                   want = Z_MID;
    else if (v > 12.0 && cur == Z_MID)   want = Z_MID;
    else                                 want = Z_SLOW;

    if (want != cur) { map_set_zoom(want, fix); g_lastZoomChange = now; }
}

static void drawStatus(const GnssFix &fix) {
    MapStats st; map_stats(&st);
    char buf[128];
    const int W = M5.Display.width();

    bool have = (fix.status == 'A');
    M5.Display.fillRect(0, 0, W, 52, have ? TFT_DARKGREEN : 0x6000);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_WHITE);

    if (have) {
        snprintf(buf, sizeof buf, "%.5f %.5f  z%u  %.0f km/h  %s HDOP %.1f sats %d",
                 fix.lat, fix.lon, map_zoom(), fix.speedKmh,
                 fix.mode == 3 ? "3D" : fix.mode == 2 ? "2D" : "--",
                 fix.hdop, fix.sats);
    } else {
        snprintf(buf, sizeof buf, "acquiring - open sky, 30-90s cold start   sats %d  %lu sent",
                 fix.sats, (unsigned long)gnss_sentences());
    }
    M5.Display.drawString(buf, 12, 6);

    snprintf(buf, sizeof buf, "tiles %lu  queue %lu  drop %lu  empty %lu  fail %lu  last %lums",
             (unsigned long)st.rendered, (unsigned long)st.queue_depth,
             (unsigned long)st.dropped, (unsigned long)st.notfound,
             (unsigned long)st.failed, (unsigned long)st.last_render_ms);
    M5.Display.setTextColor(TFT_LIGHTGREY);
    M5.Display.drawString(buf, 12, 28);

    // Stale-link warning: the module going quiet looks identical to "no fix"
    // unless it is called out separately.
    if (millis() - fix.lastSentence > 3000) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.drawString("NO GNSS DATA", W - 200, 6);
    }
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(3);                 // landscape, as in the GNSS sketch
    M5.Display.fillScreen(M5.Display.color565(228, 226, 220));

    Serial.begin(115200);
    M5.Power.setExtOutput(true);               // powers the M135
    delay(500);

    // No Wire.begin() anywhere: M5.begin() already configured the internal
    // I2C bus, and reconfiguring it breaks touch, the RTC and the IMU.

    Serial.printf("\n=== Tab5 map ===\nPSRAM %u KB free\n",
                  (unsigned)(ESP.getFreePsram() / 1024));

    const char *bus = "none";
    if (!mountSD(&bus)) {
        Serial.println("SD mount failed");
        M5.Display.setTextColor(TFT_RED);
        M5.Display.drawString("SD mount failed", 20, 100);
        return;
    }
    Serial.printf("SD via %s\n", bus);

    // WiFi is optional: the map runs entirely offline from the local archive.
    // Setup is only forced when asked for, or when there is nothing stored.
    wifistore_diag();

    bool forced = wantsSetup();
    bool radio = wifiRadioUp();

    if (!radio) {
        Serial.println("wifi: radio unavailable, continuing offline");
    } else if (forced || !wifistore_exists()) {
        Serial.println(forced ? "wifi: setup forced by touch"
                              : "wifi: no stored credential, starting portal");
        M5.Display.fillScreen(TFT_BLACK);
        if (!portal_run(300000))
            Serial.println("wifi: portal exited without saving");
    } else if (!connectWifi(12000)) {
        Serial.println("wifi: stored credential did not work, starting portal");
        M5.Display.fillScreen(TFT_BLACK);
        portal_run(300000);
    }
    M5.Display.fillScreen(M5.Display.color565(228, 226, 220));

    // GNSS first, and at high priority: the FIFO overflows if the drain is
    // starved, and the renderer will happily saturate its core.
    if (!gnss_start(PIN_GNSS_TX, PIN_GNSS_RX, GNSS_BAUD, PIN_PPS, 0, 5))
        Serial.println("gnss task failed to start");

    if (!map_begin(PMT_PATH, Z_SLOW, 1, 1)) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.drawString("map init failed - see serial", 20, 100);
        return;
    }
    Serial.println("running");
}

void loop() {
    M5.update();

    GnssFix fix;
    gnss_get(&fix);

    map_update(fix);
    pickZoom(fix);

    // ~15 fps. The map only changes when a tile commits or the marker moves,
    // neither of which happens at frame rate.
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last >= 66) {
        last = now;
        map_draw(fix);
        drawStatus(fix);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}
