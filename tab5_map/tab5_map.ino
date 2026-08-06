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

#include <Arduino.h>
#include <time.h>
#include <M5Unified.h>
#include <SD.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <WiFi.h>

#include <sunset.h>

#include "gnss.h"
#include "mapengine.h"
#include "wifistore.h"
#include "portal.h"
#include "netsource.h"
#include "tilecache.h"
#include "mapconfig.h"
#include "style.h"

constexpr int      PIN_GNSS_TX = 7;    // module transmits here -> ESP32 RX
constexpr int      PIN_GNSS_RX = 6;    // module listens here   -> ESP32 TX
constexpr int      PIN_PPS     = 51;
constexpr uint32_t GNSS_BAUD   = 38400;

// The offline floor. Fetched tiles cache alongside it under /t/<build>/;
// this archive only has to cover the low zooms.
static const char *PMT_PATH = "/world.pmtiles";

// The clock runs in UTC throughout - displayed with a trailing Z so there is
// no ambiguity about which it is.
//
// Local time would mean either a hardcoded zone, wrong the moment the device
// travels, or embedded DST rules that go stale as governments change them.
// GNSS and the tile build dates are both UTC already, so keeping one clock
// removes a whole category of off-by-an-hour and off-by-a-day bugs.

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
// Working zoom levels come from mapconfig.h so the engine and this file
// cannot disagree. Both default to Z_FLOOR, which pins the map to a single
// level - no speed-driven switching, and a quarter of the tiles to cache.
//
// z15 is the ceiling if you do want two: the Protomaps planet build carries
// zoom 0 to 15 only, so z16 returns NOTFOUND everywhere.
static const uint8_t Z_WIDE = Z_LEVEL_WIDE, Z_CLOSE = Z_LEVEL_CLOSE;
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
        // Only the date matters, so no timezone is configured - everything
        // downstream works in UTC. The sync is asynchronous; netsource polls
        // for completion rather than blocking here.
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        Serial.println("wifi: SNTP requested");
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

// DURABILITY
//
// The cache flushes its index every 256 writes, so an abrupt power cut can
// leave that many tiles present in the blob but absent from the index. They
// are not lost - every record carries a header, and startup rescans to
// recover them - but the restart cost is avoidable.
//
// Two cheap measures shrink the window to nearly nothing in practice:
//
//   1. Flush whenever the renderer goes quiet. Tiles arrive in bursts around
//      a grid shift; a couple of seconds later nothing is in flight and the
//      index can be written with no contention.
//
//   2. Flush on any power-button activity, since every shutdown gesture
//      starts with a press. Whether the PMU gives firmware a chance to act
//      before cutting the rails is board-specific, so this is opportunistic -
//      the rescan path remains the actual guarantee.
static void flushIfIdle() {
    static uint32_t lastActivity = 0;
    static uint32_t lastPending = 0;

    uint32_t pending = tilecache_pending();
    if (pending != lastPending) { lastPending = pending; lastActivity = millis(); }
    if (!pending) return;

    MapStats st; map_stats(&st);
    if (st.queue_depth) return;                  // still rendering, stay out of the way
    if (millis() - lastActivity < 2000) return;  // let a burst finish

    tilecache_flush();
    lastPending = 0;
}

// Cache warming: a visible button in the corner, with a confirmation step.
//
// Radius 7 is 15x15 tiles - about 13 km at z15, 27 km at z14 - which takes
// several minutes over HTTP. That is too much to start by accident, and an
// invisible whole-screen tap target would do exactly that every time the
// device was picked up.
static const int PREFETCH_RADIUS = 7;

// ---- day / night -----------------------------------------------------------
// There is no ambient light sensor on this board, so the palette is driven by
// the sun's actual position - which the device can compute exactly, having a
// position from GNSS and a UTC date from GNSS or SNTP. That beats a fixed
// clock time, which would be wrong by hours across a year and wrong by more
// if the device travels.
//
// Everything here works in UTC, matching the rest of the firmware, so the
// SunSet timezone offset is zero and its results are minutes past UTC
// midnight.
enum ThemeMode { THEME_AUTO = 0, THEME_DAY, THEME_NIGHT };
static ThemeMode g_themeMode = THEME_AUTO;

static SunSet g_sun;
static bool   g_sunValid = false;
static double g_sunriseMin = 0, g_sunsetMin = 0;
static int    g_sunDay = -1;

// Backlight levels. Night is dim enough not to ruin dark adaptation but
// still readable; day is full, since sunlight is the harder problem.
static const uint8_t BRIGHT_DAY = 255, BRIGHT_NIGHT = 60;
static uint8_t g_brightness = BRIGHT_DAY;

static bool sunIsUp(const GnssFix &fix) {
    if (fix.status != 'A' || !fix.utc[0]) return true;   // assume day if unsure

    time_t now = time(nullptr);
    if (now < 1767225600) return true;                   // clock not set yet
    struct tm t;
    gmtime_r(&now, &t);

    // Recompute once a day, or when the position moves far enough to matter.
    if (t.tm_yday != g_sunDay) {
        g_sun.setPosition(fix.lat, fix.lon, 0);          // 0 = UTC
        g_sun.setCurrentDate(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
        g_sunriseMin = g_sun.calcSunrise();
        g_sunsetMin  = g_sun.calcSunset();
        g_sunDay = t.tm_yday;
        g_sunValid = true;
        int rh = ((int)g_sunriseMin % 1440) / 60, rm = ((int)g_sunriseMin % 1440) % 60;
        int sh = ((int)g_sunsetMin  % 1440) / 60, sm = ((int)g_sunsetMin  % 1440) % 60;
        Serial.printf("sun: rise %02d:%02dZ set %02d:%02dZ%s at %.3f,%.3f\n",
                      rh, rm, sh, sm,
                      g_sunsetMin >= 1440 ? " (next UTC day)" : "",
                      fix.lat, fix.lon);
    }
    if (!g_sunValid) return true;

    double nowMin = t.tm_hour * 60.0 + t.tm_min;

    // SunSet returns minutes past midnight in the timezone it was given, and
    // we give it UTC - so a location whose evening falls after 00:00Z gets a
    // sunset past 1440. At this longitude sunset is 00:47Z the following day,
    // reported as 24:47. Normalising and handling the wrap is not optional
    // here; without it the palette flips to night for the 47 minutes after
    // UTC midnight while the sun is still up.
    double rise = fmod(g_sunriseMin, 1440.0); if (rise < 0) rise += 1440.0;
    double set  = fmod(g_sunsetMin,  1440.0); if (set  < 0) set  += 1440.0;

    if (rise < set) return nowMin >= rise && nowMin < set;   // ordinary day
    if (rise > set) return nowMin >= rise || nowMin < set;   // spans midnight

    // Equal means no crossing at all: polar day or polar night, and SunSet
    // cannot tell us which. Daylight is the safer guess - a map too bright
    // is a nuisance, a map too dark at noon is unusable.
    return true;
}

static void applyTheme(const GnssFix &fix) {
    bool wantDark;
    switch (g_themeMode) {
        case THEME_DAY:   wantDark = false; break;
        case THEME_NIGHT: wantDark = true;  break;
        default:          wantDark = !sunIsUp(fix); break;
    }
    if (wantDark != map_is_dark()) map_set_dark(wantDark);

    uint8_t want = wantDark ? BRIGHT_NIGHT : BRIGHT_DAY;
    if (want != g_brightness) {
        g_brightness = want;
        M5.Display.setBrightness(want);
    }
}

// ---- footer buttons --------------------------------------------------------
// Three across the bottom: cache, theme, screen off. The map is clipped out
// of this strip (FOOTER_H in mapengine) so they are not fighting it for
// pixels every frame.
static const int BTN_H = 54, BTN_M = 12;

// The touch target is taller than the drawn button. A 54 px outline is a
// small thing to hit on a moving vehicle, and there is nothing else along
// the bottom edge to steal a press from - so the target extends upward into
// the map and downward to the screen edge.
static const int BTN_PAD_TOP = 26, BTN_PAD_SIDE = 6;

enum { BTN_CACHE = 0, BTN_THEME, BTN_SLEEP, BTN_COUNT };

static uint32_t g_confirmUntil = 0;      // armed state for the cache button
static bool     g_screenOff = false;
static uint32_t g_lastTouchMs = 0;

static void buttonRect(int i, int *x, int *y, int *w, int *h) {
    int total = M5.Display.width() - BTN_M * (BTN_COUNT + 1);
    *w = total / BTN_COUNT;
    *h = BTN_H;
    *x = BTN_M + i * (*w + BTN_M);
    *y = M5.Display.height() - BTN_H - BTN_M;
}

static int buttonAt(int px, int py) {
    for (int i = 0; i < BTN_COUNT; i++) {
        int x, y, w, h; buttonRect(i, &x, &y, &w, &h);
        int hx = x - BTN_PAD_SIDE, hw = w + BTN_PAD_SIDE * 2;
        int hy = y - BTN_PAD_TOP;
        int hh = M5.Display.height() - hy;      // down to the screen edge
        if (px >= hx && px < hx + hw && py >= hy && py < hy + hh) return i;
    }
    return -1;
}

// Waking is restricted to the middle ninth of the screen.
//
// The device is meant to be carried, and an edge brush against a bag or a
// leg should not light it up and drain the battery. The centre requires a
// deliberate, flat-handed press, and it is also the one region no button
// occupies - so a wake tap can never be mistaken for a button tap.
static bool inWakeZone(int px, int py) {
    int W = M5.Display.width(), H = M5.Display.height();
    return px >= W / 3 && px < 2 * W / 3 &&
           py >= H / 3 && py < 2 * H / 3;
}

static void drawButton(int i, const char *label, uint16_t bg) {
    int x, y, w, h; buttonRect(i, &x, &y, &w, &h);
    M5.Display.fillRoundRect(x, y, w, h, 10, bg);
    M5.Display.drawRoundRect(x, y, w, h, 10, TFT_WHITE);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(2);
    M5.Display.drawString(label, x + w / 2, y + h / 2);
    M5.Display.setTextDatum(top_left);
}

static void drawFooter() {
    if (g_screenOff) return;

    bool armed = (millis() < g_confirmUntil);
    bool busy  = map_prefetch_busy();
    bool net   = (WiFi.status() == WL_CONNECTED);

    char label[40];
    if (busy)       snprintf(label, sizeof label, "caching %d%%", map_prefetch_progress());
    else if (armed) snprintf(label, sizeof label, "tap to confirm");
    else if (!net)  snprintf(label, sizeof label, "set up wifi");
    else            snprintf(label, sizeof label, "cache %d km",
                             (int)(((2 * PREFETCH_RADIUS + 1) * 40075.0
                                    * 0.74 / (1 << DATA_ZOOM_OF(Z_FLOOR)))));
    drawButton(BTN_CACHE, label,
               busy  ? TFT_DARKGREY :
               armed ? TFT_ORANGE   :
               net   ? M5.Display.color565(40, 70, 150)
                     : M5.Display.color565(70, 70, 70));

    // The theme button names the mode, and in auto also shows which way it
    // currently resolves - otherwise "auto" tells you nothing about why the
    // screen looks the way it does.
    const char *tl = g_themeMode == THEME_DAY   ? "day"
                   : g_themeMode == THEME_NIGHT ? "night"
                   : (map_is_dark() ? "auto - night" : "auto - day");
    drawButton(BTN_THEME, tl,
               g_themeMode == THEME_AUTO ? M5.Display.color565(60, 90, 60)
                                         : M5.Display.color565(70, 70, 90));

    drawButton(BTN_SLEEP, "screen off", M5.Display.color565(70, 70, 70));
}

static void screenOff() {
    // Draw the wake target before the backlight goes down, so it is clear
    // where to press. An unmarked centre-only zone is undiscoverable.
    int W = M5.Display.width(), H = M5.Display.height();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.drawRoundRect(W / 3, H / 3, W / 3, H / 3, 16, TFT_DARKGREY);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_DARKGREY);
    M5.Display.setTextSize(2);
    M5.Display.drawString("touch here to wake", W / 2, H / 2);
    M5.Display.setTextDatum(top_left);
    delay(700);

    g_screenOff = true;
    map_set_visible(false);
    M5.Display.setBrightness(0);
    M5.Display.fillScreen(TFT_BLACK);
    // GNSS and the render worker are untouched: the position stays current
    // and tiles keep arriving, so waking is instant rather than a cold start.
    Serial.println("screen: off (GPS and rendering continue)");
}

static void screenOn() {
    g_screenOff = false;
    map_set_visible(true);
    M5.Display.setBrightness(g_brightness);
    M5.Display.fillScreen(style_background());
    Serial.println("screen: on");
}

static void handleTouch() {
    static bool wasDown = false;
    bool down = M5.Touch.getCount() > 0;
    bool tapped = down && !wasDown;
    wasDown = down;
    if (!tapped) return;

    g_lastTouchMs = millis();

    // Waking takes a press in the middle ninth, and that press does nothing
    // else - waking straight into a button would let one tap turn the screen
    // off again.
    if (g_screenOff) {
        auto w = M5.Touch.getDetail();
        if (inWakeZone(w.x, w.y)) screenOn();
        return;
    }

    auto t = M5.Touch.getDetail();
    int b = buttonAt(t.x, t.y);
    if (b != BTN_CACHE) g_confirmUntil = 0;

    switch (b) {
    case BTN_SLEEP:
        screenOff();
        break;

    case BTN_THEME:
        g_themeMode = (ThemeMode)((g_themeMode + 1) % 3);
        Serial.printf("theme: %s\n",
                      g_themeMode == THEME_AUTO ? "auto" :
                      g_themeMode == THEME_DAY  ? "day" : "night");
        break;

    case BTN_CACHE:
        if (map_prefetch_busy()) break;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("wifi: opening setup portal from button");
            wifiSetPins();
            portal_run(300000);
            if (WiFi.status() == WL_CONNECTED)
                configTime(0, 0, "pool.ntp.org", "time.nist.gov");
            M5.Display.fillScreen(style_background());
            break;
        }
        if (millis() < g_confirmUntil) {
            g_confirmUntil = 0;
            map_prefetch_start(PREFETCH_RADIUS, Z_WIDE, Z_CLOSE);
        } else {
            g_confirmUntil = millis() + 5000;
            Serial.println("prefetch: tap again within 5s to start");
        }
        break;

    default:
        break;
    }
}

static void handlePowerButton() {
    // Any press at all is treated as "something is about to happen".
    // Cheap to act on, and the alternative is guessing which gesture the
    // PMU maps to shutdown on this particular board.
    if (M5.BtnPWR.wasPressed() || M5.BtnPWR.wasClicked()) {
        Serial.println("power: button activity, flushing cache");
        tilecache_flush();
    }
}

// ---- boot screen -----------------------------------------------------------
// Startup takes several seconds - SD mount, wifi association, SNTP, archive
// open - and a blank panel through all of it gives no indication whether the
// device is working or hung. Each step reports as it happens.
static int  g_bootLine = 0;
static bool g_bootActive = true;

static void bootBegin() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(3);
    M5.Display.drawString("Tab5 Map", 40, 36);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_DARKGREY);
    M5.Display.drawString("starting up", 40, 76);
    g_bootLine = 0;
    g_bootActive = true;
}

static void bootStepEx(const char *msg, bool ok, bool pending) {
    if (!g_bootActive) return;
    int y = 130 + g_bootLine * 30;
    if (y > M5.Display.height() - 40) return;
    // A pending line is overwritten in place by its result, and the result is
    // often shorter than the message it replaces ("connecting to wifi" ->
    // "wifi 192.168.5.62"). Without clearing the row first, the tail of the
    // longer text stays on screen underneath the new one.
    M5.Display.fillRect(0, y - 2, M5.Display.width(), 28, TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(pending ? TFT_YELLOW : (ok ? TFT_GREEN : TFT_RED));
    M5.Display.drawString(pending ? "..." : (ok ? " ok " : "fail"), 40, y);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.drawString(msg, 110, y);
    if (!pending) g_bootLine++;
    Serial.printf("boot: %-6s %s\n", pending ? "..." : (ok ? "ok" : "FAIL"), msg);
}

// No default arguments: the Arduino preprocessor copies them into the
// prototype it generates, and the definition then repeats them, which is a
// compile error rather than a warning.
static void bootStep(const char *msg)          { bootStepEx(msg, true, false); }
static void bootStepFail(const char *msg)      { bootStepEx(msg, false, false); }
static void bootStepBusy(const char *msg)      { bootStepEx(msg, true, true); }

static void bootEnd() {
    g_bootActive = false;
    M5.Display.fillScreen(style_background());
}

static bool mountSD(const char **bus) {
    if (SD_MMC.begin("/sdcard", false)) { *bus = "SDMMC 4-bit"; return true; }
    if (SD_MMC.begin("/sdcard", true))  { *bus = "SDMMC 1-bit"; return true; }
    if (SD.begin())                     { *bus = "SPI";         return true; }
    return false;
}

static void pickZoom(const GnssFix &fix) {
    // Nothing to choose between when both levels are the same, and skipping
    // early avoids the hysteresis timer holding a change that never comes.
    if (Z_WIDE == Z_CLOSE) return;
    if (!gnss_fine(fix)) return;
    uint32_t now = millis();
    if (now - g_lastZoomChange < ZOOM_HOLD_MS) return;

    uint8_t cur = map_zoom(), want;
    double v = fix.speedKmh;
    // Separate up and down thresholds; the dead band between them stops a
    // vehicle sitting near a boundary from thrashing the grid, and every
    // change invalidates all nine tiles.
    if (v > 25.0)                         want = Z_WIDE;
    else if (v > 15.0 && cur == Z_WIDE)   want = Z_WIDE;
    else                                  want = Z_CLOSE;

    if (want != cur) { map_set_zoom(want, fix); g_lastZoomChange = now; }
}

// Clock and battery, right-aligned in the status bar. Always UTC.
//
// The clock prefers the system time, which SNTP sets and the RTC holds
// across a reboot; GNSS is the fallback, correct as soon as there is a fix.
// Both sources are UTC, so neither needs converting.
static void drawClockBattery(const GnssFix &fix) {
    const int W = M5.Display.width();
    char buf[48];

    bool haveTime = false;
    struct tm lt;
    time_t now = time(nullptr);
    if (now > 1767225600) {                 // 2026-01-01, so plainly set
        gmtime_r(&now, &lt);
        haveTime = true;
    } else if (fix.utc[0] && fix.status == 'A') {
        // NMEA UTC is hhmmss.sss; no date arithmetic, so this is raw UTC.
        int hh = (fix.utc[0]-'0')*10 + (fix.utc[1]-'0');
        int mm = (fix.utc[2]-'0')*10 + (fix.utc[3]-'0');
        snprintf(buf, sizeof buf, "%02d:%02dZ", hh, mm);
        M5.Display.setTextDatum(top_right);
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.drawString(buf, W - 12, 6);
        M5.Display.setTextDatum(top_left);
        return;
    }

    int pct = M5.Power.getBatteryLevel();
    bool charging = M5.Power.isCharging();

    if (haveTime && pct >= 0)
        snprintf(buf, sizeof buf, "%02d:%02dZ   %s%d%%",
                 lt.tm_hour, lt.tm_min, charging ? "+" : "", pct);
    else if (haveTime)
        snprintf(buf, sizeof buf, "%02d:%02dZ", lt.tm_hour, lt.tm_min);
    else if (pct >= 0)
        snprintf(buf, sizeof buf, "%s%d%%", charging ? "+" : "", pct);
    else
        return;

    // Colour follows the battery, since that is the part worth noticing at a
    // glance on a device you are carrying.
    uint16_t col = TFT_WHITE;
    if (!charging && pct >= 0) {
        if (pct <= 10)      col = TFT_RED;
        else if (pct <= 25) col = TFT_ORANGE;
    } else if (charging) {
        col = TFT_GREENYELLOW;
    }
    M5.Display.setTextDatum(top_right);
    M5.Display.setTextColor(col);
    M5.Display.drawString(buf, W - 12, 6);
    M5.Display.setTextDatum(top_left);
}

static void drawStatus(const GnssFix &fix) {
    MapStats st; map_stats(&st);
    char buf[128];
    const int W = M5.Display.width();

    // The map is clipped to leave these rows alone (see STATUS_H in
    // mapengine), so the bar can be repainted only when its text actually
    // changes. Both halves of that are needed: redrawing a filled rect every
    // frame is what caused the flash, and skipping the redraw without the
    // clip would simply let the map paint over it.
    static char last[300] = "";
    static uint32_t lastDraw = 0;

    bool have = (fix.status == 'A');
    char statusLine1[128];
    if (have) {
        snprintf(statusLine1, sizeof statusLine1,
                 "%.5f %.5f  z%u  %.0f km/h  %s HDOP %.1f",
                 fix.lat, fix.lon, map_zoom(), fix.speedKmh,
                 fix.mode == 3 ? "3D" : fix.mode == 2 ? "2D" : "--",
                 fix.hdop);
    } else {
        snprintf(statusLine1, sizeof statusLine1,
                 "acquiring - open sky, 30-90s cold start   sats %d  %lu sent",
                 fix.sats, (unsigned long)gnss_sentences());
    }

    NetStats ns; netsource_stats(&ns);
    CacheStats cs; tilecache_stats(&cs);
    snprintf(buf, sizeof buf,
             "tiles %lu q%lu  c%lu n%lu  render %lums  blit %lu/%lums  blob %lu  %s%s",
             (unsigned long)st.rendered, (unsigned long)st.queue_depth,
             (unsigned long)ns.cache_hits, (unsigned long)ns.net_hits,
             (unsigned long)st.last_render_ms,
             (unsigned long)st.last_draw_ms, (unsigned long)st.max_draw_ms,
             (unsigned long)cs.entries,
             ns.build[0] ? ns.build : "none", ns.online ? "" : " (offline)");
    if (map_prefetch_busy()) {
        char pf[64];
        snprintf(pf, sizeof pf, "  PREFETCH %d%%", map_prefetch_progress());
        strncat(buf, pf, sizeof buf - strlen(buf) - 1);
    }

    // Include the minute in the change test, or the clock would sit stale
    // until something else on the bar happened to change.
    time_t nowt = time(nullptr);
    char combined[300];
    snprintf(combined, sizeof combined, "%s|%s|%ld",
             statusLine1, buf, (long)(nowt / 60));
    if (strcmp(combined, last) == 0 && millis() - lastDraw < 2000) return;
    strncpy(last, combined, sizeof last - 1);
    lastDraw = millis();

    M5.Display.fillRect(0, 0, W, 52,
                        have ? (map_is_dark() ? M5.Display.color565(10, 40, 20)
                                              : TFT_DARKGREEN)
                             : 0x6000);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.drawString(statusLine1, 12, 6);
    M5.Display.setTextColor(TFT_LIGHTGREY);
    M5.Display.drawString(buf, 12, 28);
    drawClockBattery(fix);

    // Stale-link warning: the module going quiet looks identical to "no fix"
    // unless it is called out separately.
    if (millis() - fix.lastSentence > 3000) {
        M5.Display.setTextDatum(top_right);
        M5.Display.setTextColor(TFT_RED);
        M5.Display.drawString("NO GNSS", W - 12, 30);
        M5.Display.setTextDatum(top_left);
    }
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(3);                 // landscape, as in the GNSS sketch
    M5.Display.setBrightness(BRIGHT_DAY);
    M5.Display.fillScreen(TFT_BLACK);

    Serial.begin(115200);
    M5.Power.setExtOutput(true);               // powers the M135
    delay(500);

    // No Wire.begin() anywhere: M5.begin() already configured the internal
    // I2C bus, and reconfiguring it breaks touch, the RTC and the IMU.

    Serial.printf("\n=== Tab5 map ===\nPSRAM %u KB free\n",
                  (unsigned)(ESP.getFreePsram() / 1024));

    bootBegin();

    const char *bus = "none";
    bootStepBusy("mounting SD card");
    if (!mountSD(&bus)) {
        bootStepFail("SD card - not detected");
        M5.Display.setTextColor(TFT_RED);
        M5.Display.setTextSize(2);
        M5.Display.drawString("Insert a card and restart", 40,
                              M5.Display.height() - 60);
        g_bootActive = false;
        return;
    }
    {
        char m[48]; snprintf(m, sizeof m, "SD card via %s", bus);
        bootStep(m);
    }

    // WiFi is optional: the map runs entirely offline from the local archive.
    // Setup is only forced when asked for, or when there is nothing stored.
    wifistore_diag();

    bootStepBusy("touch now to force wifi setup");
    bool forced = wantsSetup();

    bootStepBusy("starting wifi radio");
    bool radio = wifiRadioUp();
    if (radio) bootStep("wifi radio ready"); else bootStepFail("wifi radio unavailable");

    if (!radio) {
        Serial.println("wifi: radio unavailable, continuing offline");
    } else if (forced || !wifistore_exists()) {
        Serial.println(forced ? "wifi: setup forced by touch"
                              : "wifi: no stored credential, starting portal");
        M5.Display.fillScreen(TFT_BLACK);
        if (!portal_run(300000))
            Serial.println("wifi: portal exited without saving");
    } else {
        bootStepBusy("connecting to wifi");
        if (connectWifi(12000)) {
            char m[64];
            snprintf(m, sizeof m, "wifi %s", WiFi.localIP().toString().c_str());
            bootStep(m);
        } else {
            bootStepFail("wifi - stored credential rejected");
            M5.Display.fillScreen(TFT_BLACK);
            portal_run(300000);
            bootBegin();
        }
    }
    M5.Display.fillScreen(style_background());

    // GNSS first, and at high priority: the FIFO overflows if the drain is
    // starved, and the renderer will happily saturate its core.
    if (!gnss_start(PIN_GNSS_TX, PIN_GNSS_RX, GNSS_BAUD, PIN_PPS, 0, 5))
        Serial.println("gnss task failed to start");

    bootStepBusy("opening map data");
    if (!map_begin(PMT_PATH, Z_CLOSE, 1, 1)) {
        bootStepFail("map data - init failed");
        M5.Display.setTextColor(TFT_RED);
        M5.Display.setTextSize(2);
        M5.Display.drawString("See serial output", 40, M5.Display.height() - 60);
        g_bootActive = false;
        return;
    }
    {
        char m[64];
        snprintf(m, sizeof m, "map ready, %dpx tiles, z%d+", SUBTILE_PX, Z_FLOOR);
        bootStep(m);
    }
    // Kick off the world floor if it has never been stored. It runs in the
    // background and survives being interrupted, so there is no reason to
    // hold up startup for it.
    if (WiFi.status() == WL_CONNECTED && !netsource_world_ready()) {
        bootStep("storing world floor in background");
        map_world_floor_start();
    }

    bootStepBusy("waiting for GPS fix");
    delay(600);
    bootEnd();
    Serial.println("running");
}

void loop() {
    M5.update();

    GnssFix fix;
    gnss_get(&fix);

    handleTouch();
    handlePowerButton();
    flushIfIdle();
    applyTheme(fix);

    map_update(fix);
    pickZoom(fix);

    // Build discovery needs a calendar date. Two independent sources, since
    // each can be unavailable: SNTP needs the network, GNSS needs sky. The
    // first one to arrive wins and the other is skipped.
    static bool dateSet = false;
    if (!dateSet) {
        if (fix.date[0] && fix.status == 'A') {
            netsource_set_date(fix.date);
            dateSet = true;
            Serial.printf("netsource: UTC date %s from GNSS\n", fix.date);
        } else {
            static uint32_t lastTry = 0;
            if (millis() - lastTry > 3000) {
                lastTry = millis();
                if (netsource_set_date_from_clock()) {
                    dateSet = true;
                    // Write the verified time back to the RTC, so the next
                    // cold boot starts from something sane instead of
                    // whatever the chip happened to be holding.
                    time_t now = time(nullptr);
                    struct tm t;
                    gmtime_r(&now, &t);
                    m5::rtc_datetime_t dt;
                    dt.date.year = t.tm_year + 1900;
                    dt.date.month = t.tm_mon + 1;
                    dt.date.date = t.tm_mday;
                    dt.date.weekDay = t.tm_wday;
                    dt.time.hours = t.tm_hour;
                    dt.time.minutes = t.tm_min;
                    dt.time.seconds = t.tm_sec;
                    M5.Rtc.setDateTime(dt);
                    Serial.println("rtc: updated from SNTP");
                }
            }
        }
    }

    // Periodic timing to serial. The status bar carries this too, but the
    // screen is not always the thing being watched - and blit cost against
    // render cost is the number that says whether compositing is stealing
    // time from the renderer.
    {
        static uint32_t lastStats = 0;
        if (millis() - lastStats > 15000) {
            lastStats = millis();
            MapStats st; map_stats(&st);
            NetStats ns; netsource_stats(&ns);
            CacheStats cs; tilecache_stats(&cs);
            Serial.printf("stats: rendered %lu (last %lu ms)  blit last %lu ms "
                          "max %lu avg %lu over %lu  q%lu  cache %lu/%lu net %lu  "
                          "psram %u KB\n",
                          (unsigned long)st.rendered, (unsigned long)st.last_render_ms,
                          (unsigned long)st.last_draw_ms, (unsigned long)st.max_draw_ms,
                          (unsigned long)(st.draws ? st.draw_total_ms / st.draws : 0),
                          (unsigned long)st.draws, (unsigned long)st.queue_depth,
                          (unsigned long)ns.cache_hits, (unsigned long)cs.entries,
                          (unsigned long)ns.net_hits,
                          (unsigned)(ESP.getFreePsram() / 1024));
        }
    }

    // ~15 fps. The map only changes when a tile commits or the marker moves,
    // neither of which happens at frame rate.
    static uint32_t last = 0;
    uint32_t now = millis();
    if (!g_screenOff && now - last >= 66) {
        last = now;
        map_draw(fix);
        drawStatus(fix);
        drawFooter();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}
