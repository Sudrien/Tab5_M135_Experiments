// gnss.cpp - NMEA parsing lifted from tab5_gnss_sensors.ino, wrapped in a task.

#include "gnss.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include <stdlib.h>

static SemaphoreHandle_t g_lock = nullptr;
static GnssFix g_pub;                 // published copy, guarded by g_lock
static uint32_t g_sentences = 0;

// PPS is edge-counted in an ISR. Polling misses pulses, because the render
// path can occupy far longer than the pulse width.
static volatile uint32_t g_ppsCount = 0, g_ppsLast = 0, g_ppsInterval = 0;

static void IRAM_ATTR ppsIsr() {
    uint32_t now = millis();
    if (g_ppsLast) g_ppsInterval = now - g_ppsLast;
    g_ppsLast = now;
    g_ppsCount++;
}

// ---- parsing (unchanged from the working sketch) ----------------------------
static int splitFields(char *s, char **f, int maxf) {
    int n = 0; f[n++] = s;
    for (char *p = s; *p && n < maxf; p++) {
        if (*p == ',') { *p = 0; f[n++] = p + 1; }
        else if (*p == '*') { *p = 0; break; }
    }
    return n;
}

// Handles both 2-digit latitude and 3-digit longitude without being told
// which: integer-dividing by 100 strips whatever degree field is present.
static double nmeaCoord(const char *v, const char *h) {
    if (!v || !*v) return 0;
    double raw = atof(v); int deg = (int)(raw / 100);
    double d = deg + (raw - deg * 100) / 60.0;
    if (h && (*h == 'S' || *h == 'W')) d = -d;
    return d;
}

static int conIndex(const char *t) {
    if (!strncmp(t, "GP", 2)) return 0;
    if (!strncmp(t, "GL", 2)) return 1;
    if (!strncmp(t, "GA", 2)) return 2;
    if (!strncmp(t, "GB", 2)) return 3;
    return -1;
}

static void parseSentence(char *s, GnssFix &fix) {
    if (s[0] != '$') return;
    g_sentences++;
    fix.lastSentence = millis();
    char talker[3] = { s[1], s[2], 0 };
    char type[4]   = { s[3], s[4], s[5], 0 };
    char *f[24];
    int n = splitFields(s, f, 24);

    if (!strcmp(type, "RMC") && n > 9) {
        fix.status = f[2][0] ? f[2][0] : 'V';
        strncpy(fix.utc, f[1], sizeof(fix.utc) - 1);
        strncpy(fix.date, f[9], sizeof(fix.date) - 1);
        if (fix.status == 'A') {
            fix.lat = nmeaCoord(f[3], f[4]);
            fix.lon = nmeaCoord(f[5], f[6]);
        }
    } else if (!strcmp(type, "VTG") && n > 7) {
        fix.course   = atof(f[1]);
        fix.speedKmh = atof(f[7]);          // km/h directly
    } else if (!strcmp(type, "GGA") && n > 9) {
        fix.sats = atoi(f[7]);
        fix.hdop = f[8][0] ? atof(f[8]) : 99.99;
        fix.altitude = atof(f[9]);
    } else if (!strcmp(type, "GSA") && n > 17) {
        int m = atoi(f[2]); if (m > fix.mode || m == 1) fix.mode = m;
    } else if (!strcmp(type, "GSV") && n >= 4) {
        int ci = conIndex(talker);
        if (ci >= 0) {
            if (atoi(f[2]) == 1) fix.cons[ci].bestSnr = 0;
            fix.cons[ci].visible = atoi(f[3]);
            for (int i = 4; i + 3 < n; i += 4) {
                int snr = atoi(f[i + 3]);
                if (snr > fix.cons[ci].bestSnr) fix.cons[ci].bestSnr = snr;
            }
        }
    }
}

// ---- task ------------------------------------------------------------------
static void gnss_task(void *arg) {
    (void)arg;
    char line[128];
    int  pos = 0;
    GnssFix local;                    // parsed into privately, published whole

    for (;;) {
        if (!Serial1.available()) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        while (Serial1.available()) {
            char c = Serial1.read();
            if (c == '\n') {
                line[pos] = 0;
                parseSentence(line, local);
                pos = 0;
                // Publish the whole struct at once so a reader never sees a
                // fix half-updated across sentences.
                if (xSemaphoreTake(g_lock, portMAX_DELAY) == pdTRUE) {
                    g_pub = local;
                    xSemaphoreGive(g_lock);
                }
            } else if (c != '\r' && pos < (int)sizeof(line) - 1) {
                line[pos++] = c;
            }
        }
    }
}

bool gnss_start(int rx_pin, int tx_pin, uint32_t baud, int pps_pin,
                int core, int priority)
{
    g_lock = xSemaphoreCreateMutex();
    if (!g_lock) return false;

    // Must precede begin() - ignored once the port is open. The default 256 B
    // FIFO overflows during a long draw at 38400 baud.
    Serial1.setRxBufferSize(2048);
    Serial1.begin(baud, SERIAL_8N1, rx_pin, tx_pin);

    if (pps_pin >= 0) {
        pinMode(pps_pin, INPUT_PULLDOWN);
        attachInterrupt(pps_pin, ppsIsr, RISING);
    }

    g_pub.lastSentence = millis();

    return xTaskCreatePinnedToCore(gnss_task, "gnss", 4096, nullptr,
                                   priority, nullptr, core) == pdPASS;
}

void gnss_get(GnssFix *out) {
    if (xSemaphoreTake(g_lock, portMAX_DELAY) == pdTRUE) {
        *out = g_pub;
        xSemaphoreGive(g_lock);
    }
}

uint32_t gnss_sentences()    { return g_sentences; }
uint32_t gnss_pps_count()    { return g_ppsCount; }
uint32_t gnss_pps_interval() { return g_ppsInterval; }
