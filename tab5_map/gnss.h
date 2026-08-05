// gnss.h - M135 GNSS reader as a FreeRTOS task.
//
// The parsing is taken verbatim from the working tab5_gnss_sensors sketch;
// only the plumbing around it is new. Instead of pumpGnss() being called
// three times per loop() to survive a long draw, the drain lives in its own
// high-priority task and simply preempts whatever else is running. That
// matters more here than it did before: a tile render occupies its core for
// hundreds of milliseconds at a stretch.

#ifndef GNSS_H
#define GNSS_H

#include <stdint.h>

struct Constellation {
    const char *name;
    int visible;
    int bestSnr;
};

struct GnssFix {
    char   status = 'V';               // RMC: 'A' = valid
    int    mode = 1;                   // GSA: 1 none, 2 = 2D, 3 = 3D
    int    sats = 0;
    double lat = 0, lon = 0, altitude = 0, hdop = 99.99;
    double speedKmh = 0, course = 0;
    char   utc[16] = "", date[16] = "";
    uint32_t lastSentence = 0;         // millis() of last parsed sentence
    Constellation cons[4] = {{"GPS",0,0},{"GLO",0,0},{"GAL",0,0},{"BDS",0,0}};
};

// Quality gates for the staged zoom-in.
static inline bool gnss_coarse(const GnssFix &f) { return f.status == 'A'; }
static inline bool gnss_fine(const GnssFix &f) {
    return f.status == 'A' && f.mode == 3 && f.hdop > 0 && f.hdop < 2.5;
}

// Pin names follow the module's own labels, as in the original sketch:
// rx_pin is the pin the module TRANSMITS on (G7 with the DIP in position 1),
// tx_pin is the pin it listens on (G6). They are passed to Serial1.begin in
// that same order.
bool gnss_start(int rx_pin, int tx_pin, uint32_t baud, int pps_pin,
                int core, int priority);

void gnss_get(GnssFix *out);

uint32_t gnss_sentences();
uint32_t gnss_pps_count();
uint32_t gnss_pps_interval();

#endif // GNSS_H
