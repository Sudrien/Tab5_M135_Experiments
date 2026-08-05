// mapengine.h - tile grid, render worker, and screen composition.
//
// Threading model
// ---------------
//   render worker (core 1, low priority)  owns the PMTiles handle and the SD
//       file. It is the only task that touches either, which makes FatFs
//       thread-safety a non-issue. It writes pixels only into slots marked
//       PENDING.
//
//   UI task (core 0, loop())  reads the GNSS fix, decides when the grid must
//       shift, enqueues jobs, and composites READY slots to the panel.
//
// The one shared structure is the grid's metadata, guarded by a mutex held
// only for microseconds. Pixel buffers need no lock because of a single
// invariant: a PENDING slot is never blitted, and only PENDING slots are
// written. A shift that recycles a buffer mid-render is harmless - the
// worker's generation check fails, the result is dropped, and the slot stays
// PENDING until the replacement job lands.

#ifndef MAPENGINE_H
#define MAPENGINE_H

#include <stdint.h>
#include "gnss.h"

extern "C" {
  #include "tile_grid.h"
}

struct MapStats {
    uint32_t rendered = 0, dropped = 0, notfound = 0, failed = 0;
    uint32_t shifts = 0, queued = 0;
    uint32_t last_render_ms = 0;
    uint32_t queue_depth = 0;
};

// Bring up buffers, open the archive, and start the render worker.
// `path` is the .pmtiles file on the mounted card.
bool map_begin(const char *path, uint8_t zoom, int worker_core, int worker_prio);

// Feed a fix. Recentres the grid and enqueues work when the marker leaves the
// centre tile. Safe to call every loop iteration; cheap when nothing changed.
void map_update(const GnssFix &fix);

// Composite READY slots and the position marker onto the panel.
void map_draw(const GnssFix &fix);

// Force a full re-render, e.g. after a zoom change.
void map_set_zoom(uint8_t zoom, const GnssFix &fix);

uint8_t  map_zoom();
void     map_stats(MapStats *out);
bool     map_has_fix_position();   // true once the grid has been centred

#endif // MAPENGINE_H
