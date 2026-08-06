// mapengine.cpp

#include "mapengine.h"
#include "style.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>
#include <SD_MMC.h>
#include <WiFi.h>          // world_task checks the link before each fetch
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <math.h>

#include "netsource.h"

extern "C" {
  #include "pmtiles.h"
  #include "mvt.h"
  #include "inflate.h"
  #include "raster.h"
}
#include "mercator.h"
#include "mapconfig.h"

// ---- configuration ---------------------------------------------------------
// Scratch for one tile, compressed and inflated.
//
// These were sized against an 84 KB z14 tile. A z13 tile covers four times
// the ground and can be several times larger, so both grow on demand: gzip
// records its uncompressed size in the trailer, which makes the requirement
// exact rather than a guess.
static uint32_t TILE_CAP = 192 * 1024;
static uint32_t MVT_CAP  = 384 * 1024;
static const uint32_t MVT_CAP_MAX = 3 * 1024 * 1024;
static const uint32_t PT_CAP    =  2048;
static const uint32_t VAL_CAP   =  1024;
static const uint32_t EDGE_CAP  = 16384;
static const uint32_t XS_CAP    =  4096;

// Screen geometry. The status bar owns the top strip and the buttons the
// bottom one; the map is clipped out of both so they are not fighting over
// the same pixels.
static const int STATUS_H = 52;
static const int FOOTER_H = 82;

// pushImage handled pixel format for us; writePixels is lower level and takes
// byte order explicitly. If red and blue come out exchanged, flip this.
#ifndef BLIT_SWAP_BYTES
#define BLIT_SWAP_BYTES false
#endif

// ---- state -----------------------------------------------------------------
static tile_grid_t      g_grid;
static SemaphoreHandle_t g_glock = nullptr;
static QueueHandle_t     g_jobs  = nullptr;

static uint8_t           g_zoom = 14;
static bool              g_centred = false;
static MapStats          g_stats;

// Archive access lives entirely in netsource now: it owns the local floor,
// the remote build, and the SD cache between them. Nothing here touches
// pmtiles directly.

// Worker-owned scratch. Only the render task reads or writes any of this,
// which is why none of it needs a lock.
static uint8_t   *w_tile = nullptr;    // compressed tile, straight from the source
static uint8_t   *w_mvt  = nullptr;    // inflated MVT
static int32_t   *w_pts  = nullptr;    // decoded points for one geometry part
static uint8_t   *w_val  = nullptr;    // one style byte per value-table entry
static rs_edge_t *w_edges = nullptr;
static uint16_t  *w_active = nullptr;
static int32_t   *w_xs = nullptr;
static int8_t    *w_dirs = nullptr;
static uint16_t  *w_cov = nullptr;

// ---- coarse overview -------------------------------------------------------
// One tile several zoom levels below the working grid, covering far more
// ground than the screen. New slots are filled by upscaling from it, so the
// map is never blank - only soft until the real tile lands.
//
// COARSE_STEP 3 means one coarse tile spans 8x8 working tiles, which the 3x3
// grid stays inside for a long time before it needs re-rendering. Rendering
// it at 1024 rather than 512 halves the upscale factor, and coarse zooms
// carry far less geometry so it costs less than a working tile despite the
// larger surface.
// COARSE_STEP 3 spans 8x8 working tiles. With SUBTILE_PX at 1024 that means
// upscaling 128 source pixels to 1024, which is very soft - acceptable for a
// placeholder that lives under a second, and the alternative is re-rendering
// the overview far more often.
static const int COARSE_STEP = 3;
static const uint8_t COARSE_SLOT = 0xFE;   // sentinel job slot

static uint16_t *g_coarse_px = nullptr;
static tile_id_t g_coarse_id = { 0, 0, 0 };
static bool      g_coarse_ok = false;
static tile_id_t g_coarse_want = { 0, 0, 0 };
static int64_t   g_coarse_retry_at = 0;

// Spare buffer the worker renders into before swapping it into a slot.
static uint16_t *g_spare = nullptr;

// Marker position in world coordinates (fractional tiles at the working
// zoom), and the view's top-left corner in the same units.
//
// The view is held in world coordinates rather than canvas pixels so that a
// grid shift - which moves the canvas origin by a whole tile - does not move
// the picture. Converting to canvas pixels happens at draw time against the
// current origin.
static double g_marker_wx = 0, g_marker_wy = 0;
static double g_view_wx = 0, g_view_wy = 0;
static bool   g_view_set = false;

// ---- rendering one tile ----------------------------------------------------
static const char *g_want_layer = nullptr;
static int rl_layer(void *ctx, const mvt_layer_t *l) {
    (void)ctx;
    return g_want_layer && l->name_len == strlen(g_want_layer) &&
           memcmp(l->name, g_want_layer, l->name_len) == 0;
}

// `split` is how many levels below id.z the data is taken from. The tile id
// stays in display-zoom space; only the fetch and the source rectangle move.
static tile_state_t render_tile(tile_id_t id, uint16_t *px, int size, int split) {
    // One data tile covers 2^split subtiles per axis, so the id's low bits
    // select which quadrant of it this subtile shows.
    uint8_t  dz = (uint8_t)(id.z - split);
    uint32_t dx = (uint32_t)(id.x >> split);
    uint32_t dy = (uint32_t)(id.y >> split);
    uint32_t qx = (uint32_t)(id.x & ((1 << split) - 1));
    uint32_t qy = (uint32_t)(id.y & ((1 << split) - 1));

    // Cache, then network, then the offline floor - netsource decides.
    uint32_t got = TILE_CAP;
    bool from_net = false;
    if (!netsource_get(dz, dx, dy, w_tile, &got, &from_net))
        return TILE_NODATA;
    if (got == 0) return TILE_NODATA;

    // Fingerprint the payload as soon as it lands, and again just before it
    // is used. A mismatch means something wrote into this buffer in between,
    // which narrows the search from "somewhere in the fetch path" to
    // "something else holds this pointer".
    uint32_t sum_after_fetch = 0;
    for (uint32_t i = 0; i < got; i++) sum_after_fetch = sum_after_fetch * 31 + w_tile[i];

    // Sanity-check the payload before trusting it. A tile that is not gzip at
    // all means the fetch returned something other than this tile - a
    // different range, or another response's body - which is a transport
    // problem wearing an inflate problem's clothes.
    if (got < 18 || w_tile[0] != 0x1F || w_tile[1] != 0x8B) {
        Serial.printf("map: %u/%d/%d payload is not gzip "
                      "(%lu bytes, starts %02X %02X)\n",
                      dz, dx, dy, (unsigned long)got,
                      got > 0 ? w_tile[0] : 0, got > 1 ? w_tile[1] : 0);
        return TILE_ERROR;
    }

    // Grow the inflate buffer to whatever this tile actually needs, before
    // trying. The gzip trailer states it exactly, so there is no reason to
    // fail first and find out afterwards.
    uint32_t need = gzip_isize(w_tile, got);
    if (need > MVT_CAP) {
        if (need > MVT_CAP_MAX) {
            Serial.printf("map: tile inflates to %lu KB, past the %lu KB limit\n",
                          (unsigned long)(need / 1024),
                          (unsigned long)(MVT_CAP_MAX / 1024));
            return TILE_ERROR;
        }
        uint32_t want = need + need / 4;          // headroom for the next one
        uint8_t *bigger = (uint8_t *)ps_malloc(want);
        if (!bigger) {
            Serial.printf("map: cannot grow inflate buffer to %lu KB "
                          "(%u KB PSRAM free) - lower SUBTILE_PX\n",
                          (unsigned long)(want / 1024),
                          (unsigned)(ESP.getFreePsram() / 1024));
            return TILE_ERROR;
        }
        free(w_mvt);
        w_mvt = bigger;
        MVT_CAP = want;
        Serial.printf("map: inflate buffer -> %lu KB (%u KB PSRAM left)\n",
                      (unsigned long)(want / 1024),
                      (unsigned)(ESP.getFreePsram() / 1024));
    }

    uint32_t sum_before_inflate = 0;
    for (uint32_t i = 0; i < got; i++) sum_before_inflate = sum_before_inflate * 31 + w_tile[i];
    if (sum_before_inflate != sum_after_fetch) {
        Serial.printf("map: BUFFER CLOBBERED between fetch and inflate "
                      "(%08lx -> %08lx) for %u/%d/%d\n",
                      (unsigned long)sum_after_fetch,
                      (unsigned long)sum_before_inflate, dz, dx, dy);
    }

    uint32_t mlen = MVT_CAP;
    // Tile payloads skip the CRC: corruption here shows up as visibly wrong
    // geometry rather than a wrong offset, and the check costs ~3.5x.
    inf_err_t ie = inflate_auto_fast(w_tile, got, w_mvt, &mlen);
    if (ie != INF_OK) {
        Serial.printf("map: inflate %u/%d/%d failed: %s (%lu compressed, "
                      "%lu expected)\n", dz, dx, dy, inflate_strerror(ie),
                      (unsigned long)got, (unsigned long)need);

        // Write the payload out so it can be examined off-device. A stream
        // that fails identically every time is a decoder bug, and the only
        // way to fix one is against the bytes that trigger it - the fuzzer
        // that cleared this decoder only ever fed it zlib's own output.
        // One file per tile id, and only if not already present.
        char p[64];
        snprintf(p, sizeof p, "/fail_%u_%lu_%lu.gz", dz,
                 (unsigned long)dx, (unsigned long)dy);
        fs::FS *fsp = (SD_MMC.cardType() != CARD_NONE) ? (fs::FS *)&SD_MMC
                                                       : (fs::FS *)&SD;
        File chk = fsp->open(p, FILE_READ);
        bool exists = (bool)chk;
        if (chk) chk.close();
        if (!exists) {
            File f = fsp->open(p, FILE_WRITE);
            if (f) {
                f.write(w_tile, got);
                f.close();
                Serial.printf("map: wrote %s (%lu bytes) for analysis\n",
                              p, (unsigned long)got);
            }
        }
        return TILE_ERROR;
    }

    rs_t r;
    memset(&r, 0, sizeof r);
    r.px = px; r.w = r.h = size; r.extent = 4096;
    r.src_span = 4096 >> split;
    r.src_x0 = (int32_t)(qx * r.src_span);
    r.src_y0 = (int32_t)(qy * r.src_span);
    r.edges = w_edges;  r.edge_cap = EDGE_CAP;
    r.active = w_active; r.active_cap = XS_CAP;
    r.xs = w_xs; r.dirs = w_dirs; r.xs_cap = XS_CAP;
    r.cov = w_cov;
    r.styles = STYLES; r.n_styles = S_COUNT;
    r.cur_feature = -1;

    mvt_decoder_t d;
    memset(&d, 0, sizeof d);
    d.layer_cb = rl_layer;
    d.style_cb = style_lookup;
    d.part_cb  = rs_part;
    d.ctx      = &r;
    d.pt_buf   = w_pts;  d.pt_cap  = PT_CAP;
    d.val_style = w_val; d.val_cap = VAL_CAP;

    rs_clear(&r, style_background());
    // Layers are drawn bottom-up by decoding once per layer with the filter
    // set. Skipped layers cost only a length-walk, so total decode work stays
    // close to a single full pass.
    for (int i = 0; i < N_DRAW_ORDER; i++) {
        g_want_layer = DRAW_ORDER[i];
        mvt_decode(&d, w_mvt, mlen);
        rs_flush(&r);
    }
    return TILE_READY;
}

// ---- render worker ---------------------------------------------------------
static void worker_task(void *arg) {
    (void)arg;
    render_job_t job;
    for (;;) {
        if (xQueueReceive(g_jobs, &job, portMAX_DELAY) != pdTRUE) continue;

        // The overview tile is rendered by the same worker, into its own
        // buffer. Nothing reads it while it is being drawn except the coarse
        // fill, which tolerates a half-updated source - it is an
        // approximation either way.
        if (job.slot == COARSE_SLOT) {
            uint64_t t0 = esp_timer_get_time();
            g_coarse_ok = false;
            tile_state_t res = render_tile(job.id, g_coarse_px, COARSE_PX, 0);
            if (res == TILE_READY) { g_coarse_id = job.id; g_coarse_ok = true; }
            Serial.printf("map: overview z%u %s in %lu ms\n", job.id.z,
                          res == TILE_READY ? "rendered" : "FAILED",
                          (unsigned long)((esp_timer_get_time() - t0) / 1000));
            // Everything on screen depends on this one tile when the working
            // level is missing, so a transient failure is worth retrying
            // rather than leaving the map with no fallback until the grid
            // happens to move far enough to request a different overview.
            if (res != TILE_READY) {
                g_coarse_retry_at = esp_timer_get_time() + 10000000;  // 10 s
                g_coarse_want = job.id;
            }
            continue;
        }

        // Cheap early-out: if the grid has already moved on, skip the work
        // entirely rather than rendering something that will be discarded.
        xSemaphoreTake(g_glock, portMAX_DELAY);
        uint32_t gen_now = g_grid.generation;
        xSemaphoreGive(g_glock);
        if (job.generation != gen_now) { g_stats.dropped++; continue; }

        // Render into the spare buffer, never into the slot: the slot may be
        // showing a coarse placeholder that the UI task is blitting right now.
        uint64_t t0 = esp_timer_get_time();
        tile_state_t res = render_tile(job.id, g_spare, SUBTILE_PX, SUBTILE_SPLIT);
        uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

        int committed;
        xSemaphoreTake(g_glock, portMAX_DELAY);
        if (res == TILE_READY) {
            uint16_t *recycled = nullptr;
            committed = grid_commit_swap(&g_grid, &job, res, g_spare, &recycled);
            g_spare = recycled;              // O(1) handover, no 512 KB copy
        } else {
            committed = grid_commit(&g_grid, &job, res);
        }
        xSemaphoreGive(g_glock);

        if (!committed)                  g_stats.dropped++;
        else if (res == TILE_READY)    { g_stats.rendered++; g_stats.last_render_ms = ms; }
        else if (res == TILE_NODATA)     g_stats.notfound++;
        else                             g_stats.failed++;

        // A tile that does not render is the interesting case, and the status
        // line has no room for the reason. Log the first few of each kind.
        if (res != TILE_READY || !committed) {
            static uint32_t logged = 0;
            if (logged < 12) {
                logged++;
                Serial.printf("map: tile %u/%d/%d (data %u/%d/%d) -> %s%s (%lu ms)\n",
                    job.id.z, job.id.x, job.id.y,
                    job.id.z - SUBTILE_SPLIT,
                    job.id.x >> SUBTILE_SPLIT, job.id.y >> SUBTILE_SPLIT,
                    res == TILE_NODATA ? "NODATA" :
                    res == TILE_ERROR  ? "ERROR"  : "READY",
                    committed ? "" : " [stale, dropped]",
                    (unsigned long)ms);
            }
        }
    }
}

// ---- coarse fill -----------------------------------------------------------
// Paint a slot from the overview tile so it has something to show while the
// real render is queued. Nearest-neighbour is deliberate: the result is a
// placeholder that will be replaced within a second, and a bilinear pass over
// 262k pixels would cost more than it buys.
//
// Runs on the UI task. It writes into a slot the worker is not touching -
// the worker renders into the spare buffer and only swaps on commit - so no
// lock is needed beyond the one already held for slot metadata.
static bool coarse_fill(subtile_t *s) {
    if (!g_coarse_ok || !g_coarse_px) return false;
    if (s->id.z <= g_coarse_id.z) return false;

    int step = s->id.z - g_coarse_id.z;
    int span = 1 << step;                    // working tiles per coarse tile

    // Where this tile sits inside the coarse tile, in coarse-tile pixels.
    int32_t base_x = g_coarse_id.x * span;
    int32_t base_y = g_coarse_id.y * span;
    int32_t ox = s->id.x - base_x;
    int32_t oy = s->id.y - base_y;
    if (ox < 0 || oy < 0 || ox >= span || oy >= span) return false;  // outside

    int sub = COARSE_PX / span;              // source pixels per working tile
    if (sub <= 0) return false;
    int32_t sx0 = ox * sub, sy0 = oy * sub;

    // Integer step through the source, 16.16 fixed point.
    uint32_t inc = ((uint32_t)sub << 16) / SUBTILE_PX;
    uint16_t *dst = s->pixels;
    uint32_t sy = 0;
    for (int y = 0; y < SUBTILE_PX; y++, sy += inc) {
        const uint16_t *row = g_coarse_px + (size_t)(sy0 + (sy >> 16)) * COARSE_PX + sx0;
        uint32_t sx = 0;
        for (int x = 0; x < SUBTILE_PX; x++, sx += inc)
            *dst++ = row[sx >> 16];
    }
    return true;
}

// Fill slots that have nothing to show yet.
//
// Two constraints shape this. The scaling loop writes 512 KB per slot, so
// doing all nine on one pass would stall the UI for a noticeable fraction of
// a second - hence the per-pass budget. And the loop must not run under the
// grid lock, or it would block the worker's commit for just as long.
//
// Dropping the lock during the fill is safe because of who touches what: the
// worker only ever writes to its spare buffer, and map_update and map_draw
// both run on the UI task, so nothing can reassign a slot's pixel pointer
// while this is using it. The lock is taken only around the state fields.
static const int COARSE_FILLS_PER_PASS = 2;

static void coarse_fill_pending() {
    // Retry a failed overview on a timer.
    if (!g_coarse_ok && g_coarse_retry_at &&
        esp_timer_get_time() > g_coarse_retry_at) {
        g_coarse_retry_at = 0;
        render_job_t j;
        j.id = g_coarse_want;
        j.slot = COARSE_SLOT;
        j.generation = 0;
        xQueueSendToFront(g_jobs, &j, 0);
        Serial.println("map: retrying overview");
    }
    if (!g_coarse_ok) return;

    for (int done = 0; done < COARSE_FILLS_PER_PASS; done++) {
        subtile_t *target = nullptr;
        subtile_t snapshot;
        tile_state_t was = TILE_EMPTY;

        xSemaphoreTake(g_glock, portMAX_DELAY);
        for (int i = 0; i < GRID_COUNT; i++) {
            tile_state_t st = g_grid.slots[i].state;
            // NODATA and ERROR need this as much as PENDING does - arguably
            // more. A pending tile is about to be replaced anyway; a tile the
            // archive could not supply stays blank forever otherwise, which
            // is what turns driving out of cached territory into a white
            // screen rather than a coarse one.
            if (st == TILE_PENDING || st == TILE_NODATA || st == TILE_ERROR) {
                target = &g_grid.slots[i];
                snapshot = *target;
                was = st;
                break;
            }
        }
        xSemaphoreGive(g_glock);
        if (!target) return;

        bool ok = coarse_fill(&snapshot);

        xSemaphoreTake(g_glock, portMAX_DELAY);
        // Re-check: the worker may have committed a real render in the gap,
        // in which case the placeholder is already obsolete and must not
        // overwrite the READY state.
        if (target->state == was && target->pixels == snapshot.pixels) {
            if (ok) target->state = TILE_COARSE;
            else if (was != TILE_PENDING) {
                // No overview to draw from and no tile of its own. Leave it
                // marked so the search does not spin on it every pass.
                target->state = TILE_EMPTY;
            }
        }
        xSemaphoreGive(g_glock);

        if (!ok) return;    // outside coverage; the rest will be too
    }
}

// Queue an overview render if the grid has moved outside what the current
// overview covers. One coarse tile spans 8x8 working tiles, so this fires
// rarely - roughly once per eight tile crossings.
static void ensure_coarse(tile_id_t origin) {
    int cz = (int)origin.z - COARSE_STEP;
    if (cz < 0) cz = 0;
    int step = origin.z - cz;
    // Centre of the grid, not its corner, so the overview stays useful for
    // the whole canvas rather than drifting off one edge of it.
    int32_t cx = origin.x + GRID_N / 2, cy = origin.y + GRID_N / 2;
    tile_id_t want = { (uint8_t)cz, cx >> step, cy >> step };

    if (g_coarse_ok && want.z == g_coarse_id.z &&
        want.x == g_coarse_id.x && want.y == g_coarse_id.y) return;

    render_job_t j;
    j.id = want;
    j.slot = COARSE_SLOT;
    j.generation = 0;                        // never stale; not a grid slot
    g_coarse_want = want;
    g_coarse_retry_at = 0;
    // Front of the queue: with the working level missing, this tile is the
    // only thing standing between the user and a blank screen.
    xQueueSendToFront(g_jobs, &j, 0);
}

// ---- setup -----------------------------------------------------------------
static bool alloc_all() {
    w_tile   = (uint8_t *)ps_malloc(TILE_CAP);
    w_mvt    = (uint8_t *)ps_malloc(MVT_CAP);
    w_pts    = (int32_t *)ps_malloc(PT_CAP * 2 * sizeof(int32_t));
    w_val    = (uint8_t *)ps_malloc(VAL_CAP);
    w_edges  = (rs_edge_t *)ps_malloc(EDGE_CAP * sizeof(rs_edge_t));
    w_active = (uint16_t *)ps_malloc(XS_CAP * sizeof(uint16_t));
    w_xs     = (int32_t *)ps_malloc(XS_CAP * sizeof(int32_t));
    w_dirs   = (int8_t *)ps_malloc(XS_CAP);
    w_cov    = (uint16_t *)ps_malloc(SUBTILE_PX * sizeof(uint16_t));
    return w_tile && w_mvt && w_pts && w_val &&
           w_edges && w_active && w_xs && w_dirs && w_cov;
}

bool map_begin(const char *path, uint8_t zoom, int worker_core, int worker_prio) {
    if (!alloc_all()) { Serial.println("map: PSRAM alloc failed"); return false; }

    // 9 tile buffers, allocated once and never freed. Ownership rotates
    // between slots on every shift; the count stays constant for the life of
    // the program.
    // One extra buffer beyond the nine: the worker renders into it and swaps
    // it into a slot on commit, taking the slot's old buffer as the next
    // spare. Allocation count stays fixed at ten for the life of the program.
    g_spare = (uint16_t *)ps_malloc((size_t)SUBTILE_PX * SUBTILE_PX * 2);
    g_coarse_px = (uint16_t *)ps_malloc((size_t)COARSE_PX * COARSE_PX * 2);
    if (!g_spare || !g_coarse_px) {
        Serial.printf("map: spare/overview alloc failed at SUBTILE_PX=%d\n",
                      SUBTILE_PX);
        Serial.println("     lower SUBTILE_PX in mapconfig.h (768, or 512)");
        return false;
    }

    static uint16_t *bufs[GRID_COUNT];
    for (int i = 0; i < GRID_COUNT; i++) {
        bufs[i] = (uint16_t *)ps_malloc((size_t)SUBTILE_PX * SUBTILE_PX * 2);
        if (!bufs[i]) {
            Serial.printf("map: tile buffer %d/%d failed at SUBTILE_PX=%d "
                          "(%u KB free)\n", i, GRID_COUNT, SUBTILE_PX,
                          (unsigned)(ESP.getFreePsram() / 1024));
            Serial.println("     lower SUBTILE_PX in mapconfig.h (768, or 512)");
            return false;
        }
        for (int p = 0; p < SUBTILE_PX * SUBTILE_PX; p++)
            bufs[i][p] = rs_rgb(228, 226, 220);
    }

    if (!netsource_begin(path)) { Serial.println("map: netsource init failed"); return false; }

    g_zoom = zoom;

    style_init(SUBTILE_PX, 0);

    g_glock = xSemaphoreCreateMutex();
    g_jobs  = xQueueCreate(32, sizeof(render_job_t));
    if (!g_glock || !g_jobs) return false;

    tile_id_t c = { g_zoom, 0, 0 };
    grid_init(&g_grid, bufs, c);
    g_centred = false;

    BaseType_t ok = xTaskCreatePinnedToCore(
        worker_task, "tilerender", 12288, nullptr, worker_prio, nullptr, worker_core);
    if (ok != pdPASS) { Serial.println("map: worker task failed"); return false; }

    Serial.printf("map: display z%u from z%u data, %dpx subtiles "
                  "(%dx%d per data tile), %u KB PSRAM left\n",
                  g_zoom, g_zoom - SUBTILE_SPLIT, SUBTILE_PX,
                  SPLIT_N, SPLIT_N, (unsigned)(ESP.getFreePsram() / 1024));
    return true;
}

// Hold the marker inside a band in the middle of the visible area.
//
// The view only moves when the marker would leave the band, and then only far
// enough to keep it on the boundary. Inside the band nothing moves at all,
// which is what stops fix noise from shuffling the map.
//
// Everything is in world units (fractional tiles); the band is converted from
// pixels once, here.
static void view_follow() {
    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;
    const double visH = visBot - visTop;

    // Band edges, as offsets from the top-left of the visible area.
    double bx = SW * (1.0 - MARKER_BAND) / 2.0;
    double by = visH * (1.0 - MARKER_BAND) / 2.0;
    double bxLo = bx, bxHi = SW - bx;
    double byLo = by, byHi = visH - by;

    if (!g_view_set) {
        g_view_wx = g_marker_wx - (SW / 2.0) / SUBTILE_PX;
        g_view_wy = g_marker_wy - (visH / 2.0) / SUBTILE_PX;
        g_view_set = true;
        return;
    }

    double loX = g_marker_wx - bxHi / SUBTILE_PX;
    double hiX = g_marker_wx - bxLo / SUBTILE_PX;
    if (g_view_wx < loX) g_view_wx = loX;
    if (g_view_wx > hiX) g_view_wx = hiX;

    double loY = g_marker_wy - byHi / SUBTILE_PX;
    double hiY = g_marker_wy - byLo / SUBTILE_PX;
    if (g_view_wy < loY) g_view_wy = loY;
    if (g_view_wy > hiY) g_view_wy = hiY;

    // The canvas is finite, so the view cannot wander past its edges however
    // the band would like it to. This takes priority - showing background is
    // worse than the marker briefly leaving the band, and the grid shift that
    // follows will restore it.
    xSemaphoreTake(g_glock, portMAX_DELAY);
    double ox = g_grid.origin.x, oy = g_grid.origin.y;
    xSemaphoreGive(g_glock);

    double maxX = ox + GRID_N - (double)SW / SUBTILE_PX;
    double maxY = oy + GRID_N - visH / SUBTILE_PX;
    if (g_view_wx < ox) g_view_wx = ox;
    if (g_view_wx > maxX) g_view_wx = maxX;
    if (g_view_wy < oy) g_view_wy = oy;
    if (g_view_wy > maxY) g_view_wy = maxY;
}

// ---- job dispatch ----------------------------------------------------------
static void enqueue(render_job_t *jobs, int n) {
    for (int i = 0; i < n; i++) {
        if (xQueueSend(g_jobs, &jobs[i], 0) == pdTRUE) g_stats.queued++;
        // A full queue means the worker is far behind; the tiles it is about
        // to finish are stale anyway, so dropping the newest is wrong.
        // Instead drain one stale entry and retry once.
        else {
            render_job_t junk;
            if (xQueueReceive(g_jobs, &junk, 0) == pdTRUE) {
                g_stats.dropped++;
                if (xQueueSend(g_jobs, &jobs[i], 0) == pdTRUE) g_stats.queued++;
            }
        }
    }
}

static void recentre(const GnssFix &fix) {
    merc_pt_t p = merc_from_ll(fix.lat, fix.lon, g_zoom);
    // Anchor by origin, which centres the canvas on the marker for odd and
    // even grids alike - an even grid has no middle tile to sit inside.
    tile_id_t c = { g_zoom, grid_origin_for(p.x), grid_origin_for(p.y) };

    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    int n = grid_set_zoom(&g_grid, c, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    ensure_coarse(c);
    enqueue(jobs, n);
    g_centred = true;
    // A fresh grid means the previous view may sit outside it entirely.
    g_view_set = false;
    g_stats.shifts++;
}

void map_update(const GnssFix &fix) {
    if (!gnss_coarse(fix)) return;

    merc_pt_t p = merc_from_ll(fix.lat, fix.lon, g_zoom);

    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t centre = g_grid.origin;
    xSemaphoreGive(g_glock);

    if (!g_centred) { recentre(fix); return; }

    g_marker_wx = p.x;
    g_marker_wy = p.y;
    view_follow();

    int dx, dy;
    xSemaphoreTake(g_glock, portMAX_DELAY);
    grid_drift(&g_grid, p.x, p.y, &dx, &dy);
    xSemaphoreGive(g_glock);
    if (!dx && !dy) return;

    // A jump of more than one tile - tunnel exit, cold relocate - is cheaper
    // to handle as a full recentre than as repeated single-step shifts.
    double mid = (double)GRID_N / 2.0;
    double rx = p.x - ((double)centre.x + mid), ry = p.y - ((double)centre.y + mid);
    if (rx < -1.5 || rx > 1.5 || ry < -1.5 || ry > 1.5) { recentre(fix); return; }

    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    int n = grid_shift(&g_grid, dx, dy, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t nc = g_grid.origin;
    xSemaphoreGive(g_glock);
    ensure_coarse(nc);

    enqueue(jobs, n);
    g_stats.shifts++;
}

void map_set_zoom(uint8_t zoom, const GnssFix &fix) {
    if (zoom == g_zoom) return;
    g_zoom = zoom;
    if (gnss_coarse(fix)) recentre(fix);
}

// ---- compositing -----------------------------------------------------------
static bool g_visible = true;
static bool g_force_redraw = true;

// Anything that changes the image without changing the grid or the marker -
// a palette switch, a wake from screen-off, the status bar being cleared -
// has to say so, or the signature check will conclude nothing happened.
void map_invalidate() { g_force_redraw = true; }

void map_set_visible(bool visible) {
    g_visible = visible;
    if (visible) g_force_redraw = true;
}

void map_set_dark(bool dark) {
    if ((bool)style_is_dark() == dark) return;
    style_init(SUBTILE_PX, dark ? 1 : 0);

    // Colours are chosen during rasterisation, so nothing already drawn can
    // be reused. Invalidate the whole grid and the overview; the coarse
    // fallback covers the gap, though it is itself stale until it re-renders.
    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t o = g_grid.origin;
    int n = grid_set_zoom(&g_grid, o, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    g_coarse_ok = false;
    ensure_coarse(o);
    enqueue(jobs, n);
    g_force_redraw = true;
    Serial.printf("map: switched to %s palette, re-rendering %d tiles\n",
                  dark ? "night" : "day", n);
}

bool map_is_dark() { return style_is_dark(); }

// Push one screen rectangle from whichever tiles cover it.
//
// Everything drawn from the tile buffers goes through here: a full repaint is
// just the whole visible area, and erasing the marker is the two small rects
// it moved between. Sending only the intersection matters - a 1280px tile
// pushed whole is 3.3 MB, and doing that for a 70-pixel dot would cost more
// than the entire visible area.
static void blit_region(int32_t rx, int32_t ry, int32_t rw, int32_t rh,
                        int32_t canvas_x, int32_t canvas_y)
{
    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;

    if (rx < 0) { rw += rx; rx = 0; }
    if (ry < visTop) { rh -= visTop - ry; ry = visTop; }
    if (rx + rw > SW) rw = SW - rx;
    if (ry + rh > visBot) rh = visBot - ry;
    if (rw <= 0 || rh <= 0) return;

    // Paint background first when any tile is absent, so the gaps behind
    // them do not keep whatever was there before - which is how the marker
    // used to smear across the screen.
    bool any_missing = false;
    xSemaphoreTake(g_glock, portMAX_DELAY);
    for (int i = 0; i < GRID_COUNT; i++)
        if (!tile_drawable(g_grid.slots[i].state)) { any_missing = true; break; }
    xSemaphoreGive(g_glock);
    if (any_missing) M5.Display.fillRect(rx, ry, rw, rh, style_background());

    xSemaphoreTake(g_glock, portMAX_DELAY);
    M5.Display.startWrite();
    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {
            int i = r * GRID_N + c;
            if (!tile_drawable(g_grid.slots[i].state)) continue;

            // Tile's top-left in screen coordinates.
            int32_t sx = c * SUBTILE_PX - canvas_x;
            int32_t sy = visTop + r * SUBTILE_PX - canvas_y;

            int32_t dx0 = sx > rx ? sx : rx;
            int32_t dy0 = sy > ry ? sy : ry;
            int32_t dx1 = sx + SUBTILE_PX < rx + rw ? sx + SUBTILE_PX : rx + rw;
            int32_t dy1 = sy + SUBTILE_PX < ry + rh ? sy + SUBTILE_PX : ry + rh;
            if (dx0 >= dx1 || dy0 >= dy1) continue;

            int32_t w = dx1 - dx0, h = dy1 - dy0;
            int32_t srcx = dx0 - sx, srcy = dy0 - sy;
            const uint16_t *px = g_grid.slots[i].pixels;

            M5.Display.setAddrWindow(dx0, dy0, w, h);
            for (int32_t y = 0; y < h; y++)
                M5.Display.writePixels(px + (size_t)(srcy + y) * SUBTILE_PX + srcx,
                                       w, BLIT_SWAP_BYTES);
        }
    }
    M5.Display.endWrite();
    xSemaphoreGive(g_glock);
}

static void draw_marker(const GnssFix &fix, int mx, int my) {
    if (!gnss_coarse(fix)) return;
    uint16_t col = gnss_fine(fix) ? M5.Display.color565(30, 90, 220)
                                  : M5.Display.color565(150, 150, 160);
    if (fix.speedKmh > 3.0) {
        float a = (fix.course - 90.0f) * 0.017453292f;
        M5.Display.drawLine(mx, my, mx + (int)(26 * cosf(a)),
                            my + (int)(26 * sinf(a)), col);
    }
    M5.Display.fillCircle(mx, my, 9, col);
    M5.Display.drawCircle(mx, my, 9, TFT_WHITE);
    M5.Display.drawCircle(mx, my, 10, TFT_WHITE);
}

void map_draw(const GnssFix &fix) {
    if (!g_visible || !g_view_set) return;
    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;

    coarse_fill_pending();

    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t origin = g_grid.origin;
    uint32_t gen = g_grid.generation;
    uint32_t states = 0;
    for (int i = 0; i < GRID_COUNT; i++)
        states = states * 7 + (uint32_t)g_grid.slots[i].state;
    xSemaphoreGive(g_glock);

    // View position within the canvas, and the marker's screen position.
    int32_t canvas_x = (int32_t)((g_view_wx - (double)origin.x) * SUBTILE_PX);
    int32_t canvas_y = (int32_t)((g_view_wy - (double)origin.y) * SUBTILE_PX);
    int mx = (int)((g_marker_wx - g_view_wx) * SUBTILE_PX);
    int my = visTop + (int)((g_marker_wy - g_view_wy) * SUBTILE_PX);

    static int32_t last_cx = INT32_MIN, last_cy = INT32_MIN;
    static uint32_t last_gen = 0, last_states = 0;
    static int last_mx = INT32_MIN, last_my = INT32_MIN;
    static bool have_last = false;

    bool viewMoved  = (canvas_x != last_cx || canvas_y != last_cy);
    bool tilesMoved = (gen != last_gen || states != last_states);
    bool markerMoved = (mx != last_mx || my != last_my);

    if (have_last && !viewMoved && !tilesMoved && !markerMoved && !g_force_redraw)
        return;

    uint64_t t0 = esp_timer_get_time();

    if (!have_last || viewMoved || tilesMoved || g_force_redraw) {
        // Everything can have changed; repaint the visible area.
        blit_region(0, visTop, SW, visBot - visTop, canvas_x, canvas_y);
    } else {
        // Only the marker moved, which is the common case now that the view
        // sits still inside the band. Repainting its old and new
        // neighbourhoods is a few thousand pixels instead of a million.
        const int R = MARKER_CLEAR_R;
        blit_region(last_mx - R, last_my - R, R * 2, R * 2, canvas_x, canvas_y);
        blit_region(mx - R, my - R, R * 2, R * 2, canvas_x, canvas_y);
    }

    M5.Display.setClipRect(0, visTop, SW, visBot - visTop);
    draw_marker(fix, mx, my);
    M5.Display.clearClipRect();

    last_cx = canvas_x; last_cy = canvas_y;
    last_gen = gen; last_states = states;
    last_mx = mx; last_my = my;
    have_last = true;
    g_force_redraw = false;

    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    g_stats.last_draw_ms = ms;
    g_stats.draw_total_ms += ms;
    g_stats.draws++;
    if (ms > g_stats.max_draw_ms) g_stats.max_draw_ms = ms;
}

// ---- background fetchers ---------------------------------------------------
// These hold the archive lock for a whole tile, so they can stall the
// renderer. Yielding while the render queue has work keeps the visible map
// responsive: a tile the user is looking at always beats one being stored
// for later.
static volatile int  g_pf_progress = 0;
static volatile bool g_pf_busy = false;

static void yield_to_renderer() {
    for (int i = 0; i < 200; i++) {                 // ~10 s ceiling
        if (uxQueueMessagesWaiting(g_jobs) == 0) return;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---- radius prefetch -------------------------------------------------------
struct PrefetchArgs {
    double lat, lon;
    int radius;
    uint8_t z1, z2;
};

static void prefetch_task(void *arg) {
    PrefetchArgs a = *(PrefetchArgs *)arg;
    free(arg);

    // Its own scratch: the render worker owns w_tile and may be using it.
    uint8_t *buf = (uint8_t *)ps_malloc(TILE_CAP);
    if (!buf) { g_pf_busy = false; vTaskDelete(nullptr); return; }

    int side = a.radius * 2 + 1;
    // Fetch at the data zoom: those are the tiles that actually get stored,
    // and asking for display-zoom ids would cache four times as many of the
    // wrong thing.
    const uint8_t zs[2] = { (uint8_t)DATA_ZOOM_OF(a.z1),
                            (uint8_t)DATA_ZOOM_OF(a.z2) };
    int levels = (zs[0] == zs[1]) ? 1 : 2;
    int total = side * side * levels;

    int done = 0, fetched = 0, cached = 0, missing = 0;
    uint64_t t0 = esp_timer_get_time();

    for (int zi = 0; zi < levels; zi++) {
        merc_pt_t p = merc_from_ll(a.lat, a.lon, zs[zi]);
        int32_t cx = (int32_t)p.x, cy = (int32_t)p.y;

        for (int dy = -a.radius; dy <= a.radius; dy++) {
            for (int dx = -a.radius; dx <= a.radius; dx++) {
                done++;
                g_pf_progress = (done * 100) / total;

                uint32_t n = TILE_CAP;
                bool from_net = false;
                if (netsource_get(zs[zi], (uint32_t)(cx + dx), (uint32_t)(cy + dy),
                                  buf, &n, &from_net)) {
                    if (from_net) fetched++; else cached++;
                } else {
                    missing++;
                }

                vTaskDelay(pdMS_TO_TICKS(5));
                yield_to_renderer();
            }
        }
        Serial.printf("prefetch: z%u done (%d fetched, %d cached, %d empty)\n",
                      zs[zi], fetched, cached, missing);
    }

    free(buf);
    Serial.printf("prefetch: %d tiles in %lu s (%d from network)\n",
                  total, (unsigned long)((esp_timer_get_time() - t0) / 1000000),
                  fetched);
    g_pf_progress = 100;
    g_pf_busy = false;
    vTaskDelete(nullptr);
}

bool map_prefetch_start(int radius, uint8_t z_wide, uint8_t z_close) {
    if (g_pf_busy || !g_centred) return false;
    PrefetchArgs *a = (PrefetchArgs *)malloc(sizeof(PrefetchArgs));
    if (!a) return false;

    // Centre on the grid rather than a fix, so this works even if the fix
    // has momentarily dropped.
    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t c = g_grid.origin;
    xSemaphoreGive(g_glock);
    merc_pt_t mid = { (double)c.x + GRID_N / 2.0, (double)c.y + GRID_N / 2.0, c.z };
    merc_to_ll(mid, &a->lat, &a->lon);

    a->radius = radius; a->z1 = z_wide; a->z2 = z_close;
    g_pf_busy = true;
    g_pf_progress = 0;

    // 10 KiB: inflate builds its Huffman tables on the stack (~3.5 KiB) on
    // top of this task's own frames.
    if (xTaskCreatePinnedToCore(prefetch_task, "prefetch", 10240, a, 1, nullptr, 1)
        != pdPASS) {
        free(a); g_pf_busy = false; return false;
    }
    Serial.printf("prefetch: %d x %d tiles at z%u around %.4f,%.4f\n",
                  radius * 2 + 1, radius * 2 + 1,
                  (unsigned)DATA_ZOOM_OF(z_close), a->lat, a->lon);
    return true;
}

// ---- world floor -----------------------------------------------------------
// z0-6 is only 5461 tiles but covers the entire planet, so there is always
// something to fall back to when the working level is missing. Checkpointed,
// because at roughly one tile a second the walk takes hours and will be
// interrupted.
static void world_task(void *arg) {
    (void)arg;
    uint8_t *buf = (uint8_t *)ps_malloc(TILE_CAP);
    if (!buf) { g_pf_busy = false; vTaskDelete(nullptr); return; }

    uint32_t total = 0;
    for (int z = 0; z <= WORLD_FLOOR_ZOOM; z++) total += 1u << (2 * z);
    uint32_t done = 0, fetched = 0, cached = 0;
    uint32_t done_this_run = 0;      // rate must not count the resumed part
    int64_t started = esp_timer_get_time();
    int64_t next_report = started;
    int64_t next_ckpt = started;

    uint8_t rz = 0; uint32_t rx = 0, ry = 0;
    bool resuming = netsource_world_load_pos(&rz, &rx, &ry);
    if (resuming) {
        for (int z = 0; z < rz; z++) done += 1u << (2 * z);
        // x is the outer loop and y the inner, so the linear index within a
        // level is x * n + y - not the other way round.
        done += rx * (1u << rz) + ry;
        Serial.printf("world: resuming at z%u %lu/%lu (%lu of %lu already done)\n",
                      rz, (unsigned long)rx, (unsigned long)ry,
                      (unsigned long)done, (unsigned long)total);
    }

    for (int z = 0; z <= WORLD_FLOOR_ZOOM; z++) {
        uint32_t n = 1u << z;
        if (resuming && z < rz) continue;
        for (uint32_t x = 0; x < n; x++) {
            if (resuming && z == rz && x < rx) continue;
            for (uint32_t y = 0; y < n; y++) {
                if (resuming && z == rz && x == rx && y < ry) continue;

                uint32_t len = TILE_CAP;
                bool from_net = false;
                if (netsource_get((uint8_t)z, x, y, buf, &len, &from_net)) {
                    if (from_net) fetched++; else cached++;
                }
                done++;
                done_this_run++;
                g_pf_progress = (int)((done * 100) / total);

                int64_t now = esp_timer_get_time();
                if (now >= next_report) {
                    next_report = now + 10000000;          // every 10 s
                    // Rate over work actually done in this run: on a resume
                    // `done` starts in the thousands while elapsed starts at
                    // zero, which reported six-figure tiles per second.
                    double elapsed = (now - started) / 1e6;
                    double rate = done_this_run / (elapsed > 0.5 ? elapsed : 0.5);
                    double left = (total - done) / (rate > 0.01 ? rate : 0.01);
                    Serial.printf("world: %lu/%lu (%d%%) z%d  %.1f tiles/s  "
                                  "~%.0f min left  [%lu new, %lu cached]\n",
                                  (unsigned long)done, (unsigned long)total,
                                  g_pf_progress, z, rate, left / 60.0,
                                  (unsigned long)fetched, (unsigned long)cached);
                }

                // Checkpoint on a timer rather than a tile count: the cost is
                // one small file write, and 30 s is a bounded amount of work
                // to redo after a power cut.
                if (now >= next_ckpt) {
                    next_ckpt = now + 30000000;
                    netsource_world_save_pos((uint8_t)z, x, y);
                }

                vTaskDelay(pdMS_TO_TICKS(3));
                yield_to_renderer();
                if (WiFi.status() != WL_CONNECTED) {
                    netsource_world_save_pos((uint8_t)z, x, y);
                    Serial.printf("world: link lost at %lu/%lu, resumes here "
                                  "next boot\n",
                                  (unsigned long)done, (unsigned long)total);
                    free(buf);
                    g_pf_busy = false;
                    vTaskDelete(nullptr);
                    return;
                }
            }
        }
    }

    free(buf);
    netsource_world_mark_done();
    Serial.printf("world: floor stored in %.0f min (%lu fetched, %lu already "
                  "cached) - the map will now draw anywhere offline\n",
                  (esp_timer_get_time() - started) / 6e7,
                  (unsigned long)fetched, (unsigned long)cached);
    g_pf_progress = 100;
    g_pf_busy = false;
    vTaskDelete(nullptr);
}

bool map_world_floor_start() {
    if (g_pf_busy) return false;
    g_pf_busy = true;
    g_pf_progress = 0;
    if (xTaskCreatePinnedToCore(world_task, "worldfloor", 10240, nullptr, 1,
                                nullptr, 1) != pdPASS) {
        g_pf_busy = false;
        return false;
    }
    Serial.printf("world: storing z0-%d floor in the background\n",
                  WORLD_FLOOR_ZOOM);
    return true;
}

int  map_prefetch_progress() { return g_pf_progress; }
bool map_prefetch_busy()     { return g_pf_busy; }

uint8_t map_zoom() { return g_zoom; }
bool map_has_fix_position() { return g_centred; }

void map_stats(MapStats *out) {
    *out = g_stats;
    out->queue_depth = g_jobs ? uxQueueMessagesWaiting(g_jobs) : 0;
}
