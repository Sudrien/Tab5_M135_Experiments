// mapengine.cpp

#include "mapengine.h"
#include "style.h"
#include <M5Unified.h>
#include <SD.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <math.h>

extern "C" {
  #include "pmtiles.h"
  #include "mvt.h"
  #include "inflate.h"
  #include "raster.h"
}
#include "mercator.h"

// ---- configuration ---------------------------------------------------------
static const uint32_t TILE_CAP  = 192 * 1024;   // compressed tile scratch
static const uint32_t MVT_CAP   = 256 * 1024;   // inflated tile scratch
static const uint32_t DIR_CAP   =  32 * 1024;
static const uint32_t PT_CAP    =  2048;
static const uint32_t VAL_CAP   =  1024;
static const uint32_t EDGE_CAP  = 16384;
static const uint32_t XS_CAP    =  4096;

// ---- state -----------------------------------------------------------------
static tile_grid_t      g_grid;
static SemaphoreHandle_t g_glock = nullptr;
static QueueHandle_t     g_jobs  = nullptr;
static fs::File          g_file;
static pmt_t             g_pmt;
static uint8_t           g_zoom = 14;
static bool              g_centred = false;
static MapStats          g_stats;

// Worker-owned scratch. Nothing here is touched by the UI task.
static uint8_t *w_tile = nullptr, *w_mvt = nullptr;
static uint8_t *w_dir = nullptr, *w_raw = nullptr, *w_root = nullptr;
static int32_t *w_pts = nullptr;
static uint8_t *w_val = nullptr;
static rs_edge_t *w_edges = nullptr;
static uint16_t  *w_active = nullptr;
static int32_t   *w_xs = nullptr;
static int8_t    *w_dirs = nullptr;
static uint16_t  *w_cov = nullptr;

// Marker position within the grid canvas, in output pixels.
static double g_marker_gx = 0, g_marker_gy = 0;

// ---- PMTiles IO (worker task only) -----------------------------------------
static int sd_read(void *ctx, uint64_t off, uint32_t len, uint8_t *dst) {
    (void)ctx;
    if (!g_file.seek((uint32_t)off)) return -1;
    return g_file.read(dst, len) == (int)len ? 0 : -1;
}
static int sd_inflate(void *ctx, uint8_t codec, const uint8_t *src,
                      uint32_t src_len, uint8_t *dst, uint32_t *dst_len) {
    (void)ctx;
    if (codec != PMT_COMPRESS_GZIP) return -1;
    // Directories are verified: a corrupt one yields wrong tile offsets, which
    // is far harder to diagnose than wrong pixels.
    return inflate_auto(src, src_len, dst, dst_len) == INF_OK ? 0 : -1;
}

// ---- rendering one tile ----------------------------------------------------
static const char *g_want_layer = nullptr;
static int rl_layer(void *ctx, const mvt_layer_t *l) {
    (void)ctx;
    return g_want_layer && l->name_len == strlen(g_want_layer) &&
           memcmp(l->name, g_want_layer, l->name_len) == 0;
}

static tile_state_t render_tile(tile_id_t id, uint16_t *px) {
    uint64_t off; uint32_t len;
    pmt_err_t e = pmt_find(&g_pmt, id.z, (uint32_t)id.x, (uint32_t)id.y, &off, &len);
    if (e == PMT_NOTFOUND || e == PMT_ERANGE) return TILE_NODATA;
    if (e != PMT_OK) return TILE_ERROR;
    if (len > TILE_CAP) return TILE_ERROR;

    uint32_t got = TILE_CAP;
    if (pmt_get(&g_pmt, id.z, (uint32_t)id.x, (uint32_t)id.y, w_tile, &got) != PMT_OK)
        return TILE_ERROR;

    uint32_t mlen = MVT_CAP;
    // Tile payloads skip the CRC: corruption here shows up as visibly wrong
    // geometry rather than a wrong offset, and the check costs ~3.5x.
    if (inflate_auto_fast(w_tile, got, w_mvt, &mlen) != INF_OK) return TILE_ERROR;

    rs_t r;
    memset(&r, 0, sizeof r);
    r.px = px; r.w = r.h = SUBTILE_PX; r.extent = 4096;
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

    rs_clear(&r, rs_rgb(243, 240, 232));
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

        // Cheap early-out: if the grid has already moved on, skip the work
        // entirely rather than rendering something that will be discarded.
        uint32_t gen_now;
        uint16_t *px = nullptr;
        xSemaphoreTake(g_glock, portMAX_DELAY);
        gen_now = g_grid.generation;
        if (job.generation == gen_now && job.slot < GRID_COUNT)
            px = g_grid.slots[job.slot].pixels;
        xSemaphoreGive(g_glock);

        if (!px || job.generation != gen_now) { g_stats.dropped++; continue; }

        uint64_t t0 = esp_timer_get_time();
        tile_state_t res = render_tile(job.id, px);
        uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

        xSemaphoreTake(g_glock, portMAX_DELAY);
        int committed = grid_commit(&g_grid, &job, res);
        xSemaphoreGive(g_glock);

        if (!committed)                  g_stats.dropped++;
        else if (res == TILE_READY)    { g_stats.rendered++; g_stats.last_render_ms = ms; }
        else if (res == TILE_NODATA)     g_stats.notfound++;
        else                             g_stats.failed++;
    }
}

// ---- setup -----------------------------------------------------------------
static bool alloc_all() {
    w_tile   = (uint8_t *)ps_malloc(TILE_CAP);
    w_mvt    = (uint8_t *)ps_malloc(MVT_CAP);
    w_dir    = (uint8_t *)ps_malloc(DIR_CAP);
    w_raw    = (uint8_t *)ps_malloc(DIR_CAP);
    w_root   = (uint8_t *)ps_malloc(DIR_CAP);
    w_pts    = (int32_t *)ps_malloc(PT_CAP * 2 * sizeof(int32_t));
    w_val    = (uint8_t *)ps_malloc(VAL_CAP);
    w_edges  = (rs_edge_t *)ps_malloc(EDGE_CAP * sizeof(rs_edge_t));
    w_active = (uint16_t *)ps_malloc(XS_CAP * sizeof(uint16_t));
    w_xs     = (int32_t *)ps_malloc(XS_CAP * sizeof(int32_t));
    w_dirs   = (int8_t *)ps_malloc(XS_CAP);
    w_cov    = (uint16_t *)ps_malloc(SUBTILE_PX * sizeof(uint16_t));
    return w_tile && w_mvt && w_dir && w_raw && w_root && w_pts && w_val &&
           w_edges && w_active && w_xs && w_dirs && w_cov;
}

bool map_begin(const char *path, uint8_t zoom, int worker_core, int worker_prio) {
    if (!alloc_all()) { Serial.println("map: PSRAM alloc failed"); return false; }

    // 9 tile buffers, allocated once and never freed. Ownership rotates
    // between slots on every shift; the count stays constant for the life of
    // the program.
    static uint16_t *bufs[GRID_COUNT];
    for (int i = 0; i < GRID_COUNT; i++) {
        bufs[i] = (uint16_t *)ps_malloc((size_t)SUBTILE_PX * SUBTILE_PX * 2);
        if (!bufs[i]) { Serial.println("map: tile buffer alloc failed"); return false; }
        for (int p = 0; p < SUBTILE_PX * SUBTILE_PX; p++)
            bufs[i][p] = rs_rgb(228, 226, 220);
    }

    g_file = SD_MMC.open(path, FILE_READ);
    if (!g_file) g_file = SD.open(path, FILE_READ);
    if (!g_file) { Serial.printf("map: cannot open %s\n", path); return false; }

    memset(&g_pmt, 0, sizeof g_pmt);
    g_pmt.read = sd_read; g_pmt.inflate = sd_inflate;
    g_pmt.dir_buf = w_dir; g_pmt.dir_cap = DIR_CAP;
    g_pmt.raw_buf = w_raw; g_pmt.raw_cap = DIR_CAP;
    g_pmt.root_cache = w_root; g_pmt.root_cache_cap = DIR_CAP;
    pmt_err_t e = pmt_open(&g_pmt);
    if (e != PMT_OK) { Serial.printf("map: pmt_open %s\n", pmt_strerror(e)); return false; }

    g_zoom = zoom;
    if (g_zoom > g_pmt.hdr.max_zoom) g_zoom = g_pmt.hdr.max_zoom;
    if (g_zoom < g_pmt.hdr.min_zoom) g_zoom = g_pmt.hdr.min_zoom;

    style_init(SUBTILE_PX);

    g_glock = xSemaphoreCreateMutex();
    g_jobs  = xQueueCreate(32, sizeof(render_job_t));
    if (!g_glock || !g_jobs) return false;

    tile_id_t c = { g_zoom, 0, 0 };
    grid_init(&g_grid, bufs, c);
    g_centred = false;

    BaseType_t ok = xTaskCreatePinnedToCore(
        worker_task, "tilerender", 8192, nullptr, worker_prio, nullptr, worker_core);
    if (ok != pdPASS) { Serial.println("map: worker task failed"); return false; }

    Serial.printf("map: %s zoom %u..%u, rendering at z%u\n",
                  path, g_pmt.hdr.min_zoom, g_pmt.hdr.max_zoom, g_zoom);
    return true;
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
    tile_id_t c = { g_zoom, (int32_t)p.x, (int32_t)p.y };

    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    int n = grid_set_zoom(&g_grid, c, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    enqueue(jobs, n);
    g_centred = true;
    g_stats.shifts++;
}

void map_update(const GnssFix &fix) {
    if (!gnss_coarse(fix)) return;

    merc_pt_t p = merc_from_ll(fix.lat, fix.lon, g_zoom);

    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t centre = g_grid.center;
    xSemaphoreGive(g_glock);

    if (!g_centred) { recentre(fix); return; }

    // Marker position on the 3x3 canvas, used by the compositor.
    g_marker_gx = (p.x - (centre.x - GRID_MID)) * SUBTILE_PX;
    g_marker_gy = (p.y - (centre.y - GRID_MID)) * SUBTILE_PX;

    int dx, dy;
    xSemaphoreTake(g_glock, portMAX_DELAY);
    grid_drift(&g_grid, p.x, p.y, &dx, &dy);
    xSemaphoreGive(g_glock);
    if (!dx && !dy) return;

    // A jump of more than one tile - tunnel exit, cold relocate - is cheaper
    // to handle as a full recentre than as repeated single-step shifts.
    double rx = p.x - centre.x, ry = p.y - centre.y;
    if (rx < -1.0 || rx > 2.0 || ry < -1.0 || ry > 2.0) { recentre(fix); return; }

    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    int n = grid_shift(&g_grid, dx, dy, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    enqueue(jobs, n);
    g_stats.shifts++;
}

void map_set_zoom(uint8_t zoom, const GnssFix &fix) {
    if (zoom == g_zoom) return;
    if (zoom > g_pmt.hdr.max_zoom || zoom < g_pmt.hdr.min_zoom) return;
    g_zoom = zoom;
    if (gnss_coarse(fix)) recentre(fix);
}

// ---- compositing -----------------------------------------------------------
void map_draw(const GnssFix &fix) {
    const int SW = M5.Display.width(), SH = M5.Display.height();

    // The canvas is GRID_N * SUBTILE_PX square; the visible window is centred
    // on the marker so it stays put while the world moves underneath.
    int32_t cropx = (int32_t)(g_marker_gx) - SW / 2;
    int32_t cropy = (int32_t)(g_marker_gy) - SH / 2;

    xSemaphoreTake(g_glock, portMAX_DELAY);
    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {
            int i = r * GRID_N + c;
            // PENDING slots are skipped, which is the invariant that lets the
            // worker write pixels without any lock.
            if (g_grid.slots[i].state != TILE_READY) continue;
            int32_t sx = c * SUBTILE_PX - cropx;
            int32_t sy = r * SUBTILE_PX - cropy;
            if (sx >= SW || sy >= SH ||
                sx + SUBTILE_PX <= 0 || sy + SUBTILE_PX <= 0) continue;
            M5.Display.pushImage(sx, sy, SUBTILE_PX, SUBTILE_PX,
                                 g_grid.slots[i].pixels);
        }
    }
    xSemaphoreGive(g_glock);

    // Position marker, drawn on top rather than into any tile buffer.
    int mx = SW / 2, my = SH / 2;
    if (gnss_coarse(fix)) {
        uint16_t col = gnss_fine(fix) ? M5.Display.color565(30, 90, 220)
                                      : M5.Display.color565(150, 150, 160);
        M5.Display.fillCircle(mx, my, 9, col);
        M5.Display.drawCircle(mx, my, 9, TFT_WHITE);
        M5.Display.drawCircle(mx, my, 10, TFT_WHITE);

        // Heading needle. Course over ground is noise below walking pace, so
        // it is only drawn once actually moving.
        if (fix.speedKmh > 3.0f) {
            float a = (fix.course - 90.0f) * 0.017453292f;
            M5.Display.drawLine(mx, my,
                                mx + (int)(26 * cosf(a)),
                                my + (int)(26 * sinf(a)), col);
        }
    }
}

uint8_t map_zoom() { return g_zoom; }
bool map_has_fix_position() { return g_centred; }

void map_stats(MapStats *out) {
    *out = g_stats;
    out->queue_depth = g_jobs ? uxQueueMessagesWaiting(g_jobs) : 0;
}
