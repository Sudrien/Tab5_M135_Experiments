// tab5_tile_bench.ino - measure the real cost of the tile pipeline on hardware.
//
// Answers the three numbers the design hinges on:
//   1. SD random-read latency at PMTiles access sizes (directory ~0.5 KB,
//      tile ~60 KB). This dominates a cache miss.
//   2. PMTiles directory traversal cost, end to end.
//   3. MVT decode cost per tile, which desktop timing put at ~0.25 ms and
//      extrapolated to 4-8 ms here. That extrapolation is a guess; this
//      replaces it with a fact.
//
// SETUP
//   Copy to the SD card root:
//     local.pmtiles   - your extract
//     boston.mvt      - inflated tile from `pmt_test --probe`
//   Put these next to the .ino:
//     pmtiles.c pmtiles.h mercator.h mvt.c mvt.h
//
// Nothing here touches the display or I2C, so it runs standalone.

#include <M5Unified.h>
#include <esp_timer.h>

#include "bench_types.h"
#include <SD.h>
#include <SD_MMC.h>
#include <SPI.h>

extern "C" {
  #include "pmtiles.h"
  #include "mvt.h"
  #include "inflate.h"
  #include "raster.h"
  #include "style.h"
}
#include "mercator.h"

// Inflate comes from bundled inflate.c rather than zlib: the Arduino ESP32
// core does not reliably expose zlib headers, and this way the build has no
// external dependency at all.

// ---------------------------------------------------------------------------
static fs::FS *g_fs      = nullptr;
static const char *g_bus = "none";

static const char *PMT_PATH = "/local.pmtiles";
static const char *MVT_PATH = "/boston.mvt";
static const int SUBTILE_PX = 512;

// Scratch, all PSRAM.
static uint8_t *tile_buf = nullptr;   static const uint32_t TILE_CAP = 192 * 1024;
static uint8_t *dir_buf  = nullptr;   static const uint32_t DIR_CAP  =  32 * 1024;
static uint8_t *raw_buf  = nullptr;
static uint8_t *root_c   = nullptr;
static int32_t *pt_buf   = nullptr;   static const uint32_t PT_CAP   = 2048;
static uint8_t *val_sty  = nullptr;   static const uint32_t VAL_CAP  = 1024;

// ---- PMTiles IO backend: SD -----------------------------------------------
// One handle held open for the whole run. Reopening per read would swamp the
// measurement with FatFs directory lookups, which is not what we are testing.
static File g_pmt;

static int sd_read(void *ctx, uint64_t off, uint32_t len, uint8_t *dst) {
  (void)ctx;
  if (!g_pmt.seek((uint32_t)off)) return -1;
  return g_pmt.read(dst, len) == (int)len ? 0 : -1;
}

static int sd_inflate(void *ctx, uint8_t codec, const uint8_t *src,
                      uint32_t src_len, uint8_t *dst, uint32_t *dst_len) {
  (void)ctx;
  if (codec != PMT_COMPRESS_GZIP) return -1;
  // Directories get the CRC check: a corrupt one yields wrong tile offsets,
  // which is far harder to diagnose than wrong pixels.
  return inflate_auto(src, src_len, dst, dst_len) == INF_OK ? 0 : -1;
}

// ---- timing helpers --------------------------------------------------------
static inline uint64_t us() { return (uint64_t)esp_timer_get_time(); }

static void add(Stat &s, uint64_t v) {
  s.n++; s.sum += v;
  if (v < s.mn) s.mn = v;
  if (v > s.mx) s.mx = v;
}
static void report(const char *name, Stat &s) {
  if (!s.n) { Serial.printf("%-28s no samples\n", name); return; }
  Serial.printf("%-28s n=%-5llu avg %7.1f %s   min %llu  max %llu\n",
                name, (unsigned long long)s.n,
                (double)s.sum / s.n, "us",
                (unsigned long long)s.mn, (unsigned long long)s.mx);
}

// ---- mount -----------------------------------------------------------------
// Prefer 4-bit SDMMC: it is several times faster than SPI, and this workload
// is latency-bound. Falls back to SPI so the sketch still runs if the Tab5
// routes the slot that way.
static bool mountSD() {
  if (SD_MMC.begin("/sdcard", false)) { g_fs = &SD_MMC; g_bus = "SDMMC 4-bit"; return true; }
  if (SD_MMC.begin("/sdcard", true))  { g_fs = &SD_MMC; g_bus = "SDMMC 1-bit"; return true; }
  if (SD.begin())                     { g_fs = &SD;     g_bus = "SPI";         return true; }
  return false;
}

// ---- benchmarks ------------------------------------------------------------
static void benchSequential() {
  File f = g_fs->open(PMT_PATH, FILE_READ);
  if (!f) { Serial.println("seq: open failed"); return; }

  const uint32_t BS = 64 * 1024;
  uint32_t total = 0;
  uint64_t t0 = us();
  while (total < 8u * 1024 * 1024) {
    int n = f.read(tile_buf, BS);
    if (n <= 0) break;
    total += n;
  }
  uint64_t dt = us() - t0;
  f.close();
  Serial.printf("sequential read   %.2f MB in %.1f ms -> %.2f MB/s\n",
                total / 1048576.0, dt / 1000.0,
                (total / 1048576.0) / (dt / 1e6));
}

static void benchRandom(uint32_t sz, uint32_t iters, uint32_t fileSize) {
  File f = g_fs->open(PMT_PATH, FILE_READ);
  if (!f) return;
  Stat s;
  randomSeed(12345);
  for (uint32_t i = 0; i < iters; i++) {
    uint32_t off = random(0, (fileSize > sz) ? (fileSize - sz) : 1);
    uint64_t t0 = us();
    f.seek(off);
    int n = f.read(tile_buf, sz);
    uint64_t dt = us() - t0;
    if (n == (int)sz) add(s, dt);
  }
  f.close();
  char name[48];
  snprintf(name, sizeof name, "random read %u B", (unsigned)sz);
  report(name, s);
}

// ---- MVT decode ------------------------------------------------------------
// Counting callbacks only. The rasteriser is not written yet; this isolates
// parse cost so we know how much of the frame budget is left for drawing.
static int cb_layer(void *ctx, const mvt_layer_t *l) { (void)ctx; (void)l; return 1; }
static uint8_t cb_style(void *ctx, const mvt_layer_t *l, const char *s, uint32_t n) {
  (void)ctx; (void)l; (void)s; return n ? 1 : 0;
}
static int cb_part(void *ctx, const mvt_part_t *p) {
  Counters *c = (Counters *)ctx;
  c->parts++; c->points += p->n_pts;
  return 0;
}

// The decoder makes several passes over the tile and reads varints a byte at
// a time. On PSRAM that is a lot of slow, cache-hostile traffic; the same
// buffer in internal SRAM should be markedly faster. This measures the gap.
static void benchDecodeIn(const char *where, uint8_t *buf, uint32_t sz) {
  mvt_decoder_t d; memset(&d, 0, sizeof d);
  Counters c;
  d.layer_cb = cb_layer; d.style_cb = cb_style; d.part_cb = cb_part; d.ctx = &c;
  d.pt_buf = pt_buf;     d.pt_cap  = PT_CAP;
  d.val_style = val_sty; d.val_cap = VAL_CAP;

  c = Counters();
  if (mvt_decode(&d, buf, sz) != MVT_OK) {
    Serial.printf("%-28s decode failed\n", where);
    return;
  }
  Stat s;
  for (int i = 0; i < 20; i++) {
    c = Counters();
    uint64_t t0 = us();
    mvt_decode(&d, buf, sz);
    add(s, us() - t0);
  }
  report(where, s);
}

static void benchDecode() {
  File f = g_fs->open(MVT_PATH, FILE_READ);
  if (!f) { Serial.printf("decode: %s not found\n", MVT_PATH); return; }
  uint32_t sz = f.size();
  if (sz > TILE_CAP) { Serial.println("decode: tile larger than buffer"); f.close(); return; }
  f.read(tile_buf, sz);
  f.close();
  Serial.printf("\nMVT tile: %u bytes\n", (unsigned)sz);

  mvt_decoder_t d; memset(&d, 0, sizeof d);
  Counters c;
  d.layer_cb = cb_layer; d.style_cb = cb_style; d.part_cb = cb_part; d.ctx = &c;
  d.pt_buf = pt_buf;    d.pt_cap  = PT_CAP;
  d.val_style = val_sty; d.val_cap = VAL_CAP;

  // one warm run for correctness
  c = Counters();
  mvt_err_t e = mvt_decode(&d, tile_buf, sz);
  Serial.printf("decode: %s  layers=%u features=%u parts=%u points=%u\n",
                mvt_strerror(e), d.stat_layers, d.stat_features,
                d.stat_parts, d.stat_points);
  Serial.printf("peak part %u pts (cap %u), peak values %u (cap %u)\n",
                d.stat_max_part_pts, (unsigned)PT_CAP,
                d.stat_max_values, (unsigned)VAL_CAP);
  if (e != MVT_OK) return;

  benchDecodeIn("decode from PSRAM", tile_buf, sz);

  // Same tile, internal SRAM. 84 KB against ~420 KB free, so it fits.
  uint8_t *sram = (uint8_t *)heap_caps_malloc(sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (sram) {
    memcpy(sram, tile_buf, sz);
    benchDecodeIn("decode from internal SRAM", sram, sz);
    heap_caps_free(sram);
  } else {
    Serial.println("decode from internal SRAM  alloc failed");
  }

  // Scratch buffers in SRAM too - pt_buf is written once per point.
  int32_t *pt_s  = (int32_t *)heap_caps_malloc(PT_CAP * 2 * sizeof(int32_t),
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  uint8_t *val_s = (uint8_t *)heap_caps_malloc(VAL_CAP,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  uint8_t *tile_s = (uint8_t *)heap_caps_malloc(sz,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (pt_s && val_s && tile_s) {
    memcpy(tile_s, tile_buf, sz);
    int32_t *old_pt = pt_buf;  uint8_t *old_val = val_sty;
    pt_buf = pt_s; val_sty = val_s;
    benchDecodeIn("decode, all buffers SRAM", tile_s, sz);
    pt_buf = old_pt; val_sty = old_val;
  }
  if (pt_s) heap_caps_free(pt_s);
  if (val_s) heap_caps_free(val_s);
  if (tile_s) heap_caps_free(tile_s);
}

// ---- PMTiles lookup --------------------------------------------------------
static void benchLookup() {
  g_pmt = g_fs->open(PMT_PATH, FILE_READ);
  if (!g_pmt) { Serial.println("pmtiles: open failed"); return; }

  pmt_t p; memset(&p, 0, sizeof p);
  p.read = sd_read; p.inflate = sd_inflate; p.io_ctx = nullptr;
  p.dir_buf = dir_buf; p.dir_cap = DIR_CAP;
  p.raw_buf = raw_buf; p.raw_cap = DIR_CAP;
  p.root_cache = root_c; p.root_cache_cap = DIR_CAP;

  uint64_t t0 = us();
  pmt_err_t e = pmt_open(&p);
  Serial.printf("\npmt_open: %s (%llu us)\n", pmt_strerror(e),
                (unsigned long long)(us() - t0));
  if (e != PMT_OK) { g_pmt.close(); return; }
  Serial.printf("  zoom %u..%u  root %llu B  entries %llu\n",
                p.hdr.min_zoom, p.hdr.max_zoom,
                (unsigned long long)p.hdr.root_len,
                (unsigned long long)p.hdr.n_entries);

  // Walk a 3x3 grid around Boston at max zoom: exactly the access pattern a
  // real shift produces.
  uint8_t z = p.hdr.max_zoom;
  merc_pt_t ctr = merc_from_ll(42.3601, -71.0589, z);
  uint32_t cx = (uint32_t)ctr.x, cy = (uint32_t)ctr.y;

  Stat find, get, infl;
  uint32_t inflated_bytes = 0;
  uint32_t hits = 0, misses = 0, errors = 0, bytes = 0;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      uint64_t off; uint32_t len;
      uint64_t t1 = us();
      pmt_err_t r = pmt_find(&p, z, cx + dx, cy + dy, &off, &len);
      add(find, us() - t1);
      if (r == PMT_NOTFOUND) { misses++; continue; }
      if (r != PMT_OK) {
        Serial.printf("  pmt_find %u/%d/%d ERROR: %s\n",
                      z, cx + dx, cy + dy, pmt_strerror(r));
        errors++; continue;
      }

      uint32_t got = TILE_CAP;
      uint64_t t2 = us();
      r = pmt_get(&p, z, cx + dx, cy + dy, tile_buf, &got);
      add(get, us() - t2);
      if (r != PMT_OK) { errors++; continue; }
      hits++; bytes += got;

      // Inflate the payload we just read, so the timing reflects real tile
      // sizes rather than a synthetic buffer.
      uint32_t inf_len = TILE_CAP - got;
      uint64_t t3 = us();
      inf_err_t ie = inflate_auto_fast(tile_buf, got, tile_buf + got, &inf_len);
      add(infl, us() - t3);
      if (ie != INF_OK)
        Serial.printf("  inflate failed: %s\n", inflate_strerror(ie));
      else if (!inflated_bytes) inflated_bytes = inf_len;
    }
  }
  Serial.printf("3x3 grid at z%u: %u hits, %u empty, %u ERRORS, %u bytes\n",
                z, hits, misses, errors, bytes);
  report("pmt_find (dir traversal)", find);
  report("pmt_get  (find + read)", get);
  report("inflate (no CRC)", infl);
  if (inflated_bytes)
    Serial.printf("  one tile inflates to %u bytes\n", (unsigned)inflated_bytes);

  if (hits) {
    Serial.printf("\n-> one full grid fill costs ~%.1f ms of SD time\n",
                  (double)get.sum / 1000.0);
    double per = (double)get.sum / get.n + (double)infl.sum / (infl.n ? infl.n : 1);
    Serial.printf("-> read+inflate per tile ~%.1f ms\n", per / 1000.0);
    Serial.printf("-> 3-tile axis shift ~%.1f ms, 5-tile diagonal ~%.1f ms\n",
                  per * 3 / 1000.0, per * 5 / 1000.0);
    Serial.println("   (decode and rasterise are on top of this)");
  }
  g_pmt.close();
}

// ---- rasterise -------------------------------------------------------------
// Draws the real tile in cartographic order, timing each layer. Layer order
// is achieved by decoding once per layer with the filter set; skipped layers
// cost only a length-walk, so the total decode work is close to a single
// full pass.
static const char *g_want = nullptr;

static int rl_layer(void *ctx, const mvt_layer_t *l) {
  (void)ctx;
  return g_want && l->name_len == strlen(g_want) &&
         memcmp(l->name, g_want, l->name_len) == 0;
}

static void benchRaster() {
  File f = g_fs->open(MVT_PATH, FILE_READ);
  if (!f) { Serial.printf("\nraster: %s not found\n", MVT_PATH); return; }
  uint32_t sz = f.size();
  f.read(tile_buf, sz);
  f.close();

  const int PX = SUBTILE_PX;
  uint16_t *px = (uint16_t *)ps_malloc((size_t)PX * PX * sizeof(uint16_t));
  rs_edge_t *edges = (rs_edge_t *)ps_malloc(16384 * sizeof(rs_edge_t));
  uint16_t *active = (uint16_t *)ps_malloc(4096 * sizeof(uint16_t));
  int32_t  *xs     = (int32_t *)ps_malloc(4096 * sizeof(int32_t));
  int8_t   *dirs   = (int8_t *)ps_malloc(4096);
  uint16_t *cov    = (uint16_t *)ps_malloc(PX * sizeof(uint16_t));
  if (!px || !edges || !active || !xs || !dirs || !cov) {
    Serial.println("raster: PSRAM alloc failed"); return;
  }

  style_init(PX);

  rs_t r;
  memset(&r, 0, sizeof r);
  r.px = px; r.w = r.h = PX; r.extent = 4096;
  r.edges = edges;  r.edge_cap = 16384;
  r.active = active; r.active_cap = 4096;
  r.xs = xs; r.dirs = dirs; r.xs_cap = 4096;
  r.cov = cov;
  r.styles = STYLES; r.n_styles = S_COUNT;
  r.cur_feature = -1;

  mvt_decoder_t d;
  memset(&d, 0, sizeof d);
  d.layer_cb = rl_layer;
  d.style_cb = style_lookup;
  d.part_cb  = rs_part;
  d.ctx      = &r;
  d.pt_buf   = pt_buf;    d.pt_cap  = PT_CAP;
  d.val_style = val_sty;  d.val_cap = VAL_CAP;

  Serial.printf("\nrasterising %ux%u, AA=%dx\n", PX, PX, RS_SUBSAMPLES);
  rs_clear(&r, rs_rgb(243, 240, 232));

  uint64_t total = 0;
  for (int i = 0; i < N_DRAW_ORDER; i++) {
    g_want = DRAW_ORDER[i];
    uint64_t t0 = us();
    mvt_decode(&d, tile_buf, sz);
    rs_flush(&r);
    uint64_t dt = us() - t0;
    total += dt;
    Serial.printf("  %-11s %7.1f ms\n", DRAW_ORDER[i], dt / 1000.0);
  }
  Serial.printf("  %-11s %7.1f ms   (spans %u, lines %u)\n",
                "TOTAL", total / 1000.0, r.stat_spans, r.stat_lines);

  // Blit cost: the number that decides whether progressive publishing per
  // layer is worth doing at all.
  uint64_t t0 = us();
  M5.Display.pushImage(0, 0, PX, PX, px);
  uint64_t blit = us() - t0;
  Serial.printf("  blit %ux%u to panel: %.1f ms\n", PX, PX, blit / 1000.0);

  Serial.printf("\n-> full pipeline per tile: read %.0f + inflate %.0f + decode %.0f"
                " + raster %.0f ms\n",
                10.5, 18.6, 19.6, total / 1000.0);

  free(px); free(edges); free(active); free(xs); free(dirs); free(cov);
}

// ---------------------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  delay(400);

  Serial.println("\n=== Tab5 tile pipeline benchmark ===");
  Serial.printf("PSRAM free: %u KB, internal free: %u KB\n",
                (unsigned)(ESP.getFreePsram() / 1024),
                (unsigned)(ESP.getFreeHeap() / 1024));

  tile_buf = (uint8_t *)ps_malloc(TILE_CAP);
  dir_buf  = (uint8_t *)ps_malloc(DIR_CAP);
  raw_buf  = (uint8_t *)ps_malloc(DIR_CAP);
  root_c   = (uint8_t *)ps_malloc(DIR_CAP);
  pt_buf   = (int32_t *)ps_malloc(PT_CAP * 2 * sizeof(int32_t));
  val_sty  = (uint8_t *)ps_malloc(VAL_CAP);
  if (!tile_buf || !dir_buf || !raw_buf || !root_c || !pt_buf || !val_sty) {
    Serial.println("PSRAM allocation failed"); return;
  }

  if (!mountSD()) { Serial.println("SD mount FAILED on all buses"); return; }
  Serial.printf("SD mounted via %s\n", g_bus);

  File f = g_fs->open(PMT_PATH, FILE_READ);
  uint32_t fsz = f ? f.size() : 0;
  if (f) f.close();
  Serial.printf("%s: %u bytes\n\n", PMT_PATH, (unsigned)fsz);
  if (!fsz) { Serial.println("archive missing - copy it to the card root"); return; }

  benchSequential();
  Serial.println();
  benchRandom(512,   200, fsz);   // directory-sized
  benchRandom(4096,  200, fsz);   // one FAT cluster
  benchRandom(65536, 100, fsz);   // tile-sized

  benchLookup();
  benchDecode();

  benchRaster();
  Serial.println("\n=== done ===");
}

void loop() { delay(1000); }
