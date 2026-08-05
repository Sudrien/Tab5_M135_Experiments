// pmt_test.c - desktop harness for the PMTiles reader.
//
// Verifies every tile in the ground-truth list resolves to the right payload,
// that gaps report NOTFOUND, and that the Hilbert mapping round-trips.
//
//   cc -O2 -Wall -Wextra pmt_test.c pmtiles.c -lz -lm -o pmt_test
//   ./pmt_test flat.pmtiles expect.txt

#include "pmtiles.h"
#include "mercator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#define SUBTILE_PROBE_PX 512
#include <stdarg.h>

// ---- IO backend: plain FILE* -----------------------------------------------
// On the Tab5 this is the only piece that changes: fseek/fread against the SD
// card, or an HTTP Range request. The reader core is untouched.
static int file_read(void *ctx, uint64_t off, uint32_t len, uint8_t *dst) {
    FILE *f = (FILE *)ctx;
    if (fseek(f, (long)off, SEEK_SET) != 0) return -1;
    return fread(dst, 1, len, f) == len ? 0 : -1;
}

// ---- gzip inflate ----------------------------------------------------------
// windowBits 15|16 selects gzip framing. On ESP-IDF the same call works via
// the bundled zlib; miniz in ROM is an alternative if flash is tight.
static int gz_inflate(void *ctx, uint8_t codec, const uint8_t *src,
                      uint32_t src_len, uint8_t *dst, uint32_t *dst_len) {
    (void)ctx;
    if (codec != PMT_COMPRESS_GZIP) return -1;

    z_stream s;
    memset(&s, 0, sizeof s);
    if (inflateInit2(&s, 15 | 16) != Z_OK) return -1;
    s.next_in   = (Bytef *)src;
    s.avail_in  = src_len;
    s.next_out  = dst;
    s.avail_out = *dst_len;
    int r = inflate(&s, Z_FINISH);
    uint32_t produced = (uint32_t)s.total_out;
    inflateEnd(&s);
    if (r != Z_STREAM_END) return -1;
    *dst_len = produced;
    return 0;
}

// ---- helpers ---------------------------------------------------------------
static int fails = 0;
static void check(int cond, const char *fmt, ...) {
    if (!cond) {
        va_list ap; va_start(ap, fmt);
        fputs("  FAIL: ", stderr); vfprintf(stderr, fmt, ap); fputc('\n', stderr);
        va_end(ap);
        fails++;
    }
}

int main(int argc, char **argv) {
    const char *arc = argc > 1 ? argv[1] : "flat.pmtiles";
    const char *exp = argc > 2 ? argv[2] : "expect.txt";

    FILE *f = fopen(arc, "rb");
    if (!f) { perror(arc); return 1; }

    static uint8_t dir_buf[64 * 1024];
    static uint8_t raw_buf[64 * 1024];
    static uint8_t root_cache[64 * 1024];
    static uint8_t tile_buf[256 * 1024];

    pmt_t p;
    memset(&p, 0, sizeof p);
    p.read = file_read;
    p.inflate = gz_inflate;
    p.io_ctx = f;
    p.dir_buf = dir_buf;   p.dir_cap = sizeof dir_buf;
    p.raw_buf = raw_buf;   p.raw_cap = sizeof raw_buf;
    p.root_cache = root_cache; p.root_cache_cap = sizeof root_cache;

    pmt_err_t e = pmt_open(&p);
    if (e != PMT_OK) { fprintf(stderr, "pmt_open: %s\n", pmt_strerror(e)); return 1; }

    // ---- probe mode: dump one tile by lat/lon/zoom -------------------------
    //   ./pmt_test ARCHIVE --probe LAT LON Z [OUT.mvt]
    if (argc > 2 && strcmp(argv[2], "--probe") == 0) {
        if (argc < 6) { fprintf(stderr, "need LAT LON Z\n"); return 2; }
        double lat = atof(argv[3]), lon = atof(argv[4]);
        uint8_t pz = (uint8_t)atoi(argv[5]);
        const char *out = argc > 6 ? argv[6] : NULL;

        merc_pt_t pt = merc_from_ll(lat, lon, pz);
        uint32_t tx = (uint32_t)pt.x, ty = (uint32_t)pt.y;
        printf("%.5f,%.5f z%u -> tile %u/%u/%u (id %llu)\n",
               lat, lon, pz, pz, tx, ty,
               (unsigned long long)pmt_zxy_to_tileid(pz, tx, ty));
        printf("ground res: %.3f m/px at %d px/tile\n",
               merc_ground_res(lat, pz, SUBTILE_PROBE_PX), SUBTILE_PROBE_PX);

        uint64_t off; uint32_t len;
        e = pmt_find(&p, pz, tx, ty, &off, &len);
        if (e != PMT_OK) { printf("lookup: %s\n", pmt_strerror(e)); return 1; }
        printf("found at offset %llu, %u bytes (compressed)\n",
               (unsigned long long)off, len);

        uint32_t got = sizeof tile_buf;
        e = pmt_get(&p, pz, tx, ty, tile_buf, &got);
        if (e != PMT_OK) { printf("read: %s\n", pmt_strerror(e)); return 1; }

        // inflate to see the real MVT size
        static uint8_t mvt[2 * 1024 * 1024];
        uint32_t mlen = sizeof mvt;
        if (gz_inflate(NULL, p.hdr.tile_compression, tile_buf, got, mvt, &mlen) == 0) {
            printf("inflated: %u bytes MVT (ratio %.1fx)\n", mlen, (double)mlen / got);
            if (out) {
                FILE *o = fopen(out, "wb");
                if (o) { fwrite(mvt, 1, mlen, o); fclose(o);
                         printf("wrote %s\n", out); }
            }
        } else {
            printf("inflate failed (codec %u)\n", p.hdr.tile_compression);
        }
        fclose(f);
        return 0;
    }

    printf("== %s ==\n", arc);
    printf("zoom %u..%u  type=%u  tile_comp=%u  int_comp=%u  clustered=%u\n",
           p.hdr.min_zoom, p.hdr.max_zoom, p.hdr.tile_type,
           p.hdr.tile_compression, p.hdr.internal_compression, p.hdr.clustered);
    printf("root@%llu+%llu  leaf@%llu+%llu  data@%llu+%llu  entries=%llu\n",
           (unsigned long long)p.hdr.root_off, (unsigned long long)p.hdr.root_len,
           (unsigned long long)p.hdr.leaf_off, (unsigned long long)p.hdr.leaf_len,
           (unsigned long long)p.hdr.data_off, (unsigned long long)p.hdr.data_len,
           (unsigned long long)p.hdr.n_entries);

    // ---- 1. Hilbert round-trip over every tile in z0..z8 -------------------
    int rt = 0;
    for (uint8_t z = 0; z <= 8; z++) {
        uint32_t n = 1u << z;
        for (uint32_t x = 0; x < n; x++)
            for (uint32_t y = 0; y < n; y++) {
                uint64_t id = pmt_zxy_to_tileid(z, x, y);
                uint8_t z2; uint32_t x2, y2;
                pmt_tileid_to_zxy(id, &z2, &x2, &y2);
                if (z2 != z || x2 != x || y2 != y) {
                    check(0, "hilbert roundtrip %u/%u/%u -> id %llu -> %u/%u/%u",
                          z, x, y, (unsigned long long)id, z2, x2, y2);
                    goto rt_done;
                }
                rt++;
            }
    }
rt_done:
    printf("hilbert roundtrip: %d tiles ok\n", rt);

    // ---- 2. every expected tile resolves to the right payload --------------
    FILE *ef = fopen(exp, "r");
    if (!ef) { perror(exp); return 1; }
    int checked = 0, notfound = 0;
    int z, x, y, len;
    while (fscanf(ef, "%d %d %d %d", &z, &x, &y, &len) == 4) {
        uint32_t got = sizeof tile_buf;
        e = pmt_get(&p, (uint8_t)z, (uint32_t)x, (uint32_t)y, tile_buf, &got);
        if (e != PMT_OK) {
            check(0, "%d/%d/%d: %s", z, x, y, pmt_strerror(e));
            continue;
        }
        // payload is gzip'd by the writer only for real tiles; ours are raw
        char want[64];
        int wl = snprintf(want, sizeof want, "TILE:%d/%d/%d;", z, x, y);
        check((int)got == len, "%d/%d/%d: length %u want %d", z, x, y, got, len);
        check(memcmp(tile_buf, want, wl) == 0,
              "%d/%d/%d: payload mismatch", z, x, y);
        checked++;
    }
    fclose(ef);
    printf("payload check: %d tiles ok\n", checked);

    // ---- 3. deliberate gaps report NOTFOUND, not garbage -------------------
    for (int zz = 3; zz <= 4; zz++) {
        uint32_t n = 1u << zz;
        for (uint32_t xx = 0; xx < n; xx++)
            for (uint32_t yy = 0; yy < n; yy++) {
                if ((xx + yy) % 3 != 2) continue;
                uint32_t got = sizeof tile_buf;
                e = pmt_get(&p, (uint8_t)zz, xx, yy, tile_buf, &got);
                check(e == PMT_NOTFOUND, "%d/%u/%u: expected NOTFOUND, got %s",
                      zz, xx, yy, pmt_strerror(e));
                notfound++;
            }
    }
    printf("gap check: %d gaps correctly reported\n", notfound);

    // ---- 4. out-of-range zoom ---------------------------------------------
    uint32_t got = sizeof tile_buf;
    check(pmt_get(&p, 15, 0, 0, tile_buf, &got) == PMT_ERANGE,
          "zoom 15 should be out of range");

    // ---- 5. mercator sanity ------------------------------------------------
    struct { double lat, lon; uint8_t z; uint32_t x, y; } known[] = {
        // Greenwich observatory, z=0 -> single tile
        { 51.4779, -0.0015, 0, 0, 0 },
        // Null Island at z=1 sits on the seam; floor lands in tile (1,1)
        {  0.0,     0.0,    1, 1, 1 },
        // Tokyo Station z=10
        { 35.6812, 139.7671, 10, 909, 403 },
        // Sydney Opera House z=12
        { -33.8568, 151.2153, 12, 3768, 2457 },
    };
    for (size_t i = 0; i < sizeof known / sizeof known[0]; i++) {
        merc_pt_t pt = merc_from_ll(known[i].lat, known[i].lon, known[i].z);
        uint32_t tx = (uint32_t)pt.x, ty = (uint32_t)pt.y;
        check(tx == known[i].x && ty == known[i].y,
              "mercator %.4f,%.4f z%u -> %u/%u want %u/%u",
              known[i].lat, known[i].lon, known[i].z, tx, ty,
              known[i].x, known[i].y);

        // inverse should land back inside the same tile
        double lat2, lon2;
        merc_to_ll(pt, &lat2, &lon2);
        check(fabs(lat2 - known[i].lat) < 1e-6 && fabs(lon2 - known[i].lon) < 1e-6,
              "mercator inverse %.6f,%.6f != %.6f,%.6f",
              lat2, lon2, known[i].lat, known[i].lon);
    }
    printf("mercator: %zu reference points checked\n", sizeof known / sizeof known[0]);

    fclose(f);
    printf(fails ? "\n%d FAILURES\n" : "\nall ok (%d failures)\n", fails);
    return fails ? 1 : 0;
}
