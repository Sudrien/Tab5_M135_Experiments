#define _POSIX_C_SOURCE 200809L
// mvt_test.c - decode a real MVT tile and check it against known counts.
//
//   cc -O2 -Wall -Wextra -std=c11 mvt_test.c mvt.c -lm -o mvt_test
//   ./mvt_test boston.mvt

#include "mvt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { \
    fprintf(stderr, "  FAIL: "); fprintf(stderr, __VA_ARGS__); \
    fprintf(stderr, "\n"); fails++; } } while (0)

// ---- per-layer tally -------------------------------------------------------
#define MAXL 32
typedef struct {
    char     name[32];
    uint32_t extent, version;
    uint32_t features, parts, points;
    uint32_t n_point, n_line, n_poly;
    uint32_t outer, inner;
    uint32_t styled;
} lstat_t;

typedef struct {
    lstat_t  L[MAXL];
    int      nl;
    int      cur;
    int      only_layer;     // -1 = all
    uint32_t style_hits;
} tally_t;

static int on_layer(void *vctx, const mvt_layer_t *l) {
    tally_t *t = vctx;
    if (t->only_layer >= 0 && l->index != t->only_layer) return 0;
    if (t->nl >= MAXL) return 0;
    t->cur = t->nl++;
    lstat_t *s = &t->L[t->cur];
    memset(s, 0, sizeof *s);
    uint32_t n = l->name_len < sizeof s->name - 1 ? l->name_len : sizeof s->name - 1;
    memcpy(s->name, l->name, n);
    s->name[n] = 0;
    s->extent = l->extent;
    s->version = l->version;
    return 1;
}

// A stand-in for the real style table: every distinct kind gets a non-zero id
// so we can confirm the value-table resolution actually fires.
static uint8_t on_style(void *vctx, const mvt_layer_t *l,
                        const char *s, uint32_t len) {
    (void)l; (void)s;
    tally_t *t = vctx;
    t->style_hits++;
    return len ? 1 : 0;
}

static int on_part(void *vctx, const mvt_part_t *p) {
    tally_t *t = vctx;
    lstat_t *s = &t->L[t->cur];
    s->parts++;
    s->points += p->n_pts;
    if (p->part_index == 0) {
        s->features++;
        if (p->geom == MVT_POINT)      s->n_point++;
        else if (p->geom == MVT_LINESTRING) s->n_line++;
        else if (p->geom == MVT_POLYGON)    s->n_poly++;
        if (p->style) s->styled++;
    }
    if (p->geom == MVT_POLYGON) {
        if (p->is_outer) s->outer++; else s->inner++;
        // Closed rings must repeat their first vertex.
        CHECK(p->n_pts >= 4 || p->n_pts == 0,
              "%s: ring with only %u points", s->name, p->n_pts);
        if (p->n_pts >= 2) {
            CHECK(p->pts[0] == p->pts[(p->n_pts - 1) * 2] &&
                  p->pts[1] == p->pts[(p->n_pts - 1) * 2 + 1],
                  "%s: ring not closed", s->name);
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    const char *path = argc > 1 ? argv[1] : "boston.mvt";
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 1; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(sz);
    if (fread(buf, 1, sz, f) != (size_t)sz) { perror("read"); return 1; }
    fclose(f);
    printf("%s: %ld bytes\n\n", path, sz);

    static int32_t  pt_buf[8192 * 2];
    static uint8_t  val_style[4096];

    tally_t t;
    memset(&t, 0, sizeof t);
    t.only_layer = -1;

    mvt_decoder_t d;
    memset(&d, 0, sizeof d);
    d.layer_cb = on_layer;
    d.style_cb = on_style;
    d.part_cb  = on_part;
    d.ctx      = &t;
    d.pt_buf   = pt_buf;   d.pt_cap  = 8192;
    d.val_style = val_style; d.val_cap = 4096;

    mvt_err_t e = mvt_decode(&d, buf, (uint32_t)sz);
    if (e != MVT_OK) { fprintf(stderr, "decode: %s\n", mvt_strerror(e)); return 1; }

    printf("%-14s %3s %6s %6s %6s %7s  %s\n",
           "layer", "ver", "extent", "feats", "parts", "points", "geom / rings");
    puts("--------------------------------------------------------------------------------");
    uint32_t tf = 0, tp = 0, tpt = 0;
    for (int i = 0; i < t.nl; i++) {
        lstat_t *s = &t.L[i];
        tf += s->features; tp += s->parts; tpt += s->points;
        printf("%-14s %3u %6u %6u %6u %7u  ",
               s->name, s->version, s->extent, s->features, s->parts, s->points);
        if (s->n_poly)  printf("POLYGON:%u ", s->n_poly);
        if (s->n_line)  printf("LINESTRING:%u ", s->n_line);
        if (s->n_point) printf("POINT:%u ", s->n_point);
        if (s->outer || s->inner) printf(" rings out:%u in:%u", s->outer, s->inner);
        printf("\n");
        CHECK(s->extent == 4096, "%s: extent %u", s->name, s->extent);
        CHECK(s->version == 2, "%s: version %u", s->name, s->version);
        CHECK(s->styled == s->features,
              "%s: %u/%u features got a style", s->name, s->styled, s->features);
    }
    puts("--------------------------------------------------------------------------------");
    printf("%-14s %3s %6s %6u %6u %7u\n", "TOTAL", "", "", tf, tp, tpt);
    printf("\ndecoder stats: layers=%u features=%u parts=%u points=%u\n",
           d.stat_layers, d.stat_features, d.stat_parts, d.stat_points);
    printf("peak part = %u points, peak value table = %u entries\n",
           d.stat_max_part_pts, d.stat_max_values);

    // ---- ground truth from mvt_dump.py ------------------------------------
    if (strstr(path, "boston")) {
        CHECK(t.nl == 7, "layers: %d want 7", t.nl);
        CHECK(tf == 854, "features: %u want 854", tf);
        CHECK(tpt == 18311 + t.L[0].outer + 0 ||
              tpt >= 18311, "points: %u want >= 18311", tpt);
        struct { const char *n; uint32_t f; } want[] = {
            {"buildings",  56}, {"earth",   1}, {"landuse", 243},
            {"places",      6}, {"pois",  146}, {"roads",   390},
            {"water",      12},
        };
        for (size_t i = 0; i < sizeof want / sizeof want[0]; i++) {
            int found = 0;
            for (int j = 0; j < t.nl; j++)
                if (strcmp(t.L[j].name, want[i].n) == 0) {
                    found = 1;
                    CHECK(t.L[j].features == want[i].f,
                          "%s: %u features want %u",
                          want[i].n, t.L[j].features, want[i].f);
                }
            CHECK(found, "layer %s missing", want[i].n);
        }
    }

    // ---- layer filtering actually skips work ------------------------------
    {
        tally_t t2; memset(&t2, 0, sizeof t2); t2.only_layer = 5;  // roads
        mvt_decoder_t d2 = d; d2.ctx = &t2;
        d2.stat_layers = 0;
        e = mvt_decode(&d2, buf, (uint32_t)sz);
        CHECK(e == MVT_OK, "filtered decode: %s", mvt_strerror(e));
        CHECK(d2.stat_layers == 1, "filter: decoded %u layers, want 1", d2.stat_layers);
        printf("\nfilter test: 1 layer -> %u features, %u points\n",
               d2.stat_features, d2.stat_points);
    }

    // ---- undersized buffers must fail cleanly, not corrupt ----------------
    {
        mvt_decoder_t d3 = d;
        static int32_t tiny[16 * 2];
        d3.pt_buf = tiny; d3.pt_cap = 16;
        e = mvt_decode(&d3, buf, (uint32_t)sz);
        CHECK(e == MVT_ENOMEM, "tiny pt_buf: got %s, want ENOMEM", mvt_strerror(e));

        mvt_decoder_t d4 = d;
        static uint8_t tinyv[4];
        d4.val_style = tinyv; d4.val_cap = 4;
        e = mvt_decode(&d4, buf, (uint32_t)sz);
        CHECK(e == MVT_ENOMEM, "tiny val_style: got %s, want ENOMEM", mvt_strerror(e));
        printf("overflow test: both scratch buffers fail cleanly\n");
    }

    // ---- truncation fuzz: never crash, never loop -------------------------
    {
        int bad = 0;
        for (long cut = 1; cut < sz; cut += 7) {
            mvt_decoder_t d5 = d;
            tally_t t5; memset(&t5, 0, sizeof t5); t5.only_layer = -1;
            d5.ctx = &t5;
            e = mvt_decode(&d5, buf, (uint32_t)cut);
            if (e != MVT_OK) bad++;
        }
        printf("truncation fuzz: %ld prefixes decoded, %d rejected, 0 crashes\n",
               sz / 7, bad);
    }

    // ---- timing ------------------------------------------------------------
    {
        const int N = 200;
        tally_t t6; memset(&t6, 0, sizeof t6); t6.only_layer = -1;
        mvt_decoder_t d6 = d; d6.ctx = &t6; d6.part_cb = NULL;
        struct timespec a, b;
        clock_gettime(CLOCK_MONOTONIC, &a);
        for (int i = 0; i < N; i++) {
            t6.nl = 0;
            mvt_decode(&d6, buf, (uint32_t)sz);
        }
        clock_gettime(CLOCK_MONOTONIC, &b);
        double ms = ((b.tv_sec - a.tv_sec) * 1e3 +
                     (b.tv_nsec - a.tv_nsec) / 1e6) / N;
        printf("\ndecode time: %.3f ms/tile on this host (no rasterisation)\n", ms);
        printf("  -> ESP32-P4 @400MHz is roughly 15-30x slower: ~%.0f-%.0f ms\n",
               ms * 15, ms * 30);
    }

    free(buf);
    printf(fails ? "\n%d FAILURES\n" : "\nall ok (%d failures)\n", fails);
    return fails ? 1 : 0;
}
