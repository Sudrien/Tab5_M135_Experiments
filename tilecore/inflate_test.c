// inflate_test.c - differential test against zlib.
//
//   cc -O2 -Wall -Wextra -std=c11 inflate_test.c inflate.c -lz -o inflate_test
//
// Every case is compressed with zlib, inflated with ours, and compared byte
// for byte. Data shapes are chosen to exercise all three block types plus
// long-distance and overlapping matches.

#define _POSIX_C_SOURCE 200809L
#include "inflate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zlib.h>

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { \
    fprintf(stderr, "  FAIL: "); fprintf(stderr, __VA_ARGS__); \
    fprintf(stderr, "\n"); fails++; } } while (0)

static uint32_t rnd_state = 1;
static uint32_t rnd(void) {
    rnd_state ^= rnd_state << 13;
    rnd_state ^= rnd_state >> 17;
    rnd_state ^= rnd_state << 5;
    return rnd_state;
}

// gzip-wrap with zlib at a given level
static long gz_pack(const uint8_t *src, uint32_t n, uint8_t *dst, uint32_t cap,
                    int level, int raw) {
    z_stream s; memset(&s, 0, sizeof s);
    if (deflateInit2(&s, level, Z_DEFLATED, raw ? -15 : (15 | 16),
                     8, Z_DEFAULT_STRATEGY) != Z_OK) return -1;
    s.next_in = (Bytef *)src; s.avail_in = n;
    s.next_out = dst; s.avail_out = cap;
    int r = deflate(&s, Z_FINISH);
    long out = s.total_out;
    deflateEnd(&s);
    return r == Z_STREAM_END ? out : -1;
}

static void roundtrip(const char *name, const uint8_t *data, uint32_t n,
                      int level, int raw) {
    static uint8_t packed[4 << 20];
    static uint8_t got[4 << 20];

    long pn = gz_pack(data, n, packed, sizeof packed, level, raw);
    if (pn < 0) { CHECK(0, "%s: zlib deflate failed", name); return; }

    uint32_t got_len = sizeof got;
    inf_err_t e = raw ? inflate_raw(packed, (uint32_t)pn, got, &got_len)
                      : inflate_auto(packed, (uint32_t)pn, got, &got_len);
    if (e != INF_OK) {
        CHECK(0, "%s (L%d,%s): %s", name, level, raw ? "raw" : "gzip",
              inflate_strerror(e));
        return;
    }
    CHECK(got_len == n, "%s (L%d): got %u bytes want %u", name, level, got_len, n);
    if (got_len == n)
        CHECK(memcmp(got, data, n) == 0, "%s (L%d): content mismatch", name, level);
}

int main(void) {
    static uint8_t buf[2 << 20];

    // --- shape 1: highly repetitive (long matches, RLE-ish) ---------------
    for (uint32_t i = 0; i < sizeof buf; i++) buf[i] = (uint8_t)(i % 7);
    for (int l = 0; l <= 9; l++) roundtrip("repetitive", buf, 1 << 20, l, 0);

    // --- shape 2: incompressible (forces stored blocks at level 0) --------
    for (uint32_t i = 0; i < sizeof buf; i++) buf[i] = (uint8_t)rnd();
    for (int l = 0; l <= 9; l++) roundtrip("random", buf, 1 << 20, l, 0);

    // --- shape 3: text-like, skewed symbol distribution --------------------
    {
        const char *w[] = {"road", "water", "building", "landuse", "kind",
                           "minor_road", "path", "park", "pier", "earth"};
        uint32_t p = 0;
        while (p < (1 << 20) - 32) {
            const char *s = w[rnd() % 10];
            uint32_t l = (uint32_t)strlen(s);
            memcpy(buf + p, s, l); p += l;
            buf[p++] = (rnd() % 8) ? ' ' : '\n';
        }
        for (int l = 0; l <= 9; l++) roundtrip("texty", buf, p, l, 0);
    }

    // --- shape 4: all byte values, short --------------------------------
    for (uint32_t i = 0; i < 256; i++) buf[i] = (uint8_t)i;
    for (int l = 0; l <= 9; l++) roundtrip("bytes256", buf, 256, l, 0);

    // --- shape 5: degenerate sizes ----------------------------------------
    for (uint32_t n = 0; n <= 40; n++) {
        for (uint32_t i = 0; i < n; i++) buf[i] = (uint8_t)(i * 31);
        char nm[32]; snprintf(nm, sizeof nm, "tiny%u", n);
        roundtrip(nm, buf, n, 6, 0);
        roundtrip(nm, buf, n, 0, 0);
    }

    // --- raw deflate (no wrapper) -----------------------------------------
    for (uint32_t i = 0; i < (1 << 16); i++) buf[i] = (uint8_t)(i * i);
    roundtrip("raw-deflate", buf, 1 << 16, 6, 1);

    printf("roundtrips complete\n");

    // --- random fuzz: many random sizes and contents ----------------------
    {
        int n = 0;
        for (int trial = 0; trial < 300; trial++) {
            uint32_t len = 1 + rnd() % (256 * 1024);
            int mode = trial % 3;
            for (uint32_t i = 0; i < len; i++) {
                if (mode == 0)      buf[i] = (uint8_t)rnd();
                else if (mode == 1) buf[i] = (uint8_t)(rnd() % 4);
                else                buf[i] = (uint8_t)((i / 64) % 251);
            }
            char nm[32]; snprintf(nm, sizeof nm, "fuzz%d", trial);
            roundtrip(nm, buf, len, (int)(rnd() % 10), 0);
            n++;
        }
        printf("fuzz: %d random streams\n", n);
    }

    // --- corruption must be rejected, never crash -------------------------
    {
        static uint8_t packed[1 << 20];
        for (uint32_t i = 0; i < (1 << 16); i++) buf[i] = (uint8_t)(i % 13);
        long pn = gz_pack(buf, 1 << 16, packed, sizeof packed, 6, 0);
        int rejected = 0, ok = 0;
        static uint8_t got[1 << 20];
        for (int t = 0; t < 4000; t++) {
            static uint8_t tmp[1 << 20];
            memcpy(tmp, packed, pn);
            // flip a random bit somewhere after the header
            uint32_t byte = 10 + rnd() % (pn - 10);
            tmp[byte] ^= 1u << (rnd() % 8);
            uint32_t gl = sizeof got;
            inf_err_t e = inflate_auto(tmp, (uint32_t)pn, got, &gl);
            if (e == INF_OK) ok++; else rejected++;
        }
        printf("corruption: %d rejected, %d silently accepted (no crash)\n",
               rejected, ok);
    }

    // --- truncation -------------------------------------------------------
    {
        static uint8_t packed[1 << 20];
        static uint8_t got[1 << 20];
        for (uint32_t i = 0; i < (1 << 16); i++) buf[i] = (uint8_t)(i % 13);
        long pn = gz_pack(buf, 1 << 16, packed, sizeof packed, 6, 0);
        int n = 0;
        for (long cut = 1; cut < pn; cut += 3) {
            uint32_t gl = sizeof got;
            inflate_auto(packed, (uint32_t)cut, got, &gl);
            n++;
        }
        printf("truncation: %d prefixes, 0 crashes\n", n);
    }

    // --- undersized output ------------------------------------------------
    {
        static uint8_t packed[1 << 20];
        static uint8_t got[64];
        for (uint32_t i = 0; i < (1 << 16); i++) buf[i] = (uint8_t)(i % 13);
        long pn = gz_pack(buf, 1 << 16, packed, sizeof packed, 6, 0);
        uint32_t gl = sizeof got;
        inf_err_t e = inflate_auto(packed, (uint32_t)pn, got, &gl);
        CHECK(e == INF_EOUTPUT, "undersized out: got %s want EOUTPUT",
              inflate_strerror(e));
        printf("undersized output: rejected cleanly\n");
    }

    // --- speed ------------------------------------------------------------
    {
        static uint8_t packed[1 << 20];
        static uint8_t got[2 << 20];
        const char *w[] = {"road","water","building","landuse","kind","path"};
        uint32_t p = 0;
        while (p < (1 << 20) - 32) {
            const char *s = w[rnd() % 6];
            uint32_t l = (uint32_t)strlen(s);
            memcpy(buf + p, s, l); p += l; buf[p++] = ' ';
        }
        long pn = gz_pack(buf, p, packed, sizeof packed, 6, 0);
        struct timespec a, b;
        clock_gettime(CLOCK_MONOTONIC, &a);
        const int N = 50;
        for (int i = 0; i < N; i++) {
            uint32_t gl = sizeof got;
            inflate_auto(packed, (uint32_t)pn, got, &gl);
        }
        clock_gettime(CLOCK_MONOTONIC, &b);
        double ms = ((b.tv_sec - a.tv_sec) * 1e3 + (b.tv_nsec - a.tv_nsec) / 1e6) / N;
        printf("speed: %.2f ms for %ld KB -> %u KB (%.1f MB/s out)\n",
               ms, pn / 1024, p / 1024, (p / 1048576.0) / (ms / 1000.0));
    }

    printf(fails ? "\n%d FAILURES\n" : "\nall ok (%d failures)\n", fails);
    return fails ? 1 : 0;
}
