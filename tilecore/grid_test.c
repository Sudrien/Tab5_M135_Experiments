// grid_test.c - invariant checks for the tile grid.
//
//   cc -O2 -Wall -Wextra grid_test.c tile_grid.c -lm -o grid_test

#include "tile_grid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { \
    fprintf(stderr, "  FAIL: "); fprintf(stderr, __VA_ARGS__); \
    fprintf(stderr, "\n"); fails++; } } while (0)

static uint16_t *bufs[GRID_COUNT];
static uint16_t *original[GRID_COUNT];

static void alloc_bufs(void) {
    for (int i = 0; i < GRID_COUNT; i++) {
        // 4-byte stubs: we only ever compare pointer identity here.
        bufs[i] = malloc(4);
        original[i] = bufs[i];
    }
}

// Invariant 1: the multiset of pixel pointers is exactly the original set.
static void check_buffers(tile_grid_t *g, const char *where) {
    int seen[GRID_COUNT];
    memset(seen, 0, sizeof seen);
    for (int i = 0; i < GRID_COUNT; i++) {
        int found = -1;
        for (int j = 0; j < GRID_COUNT; j++)
            if (g->slots[i].pixels == original[j]) { found = j; break; }
        CHECK(found >= 0, "%s: slot %d has a foreign pixel pointer", where, i);
        if (found >= 0) {
            CHECK(!seen[found], "%s: buffer %d used by two slots", where, found);
            seen[found] = 1;
        }
    }
}

// Invariant 2: every slot id matches its position relative to centre.
static void check_ids(tile_grid_t *g, const char *where) {
    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N; c++) {
            int i = r * GRID_N + c;
            tile_id_t want = grid_tile_at(g->center, r - GRID_MID, c - GRID_MID);
            CHECK(g->slots[i].id.x == want.x && g->slots[i].id.y == want.y &&
                  g->slots[i].id.z == want.z,
                  "%s: slot %d id %u/%d/%d want %u/%d/%d", where, i,
                  g->slots[i].id.z, g->slots[i].id.x, g->slots[i].id.y,
                  want.z, want.x, want.y);
        }
}

int main(void) {
    alloc_bufs();
    tile_grid_t g;
    tile_id_t c0 = { .z = 14, .x = 8000, .y = 5000 };

    grid_init(&g, bufs, c0);
    check_buffers(&g, "init");
    check_ids(&g, "init");
    printf("init: centre %u/%d/%d, %d slots\n", g.center.z, g.center.x, g.center.y, GRID_COUNT);

    // ---- axis shifts: expect 3 new tiles ----------------------------------
    struct { int dx, dy; const char *name; int want_jobs; } axis[] = {
        { +1,  0, "east",  GRID_N },
        { -1,  0, "west",  GRID_N },
        {  0, +1, "south", GRID_N },
        {  0, -1, "north", GRID_N },
    };
    for (size_t k = 0; k < sizeof axis / sizeof axis[0]; k++) {
        grid_init(&g, bufs, c0);
        render_job_t jobs[GRID_COUNT];
        int n = grid_shift(&g, axis[k].dx, axis[k].dy, jobs, GRID_COUNT);
        CHECK(n == axis[k].want_jobs, "%s shift: %d jobs, want %d",
              axis[k].name, n, axis[k].want_jobs);
        // A +/-1 shift always promotes an existing ring-1 slot to centre, so
        // the centre is already rendered and must NOT appear as a job.
        for (int j = 0; j < n; j++)
            CHECK(jobs[j].slot != GRID_COUNT / 2,
                  "%s: centre queued for render but should be reused", axis[k].name);
        check_buffers(&g, axis[k].name);
        check_ids(&g, axis[k].name);
        printf("%-6s shift: %d jobs, centre %d/%d\n",
               axis[k].name, n, g.center.x, g.center.y);
    }

    // ---- diagonal shifts: expect 5 new tiles -------------------------------
    struct { int dx, dy; const char *name; } diag[] = {
        { +1, +1, "SE" }, { -1, +1, "SW" }, { +1, -1, "NE" }, { -1, -1, "NW" },
    };
    for (size_t k = 0; k < sizeof diag / sizeof diag[0]; k++) {
        grid_init(&g, bufs, c0);
        render_job_t jobs[GRID_COUNT];
        int n = grid_shift(&g, diag[k].dx, diag[k].dy, jobs, GRID_COUNT);
        CHECK(n == 5, "%s shift: %d jobs, want 5", diag[k].name, n);
        for (int j = 0; j < n; j++)
            CHECK(jobs[j].slot != GRID_COUNT / 2,
                  "%s: centre queued for render but should be reused", diag[k].name);
        check_buffers(&g, diag[k].name);
        check_ids(&g, diag[k].name);
        printf("%-6s shift: %d jobs, centre %d/%d\n",
               diag[k].name, n, g.center.x, g.center.y);
    }

    // ---- reuse actually preserves READY state ------------------------------
    grid_init(&g, bufs, c0);
    for (int i = 0; i < GRID_COUNT; i++) g.slots[i].state = TILE_READY;
    {
        render_job_t jobs[GRID_COUNT];
        grid_shift(&g, +1, 0, jobs, GRID_COUNT);
        int ready = 0, pending = 0;
        for (int i = 0; i < GRID_COUNT; i++) {
            if (g.slots[i].state == TILE_READY) ready++;
            if (g.slots[i].state == TILE_PENDING) pending++;
        }
        CHECK(ready == 6, "east shift kept %d READY, want 6", ready);
        CHECK(pending == 3, "east shift made %d PENDING, want 3", pending);
        printf("state carry: %d ready, %d pending after east shift\n", ready, pending);
    }

    // ---- stale commits are rejected ---------------------------------------
    grid_init(&g, bufs, c0);
    {
        render_job_t jobs[GRID_COUNT];
        render_job_t stale = { .id = g.slots[0].id, .slot = 0,
                               .generation = g.generation };
        grid_shift(&g, +1, +1, jobs, GRID_COUNT);   // bumps generation
        CHECK(grid_commit(&g, &stale, TILE_READY) == 0,
              "stale job was accepted after shift");
        CHECK(grid_commit(&g, &jobs[0], TILE_READY) == 1,
              "fresh job was rejected");
        printf("staleness: old generation rejected, current accepted\n");
    }

    // ---- polar clamping ----------------------------------------------------
    grid_init(&g, bufs, (tile_id_t){ .z = 2, .x = 1, .y = 0 });
    {
        int nodata = 0;
        for (int i = 0; i < GRID_COUNT; i++)
            if (g.slots[i].state == TILE_NODATA) nodata++;
        CHECK(nodata == GRID_N, "north edge: %d NODATA slots, want %d", nodata, GRID_N);
        printf("polar edge: %d slots marked NODATA at y=0\n", nodata);
    }

    // ---- antimeridian wrap -------------------------------------------------
    grid_init(&g, bufs, (tile_id_t){ .z = 4, .x = 15, .y = 8 });   // 2^4-1
    {
        tile_id_t east = grid_tile_at(g.center, 0, +1);
        CHECK(east.x == 0, "antimeridian: east of x=15 at z4 is %d, want 0", east.x);
        printf("antimeridian: x=15 -> east neighbour x=%d\n", east.x);
    }

    // ---- zoom change: everything invalidates, centre goes first -----------
    grid_init(&g, bufs, c0);
    for (int i = 0; i < GRID_COUNT; i++) g.slots[i].state = TILE_READY;
    {
        render_job_t jobs[GRID_COUNT];
        uint16_t *before[GRID_COUNT];
        for (int i = 0; i < GRID_COUNT; i++) before[i] = g.slots[i].pixels;

        tile_id_t zc = { .z = 13, .x = 4000, .y = 2500 };
        int n = grid_set_zoom(&g, zc, jobs, GRID_COUNT);
        CHECK(n == GRID_COUNT, "zoom change: %d jobs, want %d", n, GRID_COUNT);
        CHECK(jobs[0].slot == GRID_COUNT / 2, "zoom change: centre not first job");
        // pixels must survive so the stale frame stays displayable
        for (int i = 0; i < GRID_COUNT; i++)
            CHECK(g.slots[i].pixels == before[i],
                  "zoom change: slot %d pixels were dropped", i);
        check_buffers(&g, "zoom");
        check_ids(&g, "zoom");
        printf("zoom change: %d jobs, centre first, pixels retained\n", n);
    }

    // ---- random walk: 200k shifts, invariants hold throughout --------------
    grid_init(&g, bufs, c0);
    srand(12345);
    long total_jobs = 0;
    for (int step = 0; step < 200000; step++) {
        int dx = (rand() % 3) - 1;
        int dy = (rand() % 3) - 1;
        if (!dx && !dy) continue;
        // stay away from the poles so NODATA does not skew the job counts
        if (g.center.y + dy < 1 || g.center.y + dy > (1 << g.center.z) - 2) dy = 0;
        render_job_t jobs[GRID_COUNT];
        total_jobs += grid_shift(&g, dx, dy, jobs, GRID_COUNT);
        if ((step % 5000) == 0) {
            check_buffers(&g, "walk");
            check_ids(&g, "walk");
        }
    }
    check_buffers(&g, "walk-end");
    check_ids(&g, "walk-end");
    printf("random walk: 200k steps, %ld render jobs, centre %d/%d\n",
           total_jobs, g.center.x, g.center.y);

    printf(fails ? "\n%d FAILURES\n" : "\nall ok (%d failures)\n", fails);
    return fails ? 1 : 0;
}
