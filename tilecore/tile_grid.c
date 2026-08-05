// tile_grid.c

#include "tile_grid.h"
#include <string.h>

// ---- tile identity ---------------------------------------------------------
static int32_t wrap_x(int64_t x, uint8_t z) {
    int64_t n = (int64_t)1 << z;
    x %= n;
    if (x < 0) x += n;
    return (int32_t)x;
}

tile_id_t grid_tile_at(tile_id_t c, int drow, int dcol) {
    tile_id_t t;
    t.z = c.z;
    t.x = wrap_x((int64_t)c.x + dcol, c.z);
    t.y = c.y + drow;              // deliberately unwrapped; poles are edges
    return t;
}

int grid_id_valid(tile_id_t id) {
    int64_t n = (int64_t)1 << id.z;
    return id.y >= 0 && id.y < n;
}

static int same_tile(tile_id_t a, tile_id_t b) {
    return a.z == b.z && a.x == b.x && a.y == b.y;
}

// ---- job emission ----------------------------------------------------------
// Centre first: it is the only slot guaranteed to be on screen, so it should
// reach the worker ahead of the ring. The rest follow in slot order.
static int emit_jobs(tile_grid_t *g, const int *needs,
                     render_job_t *jobs, int max_jobs)
{
    int n = 0;
    const int mid = GRID_COUNT / 2;

    if (needs[mid] && n < max_jobs) {
        jobs[n].id = g->slots[mid].id;
        jobs[n].slot = (uint8_t)mid;
        jobs[n].generation = g->generation;
        n++;
    }
    for (int i = 0; i < GRID_COUNT && n < max_jobs; i++) {
        if (i == mid || !needs[i]) continue;
        jobs[n].id = g->slots[i].id;
        jobs[n].slot = (uint8_t)i;
        jobs[n].generation = g->generation;
        n++;
    }
    return n;
}

// ---- init ------------------------------------------------------------------
void grid_init(tile_grid_t *g, uint16_t *const *bufs, tile_id_t center) {
    memset(g, 0, sizeof *g);
    g->center = center;
    g->generation = 1;
    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N; c++) {
            int i = r * GRID_N + c;
            g->slots[i].id = grid_tile_at(center, r - GRID_MID, c - GRID_MID);
            g->slots[i].pixels = bufs[i];
            g->slots[i].generation = g->generation;
            g->slots[i].state = grid_id_valid(g->slots[i].id)
                                ? TILE_PENDING : TILE_NODATA;
        }
    g->initialised = 1;
}

// ---- shift -----------------------------------------------------------------
int grid_shift(tile_grid_t *g, int dx, int dy,
               render_job_t *jobs, int max_jobs)
{
    if (dx == 0 && dy == 0) return 0;

    subtile_t old[GRID_COUNT];
    memcpy(old, g->slots, sizeof old);

    // Which old slots survive into the new layout.
    int reused[GRID_COUNT];
    memset(reused, 0, sizeof reused);

    g->generation++;
    g->center = grid_tile_at(g->center, dy, dx);

    int needs[GRID_COUNT];
    memset(needs, 0, sizeof needs);

    // New slot (r,c) shows the tile that old slot (r+dy, c+dx) showed, when
    // that source is still inside the window. Moving east (dx=+1) means the
    // new west column comes from the old middle column, and so on.
    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {
            int dst = r * GRID_N + c;
            int sr = r + dy, sc = c + dx;

            g->slots[dst].id = grid_tile_at(g->center, r - GRID_MID, c - GRID_MID);

            if (sr >= 0 && sr < GRID_N && sc >= 0 && sc < GRID_N) {
                int src = sr * GRID_N + sc;
                // Carry pixels and state across, but only if the tile really
                // is the same one - guards against an off-by-one in the map.
                if (same_tile(old[src].id, g->slots[dst].id)) {
                    g->slots[dst].pixels     = old[src].pixels;
                    g->slots[dst].state      = old[src].state;
                    g->slots[dst].generation = old[src].generation;
                    reused[src] = 1;
                    continue;
                }
            }
            needs[dst] = 1;
        }
    }

    // Hand the freed buffers to the slots that need one. Every non-reused old
    // slot donates exactly one buffer, and the count always matches because
    // the mapping above is a bijection on slot indices.
    int donor = 0;
    for (int dst = 0; dst < GRID_COUNT; dst++) {
        if (!needs[dst]) continue;
        while (donor < GRID_COUNT && reused[donor]) donor++;
        g->slots[dst].pixels     = old[donor].pixels;
        g->slots[dst].generation = g->generation;
        g->slots[dst].state      = grid_id_valid(g->slots[dst].id)
                                   ? TILE_PENDING : TILE_NODATA;
        donor++;
    }

    // Off-world slots need no render job.
    for (int i = 0; i < GRID_COUNT; i++)
        if (needs[i] && g->slots[i].state == TILE_NODATA) needs[i] = 0;

    return emit_jobs(g, needs, jobs, max_jobs);
}

// ---- zoom ------------------------------------------------------------------
int grid_set_zoom(tile_grid_t *g, tile_id_t new_center,
                  render_job_t *jobs, int max_jobs)
{
    g->generation++;
    g->center = new_center;

    int needs[GRID_COUNT];
    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N; c++) {
            int i = r * GRID_N + c;
            g->slots[i].id = grid_tile_at(new_center, r - GRID_MID, c - GRID_MID);
            g->slots[i].generation = g->generation;
            if (grid_id_valid(g->slots[i].id)) {
                g->slots[i].state = TILE_PENDING;
                needs[i] = 1;
            } else {
                g->slots[i].state = TILE_NODATA;
                needs[i] = 0;
            }
            // pixels deliberately untouched: the stale image stays displayable
            // until the replacement commits, which is what keeps zoom changes
            // from flashing black.
        }
    return emit_jobs(g, needs, jobs, max_jobs);
}

// ---- drift trigger ---------------------------------------------------------
void grid_drift(tile_grid_t *g, double frac_x, double frac_y,
                int *dx, int *dy)
{
    // frac_* are fractional tile coords at the grid's zoom. The centre tile
    // spans [center.x, center.x+1) x [center.y, center.y+1).
    double rx = frac_x - (double)g->center.x;
    double ry = frac_y - (double)g->center.y;

    *dx = (rx < 0.0) ? -1 : (rx >= 1.0) ? 1 : 0;
    *dy = (ry < 0.0) ? -1 : (ry >= 1.0) ? 1 : 0;

    // A single fix can jump more than one tile (tunnel exit, cold-start
    // relocate). Clamp to +/-1 here; the caller loops or forces a re-centre.
}

// ---- worker commit ---------------------------------------------------------
int grid_commit(tile_grid_t *g, const render_job_t *job, tile_state_t result) {
    if (job->slot >= GRID_COUNT) return 0;
    subtile_t *s = &g->slots[job->slot];
    if (job->generation != g->generation) return 0;   // grid moved on
    if (!same_tile(s->id, job->id))       return 0;   // slot reassigned
    s->state = result;
    return 1;
}
