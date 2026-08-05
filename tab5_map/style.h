// style.h - shared cartographic style table and draw order.
#ifndef STYLE_H
#define STYLE_H
#include "raster.h"
#ifdef __cplusplus
extern "C" {
#endif
enum {
    S_NONE = 0, S_EARTH, S_WATER, S_PARK, S_GRASS, S_URBAN, S_PIER,
    S_BUILDING, S_MAJOR, S_MINOR, S_PATH, S_RAIL, S_FERRY,
    S_POI, S_PLACE, S_COUNT
};
extern rs_style_t STYLES[S_COUNT];
extern const char *DRAW_ORDER[];
extern const int   N_DRAW_ORDER;
void    style_init(int px);
uint8_t style_lookup(void *ctx, const mvt_layer_t *l, const char *s, uint32_t n);
#ifdef __cplusplus
}
#endif
#endif
