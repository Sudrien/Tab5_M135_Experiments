// bench_types.h
//
// Types used by the .ino live here rather than in the sketch itself.
// The Arduino preprocessor auto-generates function prototypes and inserts
// them near the top of the .ino, above any struct defined there - so a
// function taking `Stat&` gets declared before `Stat` exists. Anything
// pulled in by #include is already visible at that insertion point.

#ifndef BENCH_TYPES_H
#define BENCH_TYPES_H

#include <stdint.h>

struct Stat {
    uint64_t n  = 0;
    uint64_t sum = 0;
    uint64_t mn = UINT64_MAX;
    uint64_t mx = 0;
};

struct Counters {
    uint32_t parts  = 0;
    uint32_t points = 0;
};

#endif // BENCH_TYPES_H
