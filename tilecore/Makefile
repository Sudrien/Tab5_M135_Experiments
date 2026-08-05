# Desktop test build. The same .c files compile unchanged for ESP32-P4;
# only the pmt_read_fn / pmt_inflate_fn backends differ.
CFLAGS = -O2 -Wall -Wextra -std=c11

all: pmt_test grid_test mvt_test inflate_test

pmt_test: pmt_test.c pmtiles.c pmtiles.h mercator.h
	$(CC) $(CFLAGS) pmt_test.c pmtiles.c -lz -lm -o $@

grid_test: grid_test.c tile_grid.c tile_grid.h
	$(CC) $(CFLAGS) grid_test.c tile_grid.c -lm -o $@

mvt_test: mvt_test.c mvt.c mvt.h
	$(CC) $(CFLAGS) mvt_test.c mvt.c -lm -o $@

fixtures:
	python3 make_test_pmtiles.py

# Pull a real tile out of an extract:
#   make probe ARCHIVE=local.pmtiles LAT=42.3601 LON=-71.0589 Z=14
probe: pmt_test
	./pmt_test $(ARCHIVE) --probe $(LAT) $(LON) $(Z) real.mvt

test: all fixtures
	./pmt_test flat.pmtiles expect.txt
	./pmt_test leaf.pmtiles expect.txt
	./grid_test
	./inflate_test
	@if [ -f boston.mvt ]; then ./mvt_test boston.mvt; \
	 else echo "(skipping mvt_test: no boston.mvt)"; fi

clean:
	rm -f pmt_test grid_test mvt_test *.pmtiles expect.txt

inflate_test: inflate_test.c inflate.c inflate.h
	$(CC) $(CFLAGS) inflate_test.c inflate.c -lz -o $@
