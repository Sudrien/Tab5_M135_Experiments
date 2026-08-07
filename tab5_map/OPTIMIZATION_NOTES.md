# tab5_map — optimisation pass

Everything below was measured, not guessed. The rasteriser compiles standalone,
so I built it on the host with synthetic geometry shaped like a dense z14 urban
tile (12 area polygons, 2500 buildings, 900 roads at 1280 px) and worked from
the profile.

**Net: 38.5 ms → 26.5 ms per tile on the host, −31%, with pixel-identical output.**

Output equivalence is verified by a randomised differential test: 15,000 cases
across three `SUBTILE_SPLIT` values, comparing a full-buffer checksum between
the old and new rasteriser. All identical.

| stage | before | after | change |
|---|---|---|---|
| rs_clear | 0.48 ms | 0.48 ms | — |
| area polygons | 5.00 ms | 4.68 ms | −6.4% |
| buildings | 2.45 ms | 2.46 ms | — |
| roads | 30.56 ms | 18.91 ms | **−38.1%** |
| **total** | **38.49 ms** | **26.54 ms** | **−31.0%** |

---

## 1. `fill_quad` was doing bbox-sized work for span-sized output

This is the big one, and it's the same bug you'd already found and fixed in the
generic filler — the comment there says it outright: *"a road segment spanning
3 rows was memsetting 3 KB to touch 12 pixels."* The quad fast path was
reintroducing it in a place where the shape looked small enough not to matter.

Per row, `fill_quad` was clearing and then scanning `px0..px1`, the bounding box
of the **whole quad**. For a road segment running diagonally, that box is L×L
while the segment covers only L×width pixels. A 45° segment 80 px long cleared
and scanned ~6400 coverage entries per shape to write ~400. Cost scaled O(L²)
against O(L·w) of real work.

Fixed by computing the crossings for every subsample first, then clearing and
compositing only the pixel range that row actually touches. Roads are 78% of
render time, and this took 38% off them.

## 2. Coverage buffer is now cleared by range, not by bounding box

Same reasoning generalised. Both fillers now keep `cov` zeroed as an invariant:
`span_coverage` records the range it dirtied (`cov_lo`/`cov_hi`), the composite
loop walks only that, and the row hands the buffer back zeroed. The top row of
a circle no longer clears a span the full width of the circle to touch six
pixels.

`rs_clear` establishes the invariant once per tile, and `alloc_all` zeroes
`w_cov` at startup so it doesn't begin life as uninitialised heap.

## 3. `qsort` → hybrid insertion sort for the edge list

`sort_crossings` already had this reasoning applied to it; the edge list has the
same profile and didn't. A building is a four-edge quad, and dispatching through
a function pointer per comparison costs more than the comparison. Insertion sort
below 48 edges, `qsort` above so a coastline doesn't hit the quadratic term.

Worth ~12% on the building pass in isolation. It'll be worth more on the P4 than
on x86 — indirect calls are relatively more expensive there.

## 4. Hot scratch moved from PSRAM to internal SRAM

`w_cov`, `w_xs`, `w_dirs`, `w_active`, `w_pts` and `w_val` were all in PSRAM.
The coverage row is read and written several times per output pixel and the
crossing tables once per edge per subsample — scattered, non-streaming access,
which is exactly what PSRAM is worst at.

Together they're ~45 KB against roughly 700 KB of internal SRAM, so it's a cheap
trade. `alloc_fast()` tries internal and falls back to PSRAM, so a tight build
still works, just slower. `w_tile`, `w_mvt` and `w_edges` are too large and are
streamed rather than random-accessed, so they stay put.

**The host benchmark cannot show this at all.** I'd expect it to be one of the
larger on-device wins, but you'll need `last_render_ms` on real hardware to
size it.

### Correction — this version starved TLS, and is fixed

The first cut of this took all ~47.5 KB unconditionally, and that broke
networking on device:

```
E (33433) esp-aes: Failed to allocate memory for len descriptor
netsource: remote 6/31/11 failed: read failed
```

Internal SRAM is not spare memory — it is the *only* memory mbedTLS and the AES
DMA engine can use. `http_range` sets `setReuse(false)`, so every tile fetch runs
a fresh TLS handshake, which is the peak internal-heap moment in the program.
The log shows it exactly: early fetches succeed, then once the render buffers are
live the handshake can't get a DMA descriptor and every subsequent remote read
fails. Free internal heap had gone from ~172 KB to ~124 KB. The truncated fetch
(`asked 131665, got 44460 in 7528 reads`) is the same root cause downstream — the
TLS stream stalling, not a slow server.

I had also put the wrong buffers there. `w_pts` is 16 KB and I had already
measured its access pattern as noise. The benefit is concentrated almost entirely
in `w_cov`, which is touched several times per output pixel.

Now: **3.5 KB internal (`w_cov` + `w_val`), 44 KB returned to the heap**, behind
two guards — a byte budget (`MAP_SCRATCH_INTERNAL_KB`, default 4) and a hard
floor on remaining internal heap (`MAP_INTERNAL_RESERVE`, default 110 KB).
Buffers are requested hottest-first so the budget is spent where it pays, and
`alloc_all` now prints what it took plus the largest free DMA block.

This was my error, and the general lesson is worth stating: internal SRAM on this
part is a contended resource with a hard consumer, so "there's plenty free"
is not a sufficient reason to take it.

## 5. Per-tile debug fingerprints made opt-in

`render_tile` hashed the entire compressed payload twice on every render — two
scattered passes over up to 192 KB of PSRAM per tile — to detect a clobber
between fetch and inflate. Now behind `MAP_CHECK_TILE_BUFFER`, so it's one
`-D` away when you need it again.

---

## Round 3 — network requests (from the second device log)

Heap is fixed: `227 KB internal heap free (largest DMA block 95 KB)`, no AES
errors, fetches flowing. With that unblocked, the log shows where the time
actually goes — and it is not the rasteriser.

```
netsource: 6/31/37 at offset 14969040 len 93
netsource: 6/31/38 at offset 14969040 len 93
netsource: 6/31/39 at offset 14969040 len 93   ... 40, 41, 42, 43, 44, 45, 46
```

Every one of those tiles resolves to **the same offset and the same length**.
That is PMTiles deduplication working as designed — identical tiles are stored
once and every tile that matches points at the same blob. Whole oceans at low
zoom are one 93-byte payload.

The device was fetching it again for every single tile, each with its own TLS
handshake. That is essentially the entire cost of the world floor walk: at the
observed 0.2-0.3 tiles/s, ~155 min remaining, nearly all of it re-downloading
93 bytes it already had.

**Blob memo.** Two tiles resolving to the same `(offset, length)` are the same
bytes by construction, so the second never needs fetching. Holding just the last
blob is enough, because the walk is raster order and identical tiles arrive in
long runs. Bounded at 4 KB — this exists for small shared payloads, and a real
tile misses the memo and takes the normal path. Cleared whenever the archive is
reopened, since offsets are only meaningful within one build.

Modelling the walk with ~9% land gives **82% of data fetches removed, ~364 min to
~66 min**. That is a model, not a measurement — the real figure depends on your
archive's dedup ratio, and the new `(memo)` tag in the offset log line will show
you the true hit rate.

**The directory was being walked twice per tile.** `netsource_get_locked` called
`pmt_find` for a debug print, then `pmt_get` — which calls `pmt_find` again
internally. The root directory is cached so this was free for the world floor,
but a working z14 tile resolves through a **leaf** directory, and leaves are not
cached. So each working tile was: fetch leaf, fetch leaf again, fetch tile —
three HTTPS requests with three handshakes, where two would do.

Added `pmt_read_blob()` to read a payload already located by `pmt_find`, and
`pmt_get` is now written in terms of it, so its behaviour is unchanged for any
other caller. The ENOMEM leaf-growth retry moved onto the lookup, which is where
leaf directories are actually loaded.

**Worth doing next: cache the last leaf directory.** `load_dir` already caches
the root; neighbouring tiles share a leaf, so a single-entry leaf cache on the
same pattern would take working tiles from two requests to roughly one. It needs
a buffer allocated alongside the existing root cache, which is why I have not
done it blind.

---

## On the 2011 ms render time

Both logs show `last 2011 ms` / `last 2016 ms` per tile, against 26 ms for a
comparable synthetic tile on the host — about 77x. Some of that is simply the
part (400 MHz RISC-V against a ~4 GHz desktop core), but not all of it.

The likely remainder is that the 3.3 MB pixel buffer lives in PSRAM, and every
`blend565` is a read-modify-write against it. The rasteriser work above cut the
*number* of those operations by ~31%; it did nothing about their cost.

I have no baseline render figure from before the rasteriser changes, so I cannot
tell you what they bought on device. If that matters, flashing the original
`raster.c` and comparing `last N ms` would settle it in one boot.

---

## Round 4 — the permanent stall at 65%

The memo worked: `(memo)` hits are landing, the rate went from 0.2 to 0.7-2.0
tiles/s, and the world floor estimate dropped from ~155 min to ~16 min. Then
this happens at 135 s and never recovers:

```
netsource: range 136840594756+131665 -> got 43055 in 7527 reads, 0 left over
netsource: remote 6/34/55 failed: read failed
netsource: remote 6/34/56 failed: read failed        ... 57, 58, 59, 60, 61, 62
```

`net` stays at 45 and `cache` at 43/2788 for the next 195 seconds. Every tile
burns the full 15 s deadline and fails.

**Retraction: the heap is not leaking.** I flagged a leak from three samples
(124 → 115 → 106 KB). The longer log shows it recovering to ~149 KB and holding
flat for four minutes. That was in-flight connection state, and I called it too
early on too little data.

### What is actually wrong

The failing read is the same offset and the same length in two separate boots,
for different tiles, each time hitting eight consecutive tiles. That is a
**leaf directory**, 129 KB, shared by a contiguous run of tiles — and two
things are wrong with how it is handled.

**It cannot finish.** The deadline was a flat 15 s regardless of size. At the
observed 2.8 KB/s the 129 KB read reaches 33% and is cut off, every time.
The deadline now scales with the request (`10 s + len/4`), and reads above
32 KB are split into chunks — because requests of the size that already work
(93 B, 342 B, 3 KB, 3.8 KB all succeed promptly) are the reliable shape,
whatever is going on upstream with big ranges.

**It should not be fetched per tile.** There was no leaf cache, so all eight
tiles re-requested the same 129 KB. `load_dir` cached the root but nothing else.

The fix costs no memory. `dir_buf` has to hold a decompressed directory anyway,
and once the root is being served from `root_cache` it stops being touched — so
what survives in `dir_buf` is precisely the last leaf. Recording which range it
holds (`dir_off`, `dir_srclen`, `dir_len`) turns a repeat lookup into a pointer
return. `fit_buffers` clears the identity when it reallocates, or a stale hit
would hand back freed memory.

At 64 tiles per leaf, that is 232 MB of leaf traffic down to 3.6 MB for the
tiles remaining in your log — about 64x. Model, not measurement; the real
figure depends on the archive's leaf fan-out.

---

## Round 5 — the blinking status bar and buttons

Both the UI strips and the map compositor painted straight to the panel in two
visible steps:

```
fillRect(...)      region goes flat colour
drawString(...)    text appears
```

There is no framebuffer behind the display, so those are two separate states on
the glass and the gap between them is the blink. It showed specifically on text
updates because the change-detection in `drawStatus` is working correctly and
suppresses the redraw the rest of the time.

**Status bar and buttons now compose off-screen.** One 1280x52 canvas for the
status bar and one button-sized canvas reused for all three buttons, pushed in a
single write of final pixels. 173 KB of PSRAM against ~6890 KB free. If either
allocation fails the original direct path still runs, so a tight build degrades
to the old blink rather than losing its UI, and the boot line says which.

`drawClockBattery` writes into the status strip, so it now takes its target as a
parameter instead of always using `M5.Display`. The canvas origin coincides with
the panel's, so its coordinates are unchanged.

Two details that would have looked like new bugs: the buttons are rounded rects,
so the sprite corners have to carry `style_background()` rather than black -
which changes with the theme, and filling them black would have stamped dark
squares around each button in day mode. And I used the qualified
`lgfx::LovyanGFX` for the target pointer rather than assuming an unqualified
alias is exported by M5GFX.

**`blit_region` had the same shape, worse.** If any slot was absent it wiped the
*entire* region to background and painted the tiles back over it - so one
pending tile made the whole map flash on every repaint, and most pixels were
written twice. It now fills only the uncovered parts: the bands of the region
outside the grid's footprint, plus the individual slots with nothing to show.
The two grid-lock acquisitions collapsed into one at the same time.

That change is verified exhaustively rather than by inspection, because a gap
left unfilled is a worse bug than the blink: 1,050,624 cases over every
combination of slot states, view offsets and clip rectangles, comparing final
per-pixel ownership. **Zero regressions** - every pixel the old path painted,
the new one also paints.

The comparison also turned up a latent bug in the original. All 50.7M
differences run the other way, and all of them occur when *every* slot is
drawable: the `any_missing` gate meant that in that case nothing was filled at
all, so any part of the region outside the grid footprint kept whatever was
there before. That is the same marker-smear the old comment describes, fixed
only for the missing-tile case and not for the outside-the-grid case. The new
code covers both.

### Corner colour fix

The button sprite filled its corners with `style_background()` — my guess at
what the footer sits on. It was wrong on the device: the corners came out a
different colour from the surrounding strip.

Fixed by not writing those pixels at all. The sprite is filled with a colour
key (magenta, absent from a button made of greys, blues, greens and white) and
pushed with `pushSprite(x, y, KEY)`, so the corner pixels are skipped and
whatever is behind them stays. That is correct regardless of what painted the
footer or when, which the guess never could be.

If a magenta fringe ever appears around a button corner, it means the rounded
rect is being drawn antialiased — the key would then be blended into the edge
pixels. `fillRoundRect` is not antialiased (`fillSmoothRoundRect` is), so this
should not arise, but that is the symptom to look for.

### Lit screen with nothing drawn

Possibly the condition seen before this session. `map_draw` skips the repaint
when the view, the tile states and the marker are all unchanged, so anything
that clears the panel behind the engine's back has to call `map_invalidate()` —
which is exactly what the function exists for, and it was never called anywhere
in the sketch.

Most paths turn out to be covered indirectly: `screenOn` goes through
`map_set_visible(true)`, a theme switch through `map_set_dark`, and boot by the
flag starting set. One path was not. `portal_run` paints full screen, and on
return from the wifi-setup button the sketch does
`fillScreen(style_background())` and carries on — leaving the engine with
nothing it considers changed, so it never repaints. Backlight on, empty map,
until a fix moves the marker or a tile lands.

`map_invalidate()` added there. Whether this is the condition you saw depends on
whether the portal had been opened in those sessions; if it recurs without ever
touching the portal, it is something else and the heartbeat line will at least
show whether `loop()` is still running.

---

## Round 6 — dark screen on the first flash

Reported as a backlight problem, needing a second flash to light up. It is not
the backlight. The panel never initialises at all.

The evidence is in the boot banner, not the reset reason:

```
boot 1 (dark):  PSRAM 32764 KB free   ui: canvases FAILED
boot 2 (works): PSRAM 30963 KB free   ui: canvases ok
                      ---------
                       1801 KB
```

A 1280x720 framebuffer at 16bpp is 1800 KB. On the dark boot PSRAM reads back
as essentially the full 32 MB, because that framebuffer was never allocated.
`width()` and `height()` return 0, every subsequent draw silently goes nowhere,
and the board looks like it has a backlight fault when in fact nothing has been
initialised to light up.

Two things corroborate it. The canvas allocations failed because
`createSprite(width(), 52)` was asked for a zero-width sprite - a downstream
symptom, not a second bug. And every `boot:` line vanished from the dark boot
because `bootStepEx` computes `y > M5.Display.height() - 40`, which with
`height()` at 0 is `130 > -40`, returning *before* its `Serial.printf`.

**My previous explanation was wrong.** I attributed this to a stale backlight
bit in the I2C IO expander surviving a soft reset, and added a brightness
off/on cycle to force a write. The reset reason I added to test that theory
reports `unknown` on both the dark boot and the good one, so it does not
discriminate at all. The brightness cycle has been removed rather than left in
- code justified by a disproven premise is worse than no code.

### What is actually different between the two boots

The dark boot follows a full flash write: 825 KB compressed, 5.8 s of sustained
writing. The boot that works follows an upload that wrote nothing at all - every
sector matched, so esptool only verified. The reset that fails is the one
immediately after a long write, not the one after an idle bus.

### Fixes

- `bootStepEx` logs to serial **first**, before any panel-dependent early
  return. Diagnostics should not disappear along with the thing being
  diagnosed.
- `panelBegin()` checks the geometry after `M5.begin()`, retries `init()` once
  after a short delay, and failing that restarts once - guarded by an
  `RTC_NOINIT` counter so it cannot become a boot loop. If it still does not
  come up, the sketch continues headless rather than pretending.
- A 150 ms settle before `M5.begin()`, since the failing case is the boot that
  follows sustained flash activity.
- The banner now prints `panel WxH ok|DOWN`, which is the signal that actually
  discriminates.

The retry and the delay are reasoned from the evidence but untested by me - I
can confirm the diagnosis from the logs, not the cure. If a dark boot still
appears, the banner will now say `panel 0x0 DOWN` and the `boot:` trace will
survive, which is enough to tell whether the retry ran and failed or never
triggered.

---

## Round 7 — pooled TLS connection

Every fetch opened a fresh HTTPS connection. The handshake dominated the cost of
a small tile and was the peak internal-heap moment in the program. The socket is
now kept between requests.

### Handling the reason it was off

The original comment was right, and the hazard is a corruption bug rather than a
connection bug: a response whose body is not fully drained leaves those bytes in
the socket, and the next request reads them as its own reply. That surfaces as a
tile which fetches "successfully" and then fails to inflate — traced back here
only with difficulty.

So the rule is strict. The connection survives **only** a request that completed
exactly as expected. Wrong status, length mismatch, short read, or a single
trailing byte all tear it down. Reconnecting costs a handshake; guessing costs
correctness.

There is also a hazard that did not exist before: a pooled socket the server has
closed with no warning. That failure is indistinguishable from a real error at
the call site, so a pooled request that fails is retried once on a fresh
connection. Without that, every server-side keep-alive timeout would surface as
a lost tile.

The state machine is verified rather than reasoned about — every ordered pair of
outcomes from both starting states, against two invariants: a failed call never
leaves a socket pooled, and a successful call always does. Both hold. A run of
100 clean requests uses 1 handshake and 99 reuses; a mid-run server close costs
one reconnect and no lost request.

### The gap

The handshake used to space requests out as a side effect. Removing it without
replacing it would turn a polite client into a hostile one against a bucket we
are asked not to hotlink, so `NET_REQUEST_GAP_MS` (default 150) enforces the
spacing deliberately, measured from the end of the previous request.
`NET_KEEPALIVE_IDLE_MS` (default 10 s) retires an idle socket rather than
discovering it closed.

The gap now sets the ceiling — about 6.7 requests/s — rather than the protocol,
which is the intent. Against the observed 1.6 tiles/s, expect somewhere around
1.9–3.0 tiles/s depending on what share of the old 625 ms was handshake. That is
a bracket, not a prediction; the actual figure comes from the new log line:

```
netsource: conn N reused, M fresh, last fetch X ms
```

If `fresh` climbs in step with `reused`, pooling is not happening and something
about `begin()` on a live connection is not behaving as assumed.

One consequence worth knowing: the chunked path for reads over 32 KB now pays
the gap per chunk, so a 129 KB leaf directory costs about 450 ms of deliberate
spacing. Those chunks do share one socket, which they previously did not.

### Measured on device

```
netsource: conn 24 reused, 1 fresh, last fetch 319 ms
netsource: conn 49 reused, 1 fresh, last fetch 401 ms
netsource: conn 74 reused, 1 fresh, last fetch 357 ms
```

**75 requests, one handshake.** `fresh` stays at 1 throughout, so the socket is
surviving between requests as intended.

Rate is ~2.3 req/s (net counter 28 to 62 across one stats interval) against a
prior best of 1.6/s — but that understates it badly. These are z14 tiles
averaging 21 KB and running up to 37 KB; the 1.6/s figure was world-floor tiles
of 93 B to 6 KB, many of them exactly 93 B. The same client is now sustaining a
roughly 20x heavier payload at a higher request rate, at about 57 KB/s in
transfer.

### A consequence worth guarding

The burst runs the largest free DMA block down to 43 KB and the heap low-water
mark to 91 KB. For comparison, the boot where TLS died reported
`esp-aes: Failed to allocate memory for len descriptor` at around 124 KB free.

So the prefetch now operates *below* the level at which handshakes were failing
when every fetch made one. It is safe because pooling means no handshake is
attempted down there — the single handshake happens at the start of the burst
with 143 KB free and a 79 KB DMA block.

The exposure is the retry path. A keep-alive dropped by the far end mid-burst
reconnects at exactly the low-water mark, which is the condition that produced
the original failure. `NET_HANDSHAKE_MIN_DMA` (default 32 KB) now checks for a
usable DMA block before a fresh handshake, waits once for a transient dip, and
declines the fetch with a clear message rather than triggering an AES
allocation failure that would take the connection down with it. Losing one tile
is much cheaper.

It did not trigger in this run — 43 KB observed against a 32 KB threshold — so
it is a safety net rather than a throttle. Its failure path is identical to a
`begin()` failure on a closed pool, which the state-machine test already covers.

---

## Things I found but did not change

**`blit_region` holds `g_glock` across the entire SPI transfer.** `mapengine.h`
says the grid mutex is *"held only for microseconds"*, and for a full-screen
repaint that isn't true — it's held for the whole `writePixels` sequence, which
is the multi-millisecond figure you're already tracking in `last_draw_ms`. That
blocks the worker's commit for the same duration.

I left it alone deliberately. The obvious fix — snapshot the slot pointers under
the lock, release, then blit — weakens the invariant that makes the whole
lock-free pixel scheme sound: the worker could swap a buffer out and start
writing to it mid-blit, giving a torn tile. That's probably rare and probably
transient, but it's your invariant and it's load-bearing, so it's your call
rather than mine. A per-slot "blit in progress" flag the worker respects would
get the latency without the tear, at the cost of some complexity.

Two cheaper things in the same function that are safe: it takes `g_glock` twice
(once for the `any_missing` scan, once for the blit) where one would do, and the
`any_missing` scan walks the whole grid on every call including the small
marker-erase rects.

**`rs_clear` is bandwidth-bound.** It writes 3.3 MB to PSRAM per tile at
`SUBTILE_PX 1280`. I benchmarked 32-bit-word and row-memcpy variants; on x86 the
compiler already vectorises the existing loop so all three tie. Whether wider
stores help on the P4 depends on whether the store width or the PSRAM bus is the
limit, and only the device can answer that. Easy to test — the variants are
straightforward — but I didn't want to claim a win I couldn't demonstrate.

**`to_fx_x`/`to_fx_y` do a 64-bit divide per coordinate.** My first instinct was
that this would be significant, since `src_span` is always a power of two and it
could be a shift. Measuring killed it: they run ~20k times per tile against
570k spans, so it's noise. Mentioning it because it looks like a win and isn't —
and note a plain `>>` wouldn't be bit-exact anyway, since coordinates legitimately
go negative outside the tile and shifts floor where division truncates.

**A fresh TLS handshake per tile is the biggest remaining cost, and I am not
touching it.** `http_range` sets `setReuse(false)` with a comment explaining
why: an undrained response body poisons the next request, surfacing as a tile
that fetches "successfully" and then fails to inflate. That reasoning is sound
and the failure mode is nasty.

But it means every tile pays a full handshake — which is both the dominant
per-fetch latency and, as above, the peak internal-heap moment. The drain loop
before `http.end()` already handles the hazard on the success path; what is
missing is that the two early returns (`code != 206`, and the content-length
mismatch) call `http.end()` without draining. If reuse were enabled, those
paths are exactly where the poison would come from.

So the change is: drain on every exit path, then enable reuse. I think that is
correct and would be a large win on both fetch time and heap pressure. I am
leaving it to you rather than doing it, because I have just demonstrated what
happens when I change something in this codebase that you had already reasoned
carefully about.

**Possible latent bug, unrelated to performance.** `rs_part` sets
`r->extent = part->layer->extent` per layer, but `src_span` is only initialised
if it's `<= 0` — and `mapengine.cpp` sets it to `4096 >> split` up front. A layer
declaring an extent other than 4096 would be transformed against the wrong span
and land in the wrong place. Everything in practice uses 4096, so this may never
have fired, but the two fields can disagree.

---

## Files

- `tab5_map_optimized.zip` — full project with the changes applied
- `verification/bench.c` — the benchmark harness
- `verification/difftest.c` — the differential test

To re-verify: build `difftest.c` against both the old and new `raster.c` and
diff the output. Both harnesses need only `raster.c`, `raster.h` and `mvt.h`.

I'd suggest taking the rasteriser changes and the buffer placement separately,
so `last_render_ms` tells you what each one bought on real hardware.
