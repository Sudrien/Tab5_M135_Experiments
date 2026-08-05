#!/usr/bin/env python3
"""Synthesise a minimal but spec-correct PMTiles v3 archive for testing
the C reader. Not a general-purpose writer - it exists to produce known
ground truth, including a two-level (root + leaf) directory layout."""

import gzip, struct, sys, json

def varint(v):
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)

def zxy_to_tileid(z, x, y):
    acc = sum((1 << t) * (1 << t) for t in range(z))
    n = 1 << z
    rx = ry = 0
    d = 0
    s = n // 2
    tx, ty = x, y
    while s > 0:
        rx = 1 if (tx & s) else 0
        ry = 1 if (ty & s) else 0
        d += s * s * ((3 * rx) ^ ry)
        if ry == 0:
            if rx == 1:
                tx = s - 1 - tx
                ty = s - 1 - ty
            tx, ty = ty, tx
        s //= 2
    return acc + d

def serialize_dir(entries):
    """entries: list of (tile_id, offset, length, run_length), sorted by id."""
    out = bytearray()
    out += varint(len(entries))
    last = 0
    for e in entries:
        out += varint(e[0] - last)
        last = e[0]
    for e in entries:
        out += varint(e[3])
    for e in entries:
        out += varint(e[2])
    # offsets: emit sentinel 0 when contiguous with the previous entry
    prev_off = prev_len = None
    for e in entries:
        if prev_off is not None and e[1] == prev_off + prev_len:
            out += varint(0)
        else:
            out += varint(e[1] + 1)
        prev_off, prev_len = e[1], e[2]
    return bytes(out)


def build(path, tiles, leaf_split=None):
    """tiles: dict {(z,x,y): payload_bytes}. If leaf_split is an int, entries
    are chunked into leaf directories of that size, producing a 2-level tree."""

    # lay out tile data, deduplicating identical payloads (as real writers do)
    blob = bytearray()
    seen = {}
    entries = []
    for (z, x, y), payload in sorted(tiles.items(), key=lambda kv: zxy_to_tileid(*kv[0])):
        tid = zxy_to_tileid(z, x, y)
        if payload in seen:
            off, ln = seen[payload]
        else:
            off, ln = len(blob), len(payload)
            blob += payload
            seen[payload] = (off, ln)
        entries.append((tid, off, ln, 1))

    entries.sort(key=lambda e: e[0])

    if leaf_split:
        chunks = [entries[i:i + leaf_split] for i in range(0, len(entries), leaf_split)]
        leaf_blob = bytearray()
        root_entries = []
        for ch in chunks:
            ser = gzip.compress(serialize_dir(ch), mtime=0)
            root_entries.append((ch[0][0], len(leaf_blob), len(ser), 0))  # run=0 -> leaf
            leaf_blob += ser
        root_ser = gzip.compress(serialize_dir(root_entries), mtime=0)
        leaf_bytes = bytes(leaf_blob)
    else:
        root_ser = gzip.compress(serialize_dir(entries), mtime=0)
        leaf_bytes = b""

    meta = gzip.compress(json.dumps({"name": "test"}).encode(), mtime=0)

    root_off = 127
    meta_off = root_off + len(root_ser)
    leaf_off = meta_off + len(meta)
    data_off = leaf_off + len(leaf_bytes)

    h = bytearray(127)
    h[0:7] = b"PMTiles"
    h[7] = 3
    struct.pack_into("<Q", h, 8,  root_off);  struct.pack_into("<Q", h, 16, len(root_ser))
    struct.pack_into("<Q", h, 24, meta_off);  struct.pack_into("<Q", h, 32, len(meta))
    struct.pack_into("<Q", h, 40, leaf_off);  struct.pack_into("<Q", h, 48, len(leaf_bytes))
    struct.pack_into("<Q", h, 56, data_off);  struct.pack_into("<Q", h, 64, len(blob))
    struct.pack_into("<Q", h, 72, len(entries))
    struct.pack_into("<Q", h, 80, len(entries))
    struct.pack_into("<Q", h, 88, len(seen))
    h[96] = 1   # clustered
    h[97] = 2   # internal compression: gzip
    h[98] = 2   # tile compression: gzip
    h[99] = 1   # tile type: MVT
    h[100] = min(z for (z, _, _) in tiles)
    h[101] = max(z for (z, _, _) in tiles)
    for i, v in ((102, -1800000000), (106, -850000000),
                 (110, 1800000000), (114, 850000000)):
        struct.pack_into("<i", h, i, v)
    h[118] = h[100]
    struct.pack_into("<i", h, 119, 0)
    struct.pack_into("<i", h, 123, 0)

    with open(path, "wb") as f:
        f.write(h); f.write(root_ser); f.write(meta); f.write(leaf_bytes); f.write(blob)

    return entries


if __name__ == "__main__":
    # A spread of tiles across zooms, plus deliberate gaps to exercise NOTFOUND
    tiles = {}
    for z in range(0, 5):
        n = 1 << z
        for x in range(n):
            for y in range(n):
                if (x + y) % 3 == 2 and z >= 3:
                    continue           # gap
                tiles[(z, x, y)] = b"TILE:%d/%d/%d;" % (z, x, y) + bytes(20)

    e1 = build("flat.pmtiles", tiles)
    e2 = build("leaf.pmtiles", tiles, leaf_split=17)
    print(f"flat.pmtiles: {len(e1)} entries")
    print(f"leaf.pmtiles: {len(e2)} entries, leaf chunks of 17")

    # dump ground truth for the C harness to check against
    with open("expect.txt", "w") as f:
        for (z, x, y), payload in sorted(tiles.items()):
            f.write(f"{z} {x} {y} {len(payload)}\n")
    print(f"expect.txt: {len(tiles)} rows")
