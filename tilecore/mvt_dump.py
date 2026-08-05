#!/usr/bin/env python3
"""Decode an MVT tile with a hand-rolled protobuf reader (no deps) and print
its structure. This is ground truth for validating the C decoder."""

import sys, struct
from collections import Counter, defaultdict

def rd_varint(b, p):
    v = 0; s = 0
    while True:
        c = b[p]; p += 1
        v |= (c & 0x7F) << s
        if not (c & 0x80): return v, p
        s += 7

def rd_key(b, p):
    k, p = rd_varint(b, p)
    return k >> 3, k & 7, p

def skip(b, p, wt):
    if wt == 0: _, p = rd_varint(b, p); return p
    if wt == 1: return p + 8
    if wt == 2:
        n, p = rd_varint(b, p); return p + n
    if wt == 5: return p + 4
    raise ValueError(f"wire type {wt}")

def zigzag(v): return (v >> 1) ^ -(v & 1)

GEOM = {0: "UNKNOWN", 1: "POINT", 2: "LINESTRING", 3: "POLYGON"}

def parse_value(b, s, e):
    p = s
    while p < e:
        f, wt, p = rd_key(b, p)
        if f == 1 and wt == 2:
            n, p = rd_varint(b, p); return b[p:p+n].decode("utf8", "replace")
        elif f == 2 and wt == 5:
            return struct.unpack_from("<f", b, p)[0]
        elif f == 3 and wt == 1:
            return struct.unpack_from("<d", b, p)[0]
        elif f in (4, 5) and wt == 0:
            v, p = rd_varint(b, p); return v
        elif f == 6 and wt == 0:
            v, p = rd_varint(b, p); return zigzag(v)
        elif f == 7 and wt == 0:
            v, p = rd_varint(b, p); return bool(v)
        else:
            p = skip(b, p, wt)
    return None

def parse_geometry(b, s, e):
    """Return (n_moveto, n_lineto, n_close, n_points, bbox)."""
    p = s
    x = y = 0
    nm = nl = nc = 0
    npts = 0
    minx = miny = 1 << 30; maxx = maxy = -(1 << 30)
    while p < e:
        cmd, p = rd_varint(b, p)
        cid, cnt = cmd & 7, cmd >> 3
        if cid == 7:
            nc += cnt
            continue
        for _ in range(cnt):
            dx, p = rd_varint(b, p); dy, p = rd_varint(b, p)
            x += zigzag(dx); y += zigzag(dy)
            npts += 1
            minx = min(minx, x); maxx = max(maxx, x)
            miny = min(miny, y); maxy = max(maxy, y)
        if cid == 1: nm += cnt
        elif cid == 2: nl += cnt
    return nm, nl, nc, npts, (minx, miny, maxx, maxy)

def parse_feature(b, s, e):
    p = s
    ftype = 0; tags = []; geom = None; fid = None
    while p < e:
        f, wt, p = rd_key(b, p)
        if f == 1 and wt == 0:
            fid, p = rd_varint(b, p)
        elif f == 2 and wt == 2:
            n, p = rd_varint(b, p); q = p; end = p + n
            while q < end:
                v, q = rd_varint(b, q); tags.append(v)
            p = end
        elif f == 3 and wt == 0:
            ftype, p = rd_varint(b, p)
        elif f == 4 and wt == 2:
            n, p = rd_varint(b, p); geom = (p, p + n); p += n
        else:
            p = skip(b, p, wt)
    return fid, ftype, tags, geom

def parse_layer(b, s, e):
    p = s
    name = None; extent = 4096; version = 1
    keys = []; values = []; feats = []
    while p < e:
        f, wt, p = rd_key(b, p)
        if f == 1 and wt == 2:
            n, p = rd_varint(b, p); name = b[p:p+n].decode(); p += n
        elif f == 2 and wt == 2:
            n, p = rd_varint(b, p); feats.append((p, p + n)); p += n
        elif f == 3 and wt == 2:
            n, p = rd_varint(b, p); keys.append(b[p:p+n].decode()); p += n
        elif f == 4 and wt == 2:
            n, p = rd_varint(b, p); values.append(parse_value(b, p, p + n)); p += n
        elif f == 5 and wt == 0:
            extent, p = rd_varint(b, p)
        elif f == 15 and wt == 0:
            version, p = rd_varint(b, p)
        else:
            p = skip(b, p, wt)
    return name, version, extent, keys, values, feats

def main(path):
    b = open(path, "rb").read()
    print(f"{path}: {len(b)} bytes\n")

    p = 0
    layers = []
    while p < len(b):
        f, wt, p = rd_key(b, p)
        if f == 3 and wt == 2:
            n, p = rd_varint(b, p)
            layers.append((p, p + n)); p += n
        else:
            p = skip(b, p, wt)

    print(f"{len(layers)} layers\n")
    total_pts = 0
    grand = Counter()
    print(f"{'layer':<22} {'ver':>3} {'extent':>6} {'feats':>6} {'pts':>7}  geom types / top keys")
    print("-" * 110)
    for s, e in layers:
        name, ver, extent, keys, values, feats = parse_layer(b, s, e)
        tc = Counter(); pts = 0
        kinds = Counter()
        kind_idx = keys.index("kind") if "kind" in keys else None
        for fs, fe in feats:
            fid, ftype, tags, geom = parse_feature(b, fs, fe)
            tc[GEOM.get(ftype, "?")] += 1
            if geom:
                _, _, _, np_, _ = parse_geometry(b, *geom)
                pts += np_
            if kind_idx is not None:
                for i in range(0, len(tags) - 1, 2):
                    if tags[i] == kind_idx:
                        kinds[values[tags[i + 1]]] += 1
        total_pts += pts
        grand.update(tc)
        tstr = ",".join(f"{k}:{v}" for k, v in tc.most_common())
        kstr = ",".join(f"{k}({v})" for k, v in kinds.most_common(4))
        print(f"{name:<22} {ver:>3} {extent:>6} {len(feats):>6} {pts:>7}  {tstr}")
        if kstr:
            print(f"{'':<22} {'':>3} {'':>6} {'':>6} {'':>7}    kinds: {kstr}")
    print("-" * 110)
    print(f"{'TOTAL':<22} {'':>3} {'':>6} {sum(len(parse_layer(b,s,e)[5]) for s,e in layers):>6} {total_pts:>7}  " +
          ",".join(f"{k}:{v}" for k, v in grand.most_common()))

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "boston.mvt")
