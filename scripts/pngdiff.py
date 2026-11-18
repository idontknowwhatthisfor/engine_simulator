#!/usr/bin/env python3
"""Pixel diff of two PNG screenshots over the whole image, HUD included (standard library only).

Usage: python3 -I scripts/pngdiff.py a.png b.png
Prints "N of M pixels differ, max channel diff D" (RGB only) and exits 0 when identical, 1 when not,
2 when a file cannot be read. engine_sim --compare skips the HUD rows; this does not.
"""
import struct
import sys
import zlib


def read_png(path):
    data = open(path, "rb").read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG: " + path)
    pos, idat, w = 8, b"", None
    while pos < len(data):
        n, kind = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        pos += 12 + n
        if kind == b"IHDR":
            w, h, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
            if depth != 8 or ctype not in (2, 6) or interlace:
                raise ValueError("only 8-bit RGB/RGBA non-interlaced PNGs: " + path)
            bpp = 4 if ctype == 6 else 3
        elif kind == b"IDAT":
            idat += body
        elif kind == b"IEND":
            break
    raw = zlib.decompress(idat)
    stride = w * bpp
    rows, prev = [], bytearray(stride)
    for y in range(h):
        f = raw[y * (stride + 1)]
        line = bytearray(raw[y * (stride + 1) + 1:(y + 1) * (stride + 1)])
        for x in range(stride):
            a = line[x - bpp] if x >= bpp else 0
            b = prev[x]
            c = prev[x - bpp] if x >= bpp else 0
            if f == 1:
                line[x] = (line[x] + a) & 255
            elif f == 2:
                line[x] = (line[x] + b) & 255
            elif f == 3:
                line[x] = (line[x] + (a + b) // 2) & 255
            elif f == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                line[x] = (line[x] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
        rows.append(bytes(line))
        prev = line
    return w, h, bpp, rows


def main():
    try:
        wa, ha, ba, ra = read_png(sys.argv[1])
        wb, hb, bb, rb = read_png(sys.argv[2])
    except (OSError, ValueError, zlib.error) as ex:
        print("cannot read: %s" % ex)
        return 2
    if (wa, ha) != (wb, hb):
        print("size differs: %dx%d vs %dx%d" % (wa, ha, wb, hb))
        return 1
    diff, worst = 0, 0
    for y in range(ha):
        if ra[y] == rb[y]:
            continue
        for x in range(wa):
            d = max(abs(ra[y][x * ba + k] - rb[y][x * bb + k]) for k in range(3))
            if d:
                diff += 1
                worst = max(worst, d)
    print("%d of %d pixels differ, max channel diff %d" % (diff, wa * ha, worst))
    return 0 if diff == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
