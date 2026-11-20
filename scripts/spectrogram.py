#!/usr/bin/env python3
"""Spectrogram of a WAV as a PNG, optionally stacked over a reference recording (standard library only).

Usage:
  python3 -I scripts/spectrogram.py sim.wav [-o sim.png]
  python3 -I scripts/spectrogram.py sim.wav --ref real.wav [--ref-offset 2.3] [-o cmp.png]
  python3 -I scripts/spectrogram.py sim.wav --stats

The frequency axis is logarithmic (20 Hz up to --fmax, 12 kHz by default) with grid lines at 50, 100,
200, 500 Hz, 1, 2, 5, 10 kHz (labelled on the left); the time axis has a tick every second (labelled
every 5). Colour is level in dB below the loudest bin of the image (--range dB shown, 80 by default).

With --ref the reference is drawn under the simulation on the same time scale (pixels per second);
--ref-offset S skips the first S seconds of the reference (negative: delays it), so a rev or a pass
can be lined up with the same moment of the --wav script. Both panels share one dB scale unless
--normalize each scales each to its own loudest bin (recordings are rarely at the same level).
--from / --to crop the simulation's time range (the reference follows it through the offset).

--stats prints, instead of an image: the peak (dBFS and in 16-bit counts), how many samples sit within
1% of full scale (clipping), the overall RMS and the RMS of every half second, for checking loudness
per phase and headroom. Real recordings: keep them out of git (*.wav is ignored) or check their licence.

Reads 8/16/24/32-bit integer PCM and 32-bit float WAVs at any rate; stereo is mixed to mono.
"""
import argparse
import array
import cmath
import math
import struct
import sys
import zlib


def read_wav(path):
    """(samples in [-1, 1] as a list of floats, sample rate)"""
    data = open(path, "rb").read()
    if data[:4] != b"RIFF" or data[8:12] != b"WAVE":
        raise ValueError("not a WAV: " + path)
    pos, fmt, raw = 12, None, None
    while pos + 8 <= len(data):
        kind, n = struct.unpack("<4sI", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        if kind == b"fmt ":
            fmt = struct.unpack("<HHIIHH", body[:16])
            if fmt[0] == 0xFFFE and len(body) >= 26:  # WAVE_FORMAT_EXTENSIBLE: the real tag is in the GUID
                fmt = (struct.unpack("<H", body[24:26])[0],) + fmt[1:]
        elif kind == b"data":
            raw = body
        pos += 8 + n + (n & 1)
    if fmt is None or raw is None:
        raise ValueError("no fmt or data chunk: " + path)
    tag, channels, rate, _, _, bits = fmt
    width = bits // 8
    frames = len(raw) // (width * channels)
    raw = raw[:frames * width * channels]
    if tag == 3 and bits == 32:
        vals = array.array("f", raw)
        scale = 1.0
    elif tag == 1 and bits == 16:
        vals, scale = array.array("h", raw), 1.0 / 32768.0
    elif tag == 1 and bits == 32:
        vals, scale = array.array("i", raw), 1.0 / 2147483648.0
    elif tag == 1 and bits == 8:
        vals, scale = [b - 128 for b in raw], 1.0 / 128.0
    elif tag == 1 and bits == 24:
        vals = [int.from_bytes(raw[i:i + 3], "little", signed=True) for i in range(0, len(raw), 3)]
        scale = 1.0 / 8388608.0
    else:
        raise ValueError("unsupported WAV format (tag %d, %d bits): %s" % (tag, bits, path))
    if sys.byteorder != "little" and isinstance(vals, array.array):
        vals.byteswap()
    if channels == 1:
        return [v * scale for v in vals], rate
    s = scale / channels
    return [sum(vals[i:i + channels]) * s for i in range(0, len(vals), channels)], rate


def stats(path):
    x, rate = read_wav(path)
    peak = max((abs(v) for v in x), default=0.0)
    near = sum(1 for v in x if abs(v) >= 0.99 * 24000.0 / 32768.0) if x else 0
    rms = math.sqrt(sum(v * v for v in x) / max(1, len(x)))
    db = lambda v: 20.0 * math.log10(max(v, 1e-9))
    print("%s: %.2f s at %d Hz" % (path, len(x) / rate, rate))
    print("  peak %.1f dBFS (%d of 32768), %d samples within 1%% of the old soft-clip ceiling (24000)"
          % (db(peak), round(peak * 32768), near))
    print("  rms  %.1f dBFS overall" % db(rms))
    half = rate // 2
    line = []
    for i in range(0, len(x), half):
        w = x[i:i + half]
        r = math.sqrt(sum(v * v for v in w) / len(w))
        p = max(abs(v) for v in w)
        line.append("%5.1fs %6.1f/%5.1f" % (i / rate, db(r), db(p)))
    print("  every 0.5 s (start, rms dBFS / peak dBFS):")
    for k in range(0, len(line), 4):
        print("    " + "   ".join(line[k:k + 4]))


def fft(a):
    """in-place iterative radix-2 FFT of a list of complex numbers (len a power of two)"""
    n = len(a)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    size = 2
    while size <= n:
        half = size // 2
        w = [cmath.exp(-2j * math.pi * k / size) for k in range(half)]
        for start in range(0, n, size):
            for k in range(half):
                t = w[k] * a[start + k + half]
                u = a[start + k]
                a[start + k] = u + t
                a[start + k + half] = u - t
        size *= 2
    return a


def spectrogram(x, rate, t0, t1, cols, rows, fmin, fmax):
    """power in dB, rows x cols (row 0 = fmax), columns centred from t0 to t1 seconds"""
    n = 1
    while n < rate / 12.0:  # bins of ~11 Hz or finer, so the low firing notes resolve
        n *= 2
    win = [0.5 - 0.5 * math.cos(2.0 * math.pi * k / n) for k in range(n)]
    # each image row takes the strongest FFT bin in its log-frequency band (or the nearest one)
    edges = [fmin * (fmax / fmin) ** (r / rows) for r in range(rows + 1)]
    bands = []
    for r in range(rows):
        lo = max(1, int(edges[r] * n / rate))
        hi = max(lo + 1, int(math.ceil(edges[r + 1] * n / rate)))
        bands.append((lo, min(hi, n // 2)))
    out = [[-200.0] * cols for _ in range(rows)]
    for c in range(cols):
        t = t0 + (t1 - t0) * (c + 0.5) / cols
        mid = int(t * rate)
        start = mid - n // 2
        if start + n <= 0 or start >= len(x):
            continue
        frame = [complex((x[start + k] if 0 <= start + k < len(x) else 0.0) * win[k]) for k in range(n)]
        spec = fft(frame)
        mag = [abs(v) ** 2 for v in spec[:n // 2]]
        for r, (lo, hi) in enumerate(bands):
            p = max(mag[lo:hi]) if hi > lo else mag[lo]
            out[rows - 1 - r][c] = 10.0 * math.log10(p + 1e-20)
    return out


# a small heat ramp: black - purple - red - orange - yellow - white
RAMP = [(0, 0, 4), (40, 11, 84), (101, 21, 110), (159, 42, 99), (212, 72, 66), (245, 125, 21),
        (250, 193, 39), (252, 255, 164)]


def colour(u):
    u = min(1.0, max(0.0, u)) * (len(RAMP) - 1)
    i = min(int(u), len(RAMP) - 2)
    f = u - i
    a, b = RAMP[i], RAMP[i + 1]
    return tuple(int(a[k] + (b[k] - a[k]) * f) for k in range(3))


# 3x5 glyphs for the axis labels
FONT = {
    "0": "111101101101111", "1": "010110010010111", "2": "111001111100111", "3": "111001111001111",
    "4": "101101111001001", "5": "111100111001111", "6": "111100111101111", "7": "111001001001001",
    "8": "111101111101111", "9": "111101111001111", "k": "100101110101101", "s": "000011110001110",
    ".": "000000000000010", "-": "000000111000000",
}


def text(img, w, h, x, y, s, rgb):
    for ch in s:
        g = FONT.get(ch)
        if g:
            for k, bit in enumerate(g):
                px, py = x + k % 3, y + k // 3
                if bit == "1" and 0 <= px < w and 0 <= py < h:
                    img[py][px] = rgb
        x += 4


def write_png(path, img):
    h, w = len(img), len(img[0])
    raw = bytearray()
    for row in img:
        raw.append(0)
        for r, g, b in row:
            raw += bytes((r, g, b))
    chunk = lambda kind, body: struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
        f.write(chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wav")
    ap.add_argument("-o", "--out", help="PNG to write (default: the WAV's name with .png)")
    ap.add_argument("--ref", help="reference recording drawn under it on the same time scale")
    ap.add_argument("--ref-offset", type=float, default=0.0, help="seconds of the reference to skip (negative: delay it)")
    ap.add_argument("--from", dest="t0", type=float, default=0.0, help="start, s")
    ap.add_argument("--to", dest="t1", type=float, help="end, s (default: the end of the longer file)")
    ap.add_argument("--fmin", type=float, default=20.0)
    ap.add_argument("--fmax", type=float, default=12000.0)
    ap.add_argument("--px-per-s", type=float, default=50.0, help="horizontal scale (default 50)")
    ap.add_argument("--rows", type=int, default=360, help="height of each panel")
    ap.add_argument("--range", type=float, default=80.0, help="dB shown below the loudest bin")
    ap.add_argument("--normalize", choices=("shared", "each"), default="shared")
    ap.add_argument("--stats", action="store_true", help="print peak and loudness instead of drawing")
    a = ap.parse_args()
    try:
        if a.stats:
            stats(a.wav)
            if a.ref:
                stats(a.ref)
            return 0
        x, rate = read_wav(a.wav)
        panels = [(x, rate, 0.0)]
        if a.ref:
            y, rrate = read_wav(a.ref)
            panels.append((y, rrate, a.ref_offset))
    except (OSError, ValueError) as e:
        print(e, file=sys.stderr)
        return 2
    t1 = a.t1 if a.t1 is not None else max(len(p[0]) / p[1] - p[2] for p in panels)
    t0 = a.t0
    fmax = min(a.fmax, min(p[1] for p in panels) / 2.0)
    cols = max(16, int((t1 - t0) * a.px_per_s))
    grams = []
    for i, (s, r, off) in enumerate(panels):
        print("panel %d: %d columns ..." % (i + 1, cols), file=sys.stderr)
        grams.append(spectrogram(s, r, t0 + off, t1 + off, cols, a.rows, a.fmin, fmax))
    tops = [max(max(row) for row in g) for g in grams]
    if a.normalize == "shared":
        tops = [max(tops)] * len(tops)

    left, gap, bottom = 22, 6, 10
    w = left + cols
    h = len(grams) * a.rows + (len(grams) - 1) * gap + bottom
    bg, grid, label = (24, 24, 28), (90, 90, 100), (200, 200, 210)
    img = [[bg] * w for _ in range(h)]
    lines = [50, 100, 200, 500, 1000, 2000, 5000, 10000]
    for p, g in enumerate(grams):
        y0 = p * (a.rows + gap)
        for r in range(a.rows):
            row = img[y0 + r]
            for c in range(cols):
                row[left + c] = colour(1.0 + (g[r][c] - tops[p]) / a.range)
        for f in lines:
            if a.fmin < f < fmax:
                r = y0 + int(round(a.rows * (1.0 - math.log(f / a.fmin) / math.log(fmax / a.fmin))))
                for c in range(left - 3, w):
                    if c < left or c % 6 < 2:  # dashed over the image
                        img[r][c] = grid
                lab = "%dk" % (f // 1000) if f >= 1000 else str(f)
                text(img, w, h, 1, max(y0, r - 2), lab, label)
    ybase = h - bottom
    for sec in range(int(math.ceil(t0)), int(t1) + 1):
        c = left + int((sec - t0) * a.px_per_s)
        if c >= w:
            break
        for r in range(ybase, ybase + (4 if sec % 5 else 6)):
            img[r][c] = label
        if sec % 5 == 0:
            text(img, w, h, c + 2, ybase + 4, "%ds" % sec, label)
    out = a.out or a.wav.rsplit(".", 1)[0] + ".png"
    write_png(out, img)
    print("wrote %s (%dx%d, %.1f-%.1f s, %d-%d Hz)" % (out, w, h, t0, t1, a.fmin, fmax))
    return 0


if __name__ == "__main__":
    sys.exit(main())
