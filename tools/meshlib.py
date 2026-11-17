"""Shared mesh helpers for the engine builders (standard library only).

read_i4 reads the inline-4 engine.mesh (engines/inline4_fit/extract_parts.py makes it); Mesh, prism,
cylinder, box, ... build the procedural parts the V4 and S 1000 RR builders add.
"""
import math
import struct

STATIC, CRANK, PISTON, ROD, BLOCK = range(5)


# ---- engine.mesh v1 reader -------------------------------------------------------------------
def read_i4(path):
    d = open(path, "rb").read()
    assert d[:8] == b"I4ENGv1\0", "not an inline-4 engine.mesh"
    p = 8
    root = struct.unpack_from("<16f", d, p); p += 64
    nm, = struct.unpack_from("<I", d, p); p += 4
    mats = []
    for _ in range(nm):
        mats.append(struct.unpack_from("<4f2fI", d, p)); p += 28
    axis = struct.unpack_from("<3f", d, p); p += 12
    nc, = struct.unpack_from("<I", d, p); p += 4
    cyls = []
    for _ in range(nc):
        cyls.append(struct.unpack_from("<8f", d, p)); p += 32
    ng, = struct.unpack_from("<I", d, p); p += 4
    groups = []
    for _ in range(ng):
        kind, cyl, mat, nv, ni = struct.unpack_from("<IiIII", d, p); p += 20
        v = struct.unpack_from("<%df" % (nv * 6), d, p); p += nv * 24
        i = struct.unpack_from("<%dI" % ni, d, p); p += ni * 4
        groups.append((kind, cyl, mat, v, i))
    return root, mats, axis, cyls, groups


# ---- small mesh helpers ----------------------------------------------------------------------
class Mesh:
    def __init__(self):
        self.v = []   # flat px py pz nx ny nz
        self.i = []

    def vert(self, p, n):
        self.v += [p[0], p[1], p[2], n[0], n[1], n[2]]
        return len(self.v) // 6 - 1

    def tri(self, a, b, c):
        self.i += [a, b, c]

    def add(self, other):
        base = len(self.v) // 6
        self.v += other.v
        self.i += [k + base for k in other.i]

    def transformed(self, fn_p, fn_n):
        m = Mesh()
        m.i = list(self.i)
        for k in range(0, len(self.v), 6):
            m.v += list(fn_p(self.v[k:k + 3])) + list(fn_n(self.v[k + 3:k + 6]))
        return m


def norm(v):
    l = math.sqrt(sum(c * c for c in v)) or 1.0
    return [c / l for c in v]


def convex_hull(pts):
    pts = sorted(set((round(x, 4), round(y, 4)) for x, y in pts))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]  # counter-clockwise


def prism(poly, z0, z1, smooth_deg=25.0):
    """Convex CCW polygon in XY extruded from z0 to z1, outward-facing CCW triangles."""
    m = Mesh()
    n = len(poly)
    edge_n = []
    for k in range(n):
        (x0, y0), (x1, y1) = poly[k], poly[(k + 1) % n]
        edge_n.append(norm([y1 - y0, -(x1 - x0), 0.0]))
    for k in range(n):
        a, b = poly[k], poly[(k + 1) % n]
        en = edge_n[k]
        # smooth across shallow corners so arcs shade round, keep sharp corners sharp
        def corner(j, e):
            other = edge_n[(j - 1) % n] if e is a else edge_n[(j + 1) % n]
            if en[0] * other[0] + en[1] * other[1] > math.cos(math.radians(smooth_deg)):
                return norm([en[0] + other[0], en[1] + other[1], 0.0])
            return en
        na, nb = corner(k, a), corner(k + 1, b)
        i0 = m.vert((a[0], a[1], z0), na)
        i1 = m.vert((b[0], b[1], z0), nb)
        i2 = m.vert((b[0], b[1], z1), nb)
        i3 = m.vert((a[0], a[1], z1), na)
        m.tri(i0, i1, i2)
        m.tri(i0, i2, i3)
    cx = sum(p[0] for p in poly) / n
    cy = sum(p[1] for p in poly) / n
    for z, nz in ((z1, 1.0), (z0, -1.0)):
        c = m.vert((cx, cy, z), (0, 0, nz))
        ring = [m.vert((p[0], p[1], z), (0, 0, nz)) for p in poly]
        for k in range(n):
            if nz > 0:
                m.tri(c, ring[k], ring[(k + 1) % n])
            else:
                m.tri(c, ring[(k + 1) % n], ring[k])
    return m


def circle(cx, cy, r, seg=48, a0=0.0, a1=2 * math.pi):
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    count = seg if full else seg + 1
    return [(cx + r * math.cos(a0 + (a1 - a0) * k / seg), cy + r * math.sin(a0 + (a1 - a0) * k / seg))
            for k in range(count)]


def cylinder(cx, cy, r, z0, z1, seg=48):
    return prism(circle(cx, cy, r, seg), z0, z1, smooth_deg=360.0 / seg + 1.0)


def box(lo, hi):
    poly = [(lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1])]
    return prism(poly, lo[2], hi[2], smooth_deg=0.0)


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (lambda p: (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]),
            lambda n: (c * n[0] - s * n[1], s * n[0] + c * n[1], n[2]))
