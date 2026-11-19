"""Shared mesh helpers and engine-building steps for the engine builders (standard library only).

Engine mesh format ENGMSHv3 (little endian), written by write_engine for every engine and read by
src/model.hpp (which also still reads the older I4ENGv1, ENGMSHv2 and BMWS1KRR files):
    char[8]  magic "ENGMSHv3"
    f32[16]  root matrix (column major, as in glTF)
    u32      material count, then per material: f32 rgba[4], f32 metallic, f32 roughness, u32 blend
    f32[3]   crank axis point (x, y, z); the axis direction is +Z
    u32      cylinder count (any number), then per cylinder 12 x f32:
                 z_centre, gudgeon_x, gudgeon_y, crankpin_x, crankpin_y, throw_r,
                 theta0 (rad, rest crankpin angle from the bore axis), rod_L,
                 bank (rad: the cylinder frame, bore toward -Y, turned by R_z(bank) about the crank axis),
                 deck_y (cylinder frame), bore (mm), firing lag (deg of the 720 deg cycle behind cylinder 1)
    u32      metadata count, then per entry: u32 name length, char[length] name (no terminator),
             u32 value count, f32[value count] values. Named values the simulator reads:
                 banked           1: built by these steps (pivots and piston / rod vertices in the
                                  cylinder frame, the crank-vs-skirt clearance test applies); 0: the
                                  original inline-4 model
                 zPrimary         where the crank's primary gear sits along the crank (bike gearbox)
                 supercharger_at  a supercharger's frame origin in mesh space (3 values)
    u32      group count, then per group:
                 u32 kind (0 static, 1 crank, 2 piston, 3 rod, 4 block), i32 cylinder (-1 if none),
                 u32 material, u32 vertex count, u32 index count,
                 f32[6 * vertex count] (px py pz nx ny nz), u32[index count]

read_i4 reads the inline-4 engine.mesh (engines/inline4_fit/extract_parts.py makes it); Mesh, prism,
cylinder, box, ... build procedural parts. The steps every engine built from the inline-4's parts
shares are here too, so a builder only declares its own dimensions, layout and extra parts:
    Donor(path).fit(bore, throw, rod_l)   the inline-4's piston and rod, scaled, with the rod beam
                                          shortened; .cylinders(layout, deck_y) places them
    build_crank, main_shells              a procedural crankshaft from its crankpins, and the shells
    build_block                           crankcase, barrels, heads and cam covers per bank
    write_engine                          the banked engine mesh
    inline_engine                         all of the above for an inclined inline engine
"""
import math
import struct

STATIC, CRANK, PISTON, ROD, BLOCK = range(5)


# ---- engine.mesh reader (the inline-4: v1 or v3) ---------------------------------------------
def read_i4(path):
    """root, materials, crank axis, cylinders (their first 8 values), groups of an engine mesh"""
    d = open(path, "rb").read()
    v3 = d[:8] == b"ENGMSHv3"
    assert v3 or d[:8] == b"I4ENGv1\0", "not an inline-4 engine.mesh"
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
        cyls.append(struct.unpack_from("<8f", d, p)); p += 48 if v3 else 32
    if v3:
        nmeta, = struct.unpack_from("<I", d, p); p += 4
        for _ in range(nmeta):
            n, = struct.unpack_from("<I", d, p); p += 4 + n
            n, = struct.unpack_from("<I", d, p); p += 4 + 4 * n
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


# ---- building an engine from the inline-4's parts --------------------------------------------
# Mesh space (mm, the engine.mesh conventions): crank axis = Z through (0, 0), "up" is -Y. A
# cylinder's own frame has its bore toward -Y; R_z(bank) turns it into mesh space.

def pin_xy(angle_deg, r):
    """crankpin position at an angle measured from -Y towards +X (the R_z sense)."""
    a = math.radians(angle_deg)
    return (r * math.sin(a), -r * math.cos(a))


class Donor:
    """Cylinder 1's piston + gudgeon pin and connecting rod + cap bolts/nuts + big-end shell from the
    inline-4 engine.mesh (engines/inline4_fit/extract_parts.py makes it), with its materials and root
    matrix, to be scaled onto another engine with fit()."""

    def __init__(self, path):
        self.root, self.mats, self.axis, self.cyls, groups = read_i4(path)
        self.z_c, self.gx, self.gy, cx, cy, _, _, self.L = self.cyls[0]
        self.ux, self.uy = (cx - self.gx) / self.L, (cy - self.gy) / self.L
        self.piston_parts = [(m, v, i) for k, c, m, v, i in groups if k == PISTON and c == 0]
        self.rod_parts = [(m, v, i) for k, c, m, v, i in groups if k == ROD and c == 0]
        assert len(self.piston_parts) >= 2 and len(self.rod_parts) >= 2

    def fit(self, bore, throw, rod_l):
        return FittedParts(self, bore, throw, rod_l)


class FittedParts:
    """The donor's piston and rod scaled from its 85 mm bore to `bore`, with the rod beam shortened to
    rod_l. Only the beam stretches: the small eye and the big end with its bolts stay rigid, so both
    eyes stay round and the rods still fit their pins."""

    def __init__(self, d, bore, throw, rod_l):
        self.d, self.bore, self.throw, self.rod_l = d, bore, throw, rod_l
        s = self.s = bore / 85.0  # the inline-4 bore, see extract_parts.py
        # piston crown / skirt relative to the gudgeon pin (scaled), from the piston body (largest part)
        body = max(d.piston_parts, key=lambda t: len(t[1]))[1]
        ys = body[1::6]
        self.crown_above = s * (d.gy - min(ys))
        self.skirt_below = s * (max(ys) - d.gy)
        self.rod_half_w = s * max(abs(z - d.z_c) for _, v, _ in d.rod_parts for z in v[2::6])
        # rod beam stretch: rigid small eye (a < A_S) and big end (a > A_B), linear in between
        self.L_s = s * d.L
        self.A_S, self.A_B = 19.0, self.L_s - 44.0
        self.shrink = self.L_s - rod_l
        assert rod_l - 44.0 > self.A_S + 5.0, "rod too short for the donor's eyes"
        self.k_beam = (self.A_B - self.shrink - self.A_S) / (self.A_B - self.A_S)

    def stretch(self, a):
        if a <= self.A_S:
            return a, 1.0
        if a >= self.A_B:
            return a - self.shrink, 1.0
        return self.A_S + (a - self.A_S) * self.k_beam, self.k_beam

    def slider(self, theta_deg):
        """gudgeon and crankpin centres in the cylinder frame at local crank angle theta."""
        t = math.radians(theta_deg)
        throw, rod_l = self.throw, self.rod_l
        pin = (throw * math.sin(t), -throw * math.cos(t))
        gyy = -(throw * math.cos(t) + math.sqrt(rod_l ** 2 - (throw * math.sin(t)) ** 2))
        return (0.0, gyy), pin

    def place_piston(self, theta_deg, z_new):
        d, s = self.d, self.s
        g, _ = self.slider(theta_deg)
        out = []
        for m, v, idx in d.piston_parts:
            mesh = Mesh()
            mesh.i = list(idx)
            for k in range(0, len(v), 6):
                p = (g[0] + s * (v[k] - d.gx), g[1] + s * (v[k + 1] - d.gy), z_new + s * (v[k + 2] - d.z_c))
                mesh.v += list(p) + list(v[k + 3:k + 6])
            out.append((m, mesh))
        return out

    def place_rod(self, theta_deg, z_new):
        d, s, ux, uy = self.d, self.s, self.d.ux, self.d.uy
        g, pin = self.slider(theta_deg)
        dd = ((pin[0] - g[0]) / self.rod_l, (pin[1] - g[1]) / self.rod_l)  # small end -> big end
        e = (dd[1], -dd[0])                                                 # rod's sideways axis
        out = []
        for m, v, idx in d.rod_parts:
            mesh = Mesh()
            mesh.i = list(idx)
            for k in range(0, len(v), 6):
                # donor rod frame: b sideways, a along the rod from the small end
                px, py = v[k] - d.gx, v[k + 1] - d.gy
                b = s * (uy * px - ux * py)
                a = s * (ux * px + uy * py)
                a2, kk = self.stretch(a)
                nx, ny = v[k + 3], v[k + 4]
                nb = uy * nx - ux * ny
                na = (ux * nx + uy * ny) / kk   # normals of a stretched surface tilt the other way
                nz = v[k + 5]
                ln = math.sqrt(nb * nb + na * na + nz * nz) or 1.0
                nb, na, nz = nb / ln, na / ln, nz / ln
                p = (g[0] + b * e[0] + a2 * dd[0], g[1] + b * e[1] + a2 * dd[1], z_new + s * (v[k + 2] - d.z_c))
                n = (nb * e[0] + na * dd[0], nb * e[1] + na * dd[1], nz)
                mesh.v += list(p) + list(n)
            out.append((m, mesh))
        return out

    def deck_y(self, squish):
        """block deck in the cylinder frame: the crown sits `squish` below it at TDC."""
        return -(self.throw + self.rod_l + self.crown_above + squish)

    def cylinders(self, layout, deck_y):
        """layout: per cylinder (z centre, local crank angle at rest in deg, bank in deg, firing lag in
        deg). Returns the cylinder records and the piston and rod groups (in the cylinder frame)."""
        out_cyls, out_groups = [], []
        for c, (z, theta, bank, lag) in enumerate(layout):
            g, p = self.slider(theta)
            out_cyls.append((z, g[0], g[1], p[0], p[1], self.throw, math.radians(theta), self.rod_l,
                             math.radians(bank), deck_y, self.bore, lag))
            for m, mesh in self.place_piston(theta, z):
                out_groups.append((PISTON, c, m, mesh))
            for m, mesh in self.place_rod(theta, z):
                out_groups.append((ROD, c, m, mesh))
        return out_cyls, out_groups


# ---- swept tubes and flat-shaded solids (the H2's supercharger, the RS 3's turbo) ---------------
def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def tube(path, radii, seg=28, caps=False):
    """circle swept along a polyline (parallel-transport frames), outward normals; radii: one per point.
    caps: close both ends with flat discs."""
    m = Mesh()
    n = len(path)
    tans = []
    for k in range(n):
        a, b = path[max(k - 1, 0)], path[min(k + 1, n - 1)]
        tans.append(norm(sub(b, a)))
    t0 = tans[0]
    ref = (0.0, 1.0, 0.0) if abs(t0[1]) < 0.9 else (1.0, 0.0, 0.0)
    u = norm(cross(t0, ref))
    rings = []
    for k in range(n):
        t = tans[k]
        u = norm(sub(u, tuple(t[j] * sum(u[i] * t[i] for i in range(3)) for j in range(3))))
        v = cross(t, u)
        ring = []
        for j in range(seg):
            a = 2 * math.pi * j / seg
            d = tuple(math.cos(a) * u[i] + math.sin(a) * v[i] for i in range(3))
            ring.append(m.vert(tuple(path[k][i] + radii[k] * d[i] for i in range(3)), d))
        rings.append(ring)
    for k in range(n - 1):
        for j in range(seg):
            a, b = rings[k][j], rings[k][(j + 1) % seg]
            c, d = rings[k + 1][(j + 1) % seg], rings[k + 1][j]
            m.tri(a, c, b)
            m.tri(a, d, c)
    if caps:
        for k, sgn in ((0, -1.0), (n - 1, 1.0)):
            t = tuple(sgn * c for c in tans[k])
            ctr = m.vert(path[k], t)
            ring = [m.vert(tuple(m.v[6 * i:6 * i + 3]), t) for i in rings[k]]
            for j in range(seg):
                if sgn > 0:
                    m.tri(ctr, ring[j], ring[(j + 1) % seg])
                else:
                    m.tri(ctr, ring[(j + 1) % seg], ring[j])
    return m


def solid(faces):
    """flat-shaded triangles from (a, b, c) point triples."""
    m = Mesh()
    for a, b, c in faces:
        nrm = norm(cross(sub(b, a), sub(c, a)))
        m.tri(m.vert(a, nrm), m.vert(b, nrm), m.vert(c, nrm))
    return m


def build_crank(pins, throw, r_main, r_pin, r_cw, web, alt_r, hub_r, main_order=None):
    """A crankshaft in mesh space. pins: per crankpin (z centre, half length, mesh angle in deg); a web
    with a counterweight opposite the pin on each side of it, a main journal at each end and between
    neighbouring pins' webs, stubs, the alternator rotor on the left (-z) and the primary gear hub on
    the right. main_order: the order the mains are written in (indices into left, between..., right).
    Returns the crank Mesh, the mains [(z0, z1)] in that order, the crank's z extent and the primary
    gear's z."""
    crank = Mesh()
    webs = []  # (z0, z1, pin angle)
    for zp, half, ang in pins:
        px, py = pin_xy(ang, throw)
        crank.add(cylinder(px, py, r_pin, zp - half, zp + half, 40))
        webs.append((zp - half - web, zp - half, ang))
        webs.append((zp + half, zp + half + web, ang))
    for z0, z1, ang in webs:
        px, py = pin_xy(ang, throw)
        cw = math.radians(ang + 180.0)
        # pin boss, a hub round the main, and a counterweight arc opposite the pin
        pts = circle(px, py, r_pin + 7.0, 40) + circle(0.0, 0.0, r_main + 9.0, 40)
        a_mid = math.atan2(-math.cos(cw), math.sin(cw))  # pin_xy convention -> atan2 angle
        pts += circle(0.0, 0.0, r_cw, 40, a_mid - math.radians(65), a_mid + math.radians(65))
        crank.add(prism(convex_hull(pts), z0, z1))
    w_lo, w_hi = webs[0][0], webs[-1][1]
    mains = [(w_lo - 20.0, w_lo)]
    for c in range(len(pins) - 1):
        mains.append((webs[2 * c + 1][1], webs[2 * c + 2][0]))
    mains.append((w_hi, w_hi + 20.0))
    if main_order is not None:
        mains = [mains[k] for k in main_order]
    for z0, z1 in mains:
        crank.add(cylinder(0, 0, r_main, z0, z1))
    crank.add(cylinder(0, 0, r_main - 4.0, w_lo - 48.0, w_lo - 20.0))  # left stub
    crank.add(cylinder(0, 0, alt_r, w_lo - 46.0, w_lo - 26.0, 64))     # alternator rotor
    crank.add(cylinder(0, 0, r_main - 4.0, w_hi + 20.0, w_hi + 42.0))  # right stub
    crank.add(cylinder(0, 0, hub_r, w_hi + 24.0, w_hi + 38.0, 72))     # primary gear hub (teeth: the sim)
    return crank, mains, w_lo, w_hi, w_hi + 31.0


def main_shells(mains, r_main):
    """main bearing shells (static, bronze like the inline-4's), one per main journal."""
    return [(STATIC, -1, 1, cylinder(0, 0, r_main + 2.5, z0 + 1, z1 - 1)) for z0, z1 in mains]


def build_block(banks, bore, deck_y, case_r, case_z, sump_y, margin, head_h, head_out, cover_out):
    """A see-through crankcase and, per bank (bank angle in deg, lowest and highest cylinder z), a
    one-piece barrel, a head head_h tall and a 20 mm cam cover on it, built in the bank's frame and
    turned into mesh space. margin: barrel length past the outer bores; head_out / cover_out: how far
    the head and the cover reach past the bore sideways. Returns (block Mesh, covers Mesh)."""
    block, covers = Mesh(), Mesh()
    sump = circle(0, 0, case_r, 64) + [(-case_r + 8, sump_y), (case_r - 8, sump_y)]
    block.add(prism(convex_hull(sump), case_z[0], case_z[1]))
    for bank, z_lo, z_hi in banks:
        z0, z1 = z_lo - bore / 2 - margin, z_hi + bore / 2 + margin
        fp, fn = rot_z(bank)
        barrel = box((-bore / 2 - 9.0, deck_y, z0), (bore / 2 + 9.0, -case_r + 12.0, z1))
        head = box((-bore / 2 - head_out, deck_y - head_h, z0 - 3.0), (bore / 2 + head_out, deck_y, z1 + 3.0))
        cover = box((-bore / 2 - cover_out, deck_y - (head_h + 20.0), z0 + 4.0),
                    (bore / 2 + cover_out, deck_y - head_h, z1 - 4.0))
        block.add(barrel.transformed(fp, fn))
        block.add(head.transformed(fp, fn))
        covers.add(cover.transformed(fp, fn))
    return block, covers


def write_engine(out_file, root, mats, cyls, groups, meta=(), axis=(0.0, 0.0, 0.0)):
    """an ENGMSHv3 engine mesh (the format is at the top of this file). cyls: 12 values each; meta:
    (name, [values]) pairs; groups: (kind, cylinder, material, mesh with .v and .i). Returns the
    byte count."""
    out = bytearray(b"ENGMSHv3")
    out += struct.pack("<16f", *root)
    out += struct.pack("<I", len(mats))
    for m in mats:
        out += struct.pack("<4f2fI", *m)
    out += struct.pack("<3f", *axis)
    out += struct.pack("<I", len(cyls))
    for cy_ in cyls:
        out += struct.pack("<12f", *cy_)
    out += struct.pack("<I", len(meta))
    for name, values in meta:
        b = name.encode("ascii")
        out += struct.pack("<I", len(b)) + b + struct.pack("<I%df" % len(values), len(values), *values)
    out += struct.pack("<I", len(groups))
    for kind, cyl, mat, mesh in groups:
        out += struct.pack("<IiIII", kind, cyl, mat, len(mesh.v) // 6, len(mesh.i))
        out += struct.pack("<%df" % len(mesh.v), *mesh.v)
        out += struct.pack("<%dI" % len(mesh.i), *mesh.i)
    with open(out_file, "wb") as f:
        f.write(out)
    return len(out)


def primary_meta(z_primary):
    """the zPrimary metadata entry. Stored to 0.1 mm, as the simulator has always drawn it (the value
    used to be copied by hand from the builder's printout)."""
    return ("zPrimary", [round(z_primary, 1)])


def inline_engine(donor_path, out_file, name, bore, stroke, rod_l, pitch, incline, firing, cover_mat,
                  rest_local1=30.0, squish=1.0, web=12.0, meta=()):
    """An inline engine from the inline-4's parts, the bank inclined `incline` deg forward (toward +X,
    with the front of the bike at +X the crank turns forward): cylinders `pitch` apart along the crank,
    a crank with one pin per cylinder, the pins placed for the even firing order `firing` (cylinder
    numbers from 1: each cylinder's pin sits its firing lag round, 720/n deg apart in firing order, so
    a four gets a flat-plane crank, a three 120 deg and a five 72 deg pins), a see-through crankcase, block, head and a cam cover in material cover_mat (rgba, metallic,
    roughness, blend). meta: more named values for the file. Writes out_file and returns what the
    builder may need for extra parts."""
    donor = Donor(donor_path)
    mats = list(donor.mats) + [cover_mat]
    mat_cover, mat_crank = len(mats) - 1, 2
    throw = stroke / 2.0
    f = donor.fit(bore, throw, rod_l)
    n = len(firing)
    zs = [(-(n - 1) / 2.0 + c) * pitch for c in range(n)]
    lag = [0.0] * n
    for k, cn in enumerate(firing):
        lag[cn - 1] = 720.0 / n * k
    # local crank angle at rest: cylinder 1 at rest_local1 past TDC, each other one its lag behind
    theta_rest = [(rest_local1 - lag[c]) % 360.0 for c in range(n)]
    deck_y = f.deck_y(squish)
    cyls, groups = f.cylinders([(zs[c], theta_rest[c], incline, lag[c]) for c in range(n)], deck_y)

    r_main, r_pin = f.s * 19.2, f.s * 17.0
    skirt_bdc = rod_l - throw - f.skirt_below
    r_cw = skirt_bdc - 6.0          # counterweights clear the piston skirts at BDC, with margin
    pins = [(zs[c], f.rod_half_w + 1.0, incline + theta_rest[c]) for c in range(n)]
    crank, mains, w_lo, w_hi, z_primary = build_crank(pins, throw, r_main, r_pin, r_cw, web, 48.0, 26.0)
    groups.append((CRANK, -1, mat_crank, crank))
    groups += main_shells(mains, r_main)

    case_r = max(r_cw, r_pin + throw + 30.0) + 14.0
    block, cover = build_block([(incline, zs[0], zs[-1])], bore, deck_y, case_r, (w_lo - 50.0, w_hi + 45.0), 100.0,
                               8.0, 82.0, 20.0, 12.0)
    groups.append((BLOCK, -1, 0, block))
    groups.append((BLOCK, -1, mat_cover, cover))

    size = write_engine(out_file, donor.root, mats, cyls, groups,
                        [("banked", [1.0]), primary_meta(z_primary)] + list(meta))
    disp = n * math.pi / 4 * bore ** 2 * stroke / 1000.0
    print("%s from the inline-4 parts" % name)
    print("  bore %.1f  stroke %.1f  rod %.1f  -> %.0f cc   bank leans %.0f deg forward, bore pitch %.0f"
          % (bore, stroke, rod_l, disp, incline, pitch))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (f.s, f.L_s, rod_l, f.k_beam))
    print("  piston crown %.1f above pin, skirt %.1f below; deck at %.1f; counterweight R %.1f (skirt at BDC %.1f)"
          % (f.crown_above, f.skirt_below, -deck_y, r_cw, skirt_bdc))
    print("  crank z %.1f .. %.1f, primary gear at z %.1f" % (w_lo - 48.0, w_hi + 42.0, z_primary))
    for c in range(n):
        print("  cyl %d  z %+6.1f  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, cyls[c][0], math.degrees(cyls[c][6]) % 360, lag[c]))
    print("wrote %s (%.1f MB, %d groups)" % (out_file, size / 1e6, len(groups)))
    return {"deck_y": deck_y, "zs": zs, "z_primary": z_primary, "fit": f}
