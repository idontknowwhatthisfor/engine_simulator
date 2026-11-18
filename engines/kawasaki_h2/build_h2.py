#!/usr/bin/env python3
"""Build a Kawasaki Ninja H2 style supercharged inline-4 from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/kawasaki_h2/build_h2.py [i4_mesh] [out_file] [supercharger_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/kawasaki_h2/h2.mesh, h2_supercharger.mesh)

The engine is built like engines/bmw_s1000rr/build_s1000rr.py (same donor parts, procedural crank,
crankcase, block and head): cylinder 1's piston and rod from the inline-4, scaled to the H2's 76 mm
bore with the rod beam shortened.

Kawasaki Ninja H2 (2015-): 998 cc, 76.0 x 55.0 mm, 8.5:1, 180 deg crank, firing 1-2-4-3, forward
crank rotation. Estimates (not published): rod length 107 mm, bore pitch 84 mm, cylinders inclined
30 deg forward.

h2.mesh: the v2 engine layout (magic "ENGMSHv2"). Mesh space as for the S 1000 RR: crank axis = Z,
"up" is -Y, front of the bike +X, left side -Z.

h2_supercharger.mesh: the centrifugal supercharger in the "TURBOv1" layout of
engines/inline4_fit/build_turbo.py, so src/turbo.hpp draws it and spins the impeller. It sits behind
the cylinders on the left, above the gearbox, its impeller axis across the bike, the inlet facing
left. Its frame: origin SC_AT (mesh space) at the impeller's back plate, +x along the shaft out of
the inlet (mesh -Z), y = mesh Y, z = mesh +X (forward). kawasaki_h2.hpp places it with the same
SC_AT. Parts:
    - impeller (rotor): 69 mm, 6 full blades and 6 splitters on a hub, turning at 9.2x the crank;
    - volute, inlet bellmouth, ram-air duct to the front of the fairing, outlet into the plenum
      (surge tank) over the throttle bodies, and the throttle bodies: housings, see-through;
    - shaft and the planetary step-up drive behind the impeller (bearing, not turning).
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import (STATIC, CRANK, PISTON, ROD, BLOCK, Mesh, read_i4, convex_hull, prism,  # noqa: E402
                      circle, cylinder, box, rot_z, norm)

# ---- Kawasaki Ninja H2 numbers -------------------------------------------------------------------
BORE = 76.0
STROKE = 55.0
THROW = STROKE / 2.0
ROD_L = 107.0           # estimate
PITCH = 84.0            # bore centre spacing (estimate: 76 mm bore + 8 mm between liners)
INCLINE = 30.0          # cylinder bank leans forward (toward +X) by this much (estimate)
FIRING = (1, 2, 4, 3)
REST_LOCAL1 = 30.0      # rest pose: cylinder 1 is 30 deg past TDC
SQUISH = 1.0            # crown to deck at TDC

COVER_GLASS = (0.10, 0.11, 0.12, 0.35, 0.0, 0.35, 1)   # dark graphite cam cover, see-through like the block

# supercharger frame origin in mesh space (mm): impeller back plate. Keep in step with H2_SC_AT in
# kawasaki_h2.hpp
SC_AT = (-165.0, -195.0, -95.0)
IMPELLER_R = 34.5       # 69 mm impeller
HOUSING, ROTOR, BEARING = range(3)
GHOST = ((0.3662526, 0.3564001, 0.3324515, 0.25), 0.65, 0.0)   # the inline-4 block's see-through material
ALU = ((0.80, 0.80, 0.82, 1.0), 0.25, 0.95)                     # machined impeller
STEEL = ((0.45, 0.45, 0.48, 1.0), 0.35, 0.9)


def pin_xy(angle_deg, r=THROW):
    """crankpin position at an angle measured from -Y towards +X (the R_z sense)."""
    a = math.radians(angle_deg)
    return (r * math.sin(a), -r * math.cos(a))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "h2.mesh")
    sc_file = sys.argv[3] if len(sys.argv) > 3 else os.path.join(here, "h2_supercharger.mesh")
    root, mats, axis, cyls, groups = read_i4(src)
    mats = list(mats) + [COVER_GLASS]
    MAT_COVER = len(mats) - 1
    MAT_CRANK = 2

    z_c, gx, gy, cx, cy, r_i4, th_i4, L_i4 = cyls[0]
    s = BORE / 85.0
    ux, uy = (cx - gx) / L_i4, (cy - gy) / L_i4

    piston_parts = [(m, v, i) for k, c, m, v, i in groups if k == PISTON and c == 0]
    rod_parts = [(m, v, i) for k, c, m, v, i in groups if k == ROD and c == 0]
    assert len(piston_parts) >= 2 and len(rod_parts) >= 2

    body = max(piston_parts, key=lambda t: len(t[1]))[1]
    ys = body[1::6]
    crown_above = s * (gy - min(ys))
    skirt_below = s * (max(ys) - gy)
    rod_half_w = s * max(abs(z - z_c) for _, v, _ in rod_parts for z in v[2::6])

    # rod beam stretch: rigid small eye (a < A_S) and big end (a > A_B), linear in between
    L_s = s * L_i4
    A_S, A_B = 19.0, L_s - 44.0
    shrink = L_s - ROD_L
    assert ROD_L - 44.0 > A_S + 5.0, "rod too short for the donor's eyes"
    k_beam = (A_B - shrink - A_S) / (A_B - A_S)

    def stretch(a):
        if a <= A_S:
            return a, 1.0
        if a >= A_B:
            return a - shrink, 1.0
        return A_S + (a - A_S) * k_beam, k_beam

    def slider(theta_deg):
        t = math.radians(theta_deg)
        pin = (THROW * math.sin(t), -THROW * math.cos(t))
        gyy = -(THROW * math.cos(t) + math.sqrt(ROD_L ** 2 - (THROW * math.sin(t)) ** 2))
        return (0.0, gyy), pin

    def place_piston(theta_deg, z_new):
        g, _ = slider(theta_deg)
        out = []
        for m, v, idx in piston_parts:
            mesh = Mesh()
            mesh.i = list(idx)
            for k in range(0, len(v), 6):
                p = (g[0] + s * (v[k] - gx), g[1] + s * (v[k + 1] - gy), z_new + s * (v[k + 2] - z_c))
                mesh.v += list(p) + list(v[k + 3:k + 6])
            out.append((m, mesh))
        return out

    def place_rod(theta_deg, z_new):
        g, pin = slider(theta_deg)
        d = ((pin[0] - g[0]) / ROD_L, (pin[1] - g[1]) / ROD_L)
        e = (d[1], -d[0])
        out = []
        for m, v, idx in rod_parts:
            mesh = Mesh()
            mesh.i = list(idx)
            for k in range(0, len(v), 6):
                px, py = v[k] - gx, v[k + 1] - gy
                b = s * (uy * px - ux * py)
                a = s * (ux * px + uy * py)
                a2, kk = stretch(a)
                nx, ny = v[k + 3], v[k + 4]
                nb = uy * nx - ux * ny
                na = (ux * nx + uy * ny) / kk
                nz = v[k + 5]
                ln = math.sqrt(nb * nb + na * na + nz * nz) or 1.0
                nb, na, nz = nb / ln, na / ln, nz / ln
                p = (g[0] + b * e[0] + a2 * d[0], g[1] + b * e[1] + a2 * d[1], z_new + s * (v[k + 2] - z_c))
                n = (nb * e[0] + na * d[0], nb * e[1] + na * d[1], nz)
                mesh.v += list(p) + list(n)
            out.append((m, mesh))
        return out

    # ---- cylinders: 1..4 from left to right, flat-plane crank, firing 1-2-4-3 ------------------
    bank = INCLINE
    zs = [(-1.5 + c) * PITCH for c in range(4)]
    # local crank angle at rest: 1 and 4 share a crank position, 2 and 3 are 180 deg away
    theta_rest = [REST_LOCAL1, REST_LOCAL1 + 180.0, REST_LOCAL1 + 180.0, REST_LOCAL1]
    lag = [0.0] * 4
    for k, cn in enumerate(FIRING):
        lag[cn - 1] = 180.0 * k
    deck_y = -(THROW + ROD_L + crown_above + SQUISH)

    out_cyls, out_groups = [], []
    for c in range(4):
        theta = theta_rest[c]
        g, p = slider(theta)
        out_cyls.append((zs[c], g[0], g[1], p[0], p[1], THROW, math.radians(theta), ROD_L,
                         math.radians(bank), deck_y, BORE, lag[c]))
        for m, mesh in place_piston(theta, zs[c]):
            out_groups.append((PISTON, c, m, mesh))
        for m, mesh in place_rod(theta, zs[c]):
            out_groups.append((ROD, c, m, mesh))

    # ---- crankshaft (mesh space) ----------------------------------------------------------------
    R_MAIN = s * 19.2
    R_PIN = s * 17.0
    WEB = 12.0
    half_pin = rod_half_w + 1.0
    skirt_bdc = ROD_L - THROW - skirt_below
    R_CW = skirt_bdc - 6.0
    crank = Mesh()
    webs = []
    for c in range(4):
        ang = bank + theta_rest[c]          # pin angle in mesh space
        zp = zs[c]
        px, py = pin_xy(ang)
        crank.add(cylinder(px, py, R_PIN, zp - half_pin, zp + half_pin, 40))
        webs.append((zp - half_pin - WEB, zp - half_pin, ang))
        webs.append((zp + half_pin, zp + half_pin + WEB, ang))
    for z0, z1, ang in webs:
        px, py = pin_xy(ang)
        cw = math.radians(ang + 180.0)
        pts = circle(px, py, R_PIN + 7.0, 40) + circle(0.0, 0.0, R_MAIN + 9.0, 40)
        a_mid = math.atan2(-math.cos(cw), math.sin(cw))
        pts += circle(0.0, 0.0, R_CW, 40, a_mid - math.radians(65), a_mid + math.radians(65))
        crank.add(prism(convex_hull(pts), z0, z1))
    w_lo, w_hi = webs[0][0], webs[-1][1]
    mains = [(w_lo - 20.0, w_lo)]
    for c in range(3):
        mains.append((webs[2 * c + 1][1], webs[2 * c + 2][0]))
    mains.append((w_hi, w_hi + 20.0))
    for z0, z1 in mains:
        crank.add(cylinder(0, 0, R_MAIN, z0, z1))
    crank.add(cylinder(0, 0, R_MAIN - 4.0, w_lo - 48.0, w_lo - 20.0))  # left stub
    crank.add(cylinder(0, 0, 48.0, w_lo - 46.0, w_lo - 26.0, 64))      # alternator rotor
    crank.add(cylinder(0, 0, R_MAIN - 4.0, w_hi + 20.0, w_hi + 42.0))  # right stub
    crank.add(cylinder(0, 0, 26.0, w_hi + 24.0, w_hi + 38.0, 72))      # primary gear hub (teeth: the sim)
    z_primary = w_hi + 31.0
    out_groups.append((CRANK, -1, MAT_CRANK, crank))

    for z0, z1 in mains:  # main bearing shells
        out_groups.append((STATIC, -1, 1, cylinder(0, 0, R_MAIN + 2.5, z0 + 1, z1 - 1)))

    # ---- block (see-through) -------------------------------------------------------------------
    block = Mesh()
    case_r = max(R_CW, R_PIN + THROW + 30.0) + 14.0
    case_z0, case_z1 = w_lo - 50.0, w_hi + 45.0
    sump = circle(0, 0, case_r, 64) + [(-case_r + 8, 100.0), (case_r - 8, 100.0)]
    block.add(prism(convex_hull(sump), case_z0, case_z1))
    z0, z1 = zs[0] - BORE / 2 - 8.0, zs[-1] + BORE / 2 + 8.0
    fp, fn = rot_z(bank)
    barrel = box((-BORE / 2 - 9.0, deck_y, z0), (BORE / 2 + 9.0, -case_r + 12.0, z1))
    head = box((-BORE / 2 - 20.0, deck_y - 82.0, z0 - 3.0), (BORE / 2 + 20.0, deck_y, z1 + 3.0))
    cover = box((-BORE / 2 - 12.0, deck_y - 102.0, z0 + 4.0), (BORE / 2 + 12.0, deck_y - 82.0, z1 - 4.0))
    block.add(barrel.transformed(fp, fn))
    block.add(head.transformed(fp, fn))
    blue = cover.transformed(fp, fn)
    out_groups.append((BLOCK, -1, 0, block))
    out_groups.append((BLOCK, -1, MAT_COVER, blue))

    # ---- report ---------------------------------------------------------------------------------
    disp = 4 * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("Kawasaki Ninja H2 style inline-4 from the inline-4 parts")
    print("  bore %.1f  stroke %.1f  rod %.1f  -> %.0f cc   bank leans %.0f deg forward, bore pitch %.0f"
          % (BORE, STROKE, ROD_L, disp, INCLINE, PITCH))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (s, L_s, ROD_L, k_beam))
    print("  piston crown %.1f above pin, skirt %.1f below; deck at %.1f; counterweight R %.1f (skirt at BDC %.1f)"
          % (crown_above, skirt_below, -deck_y, R_CW, skirt_bdc))
    print("  crank z %.1f .. %.1f, primary gear at z %.1f" % (w_lo - 48.0, w_hi + 42.0, z_primary))
    for c in range(4):
        cy_ = out_cyls[c]
        print("  cyl %d  z %+6.1f  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, cy_[0], math.degrees(cy_[6]) % 360, lag[c]))

    out = bytearray(b"ENGMSHv2")
    out += struct.pack("<16f", *root)
    out += struct.pack("<I", len(mats))
    for m in mats:
        out += struct.pack("<4f2fI", *m)
    out += struct.pack("<3f", 0.0, 0.0, 0.0)
    out += struct.pack("<I", len(out_cyls))
    for cy_ in out_cyls:
        out += struct.pack("<12f", *cy_)
    out += struct.pack("<I", len(out_groups))
    for kind, cyl, mat, mesh in out_groups:
        out += struct.pack("<IiIII", kind, cyl, mat, len(mesh.v) // 6, len(mesh.i))
        out += struct.pack("<%df" % len(mesh.v), *mesh.v)
        out += struct.pack("<%dI" % len(mesh.i), *mesh.i)
    with open(out_file, "wb") as f:
        f.write(out)
    print("wrote %s (%.1f MB, %d groups)" % (out_file, len(out) / 1e6, len(out_groups)))
    build_supercharger(sc_file, deck_y, zs)


# ---- supercharger (TURBOv1) -------------------------------------------------------------------------
def to_sc(p):
    """mesh space -> supercharger frame."""
    r = (p[0] - SC_AT[0], p[1] - SC_AT[1], p[2] - SC_AT[2])
    return (-r[2], r[1], r[0])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def tube(path, radii, seg=28, caps=False):
    """circle swept along a polyline (parallel-transport frames), outward normals."""
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


def blade(theta0, x_hi=26.0, steps=10, thick=0.9):
    """one impeller blade in the supercharger frame (axis +x): from the back plate (x = 0, out to
    the tip) to x_hi (the inducer at 26; a splitter stops short); it leans back toward the tip and twists at the inducer."""
    faces = []
    secs = []
    for k in range(steps + 1):
        x = x_hi * k / steps
        f = x / 26.0
        r_hub = 9.0 + 16.0 * (1.0 - f) ** 2               # hub flares out to the back plate
        r_tip = IMPELLER_R - 10.0 * f ** 1.5               # shroud line: tip at the back, inducer eye
        r_hub = min(r_hub, r_tip - 2.0)
        th = theta0 + 0.55 * f ** 2                        # inducer twist
        lean = -0.35                                       # backsweep toward the tip
        pts = []
        for r, side in ((r_hub, -1), (r_tip, -1), (r_tip, 1), (r_hub, 1)):
            a = th + lean * (r - r_hub) / IMPELLER_R + side * thick / max(r, 1.0)
            pts.append((x, r * math.cos(a), r * math.sin(a)))
        secs.append(pts)
    for k in range(steps):
        p, q = secs[k], secs[k + 1]
        for j in range(4):
            a, b, c, d = p[j], p[(j + 1) % 4], q[(j + 1) % 4], q[j]
            faces += [(a, b, c), (a, c, d)]
    for p, flip in ((secs[0], True), (secs[-1], False)):
        if flip:
            faces += [(p[0], p[2], p[1]), (p[0], p[3], p[2])]
        else:
            faces += [(p[0], p[1], p[2]), (p[0], p[2], p[3])]
    return solid(faces)


def build_supercharger(out_file, deck_y, zs):
    groups = []  # (kind, look, Mesh) in the supercharger frame

    # impeller: back plate, hub, 6 full blades and 6 splitters (start further out on the hub)
    imp = Mesh()
    imp.add(tube([(-3.0, 0, 0), (0.0, 0, 0)], [IMPELLER_R, IMPELLER_R], 48, caps=True))
    hub_path = [(0.0 + 26.0 * k / 8, 0, 0) for k in range(9)]
    imp.add(tube(hub_path, [9.0 + 16.0 * (1.0 - k / 8) ** 2 for k in range(9)], 32, caps=True))
    imp.add(tube([(26.0, 0, 0), (31.0, 0, 0)], [6.0, 4.0], 16, caps=True))   # nut
    for b in range(6):
        imp.add(blade(2 * math.pi * b / 6))
        imp.add(blade(2 * math.pi * (b + 0.5) / 6, x_hi=14.0, steps=6))   # splitter
    groups.append((ROTOR, ALU, imp))

    # shaft and the planetary step-up drive behind the impeller (toward the engine, -x)
    drv = Mesh()
    drv.add(tube([(-3.0, 0, 0), (-45.0, 0, 0)], [7.0, 7.0], 20, caps=True))
    drv.add(tube([(-40.0, 0, 0), (-62.0, 0, 0)], [38.0, 38.0], 48, caps=True))
    drv.add(tube([(-62.0, 0, 0), (-80.0, 0, 0)], [14.0, 14.0], 24, caps=True))  # input from the crank's gear train
    groups.append((BEARING, STEEL, drv))

    # housings (see-through)
    hs = Mesh()
    # back plate of the volute and the impeller shroud
    hs.add(tube([(-10.0, 0, 0), (-4.0, 0, 0)], [62.0, 62.0], 56, caps=True))
    hs.add(tube([(-4.0, 0, 0), (8.0, 0, 0), (20.0, 0, 0), (30.0, 0, 0)], [IMPELLER_R + 3, IMPELLER_R + 1, 28.0, 27.0], 48))
    # volute: a spiral round the impeller whose section grows toward the outlet
    vol = []
    rad = []
    turns = 40
    for k in range(turns + 1):
        f = k / turns
        a = 2 * math.pi * f
        rr = 44.0 + 12.0 * f
        vol.append((2.0, rr * math.cos(a), rr * math.sin(a)))
        rad.append(7.0 + 11.0 * f)
    # outlet: leaves the volute tangentially and rises to the plenum
    end = vol[-1]
    vol += [(2.0, end[1] + 6.0, end[2] + 30.0)]
    rad += [rad[-1]]
    hs.add(tube(vol, rad, 24))
    # inlet bellmouth and the ram-air duct: out to the left (+x), then forward (+z) to the fairing
    inlet = [(30.0, 0, 0), (60.0, 0, 0), (95.0, -10.0, 20.0), (110.0, -35.0, 90.0), (112.0, -55.0, 200.0),
             (110.0, -60.0, 330.0), (100.0, -40.0, 430.0)]
    hs.add(tube(inlet, [27.0, 32.0, 36.0, 38.0, 40.0, 42.0, 48.0], 32))
    groups.append((HOUSING, GHOST, hs))

    # plenum (surge tank) over the throttle bodies, behind the inclined head, and the throttle bodies
    # from it to each intake port (mesh space, converted)
    c, s = math.cos(math.radians(INCLINE)), math.sin(math.radians(INCLINE))

    def bank(lx, ly, z):
        return (c * lx - s * ly, s * lx + c * ly, z)
    pl = Mesh()
    tb = []
    for zc in zs:
        port = bank(-(BORE / 2 + 18.0), deck_y - 41.0, zc)
        top = (port[0] - 80.0, port[1] - 95.0, zc)
        tb.append(top)
        pl.add(tube([to_sc(port), to_sc(((port[0] + top[0]) / 2, (port[1] + top[1]) / 2 - 10, zc)), to_sc(top)],
                    [17.0, 20.0, 22.0], 24))
    x_mid = sum(p[0] for p in tb) / 4 - 15.0
    y_mid = sum(p[1] for p in tb) / 4 - 25.0
    zlo, zhi = zs[0] - 70.0, zs[-1] + 70.0
    pc = box((x_mid - 85.0, y_mid - 45.0, zlo), (x_mid + 70.0, y_mid + 40.0, zhi))
    pl.add(pc.transformed(to_sc, lambda n: (-n[2], n[1], n[0])))
    # the volute outlet duct climbs into the plenum's rear underside
    out_sc = vol[-1]
    entry = to_sc((x_mid - 60.0, y_mid + 35.0, SC_AT[2] + 5.0))
    pl.add(tube([out_sc, (out_sc[0], out_sc[1] - 30.0, out_sc[2] + 10.0), entry], [18.0, 20.0, 22.0], 24))
    groups.append((HOUSING, GHOST, pl))

    out = bytearray(b"TURBOv1\0")
    out += struct.pack("<I", len(groups))
    for kind, (rgba, rough, metal), mesh in groups:
        out += struct.pack("<I4f2fII", kind, *rgba, rough, metal, len(mesh.v) // 6, len(mesh.i))
        out += struct.pack("<%df" % len(mesh.v), *mesh.v)
        out += struct.pack("<%dI" % len(mesh.i), *mesh.i)
    with open(out_file, "wb") as f:
        f.write(out)
    print("  supercharger at (%.0f, %.0f, %.0f), plenum x %.0f y %.0f; wrote %s"
          % (SC_AT + (x_mid, y_mid, out_file)))


if __name__ == "__main__":
    main()
