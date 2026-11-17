#!/usr/bin/env python3
"""Build a BMW S 1000 RR style inline-4 (2019+ ShiftCam engine) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/bmw_s1000rr/build_s1000rr.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/bmw_s1000rr/s1000rr.mesh)

Reused from the inline-4 (engine.mesh): cylinder 1's piston + gudgeon pin and connecting rod + cap
bolts/nuts + big-end shell, scaled from the 85 mm bore to BMW's 80 mm, rod beam shortened to 103 mm
(the same rigid-eye stretch as engines/ducati_v4/build_v4.py). New (procedural): a forged flat-plane (180 deg)
crank with four throws, five mains and counterweights, alternator rotor on the left and the primary
gear on the right; a see-through crankcase, one-piece cylinder block, head and a blue cam cover.

BMW S 1000 RR (K67, 2019-): 999 cc, 80.0 x 49.7 mm, 13.3:1, 103 mm rods, 180 deg crank, cylinders
inclined 32 deg forward, firing order 1-2-4-3, conventional (forward) crank rotation.

Layout (mesh space, mm, the engine.mesh conventions): crank axis = Z through (0, 0), "up" is -Y.
Unlike the V4 the front of the bike is +X here: the crank turns R_z(+delta), which moves the top of
the crank toward +X, so with the front at +X the crank turns forward like the wheels. The block leans
forward (bank +32 deg) and the simulator puts the gearbox behind the crank (-X).

s1000rr.mesh = the v2 layout of engines/ducati_v4/build_v4.py with the magic "BMWS1KRR".
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import (STATIC, CRANK, PISTON, ROD, BLOCK, Mesh, read_i4, convex_hull, prism,  # noqa: E402
                      circle, cylinder, box, rot_z)

# ---- BMW S 1000 RR numbers --------------------------------------------------------------------
BORE = 80.0
STROKE = 49.7
THROW = STROKE / 2.0
ROD_L = 103.0           # BMW: "103 mm connecting rods"
PITCH = 92.0            # bore centre spacing (estimate: 80 mm bore + 12 mm between liners)
INCLINE = 32.0          # cylinder bank leans forward (toward +X) by this much
FIRING = (1, 2, 4, 3)
REST_LOCAL1 = 30.0      # rest pose: cylinder 1 is 30 deg past TDC
SQUISH = 1.0            # crown to deck at TDC

BLUE_GLASS = (0.05, 0.22, 0.62, 0.30, 0.0, 0.5, 1)   # cam cover, see-through like the block


def pin_xy(angle_deg, r=THROW):
    """crankpin position at an angle measured from -Y towards +X (the R_z sense)."""
    a = math.radians(angle_deg)
    return (r * math.sin(a), -r * math.cos(a))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "s1000rr.mesh")
    root, mats, axis, cyls, groups = read_i4(src)
    mats = list(mats) + [BLUE_GLASS]
    MAT_BLUE = len(mats) - 1
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
    out_groups.append((BLOCK, -1, MAT_BLUE, blue))

    # ---- report ---------------------------------------------------------------------------------
    disp = 4 * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("BMW S 1000 RR style inline-4 from the inline-4 parts")
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

    out = bytearray(b"BMWS1KRR")
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


if __name__ == "__main__":
    main()
