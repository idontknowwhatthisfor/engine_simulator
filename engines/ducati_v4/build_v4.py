#!/usr/bin/env python3
"""Build a Ducati-style 90 degree V4 (Desmosedici Stradale layout) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/ducati_v4/build_v4.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/ducati_v4/v4.mesh)

What is reused from the inline-4 (engine.mesh, made by extract_parts.py):
    piston + gudgeon pin, connecting rod + cap bolts/nuts + big-end shell (cylinder 1's copies).
    They are scaled from the 85 mm bore to the Ducati's 81 mm, and the rod beam is shortened
    from 160 mm to ROD_L. Only the beam stretches; the small eye and the big end with its bolts
    stay rigid, so both eyes stay round and the rods still fit their pins.
What is new (procedural):
    a twin-pin crankshaft with the pins 70 degrees apart ("Twin Pulse"), each pin carrying a
    front and a rear rod side by side, three mains, webs with counterweights, alternator rotor and
    primary gear; a see-through crankcase, two cylinder banks, heads and red cam covers.

Layout (mesh space, mm, same conventions as engine.mesh): crank axis = Z through (0, 0), "up" is
-Y (the root matrix flips Y and Z), the front of the bike is -X. The V is 90 degrees and is tilted
back so the front bank sits 42 degrees above horizontal, as on the Desmosedici Stradale. The crank
turns R_z(+delta), which moves the top of the crank backwards: counter-rotating, like Ducati's.

Cylinders: 1 front-left, 2 rear-left (crankpin A), 3 front-right, 4 rear-right (crankpin B).
Firing 1-2-3-4 at 0, 90, 290, 380 degrees: the Twin Pulse order (the sim holds the lags).

v4.mesh = engine.mesh layout with magic "ENGMSHv2", and three more floats per cylinder:
    f32 bank (rad, bore axis = R_z(bank) applied to -Y), f32 deck_y (local frame), f32 bore (mm),
    f32 firing lag (deg of the 720 deg cycle behind cylinder 1)
Cylinder pivots (gudgeon, crankpin, theta0) and the vertices of piston and rod groups are stored in
the cylinder's local frame (bore toward -Y). The renderer rotates them by `bank` about the crank
axis. Crank and block groups are stored in mesh space.
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import (STATIC, CRANK, PISTON, ROD, BLOCK, Mesh, read_i4, norm, convex_hull, prism,  # noqa: E402
                     circle, cylinder, box, rot_z)

# ---- Ducati Desmosedici Stradale (Panigale V4) numbers --------------------------------------
BORE = 81.0
STROKE = 53.5
THROW = STROKE / 2.0
ROD_L = 110.0           # estimate, rod/stroke ~2.06 (Ducati does not publish it)
V_ANGLE = 90.0
FRONT_ELEV = 42.0       # front bank angle above horizontal
PIN_SPLIT = 70.0        # crankpin B leads A by this much
PIN_GAP = 100.0         # crankpin A to crankpin B along the crank (= cylinder pitch in a bank)
ROD_PITCH = 24.0        # front and rear big ends side by side on one pin
REST_LOCAL1 = 30.0      # rest pose: cylinder 1 is 30 deg past TDC
SQUISH = 1.2            # crown to deck at TDC

# bank angles: bore axis = R_z(bank) * (0, -1). front points forward-up, rear 90 deg behind it
BANK_FRONT = -(90.0 - FRONT_ELEV)
BANK_REAR = BANK_FRONT + V_ANGLE

# extra materials appended to the inline-4 ones
RED_GLASS = (0.55, 0.03, 0.03, 0.30, 0.0, 0.5, 1)   # cam covers, see-through like the block


def pin_xy(angle_deg, r=THROW):
    """crankpin position at an angle measured from -Y towards +X (the R_z sense)."""
    a = math.radians(angle_deg)
    return (r * math.sin(a), -r * math.cos(a))


# ---- the build ---------------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "v4.mesh")
    root, mats, axis, cyls, groups = read_i4(src)
    mats = list(mats) + [RED_GLASS]
    MAT_RED = len(mats) - 1
    MAT_CRANK = 2

    # inline-4 cylinder 1 is the donor
    z_c, gx, gy, cx, cy, r_i4, th_i4, L_i4 = cyls[0]
    s = BORE / 85.0  # the inline-4 bore, see extract_parts.py
    ux, uy = (cx - gx) / L_i4, (cy - gy) / L_i4

    piston_parts = [(m, v, i) for k, c, m, v, i in groups if k == PISTON and c == 0]
    rod_parts = [(m, v, i) for k, c, m, v, i in groups if k == ROD and c == 0]
    assert len(piston_parts) >= 2 and len(rod_parts) >= 2

    # piston crown / skirt relative to the gudgeon pin (scaled), from the piston body (largest part)
    body = max(piston_parts, key=lambda t: len(t[1]))[1]
    ys = body[1::6]
    crown_above = s * (gy - min(ys))
    skirt_below = s * (max(ys) - gy)

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
        """gudgeon and crankpin centres in the local frame at local crank angle theta."""
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
        d = ((pin[0] - g[0]) / ROD_L, (pin[1] - g[1]) / ROD_L)  # small end -> big end
        e = (d[1], -d[0])                                         # rod's sideways axis
        out = []
        for m, v, idx in rod_parts:
            mesh = Mesh()
            mesh.i = list(idx)
            for k in range(0, len(v), 6):
                # donor rod frame: b sideways, a along the rod from the small end
                px, py = v[k] - gx, v[k + 1] - gy
                b = s * (uy * px - ux * py)
                a = s * (ux * px + uy * py)
                a2, kk = stretch(a)
                nx, ny = v[k + 3], v[k + 4]
                nb = uy * nx - ux * ny
                na = (ux * nx + uy * ny) / kk   # normals of a stretched surface tilt the other way
                nz = v[k + 5]
                ln = math.sqrt(nb * nb + na * na + nz * nz) or 1.0
                nb, na, nz = nb / ln, na / ln, nz / ln
                p = (g[0] + b * e[0] + a2 * d[0], g[1] + b * e[1] + a2 * d[1], z_new + s * (v[k + 2] - z_c))
                n = (nb * e[0] + na * d[0], nb * e[1] + na * d[1], nz)
                mesh.v += list(p) + list(n)
            out.append((m, mesh))
        return out

    # ---- cylinders ---------------------------------------------------------------------------
    pin_z = {"A": -PIN_GAP / 2.0, "B": PIN_GAP / 2.0}
    rest_pin = {"A": BANK_FRONT + REST_LOCAL1}           # mesh angle of pin A at rest
    rest_pin["B"] = rest_pin["A"] + PIN_SPLIT            # B leads: it reaches TDC 70 deg earlier
    layout = [  # (name, pin, bank, z offset on the pin, firing lag behind cylinder 1)
        ("front-left", "A", BANK_FRONT, -ROD_PITCH / 2.0, 0.0),
        ("rear-left", "A", BANK_REAR, +ROD_PITCH / 2.0, V_ANGLE),
        ("front-right", "B", BANK_FRONT, -ROD_PITCH / 2.0, 360.0 - PIN_SPLIT),
        ("rear-right", "B", BANK_REAR, +ROD_PITCH / 2.0, 360.0 - PIN_SPLIT + V_ANGLE),
    ]
    deck_y = -(THROW + ROD_L + crown_above + SQUISH)

    out_cyls, out_groups = [], []
    for c, (name, pin, bank, dz, lag) in enumerate(layout):
        z_new = pin_z[pin] + dz
        theta = rest_pin[pin] - bank                       # local crank angle at rest
        g, p = slider(theta)
        out_cyls.append((z_new, g[0], g[1], p[0], p[1], THROW, math.radians(theta), ROD_L,
                         math.radians(bank), deck_y, BORE, lag))
        for m, mesh in place_piston(theta, z_new):
            out_groups.append((PISTON, c, m, mesh))
        for m, mesh in place_rod(theta, z_new):
            out_groups.append((ROD, c, m, mesh))

    # ---- crankshaft (mesh space) ----------------------------------------------------------------
    R_MAIN = s * 19.2
    R_PIN = s * 17.0
    WEB = 14.0
    # counterweights must clear the piston skirts at BDC, with margin
    skirt_bdc = ROD_L - THROW - skirt_below
    R_CW = skirt_bdc - 6.0
    crank = Mesh()
    zA, zB = pin_z["A"], pin_z["B"]
    half_pin = ROD_PITCH  # two rods per pin
    webs = []  # (z0, z1, pin angle)
    for zp, key in ((zA, "A"), (zB, "B")):
        webs.append((zp - half_pin - WEB, zp - half_pin, rest_pin[key]))
        webs.append((zp + half_pin, zp + half_pin + WEB, rest_pin[key]))
        px, py = pin_xy(rest_pin[key])
        crank.add(cylinder(px, py, R_PIN, zp - half_pin, zp + half_pin, 40))
    for z0, z1, ang in webs:
        px, py = pin_xy(ang)
        cw = math.radians(ang + 180.0)
        # pin boss, a hub round the main, and a counterweight arc opposite the pin
        pts = circle(px, py, R_PIN + 7.0, 40) + circle(0.0, 0.0, R_MAIN + 9.0, 40)
        a_mid = math.atan2(-math.cos(cw), math.sin(cw))  # pin_xy convention -> atan2 angle
        pts += circle(0.0, 0.0, R_CW, 40, a_mid - math.radians(65), a_mid + math.radians(65))
        crank.add(prism(convex_hull(pts), z0, z1))
    w_lo, w_hi = webs[0][0], webs[-1][1]
    mid0, mid1 = webs[1][1], webs[2][0]
    crank.add(cylinder(0, 0, R_MAIN, mid0, mid1))                      # centre main
    crank.add(cylinder(0, 0, R_MAIN, w_lo - 20.0, w_lo))               # left main
    crank.add(cylinder(0, 0, R_MAIN, w_hi, w_hi + 20.0))               # right main
    crank.add(cylinder(0, 0, R_MAIN - 4.0, w_lo - 48.0, w_lo - 20.0))  # left stub
    crank.add(cylinder(0, 0, 50.0, w_lo - 46.0, w_lo - 26.0, 64))      # alternator rotor
    crank.add(cylinder(0, 0, R_MAIN - 4.0, w_hi + 20.0, w_hi + 42.0))  # right stub
    crank.add(cylinder(0, 0, 30.0, w_hi + 24.0, w_hi + 38.0, 72))      # primary gear hub (teeth: the sim)
    out_groups.append((CRANK, -1, MAT_CRANK, crank))

    # main bearing shells (static, bronze like the inline-4's)
    for z0, z1 in ((mid0 + 1, mid1 - 1), (w_lo - 19, w_lo - 1), (w_hi + 1, w_hi + 19)):
        out_groups.append((STATIC, -1, 1, cylinder(0, 0, R_MAIN + 2.5, z0, z1)))

    # ---- block (see-through), cylinder banks in their local frame then rotated -------------------
    block = Mesh()
    case_r = max(R_CW, R_PIN + THROW + 30.0) + 14.0
    case_z0, case_z1 = w_lo - 50.0, w_hi + 45.0
    sump = circle(0, 0, case_r, 64) + [(-case_r + 8, 110.0), (case_r - 8, 110.0)]
    block.add(prism(convex_hull(sump), case_z0, case_z1))
    red = Mesh()
    for bank in (BANK_FRONT, BANK_REAR):
        zs = [cy_[0] for cy_ in out_cyls if abs(math.degrees(cy_[8]) - bank) < 1e-6]
        z0, z1 = min(zs) - BORE / 2 - 9.0, max(zs) + BORE / 2 + 9.0
        fp, fn = rot_z(bank)
        barrel = box((-BORE / 2 - 9.0, deck_y, z0), (BORE / 2 + 9.0, -case_r + 12.0, z1))
        head = box((-BORE / 2 - 18.0, deck_y - 78.0, z0 - 3.0), (BORE / 2 + 18.0, deck_y, z1 + 3.0))
        cover = box((-BORE / 2 - 10.0, deck_y - 98.0, z0 + 4.0), (BORE / 2 + 10.0, deck_y - 78.0, z1 - 4.0))
        block.add(barrel.transformed(fp, fn))
        block.add(head.transformed(fp, fn))
        red.add(cover.transformed(fp, fn))
    out_groups.append((BLOCK, -1, 0, block))
    out_groups.append((BLOCK, -1, MAT_RED, red))

    # ---- report ---------------------------------------------------------------------------------
    disp = 4 * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("Ducati-style V4 from the inline-4 parts")
    print("  bore %.1f  stroke %.1f  rod %.1f  -> %.0f cc   V %.0f deg, front bank %.0f deg above horizontal"
          % (BORE, STROKE, ROD_L, disp, V_ANGLE, FRONT_ELEV))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (s, L_s, ROD_L, k_beam))
    print("  piston crown %.1f above pin, skirt %.1f below; deck at %.1f; counterweight R %.1f (skirt at BDC %.1f)"
          % (crown_above, skirt_below, -deck_y, R_CW, skirt_bdc))
    print("  crankpins %.0f deg apart at z %.0f / %.0f, rods %.0f mm apart on each pin" % (PIN_SPLIT, zA, zB, ROD_PITCH))
    for c, (name, pin, bank, dz, lag) in enumerate(layout):
        cy_ = out_cyls[c]
        print("  cyl %d %-11s pin %s  bank %+5.1f deg  z %+6.1f  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, name, pin, bank, cy_[0], math.degrees(cy_[6]) % 360, lag))

    # ---- write ----------------------------------------------------------------------------------
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


if __name__ == "__main__":
    main()
