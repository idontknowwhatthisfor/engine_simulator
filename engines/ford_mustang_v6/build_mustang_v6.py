#!/usr/bin/env python3
"""Build a 2014 Ford Mustang V6 style 60 degree V6 (the 3.7 L Cyclone, "Duratec 37") from the inline-4
parts.

Standard library only. Usage:
    python3 -I engines/ford_mustang_v6/build_mustang_v6.py [i4_mesh] [out_file] [blower_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/ford_mustang_v6/mustang_v6.mesh and
    mustang_v6_blower.mesh)

mustang_v6_blower.mesh: the tuned car's twin-screw supercharger in the "TURBOv1" layout of
src/turbo.hpp, in its own frame (rotor axis +x, toward the gearbox; the pulley at -x, the front):
    - male rotor (ROTOR_A, turning about its own axis): 4 helical lobes, with the drive pulley on its front end;
    - female rotor (ROTOR_B): 6 helical flutes of the other hand, turning the other way at 4/6 speed;
    - housing (see-through): the figure-8 rotor case, end plates and the outlet to the manifold;
    - belt (static): the 8-rib belt from the crank damper to the blower pulley.
Its frame origin (BLOWER_AT, mesh space) is written into mustang_v6.mesh as supercharger_at.

As the LFA (engines/lexus_lfa/build_lfa.py): the inline-4's piston and rod, scaled to the 95.5 mm bore
and the rod length, on a procedural crank, crankcase, two banks, heads and cam covers (tools/meshlib.py's
Donor and build_block). The crank is built here: a 60 deg V6 fires evenly (every 120 deg) only with
split crankpins, each pair's two halves 60 deg apart with a thin web between them, so meshlib's
build_crank (one shared pin per pair) does not fit.

Cyclone 3.7: 3726 cc, 95.5 x 86.7 mm, 10.5:1, 60 deg V, four main bearings, firing order 1-4-2-5-3-6.
Ford numbering: cylinders 1-3 the right bank front to back, 4-6 the left bank; the right bank sits
forward of the left by the rod offset on each split pin. Estimates (Ford does not publish them in the
owner literature): rod 150 mm, bore spacing 108 mm.

Layout (mesh space, mm, the car conventions of engines/audi_rs3/): crank axis = Z through (0, 0), "up"
is -Y, the front of the car at -Z, the gearbox behind the flywheel at +Z. The V stands upright, each
bank 30 deg off vertical: the right bank (1-3) toward -X, the left bank (4-6) toward +X.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
import struct  # noqa: E402
from meshlib import (CRANK, BLOCK, Donor, Mesh, cylinder, circle, prism, convex_hull, pin_xy,  # noqa: E402
                     main_shells, build_block, write_engine, primary_meta, tube, box)

# ---- Ford Cyclone 3.7 numbers ------------------------------------------------------------------
BORE = 95.5
STROKE = 86.7
THROW = STROKE / 2.0
ROD_L = 150.0           # estimate, rod / stroke 1.73
V_ANGLE = 60.0
PAIR_PITCH = 108.0      # split-pin pair to pair along the crank (= bore spacing in a bank), estimate
ROD_PITCH = 32.0        # one half of the split pin per rod (the scaled donor rod is ~32 mm wide)
FIRING = (1, 4, 2, 5, 3, 6)
REST_LOCAL1 = 30.0      # rest pose: cylinder 1 is 30 deg past TDC
SQUISH = 1.0

BANK_A = -V_ANGLE / 2.0   # right bank, cylinders 1-3
BANK_B = +V_ANGLE / 2.0   # left bank, cylinders 4-6

# Ford's black plastic cam covers, see-through like the block
FORD_BLACK = (0.07, 0.07, 0.08, 0.32, 0.0, 0.6, 1)


# ---- twin-screw blower (TURBOv1) --------------------------------------------------------------------
BLOWER_AT = (0.0, -545.0, 0.0)   # frame origin in mesh space: over the V, mid-length
HOUSING, ROTOR, BEARING, ROTOR_B, ROTOR_A = range(5)   # src/turbo.hpp turboModel kinds
GHOST = ((0.3662526, 0.3564001, 0.3324515, 0.25), 0.65, 0.0)   # the block's see-through material
ROTOR_ALU = ((0.78, 0.78, 0.80, 1.0), 0.3, 0.95)
ROTOR_COAT = ((0.25, 0.25, 0.27, 1.0), 0.45, 0.6)              # coated female rotor, darker
PULLEY = ((0.55, 0.55, 0.58, 1.0), 0.3, 0.9)
BELT = ((0.05, 0.05, 0.05, 1.0), 0.9, 0.0)
ROTOR_HALF = 140.0      # rotors from x -140 to +140 (280 mm long)
CENTRES = 92.0          # rotor axis spacing
MALE_Z, FEMALE_Z = CENTRES / 2.0, -CENTRES / 2.0   # frame z: -> mesh x = -z (male on the -x side)
CRANK_PULLEY_Z = -209.0  # the crank damper's middle, mesh z (split_pin_crank: w_lo - 53)


def to_frame(p):
    """mesh space -> blower frame (x = mesh z, y = mesh y, z = -mesh x)."""
    return (p[2] - BLOWER_AT[2], p[1] - BLOWER_AT[1], -(p[0] - BLOWER_AT[0]))


def twisted_rotor(lobes, r_root, r_tip, sharp, twist_deg, zc):
    """a helical rotor along x (-ROTOR_HALF .. ROTOR_HALF) on the axis (y 0, z zc): a lobed profile
    r(t) = r_root + (r_tip - r_root) * (0.5 + 0.5 cos(lobes t)) ** sharp, turned twist_deg over its
    length, with end caps. Smooth normals from the profile."""
    m = Mesh()
    M, K = 120, 36
    rings = []
    for k in range(K + 1):
        x = -ROTOR_HALF + 2.0 * ROTOR_HALF * k / K
        tw = math.radians(twist_deg) * k / K
        ring = []
        for j in range(M):
            t = 2 * math.pi * j / M
            def rad(tt):
                return r_root + (r_tip - r_root) * (0.5 + 0.5 * math.cos(lobes * tt)) ** sharp
            r = rad(t)
            dr = (rad(t + 1e-4) - rad(t - 1e-4)) / 2e-4
            a = t + tw
            py, pz = r * math.cos(a), r * math.sin(a)
            # outward normal of the curve (r cos a, r sin a): rotate the tangent by -90 deg
            ty, tz = dr * math.cos(a) - r * math.sin(a), dr * math.sin(a) + r * math.cos(a)
            ln = math.hypot(ty, tz) or 1.0
            ring.append(m.vert((x, py, pz + zc), (0.0, tz / ln, -ty / ln)))
        rings.append(ring)
    for k in range(K):
        for j in range(M):
            a, b = rings[k][j], rings[k][(j + 1) % M]
            c, d = rings[k + 1][(j + 1) % M], rings[k + 1][j]
            m.tri(a, b, c)
            m.tri(a, c, d)
    for k, sgn in ((0, -1.0), (K, 1.0)):
        ctr = m.vert((sgn * ROTOR_HALF, 0.0, zc), (sgn, 0.0, 0.0))
        cap = [m.vert(tuple(m.v[6 * i:6 * i + 3]), (sgn, 0.0, 0.0)) for i in rings[k]]
        for j in range(M):
            if sgn > 0:
                m.tri(ctr, cap[j], cap[(j + 1) % M])
            else:
                m.tri(ctr, cap[(j + 1) % M], cap[j])
    return m


def build_blower(out_file):
    groups = []
    # rotors: male 4 lobes (fat, round), female 6 (narrow lobes, deep flutes), opposite hands; the
    # female turns 4/6 as fast, so its twist is 4/6 of the male's
    male = twisted_rotor(4, 26.0, 52.0, 0.45, 300.0, MALE_Z)
    male.add(tube([(-ROTOR_HALF - 70.0, 0.0, MALE_Z), (ROTOR_HALF + 15.0, 0.0, MALE_Z)], [9.0, 9.0], 20, caps=True))
    groups.append((ROTOR_A, ROTOR_ALU, male))   # off the frame's axis: turns about its own
    pul = Mesh()   # the 8-rib drive pulley (~90 mm) on the male shaft, ahead of the front plate
    pul.add(tube([(-ROTOR_HALF - 68.0, 0.0, MALE_Z), (-ROTOR_HALF - 38.0, 0.0, MALE_Z)], [45.0, 45.0], 48, caps=True))
    pul.add(tube([(-ROTOR_HALF - 72.0, 0.0, MALE_Z), (-ROTOR_HALF - 66.0, 0.0, MALE_Z)], [49.0, 49.0], 48, caps=True))
    groups.append((ROTOR_A, PULLEY, pul))
    female = twisted_rotor(6, 24.0, 42.0, 1.8, -200.0, FEMALE_Z)
    female.add(tube([(-ROTOR_HALF - 20.0, 0.0, FEMALE_Z), (ROTOR_HALF + 15.0, 0.0, FEMALE_Z)], [8.0, 8.0], 20, caps=True))
    groups.append((ROTOR_B, ROTOR_COAT, female))

    # housing: the two bores' figure-8 case, the front gear case and the rear bearing plate, and the
    # outlet on the underside into the manifold below (see-through)
    hs = Mesh()
    # prism extrudes a (px, py) outline along its z: drawn with px = -frame z, then turned a quarter
    # round y so the axis is the frame's x (x <- pz, y <- py, z <- -px)
    case = convex_hull(circle(-MALE_Z, 0.0, 60.0, 48) + circle(-FEMALE_Z, 0.0, 58.0, 48))
    case_m = prism(case, -ROTOR_HALF - 5.0, ROTOR_HALF + 5.0)
    hs.add(case_m.transformed(lambda p: (p[2], p[1], -p[0]), lambda n: (n[2], n[1], -n[0])))
    for x0, x1 in ((-ROTOR_HALF - 35.0, -ROTOR_HALF - 5.0), (ROTOR_HALF + 5.0, ROTOR_HALF + 22.0)):
        hs.add(box((x0, -62.0, -112.0), (x1, 62.0, 112.0)))
    hs.add(box((-ROTOR_HALF + 10.0, 40.0, -70.0), (ROTOR_HALF - 10.0, 70.0, 70.0)))   # outlet flange
    groups.append((HOUSING, GHOST, hs))

    # belt: from the crank damper (r 62) to the pulley (r 45), its two straight runs (static)
    belt = Mesh()
    yc, zc = to_frame((0.0, 0.0, CRANK_PULLEY_Z))[1], 0.0     # crank axis in the frame (z 0 = mesh x 0)
    xb = to_frame((0.0, 0.0, CRANK_PULLEY_Z))[0]
    for side in (-1.0, 1.0):
        # outer tangents of the two pulleys in the frame's y-z plane (both axes along x)
        dy, dz = 0.0 - yc, MALE_Z - zc
        L = math.hypot(dy, dz)
        uy, uz = dy / L, dz / L
        ny, nz = -uz * side, uy * side
        a = (xb, yc + ny * 62.0, zc + nz * 62.0)
        b = (xb, 0.0 + ny * 45.0, MALE_Z + nz * 45.0)
        belt.add(tube([a, b], [4.0, 4.0], 8, caps=True))
    groups.append((BEARING, BELT, belt))

    out = bytearray(b"TURBOv1\0")
    out += struct.pack("<I", len(groups))
    for kind, (rgba, rough, metal), mesh in groups:
        out += struct.pack("<I4f2fII", kind, *rgba, rough, metal, len(mesh.v) // 6, len(mesh.i))
        out += struct.pack("<%df" % len(mesh.v), *mesh.v)
        out += struct.pack("<%dI" % len(mesh.i), *mesh.i)
    with open(out_file, "wb") as f:
        f.write(out)
    print("  twin-screw blower at (%.0f, %.0f, %.0f); wrote %s" % (BLOWER_AT + (out_file,)))


def split_pin_crank(pairs, throw, r_main, r_pin, r_cw, web, mid_web, alt_r, hub_r):
    """pairs: per split pin (z centre, half length of one pin half, front half's mesh angle, rear
    half's mesh angle). Each pair: an outer web with a counterweight on each side, a thin flying web
    joining the two halves, a main journal between pairs and at each end (four mains for a V6), a
    nose with the damper at -z and the flywheel hub at +z. Returns the crank Mesh, the mains, the
    crank's z extent and the flywheel's z."""
    crank = Mesh()

    def web_at(z0, z1, ang):
        px, py = pin_xy(ang, throw)
        cw = math.radians(ang + 180.0)
        pts = circle(px, py, r_pin + 7.0, 40) + circle(0.0, 0.0, r_main + 9.0, 40)
        a_mid = math.atan2(-math.cos(cw), math.sin(cw))
        pts += circle(0.0, 0.0, r_cw, 40, a_mid - math.radians(65), a_mid + math.radians(65))
        crank.add(prism(convex_hull(pts), z0, z1))

    outer = []
    for zc, half, a_front, a_rear in pairs:
        fx, fy = pin_xy(a_front, throw)
        rx, ry = pin_xy(a_rear, throw)
        crank.add(cylinder(fx, fy, r_pin, zc - mid_web / 2 - 2 * half, zc - mid_web / 2, 40))
        crank.add(cylinder(rx, ry, r_pin, zc + mid_web / 2, zc + mid_web / 2 + 2 * half, 40))
        # the flying web: both pin bosses joined
        crank.add(prism(convex_hull(circle(fx, fy, r_pin + 4.0, 40) + circle(rx, ry, r_pin + 4.0, 40)),
                        zc - mid_web / 2, zc + mid_web / 2))
        z_lo, z_hi = zc - mid_web / 2 - 2 * half, zc + mid_web / 2 + 2 * half
        web_at(z_lo - web, z_lo, a_front)
        web_at(z_hi, z_hi + web, a_rear)
        outer.append((z_lo - web, z_hi + web))
    w_lo, w_hi = outer[0][0], outer[-1][1]
    mains = [(w_lo - 22.0, w_lo)]
    for k in range(len(outer) - 1):
        mains.append((outer[k][1], outer[k + 1][0]))
    mains.append((w_hi, w_hi + 22.0))
    for z0, z1 in mains:
        crank.add(cylinder(0, 0, r_main, z0, z1))
    crank.add(cylinder(0, 0, r_main - 6.0, w_lo - 70.0, w_lo - 22.0))  # nose
    crank.add(cylinder(0, 0, alt_r, w_lo - 66.0, w_lo - 40.0, 64))     # harmonic damper / pulley
    crank.add(cylinder(0, 0, r_main - 4.0, w_hi + 22.0, w_hi + 40.0))  # flange
    crank.add(cylinder(0, 0, hub_r, w_hi + 26.0, w_hi + 38.0, 72))     # flywheel hub
    return crank, mains, w_lo, w_hi, w_hi + 32.0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "mustang_v6.mesh")
    blower_file = sys.argv[3] if len(sys.argv) > 3 else os.path.join(here, "mustang_v6_blower.mesh")
    donor = Donor(src)
    mats = list(donor.mats) + [FORD_BLACK]
    MAT_COVER = len(mats) - 1
    MAT_CRANK = 2
    f = donor.fit(BORE, THROW, ROD_L)
    assert 2.0 * f.rod_half_w <= ROD_PITCH + 0.5, "big ends overlap"

    n = len(FIRING)
    lag = [0.0] * n
    for k, cn in enumerate(FIRING):
        lag[cn - 1] = 720.0 / n * k

    # ---- cylinders: pair i carries cylinder i+1 (right bank, front half) and i+4 (left, rear half)
    MID_WEB = 6.0
    pair_z = [(-1.0 + i) * PAIR_PITCH for i in range(3)]
    layout, pin_ang = [], [0.0] * n
    for c in range(n):
        bank = BANK_A if c < 3 else BANK_B
        i = c % 3
        dz = -(MID_WEB / 2 + ROD_PITCH / 2) if c < 3 else (MID_WEB / 2 + ROD_PITCH / 2)
        theta = (REST_LOCAL1 - lag[c]) % 360.0      # local crank angle at rest
        pin_ang[c] = bank + theta                   # the pin's mesh angle
        layout.append((pair_z[i] + dz, theta, bank, lag[c]))
    deck_y = f.deck_y(SQUISH)
    out_cyls, out_groups = f.cylinders(layout, deck_y)
    split = [((pin_ang[i] - pin_ang[i + 3]) % 360.0) for i in range(3)]
    assert all(abs(s - split[0]) < 1e-6 for s in split), "split angle differs between pairs"

    # ---- crankshaft ---------------------------------------------------------------------------
    R_MAIN = f.s * 19.2
    R_PIN = f.s * 17.0
    skirt_bdc = ROD_L - THROW - f.skirt_below
    R_CW = skirt_bdc - 6.0
    pairs = [(pair_z[i], ROD_PITCH / 2.0, pin_ang[i], pin_ang[i + 3]) for i in range(3)]
    crank, mains, w_lo, w_hi, z_fly = split_pin_crank(pairs, THROW, R_MAIN, R_PIN, R_CW, 13.0, MID_WEB, 62.0, 40.0)
    out_groups.append((CRANK, -1, MAT_CRANK, crank))
    out_groups += main_shells(mains, R_MAIN)

    # ---- block (see-through): wet-sump crankcase with a deep pan, two banks, heads, cam covers ----
    case_r = max(R_CW, R_PIN + THROW + 30.0) + 14.0
    banks = []
    for bank in (BANK_A, BANK_B):
        zs = [cy_[0] for cy_ in out_cyls if abs(math.degrees(cy_[8]) - bank) < 1e-6]
        banks.append((bank, min(zs), max(zs)))
    block, covers = build_block(banks, BORE, deck_y, case_r, (w_lo - 45.0, w_hi + 40.0), case_r + 60.0, 10.0, 110.0, 26.0, 14.0)
    out_groups.append((BLOCK, -1, 0, block))
    out_groups.append((BLOCK, -1, MAT_COVER, covers))

    # ---- report ---------------------------------------------------------------------------------
    disp = n * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("Ford Mustang V6 style 60 deg V6 (Cyclone 3.7) from the inline-4 parts")
    print("  bore %.1f  stroke %.1f  rod %.1f  -> %.0f cc   V %.0f deg, banks %+.0f / %+.0f deg, split pins %.0f deg"
          % (BORE, STROKE, ROD_L, disp, V_ANGLE, BANK_A, BANK_B, split[0]))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (f.s, f.L_s, ROD_L, f.k_beam))
    print("  piston crown %.1f above pin, skirt %.1f below; deck at %.1f; counterweight R %.1f (skirt at BDC %.1f)"
          % (f.crown_above, f.skirt_below, -deck_y, R_CW, skirt_bdc))
    print("  crank z %.1f .. %.1f, %d mains, flywheel at z %.1f" % (w_lo - 70.0, w_hi + 40.0, len(mains), z_fly))
    for c in range(n):
        cy_ = out_cyls[c]
        print("  cyl %d %s bank  z %+6.1f  pin %6.1f deg  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, "right" if c < 3 else "left ", cy_[0], pin_ang[c] % 360, math.degrees(cy_[6]) % 360, lag[c]))

    assert abs((w_lo - 53.0) - CRANK_PULLEY_Z) < 0.5, "crank damper moved: update CRANK_PULLEY_Z"
    size = write_engine(out_file, donor.root, mats, out_cyls, out_groups,
                        [("banked", [1.0]), primary_meta(z_fly), ("supercharger_at", list(BLOWER_AT))])
    print("wrote %s (%.1f MB, %d groups)" % (out_file, size / 1e6, len(out_groups)))
    build_blower(blower_file)


if __name__ == "__main__":
    main()
