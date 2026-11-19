#!/usr/bin/env python3
"""Build a Kawasaki Ninja H2 style supercharged inline-4 from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/kawasaki_h2/build_h2.py [i4_mesh] [out_file] [supercharger_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/kawasaki_h2/h2.mesh, h2_supercharger.mesh)

The engine is tools/meshlib.py's inline_engine() (the same steps as the S 1000 RR: donor parts,
procedural crank, crankcase, block and head): cylinder 1's piston and rod from the inline-4, scaled to
the H2's 76 mm bore with the rod beam shortened. This file gives the H2's numbers and builds the
supercharger.

Kawasaki Ninja H2 (2015-): 998 cc, 76.0 x 55.0 mm, 8.5:1, 180 deg crank, firing 1-2-4-3, forward
crank rotation. Estimates (not published): rod length 107 mm, bore pitch 84 mm, cylinders inclined
30 deg forward.

h2.mesh: the ENGMSHv3 engine format (tools/meshlib.py), with zPrimary and supercharger_at. Mesh space as for the S 1000 RR: crank axis = Z,
"up" is -Y, front of the bike +X, left side -Z.

h2_supercharger.mesh: the centrifugal supercharger in the "TURBOv1" layout of
engines/inline4_fit/build_turbo.py, so src/turbo.hpp draws it and spins the impeller. It sits behind
the cylinders on the left, above the gearbox, its impeller axis across the bike, the inlet facing
left. Its frame: origin SC_AT (mesh space) at the impeller's back plate, +x along the shaft out of
the inlet (mesh -Z), y = mesh Y, z = mesh +X (forward). kawasaki_h2.hpp places it with the same
position, stored in h2.mesh. Parts:
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
from meshlib import Mesh, box, inline_engine, tube, solid  # noqa: E402

# ---- Kawasaki Ninja H2 numbers -------------------------------------------------------------------
BORE = 76.0
STROKE = 55.0
ROD_L = 107.0           # estimate
PITCH = 84.0            # bore centre spacing (estimate: 76 mm bore + 8 mm between liners)
INCLINE = 30.0          # cylinder bank leans forward (toward +X) by this much (estimate)
FIRING = (1, 2, 4, 3)

COVER_GLASS = (0.10, 0.11, 0.12, 0.35, 0.0, 0.35, 1)   # dark graphite cam cover, see-through like the block

# supercharger frame origin in mesh space (mm): impeller back plate. Written into h2.mesh as
# supercharger_at, where kawasaki_h2.hpp reads it
SC_AT = (-165.0, -195.0, -95.0)
IMPELLER_R = 34.5       # 69 mm impeller
HOUSING, ROTOR, BEARING = range(3)
GHOST = ((0.3662526, 0.3564001, 0.3324515, 0.25), 0.65, 0.0)   # the inline-4 block's see-through material
ALU = ((0.80, 0.80, 0.82, 1.0), 0.25, 0.95)                     # machined impeller
STEEL = ((0.45, 0.45, 0.48, 1.0), 0.35, 0.9)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "h2.mesh")
    sc_file = sys.argv[3] if len(sys.argv) > 3 else os.path.join(here, "h2_supercharger.mesh")
    eng = inline_engine(src, out_file, "Kawasaki Ninja H2 style inline-4", BORE, STROKE, ROD_L, PITCH,
                        INCLINE, FIRING, COVER_GLASS, meta=[("supercharger_at", list(SC_AT))])
    build_supercharger(sc_file, eng["deck_y"], eng["zs"])


# ---- supercharger (TURBOv1) -------------------------------------------------------------------------
def to_sc(p):
    """mesh space -> supercharger frame."""
    r = (p[0] - SC_AT[0], p[1] - SC_AT[1], p[2] - SC_AT[2])
    return (-r[2], r[1], r[0])


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
