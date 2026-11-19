#!/usr/bin/env python3
"""A procedural turbocharger for the RS 3, used when the downloaded turbo model is not there.

Standard library only. Usage:
    python3 -I engines/audi_rs3/build_rs3_turbo.py [out_file]
    (default: engines/audi_rs3/rs3_turbo.mesh)

The RS 3 draws the Sketchfab "Car Turbocharger" (engines/inline4_fit/turbo.mesh) when it is there. That
model comes from the optional Assets-2 download, so without it the RS 3 falls back on this one
(engineType::turboFallbackMeshFile). It is written in the same "TURBOv1" layout
(engines/inline4_fit/build_turbo.py), in the same turbo frame and with its four ports where the
Sketchfab model has them, so the RS 3's exhaust layout (audi_rs3.hpp) fits either:
    turbine outlet     (0, 0, 0)            facing -x
    turbine inlet      (45.7, -96.6, 64.1)  facing -y
    compressor inlet   (196.8, 0, 0)        facing +x
    compressor outlet  (153.3, 68.5, -85.5) facing -z
The shaft runs along +x from the turbine outlet face. Parts:
    - rotor (turns, src/turbo.hpp spins it about x): turbine wheel, shaft, compressor wheel;
    - bearing (still): the centre housing with its oil and water stubs;
    - housings (see-through, hidden with the block): the turbine scroll and its inlet flange and
      outlet, the compressor scroll with its inlet nose and outlet.
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import Mesh, tube, solid  # noqa: E402

HOUSING, ROTOR, BEARING = range(3)
GHOST = ((0.3662526, 0.3564001, 0.3324515, 0.25), 0.65, 0.0)   # the inline-4 block's see-through material
CAST = ((0.32, 0.30, 0.29, 1.0), 0.6, 0.6)                      # cast-iron centre housing
INCONEL = ((0.42, 0.38, 0.34, 1.0), 0.45, 0.85)                 # turbine wheel
ALU = ((0.80, 0.80, 0.82, 1.0), 0.25, 0.95)                     # machined compressor wheel
STEEL = ((0.45, 0.45, 0.48, 1.0), 0.35, 0.9)

T_X = 38.0              # turbine wheel centre along the shaft
C_X = 168.0             # compressor wheel centre


def wheel(x0, x1, r_hub0, r_hub1, r_tip0, r_tip1, blades, thick=1.2, sweep=0.0):
    """a radial wheel about x: a hub tapering from x0 to x1 and straight blades from hub to tip,
    each twisted by `sweep` rad from x0 to x1."""
    m = Mesh()
    n = 8
    m.add(tube([(x0 + (x1 - x0) * k / n, 0.0, 0.0) for k in range(n + 1)],
               [r_hub0 + (r_hub1 - r_hub0) * k / n for k in range(n + 1)], 32, caps=True))
    faces = []
    for b in range(blades):
        a0 = 2 * math.pi * b / blades
        secs = []
        for k in range(n + 1):
            f = k / n
            x = x0 + (x1 - x0) * f
            rh = r_hub0 + (r_hub1 - r_hub0) * f
            rt = r_tip0 + (r_tip1 - r_tip0) * f
            a = a0 + sweep * f
            pts = []
            for r, side in ((rh, -1), (rt, -1), (rt, 1), (rh, 1)):
                aa = a + side * thick / max(r, 1.0)
                pts.append((x, r * math.cos(aa), r * math.sin(aa)))
            secs.append(pts)
        for k in range(n):
            p, q = secs[k], secs[k + 1]
            for j in range(4):
                a, b2, c, d = p[j], p[(j + 1) % 4], q[(j + 1) % 4], q[j]
                faces += [(a, b2, c), (a, c, d)]
        p, q = secs[0], secs[-1]
        faces += [(p[0], p[2], p[1]), (p[0], p[3], p[2]), (q[0], q[1], q[2]), (q[0], q[2], q[3])]
    m.add(solid(faces))
    return m


def scroll(x, r0, r1, sec0, sec1, start_deg, outlet):
    """a volute about the shaft at station x: its centreline spirals from radius r0 to r1 over a
    turn, its section growing from sec0 to sec1, and then runs on through `outlet` (points)."""
    path, rad = [], []
    turns = 36
    for k in range(turns + 1):
        f = k / turns
        a = math.radians(start_deg) + 2 * math.pi * f
        r = r0 + (r1 - r0) * f
        path.append((x, r * math.cos(a), r * math.sin(a)))
        rad.append(sec0 + (sec1 - sec0) * f)
    for p in outlet:
        path.append(p)
        rad.append(sec1)
    return tube(path, rad, 24)


def build(out_file):
    groups = []  # (kind, look, Mesh) in the turbo frame

    # rotor: turbine wheel (exducer toward the outlet at x = 0), shaft, compressor wheel (inducer toward
    # the inlet at +x)
    groups.append((ROTOR, INCONEL, wheel(T_X - 18.0, T_X + 12.0, 9.0, 14.0, 22.0, 31.0, 11, sweep=0.5)))
    groups.append((ROTOR, STEEL, tube([(T_X + 10.0, 0, 0), (C_X - 12.0, 0, 0)], [5.0, 5.0], 16, caps=True)))
    groups.append((ROTOR, ALU, wheel(C_X - 14.0, C_X + 20.0, 15.0, 7.0, 34.0, 24.0, 12, thick=0.8, sweep=-0.6)))

    # bearing (centre) housing between the scrolls, with oil feed / drain and coolant stubs
    ch = Mesh()
    ch.add(tube([(T_X + 22.0, 0, 0), (T_X + 30.0, 0, 0), (C_X - 40.0, 0, 0), (C_X - 28.0, 0, 0)], [44.0, 32.0, 32.0, 50.0], 40,
                caps=True))
    mid = 0.5 * (T_X + C_X)
    ch.add(tube([(mid, -30.0, 0), (mid, -52.0, 0)], [7.0, 7.0], 12, caps=True))     # oil feed (top, -y is up)
    ch.add(tube([(mid, 30.0, 0), (mid, 56.0, 0)], [10.0, 10.0], 12, caps=True))     # oil drain
    ch.add(tube([(mid - 12.0, 0, 30.0), (mid - 12.0, 0, 48.0)], [6.0, 6.0], 12, caps=True))  # coolant
    groups.append((BEARING, CAST, ch))

    hs = Mesh()
    # turbine side: the scroll round the wheel, feeding in from the inlet flange (facing -y)
    hs.add(tube([(T_X + 22.0, 0, 0), (T_X + 14.0, 0, 0)], [62.0, 62.0], 48, caps=True))  # back wall
    inlet_end = (45.7, -96.6, 64.1)
    hs.add(scroll(T_X, 44.0, 40.0, 24.0, 12.0, 300.0, []))
    hs.add(tube([inlet_end, (45.7, -70.0, 62.0), (T_X + 2.0, -42.0, 50.0)], [27.0, 26.0, 24.0], 24))
    hs.add(tube([(inlet_end[0], inlet_end[1] - 2.0, inlet_end[2]), (inlet_end[0], inlet_end[1] + 6.0, inlet_end[2])],
                [40.0, 40.0], 24, caps=True))                                              # inlet flange
    # outlet: the exducer bore widening to the outlet face, and its flange
    hs.add(tube([(T_X - 14.0, 0, 0), (12.0, 0, 0), (0.0, 0, 0)], [26.0, 30.0, 33.0], 32))
    hs.add(tube([(-2.0, 0, 0), (6.0, 0, 0)], [48.0, 48.0], 32, caps=True))
    # compressor side: back plate, scroll, the outlet heading -z to (153.3, 68.5, -85.5), the inlet nose
    hs.add(tube([(C_X - 28.0, 0, 0), (C_X - 20.0, 0, 0)], [76.0, 76.0], 56, caps=True))
    out_end = (153.3, 68.5, -85.5)
    hs.add(scroll(C_X - 10.0, 52.0, 58.0, 10.0, 26.0, 30.0, [(153.3, 68.5, -30.0), out_end]))
    hs.add(tube([(C_X - 16.0, 0, 0), (C_X + 8.0, 0, 0), (C_X + 22.0, 0, 0), (196.8, 0, 0)], [60.0, 52.0, 40.0, 37.0], 48))
    groups.append((HOUSING, GHOST, hs))

    out = bytearray(b"TURBOv1\0")
    out += struct.pack("<I", len(groups))
    for kind, (rgba, rough, metal), mesh in groups:
        out += struct.pack("<I4f2fII", kind, *rgba, rough, metal, len(mesh.v) // 6, len(mesh.i))
        out += struct.pack("<%df" % len(mesh.v), *mesh.v)
        out += struct.pack("<%dI" % len(mesh.i), *mesh.i)
    with open(out_file, "wb") as f:
        f.write(out)
    print("procedural RS 3 turbo: turbine wheel at x %.0f, compressor wheel at x %.0f; wrote %s (%.1f MB)"
          % (T_X, C_X, out_file, len(out) / 1e6))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    build(sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "rs3_turbo.mesh"))


if __name__ == "__main__":
    main()
