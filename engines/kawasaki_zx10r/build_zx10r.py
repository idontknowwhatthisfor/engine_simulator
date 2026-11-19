#!/usr/bin/env python3
"""Build a Kawasaki Ninja ZX-10R style inline-4 (2021- engine) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/kawasaki_zx10r/build_zx10r.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/kawasaki_zx10r/zx10r.mesh)

The engine is tools/meshlib.py's inline_engine() (the same steps as the S 1000 RR and the H2: donor
piston and rod from the inline-4, scaled to the 76 mm bore with the rod beam shortened; a procedural
crank, crankcase, block and head). This file only gives the ZX-10R's numbers.

Kawasaki Ninja ZX-10R (2021-): 998 cc, 76.0 x 55.0 mm, 13.0:1, 180 deg crank, firing 1-2-4-3,
forward crank rotation. Estimates (not published): rod length 107 mm, bore pitch 84 mm (the H2 shares
the bore and stroke), cylinders inclined 30 deg forward.

Mesh space as for the S 1000 RR: crank axis = Z, "up" is -Y, front of the bike +X.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import inline_engine  # noqa: E402

# ---- Kawasaki ZX-10R numbers -------------------------------------------------------------------
BORE = 76.0
STROKE = 55.0
ROD_L = 107.0           # estimate
PITCH = 84.0            # bore centre spacing (estimate: 76 mm bore + 8 mm between liners)
INCLINE = 30.0          # cylinder bank leans forward (toward +X) by this much (estimate)
FIRING = (1, 2, 4, 3)

GREEN_GLASS = (0.18, 0.55, 0.10, 0.30, 0.0, 0.5, 1)   # Kawasaki-green cam cover, see-through like the block


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "zx10r.mesh")
    inline_engine(src, out_file, "Kawasaki ZX-10R style inline-4", BORE, STROKE, ROD_L, PITCH, INCLINE,
                  FIRING, GREEN_GLASS)


if __name__ == "__main__":
    main()
