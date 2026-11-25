#!/usr/bin/env python3
"""Build an MV Agusta F4 RR style inline-4 (2012- short-stroke engine) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/mv_agusta_f4rr/build_f4rr.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/mv_agusta_f4rr/f4rr.mesh)

The engine is tools/meshlib.py's inline_engine() (the same steps as the S 1000 RR and the ZX-10R:
donor piston and rod from the inline-4, scaled to the 79 mm bore with the rod beam shortened; a
procedural crank, crankcase, block and head). This file only gives the F4 RR's numbers.

MV Agusta F4 RR (2012-): 998 cc, 79.0 x 50.9 mm, 13.4:1, radial titanium valves, 180 deg crank,
firing 1-2-4-3, forward crank rotation. Estimates (not published): rod length 104 mm, bore pitch
87 mm, cylinders inclined 25 deg forward.

Mesh space as for the S 1000 RR: crank axis = Z, "up" is -Y, front of the bike +X.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import inline_engine  # noqa: E402

# ---- MV Agusta F4 RR numbers -------------------------------------------------------------------
BORE = 79.0
STROKE = 50.9
ROD_L = 104.0           # estimate
PITCH = 87.0            # bore centre spacing (estimate: 79 mm bore + 8 mm between liners)
INCLINE = 25.0          # cylinder bank leans forward (toward +X) by this much (estimate)
FIRING = (1, 2, 4, 3)

MV_RED_GLASS = (0.62, 0.06, 0.06, 0.30, 0.0, 0.5, 1)   # MV red cam cover, see-through like the block


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "f4rr.mesh")
    inline_engine(src, out_file, "MV Agusta F4 RR style inline-4", BORE, STROKE, ROD_L, PITCH, INCLINE,
                  FIRING, MV_RED_GLASS)


if __name__ == "__main__":
    main()
