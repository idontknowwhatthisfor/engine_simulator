#!/usr/bin/env python3
"""Build an Audi RS 3 style inline-5 (the 2.5 TFSI, EA855 evo) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/audi_rs3/build_rs3.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/audi_rs3/rs3.mesh)

The engine is tools/meshlib.py's inline_engine() (as the superbikes: donor piston and rod from the
inline-4, scaled to the bore and the rod length; a procedural crank, crankcase, block and head). This
file only gives the five-cylinder's numbers.

Audi 2.5 TFSI (RS 3 8Y, engine code DAZA): 2480 cc, 82.5 x 92.8 mm, 10.0:1, firing 1-2-4-5-3 with
the crankpins 72 deg apart (even firing every 144 deg). Rod 144 mm (the aftermarket rods for the DAZA
are listed at 144 mm centre to centre). Estimates: bore pitch 88 mm, the bank upright (the car's
crank runs along the car in this simulator, and the transverse RS 3 engine stands nearly upright).

Mesh space as for the inline-4: crank axis = Z, "up" is -Y; the car's gearbox goes behind the
flywheel at +Z.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import inline_engine  # noqa: E402

# ---- Audi 2.5 TFSI numbers ---------------------------------------------------------------------
BORE = 82.5
STROKE = 92.8
ROD_L = 144.0
PITCH = 88.0            # bore centre spacing (estimate: 82.5 mm bore + 5.5 mm between the liners)
INCLINE = 0.0
FIRING = (1, 2, 4, 5, 3)

# the RS cam cover: a red "RS" painted cover, see-through like the block
RS_RED = (0.62, 0.05, 0.06, 0.32, 0.1, 0.45, 1)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "rs3.mesh")
    inline_engine(src, out_file, "Audi 2.5 TFSI style inline-5", BORE, STROKE, ROD_L, PITCH, INCLINE,
                  FIRING, RS_RED)


if __name__ == "__main__":
    main()
