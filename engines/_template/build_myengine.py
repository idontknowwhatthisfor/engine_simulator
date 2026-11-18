#!/usr/bin/env python3
"""Template mesh builder: an inline engine from the inline-4 model's parts (engines/_template/).

Copy this folder to start a new engine (see README.md, "Adding an engine"): replace `myengine` with
the engine's id and `_template` with the folder name, in the file names and inside the files. As it
stands it builds an inline-3; change the numbers below for yours.

Standard library only. Usage:
    python3 -I engines/_template/build_myengine.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> myengine.mesh next to this file)

tools/meshlib.py's inline_engine() does the work: it scales the inline-4's piston and rod to the bore,
stroke and rod length, builds a flat crank with one throw per cylinder (pins placed for the even
firing order), a see-through crankcase, block and head and a cam cover, and writes the ENGMSHv3 mesh
with the firing lags and the metadata the simulator reads (zPrimary for the bike gearbox).
Another layout (a V, a boxer, extra parts like the H2's supercharger) builds its parts with meshlib's
Donor, build_crank, build_block and write_engine directly: see engines/ducati_v4/build_v4.py and
engines/kawasaki_h2/build_h2.py.

Mesh space (mm): crank axis = Z through (0, 0), "up" is -Y, the front of a bike at +X (the crank then
turns forward). Run the simulator's --selftest after any change: it checks the kinematics, that the
crank clears the piston skirts and that the crown stays below the deck.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import inline_engine  # noqa: E402

# ---- the engine's numbers ----------------------------------------------------------------------
BORE = 74.0             # mm
STROKE = 58.0           # mm (3 x 74 x 58: 748 cc)
ROD_L = 105.0           # connecting rod, centre to centre, mm
PITCH = 86.0            # bore centre spacing along the crank, mm
INCLINE = 30.0          # the bank leans forward (toward +X) by this much, deg
FIRING = (1, 2, 3)      # firing order, cylinder numbers from 1; any count, evenly spaced over 720 deg

# cam cover material: r, g, b, a, metallic, roughness, blend (1: see-through like the block)
COVER = (0.55, 0.12, 0.08, 0.30, 0.0, 0.5, 1)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "myengine.mesh")
    inline_engine(src, out_file, "myengine inline-%d" % len(FIRING), BORE, STROKE, ROD_L, PITCH, INCLINE,
                  FIRING, COVER)


if __name__ == "__main__":
    main()
