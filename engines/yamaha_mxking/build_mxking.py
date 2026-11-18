#!/usr/bin/env python3
"""Yamaha MX King 150 (2015-2020, Jupiter MX King / T150) mesh builder: a single from the inline-4
model's parts (engines/yamaha_mxking/). 149.7 cc SOHC four-valve, liquid cooled, the cylinder laid
well forward as in an underbone frame.

Standard library only. Usage:
    python3 -I engines/yamaha_mxking/build_mxking.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> mxking.mesh next to this file)

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
BORE = 57.0             # mm (Yamaha's spec sheet)
STROKE = 58.7           # mm (57 x 58.7: 149.8 cc, quoted as 149.7)
ROD_L = 98.0            # connecting rod, centre to centre, mm (not published; ~1.67 x the stroke)
PITCH = 70.0            # bore centre spacing (one cylinder: only sizes the crank)
INCLINE = 70.0          # the cylinder leans forward (toward +X) by this much, deg: an underbone's
                        # near-horizontal barrel
FIRING = (1,)           # a single: fires once every 720 deg

# cam cover material: r, g, b, a, metallic, roughness, blend (1: see-through like the block)
COVER = (0.10, 0.18, 0.55, 0.30, 0.0, 0.5, 1)  # Yamaha racing blue


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "mxking.mesh")
    inline_engine(src, out_file, "Yamaha MX King 150 single", BORE, STROKE, ROD_L, PITCH, INCLINE,
                  FIRING, COVER)


if __name__ == "__main__":
    main()
