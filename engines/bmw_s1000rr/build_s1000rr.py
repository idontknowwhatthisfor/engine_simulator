#!/usr/bin/env python3
"""Build a BMW S 1000 RR style inline-4 (2019+ ShiftCam engine) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/bmw_s1000rr/build_s1000rr.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/bmw_s1000rr/s1000rr.mesh)

Reused from the inline-4 (engine.mesh): cylinder 1's piston + gudgeon pin and connecting rod + cap
bolts/nuts + big-end shell, scaled from the 85 mm bore to BMW's 80 mm, rod beam shortened to 103 mm.
New (procedural): a forged flat-plane (180 deg) crank with four throws, five mains and
counterweights, alternator rotor on the left and the primary gear on the right; a see-through
crankcase, one-piece cylinder block, head and a blue cam cover. The steps are tools/meshlib.py's
inline_engine(); this file only gives the BMW's numbers.

BMW S 1000 RR (K67, 2019-): 999 cc, 80.0 x 49.7 mm, 13.3:1, 103 mm rods, 180 deg crank, cylinders
inclined 32 deg forward, firing order 1-2-4-3, conventional (forward) crank rotation.

Layout (mesh space, mm, the engine.mesh conventions): crank axis = Z through (0, 0), "up" is -Y.
Unlike the V4 the front of the bike is +X here: the crank turns R_z(+delta), which moves the top of
the crank toward +X, so with the front at +X the crank turns forward like the wheels. The block leans
forward (bank +32 deg) and the simulator puts the gearbox behind the crank (-X).

s1000rr.mesh = the v2 layout of engines/ducati_v4/build_v4.py with the magic "BMWS1KRR".
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import inline_engine  # noqa: E402

# ---- BMW S 1000 RR numbers --------------------------------------------------------------------
BORE = 80.0
STROKE = 49.7
ROD_L = 103.0           # BMW: "103 mm connecting rods"
PITCH = 92.0            # bore centre spacing (estimate: 80 mm bore + 12 mm between liners)
INCLINE = 32.0          # cylinder bank leans forward (toward +X) by this much
FIRING = (1, 2, 4, 3)

BLUE_GLASS = (0.05, 0.22, 0.62, 0.30, 0.0, 0.5, 1)   # cam cover, see-through like the block


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "s1000rr.mesh")
    inline_engine(src, out_file, b"BMWS1KRR", "BMW S 1000 RR style inline-4", BORE, STROKE, ROD_L, PITCH, INCLINE,
                  FIRING, BLUE_GLASS)


if __name__ == "__main__":
    main()
