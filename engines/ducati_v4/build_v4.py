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
What is new (procedural, with tools/meshlib.py's Donor, build_crank and build_block):
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
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import CRANK, BLOCK, Donor, build_crank, main_shells, build_block, write_engine  # noqa: E402

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


# ---- the build ---------------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "v4.mesh")
    donor = Donor(src)
    mats = list(donor.mats) + [RED_GLASS]
    MAT_RED = len(mats) - 1
    MAT_CRANK = 2
    f = donor.fit(BORE, THROW, ROD_L)

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
    deck_y = f.deck_y(SQUISH)
    # local crank angle at rest = the pin's mesh angle less the bank
    out_cyls, out_groups = f.cylinders([(pin_z[pin] + dz, rest_pin[pin] - bank, bank, lag)
                                        for name, pin, bank, dz, lag in layout], deck_y)

    # ---- crankshaft (mesh space): two pins, two rods side by side on each ----------------------
    R_MAIN = f.s * 19.2
    R_PIN = f.s * 17.0
    # counterweights must clear the piston skirts at BDC, with margin
    skirt_bdc = ROD_L - THROW - f.skirt_below
    R_CW = skirt_bdc - 6.0
    pins = [(pin_z[k], ROD_PITCH, rest_pin[k]) for k in ("A", "B")]
    # mains written centre, left, right
    crank, mains, w_lo, w_hi, _ = build_crank(pins, THROW, R_MAIN, R_PIN, R_CW, 14.0, 50.0, 30.0, main_order=(1, 0, 2))
    out_groups.append((CRANK, -1, MAT_CRANK, crank))
    out_groups += main_shells(mains, R_MAIN)

    # ---- block (see-through), cylinder banks in their local frame then rotated -------------------
    case_r = max(R_CW, R_PIN + THROW + 30.0) + 14.0
    banks = []
    for bank in (BANK_FRONT, BANK_REAR):
        zs = [cy_[0] for cy_ in out_cyls if abs(math.degrees(cy_[8]) - bank) < 1e-6]
        banks.append((bank, min(zs), max(zs)))
    block, red = build_block(banks, BORE, deck_y, case_r, (w_lo - 50.0, w_hi + 45.0), 110.0, 9.0, 78.0, 18.0, 10.0)
    out_groups.append((BLOCK, -1, 0, block))
    out_groups.append((BLOCK, -1, MAT_RED, red))

    # ---- report ---------------------------------------------------------------------------------
    disp = 4 * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("Ducati-style V4 from the inline-4 parts")
    print("  bore %.1f  stroke %.1f  rod %.1f  -> %.0f cc   V %.0f deg, front bank %.0f deg above horizontal"
          % (BORE, STROKE, ROD_L, disp, V_ANGLE, FRONT_ELEV))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (f.s, f.L_s, ROD_L, f.k_beam))
    print("  piston crown %.1f above pin, skirt %.1f below; deck at %.1f; counterweight R %.1f (skirt at BDC %.1f)"
          % (f.crown_above, f.skirt_below, -deck_y, R_CW, skirt_bdc))
    print("  crankpins %.0f deg apart at z %.0f / %.0f, rods %.0f mm apart on each pin"
          % (PIN_SPLIT, pin_z["A"], pin_z["B"], ROD_PITCH))
    for c, (name, pin, bank, dz, lag) in enumerate(layout):
        cy_ = out_cyls[c]
        print("  cyl %d %-11s pin %s  bank %+5.1f deg  z %+6.1f  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, name, pin, bank, cy_[0], math.degrees(cy_[6]) % 360, lag))

    size = write_engine(out_file, b"ENGMSHv2", donor.root, mats, out_cyls, out_groups)
    print("wrote %s (%.1f MB, %d groups)" % (out_file, size / 1e6, len(out_groups)))


if __name__ == "__main__":
    main()
