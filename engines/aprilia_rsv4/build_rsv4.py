#!/usr/bin/env python3
"""Build an Aprilia RSV4 RR style 65 degree V4 (2015-2020, 999.6 cc) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/aprilia_rsv4/build_rsv4.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/aprilia_rsv4/rsv4.mesh)

Made like the Ducati V4 (engines/ducati_v4/build_v4.py, which explains the donor parts): the inline-4's
piston and rod scaled to the 78 mm bore with the rod beam shortened, on a procedural crank, block and
heads (tools/meshlib.py). What differs is the layout:
    - a narrow 65 degree V (the Ducati's is 90);
    - the crank turns forward (front of the bike +X here, as the MV and the BMW; the Ducati's turns
      backward with its front at -X). The crank turns R_z(+delta), which moves its top toward +X;
    - two crankpins 180 degrees apart, each carrying a front and a rear rod side by side.

Bank angles: bore axis = R_z(bank) * (0, -1). The front bank points forward-up, FRONT_ELEV above
horizontal; the rear bank is 65 degrees behind it, leaning slightly back of vertical (estimate from
photographs of the engine).

Cylinders: 1 front-left, 2 rear-left (crankpin A), 3 front-right, 4 rear-right (crankpin B).
Going forward, a pin passes the rear bank 65 degrees before the front one, so on one pin the rear
cylinder fires 360 - 65 = 295 degrees after the front (or 65 before). Firing 1-4-2-3 at 0, 115, 295,
540: intervals 115-180-245-180, the RSV4's uneven 65 degree V4 beat.

Aprilia: 78 x 52.3 mm, 999.6 cc, 13.6:1. Estimates (not published): rod length 105 mm, cylinder
pitch in a bank 92 mm, front bank 35 degrees above horizontal.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import CRANK, BLOCK, Donor, build_crank, main_shells, build_block, write_engine, primary_meta  # noqa: E402

# ---- Aprilia RSV4 RR numbers -------------------------------------------------------------------
BORE = 78.0
STROKE = 52.3
THROW = STROKE / 2.0
ROD_L = 105.0           # estimate, rod/stroke ~2.0
V_ANGLE = 65.0
FRONT_ELEV = 35.0       # front bank angle above horizontal (estimate)
PIN_SPLIT = 180.0       # crankpin B is half a turn from A
PIN_GAP = 92.0          # crankpin A to crankpin B along the crank (= cylinder pitch in a bank)
ROD_PITCH = 22.0        # front and rear big ends side by side on one pin
REST_LOCAL1 = 30.0      # rest pose: cylinder 1 is 30 deg past TDC
SQUISH = 1.2            # crown to deck at TDC

# front of the bike is +X: the front bore axis R_z(bank) * (0, -1) = (sin bank, -cos bank) points +X, up
BANK_FRONT = 90.0 - FRONT_ELEV
BANK_REAR = BANK_FRONT - V_ANGLE
REAR_AFTER_FRONT = 360.0 - V_ANGLE   # forward rotation: a pin reaches the rear bank 295 deg after the front

BLACK_GLASS = (0.10, 0.10, 0.11, 0.30, 0.0, 0.5, 1)   # Aprilia's black cam covers, see-through like the block


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "rsv4.mesh")
    donor = Donor(src)
    mats = list(donor.mats) + [BLACK_GLASS]
    MAT_COVER = len(mats) - 1
    MAT_CRANK = 2
    f = donor.fit(BORE, THROW, ROD_L)

    # ---- cylinders ---------------------------------------------------------------------------
    pin_z = {"A": -PIN_GAP / 2.0, "B": PIN_GAP / 2.0}
    rest_pin = {"A": BANK_FRONT + REST_LOCAL1}           # mesh angle of pin A at rest
    rest_pin["B"] = rest_pin["A"] + PIN_SPLIT
    # B's front cylinder fires at 540 (B is half a turn away from A, on the exhaust stroke at 180)
    b_front = 360.0 + PIN_SPLIT
    layout = [  # (name, pin, bank, z offset on the pin, firing lag behind cylinder 1)
        ("front-left", "A", BANK_FRONT, -ROD_PITCH / 2.0, 0.0),
        ("rear-left", "A", BANK_REAR, +ROD_PITCH / 2.0, REAR_AFTER_FRONT),
        ("front-right", "B", BANK_FRONT, -ROD_PITCH / 2.0, b_front),
        ("rear-right", "B", BANK_REAR, +ROD_PITCH / 2.0, (b_front + REAR_AFTER_FRONT) % 720.0),
    ]
    deck_y = f.deck_y(SQUISH)
    out_cyls, out_groups = f.cylinders([(pin_z[pin] + dz, rest_pin[pin] - bank, bank, lag)
                                        for name, pin, bank, dz, lag in layout], deck_y)

    # ---- crankshaft (mesh space): two pins, two rods side by side on each ----------------------
    R_MAIN = f.s * 19.2
    R_PIN = f.s * 17.0
    skirt_bdc = ROD_L - THROW - f.skirt_below
    R_CW = skirt_bdc - 6.0
    pins = [(pin_z[k], ROD_PITCH, rest_pin[k]) for k in ("A", "B")]
    crank, mains, w_lo, w_hi, z_primary = build_crank(pins, THROW, R_MAIN, R_PIN, R_CW, 14.0, 50.0, 30.0, main_order=(1, 0, 2))
    out_groups.append((CRANK, -1, MAT_CRANK, crank))
    out_groups += main_shells(mains, R_MAIN)

    # ---- block (see-through), cylinder banks in their local frame then rotated -------------------
    case_r = max(R_CW, R_PIN + THROW + 30.0) + 14.0
    banks = []
    for bank in (BANK_FRONT, BANK_REAR):
        zs = [cy_[0] for cy_ in out_cyls if abs(math.degrees(cy_[8]) - bank) < 1e-6]
        banks.append((bank, min(zs), max(zs)))
    block, cover = build_block(banks, BORE, deck_y, case_r, (w_lo - 50.0, w_hi + 45.0), 110.0, 9.0, 74.0, 18.0, 10.0)
    out_groups.append((BLOCK, -1, 0, block))
    out_groups.append((BLOCK, -1, MAT_COVER, cover))

    # ---- report ---------------------------------------------------------------------------------
    disp = 4 * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("Aprilia RSV4 RR style V4 from the inline-4 parts")
    print("  bore %.1f  stroke %.1f  rod %.1f  -> %.1f cc   V %.0f deg, front bank %.0f deg above horizontal"
          % (BORE, STROKE, ROD_L, disp, V_ANGLE, FRONT_ELEV))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (f.s, f.L_s, ROD_L, f.k_beam))
    print("  crankpins %.0f deg apart at z %.0f / %.0f, rods %.0f mm apart on each pin"
          % (PIN_SPLIT, pin_z["A"], pin_z["B"], ROD_PITCH))
    for c, (name, pin, bank, dz, lag) in enumerate(layout):
        cy_ = out_cyls[c]
        print("  cyl %d %-11s pin %s  bank %+5.1f deg  z %+6.1f  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, name, pin, bank, cy_[0], math.degrees(cy_[6]) % 360, lag))

    size = write_engine(out_file, donor.root, mats, out_cyls, out_groups, [("banked", [1.0]), primary_meta(z_primary)])
    print("wrote %s (%.1f MB, %d groups)" % (out_file, size / 1e6, len(out_groups)))


if __name__ == "__main__":
    main()
