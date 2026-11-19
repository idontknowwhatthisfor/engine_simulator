#!/usr/bin/env python3
"""Build a Lexus LFA style 72 degree V10 (Toyota / Yamaha 1LR-GUE) from the inline-4 parts.

Standard library only. Usage:
    python3 -I engines/lexus_lfa/build_lfa.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/lexus_lfa/lfa.mesh)

As the Ducati V4 (engines/ducati_v4/build_v4.py): the inline-4's piston and rod, scaled to the 88 mm
bore and the rod length, on a procedural crank, crankcase, two banks, heads and cam covers
(tools/meshlib.py's Donor, build_crank and build_block).

1LR-GUE: 4805 cc, 88 x 79 mm, 12.0:1, 72 deg V. Five crankpins 144 deg apart, each shared by one rod
from each bank side by side, so each bank fires as an even inline-5 (every 144 deg) and the 72 deg
between the banks puts the other bank's cylinder exactly half way: even firing every 72 deg. That is
why the LFA's V is 72 and not 90 degrees. Estimates (Toyota does not publish them): rod 136 mm
(titanium), the crankpin spacing 104 mm, the firing order 1-6-5-10-2-7-3-8-4-9 (the usual V10 order,
with this numbering the only one in which every shared pin fires its two cylinders 72 deg apart).

Layout (mesh space, mm, the car conventions of engines/audi_rs3/): crank axis = Z through (0, 0), "up"
is -Y, the front of the car at -Z, the gearbox behind the flywheel at +Z. The V stands upright, each
bank 36 deg off vertical. Cylinders 1-5 are bank A (toward -X), front to back, 6-10 bank B (toward
+X); cylinder n+5 shares cylinder n's pin, behind it along the crank.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import CRANK, BLOCK, Donor, build_crank, main_shells, build_block, write_engine, primary_meta  # noqa: E402

# ---- Lexus LFA 1LR-GUE numbers ---------------------------------------------------------------
BORE = 88.0
STROKE = 79.0
THROW = STROKE / 2.0
ROD_L = 136.0           # estimate, rod / stroke 1.72
V_ANGLE = 72.0
PIN_PITCH = 104.0       # crankpin to crankpin along the crank (= bore spacing in a bank), estimate
ROD_PITCH = 30.0        # the two big ends side by side on one pin (the scaled donor rod is ~29 mm wide)
FIRING = (1, 6, 5, 10, 2, 7, 3, 8, 4, 9)
REST_LOCAL1 = 30.0      # rest pose: cylinder 1 is 30 deg past TDC
SQUISH = 1.0

# bank angles: bore axis = R_z(bank) * (0, -1); bank B is V_ANGLE round from bank A, so on a shared pin
# the B cylinder reaches TDC V_ANGLE after its A partner
BANK_A = -V_ANGLE / 2.0
BANK_B = +V_ANGLE / 2.0

# the LFA's cam covers: red crackle paint, see-through like the block
LFA_RED = (0.60, 0.04, 0.05, 0.32, 0.1, 0.55, 1)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "lfa.mesh")
    donor = Donor(src)
    mats = list(donor.mats) + [LFA_RED]
    MAT_RED = len(mats) - 1
    MAT_CRANK = 2
    f = donor.fit(BORE, THROW, ROD_L)
    assert 2.0 * f.rod_half_w < ROD_PITCH, "big ends overlap on the shared pin"

    n = len(FIRING)
    lag = [0.0] * n
    for k, cn in enumerate(FIRING):
        lag[cn - 1] = 720.0 / n * k
    for i in range(5):   # every shared pin: its B cylinder fires the V angle after its A cylinder
        assert abs((lag[i + 5] - lag[i]) % 360.0 - V_ANGLE) < 1e-6, "firing order does not fit shared pins"

    # ---- cylinders ---------------------------------------------------------------------------
    pin_z = [(-2.0 + i) * PIN_PITCH for i in range(5)]
    theta_a = [(REST_LOCAL1 - lag[i]) % 360.0 for i in range(5)]      # bank A local crank angle at rest
    pin_ang = [BANK_A + t for t in theta_a]                            # the pins' mesh angles
    layout = []  # (z, local crank angle at rest, bank, lag)
    for i in range(5):
        layout.append((pin_z[i] - ROD_PITCH / 2.0, theta_a[i], BANK_A, lag[i]))
    for i in range(5):
        layout.append((pin_z[i] + ROD_PITCH / 2.0, (pin_ang[i] - BANK_B) % 360.0, BANK_B, lag[i + 5]))
    deck_y = f.deck_y(SQUISH)
    out_cyls, out_groups = f.cylinders(layout, deck_y)

    # ---- crankshaft (mesh space): five pins 144 deg apart, two rods on each -------------------------
    R_MAIN = f.s * 19.2
    R_PIN = f.s * 17.0
    skirt_bdc = ROD_L - THROW - f.skirt_below
    R_CW = skirt_bdc - 6.0          # counterweights clear the piston skirts at BDC, with margin
    pins = [(pin_z[i], ROD_PITCH, pin_ang[i]) for i in range(5)]
    crank, mains, w_lo, w_hi, z_primary = build_crank(pins, THROW, R_MAIN, R_PIN, R_CW, 12.0, 52.0, 34.0)
    out_groups.append((CRANK, -1, MAT_CRANK, crank))
    out_groups += main_shells(mains, R_MAIN)

    # ---- block (see-through): a shallow dry-sump crankcase, two banks, heads and red cam covers -------
    case_r = max(R_CW, R_PIN + THROW + 30.0) + 14.0
    banks = []
    for bank in (BANK_A, BANK_B):
        zs = [cy_[0] for cy_ in out_cyls if abs(math.degrees(cy_[8]) - bank) < 1e-6]
        banks.append((bank, min(zs), max(zs)))
    block, red = build_block(banks, BORE, deck_y, case_r, (w_lo - 50.0, w_hi + 45.0), case_r + 15.0, 8.0, 95.0, 22.0, 12.0)
    out_groups.append((BLOCK, -1, 0, block))
    out_groups.append((BLOCK, -1, MAT_RED, red))

    # ---- report ---------------------------------------------------------------------------------
    disp = n * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("Lexus LFA style 72 deg V10 (1LR-GUE) from the inline-4 parts")
    print("  bore %.1f  stroke %.1f  rod %.1f  -> %.0f cc   V %.0f deg, banks %+.0f / %+.0f deg"
          % (BORE, STROKE, ROD_L, disp, V_ANGLE, BANK_A, BANK_B))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (f.s, f.L_s, ROD_L, f.k_beam))
    print("  piston crown %.1f above pin, skirt %.1f below; deck at %.1f; counterweight R %.1f (skirt at BDC %.1f)"
          % (f.crown_above, f.skirt_below, -deck_y, R_CW, skirt_bdc))
    print("  crank z %.1f .. %.1f, flywheel side at z %.1f" % (w_lo - 48.0, w_hi + 42.0, z_primary))
    for c in range(n):
        cy_ = out_cyls[c]
        print("  cyl %2d bank %s  z %+6.1f  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, "A" if c < 5 else "B", cy_[0], math.degrees(cy_[6]) % 360, lag[c]))

    size = write_engine(out_file, donor.root, mats, out_cyls, out_groups, [("banked", [1.0]), primary_meta(z_primary)])
    print("wrote %s (%.1f MB, %d groups)" % (out_file, size / 1e6, len(out_groups)))


if __name__ == "__main__":
    main()
