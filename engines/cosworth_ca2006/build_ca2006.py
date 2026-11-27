#!/usr/bin/env python3
"""Build a Cosworth CA2006 style 90 degree flat-plane V8 (the 2006 2.4 L Formula 1 engine) from the
inline-4 parts.

Standard library only. Usage:
    python3 -I engines/cosworth_ca2006/build_ca2006.py [i4_mesh] [out_file]
    (defaults: engines/inline4_fit/engine.mesh -> engines/cosworth_ca2006/ca2006.mesh)

As the Lexus LFA's V10 (engines/lexus_lfa/build_lfa.py): the inline-4's piston and rod, scaled to the
98 mm bore and the rod length, on a procedural crank, crankcase, two banks, heads and cam covers
(tools/meshlib.py's Donor, build_crank and build_block).

CA2006 (Cosworth, via Race Engine Technology, summarised in Grand Prix Engines Note 108): 2399 cc,
98 x 39.75 mm (the 2006 rules' 90 deg V8 and 98 mm maximum bore), 13.3:1, connecting rod 2.570 x the
stroke = 102.2 mm. A flat-plane crank (four pins, the outer two at 0 deg and the inner two at 180),
each pin shared by one rod from each bank: each bank is an inline-4 firing every 180 deg, and the
90 deg V puts the other bank's cylinder half way, so the V8 fires evenly every 90 deg. Estimates
(not published): the bore spacing 106 mm, the firing order 1-5-3-7-4-8-2-6 (the usual flat-plane V8
order; with this numbering every shared pin fires its B cylinder 90 deg after its A cylinder).

Layout (mesh space, mm, the car conventions of engines/audi_rs3/): crank axis = Z through (0, 0), "up"
is -Y, the front of the car at -Z, the gearbox behind at +Z. The V stands upright, each bank 45 deg
off vertical. Cylinders 1-4 are bank A (toward -X), front to back, 5-8 bank B (toward +X); cylinder
n+4 shares cylinder n's pin, behind it along the crank.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import CRANK, BLOCK, Donor, build_crank, main_shells, build_block, write_engine, primary_meta  # noqa: E402

# ---- Cosworth CA2006 numbers -------------------------------------------------------------------
BORE = 98.0
STROKE = 39.75
THROW = STROKE / 2.0
ROD_L = 2.570 * STROKE  # Cosworth: rod / stroke 2.570 -> 102.2 mm
V_ANGLE = 90.0
PIN_PITCH = 106.0       # crankpin to crankpin along the crank (= bore spacing in a bank), estimate
ROD_PITCH = 34.0        # the two big ends side by side on one pin
FIRING = (1, 5, 3, 7, 4, 8, 2, 6)
REST_LOCAL1 = 30.0      # rest pose: cylinder 1 is 30 deg past TDC
SQUISH = 0.8

BANK_A = -V_ANGLE / 2.0
BANK_B = +V_ANGLE / 2.0

# Cosworth's cam covers: blue, see-through like the block
COSWORTH_BLUE = (0.06, 0.22, 0.62, 0.32, 0.2, 0.45, 1)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "inline4_fit", "engine.mesh")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "ca2006.mesh")
    donor = Donor(src)
    mats = list(donor.mats) + [COSWORTH_BLUE]
    MAT_BLUE = len(mats) - 1
    MAT_CRANK = 2
    f = donor.fit(BORE, THROW, ROD_L)
    assert 2.0 * f.rod_half_w < ROD_PITCH, "big ends overlap on the shared pin"

    n = len(FIRING)
    lag = [0.0] * n
    for k, cn in enumerate(FIRING):
        lag[cn - 1] = 720.0 / n * k
    for i in range(4):   # every shared pin: its B cylinder fires the V angle after its A cylinder
        assert abs((lag[i + 4] - lag[i]) % 360.0 - V_ANGLE) < 1e-6, "firing order does not fit shared pins"

    # ---- cylinders ---------------------------------------------------------------------------
    pin_z = [(-1.5 + i) * PIN_PITCH for i in range(4)]
    theta_a = [(REST_LOCAL1 - lag[i]) % 360.0 for i in range(4)]      # bank A local crank angle at rest
    pin_ang = [BANK_A + t for t in theta_a]                            # the pins' mesh angles
    # flat plane: the outer pins together, the inner pair half a turn away
    assert abs(pin_ang[0] - pin_ang[3]) % 360.0 < 1e-6 and abs(pin_ang[1] - pin_ang[2]) % 360.0 < 1e-6
    assert abs((pin_ang[1] - pin_ang[0]) % 360.0 - 180.0) < 1e-6
    layout = []  # (z, local crank angle at rest, bank, lag)
    for i in range(4):
        layout.append((pin_z[i] - ROD_PITCH / 2.0, theta_a[i], BANK_A, lag[i]))
    for i in range(4):
        layout.append((pin_z[i] + ROD_PITCH / 2.0, (pin_ang[i] - BANK_B) % 360.0, BANK_B, lag[i + 4]))
    deck_y = f.deck_y(SQUISH)
    out_cyls, out_groups = f.cylinders(layout, deck_y)

    # ---- crankshaft (mesh space): four pins in one plane, two rods on each ------------------------
    R_MAIN = f.s * 19.2
    R_PIN = f.s * 17.0
    skirt_bdc = ROD_L - THROW - f.skirt_below
    R_CW = skirt_bdc - 6.0          # counterweights clear the piston skirts at BDC, with margin
    pins = [(pin_z[i], ROD_PITCH, pin_ang[i]) for i in range(4)]
    crank, mains, w_lo, w_hi, z_primary = build_crank(pins, THROW, R_MAIN, R_PIN, R_CW, 10.0, 44.0, 30.0)
    out_groups.append((CRANK, -1, MAT_CRANK, crank))
    out_groups += main_shells(mains, R_MAIN)

    # ---- block (see-through): a shallow dry-sump crankcase (the crank sits as low as it can in an F1
    # car), two banks, heads and the blue cam covers -----------------------------------------------
    case_r = max(R_CW, R_PIN + THROW + 30.0) + 12.0
    banks = []
    for bank in (BANK_A, BANK_B):
        zs = [cy_[0] for cy_ in out_cyls if abs(math.degrees(cy_[8]) - bank) < 1e-6]
        banks.append((bank, min(zs), max(zs)))
    block, blue = build_block(banks, BORE, deck_y, case_r, (w_lo - 45.0, w_hi + 40.0), case_r + 6.0, 8.0, 90.0, 20.0, 10.0)
    out_groups.append((BLOCK, -1, 0, block))
    out_groups.append((BLOCK, -1, MAT_BLUE, blue))

    # ---- report ---------------------------------------------------------------------------------
    disp = n * math.pi / 4 * BORE ** 2 * STROKE / 1000.0
    print("Cosworth CA2006 style 90 deg flat-plane V8 from the inline-4 parts")
    print("  bore %.1f  stroke %.2f  rod %.1f  -> %.0f cc   V %.0f deg, banks %+.0f / %+.0f deg"
          % (BORE, STROKE, ROD_L, disp, V_ANGLE, BANK_A, BANK_B))
    print("  parts scaled x%.4f; rod beam %.1f -> %.1f mm (beam x%.3f, eyes rigid)" % (f.s, f.L_s, ROD_L, f.k_beam))
    print("  piston crown %.1f above pin, skirt %.1f below; deck at %.1f; counterweight R %.1f (skirt at BDC %.1f)"
          % (f.crown_above, f.skirt_below, -deck_y, R_CW, skirt_bdc))
    print("  crank z %.1f .. %.1f, flywheel side at z %.1f" % (w_lo - 45.0, w_hi + 40.0, z_primary))
    for c in range(n):
        cy_ = out_cyls[c]
        print("  cyl %d bank %s  z %+6.1f  rest crank %6.1f deg from TDC  fires at %3.0f deg"
              % (c + 1, "A" if c < 4 else "B", cy_[0], math.degrees(cy_[6]) % 360, lag[c]))

    size = write_engine(out_file, donor.root, mats, out_cyls, out_groups, [("banked", [1.0]), primary_meta(z_primary)])
    print("wrote %s (%.1f MB, %d groups)" % (out_file, size / 1e6, len(out_groups)))


if __name__ == "__main__":
    main()
