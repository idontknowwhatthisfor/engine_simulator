// Kawasaki Ninja ZX-10R (2021-): engine, sound, gearing, test targets, demo ride and the engine type.
// Mesh: build_zx10r.py -> zx10r.mesh (built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"
#include "../../src/exhaust_layouts.hpp"

// Kawasaki Ninja ZX-10R (2021-): 998 cc, 76.0 x 55.0 mm, 13.0:1, 149.3 kW (203 PS) at 13200 rpm and
// 114.9 N m at 11400 rpm without ram air (156.8 kW with it), redline 14000 rpm. Flat-plane crank
// firing 1-2-4-3, four 47 mm Mikuni throttle bodies with oval sub-throttles, finger-follower
// valvetrain (DLC-coated rockers), a single unswitched cam. Values not published by Kawasaki (rod,
// inertia, friction, cam timing) are estimates in line with the other superbike engines here.
engineParams kawasakiZx10rParams() {
    engineParams P = superbikeBaseParams();  // a superbike; Kawasaki's own values follow
    P.bore = 0.076;
    P.compression = 13.0;
    P.ivc = 242.0 * DEGR;           // race lobe: intake closes 62 deg after BDC
    P.evo = 482.0 * DEGR;
    P.burn = 44.0 * DEGR;           // small 76 mm bore: quick burn
    P.mRecip = 0.30;                // forged 76 mm piston, pin, 1/3 of the rod
    P.inertia = 0.032;              // forged 180 deg crank, alternator rotor, primary gear
    P.vManifold = 0.7e-3;
    P.throttleArea = 6.94e-3;       // four 47 mm throttle bodies
    P.idleRpm = 1250.0;             // Kawasaki: 1100 +-50; the bike tests idle at 1150-1350
    P.starterTorque = 55.0;         // 13.0:1: needs as much as the BMW to catch on the bike
    P.fmepC = 0.0045e5;             // finger followers: less friction at the top end
    P.revLimit = 14000.0; P.revResume = 13600.0;
    // one cam, no ShiftCam: a broad curve that peaks toward the power peak. Calibrated with
    // --engine-type zx10r --phystest (no ram air) to Kawasaki's 114.9 N m at 11400 and 149.3 kW at 13200
    P.lowCam = false;
    P.veBase = 0.606; P.vePeak = 0.491; P.veRpm = 11500.0; P.veWidth = 6000.0;
    // pressure-wave tuning (estimates): stack, 47 mm throttle body and port, ~0.20 m, ram hardest at
    // ~11500 rpm (the published torque peak); the 4-2-1 headers to the collector, 0.66 m
    // acoustically (measured on the drawn layout by setupSim: 0.60 m of pipe plus the port and the open
    // end), their dip at ~7500. Together a hole at 7500-9000 rpm (-4.5%), a hump near 5000 and one at
    // 12000 (+5%)
    P.ramLen = 0.20; P.ramDia = 0.047; P.ramGain = 0.05;
    P.scavGain = 0.035;
    P.sparkSpan = 7500.0;
    P.rpmScale = 16000.0;
    P.counterweight = 0.0;          // flat-plane inline-4: 1st order cancels by symmetry
    P.blockMass = 60.0; P.blockRollInertia = 1.1;
    // the ZX-10R's voice: the inline-4 superbike bark through a 4-2-1 titanium header into a bigger,
    // more muffled silencer than the BMW's, a strong airbox honk (the ram-air intake sits right under
    // the rider), crackle on the overrun, straight-cut gear whine
    P.sound = soundParams();        // its own voice, from the defaults
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLowHz = 2400.0; S.pipeResHz = 210.0; S.pipeResGain = 0.6;
    S.pipeLength = 1.2;
    S.masterHz = 11500.0;
    S.intakeGain = 1.2;
    S.popChance = 0.3;
    S.whineGain = 0.022;
    S.drive = 0.0793;
    S.radiateHz = 300.0;
    S.burble = 3.0;
    S.tick = 0.45;
    return P;
}

// Kawasaki Ninja ZX-10R (2021-): primary 79/47 (1.681), gears 39/15 41/19 32/17 33/20 31/21 30/23
// (2.600 2.158 1.882 1.650 1.476 1.304), chain 41/17 (2.412: the 2021 bike's bigger rear sprocket),
// 190/55 ZR17 rear (0.320 m). 207 kg wet plus an 80 kg rider. Assist and slipper clutch, KQS up/down
// quickshifter, launch control, KTRC wheelie control.
drivetrainParams kawasakiZx10rDriveParams() {
    drivetrainParams P = superbikeDriveBase();  // superbike rules; Kawasaki's numbers follow
    P.primary = 79.0 / 47.0;
    P.teethIn = {0, 15, 19, 17, 20, 21, 23};
    P.teethOut = {0, 39, 41, 32, 33, 31, 30};
    ratiosFromTeeth(P);
    P.primaryIn = 47; P.primaryOut = 79;
    P.finalDrive = 41.0 / 17.0;
    P.sprocketTeeth = 17;
    P.wheelRadius = 0.320;
    P.carMass = 287.0;
    P.cdA = 0.31;
    P.clutchCapacity = 260.0;
    P.sportUp = 13600.0; P.sportDown = 9800.0;
    P.upPedal = 9400.0;             // full throttle: change up just under the 14000 limiter
    P.launchPedal = 6000.0;         // launch control: slips the clutch higher, nearer the torque peak
    return P;
}

bikeSpec kawasakiZx10rSpec() {
    bikeSpec S = {"Kawasaki Ninja ZX-10R", 996, 1000, 14000, 14600,
            "peak torque (Kawasaki: 114.9 N m)", "peak torque rpm (Kawasaki: 11400)",
            "peak power (Kawasaki: 149.3 kW / 203 PS)", "peak power rpm (Kawasaki: 13200)",
            105, 124, 10000, 12500, 140, 158, 12500, 14000, 14400,
            {180.0, 180.0, 180.0, 180.0}, "even 180-180-180-180 (max error)",
            "ZX-10R gearing, 287 kg with rider", 12800, 14100,
            "top speed after 30 s, 6th gear (Kawasaki: ~299)", 270, 325, 2.5,
            600};  // a taller 1st than the BMW's (10.5:1 overall vs 11.8): a 25% take-off bogs deeper
    // the same tall 1st: 115 N m x 10.5 through 287 kg plus the crank and wheels peaks at ~1.05 g, short
    // of the test's 1.15 g "front in the air" mark. The wheelie control (1.05 g) still cuts in on a launch
    S.liftsFront = false;
    return S;
}

// exhaust: the inline superbike 4-2-1 layout (src/exhaust_layouts.hpp) into the ZX-10R's longer,
// fatter silencer: brushed steel body with a black heat shield look
exhaustLayout kawasakiZx10rExhaust(const engineModel &e) {
    superbike421Options o;
    o.canRgba = {0.22f, 0.22f, 0.24f, 1.f};
    o.canR0 = 58.f; o.canR1 = 54.f;
    return superbike421Exhaust(e, o);
}

const engineType &kawasakiZx10rTunedType();  // --tuned yes, below

const engineType &kawasakiZx10rType() {
    static const engineType T = [] {
        engineType t;
        t.id = "zx10r";
        t.aliases = "zx-10r ninja-zx10r ninja-zx-10r zx10 kawasaki-zx10r";
        t.name = "Kawasaki Ninja ZX-10R";
        t.summary = "Kawasaki Ninja ZX-10R inline-4, 998 cc, 180 deg crank, ZX-10R gearing and sound";
        t.label = "Kawasaki Ninja ZX-10R inline-4, 998 cc";
        t.windowTitle = "Ninja ZX-10R engine";
        t.credit = "ZX-10R built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/kawasaki_zx10r/zx10r.mesh";
        t.buildHint = "python3 -I engines/kawasaki_zx10r/build_zx10r.py";
        t.bike = true;
        t.engine = kawasakiZx10rParams;
        t.drive = kawasakiZx10rDriveParams;
        t.physTest = physicsTestBike;
        t.exhaust = kawasakiZx10rExhaust;
        t.spec = kawasakiZx10rSpec;
        t.gearboxSide = -1.f;      // forward-turning crank, front at +x: the box sits toward -x
        t.primaryModule = 2.6f * 84.f / (47.f + 79.f);  // 47/79 at the Ducati's 30/54 shaft spacing
        // --wav: revs in neutral, a launch up the box with the quickshifter, braking with rev-matched
        // quickshift downshifts, then back on the throttle
        t.ride = {
            {1.0, true, 0.0, 0, "starter"},          {3.0, false, 0.0, 0, "idle (valve shut)"},
            {3.25, false, 0.45, 0, "blip"},          {4.3, false, 0.0, 0, "idle"},
            {4.5, false, 1.0, 0, "big blip"},        {5.6, false, 0.0, 0, "falling back, crackle"},
            {7.0, false, 1.0, 0, "full throttle in N, limiter"}, {10.5, false, 0.0, 0, "lift off, back to idle"},
            {11.0, false, 0.0, 1, "into 1st"},       {20.0, false, 1.0, 0, "launch, quickshifts up the box"},
            {20.6, false, 0.0, 0, "lift off, braking", true},
            {21.4, false, 0.0, -1, "quickshift down, rev-matched", true},
            {22.2, false, 0.0, -1, "quickshift down, rev-matched", true},
            {23.0, false, 0.0, -1, "quickshift down, rev-matched", true},
            {24.0, false, 0.0, 0, "brakes off, overrun", false},
            {27.0, false, 0.6, 0, "back on the throttle", false},
        };
        t.variants = {{"tuned", kawasakiZx10rTunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: a race-kit engine ----------------------------------------------------------------------
// What a Superstock-style race build of the ZX-10RR's engine does (estimates, not one team's engine):
//   - the RR's Pankl titanium rods and lighter pistons and pins, the base Kawasaki homologates for this;
//   - Kawasaki's race kit ECU and kit cams: more lift and duration, so the engine breathes best near
//     13000 rpm, with the limiter at 15500 rpm (stock 14000, RR 14700);
//   - high-compression kit pistons (13.6:1) on race fuel: more spark and a little more torque everywhere,
//     so the mid range is no weaker than stock;
//   - a full titanium race exhaust with the exhaust valve deleted (louder, more crackle, a higher note).
// The block, throttle bodies, gearbox and gearing stay stock.
engineParams kawasakiZx10rTunedParams() {
    engineParams P = kawasakiZx10rParams();
    P.compression = 13.6;           // kit pistons, race fuel
    P.mRecip = 0.27;                // titanium rods, lighter pistons and pins
    P.ivc = 252.0 * DEGR;           // kit cam: intake closes 72 deg after BDC
    P.evo = 472.0 * DEGR;           // exhaust opens 10 deg earlier: blows down in time at 15500 rpm
    P.revLimit = 15500.0; P.revResume = 15100.0;
    P.sparkHigh = 38.0;             // race fuel: the kit map runs more advance at the top
    // calibrated with --engine-type zx10r --tuned yes --phystest (no ram air): see the README table
    // a broad curve: the kit cam's peak near 13000 without losing the stock mid range
    P.veBase = 0.617; P.vePeak = 0.555; P.veRpm = 12800.0; P.veWidth = 7500.0;  // stock's wave tuning
    soundParams &S = P.sound;       // race exhaust, valve deleted
    S.flapOpenRpm = 0.0; S.flapSpan = 1.0;
    S.flapShutGain = 1.0;
    S.popChance = 0.55;
    S.drive = 0.096;
    S.pipeLength = 1.05;            // the short race can: a shorter pipe, a higher note
    S.pipeResHz = 240.0;
    S.intakeGain = 1.4;             // open kit airbox
    return P;
}

// the race exhaust: the stock 4-2-1 route in titanium with heat-blued headers a size up (41 mm), no
// exhaust valve, and a short titanium can with a carbon end cap in place of the dark silencer
exhaustLayout kawasakiZx10rTunedExhaust(const engineModel &e) {
    exhaustLayout L = kawasakiZx10rExhaust(e);
    const std::array<float, 4> BLUED{0.45f, 0.42f, 0.55f, 1.f}, TI{0.62f, 0.60f, 0.56f, 1.f},
        CARBON{0.10f, 0.10f, 0.11f, 1.f};
    L.pipes.resize(L.pipes.size() - 2);  // drop the silencer and its end cap
    int link = static_cast<int>(L.pipes.size()) - 1;
    for (pipeSpec &p : L.pipes) {
        if (p.cyl >= 0) { p.r0 = 20.5f; p.r1 = 20.5f; p.rgba = BLUED; }  // headers
        else p.rgba = TI;
        p.rough = 0.25f; p.metal = 0.95f;
    }
    pipeSpec can;
    can.pts = {{-250.f, 205.f, 140.f}, {-345.f, 135.f, 158.f}};
    can.r0 = 45.f; can.r1 = 41.f;
    can.from = {link};
    can.rgba = TI; can.rough = 0.25f; can.metal = 0.95f;
    can.capStart = can.capEnd = true;
    L.pipes.push_back(can);
    pipeSpec cap;
    cap.pts = {{-340.f, 139.f, 158.f}, {-362.f, 123.f, 161.f}};
    cap.r0 = 41.f; cap.r1 = 29.f;
    cap.from = {static_cast<int>(L.pipes.size()) - 1};
    cap.rgba = CARBON; cap.rough = 0.5f; cap.metal = 0.2f;
    cap.capStart = true;
    L.pipes.push_back(cap);
    pipeSpec tip;
    tip.pts = {{-360.f, 125.f, 161.f}, {-372.f, 116.f, 163.f}};
    tip.r0 = 25.f; tip.r1 = 25.f;
    tip.from = {static_cast<int>(L.pipes.size()) - 1};
    tip.rgba = TI; tip.rough = 0.25f; tip.metal = 0.95f;
    tip.capStart = true;
    L.pipes.push_back(tip);
    L.outlets = {{{-372.f, 116.f, 163.f}, {-12.f, -9.f, 2.f}, 25.f}};
    L.flap.on = false;              // valve deleted
    return L;
}

drivetrainParams kawasakiZx10rTunedDriveParams() {
    drivetrainParams P = kawasakiZx10rDriveParams();
    P.upPedal = 10900.0;            // full throttle: change up just under the 15500 limiter
    P.sportUp = 15100.0; P.sportDown = 11000.0;
    return P;
}

bikeSpec kawasakiZx10rTunedSpec() {
    bikeSpec S = kawasakiZx10rSpec();
    S.name = "Kawasaki Ninja ZX-10R (race kit)";
    S.limLo = 15500; S.limHi = 16100;
    S.torqueWhat = "peak torque (race kit)";
    S.torqueRpmWhat = "peak torque rpm (race kit)";
    S.powerWhat = "peak power (race kit)";
    S.powerRpmWhat = "peak power rpm (race kit: near the 15500 limiter)";
    S.torqueLo = 112; S.torqueHi = 128; S.torqueRpmLo = 11000; S.torqueRpmHi = 13500;
    S.powerLo = 160; S.powerHi = 185; S.powerRpmLo = 14000; S.powerRpmHi = 15500;
    S.sweepTo = 15500;
    S.bikeWhat = "ZX-10R gearing, 287 kg with rider (race kit)";
    S.upshiftLo = 14500; S.upshiftHi = 15600;
    S.topWhat = "top speed after 30 s, 6th gear (race kit)";
    S.topLo = 300; S.topHi = 350;
    return S;
}

const engineType &kawasakiZx10rTunedType() {
    static const engineType T = [] {
        engineType t = kawasakiZx10rType();
        t.id = "zx10r-tuned";
        t.name = "Kawasaki Ninja ZX-10R (race kit)";
        t.summary = "Kawasaki Ninja ZX-10R, race kit: RR titanium rods, kit cams and ECU, 13.6:1, 15500 limiter, race exhaust";
        t.label = "Kawasaki Ninja ZX-10R inline-4, 998 cc, RACE KIT";
        t.windowTitle = "Ninja ZX-10R engine (race kit)";
        t.hudTag = "RACE KIT";
        t.engine = kawasakiZx10rTunedParams;
        t.drive = kawasakiZx10rTunedDriveParams;
        t.spec = kawasakiZx10rTunedSpec;
        t.exhaust = kawasakiZx10rTunedExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(kawasakiZx10rType, 45);
