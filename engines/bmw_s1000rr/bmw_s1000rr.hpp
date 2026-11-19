// BMW S 1000 RR (2023-, K67 ShiftCam engine): engine, sound, gearing, test targets, demo ride and the
// engine type. Mesh: build_s1000rr.py -> s1000rr.mesh (built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"
#include "../../src/exhaust_layouts.hpp"

// BMW S 1000 RR (2023-, K67 ShiftCam engine): 999 cc, 80.0 x 49.7 mm, 13.3:1, 154 kW (210 hp) at
// 13750 rpm, 113 N m at 11000 rpm, 14600 rpm limiter. Flat-plane crank firing 1-2-4-3, 48 mm
// throttle bodies, finger-follower valvetrain with ShiftCam: a partial-load intake lobe up to ~9000 rpm,
// then the full race lobe. Values not published by BMW (inertia, reciprocating mass, friction, cam
// timing) are estimates in line with the other superbike engines here.
engineParams bmwS1000rrParams() {
    engineParams P = superbikeBaseParams();  // a superbike; BMW's own values follow
    P.bore = 0.080;
    P.compression = 13.3;
    P.ivc = 245.0 * DEGR;           // race lobe: intake closes 65 deg after BDC
    P.evo = 480.0 * DEGR;
    P.burn = 44.0 * DEGR;           // small 80 mm bore, central plug: quick burn
    P.mRecip = 0.33;                // forged box piston + pin + 1/3 of the rod
    P.inertia = 0.034;              // one-piece forged 180 deg crank, alternator rotor, primary gear
    P.vManifold = 0.8e-3;
    P.throttleArea = 7.24e-3;       // four 48 mm throttle bodies
    P.idleRpm = 1250.0;
    P.starterTorque = 55.0;         // 13.3:1 and a heavier crank than the V4's: needs more to spin up
    P.fmepC = 0.0045e5;             // finger followers, small 80 mm bore: less friction at the top end
    P.revLimit = 14600.0; P.revResume = 14200.0;
    // race lobe peaks high; the ShiftCam partial-load lobe fills the mid-range (calibrated with
    // --s1000rr --phystest to 113 N m at ~11000 rpm and ~154 kW at 13500-14000 rpm)
    P.veBase = 0.50; P.vePeak = 0.64; P.veRpm = 11600.0; P.veWidth = 8000.0;
    P.lowCam = true;
    P.veLowBase = 0.62; P.veLowPeak = 0.38; P.veLowRpm = 7500.0; P.veLowWidth = 5000.0;
    P.sparkSpan = 7500.0;
    P.rpmScale = 16000.0;
    P.counterweight = 0.0;          // flat-plane inline-4: 1st order cancels by symmetry
    P.blockMass = 58.0; P.blockRollInertia = 1.1;
    // the S 1000 RR's voice: 4-2-1 titanium header into a short silencer with an exhaust flap,
    // airbox honk at full throttle, crackle on the overrun, straight-cut gearbox whine
    P.sound = soundParams();        // its own voice, from the defaults
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLowHz = 2600.0; S.pipeResHz = 220.0; S.pipeResGain = 0.6;
    S.masterHz = 12000.0;
    S.intakeGain = 1.0;
    S.popChance = 0.3;
    S.whineGain = 0.02;
    S.drive = 0.25;
    S.radiateHz = 300.0;
    S.burble = 3.0;
    S.tick = 0.5;
    return P;
}

// BMW S 1000 RR (2023-): primary 76/46 (1.652), gears 45/17 46/22 38/22 33/22 34/25 29/23
// (2.647 2.091 1.727 1.500 1.360 1.261), 525 chain 46/17, 190/55 ZR17 rear (0.320 m).
// 197 kg wet plus an 80 kg rider. Anti-hopping clutch, Shift Assistant Pro, DTC Wheelie.
drivetrainParams bmwDriveParams() {
    drivetrainParams P = superbikeDriveBase();  // superbike rules; BMW's numbers follow
    P.primary = 76.0 / 46.0;
    P.teethIn = {0, 17, 22, 22, 22, 25, 23};
    P.teethOut = {0, 45, 46, 38, 33, 34, 29};
    ratiosFromTeeth(P);
    P.primaryIn = 46; P.primaryOut = 76;
    P.finalDrive = 46.0 / 17.0;
    P.sprocketTeeth = 17;
    P.wheelRadius = 0.320;
    P.carMass = 277.0;
    P.cdA = 0.31;
    P.clutchCapacity = 260.0;       // even 180 deg firing: smaller torque peaks than the Twin Pulse
    P.sportUp = 14100.0; P.sportDown = 10000.0;
    P.upPedal = 9900.0;             // full throttle: change up just under the 14600 limiter
    return P;
}

bikeSpec bmwSpec() {
    return {"BMW S 1000 RR", 997, 1001, 14600, 15200,
            "peak torque (BMW: 113 N m)", "peak torque rpm (BMW: 11000)",
            "peak power (BMW: 154 kW / 210 hp)", "peak power rpm (BMW: 13750)",
            104, 122, 10000, 12000, 144, 164, 13000, 14500, 15000,
            {180.0, 180.0, 180.0, 180.0}, "even 180-180-180-180 (max error)",
            "S 1000 RR gearing, 277 kg with rider", 13200, 14700,
            "top speed after 30 s, 6th gear (BMW: 303)", 280, 315, 2.5};  // heavier crank: slower to fall
}


// exhaust: the 4-2-1 titanium system (src/exhaust_layouts.hpp): headers from the front of the inclined
// head, down in front of the crankcase and under the engine, 1+4 and 2+3 merging, a collector, the
// link pipe with the flap and the short silencer rising on the right behind the gearbox.
exhaustLayout bmwS1000rrExhaust(const engineModel &e) { return superbike421Exhaust(e); }

const engineType &bmwS1000rrTunedType();  // --tuned yes, below

const engineType &bmwS1000rrType() {
    static const engineType T = [] {
        engineType t;
        t.id = "s1000rr";
        t.aliases = "bmw s1000 bmw-s1000rr";
        t.name = "BMW S 1000 RR";
        t.summary = "BMW S 1000 RR inline-4, 999 cc, ShiftCam, 180 deg crank, BMW gearing and sound";
        t.label = "BMW S 1000 RR inline-4, 999 cc";
        t.windowTitle = "S 1000 RR engine";
        t.credit = "S 1000 RR built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/bmw_s1000rr/s1000rr.mesh";
        t.buildHint = "python3 -I engines/bmw_s1000rr/build_s1000rr.py";
        t.bike = true;
        t.engine = bmwS1000rrParams;
        t.drive = bmwDriveParams;
        t.physTest = physicsTestBike;
        t.exhaust = bmwS1000rrExhaust;
        t.spec = bmwSpec;
        t.gearboxSide = -1.f;      // forward-turning crank, front at +x: the box sits toward -x
        t.primaryModule = 2.6f * 84.f / (46.f + 76.f);  // 46/76 at the Ducati's 30/54 shaft spacing
        // --wav: revs in neutral, a launch up the box with the quickshifter, braking with rev-matched
        // quickshift downshifts, then back on the throttle
        t.ride = {
            {1.0, true, 0.0, 0, "starter"},          {3.0, false, 0.0, 0, "idle (flap shut)"},
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
        t.variants = {{"tuned", bmwS1000rrTunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: the M 1000 RR's engine -----------------------------------------------------------
// BMW's own tune of the K67 for the M 1000 RR (2021-): 156 kW (212 hp) at 14500 rpm, 113 N m at
// 11000 rpm, 15100 rpm limiter, 13.5:1. Pankl titanium rods and two-ring Mahle box pistons (less
// reciprocating mass), reworked intake ports and a higher-lift, longer race lobe, so the engine pulls
// to the higher limiter; the ShiftCam partial-load lobe, the gearbox and the gearing stay stock here.
// On top: a titanium race exhaust with the exhaust valve deleted (louder, more crackle).
engineParams bmwS1000rrTunedParams() {
    engineParams P = bmwS1000rrParams();
    P.compression = 13.5;
    P.mRecip = 0.29;                // titanium rods, lighter two-ring pistons
    P.ivc = 250.0 * DEGR;           // longer race lobe: intake closes 70 deg after BDC
    P.evo = 475.0 * DEGR;           // exhaust opens 5 deg earlier: blows down in time at 15000 rpm
    P.revLimit = 15100.0; P.revResume = 14700.0;
    // calibrated with --engine-type s1000rr --tuned yes --phystest: ~115 N m at 11000 rpm and ~156 kW
    // at 14500, the stock mid-range kept (the ShiftCam lobe below 9000 is unchanged)
    P.veBase = 0.50; P.vePeak = 0.655; P.veRpm = 11800.0; P.veWidth = 8000.0;
    soundParams &S = P.sound;       // race exhaust, valve deleted
    S.flapOpenRpm = 0.0; S.flapSpan = 1.0;
    S.flapShutGain = 1.0;
    S.popChance = 0.5;
    S.drive = 0.3;
    S.pipeLength = 1.05;            // the short race can: a shorter pipe, a higher note
    S.pipeResHz = 240.0;
    return P;
}

// the race exhaust: the stock titanium 4-2-1 route with the valve deleted, and a short titanium can
// with a carbon end cap in place of the silencer
exhaustLayout bmwS1000rrTunedExhaust(const engineModel &e) {
    exhaustLayout L = bmwS1000rrExhaust(e);
    const std::array<float, 4> TI{0.62f, 0.60f, 0.56f, 1.f}, CARBON{0.10f, 0.10f, 0.11f, 1.f};
    L.pipes.resize(L.pipes.size() - 2);  // drop the silencer and its end cap
    int link = static_cast<int>(L.pipes.size()) - 1;
    pipeSpec can;
    can.pts = {{-250.f, 205.f, 140.f}, {-340.f, 140.f, 156.f}};
    can.r0 = 44.f; can.r1 = 40.f;
    can.from = {link};
    can.rgba = TI; can.rough = 0.25f; can.metal = 0.95f;
    can.capStart = can.capEnd = true;
    L.pipes.push_back(can);
    pipeSpec cap;
    cap.pts = {{-335.f, 144.f, 156.f}, {-356.f, 128.f, 159.f}};
    cap.r0 = 40.f; cap.r1 = 28.f;
    cap.from = {static_cast<int>(L.pipes.size()) - 1};
    cap.rgba = CARBON; cap.rough = 0.5f; cap.metal = 0.2f;
    cap.capStart = true;
    L.pipes.push_back(cap);
    pipeSpec tip;
    tip.pts = {{-354.f, 130.f, 159.f}, {-366.f, 121.f, 161.f}};
    tip.r0 = 24.f; tip.r1 = 24.f;
    tip.from = {static_cast<int>(L.pipes.size()) - 1};
    tip.rgba = TI; tip.rough = 0.25f; tip.metal = 0.95f;
    tip.capStart = true;
    L.pipes.push_back(tip);
    L.outlets = {{{-366.f, 121.f, 161.f}, {-12.f, -9.f, 2.f}, 24.f}};
    L.flap.on = false;              // valve deleted
    return L;
}

drivetrainParams bmwS1000rrTunedDriveParams() {
    drivetrainParams P = bmwDriveParams();
    P.upPedal = 10400.0;            // full throttle: change up just under the 15100 limiter
    P.sportUp = 14600.0; P.sportDown = 10500.0;
    return P;
}

bikeSpec bmwS1000rrTunedSpec() {
    bikeSpec S = bmwSpec();
    S.name = "BMW S 1000 RR (tuned, M 1000 RR engine)";
    S.limLo = 15100; S.limHi = 15700;
    S.torqueWhat = "peak torque (M 1000 RR: 113 N m)";
    S.torqueRpmWhat = "peak torque rpm (M 1000 RR: 11000)";
    S.powerWhat = "peak power (M 1000 RR: 156 kW / 212 hp)";
    S.powerRpmWhat = "peak power rpm (M 1000 RR: 14500)";
    S.torqueLo = 104; S.torqueHi = 122; S.torqueRpmLo = 10000; S.torqueRpmHi = 12500;
    S.powerLo = 150; S.powerHi = 168; S.powerRpmLo = 14000; S.powerRpmHi = 15000;
    S.sweepTo = 15500;
    S.bikeWhat = "S 1000 RR gearing, 277 kg with rider (tuned)";
    S.upshiftLo = 13700; S.upshiftHi = 15200;
    S.topWhat = "top speed after 30 s, 6th gear (tuned)";
    S.topLo = 285; S.topHi = 325;
    return S;
}

const engineType &bmwS1000rrTunedType() {
    static const engineType T = [] {
        engineType t = bmwS1000rrType();
        t.id = "s1000rr-tuned";
        t.name = "BMW S 1000 RR (tuned)";
        t.summary = "BMW S 1000 RR with the M 1000 RR's engine: titanium rods, race lobe, 15100 limiter, race exhaust";
        t.label = "BMW S 1000 RR inline-4, 999 cc, TUNED";
        t.windowTitle = "S 1000 RR engine (tuned)";
        t.hudTag = "TUNED";
        t.engine = bmwS1000rrTunedParams;
        t.drive = bmwS1000rrTunedDriveParams;
        t.spec = bmwS1000rrTunedSpec;
        t.exhaust = bmwS1000rrTunedExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(bmwS1000rrType, 30);
