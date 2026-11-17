// BMW S 1000 RR (2023-, K67 ShiftCam engine): engine, sound, gearing, test targets, demo ride and the
// engine type. Mesh: build_s1000rr.py -> s1000rr.mesh (built from the inline-4 model's parts).
#pragma once
#include "../ducati_v4/ducati_v4.hpp"  // the BMW starts from the Ducati's superbike values

// BMW S 1000 RR (2023-, K67 ShiftCam engine): 999 cc, 80.0 x 49.7 mm, 13.3:1, 154 kW (210 hp) at
// 13750 rpm, 113 N m at 11000 rpm, 14600 rpm limiter. Flat-plane crank firing 1-2-4-3, 48 mm
// throttle bodies, finger-follower valvetrain with ShiftCam: a partial-load intake lobe up to ~9000 rpm,
// then the full race lobe. Values not published by BMW (inertia, reciprocating mass, friction, cam
// timing) are estimates in line with the other superbike engines here.
engineParams bmwS1000rrParams() {
    engineParams P = ducatiV4Params();  // same class of engine; the differences follow
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
    P.sound = soundParams();        // its own voice from scratch, not the Ducati's
    soundParams &S = P.sound;
    S.superbike = true;
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
    drivetrainParams P = ducatiDriveParams();  // same bike rules; BMW's numbers follow
    P.primary = 76.0 / 46.0;
    P.teethIn = {0, 17, 22, 22, 22, 25, 23};
    P.teethOut = {0, 45, 46, 38, 33, 34, 29};
    for (int g = 1; g <= 6; g++) P.ratios[g] = double(P.teethOut[g]) / P.teethIn[g];
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


// exhaust: 4-2-1 titanium headers from the front of the inclined head, curving down in front of the
// crankcase and under the engine; 1+4 and 2+3 merge (360 deg apart in the 1-2-4-3 order), then a
// collector, a link pipe with the exhaust flap, and the short silencer rising on the right (+z) behind
// the gearbox. Mesh space: front +x, "up" -y.
exhaustLayout bmwS1000rrExhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> TI{0.60f, 0.57f, 0.52f, 1.f}, CAN{0.68f, 0.68f, 0.70f, 1.f}, CARBON{0.10f, 0.10f, 0.11f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from; p.rgba = c;
        p.rough = c == CARBON ? 0.5f : 0.3f;
        p.metal = c == CARBON ? 0.2f : 0.9f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    auto port = [&](int c) { return cylPoint(e, c, 0.5f * e.cyls[c].bore + 20.f, e.cyls[c].deckY - 41.f); };
    auto out = [&](int c) { return port(c) + cylDir(e, c, 1.f, 0.f) * 40.f; };
    // outer pair 1+4 runs wide and low to its merge (x 175); inner pair 2+3 merges higher (x 120)
    for (int c : {0, 3}) {
        float z = e.cyls[c].zCentre, sg = z < 0.f ? -1.f : 1.f;
        pipe({port(c), out(c), {235.f, -60.f, z}, {245.f, 50.f, z}, {225.f, 160.f, 110.f * sg}, {200.f, 225.f, 50.f * sg},
              {175.f, 235.f, 0.f}}, 18.f, 19.f, c, {}, TI);
    }
    for (int c : {1, 2}) {
        float z = e.cyls[c].zCentre, sg = z < 0.f ? -1.f : 1.f;
        pipe({port(c), out(c), {235.f, -60.f, z}, {245.f, 50.f, z}, {215.f, 140.f, 40.f * sg}, {165.f, 185.f, 20.f * sg},
              {120.f, 195.f, 0.f}}, 18.f, 19.f, c, {}, TI);
    }
    int a = pipe({{175.f, 235.f, 0.f}, {120.f, 250.f, 0.f}, {40.f, 243.f, 0.f}}, 24.f, 25.f, -1, {0, 1}, TI);
    int b = pipe({{120.f, 195.f, 0.f}, {75.f, 215.f, 0.f}, {40.f, 238.f, 0.f}}, 24.f, 25.f, -1, {2, 3}, TI);
    int col = pipe({{40.f, 240.f, 0.f}, {-20.f, 245.f, 0.f}}, 28.f, 30.f, -1, {a, b}, TI);
    int link = pipe({{-20.f, 245.f, 0.f}, {-150.f, 240.f, 40.f}, {-230.f, 215.f, 110.f}, {-262.f, 199.f, 143.f}}, 30.f, 30.f,
                    -1, {col}, TI);
    int can = pipe({{-250.f, 205.f, 140.f}, {-400.f, 110.f, 165.f}}, 52.f, 50.f, -1, {link}, CAN, true);
    pipe({{-395.f, 113.f, 165.f}, {-435.f, 90.f, 170.f}}, 20.f, 20.f, -1, {can}, CARBON, true);
    L.outlets.push_back({{-435.f, 90.f, 170.f}, {-40.f, -23.f, 5.f}, 20.f});
    L.flap.on = true;
    L.flap.at = {-190.f, 230.f, 75.f};
    L.flap.axis = normalize(vec3{-1.f, -0.25f, 0.8f});
    L.flap.shaft = {0.f, -1.f, 0.f};
    L.flap.pipeR = 30.f;
    return L;
}

const engineType &bmwS1000rrType() {
    static const engineType T = [] {
        engineType t;
        t.id = "s1000rr";
        t.aliases = "bmw s1000 bmw-s1000rr";
        t.name = "BMW S 1000 RR";
        t.summary = "BMW S 1000 RR inline-4, 999 cc, ShiftCam, 180 deg crank, BMW gearing and sound";
        t.label = "BMW S 1000 RR inline-4, 999 cc";
        t.firing = "1-2-4-3, 180 deg apart";
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
        t.zPrimary = 195.2f;       // build_s1000rr.py prints it
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
        return t;
    }();
    return T;
}
