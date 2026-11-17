// Ducati Desmosedici Stradale (Panigale V4): engine, gearing, test targets and the engine type.
// Mesh: build_v4.py -> v4.mesh (a V4 built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"

// Ducati Desmosedici Stradale (Panigale V4): 1103 cc, 81 x 53.5 mm, 14.0:1, 157 kW at 13000 rpm,
// 124 N m at 9500-10000 rpm, ~14000 rpm limiter, ~1250 rpm idle. Light crank, four big throttle
// bodies, long cam timing; the bike's gearbox is not modelled, so the engine runs on a stand.
engineParams ducatiV4Params() {
    engineParams P;
    P.bore = 0.081;
    P.compression = 14.0;
    P.ivc = 250.0 * DEGR;           // intake closes 70 deg after BDC: a high-rpm cam
    P.evo = 475.0 * DEGR;           // exhaust opens 65 deg before BDC
    P.blowdown = 14.0 * DEGR;
    P.burn = 45.0 * DEGR;
    P.mRecip = 0.36;                // forged 81 mm piston, pin, 1/3 of a titanium rod
    P.inertia = 0.030;              // crank, alternator rotor, primary gear, clutch basket
    P.vManifold = 0.9e-3;           // short runners and the airbox throat
    P.throttleArea = 8.5e-3;        // four ~52 mm oval throttle bodies
    P.idleBase = 1.6e-5; P.idleMin = 1.0e-5; P.idleMax = 1.0e-4;
    P.idleRpm = 1250.0;
    P.idleBand = 400.0;
    P.starterTorque = 45.0;         // starter through its reduction gears, at the crank
    P.starterMaxRpm = 600.0;
    P.revLimit = 14000.0; P.revResume = 13600.0;
    P.fmepA = 0.75e5; P.fmepB = 0.06e5; P.fmepC = 0.008e5;  // ~2.9 bar at 13000 rpm
    P.pedalRate = 10.0;             // twist grip
    // tuned intake and exhaust: soft low down, ~114% VE from 10000 to 12500 rpm (calibrated with
    // --v4 --phystest to 124 N m at 10000-11000 and 155 kW at 13000 rpm)
    P.veBase = 0.60; P.vePeak = 0.54; P.veRpm = 11200.0; P.veWidth = 6000.0;
    P.residualMin = 0.55;           // big valve overlap: lots of exhaust left in at idle
    P.sparkLow = 10.0; P.sparkHigh = 36.0; P.sparkFrom = 1300.0; P.sparkSpan = 7000.0;
    P.mapMin = 35e3;
    P.rpmScale = 16000.0;
    P.blockMass = 60.0; P.blockRollInertia = 1.2;
    P.mountFreqY = 18.0; P.mountFreqRoll = 14.0;
    P.counterweight = 0.5;
    P.fuelCutRpm = 2500.0;
    // the Panigale V4's voice. The Twin Pulse firing (two cylinders 90 deg apart, then a long gap)
    // comes from the physics and gives the off-beat, MotoGP-like rumble; the settings shape it:
    // longer headers from both banks (lower, darker pipe modes), a bigger bass body, an exhaust
    // valve, the airbox under the tank, crackle on the overrun and the Desmo valvetrain's clatter
    soundParams &S = P.sound;
    S.superbike = true;
    S.pipeLength = 1.45;            // front and rear bank headers merge under the engine, ~1.45 m to the tail
    S.pipeReflect = 0.6; S.pipeDampHz = 2800.0;
    S.pipeLowHz = 4500.0; S.pipeResHz = 150.0; S.pipeResGain = 0.2;
    S.radiateHz = 280.0;
    // the lumpy, loping idle; it gives way to the radiated howl as the revs rise, or the Twin Pulse's
    // strong cycle-rate (rpm/120) content would boom over the top end
    S.burble = 3.0;
    S.burbleFadeFrom = 2500.0; S.burbleFadeSpan = 4000.0; S.burbleHigh = 0.05;
    S.flapOpenRpm = 4500.0; S.flapShutHz = 1300.0; S.flapOpenHz = 7500.0;
    S.airboxHz = 160.0; S.intakeHowlHz = 650.0; S.intakeGain = 1.1;
    S.popRpm = 3000.0; S.popChance = 0.4;
    S.whineGain = 0.012;
    S.tick = 0.7; S.tickHz = 2400.0;  // Desmo rocker clatter
    S.masterHz = 10000.0;
    S.drive = 0.21;
    return P;
}

// Ducati Panigale V4: primary 54/30, gears 38/14 36/17 33/19 32/21 30/22 30/24, chain 41/16, 200/55 ZR17
// rear tyre. 198 kg wet plus an 80 kg rider. Wet slipper clutch, up/down quickshifter, wheelie control.
drivetrainParams ducatiDriveParams() {
    drivetrainParams P;
    P.bike = true;
    P.gears = 6;
    P.primary = 54.0 / 30.0;
    P.ratios = {0.0, 38.0 / 14.0, 36.0 / 17.0, 33.0 / 19.0, 32.0 / 21.0, 30.0 / 22.0, 30.0 / 24.0};
    P.reverseRatio = 0.0;
    P.finalDrive = 41.0 / 16.0;
    P.wheelRadius = 0.322;
    P.efficiency = 0.93;
    P.carMass = 278.0;
    P.wheelInertia = 1.4;           // both wheels, discs and the tyres
    P.crr = 0.018;
    P.cdA = 0.32;                   // rider tucked in
    P.clutchCapacity = 300.0;       // at the crank, enough to carry the Twin Pulse firing peaks
    P.inputInertia = 0.004;         // clutch hub and gearbox shafts, referred to the crank
    P.brakeDecel = 10.0;
    P.shiftOpen = 0.04; P.shiftClose = 0.07;
    P.normalDown = 3000.0;
    P.sportUp = 13500.0; P.sportDown = 9500.0;
    P.matchHold = 0.3;
    P.clutchPressRate = 8.0; P.clutchReleaseRate = 1.2;
    P.openRpm = 1500.0; P.idleOpenRpm = 1700.0; P.biteSpan = 2000.0;
    P.launchPedal = 4500.0;         // launch control: slips the clutch at ~6000-8000 rpm at full throttle
    P.upBase = 4000.0; P.upPedal = 9300.0; P.upMargin = 500.0;
    P.downMax = 12000.0; P.kickRpm = 9000.0; P.kickMax = 12500.0; P.loadDownMax = 10000.0;
    P.pickRpm = 6000.0; P.pickMax = 12000.0;
    P.slipper = 0.35;
    P.quickshiftTime = 0.05;
    P.wheelieAccel = 1.05 * 9.81;   // about where the front lifts with a rider on
    P.teethIn = {0, 14, 17, 19, 21, 22, 24};
    P.teethOut = {0, 38, 36, 33, 32, 30, 30};
    return P;
}

bikeSpec ducatiSpec() {
    return {"Ducati V4", 1100, 1106, 14000, 14600,
            "peak torque (Ducati: 124 N m)", "peak torque rpm (Ducati: 9500-10000)",
            "peak power (Ducati: 157 kW / 214 hp)", "peak power rpm (Ducati: 13000)",
            112, 136, 8500, 11000, 145, 170, 12000, 14000, 14000,
            {90.0, 200.0, 90.0, 340.0}, "Twin Pulse 90-200-90-340 (max error)",
            "Panigale V4 gearing, 278 kg with rider", 12500, 14100,
            "top speed after 30 s, 6th gear (Ducati ~299)", 270, 305, 2.0};
}


const engineType &ducatiV4Type() {
    static const engineType T = [] {
        engineType t;
        t.id = "v4";
        t.aliases = "ducati ducati-v4 panigale";
        t.name = "Ducati V4";
        t.summary = "Ducati Desmosedici Stradale 90 deg V4, 1103 cc, Twin Pulse, Panigale V4 gearing";
        t.label = "Ducati-style 90 deg V4, 1103 cc";
        t.firing = "Twin Pulse 0-90-290-380";
        t.windowTitle = "V4 engine";
        t.credit = "V4 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/ducati_v4/v4.mesh";
        t.buildHint = "python3 -I engines/ducati_v4/build_v4.py";
        t.bike = true;
        t.engine = ducatiV4Params;
        t.drive = ducatiDriveParams;
        t.physTest = physicsTestBike;
        t.spec = ducatiSpec;
        t.zPrimary = 119.f;        // build_v4.py: the crank's primary gear hub
        t.gearboxSide = 1.f;       // counter-rotating crank, front at -x: the box sits toward +x
        t.primaryModule = 2.6f;
        // --wav: start-up and blips in neutral, the limiter, a launch up the box with the quickshifter,
        // braking with rev-matched quickshift downshifts, then back on the throttle
        t.ride = {
            {1.0, true, 0.0, 0, "starter"},          {3.0, false, 0.0, 0, "idle (Twin Pulse lope, valve shut)"},
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
