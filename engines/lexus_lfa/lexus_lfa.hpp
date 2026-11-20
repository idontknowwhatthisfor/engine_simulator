// Lexus LFA (2010-2012): the 1LR-GUE 4.8 L 72 deg V10 (Toyota, with Yamaha), the 6-speed ASG
// single-clutch automated manual in a rear transaxle, rear-wheel drive. Engine, sound, gearing,
// exhaust, test targets and the engine type. Mesh: build_lfa.py -> lfa.mesh (built from the inline-4
// model's parts).
#pragma once
#include "../../src/engine_type.hpp"

// 1LR-GUE: 4805 cc, 88 x 79 mm, 12.0:1, 412 kW (560 PS) at 8700 rpm, 480 N m at 6800 rpm (90% of it
// from 3700 to 9000), limiter 9000 rpm. Five shared crankpins 144 deg apart and a 72 deg V: even
// firing every 72 deg. Titanium rods and valves, forged pistons, ten individual throttle bodies, dry
// sump, a light crank and flywheel: idle to 9000 rpm in about 0.6 s, which is why the LFA has a digital
// rev counter (an analogue needle could not keep up). Values Toyota does not publish (inertia,
// friction, cam timing, rod length, runner and header lengths) are estimates.
engineParams lexusLfaParams() {
    engineParams P;                 // a car: the generic baseline, then the LFA's own values
    P.bore = 0.088;
    P.compression = 12.0;
    P.ivc = 245.0 * DEGR;           // a long intake cam for 9000 rpm: closes 65 deg after BDC
    P.evo = 488.0 * DEGR;           // exhaust opens 52 deg before BDC
    P.burn = 44.0 * DEGR;           // compact pent-roof chamber, 12:1
    P.mRecip = 0.42;                // forged piston, pin and 1/3 of the titanium rod
    P.inertia = 0.14;               // light crank, small twin-plate clutch and flywheel
    P.vManifold = 2.0e-3;           // ten short runners off the surge tank: quick to fill
    P.throttleArea = 1.25e-2;       // ten individual throttle bodies (~40 mm each)
    P.idleBase = 4.0e-5; P.idleMin = 3.0e-5; P.idleMax = 2.4e-4;  // a 4.8 L's worth of idle air
    P.idleRpm = 1000.0;
    P.starterTorque = 230.0;
    P.starterMaxRpm = 320.0;
    P.revLimit = 9000.0; P.revResume = 8700.0;
    // low friction for 9000 rpm: titanium rods and valves, DLC-coated followers, dry sump (no crank
    // windage in an oil bath): ~2.6 bar FMEP at the limiter
    P.fmepA = 1.0e5; P.fmepB = 0.10e5; P.fmepC = 0.014e5;
    // calibrated with --engine-type lfa --phystest (and --dyno) to Toyota's 480 N m at 6800 rpm and
    // 412 kW at 8700
    P.veBase = 0.705; P.vePeak = 0.355; P.veRpm = 7600.0; P.veWidth = 3800.0;
    // pressure-wave tuning (estimates): ~0.33 m from the surge tank through the throttle body and port,
    // ramming at ~7000 rpm; equal-length 5-into-1 headers each bank down to the catalysts (measured on
    // the drawn layout by setupSim)
    P.ramLen = 0.33; P.ramDia = 0.042; P.ramGain = 0.035;
    P.scavGain = 0.025;
    P.sparkLow = 10.0; P.sparkHigh = 32.0; P.sparkFrom = 1000.0; P.sparkSpan = 4500.0;
    P.fuelCutRpm = 1600.0;
    P.rpmScale = 10000.0;
    // each bank is an even inline-5: no free 1st- or 2nd-order force, only rocking couples
    P.counterweight = 0.0;
    P.blockMass = 210.0; P.blockRollInertia = 8.0;
    P.mountFreqY = 11.0; P.mountFreqRoll = 8.5;

    // the V10 scream: the firing rate at 10 x rpm / 120 (750 Hz at 9000 rpm), a hard edge from the
    // equal-length headers, the intake's howl that Yamaha piped into the cabin, the exhaust valve
    // opening from ~3700 rpm, and a light crackle on the overrun
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLength = 2.2;             // headers, catalysts, mid pipes and the triple-exit silencer
    S.pipeLowHz = 3200.0; S.pipeResHz = 190.0; S.pipeResGain = 0.6;
    S.pipeReflect = 0.5;
    S.masterHz = 13000.0;
    S.flapOpenRpm = 3700.0; S.flapSpan = 900.0;
    S.flapShutGain = 0.55;
    S.airboxHz = 230.0; S.intakeHowlHz = 950.0; S.intakeGain = 1.0;
    S.popRpm = 3000.0; S.popChance = 0.4; 
    S.whineGain = 0.0;
    S.drive = 0.163;
    S.radiateHz = 380.0;
    S.burble = 2.2;
    S.burbleFadeFrom = 3500.0; S.burbleFadeSpan = 3000.0; S.burbleHigh = 0.4;
    S.tick = 0.12;
    // ten cylinders' exhaust strokes overlap (about three at once), so the summed outflow hardly drops
    // between pulses and the turbulent rasp, which follows it squared, is strong. 0.4, as on the
    // tuned V4: the straight-pipe car keeps a rasp like the other straight pipes; this car's catalysts
    // and silencer take most of it (src/sound.hpp raspThrough). It was an eighth when the rasp skipped
    // the silencers
    S.raspGain = 0.4;
    return P;
}

// 6-speed ASG (Aisin): an automated single-clutch manual in the rear transaxle. Toyota's ratios
// 3.231 / 2.188 / 1.609 / 1.233 / 0.970 / 0.795, final drive 4.556 (6th at the 9000 rpm limiter:
// ~323 km/h, the LFA's 325 km/h top speed); reverse an estimate. Rear 305/30 ZR20 tyres (0.345 m),
// 1480 kg plus a 75 kg driver. The ASG's fastest change is ~0.15 s: quicker than a manual, slower than
// a dual clutch (the drive is interrupted while the one clutch is open).
drivetrainParams lexusLfaDriveParams() {
    drivetrainParams P;
    P.ratios = {0.0, 3.231, 2.188, 1.609, 1.233, 0.970, 0.795};
    P.gears = 6;
    P.reverseRatio = 3.25;
    P.finalDrive = 4.556;
    P.wheelRadius = 0.345;
    P.carMass = 1555.0;
    P.wheelInertia = 4.2;           // 20 in forged wheels, carbon-ceramic discs
    P.efficiency = 0.90;            // torque tube to a rear transaxle
    P.cdA = 0.59;                   // Cd 0.31
    P.crr = 0.012;
    P.clutchCapacity = 720.0;       // twin-plate dry clutch
    P.clutchStiffness = 600.0;
    P.inputInertia = 0.025;
    P.brakeDecel = 10.5;
    P.shiftOpen = 0.05; P.shiftClose = 0.10;  // the ASG at its quickest setting
    P.dualClutch = false;
    P.normalDown = 1500.0;
    P.sportUp = 8800.0; P.sportDown = 6000.0;
    P.matchHold = 0.3;
    P.openRpm = 1100.0; P.idleOpenRpm = 1200.0; P.biteSpan = 1600.0;
    P.launchPedal = 3200.0;
    P.upBase = 2500.0; P.upPedal = 6300.0;  // full pedal: change up at 8800, just under the 9000 limiter
    P.upMargin = 300.0;
    P.downMax = 8400.0; P.kickRpm = 6500.0; P.kickMax = 8300.0; P.loadDownMax = 7000.0;
    P.pickRpm = 3000.0; P.pickMax = 8000.0;
    return P;
}

// --phystest targets (src/test_car.hpp): Toyota's figures, with tolerance
carSpec lexusLfaCar() {
    carSpec S;
    S.name = "Lexus LFA 1LR-GUE V10";
    S.dispLo = 4795; S.dispHi = 4815;
    S.crankLo = 150; S.crankHi = 380;
    S.flareLo = 1100; S.flareHi = 2800;
    S.idleLo = 920; S.idleHi = 1080;
    S.idleMapLo = 20; S.idleMapHi = 55;
    S.toLimiterLo = 0.3; S.toLimiterHi = 1.2;  // Toyota: idle to 9000 rpm in 0.6 s
    S.liftLo = 850; S.liftHi = 1250;
    // the light crank against a 4.8 L's pumping losses: it stops quicker than a road car's
    S.coastLo = 0.4; S.coastHi = 3.0;
    S.sweepTo = 9000;
    S.torqueWhat = "peak torque (Toyota: 480 N m at 6800)"; S.torqueLo = 460; S.torqueHi = 500;
    S.torqueRpmLo = 6000; S.torqueRpmHi = 7500;
    S.powerWhat = "peak power (Toyota: 412 kW / 560 PS at 8700)"; S.powerLo = 398; S.powerHi = 425;
    S.powerRpmLo = 8000; S.powerRpmHi = 9000;
    // Toyota: 90% of the peak torque (432 N m) from 3700 to 9000 rpm
    S.torqueAt = {{4000, 415, 480}, {5000, 430, 495}, {8500, 420, 480}};
    S.powerAt = {{8500, 395, 425}, {9000, 385, 425}};
    S.carWhat = "1555 kg with driver, 6-speed ASG, rear-wheel drive";
    S.zeroTo100What = "0-100 km/h, no launch control"; S.zeroTo100Lo = 3.5; S.zeroTo100Hi = 4.6;
    // the simulator has no tyre grip limit, so a rear-drive launch comes out a few tenths quicker
    // than the real car manages on its 305 mm rears
    S.launchTo100What = "0-100 km/h, launch control (Lexus: 3.7 s; no grip limit)"; S.launchTo100Lo = 3.0; S.launchTo100Hi = 4.1;
    S.zeroTo200What = "0-200 km/h (tests: ~11 s)"; S.zeroTo200Lo = 9.5; S.zeroTo200Hi = 12.5;
    S.launchLo = 3000; S.launchHi = 7000;
    S.wotUpLo = 8400;
    S.lightUpLo = 2400; S.lightUpHi = 4000;
    S.hillPedal = 0.35; S.hillGrade = 0.25; S.flatGearLo = 3; S.flatGearHi = 6;
    S.coastDownLo = 1350; S.coastDownHi = 1750;
    S.dumpStalls = true;
    S.reverseLo = -45; S.reverseHi = -5;
    return S;
}

// exhaust and intake, mesh space (mm, "up" is -y, the front of the car at -z, the gearbox at +z).
// The banks lean out 36 deg each way, bank A (cylinders 1-5) toward -x, bank B (6-10) toward +x; the
// exhaust ports are on the outer sides and the intake in the valley, as on the 1LR-GUE. Each bank's
// equal-length 5-into-1 header drops down the side of the block to a collector and a catalyst, the
// two mid pipes run back under the car (the left one through the exhaust valve) into one transverse
// silencer, and the LFA's three tailpipes leave it in a triangle in the middle of the rear. Intake: a
// surge tank over the V, ten throttle bodies on short runners into the ports, fed from the front.
//
// straight: the tuned car's straight-pipe system (--tuned yes). The same headers and collectors, no
// catalysts, silencer or exhaust valve: the two banks meet in an X-pipe under the gearbox (where their
// pulses now reflect), then two straight titanium pipes run back to two big blued tips.
exhaustLayout lexusLfaExhaustWith(const engineModel &e, bool straight) {
    exhaustLayout L;
    const std::array<float, 4> STEEL{0.58f, 0.56f, 0.53f, 1.f}, TI{0.52f, 0.50f, 0.56f, 1.f}, ALU{0.62f, 0.62f, 0.60f, 1.f},
        BLACK{0.08f, 0.08f, 0.09f, 1.f}, CARBON{0.10f, 0.10f, 0.11f, 1.f}, TIP{0.14f, 0.14f, 0.15f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false, int cyl = -1) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.from = from; p.rgba = c; p.cyl = cyl;
        p.rough = 0.4f; p.metal = 0.85f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    const int N = static_cast<int>(e.cyls.size());
    const float BORE = e.cyls[0].bore, DECK = e.cyls[0].deckY;
    const float LINE_Y = 250.f;     // height of the catalysts and mid pipes
    float zLo = 1e9f, zHi = -1e9f;
    for (const cylinderGeom &g : e.cyls) { zLo = std::min(zLo, g.zCentre); zHi = std::max(zHi, g.zCentre); }

    // headers: per bank, out of the outer side of the head, down the block and together at a collector
    // beside the back of the sump. Each cylinder's runner starts at its own z and finishes at the
    // same point, so they come out about the same length
    std::vector<int> cats;
    for (int b = 0; b < 2; b++) {
        const float sx = b == 0 ? -1.f : 1.f;   // bank A toward -x, B toward +x
        const vec3 MERGE{sx * 245.f, 175.f, zHi + 70.f};
        std::vector<int> runners;
        for (int c = 0; c < N; c++) {
            if ((e.cyls[c].bank < 0.f) != (b == 0)) continue;
            float z = e.cyls[c].zCentre;
            vec3 port = cylPoint(e, c, sx * (0.5f * BORE + 20.f), DECK - 45.f);
            vec3 out = port + cylDir(e, c, sx, 0.25f) * 45.f;
            runners.push_back(pipe({port, out, {sx * 300.f, -40.f, z}, {sx * 285.f, 70.f, 0.6f * z + 0.4f * MERGE.z},
                                    {MERGE.x, MERGE.y - 45.f, 0.2f * z + 0.8f * MERGE.z}, MERGE},
                                   19.f, 20.f, {}, STEEL, false, c));
        }
        int col = pipe({MERGE, {MERGE.x, MERGE.y + 40.f, MERGE.z + 40.f}, {sx * 230.f, LINE_Y, MERGE.z + 130.f}}, 30.f, 32.f, runners, STEEL);
        if (straight) { cats.push_back(col); continue; }
        cats.push_back(pipe({{sx * 230.f, LINE_Y, MERGE.z + 125.f}, {sx * 230.f, LINE_Y, MERGE.z + 290.f}}, 58.f, 58.f, {col}, ALU, true));
        L.pipes.back().acoustic = pipeAcoustic::catalyst;
    }
    const float MUF_Z = 1150.f, MUF_Y = 200.f;
    if (straight) {
        const std::array<float, 4> BLUED{0.40f, 0.38f, 0.56f, 1.f};
        const float X0 = zHi + 70.f + 330.f, X1 = X0 + 90.f;    // the X-pipe's merge, along the car
        std::vector<int> in;
        for (int b = 0; b < 2; b++) {
            const float sx = b == 0 ? -1.f : 1.f;
            float z0 = zHi + 70.f + 125.f;
            in.push_back(pipe({{sx * 230.f, LINE_Y, z0}, {sx * 220.f, LINE_Y, z0 + 90.f}, {sx * 60.f, LINE_Y, X0 - 30.f}, {0.f, LINE_Y, X0}},
                              34.f, 36.f, {cats[b]}, TI));
        }
        int x = pipe({{0.f, LINE_Y, X0}, {0.f, LINE_Y, X1}}, 46.f, 46.f, in, TI);
        for (int b = 0; b < 2; b++) {
            const float sx = b == 0 ? -1.f : 1.f;
            vec3 tip{sx * 75.f, MUF_Y + 35.f, MUF_Z + 150.f};
            int run = pipe({{0.f, LINE_Y, X1}, {sx * 70.f, LINE_Y, X1 + 110.f}, {sx * 75.f, LINE_Y - 5.f, MUF_Z - 200.f}, {tip.x, tip.y, tip.z - 40.f}},
                           36.f, 36.f, {x}, TI);
            pipe({{tip.x, tip.y, tip.z - 45.f}, {tip.x, tip.y, tip.z + 30.f}}, 52.f, 54.f, {run}, BLUED, true);
            L.pipes.back().metal = 0.95f; L.pipes.back().rough = 0.25f;
            L.outlets.push_back({{tip.x, tip.y, tip.z + 30.f}, {0.f, 0.f, 1.f}, 50.f});
        }
    } else {
        // mid pipes back to the silencer, the left one through the exhaust valve
        std::vector<int> mids;
        for (int b = 0; b < 2; b++) {
            const float sx = b == 0 ? -1.f : 1.f;
            float z0 = zHi + 70.f + 285.f;
            mids.push_back(pipe({{sx * 230.f, LINE_Y, z0}, {sx * 230.f, LINE_Y, z0 + 300.f}, {sx * 200.f, LINE_Y - 10.f, MUF_Z - 160.f},
                                 {sx * 170.f, MUF_Y, MUF_Z - 95.f}}, 33.f, 33.f, {cats[b]}, TI));
        }
        // the transverse silencer, across the car
        int muf = pipe({{-290.f, MUF_Y, MUF_Z}, {290.f, MUF_Y, MUF_Z}}, 100.f, 100.f, mids, TI, true);
        // the three tailpipes in a triangle: two below, one above, in the middle of the rear
        for (const vec3 &d : {vec3{-48.f, 30.f, 0.f}, vec3{48.f, 30.f, 0.f}, vec3{0.f, -52.f, 0.f}}) {
            vec3 a{d.x, MUF_Y + d.y, MUF_Z + 80.f}, c{d.x, MUF_Y + d.y, MUF_Z + 190.f};
            int link = pipe({a, c}, 28.f, 28.f, {muf}, TI);
            pipe({{c.x, c.y, c.z - 5.f}, {c.x, c.y, c.z + 40.f}}, 38.f, 38.f, {link}, TIP, true);
            L.outlets.push_back({{c.x, c.y, c.z + 40.f}, {0.f, 0.f, 1.f}, 34.f});
        }
        L.flap.on = true;
        L.flap.at = {-230.f, LINE_Y, zHi + 70.f + 430.f};
        L.flap.axis = {0.f, 0.f, 1.f};
        L.flap.shaft = {-1.f, 0.f, 0.f};
        L.flap.pipeR = 33.f;
    }

    // intake: the carbon surge tank over the V, ten short runners and throttle bodies into the valley-side
    // ports, and the inlet duct from the front of the tank to an air filter box ahead of the engine
    const float TANK_Y0 = -440.f, TANK_Y1 = -340.f;
    L.boxes.push_back({{-95.f, TANK_Y0, zLo - 70.f}, {95.f, TANK_Y1, zHi + 70.f}, CARBON, 0.5f, 0.2f});
    for (int c = 0; c < N; c++) {
        const float sx = e.cyls[c].bank < 0.f ? 1.f : -1.f;  // the valley side of this head (local x)
        vec3 port = cylPoint(e, c, sx * (0.5f * BORE + 4.f), DECK - 50.f);
        vec3 top{-sx * 55.f, TANK_Y1 + 5.f, e.cyls[c].zCentre};   // bank A's runner comes from +x of the tank
        vec3 mid = (port + top) * 0.5f;
        int r = pipe({top, mid, port}, 21.f, 20.f, {}, BLACK);
        L.pipes[r].rough = 0.5f; L.pipes[r].metal = 0.4f;
        L.boxes.push_back({{mid.x - 24.f, mid.y - 24.f, mid.z - 24.f}, {mid.x + 24.f, mid.y + 24.f, mid.z + 24.f}, ALU, 0.4f, 0.7f});
    }
    int duct = pipe({{0.f, 0.5f * (TANK_Y0 + TANK_Y1), zLo - 68.f}, {0.f, 0.5f * (TANK_Y0 + TANK_Y1), zLo - 180.f},
                     {0.f, -330.f, zLo - 280.f}}, 55.f, 55.f, {}, CARBON);
    L.pipes[duct].rough = 0.5f; L.pipes[duct].metal = 0.2f;
    L.boxes.push_back({{-150.f, -420.f, zLo - 420.f}, {150.f, -240.f, zLo - 280.f}, BLACK, 0.85f, 0.f});
    return L;
}
exhaustLayout lexusLfaExhaust(const engineModel &e) { return lexusLfaExhaustWith(e, false); }
exhaustLayout lexusLfaStraightExhaust(const engineModel &e) { return lexusLfaExhaustWith(e, true); }

const engineType &lexusLfaTunedType();  // --tuned yes, below

const engineType &lexusLfaType() {
    static const engineType T = [] {
        engineType t;
        t.id = "lfa";
        t.aliases = "lexus lexus-lfa v10 1lr-gue 1lr";
        t.name = "Lexus LFA";
        t.summary = "Lexus LFA 1LR-GUE 72 deg V10, 4805 cc, 9000 rpm, 6-speed ASG, rear-wheel drive";
        t.label = "Lexus LFA 1LR-GUE V10, 4805 cc";
        t.firingLabel = "1-6-5-10-2-7-3-8-4-9";
        t.windowTitle = "Lexus LFA 1LR-GUE V10";
        t.credit = "LFA V10 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/lexus_lfa/lfa.mesh";
        t.buildHint = "python3 -I engines/lexus_lfa/build_lfa.py";
        t.bike = false;
        t.engine = lexusLfaParams;
        t.drive = lexusLfaDriveParams;
        t.physTest = physicsTestCar;
        t.car = lexusLfaCar;
        t.exhaust = lexusLfaExhaust;
        t.hudFiringLine = true;
        t.variants = {{"tuned", lexusLfaTunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: straight-pipe exhaust and an ECU tune ----------------------------------------------
// What a typical straight-pipe LFA does (estimates, not one tuner's car); the engine keeps its 88 x 79 mm,
// 4805 cc, 12.0:1 and its internals:
//   - straight pipes: the catalysts, the silencer and the exhaust valve out, the banks crossed in an
//     X-pipe, two straight titanium pipes to two big tips. Less back pressure, so the top end breathes;
//   - an ECU map: a little more advance at the top, the limiter from 9000 to 9500 rpm (the titanium
//     valvetrain has the margin; the Nurburgring Edition's tune gave 571 PS);
//   - a gearbox map: full-throttle changes at ~9300 rpm. Ratios stay stock.
engineParams lexusLfaTunedParams() {
    engineParams P = lexusLfaParams();
    P.revLimit = 9500.0; P.revResume = 9200.0;
    P.rpmScale = 10000.0;
    P.pExhaust = 102500.0;          // no catalysts or silencer: almost no back pressure
    P.veRpm = 7900.0;               // breathes best higher up; refitted on the dyno
    P.vePeak = 0.37;
    P.sparkHigh = 33.0;
    // straight pipe: no silencer and no valve, so it is always "open", rings harder, louder, more
    // crackle. The rasp stays at the stock car's 0.4 (a full-strength rasp drowns the V10's note)
    soundParams &S = P.sound;
    S.flapOpenRpm = -1e6; S.flapShutGain = 1.0;  // nothing to shut
    S.pipeLength = 2.4;             // valve to the tips, no silencer volume in between
    S.pipeReflect = 0.68; S.pipeDampHz = 5000.0;
    S.pipeLowHz = 4200.0;
    S.masterHz = 15000.0;
    S.radiateHz = 330.0;
    S.drive = 0.1194;
    // the low burble: below the stock car's 2.2, since nothing muffles it now (the stock valve is shut
    // at low revs); at 2.2+ the decel boom under ~3500 rpm came out louder than full throttle
    S.burble = 1.6;
    S.popRpm = 2500.0; S.popChance = 0.9;
    S.intakeGain = 1.1;
    return P;
}

drivetrainParams lexusLfaTunedDriveParams() {
    drivetrainParams P = lexusLfaDriveParams();
    P.upPedal = 6800.0;             // full pedal: change up at 9300, just under the 9500 limiter
    P.sportUp = 9300.0;
    P.downMax = 8900.0; P.kickMax = 8800.0;
    return P;
}

carSpec lexusLfaTunedCar() {
    carSpec S = lexusLfaCar();
    S.name = "Lexus LFA 1LR-GUE V10 (straight pipe)";
    S.sweepTo = 9500;
    S.torqueWhat = "peak torque (straight pipe)"; S.torqueLo = 470; S.torqueHi = 520;
    S.torqueRpmLo = 6000; S.torqueRpmHi = 8000;
    S.powerWhat = "peak power (straight pipe, ~430 kW)"; S.powerLo = 420; S.powerHi = 450;
    S.powerRpmLo = 8500; S.powerRpmHi = 9500;
    S.torqueAt = {{4000, 400, 480}, {5000, 425, 500}, {9000, 420, 490}};
    S.powerAt = {{9000, 405, 450}, {9500, 395, 450}};
    S.carWhat = "1555 kg with driver, 6-speed ASG (tuned), rear-wheel drive, straight pipe";
    S.zeroTo100What = "0-100 km/h, no launch control (straight pipe)"; S.zeroTo100Lo = 3.3; S.zeroTo100Hi = 4.5;
    S.launchTo100What = "0-100 km/h, launch control (straight pipe; no grip limit)"; S.launchTo100Lo = 2.9; S.launchTo100Hi = 4.0;
    S.zeroTo200What = "0-200 km/h (straight pipe)"; S.zeroTo200Lo = 9.0; S.zeroTo200Hi = 12.0;
    S.wotUpLo = 8900;
    return S;
}

const engineType &lexusLfaTunedType() {
    static const engineType T = [] {
        engineType t = lexusLfaType();
        t.id = "lfa-tuned";
        t.name = "Lexus LFA (straight pipe)";
        t.summary = "Lexus LFA 1LR-GUE V10, straight-pipe build: no cats or silencer, X-pipe, ECU tune, 9500 limiter";
        t.label = "Lexus LFA 1LR-GUE V10, 4805 cc, STRAIGHT PIPE";
        t.windowTitle = "Lexus LFA 1LR-GUE V10 (straight pipe)";
        t.hudTag = "STRAIGHT PIPE";
        t.engine = lexusLfaTunedParams;
        t.drive = lexusLfaTunedDriveParams;
        t.car = lexusLfaTunedCar;
        t.exhaust = lexusLfaStraightExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(lexusLfaType, 17);
