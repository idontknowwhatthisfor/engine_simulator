// Ford Mustang V6 (2014, the S197 II's last year): the 3.7 L Cyclone ("Duratec 37") 60 deg V6, the
// Getrag MT82 6-speed manual, rear-wheel drive. Engine, sound, gearing, exhaust, test targets and the
// engine type. Mesh: build_mustang_v6.py -> mustang_v6.mesh (built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"

// Cyclone 3.7 (Mustang tune): 3726 cc, 95.5 x 86.7 mm, 10.5:1, 227 kW (305 hp) at 6500 rpm, 380 N m
// (280 lb ft) at 4250 rpm, 7000 rpm redline. Aluminium block and heads, DOHC 24 valves, twin
// independent variable cam timing (Ti-VCT) on both cams, port injection, a composite intake manifold,
// split-pin crank, firing 1-4-2-5-3-6 every 120 deg. Values Ford does not publish (inertia, friction,
// cam timing at full load, rod, runner and header lengths) are estimates.
engineParams mustangV6Params() {
    engineParams P;                 // a car: the generic baseline, then the Cyclone's own values
    P.bore = 0.0955;
    P.compression = 10.5;
    P.ivc = 232.0 * DEGR;           // intake closes 52 deg after BDC (Ti-VCT at full load, estimate)
    P.evo = 492.0 * DEGR;           // exhaust opens 48 deg before BDC
    P.burn = 48.0 * DEGR;           // pent-roof chamber, port injection
    P.mRecip = 0.66;                // cast piston, pin and 1/3 of the powder-metal rod
    P.inertia = 0.21;               // crank and the manual's dual-mass flywheel
    P.vManifold = 4.5e-3;           // composite manifold, a big plenum over the V
    P.throttleArea = 3.85e-3;       // ~70 mm electronic throttle
    P.idleBase = 3.2e-5; P.idleMin = 2.9e-5; P.idleMax = 2.0e-4;  // a 3.7 L's worth of idle air
    P.idleRpm = 700.0;
    P.starterTorque = 200.0;
    P.starterMaxRpm = 300.0;
    P.revLimit = 7000.0; P.revResume = 6750.0;
    P.fmepA = 1.1e5; P.fmepB = 0.12e5; P.fmepC = 0.030e5;
    // calibrated with --engine-type mustang-v6 --phystest to Ford's 380 N m at 4250 and 227 kW at 6500
    P.veBase = 0.74; P.vePeak = 0.355; P.veRpm = 5350.0; P.veWidth = 3450.0;
    // pressure-wave tuning (estimates): the composite manifold's long runners (~0.42 m plenum to
    // valve) ram in the mid range; short cast-in-style manifolds into close-coupled catalysts, so
    // little scavenging
    P.ramLen = 0.42; P.ramDia = 0.045; P.ramGain = 0.035;
    P.scavGain = 0.015;
    P.sparkLow = 10.0; P.sparkHigh = 30.0; P.sparkFrom = 700.0; P.sparkSpan = 3500.0;
    P.fuelCutRpm = 1400.0;
    P.rpmScale = 8000.0;
    // an even-firing 60 deg V6: no free 1st- or 2nd-order force, only rocking couples
    P.counterweight = 0.0;
    P.blockMass = 175.0; P.blockRollInertia = 6.5;
    P.mountFreqY = 11.0; P.mountFreqRoll = 8.5;

    // the Cyclone's voice: three pulses a revolution (350 Hz at 7000 rpm), a smooth, slightly raspy
    // V6 with a hollow intake moan through the composite manifold, true dual exhaust with an H-pipe
    // and two rear mufflers, no exhaust valve. Mild burble on the overrun, rare crackle
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLength = 3.2;             // manifolds, catalysts, mid pipes, H-pipe and the rear mufflers
    S.pipeLowHz = 2400.0; S.pipeResHz = 140.0; S.pipeResGain = 0.7;
    S.pipeReflect = 0.5;
    S.masterHz = 9500.0;
    S.flapOpenRpm = -1e6; S.flapShutGain = 1.0;  // no exhaust valve
    S.airboxHz = 180.0; S.intakeHowlHz = 700.0; S.intakeGain = 0.8;
    S.popRpm = 3000.0; S.popChance = 0.15;
    S.whineGain = 0.0;
    S.drive = 0.19;
    S.radiateHz = 260.0;
    S.burble = 2.5;
    S.burbleFadeFrom = 3500.0; S.burbleFadeSpan = 2500.0; S.burbleHigh = 0.45;
    S.tick = 0.14;
    S.raspGain = 0.36;
    return P;
}

// Getrag MT82 6-speed manual (2011-2014 Mustang V6): 3.66 / 2.43 / 1.69 / 1.32 / 1.00 / 0.65, reverse
// 3.42 (estimate), the 3.31 rear axle. P235/55 R17 tyres (0.345 m), 1600 kg (the coupe's curb weight)
// plus a 75 kg driver, Cd 0.35 on ~2.2 m^2. Driven by the simulator's automatic clutch and shift like
// every car, with a manual's slower changes.
drivetrainParams mustangV6DriveParams() {
    drivetrainParams P;
    P.ratios = {0.0, 3.66, 2.43, 1.69, 1.32, 1.00, 0.65};
    P.gears = 6;
    P.reverseRatio = 3.42;
    P.finalDrive = 3.31;
    P.wheelRadius = 0.345;
    P.carMass = 1675.0;
    P.wheelInertia = 4.4;           // 17 in alloys and a solid rear axle
    P.efficiency = 0.90;
    P.cdA = 0.77;
    P.crr = 0.012;
    P.clutchCapacity = 560.0;
    P.clutchStiffness = 500.0;
    P.inputInertia = 0.03;
    P.brakeDecel = 9.5;
    P.shiftOpen = 0.08; P.shiftClose = 0.22;  // a quick hand on an H-pattern box
    P.dualClutch = false;
    P.normalDown = 1300.0;
    P.sportUp = 6800.0; P.sportDown = 4500.0;
    P.matchHold = 0.4;
    P.openRpm = 1000.0; P.idleOpenRpm = 1100.0; P.biteSpan = 1400.0;
    P.launchPedal = 2200.0;
    P.upBase = 2000.0; P.upPedal = 4800.0;  // full pedal: change up at 6800, just under the 7000 limiter
    P.upMargin = 300.0;
    P.downMax = 6300.0; P.kickRpm = 4500.0; P.kickMax = 6200.0; P.loadDownMax = 5000.0;
    P.pickRpm = 2500.0; P.pickMax = 6000.0;
    return P;
}

// --phystest targets (src/test_car.hpp): Ford's figures and road tests, with tolerance
carSpec mustangV6Car() {
    carSpec S;
    S.name = "Ford Mustang V6 3.7 Cyclone";
    S.dispLo = 3715; S.dispHi = 3735;
    S.flareLo = 900; S.flareHi = 2000;
    S.idleLo = 640; S.idleHi = 780;
    S.idleMapLo = 20; S.idleMapHi = 50;
    S.toLimiterLo = 0.5; S.toLimiterHi = 2.5;
    S.liftLo = 620; S.liftHi = 950;
    S.sweepTo = 7000;
    S.torqueWhat = "peak torque (Ford: 380 N m / 280 lb ft at 4250)"; S.torqueLo = 365; S.torqueHi = 395;
    S.torqueRpmLo = 3500; S.torqueRpmHi = 5000;
    S.powerWhat = "peak power (Ford: 227 kW / 305 hp at 6500)"; S.powerLo = 218; S.powerHi = 236;
    S.powerRpmLo = 6000; S.powerRpmHi = 7000;
    S.torqueAt = {{2000, 300, 365}, {3000, 330, 385}, {6500, 315, 360}};
    S.powerAt = {{5500, 195, 230}, {7000, 205, 236}};
    S.carWhat = "1675 kg with driver, MT82 6-speed manual, 3.31 axle, rear-wheel drive";
    // road tests: 0-60 mph 5.4-5.9 s with the manual; the simulator has no grip limit
    S.zeroTo100What = "0-100 km/h (tests: ~5.8-6.2 s)"; S.zeroTo100Lo = 5.0; S.zeroTo100Hi = 6.8;
    S.zeroTo200What = "0-200 km/h (~20 s)"; S.zeroTo200Lo = 16.0; S.zeroTo200Hi = 26.0;
    S.launchLo = 1800; S.launchHi = 5000;
    S.wotUpLo = 6500;
    S.lightUpLo = 2000; S.lightUpHi = 3500;
    S.hillPedal = 0.35; S.hillGrade = 0.15; S.flatGearLo = 3; S.flatGearHi = 6;
    S.dumpStalls = true;
    S.reverseLo = -40; S.reverseHi = -5;
    return S;
}

// exhaust and intake, mesh space (mm, "up" is -y, the front of the car at -z, the gearbox at +z).
// The banks lean out 30 deg each way: the right bank (1-3) toward -x, the left (4-6) toward +x, the
// exhaust ports on the outer sides and the intake in the valley. Each bank's short 3-into-1 manifold
// runs back along the head into a close-coupled catalyst behind the bank, the two mid pipes run back
// under the gearbox, joined by an H-pipe, into two rear mufflers and a tip at each rear corner. The
// composite intake manifold sits over the V, fed from a throttle body at its front.
//
// straight: the tuned car's straight-pipe system (--tuned yes). The same manifolds, no catalysts or
// mufflers: each collector drops straight into a 2.5 in pipe, the two cross in an X-pipe under the
// gearbox (where the banks' pulses now meet and reflect), then run straight back to two big tips.
// It also carries the twin-screw supercharger (mustang_v6_blower.mesh, src/turbo.hpp) on top of the V:
// rotors along the car, the belt from the crank damper to its pulley at the front, an intercooled
// lower manifold under it in place of the composite plenum, and the throttle body on its rear inlet.
exhaustLayout mustangV6ExhaustWith(const engineModel &e, bool straight) {
    exhaustLayout L;
    const std::array<float, 4> CAST{0.34f, 0.32f, 0.30f, 1.f}, STEEL{0.55f, 0.53f, 0.50f, 1.f},
        ALU{0.62f, 0.62f, 0.60f, 1.f}, BLACK{0.08f, 0.08f, 0.09f, 1.f}, CHROME{0.80f, 0.80f, 0.82f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false, int cyl = -1) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.from = from; p.rgba = c; p.cyl = cyl;
        p.rough = 0.45f; p.metal = 0.8f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    const int N = static_cast<int>(e.cyls.size());
    const float BORE = e.cyls[0].bore, DECK = e.cyls[0].deckY;
    const float LINE_Y = 260.f;     // height of the catalysts' outlets and the mid pipes
    float zLo = 1e9f, zHi = -1e9f;
    for (const cylinderGeom &g : e.cyls) { zLo = std::min(zLo, g.zCentre); zHi = std::max(zHi, g.zCentre); }

    // manifolds: per bank, out of the outer side of the head, back along it to a collector behind the
    // bank, then down to the catalyst
    std::vector<int> cats;
    for (int b = 0; b < 2; b++) {
        const float sx = b == 0 ? -1.f : 1.f;   // right bank toward -x, left toward +x
        const vec3 MERGE{sx * 270.f, -60.f, zHi + 90.f};
        std::vector<int> runners;
        for (int c = 0; c < N; c++) {
            if ((e.cyls[c].bank < 0.f) != (b == 0)) continue;
            float z = e.cyls[c].zCentre;
            vec3 port = cylPoint(e, c, sx * (0.5f * BORE + 26.f), DECK - 50.f);
            vec3 out = port + cylDir(e, c, sx, 0.2f) * 40.f;
            runners.push_back(pipe({port, out, {sx * 290.f, -110.f, 0.7f * z + 0.3f * MERGE.z},
                                    {MERGE.x, MERGE.y - 15.f, 0.2f * z + 0.8f * MERGE.z}, MERGE},
                                   20.f, 21.f, {}, CAST, false, c));
        }
        int col = pipe({MERGE, {MERGE.x, MERGE.y + 60.f, MERGE.z + 25.f}, {sx * 255.f, 120.f, MERGE.z + 60.f}}, 30.f, 31.f, runners, CAST);
        if (straight) {             // no catalyst: the collector drops straight to the pipe line
            cats.push_back(pipe({{sx * 255.f, 115.f, MERGE.z + 58.f}, {sx * 240.f, LINE_Y, MERGE.z + 150.f}}, 32.f, 32.f, {col}, STEEL));
            continue;
        }
        cats.push_back(pipe({{sx * 255.f, 115.f, MERGE.z + 58.f}, {sx * 240.f, LINE_Y, MERGE.z + 150.f}}, 55.f, 55.f, {col}, ALU, true));
        L.pipes.back().acoustic = pipeAcoustic::catalyst;
    }
    if (straight) {
        // two 2.5 in pipes into an X-pipe under the gearbox, then straight back to two big tips
        const std::array<float, 4> BURNT{0.42f, 0.36f, 0.50f, 1.f};
        const float X0 = zHi + 600.f, X1 = X0 + 100.f, TIP_Z = zHi + 1420.f;
        std::vector<int> in;
        for (int b = 0; b < 2; b++) {
            const float sx = b == 0 ? -1.f : 1.f;
            float z0 = zHi + 90.f + 145.f;
            in.push_back(pipe({{sx * 240.f, LINE_Y, z0}, {sx * 150.f, LINE_Y + 10.f, z0 + 200.f}, {sx * 40.f, LINE_Y + 10.f, X0 - 30.f},
                               {0.f, LINE_Y + 10.f, X0}}, 32.f, 34.f, {cats[b]}, STEEL));
        }
        int x = pipe({{0.f, LINE_Y + 10.f, X0}, {0.f, LINE_Y + 10.f, X1}}, 44.f, 44.f, in, STEEL);
        for (int b = 0; b < 2; b++) {
            const float sx = b == 0 ? -1.f : 1.f;
            vec3 tip{sx * 320.f, LINE_Y + 10.f, TIP_Z};
            int run = pipe({{0.f, LINE_Y + 10.f, X1}, {sx * 60.f, LINE_Y + 10.f, X1 + 110.f}, {sx * 280.f, LINE_Y, TIP_Z - 400.f},
                            {tip.x, tip.y, tip.z - 50.f}}, 34.f, 34.f, {x}, STEEL);
            pipe({{tip.x, tip.y, tip.z - 55.f}, {tip.x, tip.y, tip.z + 30.f}}, 52.f, 54.f, {run}, BURNT, true);
            L.pipes.back().metal = 0.95f; L.pipes.back().rough = 0.25f;
            L.outlets.push_back({{tip.x, tip.y, tip.z + 30.f}, {0.f, 0.f, 1.f}, 50.f});
        }
    } else {
    // mid pipes back under the gearbox, an H-pipe between them, then the two mufflers and tips
    const float H_Z = zHi + 650.f, MUF_Z0 = zHi + 1050.f, MUF_Z1 = MUF_Z0 + 260.f;
    std::vector<int> fronts;
    for (int b = 0; b < 2; b++) {
        const float sx = b == 0 ? -1.f : 1.f;
        float z0 = zHi + 90.f + 145.f;
        fronts.push_back(pipe({{sx * 240.f, LINE_Y, z0}, {sx * 150.f, LINE_Y + 10.f, z0 + 200.f}, {sx * 120.f, LINE_Y + 10.f, H_Z}},
                              30.f, 30.f, {cats[b]}, STEEL));
    }
    int h = pipe({{-120.f, LINE_Y + 10.f, H_Z}, {120.f, LINE_Y + 10.f, H_Z}}, 28.f, 28.f, fronts, STEEL);
    for (int b = 0; b < 2; b++) {
        const float sx = b == 0 ? -1.f : 1.f;
        int rear = pipe({{sx * 120.f, LINE_Y + 10.f, H_Z}, {sx * 140.f, LINE_Y + 10.f, H_Z + 200.f}, {sx * 300.f, LINE_Y - 10.f, MUF_Z0 - 20.f}},
                        30.f, 30.f, {h}, STEEL);
        int muf = pipe({{sx * 300.f, LINE_Y - 10.f, MUF_Z0}, {sx * 300.f, LINE_Y - 10.f, MUF_Z1}}, 75.f, 75.f, {rear}, BLACK, true);
        L.pipes[muf].acoustic = pipeAcoustic::silencer;
        vec3 tip{sx * 320.f, LINE_Y + 10.f, MUF_Z1 + 110.f};
        int link = pipe({{sx * 300.f, LINE_Y - 10.f, MUF_Z1 - 5.f}, {tip.x, tip.y, tip.z - 50.f}}, 28.f, 28.f, {muf}, STEEL);
        pipe({{tip.x, tip.y, tip.z - 55.f}, {tip.x, tip.y, tip.z + 20.f}}, 44.f, 44.f, {link}, CHROME, true);
        L.pipes.back().metal = 0.95f; L.pipes.back().rough = 0.15f;
        L.outlets.push_back({{tip.x, tip.y, tip.z + 20.f}, {0.f, 0.f, 1.f}, 42.f});
    }
    }

    // intake: the composite manifold's plenum over the V (supercharged: the blower's cast lower manifold
    // with the air-to-water intercooler core in it), a runner into each valley-side port
    const float PL_Y0 = straight ? -482.f : -470.f, PL_Y1 = -370.f;
    const std::array<float, 4> COMPOSITE{0.12f, 0.12f, 0.13f, 1.f}, CASTALU{0.48f, 0.48f, 0.50f, 1.f};
    L.boxes.push_back({{-110.f, PL_Y0, zLo - 60.f}, {110.f, PL_Y1, zHi + 60.f}, straight ? CASTALU : COMPOSITE, 0.7f, straight ? 0.5f : 0.1f});
    for (int c = 0; c < N; c++) {
        const float sx = e.cyls[c].bank < 0.f ? 1.f : -1.f;  // the valley side of this head (local x)
        vec3 port = cylPoint(e, c, sx * (0.5f * BORE + 4.f), DECK - 55.f);
        vec3 top{-sx * 70.f, PL_Y1 + 5.f, e.cyls[c].zCentre};
        int r = pipe({top, (port + top) * 0.5f + vec3{0.f, -20.f, 0.f}, port}, 22.f, 21.f, {}, COMPOSITE);
        L.pipes[r].rough = 0.7f; L.pipes[r].metal = 0.1f;
    }
    if (straight) {
        // the blower (frame axes: the defaults, rotor axis +x along the car toward the gearbox)
        L.turbo.on = true;
        L.turbo.at = meshPoint(e, "supercharger_at");   // build_mustang_v6.py's BLOWER_AT
        const vec3 B = L.turbo.at;
        // intercooler core (seen through the manifold's end)
        L.boxes.push_back({{-95.f, PL_Y0 + 10.f, zLo - 45.f}, {95.f, PL_Y1 - 40.f, zHi + 45.f}, {0.10f, 0.10f, 0.11f, 1.f}, 0.8f, 0.4f});
        // its coolant loop: the front heat exchanger (a thin finned core between two end tanks, low in
        // front of the crank pulley, where the RS 3 has its intercooler), two hoses down the right side
        // (clear of the airbox on the left and the belt in the middle) and the electric pump on the return
        const float HX_Z0 = zLo - 600.f, HX_Z1 = zLo - 565.f, HX_Y0 = 40.f, HX_Y1 = 230.f, HX_L = -340.f, HX_R = 340.f;
        L.boxes.push_back({{HX_L + 40.f, HX_Y0, HX_Z0 + 5.f}, {HX_R - 40.f, HX_Y1, HX_Z1 - 5.f}, {0.10f, 0.10f, 0.11f, 1.f}, 0.8f, 0.4f});
        L.boxes.push_back({{HX_R - 40.f, HX_Y0 - 10.f, HX_Z0}, {HX_R, HX_Y1 + 10.f, HX_Z1}, ALU, 0.4f, 0.7f});
        L.boxes.push_back({{HX_L, HX_Y0 - 10.f, HX_Z0}, {HX_L + 40.f, HX_Y1 + 10.f, HX_Z1}, ALU, 0.4f, 0.7f});
        const float TANK_X = HX_R - 20.f, TOP_Y = PL_Y1 - 60.f, OUT_Z = zLo - 60.f;
        auto hose = [&](std::vector<vec3> pts) {
            int w = pipe(pts, 10.f, 10.f, {}, BLACK);
            L.pipes[w].rough = 0.9f; L.pipes[w].metal = 0.f;
        };
        // both hoses leave the manifold's front, drop straight down just ahead of the engine (clear of
        // the crank pulley and the belt) and run forward low into the tank. Control points about evenly
        // spaced: the tube's Catmull-Rom centreline overshoots where a long span meets a short one.
        // hot side, into the top of the tank
        hose({{110.f, TOP_Y, OUT_Z}, {110.f, TOP_Y, OUT_Z - 50.f}, {110.f, TOP_Y + 60.f, OUT_Z - 85.f},
              {110.f, -250.f, OUT_Z - 95.f}, {110.f, -120.f, OUT_Z - 100.f}, {110.f, 0.f, OUT_Z - 105.f},
              {120.f, 55.f, OUT_Z - 150.f}, {170.f, 60.f, OUT_Z - 240.f}, {240.f, 60.f, OUT_Z - 330.f},
              {300.f, 60.f, OUT_Z - 420.f}, {TANK_X, HX_Y0 + 20.f, HX_Z1 + 30.f}, {TANK_X, HX_Y0 + 20.f, HX_Z1 - 10.f}});
        // cold side, from the bottom of the tank through the pump on its low run and back up
        const vec3 PUMP{215.f, 195.f, OUT_Z - 330.f};
        hose({{TANK_X, HX_Y1 - 20.f, HX_Z1 - 10.f}, {TANK_X, HX_Y1 - 20.f, HX_Z1 + 30.f}, {290.f, 200.f, OUT_Z - 440.f},
              {PUMP.x + 25.f, PUMP.y, PUMP.z - 70.f}, {PUMP.x, PUMP.y, PUMP.z - 40.f}});
        int pump = pipe({{PUMP.x, PUMP.y, PUMP.z - 42.f}, {PUMP.x, PUMP.y, PUMP.z + 42.f}}, 24.f, 24.f, {}, BLACK, true);
        L.pipes[pump].rough = 0.6f; L.pipes[pump].metal = 0.3f;
        L.boxes.push_back({{PUMP.x - 18.f, PUMP.y - 42.f, PUMP.z - 20.f}, {PUMP.x + 18.f, PUMP.y - 22.f, PUMP.z + 20.f}, ALU, 0.5f, 0.6f});
        hose({{PUMP.x, PUMP.y, PUMP.z + 40.f}, {PUMP.x - 40.f, 185.f, OUT_Z - 240.f}, {110.f, 165.f, OUT_Z - 165.f},
              {60.f, 110.f, OUT_Z - 125.f}, {50.f, 0.f, OUT_Z - 110.f}, {50.f, -120.f, OUT_Z - 105.f},
              {50.f, -250.f, OUT_Z - 100.f}, {50.f, TOP_Y + 60.f, OUT_Z - 85.f}, {50.f, TOP_Y, OUT_Z - 50.f}, {50.f, TOP_Y, OUT_Z}});
        // throttle body on the rear inlet, the duct from it forward along the left to the airbox
        L.boxes.push_back({{-48.f, B.y - 48.f, B.z + 160.f}, {48.f, B.y + 48.f, B.z + 205.f}, ALU, 0.4f, 0.7f});
        int duct = pipe({{0.f, B.y, B.z + 203.f}, {0.f, B.y, B.z + 260.f}, {-150.f, B.y - 20.f, B.z + 230.f},
                         {-190.f, B.y + 10.f, zLo - 120.f}, {-160.f, -360.f, zLo - 320.f}}, 45.f, 48.f, {}, BLACK);
        L.pipes[duct].rough = 0.9f; L.pipes[duct].metal = 0.f;
        L.boxes.push_back({{-300.f, -440.f, zLo - 470.f}, {-20.f, -260.f, zLo - 310.f}, BLACK, 0.85f, 0.f});
        return L;
    }
    const float TB_Y = 0.5f * (PL_Y0 + PL_Y1);
    L.boxes.push_back({{-45.f, TB_Y - 45.f, zLo - 110.f}, {45.f, TB_Y + 45.f, zLo - 60.f}, ALU, 0.4f, 0.7f});
    int duct = pipe({{0.f, TB_Y, zLo - 108.f}, {0.f, TB_Y, zLo - 200.f}, {-120.f, -360.f, zLo - 320.f}}, 42.f, 45.f, {}, BLACK);
    L.pipes[duct].rough = 0.9f; L.pipes[duct].metal = 0.f;
    L.boxes.push_back({{-300.f, -440.f, zLo - 470.f}, {-20.f, -260.f, zLo - 310.f}, BLACK, 0.85f, 0.f});
    return L;
}
exhaustLayout mustangV6Exhaust(const engineModel &e) { return mustangV6ExhaustWith(e, false); }
exhaustLayout mustangV6StraightExhaust(const engineModel &e) { return mustangV6ExhaustWith(e, true); }

const engineType &mustangV6TunedType();  // --tuned yes, below

const engineType &mustangV6Type() {
    static const engineType T = [] {
        engineType t;
        t.id = "mustang-v6";
        t.aliases = "mustang ford ford-mustang mustang-3.7 cyclone duratec37 v6 3.7";
        t.name = "Ford Mustang V6";
        t.summary = "2014 Ford Mustang V6: 3.7 Cyclone 60 deg V6, 3726 cc, 7000 rpm, MT82 6-speed manual, rear-wheel drive";
        t.label = "Ford Mustang 3.7 Cyclone V6, 3726 cc";
        t.windowTitle = "2014 Ford Mustang V6 3.7";
        t.credit = "Mustang V6 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/ford_mustang_v6/mustang_v6.mesh";
        t.buildHint = "python3 -I engines/ford_mustang_v6/build_mustang_v6.py";
        t.bike = false;
        t.engine = mustangV6Params;
        t.drive = mustangV6DriveParams;
        t.physTest = physicsTestCar;
        t.car = mustangV6Car;
        t.exhaust = mustangV6Exhaust;
        t.hudFiringLine = true;
        t.variants = {{"tuned", mustangV6TunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: twin-screw supercharger, straight-pipe exhaust and an ECU tune ------------------------
// What a typical blown, straight-piped 3.7 Mustang does (estimates, not one tuner's kit); the engine
// keeps its 95.5 x 86.7 mm, 3726 cc, 10.5:1 and stock internals:
//   - a twin-screw supercharger (~1.5 L per turn of the male rotor) on top of the V, belted from the
//     crank at 1.9:1 (~14k rotor rpm at the limiter), on an intercooled lower manifold: ~0.2 bar by
//     2000 rpm, ~0.25-0.3 bar to the top, and a bypass valve that opens off throttle;
//   - straight pipes: the catalysts and both mufflers out, the H-pipe swapped for an X-pipe, two 2.5 in
//     pipes straight back to two big tips. Less back pressure, so the top end breathes;
//   - an ECU tune (93 octane): more advance off boost and spark pulled with it, the Ti-VCT re-phased
//     for the top end, the limiter from 7000 to 7400 rpm;
//   - full-throttle changes at ~7200 rpm. Ratios and the 3.31 axle stay stock.
// Kits like this make ~400-450 hp on the stock 3.7; with the straight pipe the sim makes ~340 kW.
engineParams mustangV6TunedParams() {
    engineParams P = mustangV6Params();
    P.revLimit = 7400.0; P.revResume = 7150.0;
    P.pExhaust = 102500.0;          // no catalysts or mufflers: almost no back pressure
    P.veRpm = 6100.0;               // breathes best higher up; refitted on the dyno
    P.vePeak = 0.41; P.veWidth = 3600.0;
    P.sparkHigh = 32.0;
    P.scavGain = 0.025;             // the X-pipe and open pipes reflect more than the catalysts did
    // the blower's drag at idle: a higher idle and air floor keep it from sagging when braked to a stop
    P.idleRpm = 800.0;
    P.idleMin = 3.1e-5;

    // the twin-screw supercharger (src/physics.hpp turboParams, screw)
    turboParams &T = P.turbo;
    T.supercharger = true;
    T.screw = true;
    // calibrated with --engine-type mustang-v6 --tuned yes --phystest (SC_DRIVE, SC_DISP, SC_LEAK):
    // ~0.2 bar by 2000 rpm, ~0.25-0.3 bar to the limiter
    T.screwDisp = 1.5e-3;
    T.screwLeak = 2.0e-4;
    T.screwLobes = 4; T.screwLobesB = 6;
    T.driveRatio = 1.9;             // crank pulley / blower pulley: ~14k rotor rpm at the limiter
    T.driveEff = 0.95;              // the belt
    T.maxRpm = 20000.0;
    T.etaC = 0.70;                  // internal compression: better than a roots blower
    T.vCharge = 6.0e-3;             // the blower's outlet and the lower manifold round the core
    T.icDrop = 0.15e5;
    T.icEffStill = 0.6; T.icEffMoving = 0.75; T.icTau = 6.0;  // air-to-water core
    T.bovArea = 2.5e-3;             // the ~55 mm bypass: passes the whole blower flow off throttle
    T.bovCrack = 30e3; T.bovSpan = 20e3;  // opened by manifold vacuum, shut under boost
    T.reliefP = 0.0;                // no relief valve: the pulley sets the boost
    T.boostMap = 155e3;             // the load the exhaust temperature follows
    T.knockRetard = 6.0;
    T.knockRetardHot = 0.08;
    // straight pipe: rings harder, louder, more rasp and crackle. No valve either way
    soundParams &S = P.sound;
    S.pipeLength = 3.0;
    S.pipeReflect = 0.68; S.pipeDampHz = 5000.0;
    S.pipeLowHz = 3800.0;
    S.masterHz = 13000.0;
    S.radiateHz = 220.0;
    S.drive = 0.081;                // peak ~0.75 in --wav (the blower adds its whine): above the stock car's ~0.5
    S.burble = 2.0;
    S.popRpm = 2500.0; S.popChance = 0.7;
    S.raspGain = 0.42;
    S.intakeGain = 0.9;
    S.turboGain = 8.0;              // the twin-screw's lobe-pass whine (src/sound.hpp turboVoice): ~12 dB under the exhaust at full throttle
    return P;
}

drivetrainParams mustangV6TunedDriveParams() {
    drivetrainParams P = mustangV6DriveParams();
    P.upPedal = 5200.0;             // full pedal: change up at 7200, just under the 7400 limiter
    P.sportUp = 7200.0;
    P.downMax = 6700.0; P.kickMax = 6600.0;
    return P;
}

carSpec mustangV6TunedCar() {
    carSpec S = mustangV6Car();
    S.name = "Ford Mustang V6 3.7 Cyclone (twin-screw, straight pipe)";
    S.sweepTo = 7400;
    S.torqueWhat = "peak torque (twin-screw ~0.25 bar)"; S.torqueLo = 465; S.torqueHi = 515;
    S.torqueRpmLo = 4000; S.torqueRpmHi = 6000;
    S.powerWhat = "peak power (twin-screw, ~340 kW / 455 hp)"; S.powerLo = 325; S.powerHi = 360;
    S.powerRpmLo = 6500; S.powerRpmHi = 7400;
    // a positive-displacement blower: the boost is there from low down, so the torque is too
    S.torqueAt = {{2000, 360, 420}, {3000, 420, 470}, {6500, 450, 500}};
    S.powerAt = {{5500, 265, 300}, {7000, 325, 360}};
    S.cylMaxLo = 90; S.cylMaxHi = 150;
    S.idleLo = 730; S.idleHi = 870;
    S.liftLo = 700; S.liftHi = 1000;
    S.carWhat = "1675 kg with driver, MT82 6-speed manual (tuned), 3.31 axle, rear-wheel drive, twin-screw, straight pipe";
    S.zeroTo100What = "0-100 km/h (twin-screw)"; S.zeroTo100Lo = 4.0; S.zeroTo100Hi = 5.6;
    S.zeroTo200What = "0-200 km/h (twin-screw)"; S.zeroTo200Lo = 11.0; S.zeroTo200Hi = 18.0;
    S.wotUpLo = 6900;
    // it holds 35% pedal up a 15% hill in its cruising gear: a steeper one makes it change down
    S.hillGrade = 0.22;
    return S;
}

const engineType &mustangV6TunedType() {
    static const engineType T = [] {
        engineType t = mustangV6Type();
        t.id = "mustang-v6-tuned";
        t.name = "Ford Mustang V6 (supercharged, straight pipe)";
        t.summary = "2014 Ford Mustang V6 3.7, twin-screw supercharger, straight pipe (no cats or mufflers, X-pipe), ECU tune, 7400 limiter";
        t.label = "Ford Mustang 3.7 Cyclone V6, 3726 cc, TWIN-SCREW, STRAIGHT PIPE";
        t.windowTitle = "2014 Ford Mustang V6 3.7 (supercharged, straight pipe)";
        t.hudTag = "SUPERCHARGED";
        t.boost = builtInBoost::supercharger;
        t.turboMeshFile = "engines/ford_mustang_v6/mustang_v6_blower.mesh";
        t.turboBuildHint = "python3 -I engines/ford_mustang_v6/build_mustang_v6.py";
        t.engine = mustangV6TunedParams;
        t.drive = mustangV6TunedDriveParams;
        t.car = mustangV6TunedCar;
        t.exhaust = mustangV6StraightExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(mustangV6Type, 18);
