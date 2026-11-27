// Cosworth CA2006 (2006): the 2.4 L 90 deg V8 Formula 1 engine of the Williams FW28, in the FW28: a
// 7-speed seamless-shift sequential gearbox, rear-wheel drive. Engine, sound, gearing, exhaust, test
// targets and the engine type. Mesh: build_ca2006.py -> ca2006.mesh (built from the inline-4 model's
// parts).
#pragma once
#include "../../src/engine_type.hpp"

// CA2006, Cosworth's figures (Race Engine Technology 20 and 21, 2007, and 73, 2013, as collected in
// Grand Prix Engines Note 108): 2399 cc, 98 x 39.75 mm (the rules' 98 mm maximum bore, so the shortest
// stroke Cosworth ever ran: stroke / bore 0.406), 13.3:1, rod 2.570 x the stroke, 41.3 mm inlet valves
// with 16 mm lift, a fixed inlet (the rules). 755 bhp (563 kW) at 19,250 rpm, 214.4 lb ft (290.7 N m)
// at 17,000; the power curve from 16,000 to 20,000 rpm (614, 694, 731, 744, 750, 754, 755, 747, 711
// bhp at 16, 17, 18, 18.5, 18.75, 19, 19.25, 19.5 and 20 krpm); 25.5 m/s mean piston speed at the
// power peak; 95 kg (the rules' minimum). Cosworth ran it to just over 20,000 rpm, the first F1 engine
// to do so; the limiter here is 20,000. Pneumatic valve springs, a flat-plane crank, eight throttle
// barrels under a carbon airbox fed from the roll-hoop scoop, a 4-plate carbon clutch and no
// flywheel. Values Cosworth does not publish (inertia, reciprocating mass, friction, cam timing,
// idle, header and trumpet lengths) are estimates.
engineParams cosworthCa2006Params() {
    engineParams P;                 // a car: the generic baseline, then the CA2006's own values
    P.bore = 0.098;
    P.compression = 13.3;
    P.ivc = 268.0 * DEGR;           // a very long intake cam for 19,000 rpm: closes 88 deg after BDC
    P.evo = 455.0 * DEGR;           // exhaust opens 85 deg before BDC
    P.blowdown = 16.0 * DEGR;
    P.burn = 42.0 * DEGR;           // a flat, very oversquare chamber with a 13.3:1 squish
    P.mRecip = 0.33;                // short-skirt forged piston, pin and 1/3 of the titanium rod
    P.inertia = 0.032;              // the crank, a 97 mm carbon clutch and no flywheel
    P.vManifold = 1.4e-3;           // the trumpets' share of the airbox: barrels right at the ports
    P.throttleArea = 1.6e-2;        // eight ~50 mm throttle barrels
    P.throttleCd = 0.85;
    P.idleBase = 6.0e-5; P.idleMin = 4.0e-5; P.idleMax = 3.0e-4;
    P.idleKp = 1.5e-8; P.idleKi = 2.0e-8;
    P.idleBand = 800.0;
    P.idleSparkBase = 10.0; P.idleSparkGain = 0.01;
    P.idleRpm = 4000.0;             // an F1 engine will not run slower: it stalls below ~3000
    P.starterTorque = 150.0;        // the external starter in the back of the gearbox
    P.starterMaxRpm = 1200.0;
    P.revLimit = 20000.0; P.revResume = 19700.0;
    // pneumatic valves, low-tension rings, roller main bearings are not allowed: plain bearings, a
    // dry sump scavenging the crankcase to a vacuum. Friction still climbs steeply with rpm:
    // FMEP ~ 4.5 bar at the limiter
    P.fmepA = 0.8e5; P.fmepB = 0.05e5; P.fmepC = 0.0063e5;
    P.pCrankcase = 70000.0;         // the scavenge pumps pull the crankcase down
    // calibrated with --engine-type ca2006 --phystest (and --dyno) to Cosworth's 290.7 N m at 17,000
    // and 563 kW at 19,250
    P.veBase = 0.68; P.vePeak = 0.74; P.veRpm = 18200.0; P.veWidth = 4700.0;
    // fixed trumpets tuned for the top: ~0.20 m from the bell mouth through the barrel and port. 4-into-1
    // equal-length headers per bank (measured on the drawn layout by setupSim)
    P.ramLen = 0.20; P.ramDia = 0.050; P.ramGain = 0.03;
    P.scavGain = 0.025; P.scavSound = 560.0;  // hotter gas at 19,000 rpm
    P.sparkLow = 14.0; P.sparkHigh = 36.0; P.sparkFrom = 4000.0; P.sparkSpan = 9000.0;
    P.fuelCutRpm = 5000.0;
    P.mapMin = 25e3;
    P.rpmScale = 20000.0;
    // the flat-plane crank: each bank is an inline-4 (no 1st-order force or couple), the two banks'
    // 2nd-order forces add sideways (the V8's horizontal 2nd-order shake)
    P.counterweight = 0.0;
    P.blockMass = 95.0; P.blockRollInertia = 2.6;   // the rules' 95 kg, a stressed member of the chassis
    P.mountFreqY = 30.0; P.mountFreqRoll = 25.0;     // bolted solid to the tub
    P.mountZeta = 0.08;

    // the 2006 V8 scream: the firing rate at 8 x rpm / 120 (1333 Hz at 20,000 rpm), short open headers
    // with nothing after them, the airbox's howl over the driver's head, no silencer, no valve
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLength = 0.9;             // primaries, collector and the short tailpipe through the bodywork
    S.pipeLowHz = 6500.0; S.pipeResHz = 380.0; S.pipeResGain = 0.5;
    S.pipeReflect = 0.7; S.pipeDampHz = 7000.0;
    S.masterHz = 17000.0;
    S.flapOpenRpm = -1e6; S.flapShutGain = 1.0;  // no exhaust valve
    S.airboxHz = 320.0; S.intakeHowlHz = 1600.0; S.intakeGain = 1.2;
    S.popRpm = 6000.0; S.popChance = 0.5;
    S.whineGain = 0.35;             // straight-cut gears in the gearbox
    S.drive = 0.091;
    S.radiateHz = 450.0;
    S.burble = 0.9;
    S.burbleFadeFrom = 6000.0; S.burbleFadeSpan = 5000.0; S.burbleHigh = 0.3;
    S.tick = 0.06;                  // pneumatic valve springs: little valvetrain clatter
    S.tickHz = 4500.0;
    S.raspGain = 0.45;
    return P;
}

// Williams' 7-speed seamless-shift sequential box behind the engine. Williams does not publish its
// ratios (each circuit gets its own): these are a medium-downforce set, 1st to ~115 km/h, 7th to ~320
// km/h at 19,500 rpm on 0.330 m rears (the 13 in rims and Bridgestones). 605 kg with the driver (the
// 2006 minimum) plus ~35 kg of fuel. CdA ~1.25 m^2 (Cd ~0.85 on ~1.5 m^2 at medium downforce). The
// seamless box changes up with no drive gap and a cut, downshifts with an auto-blip; F1 carbon brakes
// pull ~4-5 g from high speed, less once the downforce drops off.
drivetrainParams cosworthCa2006DriveParams() {
    drivetrainParams P;
    P.ratios = {0.0, 4.78, 3.67, 2.98, 2.51, 2.16, 1.90, 1.72};
    P.gears = 7;
    P.reverseRatio = 5.20;
    P.finalDrive = 4.40;
    P.wheelRadius = 0.330;
    P.carMass = 640.0;
    P.wheelInertia = 3.2;           // four magnesium wheels, tyres and carbon discs
    P.efficiency = 0.94;            // a straight-cut gearbox, no torque tube
    P.cdA = 1.25;
    P.crr = 0.015;
    P.clutchCapacity = 600.0;       // 4-plate carbon clutch
    P.clutchStiffness = 300.0;
    P.inputInertia = 0.008;
    P.brakeDecel = 25.0;
    P.shiftOpen = 0.02; P.shiftClose = 0.04;
    P.dualClutch = false;
    P.seamlessUp = true;            // the seamless box: the next gear goes in as the last comes out
    P.handoverTimeout = 0.06;
    P.normalDown = 7500.0;
    P.sportUp = 19300.0; P.sportDown = 15500.0;
    P.matchHold = 0.15;
    P.firstGearSpeed = 20.0 / 3.6;
    P.openRpm = 4800.0; P.idleOpenRpm = 5200.0; P.biteSpan = 3500.0;
    P.launchPedal = 6000.0;         // launch: the clutch bites near 11,000-12,000 rpm at full pedal
    P.upBase = 7000.0; P.upPedal = 12300.0;  // full pedal: change up at 19,300, just under the limiter
    P.upMargin = 1500.0;
    P.downMax = 18500.0; P.kickRpm = 15000.0; P.kickMax = 18000.0; P.loadDownMax = 13000.0;
    P.pickRpm = 12000.0; P.pickMax = 18000.0;
    return P;
}

// --phystest targets (src/test_car.hpp): Cosworth's figures, with tolerance
carSpec cosworthCa2006Car() {
    carSpec S;
    S.name = "Cosworth CA2006 2.4 V8";
    S.dispLo = 2390; S.dispHi = 2408;
    S.crankLo = 200; S.crankHi = 1300;
    S.flareLo = 3500; S.flareHi = 9000;  // the idle control catches it near 4000 with little flare
    S.idleLo = 3700; S.idleHi = 4300;
    S.idleMapLo = 15; S.idleMapHi = 70;
    S.toLimiterLo = 0.15; S.toLimiterHi = 1.0;  // no flywheel: idle to 20,000 in a blink
    S.limiterOver = 500.0;
    S.liftLo = 3600; S.liftHi = 4800;
    S.coastLo = 0.05; S.coastHi = 2.0;
    S.sweepTo = 20000;
    S.torqueWhat = "peak torque (Cosworth: 290.7 N m at 17,000)"; S.torqueLo = 280; S.torqueHi = 300;
    S.torqueRpmLo = 16500; S.torqueRpmHi = 18000;
    S.powerWhat = "peak power (Cosworth: 563 kW / 755 bhp at 19,250)"; S.powerLo = 550; S.powerHi = 575;
    S.powerRpmLo = 18500; S.powerRpmHi = 19500;
    // Cosworth's power curve: 614 bhp at 16,000, 731 at 18,000, 711 at 20,000
    S.torqueAt = {{16000, 260, 285}};
    S.powerAt = {{16000, 440, 475}, {18000, 535, 560}, {20000, 515, 545}};
    S.carWhat = "Williams FW28: 640 kg with driver and fuel, 7-speed seamless sequential, rear-wheel drive";
    // the simulator has no tyre grip limit: an F1 car manages ~2.5 s to 100 on its rear tyres
    S.zeroTo100What = "0-100 km/h (real car ~2.5 s; no grip limit)"; S.zeroTo100Lo = 1.4; S.zeroTo100Hi = 3.0;
    S.zeroTo200What = "0-200 km/h (2006 F1: ~5 s)"; S.zeroTo200Lo = 3.8; S.zeroTo200Hi = 5.8;
    S.launchTo100What = "0-100 km/h, launch control (no grip limit)"; S.launchTo100Lo = 1.4; S.launchTo100Hi = 2.9;
    S.wotUpLo = 18500;
    S.lightUpLo = 9000; S.lightUpHi = 13000;
    S.hillPedal = 0.35; S.hillGrade = 0.25; S.flatGearLo = 4; S.flatGearHi = 7;
    // 290 N m at 4000 rpm idle against 640 kg: a clutch dump in 1st bogs it, then it drives away
    S.dumpStalls = false;
    // 25-40% pedal is still 100+ kW in a 640 kg car: far quicker than a road car's take-off
    S.reverseLo = -90; S.reverseHi = -5;
    S.pullAwayHi = 120.0;
    return S;
}

// exhaust and intake, mesh space (mm, "up" is -y, the front of the car at -z, the gearbox at +z).
// The banks lean out 45 deg each way, bank A (cylinders 1-4) toward -x, bank B (5-8) toward +x; the
// exhaust ports are on the outer sides and the trumpets in the valley, as on every F1 V8. Each bank's
// four equal-length primaries curl down and back along the side of the block into a 4-into-1
// collector beside the gearbox, then a short tailpipe rises through the bodywork to an open end that
// points up and back over the floor. No catalyst, silencer or valve. Intake: a carbon airbox over the
// V, eight trumpets in it, fed by the snorkel from the roll-hoop scoop above the driver's head.
exhaustLayout cosworthCa2006Exhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> INCONEL{0.46f, 0.42f, 0.40f, 1.f}, CARBON{0.09f, 0.09f, 0.10f, 1.f},
        ALU{0.66f, 0.66f, 0.64f, 1.f}, BLUED{0.40f, 0.36f, 0.50f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false, int cyl = -1) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.from = from; p.rgba = c; p.cyl = cyl;
        p.rough = 0.45f; p.metal = 0.85f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    const int N = static_cast<int>(e.cyls.size());
    const float BORE = e.cyls[0].bore, DECK = e.cyls[0].deckY;
    float zLo = 1e9f, zHi = -1e9f;
    for (const cylinderGeom &g : e.cyls) { zLo = std::min(zLo, g.zCentre); zHi = std::max(zHi, g.zCentre); }

    // headers: per bank, out of the outer side of the head, down beside the block and back to a
    // collector beside the bellhousing. Each primary starts at its own z and ends at the same point
    for (int b = 0; b < 2; b++) {
        const float sx = b == 0 ? -1.f : 1.f;   // bank A toward -x, B toward +x
        const vec3 MERGE{sx * 215.f, 60.f, zHi + 150.f};
        std::vector<int> runners;
        for (int c = 0; c < N; c++) {
            if ((e.cyls[c].bank < 0.f) != (b == 0)) continue;
            float z = e.cyls[c].zCentre;
            vec3 port = cylPoint(e, c, sx * (0.5f * BORE + 18.f), DECK - 40.f);
            vec3 out = port + cylDir(e, c, sx, 0.2f) * 40.f;
            runners.push_back(pipe({port, out, {sx * 300.f, -70.f, z}, {sx * 290.f, 20.f, 0.6f * z + 0.4f * MERGE.z},
                                    {MERGE.x + sx * 20.f, MERGE.y - 20.f, 0.2f * z + 0.8f * MERGE.z}, MERGE},
                                   21.f, 22.f, {}, INCONEL, false, c));
        }
        int col = pipe({MERGE, {MERGE.x, MERGE.y + 10.f, MERGE.z + 70.f}, {sx * 200.f, 40.f, MERGE.z + 180.f}}, 33.f, 36.f,
                       runners, INCONEL);
        // the tailpipe: back and up through the sidepod's top, the open end facing up and back
        vec3 tip{sx * 190.f, -140.f, MERGE.z + 420.f};
        pipe({{sx * 200.f, 40.f, MERGE.z + 180.f}, {sx * 198.f, 10.f, MERGE.z + 300.f}, {sx * 192.f, -80.f, MERGE.z + 380.f}, tip},
             37.f, 40.f, {col}, BLUED, true);
        L.pipes.back().metal = 0.95f; L.pipes.back().rough = 0.3f;
        L.outlets.push_back({tip, normalize(vec3{0.f, -1.f, 0.6f}), 40.f});
    }

    // intake: the carbon airbox over the V with the eight trumpets standing in it over the valley-side
    // ports, and the snorkel forward and up to the roll-hoop scoop
    const float BOX_Y0 = -470.f, BOX_Y1 = -340.f;
    L.boxes.push_back({{-120.f, BOX_Y0, zLo - 75.f}, {120.f, BOX_Y1, zHi + 75.f}, CARBON, 0.5f, 0.2f});
    for (int c = 0; c < N; c++) {
        const float sx = e.cyls[c].bank < 0.f ? 1.f : -1.f;  // the valley side of this head (local x)
        vec3 port = cylPoint(e, c, sx * (0.5f * BORE + 2.f), DECK - 45.f);
        vec3 top{-sx * 50.f, BOX_Y1 + 5.f, e.cyls[c].zCentre};
        vec3 mid = (port + top) * 0.5f;
        int r = pipe({port, mid, top}, 24.f, 30.f, {}, ALU);   // the trumpet flares toward its bell mouth
        L.pipes[r].rough = 0.3f; L.pipes[r].metal = 0.9f;
    }
    int duct = pipe({{0.f, BOX_Y0 + 10.f, zLo - 40.f}, {0.f, BOX_Y0 - 80.f, zLo - 120.f}, {0.f, BOX_Y0 - 140.f, zLo - 260.f}},
                    80.f, 95.f, {}, CARBON);
    L.pipes[duct].rough = 0.5f; L.pipes[duct].metal = 0.2f;
    return L;
}

const engineType &cosworthCa2006Type() {
    static const engineType T = [] {
        engineType t;
        t.id = "ca2006";
        t.aliases = "cosworth cosworth-ca2006 ca f1 f1-v8 fw28 williams-fw28";
        t.name = "Cosworth CA2006";
        t.summary = "Cosworth CA2006 90 deg F1 V8, 2399 cc, 20,000 rpm, Williams FW28, 7-speed seamless";
        t.label = "Cosworth CA2006 F1 V8, 2399 cc (Williams FW28)";
        t.windowTitle = "Cosworth CA2006 2.4 V8";
        t.credit = "CA2006 V8 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/cosworth_ca2006/ca2006.mesh";
        t.buildHint = "python3 -I engines/cosworth_ca2006/build_ca2006.py";
        t.bike = false;
        t.engine = cosworthCa2006Params;
        t.drive = cosworthCa2006DriveParams;
        t.physTest = physicsTestCar;
        t.car = cosworthCa2006Car;
        t.exhaust = cosworthCa2006Exhaust;
        t.hudFiringLine = true;
        return t;
    }();
    return T;
}

REGISTER_ENGINE(cosworthCa2006Type, 19);
