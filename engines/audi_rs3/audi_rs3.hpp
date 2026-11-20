// Audi RS 3 (8Y, 2021-): the 2.5 TFSI turbocharged inline-5 (EA855 evo, engine code DAZA), the
// 7-speed S tronic and quattro. Engine, turbo, sound, gearing, exhaust, test targets and the engine
// type. Mesh: build_rs3.py -> rs3.mesh (built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"

// Audi 2.5 TFSI (RS 3 8Y): 2480 cc, 82.5 x 92.8 mm, 10.0:1, 294 kW (400 PS) at 5600-7000 rpm and
// 500 N m at 2250-5600 rpm, firing 1-2-4-5-3 every 144 deg. Aluminium block, forged steel crank,
// 144 mm rods, dual injection (port and direct), Audi valvelift on the exhaust cam, one turbo on an
// exhaust manifold cast into the head... in this simulator a separate manifold, so it shows. Values
// Audi does not publish (inertia, friction, cam timing, turbo map) are estimates.
engineParams audiRs3Params() {
    engineParams P;                 // a car: the generic baseline, then the RS 3's own values
    P.bore = 0.0825;
    P.compression = 10.0;
    P.ivc = 225.0 * DEGR;           // intake closes 45 deg after BDC
    P.evo = 495.0 * DEGR;           // exhaust opens 45 deg before BDC
    P.burn = 46.0 * DEGR;           // direct injection, strong tumble: a quick burn
    P.mRecip = 0.70;                // piston, pin and 1/3 of the 144 mm steel rod
    P.inertia = 0.17;               // forged crank, dual-mass flywheel, the DCT's clutch drum
    P.vManifold = 3.5e-3;
    P.throttleArea = 4.3e-3;        // 74 mm throttle body
    P.idleBase = 2.6e-5; P.idleMin = 2.0e-5; P.idleMax = 1.6e-4;  // a 2.5 L's worth of idle air
    P.idleRpm = 750.0;
    P.starterTorque = 170.0;
    P.starterMaxRpm = 300.0;
    P.revLimit = 7100.0; P.revResume = 6850.0;
    P.fmepA = 1.05e5; P.fmepB = 0.12e5; P.fmepC = 0.035e5;
    // calibrated with --engine-type rs3 --phystest, with the boost target and charge limit below, to
    // Audi's 500 N m plateau and 294 kW at 5600-7000
    P.veBase = 0.774; P.vePeak = 0.162; P.veRpm = 4600.0; P.veWidth = 3000.0;
    // pressure-wave tuning (estimates), weak on a turbo engine: ~0.30 m runners from the charge-air
    // plenum ram at ~5800 rpm (most of it is hidden on the plateau: the ECU's charge limit takes the
    // ripple out); the short manifold primaries end at the merge over the turbine, 0.25 m
    // acoustically (measured on the drawn layout by setupSim), which reflects little, so scavenging
    // is a trace
    P.ramLen = 0.30; P.ramDia = 0.045; P.ramGain = 0.03;
    P.scavGain = 0.01;
    P.sparkLow = 8.0; P.sparkHigh = 30.0; P.sparkFrom = 750.0; P.sparkSpan = 3500.0;
    P.fuelCutRpm = 1500.0;
    P.rpmScale = 8000.0;
    P.counterweight = 0.0;          // an even inline-5 has no free 1st- or 2nd-order force, only rocking couples
    P.blockMass = 180.0; P.blockRollInertia = 6.0;
    P.mountFreqY = 11.0; P.mountFreqRoll = 8.5;

    // the turbo (src/physics.hpp turboParams): one turbo spooling from ~2000 rpm: a small turbine nozzle
    // for the low-end response, a big wastegate and a high-pressure-ratio compressor for the ~0.3 kg/s
    // of air at the power peak. Its map is wide: the surge line sits well left of the engine's flow at
    // 2000 rpm (it must not surge at full throttle; the test checks) and it does not choke at 7000.
    // The ECU (engineSim::boostTarget): up to 1.5 bar, but the charge (torque) limit holds Audi's 500 N m
    // plateau with ~1.2-1.4 bar, and the boost falls from 5500 rpm to ~1.1 bar at 7000 for the flat
    // 294 kW top end
    turboParams &T = P.turbo;
    T.boostMap = 250e3;
    T.chargeMax = 1.96;
    T.boostFallRpm = 5500.0; T.boostTopRpm = 7000.0; T.boostTop = 210e3;
    T.inertia = 3.5e-5;
    T.maxRpm = 180000.0;
    T.prRef = 4.0; T.rpmRef = 160000.0;
    T.flowPeakRef = 0.09;
    T.chokeFall = 0.03;
    T.ductLA = 0.7 / 3.0e-3;
    T.vCharge = 7.0e-3;
    T.icDrop = 0.8e5;
    T.icEffStill = 0.55; T.icEffMoving = 0.85;
    T.bovArea = 4.5e-4;
    T.etaC = 0.75; T.etaT = 0.70;
    T.turbineArea = 0.42e-3;
    T.wastegateArea = 5.0e-3;
    T.vExhaust = 2.6e-3;
    T.bearingLoss = 1.6e-6;
    T.knockRetard = 7.0;            // keeps the peak cylinder pressure near 150 bar on full boost
    T.tExhLight = 850.0; T.tExhFull = 1200.0;

    // the five-cylinder's voice: the odd 2.5-order warble, a deep burble at idle, the RS sport exhaust's
    // flaps opening with the revs, crackle on the overrun, the turbo's whistle and the blow-off valve.
    // The detailed voice's stages (src/sound.hpp) give the flap, the radiated howl and the crackle
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLength = 2.6;             // manifold, turbo, cat, mid pipe and the rear silencer
    S.pipeLowHz = 1800.0; S.pipeResHz = 125.0; S.pipeResGain = 0.8;
    S.pipeReflect = 0.5;
    S.masterHz = 8000.0;
    // the flaps open under throttle (from ~2500 rpm) or with the revs from ~5000 rpm, so they shut as a
    // lift-off coasts down: the overrun and idle stay quieter than full throttle, and the straight-pipe
    // car stays louder than this one on the overrun
    S.flapOpenRpm = 5000.0; S.flapSpan = 1200.0;
    S.flapShutGain = 0.35;
    S.airboxHz = 160.0; S.intakeHowlHz = 650.0; S.intakeGain = 0.5;
    S.popRpm = 2500.0; S.popChance = 0.45;
    S.whineGain = 0.0;              // helical gears
    S.drive = 0.1143;
    S.radiateHz = 260.0;
    S.burble = 3.5;
    S.burbleFadeFrom = 4000.0; S.burbleFadeSpan = 2500.0; S.burbleHigh = 0.5;
    S.tick = 0.15;
    S.turboGain = 0.8;
    return P;
}

// 7-speed S tronic (DQ500 wet dual clutch), Audi's RS 3 data: 3.563 2.526 1.679 1.022 0.788 0.761
// 0.635, reverse 2.789, with two final drives: 4.059 on the shaft carrying 1st, 4th, 5th and reverse,
// 3.450 on the one carrying 2nd, 3rd, 6th and 7th. The simulator has one final drive (3.450), so 1st,
// 4th, 5th and reverse carry the 4.059 / 3.450 step in their own ratio. 265/30 R19 tyres (0.325 m),
// 1570 kg plus a 75 kg driver, quattro (the simulator has no tyre grip limit: all four wheels drive).
drivetrainParams audiRs3DriveParams() {
    drivetrainParams P;
    const double K = 4.059 / 3.450;
    P.ratios = {0.0, 3.563 * K, 2.526, 1.679, 1.022 * K, 0.788 * K, 0.761, 0.635};
    P.gears = 7;
    P.reverseRatio = 2.789 * K;
    P.finalDrive = 3.450;
    P.wheelRadius = 0.325;
    P.carMass = 1645.0;
    P.wheelInertia = 4.8;           // four 19 in wheels, big brake discs, the quattro shafts
    P.efficiency = 0.88;            // a wet DCT and all-wheel drive
    P.cdA = 0.72;
    P.crr = 0.012;
    P.clutchCapacity = 750.0;       // the DQ500's clutches, well above 500 N m
    P.clutchStiffness = 600.0;
    P.inputInertia = 0.03;
    P.brakeDecel = 10.0;
    // dual clutch: the next gear is preselected, one clutch hands over to the other in a few hundredths
    P.shiftOpen = 0.03; P.shiftClose = 0.08;
    P.dualClutch = true;            // drawn with its two wet clutch packs
    P.normalDown = 1300.0;
    P.sportUp = 6900.0; P.sportDown = 4500.0;
    P.matchHold = 0.3;
    P.openRpm = 900.0; P.idleOpenRpm = 1000.0; P.biteSpan = 1400.0;
    P.launchPedal = 2600.0;         // launch control: the clutch bites near 3500-4000 rpm at full pedal
    P.upBase = 2000.0; P.upPedal = 4900.0;  // full pedal: change up at 6900, just under the 7100 limiter
    P.upMargin = 300.0;
    P.downMax = 6500.0; P.kickRpm = 5000.0; P.kickMax = 6400.0; P.loadDownMax = 5200.0;
    P.pickRpm = 2500.0; P.pickMax = 6500.0;
    return P;
}

// --phystest targets (src/test_car.hpp): Audi's figures, with tolerance
carSpec audiRs3Car() {
    carSpec S;
    S.name = "Audi RS 3 2.5 TFSI";
    S.dispLo = 2470; S.dispHi = 2490;
    S.flareLo = 850; S.flareHi = 2200;
    S.idleLo = 700; S.idleHi = 820;
    S.idleMapLo = 20; S.idleMapHi = 55;
    S.toLimiterLo = 0.4; S.toLimiterHi = 3.0;
    S.liftLo = 650; S.liftHi = 1000;
    S.sweepTo = 7000;
    S.torqueWhat = "peak torque (Audi: 500 N m)"; S.torqueLo = 475; S.torqueHi = 530;
    S.torqueRpmLo = 2000; S.torqueRpmHi = 5600;
    S.powerWhat = "peak power (Audi: 294 kW / 400 PS)"; S.powerLo = 280; S.powerHi = 310;
    S.powerRpmLo = 5500; S.powerRpmHi = 7000;
    // Audi: 500 N m from 2250 to 5600 rpm, 294 kW from 5600 to 7000
    S.torqueAt = {{2000, 460, 530}, {2500, 480, 525}, {5500, 465, 525}};
    S.powerAt = {{6000, 280, 310}, {6500, 280, 310}, {7000, 280, 310}};
    S.plateauFrom = 2500; S.plateauTo = 5000; S.plateauMax = 30;
    S.cylMaxLo = 90; S.cylMaxHi = 150;
    S.turboMaxLo = 80; S.turboMaxHi = 180;
    S.spoolRpm = 2500; S.spoolLo = 0.3; S.spoolHi = 2.5;
    S.carWhat = "1645 kg with driver, 7-speed S tronic, quattro";
    S.zeroTo100What = "0-100 km/h, no launch control"; S.zeroTo100Lo = 3.6; S.zeroTo100Hi = 4.6;
    S.launchTo100What = "0-100 km/h, launch control (Audi: 3.8 s)"; S.launchTo100Lo = 3.5; S.launchTo100Hi = 4.1;
    S.zeroTo200What = "0-200 km/h (tests: ~12-13 s)"; S.zeroTo200Lo = 10.5; S.zeroTo200Hi = 14.5;
    S.launchLo = 2500; S.launchHi = 5000;
    S.wotUpLo = 6400;
    S.lightUpLo = 2000; S.lightUpHi = 3500;
    // the RS 3 holds its speed up 15% in 4th at 35% pedal: a 25% hill makes it change down
    S.hillPedal = 0.35; S.hillGrade = 0.25; S.flatGearLo = 4; S.flatGearHi = 7;
    S.coastDownLo = 1150; S.coastDownHi = 1500;
    S.dumpStalls = true;
    S.reverseLo = -45; S.reverseHi = -5;
    return S;
}

// exhaust and intake, mesh space (mm, "up" is -y, the front of the car at -z, the gearbox at +z).
// The five runners leave the head's +x side and gather in a cast manifold over the turbo, which hangs
// beside the block. The downpipe drops from the turbine and runs back under the car past the gearbox:
// a catalyst, the mid pipe with the RS sport exhaust's flap, the rear silencer and the RS 3's two big
// tailpipes. The compressor draws through the airbox behind it and blows forward to a front-mounted
// intercooler; the cooled charge comes back up the -x side to the throttle and the intake plenum.
exhaustLayout audiRs3Exhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> CAST{0.34f, 0.32f, 0.30f, 1.f}, STEEL{0.55f, 0.53f, 0.50f, 1.f},
        ALU{0.62f, 0.62f, 0.60f, 1.f}, BLACK{0.08f, 0.08f, 0.09f, 1.f}, RUBBER{0.06f, 0.06f, 0.07f, 1.f},
        TIP{0.12f, 0.12f, 0.13f, 1.f};
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
    const float DECK = e.cyls[0].deckY, HEAD_X = 0.5f * e.cyls[0].bore + 8.f;  // the head's side face

    // turbo frame (src/turbo.hpp) -> mesh: shaft +x -> mesh +z (compressor toward the gearbox), y -> y,
    // z -> -x. Its ports, in its own frame (see engines/inline4_fit/inline4_fit.hpp): turbine inlet
    // (45.7, -96.6, 64.1) facing -y, turbine outlet (0, 0, 0) facing -x, compressor inlet (196.8, 0, 0)
    // facing +x, compressor outlet (153.3, 68.5, -85.5) facing -z
    L.turbo.on = true;
    L.turbo.at = {215.f, -125.f, -70.f};
    const vec3 T = L.turbo.at, IN{T.x - 64.1f, T.y - 96.6f, T.z + 45.7f};
    const vec3 MERGE{IN.x, IN.y - 40.f, IN.z};

    // runners: out of the head, then curving down and together into the manifold over the turbine inlet
    std::vector<int> runners;
    for (int c = 0; c < N; c++) {
        float z = e.cyls[c].zCentre, py = DECK - 45.f;
        vec3 port = cylPoint(e, c, HEAD_X, py);
        runners.push_back(pipe({port, port + cylDir(e, c, 1.f, 0.f) * 35.f, {port.x + 70.f, py + 12.f, 0.75f * z + 0.25f * MERGE.z},
                                {MERGE.x - 6.f, MERGE.y - 20.f, 0.35f * z + 0.65f * MERGE.z}, MERGE},
                               19.f, 21.f, {}, CAST, false, c));
    }
    // the cast flange along the head
    L.boxes.push_back({{HEAD_X - 2.f, DECK - 70.f, e.cyls[0].zCentre - 45.f}, {HEAD_X + 10.f, DECK - 20.f, e.cyls[N - 1].zCentre + 45.f},
                       {0.30f, 0.29f, 0.28f, 1.f}, 0.7f, 0.5f});
    int feed = pipe({MERGE, {IN.x, IN.y + 6.f, IN.z}}, 30.f, 28.f, runners, CAST);

    // downpipe: forward out of the turbine, down under the turbo and back along the car
    const float LINE_Y = 255.f, LINE_X = 190.f;
    int down = pipe({{T.x, T.y, T.z + 4.f}, {T.x, T.y, T.z - 45.f}, {T.x - 10.f, T.y + 90.f, T.z - 80.f},
                     {T.x - 25.f, LINE_Y, T.z}, {LINE_X, LINE_Y, 120.f}}, 36.f, 36.f, {feed}, STEEL);
    int cat = pipe({{LINE_X, LINE_Y, 115.f}, {LINE_X, LINE_Y, 290.f}}, 62.f, 62.f, {down}, ALU, true);
    L.pipes[cat].acoustic = pipeAcoustic::catalyst;
    int mid = pipe({{LINE_X, LINE_Y, 285.f}, {LINE_X, LINE_Y, 520.f}, {LINE_X, LINE_Y, 700.f}}, 34.f, 34.f, {cat}, STEEL);
    int muf = pipe({{LINE_X, LINE_Y, 695.f}, {LINE_X, LINE_Y, 905.f}}, 82.f, 82.f, {mid}, ALU, true);
    // the two oval RS tailpipes (drawn round), splayed apart behind the silencer
    for (float dx : {-95.f, 95.f}) {
        vec3 a{LINE_X + 0.4f * dx, LINE_Y, 900.f}, b{LINE_X + dx, LINE_Y + 10.f, 960.f}, c{LINE_X + dx, LINE_Y + 10.f, 1010.f};
        int link = pipe({a, b, c}, 30.f, 30.f, {muf}, STEEL);
        pipe({{c.x, c.y, c.z - 5.f}, {c.x, c.y, c.z + 45.f}}, 48.f, 48.f, {link}, TIP, true);
        L.outlets.push_back({{c.x, c.y, c.z + 45.f}, {0.f, 0.f, 1.f}, 44.f});
    }
    // the sport exhaust's flap, in the mid pipe
    L.flap.on = true;
    L.flap.at = {LINE_X, LINE_Y, 600.f};
    L.flap.axis = {0.f, 0.f, 1.f};
    L.flap.shaft = {-1.f, 0.f, 0.f};
    L.flap.pipeR = 34.f;

    // intake, cold side first: the airbox behind the turbo feeds the compressor inlet
    const vec3 CIN{T.x, T.y, T.z + 196.8f}, COUT{T.x + 85.5f, T.y + 68.5f, T.z + 153.3f};
    int hose = pipe({{CIN.x, CIN.y, CIN.z - 4.f}, {CIN.x, CIN.y, CIN.z + 70.f}, {CIN.x + 20.f, CIN.y - 40.f, CIN.z + 120.f}}, 38.f, 40.f,
                    {}, RUBBER);
    L.pipes[hose].rough = 0.9f; L.pipes[hose].metal = 0.f;
    L.boxes.push_back({{CIN.x - 70.f, CIN.y - 170.f, CIN.z + 115.f}, {CIN.x + 110.f, CIN.y - 10.f, CIN.z + 290.f}, BLACK, 0.85f, 0.f});
    // the front-mounted intercooler ahead of the crank snout: finned core between two end tanks
    const float IC_Z0 = -470.f, IC_Z1 = -415.f, IC_Y0 = 30.f, IC_Y1 = 250.f, IC_L = -260.f, IC_R = 480.f;
    L.boxes.push_back({{IC_L + 40.f, IC_Y0, IC_Z0 + 5.f}, {IC_R - 40.f, IC_Y1, IC_Z1 - 5.f}, {0.10f, 0.10f, 0.11f, 1.f}, 0.8f, 0.4f});
    L.boxes.push_back({{IC_R - 40.f, IC_Y0 - 10.f, IC_Z0}, {IC_R, IC_Y1 + 10.f, IC_Z1}, ALU, 0.4f, 0.7f});
    L.boxes.push_back({{IC_L, IC_Y0 - 10.f, IC_Z0}, {IC_L + 40.f, IC_Y1 + 10.f, IC_Z1}, ALU, 0.4f, 0.7f});
    // hot side: compressor outlet, out sideways, then forward low down into the right end tank
    pipe({{COUT.x - 4.f, COUT.y, COUT.z}, {COUT.x + 35.f, COUT.y, COUT.z}, {COUT.x + 55.f, COUT.y + 20.f, COUT.z - 60.f},
          {COUT.x + 55.f, 110.f, -120.f}, {COUT.x + 55.f, 130.f, -340.f}, {IC_R - 20.f, 140.f, IC_Z1 + 30.f},
          {IC_R - 20.f, 140.f, IC_Z1 - 10.f}}, 30.f, 30.f, {}, ALU);
    // cold side: out of the left end tank and up the -x side to the throttle body on the plenum's front
    const float PLEN_X0 = -HEAD_X - 75.f, PLEN_X1 = -HEAD_X - 15.f, PLEN_Y0 = DECK - 85.f, PLEN_Y1 = DECK - 15.f;
    const float PLEN_Z0 = e.cyls[0].zCentre - 55.f, PLEN_Z1 = e.cyls[N - 1].zCentre + 55.f;
    const float THR_X = 0.5f * (PLEN_X0 + PLEN_X1), THR_Y = 0.5f * (PLEN_Y0 + PLEN_Y1);
    pipe({{IC_L + 20.f, 140.f, IC_Z1 - 10.f}, {IC_L + 20.f, 140.f, IC_Z1 + 40.f}, {IC_L + 30.f, 20.f, -380.f},
          {THR_X - 40.f, THR_Y + 40.f, PLEN_Z0 - 110.f}, {THR_X, THR_Y, PLEN_Z0 - 45.f}}, 32.f, 32.f, {}, ALU);
    L.boxes.push_back({{THR_X - 42.f, THR_Y - 42.f, PLEN_Z0 - 48.f}, {THR_X + 42.f, THR_Y + 42.f, PLEN_Z0 - 2.f}, BLACK, 0.5f, 0.6f});
    // the intake plenum along the head, and a short runner into each port
    L.boxes.push_back({{PLEN_X0, PLEN_Y0, PLEN_Z0}, {PLEN_X1, PLEN_Y1, PLEN_Z1}, {0.16f, 0.16f, 0.17f, 1.f}, 0.6f, 0.3f});
    for (int c = 0; c < N; c++) {
        float z = e.cyls[c].zCentre;
        int r = pipe({{PLEN_X1 - 5.f, THR_Y, z}, {-HEAD_X + 2.f, DECK - 45.f, z}}, 20.f, 18.f, {}, {0.16f, 0.16f, 0.17f, 1.f});
        L.pipes[r].rough = 0.6f; L.pipes[r].metal = 0.3f;
    }
    return L;
}

const engineType &audiRs3TunedType();  // --tuned yes, below

const engineType &audiRs3Type() {
    static const engineType T = [] {
        engineType t;
        t.id = "rs3";
        t.aliases = "audi audi-rs3 rs-3 inline5 i5 2.5tfsi daza";
        t.name = "Audi RS 3";
        t.summary = "Audi RS 3 2.5 TFSI turbo inline-5, 2480 cc, 7-speed S tronic, quattro";
        t.label = "Audi RS 3 2.5 TFSI inline-5, 2480 cc turbo";
        t.firingLabel = "1-2-4-5-3";
        t.windowTitle = "Audi RS 3 2.5 TFSI";
        t.credit = "RS 3 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/audi_rs3/rs3.mesh";
        t.buildHint = "python3 -I engines/audi_rs3/build_rs3.py";
        t.bike = false;
        t.boost = builtInBoost::turbo;  // a factory turbo: always on
        t.engine = audiRs3Params;
        t.drive = audiRs3DriveParams;
        t.physTest = physicsTestCar;
        t.car = audiRs3Car;
        t.exhaust = audiRs3Exhaust;
        t.turboMeshFile = "engines/inline4_fit/turbo.mesh";
        t.turboBuildHint = "python3 -I engines/inline4_fit/build_turbo.py (needs make assets)";
        // without that optional download: a procedural turbo with its ports in the same places
        t.turboFallbackMeshFile = "engines/audi_rs3/rs3_turbo.mesh";
        t.turboCredit = "turbo: \"Car Turbocharger\" by ml-design, CC-BY-4.0";
        t.hudFiringLine = true;
        t.variants = {{"tuned", audiRs3TunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: a big-turbo build, 500+ hp on the stock bore and stroke --------------------------------
// What a typical big-turbo RS 3 build does (estimates, not one tuner's car); the engine keeps its 82.5 x
// 92.8 mm, 2480 cc and its 10.0:1:
//   - a larger hybrid turbo: a bigger compressor wheel (a higher pressure ratio and more flow) and a
//     bigger turbine housing, so it breathes at 7000+ rpm but comes on boost ~3000-3500 rpm, not ~2000;
//   - an ECU map for an ethanol blend: ~1.8 bar, a higher torque limit, less spark retard per bar of
//     boost (the ethanol resists knock), more advance at the top;
//   - forged rods and pistons (the same dimensions): the limiter goes to 7400 rpm;
//   - a bigger intercooler, an 80 mm throttle and intake, a catless downpipe and a straight pipe
//     (no silencer, no flap) to the tips;
//   - a gearbox tune: more clutch clamping, later shifts, a higher launch rpm. Gear ratios stay stock.
engineParams audiRs3TunedParams() {
    engineParams P = audiRs3Params();
    P.throttleArea = 5.0e-3;        // 80 mm throttle
    P.revLimit = 7400.0; P.revResume = 7150.0;
    P.veRpm = 5200.0;               // intake, downpipe and the turbo's freer exhaust breathe best higher up
    P.veBase = 0.786; P.vePeak = 0.149;  // refitted around the wave tuning (same runners and manifold)
    P.sparkHigh = 31.0;
    turboParams &T = P.turbo;
    // calibrated with --engine-type rs3 --tuned yes --phystest: ~610 N m from 3500 rpm, ~410 kW (550 hp)
    // at 7000, peak cylinder pressure under 180 bar
    T.boostMap = 290e3;             // up to ~1.9 bar
    T.chargeMax = 2.3;              // the torque limit: ~610 N m
    T.boostFallRpm = 6000.0; T.boostTopRpm = 7400.0; T.boostTop = 270e3;
    T.turbineArea = 0.70e-3;        // bigger turbine housing: less back pressure, later spool
    T.prRef = 4.6; T.flowPeakRef = 0.11;  // bigger compressor wheel; still clear of surge at full throttle
    T.inertia = 4.5e-5;             // the bigger wheels spool slower
    T.maxRpm = 190000.0;
    T.icDrop = 0.5e5;               // bigger intercooler core
    T.icEffStill = 0.6; T.icEffMoving = 0.9;
    T.knockRetard = 5.5;            // ethanol blend
    T.tExhFull = 1230.0;
    // catless downpipe and a straight pipe: no silencer and no flap, so always open; the pipe rings
    // harder, louder, more crackle, more turbo
    soundParams &S = P.sound;
    S.flapOpenRpm = -1e6; S.flapShutGain = 1.0;  // nothing to shut
    S.pipeReflect = 0.65;
    S.pipeLowHz = 2300.0;
    S.popChance = 0.75;
    S.drive = 0.081;
    S.pipeLength = 2.3;
    S.intakeGain = 0.8;
    S.turboGain = 1.2;
    return P;
}

drivetrainParams audiRs3TunedDriveParams() {
    drivetrainParams P = audiRs3DriveParams();
    P.clutchCapacity = 950.0;       // the TCU tune's clutch clamping for ~610 N m
    P.upPedal = 5200.0;             // full pedal: change up at 7200, just under the 7400 limiter
    P.sportUp = 7200.0;
    P.downMax = 6800.0; P.kickMax = 6700.0;
    P.launchPedal = 3000.0;         // launch control a little higher, where the bigger turbo spools
    return P;
}

carSpec audiRs3TunedCar() {
    carSpec S = audiRs3Car();
    S.name = "Audi RS 3 2.5 TFSI (big turbo)";
    S.toLimiterLo = 0.4; S.toLimiterHi = 3.5;
    S.sweepTo = 7000;
    S.torqueWhat = "peak torque (big turbo)"; S.torqueLo = 580; S.torqueHi = 660;
    S.torqueRpmLo = 3000; S.torqueRpmHi = 5000;
    S.powerWhat = "peak power (500 hp = 373 kW at least)"; S.powerLo = 373; S.powerHi = 440;
    S.powerRpmLo = 6000; S.powerRpmHi = 7000;
    S.torqueAt = {{3500, 580, 660}, {5500, 560, 640}};
    S.powerAt = {{6500, 373, 440}, {7000, 373, 440}};
    S.plateauFrom = 3500; S.plateauTo = 5500; S.plateauMax = 40;
    S.cylMaxLo = 120; S.cylMaxHi = 185;            // forged pistons and rods
    S.turboMaxLo = 100; S.turboMaxHi = 190;
    S.boostCheckRpm = 4000;                         // the bigger turbo is on full boost from ~3500
    S.spoolRpm = 3500; S.spoolLo = 0.3; S.spoolHi = 3.0;
    S.carWhat = "1645 kg with driver, 7-speed S tronic (tuned), quattro, big turbo";
    S.zeroTo100What = "0-100 km/h, no launch control (big turbo)"; S.zeroTo100Lo = 3.0; S.zeroTo100Hi = 4.8;
    S.launchTo100What = "0-100 km/h, launch control (big turbo)"; S.launchTo100Lo = 2.8; S.launchTo100Hi = 3.8;
    S.zeroTo200What = "0-200 km/h (big turbo)"; S.zeroTo200Lo = 8.0; S.zeroTo200Hi = 13.0;
    S.launchLo = 2500; S.launchHi = 5500;
    S.wotUpLo = 6700;
    return S;
}

// the big-turbo car's exhaust: straight pipe. The stock route with the catless downpipe a size up, a
// straight pipe where the catalyst was, the mid pipe a size up without its flap, a straight pipe where
// the rear silencer was, and bigger blued titanium tips
exhaustLayout audiRs3TunedExhaust(const engineModel &e) {
    exhaustLayout L = audiRs3Exhaust(e);
    const std::array<float, 4> STEEL{0.55f, 0.53f, 0.50f, 1.f}, BLUED{0.42f, 0.40f, 0.55f, 1.f};
    for (pipeSpec &p : L.pipes) {
        if (p.r0 == 36.f && p.r1 == 36.f) { p.r0 = p.r1 = 40.f; }                 // the downpipe, 3.5 in
        else if ((p.r0 == 62.f && p.r1 == 62.f) || (p.r0 == 82.f && p.r1 == 82.f)) {  // catalyst, silencer: straight pipe
            p.r0 = p.r1 = 40.f; p.rgba = STEEL; p.capStart = p.capEnd = false;
            p.acoustic = pipeAcoustic::automatic;
        } else if (p.r0 == 34.f && p.r1 == 34.f) { p.r0 = p.r1 = 40.f; }          // the mid pipe
        else if (p.r0 == 30.f && p.r1 == 30.f && !p.from.empty()) { p.r0 = p.r1 = 36.f; }  // the links to the tips
        else if (p.r0 == 48.f && p.r1 == 48.f) { p.r0 = p.r1 = 52.f; p.rgba = BLUED; p.metal = 0.95f; p.rough = 0.25f; }
    }
    L.flap.on = false;              // no exhaust flap
    for (outletSpec &o : L.outlets) o.r = 48.f;
    return L;
}

const engineType &audiRs3TunedType() {
    static const engineType T = [] {
        engineType t = audiRs3Type();
        t.id = "rs3-tuned";
        t.name = "Audi RS 3 (big turbo)";
        t.summary = "Audi RS 3 2.5 TFSI, big-turbo build: hybrid turbo, ~1.8 bar on ethanol, forged internals, straight pipe, 7400 limiter, ~550 hp";
        t.label = "Audi RS 3 2.5 TFSI inline-5, 2480 cc, BIG TURBO";
        t.windowTitle = "Audi RS 3 2.5 TFSI (big turbo)";
        t.hudTag = "BIG TURBO";
        t.engine = audiRs3TunedParams;
        t.drive = audiRs3TunedDriveParams;
        t.car = audiRs3TunedCar;
        t.exhaust = audiRs3TunedExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(audiRs3Type, 15);
