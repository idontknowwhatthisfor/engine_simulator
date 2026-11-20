// Ducati Desmosedici Stradale (Panigale V4): engine, gearing, test targets and the engine type.
// Mesh: build_v4.py -> v4.mesh (a V4 built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"

// Ducati Desmosedici Stradale (Panigale V4): 1103 cc, 81 x 53.5 mm, 14.0:1, 157 kW at 13000 rpm,
// 124 N m at 9500-10000 rpm, ~14000 rpm limiter, ~1250 rpm idle. Light crank, four big throttle
// bodies, long cam timing; the bike's gearbox is not modelled, so the engine runs on a stand.
engineParams ducatiV4Params() {
    engineParams P = superbikeBaseParams();  // a superbike; the Ducati's own values follow
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
    // --engine-type v4 --phystest to 128 N m at 10500 and 155 kW at 12500 rpm)
    P.veBase = 0.609; P.vePeak = 0.55; P.veRpm = 11200.0; P.veWidth = 5300.0;
    // pressure-wave tuning (estimates): stack, oval 52 mm throttle body and long port, ~0.31 m in all,
    // ram hardest at ~10000 rpm (the published torque peak), a smaller hump at ~6000; the headers and
    // their pair merge reach the pre-silencer under the engine at 0.57 m acoustically (measured on the
    // drawn layout by setupSim: 0.51 m of pipe plus the port and the open end). Together a hump at
    // ~6000 rpm (+4%), a mid-range hole at 7500-8500 (-4%) and a hump at 10500-11000
    P.ramLen = 0.31; P.ramDia = 0.052; P.ramGain = 0.05;
    P.scavGain = 0.035;
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
    S.detailed = true;
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
    S.drive = 0.0875;
    return P;
}

// Ducati Panigale V4: primary 54/30, gears 38/14 36/17 33/19 32/21 30/22 30/24, chain 41/16, 200/55 ZR17
// rear tyre. 198 kg wet plus an 80 kg rider. Wet slipper clutch, up/down quickshifter, wheelie control.
drivetrainParams ducatiDriveParams() {
    drivetrainParams P = superbikeDriveBase();  // superbike rules; the Panigale's own numbers follow
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


// exhaust: the Panigale V4's layout. The front bank's headers leave forward and wrap down in front of
// the crankcase; the rear bank's leave backward and drop behind the gearbox. Each bank's pair merges
// (front 1+3, rear 2+4) and both feed the pre-silencer under the engine, which has an exhaust valve
// and two short outlets on the right (+z) that point back along the bike. Mesh space: front -x, "up" -y.
exhaustLayout ducatiV4Exhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> STEEL{0.55f, 0.48f, 0.40f, 1.f}, BLACK{0.09f, 0.09f, 0.10f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from; p.rgba = c;
        p.rough = c == BLACK ? 0.6f : 0.35f;
        p.metal = c == BLACK ? 0.3f : 0.85f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    // ports on the exhaust side of each head (front bank: local -x, rear bank: local +x), mid-head
    auto port = [&](int c, float side) { return cylPoint(e, c, side * (0.5f * e.cyls[c].bore + 18.f), e.cyls[c].deckY - 36.f); };
    auto out = [&](int c, float side) { return port(c, side) + cylDir(e, c, side, 0.25f) * 45.f; };
    // front bank: cylinders 1 (z -62) and 3 (z +38)
    for (int c : {0, 2}) {
        float z = e.cyls[c].zCentre, zm = c == 0 ? -40.f : 15.f;
        pipe({port(c, -1.f), out(c, -1.f), {-262.f, 0.f, z}, {-250.f, 110.f, z * 0.85f}, {-190.f, 190.f, z * 0.6f},
              {-150.f, 210.f, zm}, {-120.f, 215.f, -12.f}}, 20.f, 21.f, c, {}, STEEL);
    }
    // rear bank: cylinders 2 (z -38) and 4 (z +62)
    for (int c : {1, 3}) {
        float z = e.cyls[c].zCentre, zm = c == 1 ? -10.f : 35.f;
        pipe({port(c, 1.f), out(c, 1.f), {262.f, -30.f, z}, {275.f, 80.f, z * 0.85f}, {268.f, 180.f, z * 0.6f},
              {250.f, 215.f, zm}, {225.f, 225.f, 12.f}}, 20.f, 21.f, c, {}, STEEL);
    }
    int front = pipe({{-120.f, 215.f, -12.f}, {-90.f, 250.f, -6.f}, {-50.f, 270.f, 0.f}}, 26.f, 28.f, -1, {0, 1}, STEEL);
    int rear = pipe({{225.f, 225.f, 12.f}, {200.f, 252.f, 8.f}, {150.f, 270.f, 0.f}}, 26.f, 28.f, -1, {2, 3}, STEEL);
    int box = pipe({{-60.f, 270.f, 0.f}, {160.f, 270.f, 0.f}}, 46.f, 46.f, -1, {front, rear}, BLACK, true);
    // two short outlets leave the box's right side and turn back along the bike (rear is +x), one above
    // the other, ending just behind the box like the Panigale V4's low right-hand tips
    int upper = pipe({{60.f, 262.f, 30.f}, {110.f, 262.f, 62.f}, {170.f, 260.f, 74.f}, {225.f, 258.f, 80.f}}, 18.f, 18.f, -1,
                     {box}, STEEL, true);
    int lower = pipe({{60.f, 292.f, 30.f}, {110.f, 296.f, 62.f}, {170.f, 300.f, 74.f}, {220.f, 303.f, 80.f}}, 18.f, 18.f, -1,
                     {box}, STEEL, true);
    // a finisher on each outlet: a satin-black sleeve over the pipe end, a size up, with a polished lip.
    // Both are open at the back, so the outlet reads as a hole, and the flames leave from the lip
    const std::array<float, 4> SATIN{0.16f, 0.16f, 0.17f, 1.f}, POLISHED{0.78f, 0.76f, 0.72f, 1.f};
    auto finisher = [&](int from, vec3 end, vec3 dir) {
        dir = normalize(dir);
        vec3 a = end - dir * 12.f, b = end + dir * 30.f, c = b + dir * 4.f;
        pipeSpec s;
        s.pts = {a, b}; s.r0 = 22.f; s.r1 = 22.5f; s.from = {from};
        s.rgba = SATIN; s.rough = 0.55f; s.metal = 0.6f;
        s.capStart = true;
        L.pipes.push_back(s);
        pipeSpec lip;
        lip.pts = {b, c}; lip.r0 = 22.5f; lip.r1 = 22.5f; lip.from = {static_cast<int>(L.pipes.size()) - 1};
        lip.rgba = POLISHED; lip.rough = 0.15f; lip.metal = 1.f;
        L.pipes.push_back(lip);
        L.outlets.push_back({c, dir, 22.5f});
    };
    finisher(upper, {225.f, 258.f, 80.f}, {1.f, -0.04f, 0.11f});
    finisher(lower, {220.f, 303.f, 80.f}, {1.f, 0.06f, 0.12f});
    // exhaust valve in the rear secondary
    L.flap.on = true;
    L.flap.at = {205.f, 249.f, 9.f};
    L.flap.axis = normalize(vec3{-1.f, 0.6f, 0.f});
    L.flap.shaft = {0.f, 0.f, 1.f};
    L.flap.pipeR = 27.f;
    return L;
}

const engineType &ducatiV4TunedType();  // --tuned yes, below

const engineType &ducatiV4Type() {
    static const engineType T = [] {
        engineType t;
        t.id = "v4";
        t.aliases = "ducati ducati-v4 panigale";
        t.name = "Ducati V4";
        t.summary = "Ducati Desmosedici Stradale 90 deg V4, 1103 cc, Twin Pulse, Panigale V4 gearing";
        t.label = "Ducati-style 90 deg V4, 1103 cc";
        t.firingLabel = "Twin Pulse 0-90-290-380";
        t.windowTitle = "V4 engine";
        t.credit = "V4 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/ducati_v4/v4.mesh";
        t.buildHint = "python3 -I engines/ducati_v4/build_v4.py";
        t.bike = true;
        t.engine = ducatiV4Params;
        t.drive = ducatiDriveParams;
        t.physTest = physicsTestBike;
        t.exhaust = ducatiV4Exhaust;
        t.spec = ducatiSpec;
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
        t.variants = {{"tuned", ducatiV4TunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: the race exhaust and its map, a higher limiter and taller gearing -------------------
// Ducati's accessory route to more power for the Desmosedici Stradale: a full titanium racing exhaust
// (Akrapovic-made) with the ECU map that comes with it, quoted at about 226 hp (166 kW) against the
// stock 214. The engine itself is stock: the freer system lets the cylinders empty and fill better
// (the bigger, unrestricted collector and short cans), and the map adds a little spark and fuel. The
// gain starts low (~+1% at 4000-6000 rpm) and grows to ~+7% at the top.
// For top speed (estimates, not Ducati's kit): the map's limiter is 500 rpm up at 14500, and a 38-tooth
// rear sprocket in place of the 41 gears every ratio ~8% taller, so 6th runs out near the power peak
// instead of on the stock limiter. The gearbox itself is stock.
engineParams ducatiV4TunedParams() {
    engineParams P = ducatiV4Params();
    // calibrated with --engine-type v4 --tuned yes --phystest: 130 N m at 11000 and 161 kW at
    // 13000 rpm, against stock's 128 N m at 10500 and 155 kW; more everywhere, ~+1% at 4000-6000 rpm
    // growing to ~+7% at the top
    P.veBase = 0.610; P.vePeak = 0.575; P.veRpm = 11400.0; P.veWidth = 6000.0;  // stock's wave tuning
    P.revLimit = 14500.0; P.revResume = 14100.0;  // the race map's limiter
    P.sparkHigh = 37.0;             // race map
    soundParams &S = P.sound;       // open race system, no exhaust valve
    S.flapOpenRpm = 0.0; S.flapSpan = 1.0;
    S.flapShutGain = 1.0;
    S.popChance = 0.6;
    S.drive = 0.1637;
    S.pipeLength = 1.3;             // the same headers into short cans: a little shorter, a higher note
    S.pipeResHz = 170.0;
    S.burble = 3.5;                 // a harder lope at idle with the valve gone
    return P;
}

// the race exhaust: titanium headers on the stock route (a size up), the two bank pairs into a titanium
// collector under the engine in place of the black pre-silencer, then two short titanium cans stacked on
// the right with carbon end caps and open titanium tips. No exhaust valve.
exhaustLayout ducatiV4TunedExhaust(const engineModel &e) {
    exhaustLayout L = ducatiV4Exhaust(e);
    const std::array<float, 4> TI{0.62f, 0.60f, 0.56f, 1.f}, BLUED{0.45f, 0.42f, 0.55f, 1.f},
        CARBON{0.10f, 0.10f, 0.11f, 1.f};
    // the stock layout: 4 headers, 2 bank merges, then the pre-silencer and the outlets; keep the pipework
    L.pipes.resize(6);
    for (pipeSpec &p : L.pipes) {
        if (p.cyl >= 0) { p.r0 = 21.f; p.r1 = 22.f; p.rgba = BLUED; }  // heat-blued titanium headers
        else p.rgba = TI;
        p.rough = 0.25f; p.metal = 0.95f;
    }
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, std::vector<int> from, std::array<float, 4> c,
                    bool capStart, bool capEnd) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.from = from; p.rgba = c;
        p.rough = c == CARBON ? 0.5f : 0.25f;
        p.metal = c == CARBON ? 0.2f : 0.95f;
        p.capStart = capStart; p.capEnd = capEnd;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    int col = pipe({{-60.f, 270.f, 0.f}, {160.f, 270.f, 0.f}}, 32.f, 32.f, {4, 5}, TI, true, true);
    // two cans, one above the other, leaving the collector's right side and running back along the bike
    for (float y : {238.f, 304.f}) {
        int link = pipe({{40.f, 270.f, 20.f}, {75.f, y, 50.f}, {110.f, y, 72.f}}, 20.f, 20.f, {col}, TI, true, false);
        int can = pipe({{105.f, y, 72.f}, {235.f, y - 4.f, 88.f}}, 30.f, 28.f, {link}, TI, true, true);
        int cap = pipe({{230.f, y - 4.f, 88.f}, {252.f, y - 5.f, 91.f}}, 28.f, 20.f, {can}, CARBON, true, false);
        pipe({{250.f, y - 5.f, 91.f}, {262.f, y - 5.f, 92.f}}, 14.f, 14.f, {cap}, TI, true, false);
        L.outlets.push_back({{262.f, y - 5.f, 92.f}, {1.f, 0.f, 0.08f}, 14.f});
    }
    L.outlets.erase(L.outlets.begin(), L.outlets.end() - 2);  // the stock finishers' outlets go
    L.flap.on = false;              // valve deleted
    return L;
}

// taller gearing: a 38-tooth rear sprocket (16/38, 2.375) in place of the stock 41 (2.563), and the
// full-throttle upshift moved up with the limiter
drivetrainParams ducatiV4TunedDriveParams() {
    drivetrainParams P = ducatiDriveParams();
    P.finalDrive = 38.0 / 16.0;
    P.upPedal = 9800.0;             // full throttle: change up just under the 14500 limiter
    P.sportUp = 14000.0; P.sportDown = 9800.0;
    return P;
}

bikeSpec ducatiV4TunedSpec() {
    bikeSpec S = ducatiSpec();
    S.name = "Ducati V4 (tuned)";
    S.torqueWhat = "peak torque (tuned)";
    S.torqueRpmWhat = "peak torque rpm (tuned)";
    S.powerWhat = "peak power (Ducati, race exhaust: ~166 kW / 226 hp)";
    S.powerRpmWhat = "peak power rpm (tuned)";
    S.torqueLo = 118; S.torqueHi = 140; S.torqueRpmLo = 9500; S.torqueRpmHi = 12000;
    S.powerLo = 158; S.powerHi = 178; S.powerRpmLo = 12500; S.powerRpmHi = 14000;
    S.limLo = 14500; S.limHi = 15100;
    S.sweepTo = 14500;
    S.bikeWhat = "Panigale V4, 16/38 chain, 278 kg with rider (tuned)";
    S.upshiftLo = 13000; S.upshiftHi = 14600;
    S.topWhat = "top speed after 30 s, 6th gear (tuned, taller gearing)";
    S.topLo = 295; S.topHi = 335;
    S.takeoffMinRpm = 700;          // the taller 1st: a 25% take-off bogs deeper (~780 rpm)
    return S;
}

const engineType &ducatiV4TunedType() {
    static const engineType T = [] {
        engineType t = ducatiV4Type();
        t.id = "v4-tuned";
        t.name = "Ducati V4 (tuned)";
        t.summary = "Ducati Desmosedici Stradale V4, tuned: titanium race exhaust and its map, 14500 limiter, 16/38 gearing";
        t.label = "Ducati-style 90 deg V4, 1103 cc, TUNED";
        t.windowTitle = "V4 engine (tuned)";
        t.hudTag = "TUNED";
        t.engine = ducatiV4TunedParams;
        t.drive = ducatiV4TunedDriveParams;
        t.spec = ducatiV4TunedSpec;
        t.exhaust = ducatiV4TunedExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(ducatiV4Type, 20);
