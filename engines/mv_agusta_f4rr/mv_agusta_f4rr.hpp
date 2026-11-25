// MV Agusta F4 RR (2012-): engine, sound, gearing, test targets, demo ride and the engine type.
// Mesh: build_f4rr.py -> f4rr.mesh (built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"

// MV Agusta F4 RR (MV's spec sheet, Euro 3 bike): 998 cc, 79.0 x 50.9 mm (the shortest stroke of the
// inline-4 superbikes here), 13.4:1, 147.7 kW (201 hp) at 13600 rpm and 111 N m at 9600 rpm at the
// crank, 14000 rpm limiter (brochure). DOHC, four radial valves per cylinder, titanium rods, Mikuni
// ride-by-wire throttle bodies with eight injectors, Eldor EM2.0 ECU (MVICS), and the TSS: two-position
// variable-length intake runners, lifted off the throttle bodies near 10000 rpm. The TSS is folded into
// one broad VE curve and one runner length. Flat-plane crank firing 1-2-4-3. Not published by MV (rod
// length, throttle bore, inertia, friction, cam timing): estimates in line with the other superbikes.
engineParams mvAgustaF4rrParams() {
    engineParams P = superbikeBaseParams();  // a superbike; MV Agusta's own values follow
    P.bore = 0.079;
    P.compression = 13.4;
    P.ivc = 244.0 * DEGR;           // radial valves, race-like lobe: intake closes 64 deg after BDC
    P.evo = 480.0 * DEGR;
    P.burn = 46.0 * DEGR;           // wide 79 mm bore, flat radial-valve chamber
    P.mRecip = 0.30;                // forged 79 mm piston, pin, 1/3 of the titanium rod (MV: titanium)
    P.inertia = 0.034;              // forged 180 deg crank, alternator rotor, primary gear
    P.vManifold = 0.75e-3;
    P.throttleArea = 7.54e-3;       // four ~49 mm throttle bodies (bore: estimate)
    P.idleRpm = 1250.0;
    P.starterTorque = 55.0;
    P.fmepC = 0.0048e5;             // radial valves on rocker-less bucket tappets
    P.revLimit = 14000.0; P.revResume = 13600.0;  // MV: 14000 rpm limit
    // the TSS as two breathing curves (the lowCam model, src/physics.hpp): long runners (veLow*) for
    // the mid range, short runners (ve*) for the top end. The ECU lifts the runners by rpm: a blend
    // from ~9650 to ~11950 rpm (camSwitchRpm, camBlend), so the curve has no step at the change; the HUD
    // shows which is in use. Fitted together with --dyno (no ram air) to MV's 111 N m at 9600 and
    // 147.7 kW at 13600 with a smooth torque curve
    P.lowCam = true;
    P.camLowText = "TSS: long runners"; P.camHighText = "TSS: short runners";
    P.camSwitchRpm = 10800.0; P.camBlend = 2300.0;
    P.veLowBase = 0.650; P.veLowPeak = 0.410; P.veLowRpm = 9700.0; P.veLowWidth = 5800.0;
    P.veBase = 0.547; P.vePeak = 0.548; P.veRpm = 12500.0; P.veWidth = 5150.0;
    // pressure-wave tuning (estimates): one effective runner, ~0.30 m (trumpet, throttle body and
    // port), ram near the torque peak; the 4-2-1 headers to the collector
    P.ramLen = 0.30; P.ramDia = 0.049; P.ramGain = 0.05;
    P.scavGain = 0.035;
    P.sparkSpan = 7500.0;
    P.rpmScale = 15000.0;
    P.counterweight = 0.0;          // flat-plane inline-4: 1st order cancels by symmetry
    P.blockMass = 62.0; P.blockRollInertia = 1.1;
    // the F4's voice: the shrill flat-plane inline-4 howl, louder and rawer than the Japanese bikes'
    // through a short under-seat silencer and its four "organ pipe" outlets, a big airbox honk, crackle
    // on the overrun, straight-cut gear whine and the radial valvetrain's tick
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLowHz = 2800.0; S.pipeResHz = 230.0; S.pipeResGain = 0.65;
    S.pipeLength = 1.1;
    S.masterHz = 12500.0;
    S.intakeGain = 1.3;
    S.popChance = 0.4;
    S.whineGain = 0.024;
    S.drive = 0.061;
    S.radiateHz = 300.0;
    S.burble = 3.0;
    S.tick = 0.5;
    return P;
}

// MV Agusta F4 RR (MV's spec sheet): cassette six-speed, primary 48/82 (1.708), gears 14/37 16/33
// 18/31 20/30 22/29 21/25 (2.643 2.063 1.722 1.500 1.318 1.190), chain 15/41 (2.733), 200/55 ZR17
// rear (0.323 m). 190 kg dry, 17 l tank: ~208 kg wet (estimate) plus an 80 kg rider. Slipper clutch,
// EAS 2.0 up/down quickshifter, launch control, traction and wheelie control. MV: 298 km/h top speed.
drivetrainParams mvAgustaF4rrDriveParams() {
    drivetrainParams P = superbikeDriveBase();  // superbike rules; MV's numbers follow
    P.primary = 82.0 / 48.0;
    P.teethIn = {0, 14, 16, 18, 20, 22, 21};
    P.teethOut = {0, 37, 33, 31, 30, 29, 25};
    ratiosFromTeeth(P);
    P.primaryIn = 48; P.primaryOut = 82;
    P.finalDrive = 41.0 / 15.0;
    P.sprocketTeeth = 15;
    P.wheelRadius = 0.323;
    P.carMass = 288.0;
    P.cdA = 0.31;
    P.clutchCapacity = 250.0;
    P.sportUp = 13600.0; P.sportDown = 9700.0;
    P.upPedal = 9400.0;             // full throttle: change up just under the 14000 limiter
    P.launchPedal = 5500.0;
    return P;
}

bikeSpec mvAgustaF4rrSpec() {
    bikeSpec S = {"MV Agusta F4 RR", 996, 1000, 14000, 14600,
            "peak torque (MV Agusta: 111 N m)", "peak torque rpm (MV Agusta: 9600)",
            "peak power (MV Agusta: 147.7 kW / 201 hp)", "peak power rpm (MV Agusta: 13600)",
            104, 118, 9000, 10500, 140, 155, 13000, 14000, 14000,
            {180.0, 180.0, 180.0, 180.0}, "even 180-180-180-180 (max error)",
            "F4 RR gearing, 288 kg with rider", 12800, 14100,
            "top speed after 30 s, 6th gear (MV Agusta: 298)", 280, 320, 2.5,
            600};  // a tall 1st (12.3:1 overall, ~135 km/h): a 25% take-off bogs
    S.shiftRpm = 10500.0;
    return S;
}

// exhaust: the F4's signature. 4-2-1 titanium headers from the front of the inclined head, under the
// engine (1+4 and 2+3 merge), a collector and a link pipe with the exhaust valve rising behind the
// gearbox into a silencer under the seat, which ends in four short tailpipes stacked two by two under
// the tail: the "organ pipes". Mesh space: front +x, "up" -y, right +z.
// race = true: the F4 RC's Termignoni kit (estimate of its route): the same route in titanium with
// heat-blued headers a size up, no exhaust valve, and a shorter, slimmer titanium can under the seat
// ending in four titanium tips
exhaustLayout mvAgustaF4Exhaust(const engineModel &e, bool race) {
    exhaustLayout L;
    const std::array<float, 4> STEEL = race ? std::array<float, 4>{0.62f, 0.60f, 0.56f, 1.f}
                                            : std::array<float, 4>{0.62f, 0.60f, 0.57f, 1.f};
    const std::array<float, 4> BLUED{0.45f, 0.42f, 0.55f, 1.f};
    const std::array<float, 4> CAN = race ? std::array<float, 4>{0.62f, 0.60f, 0.56f, 1.f}
                                          : std::array<float, 4>{0.20f, 0.20f, 0.22f, 1.f};
    const std::array<float, 4> CHROME = race ? std::array<float, 4>{0.55f, 0.50f, 0.58f, 1.f}
                                             : std::array<float, 4>{0.80f, 0.80f, 0.82f, 1.f};
    const float hr = race ? 1.1f : 1.f;   // header radius scale
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from; p.rgba = c;
        p.rough = 0.3f; p.metal = 0.9f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    auto port = [&](int c) { return cylPoint(e, c, 0.5f * e.cyls[c].bore + 20.f, e.cyls[c].deckY - 41.f); };
    auto out = [&](int c) { return port(c) + cylDir(e, c, 1.f, 0.f) * 40.f; };
    for (int c : {0, 3}) {
        float z = e.cyls[c].zCentre, sg = z < 0.f ? -1.f : 1.f;
        pipe({port(c), out(c), {235.f, -60.f, z}, {245.f, 50.f, z}, {225.f, 160.f, 110.f * sg}, {200.f, 225.f, 50.f * sg},
              {175.f, 235.f, 0.f}}, 18.f * hr, 19.f * hr, c, {}, race ? BLUED : STEEL);
    }
    for (int c : {1, 2}) {
        float z = e.cyls[c].zCentre, sg = z < 0.f ? -1.f : 1.f;
        pipe({port(c), out(c), {235.f, -60.f, z}, {245.f, 50.f, z}, {215.f, 140.f, 40.f * sg}, {165.f, 185.f, 20.f * sg},
              {120.f, 195.f, 0.f}}, 18.f * hr, 19.f * hr, c, {}, race ? BLUED : STEEL);
    }
    int a = pipe({{175.f, 235.f, 0.f}, {120.f, 250.f, 0.f}, {40.f, 243.f, 0.f}}, 24.f, 25.f, -1, {0, 1}, STEEL);
    int b = pipe({{120.f, 195.f, 0.f}, {75.f, 215.f, 0.f}, {40.f, 238.f, 0.f}}, 24.f, 25.f, -1, {2, 3}, STEEL);
    int col = pipe({{40.f, 240.f, 0.f}, {-20.f, 245.f, 0.f}}, 28.f, 30.f, -1, {a, b}, STEEL);
    // the link pipe climbs behind the gearbox toward the seat
    int link = pipe({{-20.f, 245.f, 0.f}, {-170.f, 240.f, 20.f}, {-290.f, 170.f, 40.f}, {-340.f, 40.f, 50.f},
                     {-360.f, -40.f, 55.f}}, 30.f, 30.f, -1, {col}, STEEL);
    // the under-seat silencer, rising toward the tail
    int can = race ? pipe({{-355.f, -30.f, 55.f}, {-420.f, -82.f, 60.f}, {-500.f, -122.f, 63.f}}, 52.f, 48.f, -1, {link}, CAN, true)
                   : pipe({{-355.f, -30.f, 55.f}, {-430.f, -90.f, 60.f}, {-560.f, -150.f, 65.f}}, 62.f, 58.f, -1, {link}, CAN, true);
    // the four organ pipes, two by two in the silencer's end face, along its axis there: offsets are
    // square to that axis and each tip starts inside the can, so none floats off the end cap
    const std::vector<vec3> &cp = L.pipes[can].pts;
    const vec3 end = cp.back(), dir = normalize(cp.back() - cp[cp.size() - 2]);
    const vec3 side = normalize(cross(dir, vec3{0.f, -1.f, 0.f})), up = normalize(cross(side, dir));
    for (float du : {-22.f, 22.f})
        for (float ds : {-22.f, 22.f}) {
            vec3 s = end + up * du + side * ds - dir * 30.f;
            vec3 t = s + dir * 100.f;      // 70 mm out of the end cap
            pipe({s, t}, 15.f, 15.f, -1, {can}, CHROME, true);
            L.outlets.push_back({t, dir, 15.f});
        }
    L.flap.on = !race;              // the race kit deletes the valve
    L.flap.at = {-170.f, 240.f, 20.f};
    L.flap.axis = normalize(vec3{-1.f, 0.f, 0.15f});
    L.flap.shaft = {0.f, -1.f, 0.f};
    L.flap.pipeR = 30.f;
    return L;
}
exhaustLayout mvAgustaF4rrExhaust(const engineModel &e) { return mvAgustaF4Exhaust(e, false); }

const engineType &mvAgustaF4rcType();  // --tuned yes, below

const engineType &mvAgustaF4rrType() {
    static const engineType T = [] {
        engineType t;
        t.id = "f4rr";
        t.aliases = "mv-agusta mvagusta mv-agusta-f4 mv-f4 f4 f4-rr mv-f4rr";
        t.name = "MV Agusta F4 RR";
        t.summary = "MV Agusta F4 RR inline-4, 998 cc, radial valves, 180 deg crank, F4 RR gearing and four organ-pipe outlets";
        t.label = "MV Agusta F4 RR inline-4, 998 cc";
        t.windowTitle = "MV Agusta F4 RR engine";
        t.credit = "F4 RR built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/mv_agusta_f4rr/f4rr.mesh";
        t.buildHint = "python3 -I engines/mv_agusta_f4rr/build_f4rr.py";
        t.bike = true;
        t.engine = mvAgustaF4rrParams;
        t.drive = mvAgustaF4rrDriveParams;
        t.physTest = physicsTestBike;
        t.exhaust = mvAgustaF4rrExhaust;
        t.spec = mvAgustaF4rrSpec;
        t.gearboxSide = -1.f;      // forward-turning crank, front at +x: the box sits toward -x
        t.primaryModule = 2.6f * 84.f / (48.f + 82.f);  // 48/82 at the Ducati's 30/54 shaft spacing
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
        t.variants = {{"tuned", mvAgustaF4rcType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: the F4 RC (Reparto Corse) with its racing kit ---------------------------------------
// MV's own tune of this engine, from its F4 RC spec sheet (2015): 158.0 kW (212 hp) at 13600 rpm and
// 115 N m at 9300 rpm with the kit (151 kW / 205 hp street). What MV lists: a lighter crankshaft, light
// pistons and rods, a cylinder head with new intake ducts, four upper Magneti Marelli injectors with more
// fuel flow, and the racing kit: a Termignoni titanium exhaust with its mapped ECU. 175 kg dry with the
// kit, 302 km/h (road gearing). Same 13.4:1 and gearbox as the RR; taller track sprockets (estimate). Not published (estimates): the limiter
// (kept at 14000), the crank and piston masses, the cam timing and the exhaust's route.
engineParams mvAgustaF4rcParams() {
    engineParams P = mvAgustaF4rrParams();
    P.mRecip = 0.27;                // light pistons and rods
    P.inertia = 0.029;              // light crankshaft
    P.sparkHigh = 38.0;             // the kit's map
    P.camLowText = "TSS: long runners (RC)"; P.camHighText = "TSS: short runners (RC)";
    // new intake ducts and the open exhaust: more air everywhere, most at the top.
    // Fitted with --engine-type f4rr --tuned yes --dyno to MV's 115 N m at 9300 and 158 kW at 13600
    // fitted with the switch (blend ~9650-11650 rpm) for a smooth curve, at or above stock from 4500 rpm
    P.camSwitchRpm = 10650.0; P.camBlend = 2000.0;
    P.veLowBase = 0.645; P.veLowPeak = 0.475; P.veLowRpm = 9500.0; P.veLowWidth = 5250.0;
    P.veBase = 0.568; P.vePeak = 0.588; P.veRpm = 12850.0; P.veWidth = 5330.0;
    soundParams &S = P.sound;       // Termignoni titanium kit, valve deleted
    S.flapOpenRpm = 0.0; S.flapSpan = 1.0;
    S.flapShutGain = 1.0;
    S.popChance = 0.6;
    S.drive = 0.088;
    S.pipeLength = 0.95;            // a shorter can: a higher note
    S.pipeResHz = 250.0;
    S.intakeGain = 1.45;
    return P;
}

exhaustLayout mvAgustaF4rcExhaust(const engineModel &e) { return mvAgustaF4Exhaust(e, true); }

drivetrainParams mvAgustaF4rcDriveParams() {
    drivetrainParams P = mvAgustaF4rrDriveParams();
    P.carMass = 273.0;              // 175 kg dry with the kit, ~193 kg wet (estimate), plus the rider
    P.finalDrive = 38.0 / 15.0;     // track sprockets (estimate): 15/38 for 15/41, ~8% taller, so 6th
                                    // reaches ~330 km/h drag-limited just under the limiter
    return P;
}

bikeSpec mvAgustaF4rcSpec() {
    bikeSpec S = mvAgustaF4rrSpec();
    S.name = "MV Agusta F4 RC (racing kit)";
    S.torqueWhat = "peak torque (MV Agusta F4 RC kit: 115 N m)";
    S.torqueRpmWhat = "peak torque rpm (MV Agusta F4 RC kit: 9300)";
    S.powerWhat = "peak power (MV Agusta F4 RC kit: 158 kW / 212 hp)";
    S.powerRpmWhat = "peak power rpm (MV Agusta F4 RC kit: 13600)";
    S.torqueLo = 108; S.torqueHi = 122; S.torqueRpmLo = 8500; S.torqueRpmHi = 10500;
    S.powerLo = 150; S.powerHi = 166; S.powerRpmLo = 13000; S.powerRpmHi = 14000;
    S.bikeWhat = "F4 RR gearbox, 15/38 track sprockets, 273 kg with rider (F4 RC kit)";
    S.topWhat = "top speed after 30 s, 6th gear (MV: 302 on road gearing; 15/38 track sprockets)";
    S.topLo = 305; S.topHi = 345;
    return S;
}

const engineType &mvAgustaF4rcType() {
    static const engineType T = [] {
        engineType t = mvAgustaF4rrType();
        t.id = "f4rr-tuned";
        t.name = "MV Agusta F4 RC (racing kit)";
        t.summary = "MV Agusta F4 RC with racing kit: light crank, pistons and rods, new intake ducts, Termignoni titanium exhaust and ECU, 212 hp";
        t.label = "MV Agusta F4 RC inline-4, 998 cc, RACING KIT";
        t.windowTitle = "MV Agusta F4 RC engine (racing kit)";
        t.hudTag = "F4 RC";
        t.engine = mvAgustaF4rcParams;
        t.drive = mvAgustaF4rcDriveParams;
        t.spec = mvAgustaF4rcSpec;
        t.exhaust = mvAgustaF4rcExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(mvAgustaF4rrType, 47);
