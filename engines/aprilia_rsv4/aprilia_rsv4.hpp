// Aprilia RSV4 RR (2015-2020, 999.6 cc): engine, sound, gearing, test targets, demo ride and the engine type.
// Mesh: build_rsv4.py -> rsv4.mesh (a 65 deg V4 built from the inline-4 model's parts).
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"

// Aprilia RSV4 RR (Aprilia's spec sheet): 999.6 cc longitudinal 65 deg V4, 78 x 52.3 mm, 13.6:1, DOHC,
// four titanium valves per cylinder (33 mm intakes), Pankl rods on 36 mm crankpins, four 48 mm Marelli
// ride-by-wire throttle bodies with eight injectors, ECU-controlled variable-length intake ducts.
// 148 kW (201 hp) at 13000 rpm, 115 N m at 10500 rpm (some sheets: 117). Two crankpins 180 deg apart,
// one front and one rear rod on each: in a 65 deg V that fires 1-4-2-3 at 115-180-245-180 deg, the
// RSV4's uneven V4 beat. Not published (estimates in line with the other superbikes): rod length,
// cam timing, crank inertia, friction, the limiter (14000) and the runner lengths.
engineParams apriliaRsv4Params() {
    engineParams P = superbikeBaseParams();  // a superbike; Aprilia's own values follow
    P.bore = 0.078;
    P.compression = 13.6;
    P.ivc = 246.0 * DEGR;           // intake closes 66 deg after BDC
    P.evo = 478.0 * DEGR;
    P.burn = 45.0 * DEGR;           // compact 78 mm bore, narrow valve angle
    P.mRecip = 0.31;                // forged 78 mm piston, pin, 1/3 of the Pankl rod
    P.inertia = 0.031;              // crank, 420 W alternator rotor, primary gear
    P.vManifold = 0.8e-3;           // short runners inside the V and the airbox
    P.throttleArea = 7.24e-3;       // four 48 mm throttle bodies (Aprilia)
    P.idleRpm = 1250.0;
    P.starterTorque = 50.0;
    P.revLimit = 14000.0; P.revResume = 13600.0;  // estimate
    P.fmepC = 0.006e5;              // finger-follower valvetrain: between the MV's buckets and the Desmo
    // the variable-length intake ducts as two breathing curves (the lowCam model, src/physics.hpp):
    // long ducts (veLow*) for the mid range, short ducts (ve*) for the top end, the ECU blending from
    // one to the other by rpm (~7950 to ~10850 rpm). Fitted with --dyno to Aprilia's 115 N m at 10500 and 148 kW at 13000
    P.lowCam = true;
    P.camLowText = "Intake: long ducts"; P.camHighText = "Intake: short ducts";
    P.camSwitchRpm = 9400.0; P.camBlend = 2900.0;
    P.veLowBase = 0.680; P.veLowPeak = 0.444; P.veLowRpm = 10600.0; P.veLowWidth = 5500.0;
    P.veBase = 0.5925; P.vePeak = 0.555; P.veRpm = 11700.0; P.veWidth = 6000.0;
    // pressure-wave tuning (estimates): stack, throttle body and port, ~0.30 m, ram near the torque peak;
    // the 4-2-1 headers from both banks to the collector under the engine
    P.ramLen = 0.30; P.ramDia = 0.048; P.ramGain = 0.05;
    P.scavGain = 0.035;
    P.sparkSpan = 7200.0;
    P.rpmScale = 15000.0;
    P.blockMass = 58.0; P.blockRollInertia = 1.0;
    P.counterweight = 0.5;          // the two pins half a turn apart cancel each other's 1st order anyway
    // the RSV4's voice: the uneven 65 deg V4 beat (from the physics) gives its hard-edged, howling
    // bark, between a Ducati's rumble and an inline-4's shriek; the settings shape it: medium headers
    // from both banks into a short side silencer with a butterfly valve, the big airbox in front of the
    // tank, crackle on the overrun and the valvetrain's tick
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLength = 1.25;
    S.pipeReflect = 0.6; S.pipeDampHz = 3000.0;
    S.pipeLowHz = 3500.0; S.pipeResHz = 185.0; S.pipeResGain = 0.35;
    S.radiateHz = 290.0;
    S.burble = 3.0;
    S.burbleFadeFrom = 2500.0; S.burbleFadeSpan = 4000.0; S.burbleHigh = 0.06;
    S.flapOpenRpm = 5000.0; S.flapShutHz = 1400.0; S.flapOpenHz = 8000.0;
    S.airboxHz = 170.0; S.intakeHowlHz = 700.0; S.intakeGain = 1.25;
    S.popRpm = 3000.0; S.popChance = 0.45;
    S.whineGain = 0.02;             // straight-cut primary gears
    S.tick = 0.5; S.tickHz = 2600.0;
    S.masterHz = 11000.0;
    S.drive = 0.143;
    return P;
}

// Aprilia RSV4 RR (spec sheet): cassette six-speed, primary 73/44 (1.659) straight-cut, gears 39/15 33/16
// 34/20 31/21 31/23 34/27 (2.600 2.063 1.700 1.476 1.348 1.259), chain 16/42 (2.625), 200/55 ZR17 rear
// (0.323 m). 180-182 kg dry, 204 kg wet (18.5 l tank) plus an 80 kg rider. Slipper clutch, APRC: AQS
// up/down quickshifter, launch control, traction and wheelie control. Aprilia: ~291 km/h.
drivetrainParams apriliaRsv4DriveParams() {
    drivetrainParams P = superbikeDriveBase();  // superbike rules; Aprilia's numbers follow
    P.primary = 73.0 / 44.0;
    P.teethIn = {0, 15, 16, 20, 21, 23, 27};
    P.teethOut = {0, 39, 33, 34, 31, 31, 34};
    ratiosFromTeeth(P);
    P.primaryIn = 44; P.primaryOut = 73;
    P.finalDrive = 42.0 / 16.0;
    P.sprocketTeeth = 16;
    P.wheelRadius = 0.323;
    P.carMass = 284.0;
    P.cdA = 0.32;                   // rider tucked in behind the RSV4's screen
    P.clutchCapacity = 270.0;
    P.sportUp = 13500.0; P.sportDown = 9800.0;
    P.upPedal = 9400.0;             // full throttle: change up just under the 14000 limiter
    P.launchPedal = 7500.0;         // launch control holds ~10000 rpm, near the torque peak, for the tall 1st
    return P;
}

bikeSpec apriliaRsv4Spec() {
    bikeSpec S = {"Aprilia RSV4 RR", 997, 1002, 14000, 14600,
            "peak torque (Aprilia: 115 N m)", "peak torque rpm (Aprilia: 10500)",
            "peak power (Aprilia: 148 kW / 201 hp)", "peak power rpm (Aprilia: 13000)",
            108, 122, 9500, 11500, 141, 156, 12500, 14000, 14000,
            {115.0, 180.0, 245.0, 180.0}, "65 deg V4 115-180-245-180 (max error)",
            "RSV4 RR gearing, 284 kg with rider", 12800, 14100,
            "top speed after 30 s, 6th gear (Aprilia ~291)", 275, 315, 2.5,
            700};  // a tall 1st (11.3:1 overall, ~150 km/h at the limiter): a 25% take-off bogs
    S.shiftRpm = 10500.0;
    return S;
}

// exhaust: the RSV4's 4-2-1. The front bank's headers leave forward and drop in front of the
// crankcase, the rear bank's leave backward and drop behind the cylinders; each bank's pair merges
// (front 1+3, rear 2+4) under the engine, the two meet in a collector with the catalyst, and a link pipe
// with the butterfly valve runs back to the short side silencer on the right, angled up under the
// footpeg. Mesh space: front +x, "up" -y, right +z.
// race = true: the Akrapovic Evolution full system (tuned version): the same route in titanium, the
// headers heat-blued and a size up, a slimmer collector with no catalyst, no valve, and Akrapovic's
// shorter hexagonal titanium muffler with a carbon end cap
exhaustLayout apriliaRsv4ExhaustFor(const engineModel &e, bool race) {
    exhaustLayout L;
    const std::array<float, 4> TI{0.70f, 0.69f, 0.66f, 1.f}, BLUED{0.47f, 0.43f, 0.56f, 1.f},
        CARBON{0.10f, 0.10f, 0.11f, 1.f};
    const std::array<float, 4> STEEL = race ? std::array<float, 4>{0.64f, 0.62f, 0.58f, 1.f}
                                            : std::array<float, 4>{0.58f, 0.53f, 0.47f, 1.f};
    const std::array<float, 4> CAN = race ? std::array<float, 4>{0.66f, 0.65f, 0.62f, 1.f}
                                          : std::array<float, 4>{0.14f, 0.14f, 0.15f, 1.f};
    const float hr = race ? 1.08f : 1.f;   // header radius scale
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from; p.rgba = c;
        bool dull = c == CARBON || (!race && c == CAN);
        p.rough = dull ? 0.55f : race ? 0.25f : 0.35f;
        p.metal = dull ? 0.4f : race ? 0.95f : 0.85f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    // ports on the outside of each head (front bank: local +x, faces forward; rear bank: local -x,
    // faces backward), mid-head
    auto port = [&](int c, float side) { return cylPoint(e, c, side * (0.5f * e.cyls[c].bore + 18.f), e.cyls[c].deckY - 34.f); };
    auto out = [&](int c, float side) { return port(c, side) + cylDir(e, c, side, 0.25f) * 40.f; };
    // front bank: cylinders 1 and 3
    for (int c : {0, 2}) {
        float z = e.cyls[c].zCentre, zm = c == 0 ? -14.f : 14.f;
        pipe({port(c, 1.f), out(c, 1.f), {262.f, -10.f, z}, {268.f, 90.f, z * 0.85f}, {235.f, 185.f, z * 0.6f},
              {175.f, 222.f, zm}, {125.f, 232.f, 0.f}}, 19.f * hr, 20.f * hr, c, {}, race ? BLUED : STEEL);
    }
    // rear bank: cylinders 2 and 4
    for (int c : {1, 3}) {
        float z = e.cyls[c].zCentre, zm = c == 1 ? -14.f : 14.f;
        pipe({port(c, -1.f), out(c, -1.f), {-175.f, -150.f, z}, {-205.f, -40.f, z * 0.85f}, {-205.f, 100.f, z * 0.6f},
              {-170.f, 205.f, zm}, {-120.f, 235.f, 0.f}}, 19.f * hr, 20.f * hr, c, {}, race ? BLUED : STEEL);
    }
    int front = pipe({{125.f, 232.f, 0.f}, {85.f, 245.f, 0.f}, {40.f, 250.f, 0.f}}, 26.f, 27.f, -1, {0, 1}, STEEL);
    int rear = pipe({{-120.f, 235.f, 0.f}, {-80.f, 248.f, 0.f}, {-40.f, 252.f, 0.f}}, 26.f, 27.f, -1, {2, 3}, STEEL);
    // the collector under the engine: stock, a fat one with the catalyst; Akrapovic, a plain 2-1 merge
    int col = pipe({{50.f, 250.f, 0.f}, {-50.f, 252.f, 0.f}}, race ? 31.f : 40.f, race ? 31.f : 40.f, -1, {front, rear},
                   STEEL, true);
    if (!race) L.pipes[col].acoustic = pipeAcoustic::catalyst;
    // link pipe out of the collector's back, out to the right and up toward the silencer
    int link = pipe({{-50.f, 252.f, 0.f}, {-130.f, 248.f, 20.f}, {-200.f, 232.f, 70.f}, {-250.f, 210.f, 100.f}},
                    30.f, 30.f, -1, {col}, STEEL);
    // the side silencer, rising toward the tail: stock a short round can, Akrapovic a shorter hexagonal one
    int can = race ? pipe({{-245.f, 213.f, 100.f}, {-320.f, 180.f, 111.f}, {-405.f, 137.f, 116.f}}, 54.f, 50.f, -1, {link},
                          CAN, true)
                   : pipe({{-245.f, 213.f, 100.f}, {-330.f, 175.f, 112.f}, {-440.f, 120.f, 118.f}}, 58.f, 52.f, -1, {link},
                          CAN, true);
    if (race) L.pipes[can].sides = 6;
    // the end cap (stock titanium, Akrapovic carbon) and one outlet
    const std::vector<vec3> &cp = L.pipes[can].pts;
    const vec3 end = cp.back(), dir = normalize(cp.back() - cp[cp.size() - 2]);
    int cap = pipe({end - dir * 6.f, end + dir * 18.f}, race ? 50.f : 52.f, 34.f, -1, {can}, race ? CARBON : TI, true);
    if (race) L.pipes[cap].sides = 6;
    int tip = pipe({end + dir * 12.f, end + dir * 40.f}, 26.f, 26.f, -1, {cap}, TI, true);
    L.outlets.push_back({L.pipes[tip].pts.back(), dir, 26.f});
    // the butterfly valve in the link pipe (stock only)
    L.flap.on = !race;
    L.flap.at = {-130.f, 248.f, 20.f};
    L.flap.axis = normalize(vec3{-1.f, 0.f, 0.25f});
    L.flap.shaft = {0.f, -1.f, 0.f};
    L.flap.pipeR = 30.f;
    return L;
}
exhaustLayout apriliaRsv4Exhaust(const engineModel &e) { return apriliaRsv4ExhaustFor(e, false); }

const engineType &apriliaRsv4TunedType();  // --tuned yes, below

const engineType &apriliaRsv4Type() {
    static const engineType T = [] {
        engineType t;
        t.id = "rsv4";
        t.aliases = "aprilia aprilia-rsv4 rsv4rr rsv4-rr aprilia-rsv4rr aprilia-rsv4-rr";
        t.name = "Aprilia RSV4 RR";
        t.summary = "Aprilia RSV4 RR 65 deg V4, 999.6 cc, 115-180-245-180 firing, variable intake, RSV4 RR gearing";
        t.label = "Aprilia RSV4 RR 65 deg V4, 999.6 cc";
        t.firingLabel = "65 deg V4 1-4-2-3, 0-115-295-540";
        t.windowTitle = "Aprilia RSV4 RR engine";
        t.credit = "RSV4 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/aprilia_rsv4/rsv4.mesh";
        t.buildHint = "python3 -I engines/aprilia_rsv4/build_rsv4.py";
        t.bike = true;
        t.engine = apriliaRsv4Params;
        t.drive = apriliaRsv4DriveParams;
        t.physTest = physicsTestBike;
        t.exhaust = apriliaRsv4Exhaust;
        t.spec = apriliaRsv4Spec;
        t.gearboxSide = -1.f;      // forward-turning crank, front at +x: the box sits toward -x
        t.primaryModule = 2.6f * 84.f / (44.f + 73.f);  // 44/73 at the Ducati's 30/54 shaft spacing
        // --wav: start-up and blips in neutral, the limiter, a launch up the box with the quickshifter,
        // braking with rev-matched quickshift downshifts, then back on the throttle
        t.ride = {
            {1.0, true, 0.0, 0, "starter"},          {3.0, false, 0.0, 0, "idle (V4 beat, valve shut)"},
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
        t.variants = {{"tuned", apriliaRsv4TunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: Akrapovic Evolution full system and a race map, 14500 limiter, 16/40 gearing --------
// Akrapovic's Evolution full titanium system for the RSV4 RR (S-A10E8-RC), on Akrapovic's dyno at the
// wheel: +5.2 kW at 13500 rpm (140.8 -> 145.8 kW, +3.7%) and +3 N m at ~10800 rpm (110.3 -> 113.2), 3.2 kg
// lighter than stock (4.7 against 7.9 kg), +2 dB. With it, an ECU race map (estimates, not Akrapovic's
// figures): a little more spark, the catalyst and valve gone, and the limiter 500 rpm up at 14500. At the
// crank that is ~155 kW (208 hp) at 13500 and ~119 N m at 11000 rpm. The engine is stock.
// The stock bike already tops out on the 14000 limiter in 6th. Tuned, on the stock 16/42 it still ends
// on the (14500) limiter in 6th, at ~321 km/h: gearing-limited. So the tuned one runs Aprilia's own
// alternative 16/40 chain (2.500, listed for the RSV4 RR beside the stock 16/42), every ratio 5% taller:
// 6th now tops out at ~13800 rpm, at the power peak and short of the limiter, drag-limited at the same
// ~321 km/h (the extra power is just enough to pull the taller gear).
engineParams apriliaRsv4TunedParams() {
    engineParams P = apriliaRsv4Params();
    P.revLimit = 14500.0; P.revResume = 14100.0;  // the race map's limiter (estimate)
    P.sparkHigh = 37.0;             // race map
    P.camLowText = "Intake: long ducts (race map)"; P.camHighText = "Intake: short ducts (race map)";
    // the freer system: more air everywhere, most at the top. Fitted with --engine-type rsv4 --tuned yes
    // --dyno to ~120 N m at 11000 and ~155 kW at 13500-14000: +1% at 5000 rpm, +3% at 10800, +5% at 13500
    P.camSwitchRpm = 9300.0; P.camBlend = 3000.0;
    P.veLowBase = 0.689; P.veLowPeak = 0.473; P.veLowRpm = 10590.0; P.veLowWidth = 5300.0;
    P.veBase = 0.615; P.vePeak = 0.575; P.veRpm = 11910.0; P.veWidth = 6150.0;
    soundParams &S = P.sound;       // open titanium system, no valve
    S.flapOpenRpm = 0.0; S.flapSpan = 1.0;
    S.flapShutGain = 1.0;
    S.popChance = 0.6;
    S.pipeLength = 1.15;            // a shorter can: a higher note
    S.pipeResHz = 205.0;
    S.burble = 3.4;
    S.raspGain = 0.5;
    S.drive = 0.243;                // ~2 dB over stock (Akrapovic: +2 dB static)
    return P;
}

exhaustLayout apriliaRsv4TunedExhaust(const engineModel &e) { return apriliaRsv4ExhaustFor(e, true); }

drivetrainParams apriliaRsv4TunedDriveParams() {
    drivetrainParams P = apriliaRsv4DriveParams();
    P.finalDrive = 40.0 / 16.0;     // Aprilia's alternative rear sprocket: 16/40 for 16/42
    P.carMass = 281.0;              // 3.2 kg lighter exhaust
    P.upPedal = 9900.0;             // full throttle: change up just under the 14500 limiter
    P.sportUp = 14000.0;
    return P;
}

bikeSpec apriliaRsv4TunedSpec() {
    bikeSpec S = apriliaRsv4Spec();
    S.name = "Aprilia RSV4 RR (tuned)";
    S.limLo = 14500; S.limHi = 15100;
    S.sweepTo = 14500;
    S.torqueWhat = "peak torque (Akrapovic: +3 N m at the wheel)";
    S.torqueRpmWhat = "peak torque rpm (tuned)";
    S.powerWhat = "peak power (Akrapovic: +5.2 kW at the wheel at 13500)";
    S.powerRpmWhat = "peak power rpm (tuned)";
    S.torqueLo = 112; S.torqueHi = 126; S.torqueRpmLo = 9500; S.torqueRpmHi = 12000;
    S.powerLo = 150; S.powerHi = 163; S.powerRpmLo = 12800; S.powerRpmHi = 14500;
    S.bikeWhat = "RSV4 RR gearbox, 16/40 chain, 281 kg with rider (tuned)";
    S.upshiftLo = 13300; S.upshiftHi = 14600;
    S.topWhat = "top speed after 30 s, 6th gear (tuned, 16/40)";
    S.topLo = 300; S.topHi = 340;
    return S;
}

const engineType &apriliaRsv4TunedType() {
    static const engineType T = [] {
        engineType t = apriliaRsv4Type();
        t.id = "rsv4-tuned";
        t.name = "Aprilia RSV4 RR (tuned)";
        t.summary = "Aprilia RSV4 RR, tuned: Akrapovic Evolution titanium system and race map, 14500 limiter, 16/40 gearing";
        t.label = "Aprilia RSV4 RR 65 deg V4, 999.6 cc, TUNED";
        t.windowTitle = "Aprilia RSV4 RR engine (tuned)";
        t.hudTag = "TUNED";
        t.engine = apriliaRsv4TunedParams;
        t.drive = apriliaRsv4TunedDriveParams;
        t.spec = apriliaRsv4TunedSpec;
        t.exhaust = apriliaRsv4TunedExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(apriliaRsv4Type, 22);
