// Kawasaki Ninja H2 (2015-): the supercharged inline-4. Engine, supercharger, sound, gearing, test
// targets, demo ride and the engine type. Meshes: build_h2.py -> h2.mesh (built from the inline-4
// model's parts) and h2_supercharger.mesh (procedural, drawn by src/turbo.hpp).
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"
#include "../../src/exhaust_layouts.hpp"

// Kawasaki Ninja H2 (2015-): 998 cc, 76.0 x 55.0 mm, 8.5:1, 147.2 kW (200 PS) at 11000 rpm and
// 133.5 N m at 10500 rpm (without ram air), redline 14000 rpm. Flat-plane crank firing 1-2-4-3, four
// 50 mm throttle bodies fed from a plenum. A centrifugal supercharger behind the cylinders, gear
// driven from the crank through a planetary step-up at 9.2x crank speed (~130000 rpm at the top):
// 69 mm impeller with 6 blades at the tip and 12 at the inducer, up to ~2.4x atmospheric pressure,
// no intercooler, ram air from the left of the fairing. Values not published by Kawasaki (rod,
// inertia, friction, cam timing, compressor map) are estimates calibrated to the published figures.
engineParams kawasakiH2Params() {
    engineParams P = superbikeBaseParams();  // a superbike; Kawasaki's own values follow
    P.bore = 0.076;
    P.compression = 8.5;            // low, for the boost
    P.ivc = 230.0 * DEGR;           // milder cam than a naturally aspirated superbike: the boost fills it
    P.evo = 490.0 * DEGR;
    P.burn = 46.0 * DEGR;
    P.mRecip = 0.31;                // forged 76 mm piston, pin, 1/3 of the rod
    // crank + primary gear + the supercharger: its impeller and step-up gears seen at the crank
    // (2e-5 kg m^2 x 9.2^2 ~ 0.0017); still light enough to stall on a dumped clutch
    P.inertia = 0.034;
    P.vManifold = 0.6e-3;           // short runners from the throttle bodies
    P.throttleArea = 7.85e-3;       // four 50 mm throttle bodies
    P.idleRpm = 1200.0;             // Kawasaki: 1100 +-50; the bike tests idle at 1150-1350
    P.starterTorque = 45.0;         // 8.5:1 is easy to spin
    P.revLimit = 14000.0; P.revResume = 13600.0;
    P.fmepA = 0.70e5; P.fmepB = 0.05e5; P.fmepC = 0.0040e5;
    // a single, unswitched cam that breathes best toward the power peak; the boost grows with the revs.
    // Calibrated with --engine-type h2 --phystest, whose dyno sweep measures with the charge as hot as
    // a long full-throttle pull leaves it (~100 C here, no intercooler), without ram air: 133 N m at
    // 10500 and 147.5 kW at 11000 rpm, flat to 11500 (Kawasaki: 133.5 N m at 10500, 147.2 kW at 11000)
    P.lowCam = false;
    P.veBase = 0.66; P.vePeak = 0.26; P.veRpm = 9000.0; P.veWidth = 4500.0;
    P.veLowBase = 0.62; P.veLowPeak = 0.38; P.veLowRpm = 7500.0; P.veLowWidth = 5000.0;  // unused (lowCam off)
    P.sparkLow = 16.0; P.sparkHigh = 34.0; P.sparkFrom = 1300.0; P.sparkSpan = 7000.0;  // 8.5:1 takes more advance off boost; less with boost (knockRetard)
    P.rpmScale = 16000.0;
    P.blockMass = 64.0; P.blockRollInertia = 1.2;
    P.counterweight = 0.0;          // flat-plane inline-4: 1st order cancels by symmetry

    // the supercharger, in the turbo model's compressor (src/physics.hpp, turboParams)
    turboParams &T = P.turbo;
    T.supercharger = true;
    T.driveRatio = 9.2;
    T.driveEff = 0.95;
    T.inertia = 2.0e-5;             // impeller and shaft
    T.maxRpm = 140000.0;
    // the impeller's map: ~3.0x atmospheric at the surge line at 130k rpm; the engine runs well right
    // of it, so it makes ~2.0x at 10500 rpm. With no intercooler the boost also heats the charge (to
    // ~100 C), so it has to make up for its own heat...
    T.prRef = 3.0; T.rpmRef = 130000.0;
    T.reliefP = 203e3;              // ... and the plenum's relief valve holds it near +1.0 bar from ~10500 rpm
    T.reliefSpan = 15e3; T.reliefArea = 1.5e-3;
    T.flowPeakRef = 0.11;           // surge line at 130k rpm: the engine runs right of it at any rpm, so no surge at full throttle
    T.shutoffHead = 0.6; T.chokeFall = 0.12;
    T.etaC = 0.76;                  // Kawasaki claims a high efficiency for its impeller
    T.ductLA = 0.7 / 3.0e-3;        // ram-air duct from the fairing
    T.vCharge = 4.0e-3;             // the aluminium plenum (surge tank) and the outlet duct
    T.icDrop = 0.3e5;               // no intercooler core: just the duct
    T.icEffStill = 0.08; T.icEffMoving = 0.15;  // no intercooler: the plenum walls take a little heat
    T.icTau = 4.0;
    T.bovArea = 3.0e-4;             // the plenum's relief / bypass valve
    T.boostMap = 300e3;             // no wastegate: full grip opens the throttles wide and takes what it makes
    T.knockRetard = 4.0;            // 8.5:1 leaves room for the boost
    T.knockRetardHot = 0.05;
    T.ramGain = 0.7;                // ram air: ~+0.03 bar at 300 km/h

    // the H2's voice: the S 1000 RR's 4-2-1 bark, a deeper silencer, and the supercharger's whine
    // over everything (src/sound.hpp turboVoice: blade pass of the 6 blades and 6 splitters, the
    // rush of air, the bypass valve's chirp when the throttles snap shut)
    soundParams &S = P.sound;
    S.superbike = true;
    S.pipeResGain = 0.6;
    S.masterHz = 12000.0;
    S.drive = 0.25;
    S.radiateHz = 300.0;
    S.burble = 3.0;
    S.pipeLength = 1.25;
    S.pipeLowHz = 2400.0; S.pipeResHz = 200.0;
    S.intakeGain = 0.7;             // the airbox is the plenum: less honk, more whistle
    S.popChance = 0.25;
    S.whineGain = 0.025;            // dog-ring box, straight-cut gears
    S.turboGain = 1.6;
    S.tick = 0.4;
    return P;
}

// Kawasaki Ninja H2 (2015-): primary 1.480 (74/50), gears 51/16 48/19 45/22 38/22 32/21 31/23
// (3.188 2.526 2.045 1.727 1.524 1.348), chain 44/18 (2.444), 200/55 ZR17 rear (0.326 m).
// 238 kg wet plus an 80 kg rider. Dog-ring gearbox, quickshifter, back-torque limiting clutch.
drivetrainParams kawasakiH2DriveParams() {
    drivetrainParams P = superbikeDriveBase();  // superbike rules; Kawasaki's numbers follow
    P.primary = 74.0 / 50.0;
    P.teethIn = {0, 16, 19, 22, 22, 21, 23};
    P.teethOut = {0, 51, 48, 45, 38, 32, 31};
    ratiosFromTeeth(P);
    P.primaryIn = 50; P.primaryOut = 74;
    P.finalDrive = 44.0 / 18.0;
    P.sprocketTeeth = 18;
    P.wheelRadius = 0.326;
    P.carMass = 318.0;
    P.cdA = 0.33;
    P.clutchCapacity = 260.0;       // as the BMW: a heavier clutch would bog the 8.5:1 engine pulling away
    P.sportUp = 13600.0; P.sportDown = 9500.0;
    P.upPedal = 8600.0;             // full throttle: change up at ~12600, past the power peak
    P.launchPedal = 6000.0;         // launch control: slips the clutch at ~7500-9500 rpm, where the boost is
    return P;
}

bikeSpec kawasakiH2Spec() {
    return {"Kawasaki Ninja H2", 996, 1000, 14000, 14600,
            "peak torque (Kawasaki: 133.5 N m)", "peak torque rpm (Kawasaki: 10500)",
            "peak power (Kawasaki: 147.2 kW / 200 PS)", "peak power rpm (Kawasaki: 11000)",
            123, 145, 9000, 11500, 138, 158, 10500, 12500, 14000,
            {180.0, 180.0, 180.0, 180.0}, "even 180-180-180-180 (max error)",
            "H2 gearing, 318 kg with rider", 12000, 13600,
            "top speed after 30 s, 6th gear (Kawasaki: limited to 299)", 280, 335, 2.5,
            750};  // 41 kg more than the BMW on a softer 8.5:1 bottom end: a 25% take-off bogs deeper
}

// exhaust: the inline superbike 4-2-1 layout (src/exhaust_layouts.hpp: headers from the front of the
// inclined head, down in front of the crankcase and under the engine, collector, link pipe with the
// flap) into the H2's bigger black-chrome silencer on the right. The layout also places the
// supercharger (h2_supercharger.mesh): impeller axis across the bike, inlet facing left (-z), forward = +x.
exhaustLayout kawasakiH2Exhaust(const engineModel &e) {
    superbike421Options o;
    o.canRgba = {0.30f, 0.30f, 0.32f, 1.f};
    o.canR0 = 60.f; o.canR1 = 56.f;
    exhaustLayout L = superbike421Exhaust(e, o);
    L.turbo.on = true;
    L.turbo.at = meshPoint(e, "supercharger_at");  // build_h2.py's SC_AT, from h2.mesh
    L.turbo.x = {0.f, 0.f, -1.f};
    L.turbo.y = {0.f, 1.f, 0.f};
    L.turbo.z = {1.f, 0.f, 0.f};
    return L;
}

const engineType &kawasakiH2TunedType();  // --tuned yes, below

const engineType &kawasakiH2Type() {
    static const engineType T = [] {
        engineType t;
        t.id = "h2";
        t.aliases = "kawasaki kawasaki-h2 ninja-h2 ninjah2";
        t.name = "Kawasaki Ninja H2";
        t.summary = "Kawasaki Ninja H2 supercharged inline-4, 998 cc, 180 deg crank, H2 gearing and sound";
        t.label = "Kawasaki Ninja H2 supercharged inline-4, 998 cc";
        t.windowTitle = "Ninja H2 engine";
        t.credit = "H2 built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/kawasaki_h2/h2.mesh";
        t.buildHint = "python3 -I engines/kawasaki_h2/build_h2.py";
        t.bike = true;
        t.supercharged = true;
        t.engine = kawasakiH2Params;
        t.drive = kawasakiH2DriveParams;
        t.physTest = physicsTestBike;
        t.exhaust = kawasakiH2Exhaust;
        t.turboMeshFile = "engines/kawasaki_h2/h2_supercharger.mesh";
        t.turboBuildHint = "python3 -I engines/kawasaki_h2/build_h2.py";
        t.spec = kawasakiH2Spec;
        t.gearboxSide = -1.f;      // forward-turning crank, front at +x: the box sits toward -x
        t.primaryModule = 2.6f * 84.f / (50.f + 74.f);  // 50/74 at the Ducati's 30/54 shaft spacing
        // --wav: the supercharger whines up with the revs and chirps on each snap shut; then a launch
        // up the box and rev-matched downshifts
        t.ride = {
            {1.0, true, 0.0, 0, "starter"},          {3.0, false, 0.0, 0, "idle"},
            {3.3, false, 0.5, 0, "blip"},            {4.3, false, 0.0, 0, "idle, bypass chirp"},
            {4.6, false, 1.0, 0, "big blip"},        {5.8, false, 0.0, 0, "falling back"},
            {7.0, false, 1.0, 0, "full throttle in N, limiter"}, {10.5, false, 0.0, 0, "lift off, back to idle"},
            {11.0, false, 0.0, 1, "into 1st"},       {20.0, false, 1.0, 0, "launch, quickshifts up the box"},
            {20.6, false, 0.0, 0, "lift off, braking", true},
            {21.4, false, 0.0, -1, "quickshift down, rev-matched", true},
            {22.2, false, 0.0, -1, "quickshift down, rev-matched", true},
            {23.0, false, 0.0, -1, "quickshift down, rev-matched", true},
            {24.0, false, 0.0, 0, "brakes off, overrun", false},
            {27.0, false, 0.7, 0, "back on the throttle", false},
        };
        t.variants = {{"tuned", kawasakiH2TunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: a top-end tune ------------------------------------------------------------------
// What a race-kit H2 does to move the power peak up toward the 14000 rpm redline (estimates, not a
// published kit):
//   - higher-lift cams: the engine breathes best near 12000 rpm instead of 9000. It trades mid range
//     for top end: stock up to ~3000 rpm, 16-18% less torque at 6000-8000, stronger above ~10700
//   - a stiffer spring in the plenum relief valve: the boost keeps climbing with the revs (rpm^2)
//     instead of being held from ~10500
//   - race fuel and a remapped ECU: less spark taken out for boost and the hot charge
//   - an open exhaust and the ECU's exhaust valve held open: louder, more crackle
//   - the shift points moved up, to the power peak just under the limiter
// The block, supercharger, gearing and limiter are stock.
engineParams kawasakiH2TunedParams() {
    engineParams P = kawasakiH2Params();
    P.evo = 480.0 * DEGR;           // exhaust opens 10 deg earlier: blows down in time at 14000 rpm
    // calibrated with --engine-type h2 --tuned yes --phystest (hot charge, no ram air): 142 N m at
    // 12000 and 186 kW (250 hp) at 13500 rpm, against stock's 133 N m at 10500 and 147.5 kW at 11000
    P.veBase = 0.69; P.vePeak = 0.23; P.veRpm = 12000.0; P.veWidth = 4000.0;
    turboParams &T = P.turbo;
    T.reliefP = 225e3;              // stiffer relief spring: ~+1.2 bar at the top, against stock's ~+1.0
    T.knockRetard = 2.5;            // race fuel
    T.knockRetardHot = 0.03;
    soundParams &S = P.sound;       // open exhaust
    S.flapOpenRpm = 0.0; S.flapSpan = 1.0;
    S.flapShutGain = 1.0;
    S.popChance = 0.5;
    S.drive = 0.32;
    S.pipeLength = 1.1;             // the short race can (kawasakiH2TunedExhaust): a shorter pipe, a higher note
    S.pipeResHz = 230.0;
    return P;
}

// the race exhaust: a full titanium 4-2-1 system with no exhaust valve. The headers are a size up
// (41 mm instead of 38) and heat-blued; the stock route (under the engine, collector, link pipe) is
// kept, but the big black-chrome silencer gives way to a short, steeper titanium race can with a
// carbon end cap and a wide-open outlet.
exhaustLayout kawasakiH2TunedExhaust(const engineModel &e) {
    exhaustLayout L = kawasakiH2Exhaust(e);
    const std::array<float, 4> BLUED{0.45f, 0.42f, 0.55f, 1.f}, TI{0.62f, 0.60f, 0.56f, 1.f},
        CARBON{0.10f, 0.10f, 0.11f, 1.f};
    // the stock layout ends with the silencer and its end cap; everything before them is the pipework
    L.pipes.resize(L.pipes.size() - 2);
    int link = static_cast<int>(L.pipes.size()) - 1;
    for (pipeSpec &p : L.pipes) {
        if (p.cyl >= 0) { p.r0 = 20.5f; p.r1 = 20.5f; p.rgba = BLUED; }  // headers
        else p.rgba = TI;
        p.rough = 0.25f; p.metal = 0.95f;
    }
    pipeSpec can;
    can.pts = {{-250.f, 205.f, 140.f}, {-345.f, 135.f, 158.f}};
    can.r0 = 46.f; can.r1 = 42.f;
    can.from = {link};
    can.rgba = TI; can.rough = 0.25f; can.metal = 0.95f;
    can.capStart = can.capEnd = true;
    L.pipes.push_back(can);
    pipeSpec cap;
    cap.pts = {{-340.f, 139.f, 158.f}, {-362.f, 123.f, 161.f}};
    cap.r0 = 42.f; cap.r1 = 30.f;
    cap.from = {static_cast<int>(L.pipes.size()) - 1};
    cap.rgba = CARBON; cap.rough = 0.5f; cap.metal = 0.2f;
    cap.capStart = true;
    L.pipes.push_back(cap);
    pipeSpec tip;
    tip.pts = {{-360.f, 125.f, 161.f}, {-372.f, 116.f, 163.f}};
    tip.r0 = 26.f; tip.r1 = 26.f;
    tip.from = {static_cast<int>(L.pipes.size()) - 1};
    tip.rgba = TI; tip.rough = 0.25f; tip.metal = 0.95f;
    tip.capStart = true;
    L.pipes.push_back(tip);
    L.outlets = {{{-372.f, 116.f, 163.f}, {-12.f, -9.f, 2.f}, 26.f}};
    L.flap.on = false;              // valve deleted
    return L;
}

drivetrainParams kawasakiH2TunedDriveParams() {
    drivetrainParams P = kawasakiH2DriveParams();
    P.upPedal = 9600.0;             // full throttle: change up at ~13600, on the power peak
    P.sportUp = 13800.0; P.sportDown = 10500.0;
    return P;
}

bikeSpec kawasakiH2TunedSpec() {
    bikeSpec S = kawasakiH2Spec();
    S.name = "Kawasaki Ninja H2 (tuned)";
    S.torqueWhat = "peak torque (tuned)";
    S.torqueRpmWhat = "peak torque rpm (tuned: high in the range)";
    S.powerWhat = "peak power (tuned)";
    S.powerRpmWhat = "peak power rpm (tuned: near the 14000 redline)";
    S.torqueLo = 125; S.torqueHi = 165; S.torqueRpmLo = 11000; S.torqueRpmHi = 13500;
    S.powerLo = 165; S.powerHi = 215; S.powerRpmLo = 13000; S.powerRpmHi = 14000;
    S.bikeWhat = "H2 gearing, 318 kg with rider (tuned)";
    S.upshiftLo = 13000; S.upshiftHi = 14100;
    S.topWhat = "top speed after 30 s, 6th gear (tuned)";
    S.topLo = 290; S.topHi = 360;
    return S;
}

const engineType &kawasakiH2TunedType() {
    static const engineType T = [] {
        engineType t = kawasakiH2Type();
        t.id = "h2-tuned";
        t.name = "Kawasaki Ninja H2 (tuned)";
        t.summary = "Kawasaki Ninja H2, tuned: race cams, more boost, race fuel, open exhaust";
        t.label = "Kawasaki Ninja H2 supercharged inline-4, 998 cc, TUNED";
        t.windowTitle = "Ninja H2 engine (tuned)";
        t.hudTag = "TUNED";
        t.engine = kawasakiH2TunedParams;
        t.drive = kawasakiH2TunedDriveParams;
        t.spec = kawasakiH2TunedSpec;
        t.exhaust = kawasakiH2TunedExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(kawasakiH2Type, 40);
