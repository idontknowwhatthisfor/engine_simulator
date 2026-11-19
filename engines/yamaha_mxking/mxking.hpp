// Yamaha MX King 150 (Jupiter MX King, T150; the 2015-2020 model): a 149.7 cc liquid-cooled SOHC
// four-valve single with fuel injection, in an underbone with a manual clutch and a five-speed return
// box. Mesh: build_mxking.py.
//
// Yamaha Indonesia's spec sheet: 57.0 x 58.7 mm, 10.4:1, 11.3 kW (15.4 PS) at 8500 rpm, 13.8 N m at
// 7000 rpm, primary 73/24 (3.042), gears 34/12 30/16 30/22 24/21 22/23, final 40/14, 120/70-17 rear
// tyre, 117 kg wet. Not published, and chosen here: the rod length, the cam timing, the rev limiter
// (~10,300 rpm), the crank's inertia and the aero.
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"

// ---- engine: physics and sound (src/physics.hpp engineParams; SI units, angles in rad) ---------
engineParams mxkingParams() {
    engineParams P = superbikeBaseParams();  // the bike rules (twist grip, spark map shape); the rest is set here
    P.bore = 0.057;                 // must match BORE in build_mxking.py (the stroke comes from the mesh)
    P.compression = 10.4;
    P.ivc = 228.0 * DEGR;           // a commuter cam: intake shuts ~48 deg after BDC
    P.evo = 495.0 * DEGR;           // exhaust opens ~45 deg before BDC
    P.burn = 50.0 * DEGR;
    P.residualMin = 0.7;            // little overlap: a cleaner idle than the superbikes
    P.mRecip = 0.20;                // kg: 57 mm piston, pin, 1/3 of the rod
    P.inertia = 0.012;              // kg m^2: crank and the magneto's flywheel rotor
    P.vManifold = 0.25e-3;          // m^3: short intake runner behind the airbox
    P.throttleArea = 6.16e-4;       // m^2: one 28 mm throttle body
    // the idle valve, scaled from the superbike preset's 1000 cc four to 150 cc
    P.idleBase = 2.6e-6; P.idleMin = 1.6e-6; P.idleMax = 1.6e-5;
    P.idleRpm = 1400.0;
    P.starterTorque = 30.0;         // N m at the crank: enough to carry it over compression
    P.revLimit = 10300.0; P.revResume = 10000.0;
    P.pedalRate = 7.0;
    // VE: tuned to the published 13.8 N m at 7000 and 11.3 kW at 8500
    P.veBase = 0.56; P.vePeak = 0.48; P.veRpm = 7600.0; P.veWidth = 3300.0;
    P.fmepC = 0.03e5;              // a little less than the superbikes' at the top: 10,000 rpm, not 14,000
    P.sparkSpan = 5000.0;
    P.rpmScale = 12000.0;
    P.fuelCutRpm = 2200.0;
    // a single with the usual ~50% balance factor (modelled with no balancer shaft)
    P.counterweight = 0.5;
    P.blockMass = 28.0; P.blockRollInertia = 0.18;
    // the voice: one thump every other turn through a long header and a small silencer
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.superbike = true;
    S.pipeLength = 1.45;            // header, under the engine, into the silencer
    S.pipeLowHz = 1500.0; S.pipeResHz = 140.0; S.pipeResGain = 0.9;
    S.flapOpenRpm = 1e9;            // no exhaust valve: the silencer always sounds the same
    S.flapShutHz = 2200.0; S.flapShutGain = 0.8;
    S.masterHz = 9000.0;
    S.airboxHz = 160.0; S.intakeHowlHz = 600.0; S.intakeGain = 0.4;
    S.popRpm = 3000.0; S.popChance = 0.08;
    S.drive = 0.4;
    S.radiateHz = 250.0;
    S.burble = 3.5;
    S.tick = 0.15;
    return P;
}

// ---- drivetrain: gearbox, clutch, vehicle (drivetrainParams) -----------------------------------
drivetrainParams mxkingDriveParams() {
    drivetrainParams P = superbikeDriveBase();  // the bike drivetrain; an underbone's numbers below
    P.gears = 5;
    P.primary = 73.0 / 24.0;                    // 3.042
    P.primaryIn = 24; P.primaryOut = 73;
    P.teethIn = {0, 12, 16, 22, 21, 23};        // 2.833 1.875 1.364 1.143 0.957
    P.teethOut = {0, 34, 30, 30, 24, 22};
    ratiosFromTeeth(P);
    P.finalDrive = 40.0 / 14.0;
    P.sprocketTeeth = 14;
    P.wheelRadius = 0.300;                      // 120/70-17
    P.carMass = 187.0;                          // 117 kg wet and a 70 kg rider
    P.cdA = 0.42;                               // upright rider
    P.crr = 0.016;
    P.wheelInertia = 0.5;                       // two 17 inch spoked/cast wheels, narrow tyres
    P.inputInertia = 0.0008;
    P.clutchCapacity = 35.0;                    // N m at the crank: 2.5 x the peak, for a single's firing pulses
    P.slipper = 1.0;                            // no slipper clutch
    P.wheelieAccel = 0.0;                       // no wheelie control
    // the automatic: take-off and shift points for a 10,000 rpm single
    P.openRpm = 1700.0; P.idleOpenRpm = 1900.0; P.biteSpan = 1800.0;
    P.launchPedal = 3000.0;
    P.upBase = 3000.0; P.upPedal = 6600.0;      // 9600 rpm at full throttle
    P.upMargin = 400.0;
    P.normalDown = 2500.0;
    P.sportUp = 9800.0; P.sportDown = 7000.0;
    P.downMax = 9500.0; P.kickRpm = 7000.0; P.kickMax = 9500.0; P.loadDownMax = 8000.0;
    P.pickRpm = 5000.0; P.pickMax = 9500.0;
    P.qsMaxRpm = 10000.0;
    P.qsMatchTimeout = 0.4;                     // the heavier crank takes longer to blip up to the lower gear
    P.clutchReleaseRate = 0.7;                  // a commuter's lever, let out gently
    return P;
}

// ---- --phystest targets (src/engine_type.hpp bikeSpec) -----------------------------------------
bikeSpec mxkingSpec() {
    bikeSpec S{};
    S.name = "Yamaha MX King 150";
    S.dispLo = 148; S.dispHi = 152;             // cc: 149.7 quoted
    S.limLo = 10300; S.limHi = 10800;
    S.torqueWhat = "peak torque (13.8 N m quoted)"; S.torqueLo = 13.0; S.torqueHi = 14.6;
    S.torqueRpmWhat = "peak torque rpm (7000 quoted)"; S.torqueRpmLo = 6000; S.torqueRpmHi = 8000;
    S.powerWhat = "peak power (11.3 kW quoted)"; S.powerLo = 10.7; S.powerHi = 11.9;
    S.powerRpmWhat = "peak power rpm (8500 quoted)"; S.powerRpmLo = 8000; S.powerRpmHi = 9000;
    S.sweepTo = 10500;
    S.pattern = {720.0};                        // a single: one firing every two turns
    S.patternWhat = "single, every 720 deg (max error)";
    S.bikeWhat = "MX King gearing, 187 kg with rider";
    S.upshiftLo = 9000; S.upshiftHi = 10300;
    S.topWhat = "top speed after 30 s (estimate ~115-125 km/h)"; S.topLo = 105; S.topHi = 130;
    S.neutralSettle = 3.5;                      // the heavier crank takes longer to fall back to idle
    // a 150 cc single, not a superbike
    S.crankLo = 300; S.crankHi = 700;
    S.flareLo = 1500; S.flareHi = 3500;
    S.idleLo = 1300; S.idleHi = 1500;
    S.idleGearLo = 1250; S.idleGearHi = 1600;
    S.closedLo = 1250; S.closedHi = 1700;
    S.toLimiterLo = 0.3; S.toLimiterHi = 2.0;
    S.shakeRpm = 8000;
    S.shakeMax1st = 2300;                       // N: the test sees only the
                                                // vertical force, and with the barrel laid at 70 deg the
                                                // counterweight adds to it (it would trade fore-aft shake for it)
    S.zeroTo100What = "0-100 km/h (estimate ~12-14 s)"; S.zeroTo100Lo = 10.0; S.zeroTo100Hi = 16.0;
    S.zeroTo200What = nullptr;                  // it never gets there
    S.launchAccelLo = 0.1; S.launchAccelHi = 0.5;
    S.shiftRpm = 9000; S.shiftRpmGentle = 8000;
    S.neutralRevMin = 6500;
    S.rejoinHi = 10500;
    S.takeoffMinRpm = 700;                      // a single lugs down on the gentle take-off
    S.stallsOnClutchDump = false;               // the flywheel drags it through: it nearly stalls, then pulls
    S.dumpMinRpm = 120;                         // ... from as low as this
    S.liftsFront = false;
    S.downshiftMaxMs = 450;                     // the 1st-2nd step is 1.51: a long blip for a single
    return S;
}

// ---- exhaust (src/exhaust.hpp): one header out of the underside of the laid-down head, back under
// the crankcase and up into a silencer on the right ---------------------------------------------
exhaustLayout mxkingExhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> STEEL{0.62f, 0.60f, 0.58f, 1.f};
    const std::array<float, 4> GUARD{0.08f, 0.08f, 0.09f, 1.f};  // the black heat shield on the silencer
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, bool caps = false,
                    std::array<float, 4> rgba = {}) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from;
        p.rgba = rgba[3] > 0.f ? rgba : STEEL;
        p.rough = 0.3f; p.metal = 0.9f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    const cylinderGeom &g = e.cyls[0];
    vec3 port = cylPoint(e, 0, 0.5f * g.bore + 12.f, g.deckY - 28.f);  // the head's underside
    vec3 out = port + cylDir(e, 0, 1.f, 0.f) * 30.f;
    int header = pipe({port, out, {out.x - 10.f, 120.f, 10.f}, {40.f, 128.f, 40.f}, {-60.f, 122.f, 80.f},
                       {-150.f, 105.f, 110.f}}, 14.f, 15.f, 0, {});
    pipe({{-150.f, 105.f, 110.f}, {-410.f, 20.f, 125.f}}, 40.f, 34.f, -1, {header}, true, GUARD);  // the silencer
    L.outlets.push_back({{-410.f, 20.f, 125.f}, {-40.f, -13.f, 2.f}, 14.f});
    return L;
}

const engineType &mxkingTunedType();  // --tuned yes, below

// ---- the engine type -------------------------------------------------------------------------------
const engineType &mxkingType() {
    static const engineType T = [] {
        engineType t;
        t.id = "mxking";
        t.aliases = "mx-king jupiter-mx t150 yamaha150";
        t.name = "Yamaha MX King 150";
        t.summary = "Yamaha MX King 150 (2015-2020): 149.7 cc liquid-cooled SOHC single, 5-speed underbone";
        t.label = "Yamaha MX King 150, 149.7 cc single";
        t.firingLabel = "single, every 720 deg";
        t.windowTitle = "Yamaha MX King 150";
        t.credit = "built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/yamaha_mxking/mxking.mesh";
        t.buildHint = "python3 -I engines/yamaha_mxking/build_mxking.py";
        t.bike = true;
        t.engine = mxkingParams;
        t.drive = mxkingDriveParams;
        t.physTest = physicsTestBike;
        t.spec = mxkingSpec;
        t.exhaust = mxkingExhaust;
        t.gearboxSide = -1.f;
        t.primaryModule = 1.8f * 84.f / (24.f + 73.f);  // smaller gears than the superbikes'
        t.variants = {{"tuned", mxkingTunedType}};
        return t;
    }();
    return T;
}

// ---- --tuned yes: a road-race MX King ------------------------------------------------------------
// What the Indonesian underbone road-race builds do to the T150 engine (a typical build, not one
// team's or a published kit; every number is an estimate):
//   - bore-up: a 62 mm forged piston in a replated barrel on the stock crank, 177 cc
//     (mxking_race.mesh, written by build_mxking.py with the stock engine)
//   - 13.2:1, a race cam with more lift and overlap, stiffer valve springs: breathes best near
//     10,500 rpm, limiter at 13,000
//   - a 32 mm throttle body and an open race filter, an ECU remapped for race fuel
//   - a lighter magneto rotor: revs quicker, idles higher (1800 rpm) and rougher
//   - an open race pipe: a fatter header with an expansion chamber under the engine and a short can
//   - a stripped bike (~100 kg) with a 60 kg rider tucked in, a race clutch, the stock gearbox and
//     sprockets
engineParams mxkingTunedParams() {
    engineParams P = mxkingParams();
    P.bore = 0.062;                 // must match RACE_BORE in build_mxking.py
    P.compression = 13.2;
    P.ivc = 250.0 * DEGR;           // intake shuts ~70 deg after BDC
    P.evo = 475.0 * DEGR;           // exhaust opens ~65 deg before BDC
    P.burn = 46.0 * DEGR;           // faster burn: more squish, more advance
    P.residualMin = 0.55;           // big overlap: a lumpy idle
    P.mRecip = 0.21;                // forged slipper piston: a bigger bore for little more mass
    P.inertia = 0.011;              // lighter magneto rotor (0.012 stock)
    P.vManifold = 0.20e-3;          // short race manifold
    P.throttleArea = 8.04e-4;       // one 32 mm throttle body
    P.idleBase = 3.6e-6; P.idleMin = 2.2e-6; P.idleMax = 2.4e-5;
    P.idleRpm = 1800.0;
    P.starterTorque = 45.0;         // 13.2:1 is harder to crank over
    P.revLimit = 13000.0; P.revResume = 12700.0;
    P.pedalRate = 10.0;
    // calibrated with --engine-type mxking --tuned yes --phystest: 18.8 N m at 11000 and 22.0 kW
    // (29.5 hp) at 11500 rpm, against stock's 13.9 N m at 7000 and 11.2 kW at 8000; peaky below 8000
    P.veBase = 0.55; P.vePeak = 0.74; P.veRpm = 11300.0; P.veWidth = 3400.0;
    P.sparkFrom = 1800.0; P.sparkSpan = 6000.0;
    P.rpmScale = 14000.0;
    P.fuelCutRpm = 2800.0;
    soundParams &S = P.sound;       // open race pipe
    S.pipeLength = 1.2;             // expansion chamber and a short can
    S.pipeLowHz = 2200.0; S.pipeResHz = 180.0; S.pipeResGain = 1.1;
    S.flapShutHz = 4000.0; S.flapShutGain = 1.0;
    S.intakeGain = 0.9;             // open filter: the intake honk comes through
    S.popChance = 0.4;
    S.drive = 0.5;
    S.radiateHz = 300.0;
    S.burble = 4.0;
    S.tick = 0.25;
    return P;
}

drivetrainParams mxkingTunedDriveParams() {
    drivetrainParams P = mxkingDriveParams();
    P.carMass = 160.0;              // ~100 kg stripped and a 60 kg rider
    P.cdA = 0.33;                   // tucked in behind a small screen
    P.wheelInertia = 0.4;           // lighter rims
    P.clutchCapacity = 50.0;        // race clutch springs
    P.clutchReleaseRate = 1.2;
    P.openRpm = 2200.0; P.idleOpenRpm = 2400.0; P.biteSpan = 2000.0;
    P.launchPedal = 4500.0;
    P.upBase = 3000.0; P.upPedal = 9300.0;  // full throttle: change up at ~12,300
    P.normalDown = 3500.0;
    P.sportUp = 12500.0; P.sportDown = 9000.0;
    P.downMax = 12000.0; P.kickRpm = 9000.0; P.kickMax = 12000.0; P.loadDownMax = 10000.0;
    P.pickRpm = 7000.0; P.pickMax = 12000.0;
    P.qsMaxRpm = 12700.0;
    return P;
}

bikeSpec mxkingTunedSpec() {
    bikeSpec S = mxkingSpec();
    S.name = "Yamaha MX King race (177 cc)";
    S.dispLo = 175; S.dispHi = 179;
    S.limLo = 13000; S.limHi = 13500;
    S.torqueWhat = "peak torque (race build, ~18-20 N m)"; S.torqueLo = 17.5; S.torqueHi = 20.5;
    S.torqueRpmWhat = "peak torque rpm (race: high up)"; S.torqueRpmLo = 9500; S.torqueRpmHi = 11500;
    S.powerWhat = "peak power (race build, ~20-22 kW)"; S.powerLo = 19.5; S.powerHi = 22.5;
    S.powerRpmWhat = "peak power rpm (race: near the limiter)"; S.powerRpmLo = 11000; S.powerRpmHi = 12500;
    S.sweepTo = 13000;
    S.bikeWhat = "MX King gearing, 160 kg with rider (race)";
    S.upshiftLo = 11800; S.upshiftHi = 13000;
    S.topWhat = "top speed after 30 s (race, estimate)"; S.topLo = 145; S.topHi = 180;
    S.flareLo = 2000; S.flareHi = 5000;
    S.idleLo = 1650; S.idleHi = 1950;
    S.idleGearLo = 1600; S.idleGearHi = 2100;
    S.closedLo = 1600; S.closedHi = 2300;
    S.neutralIdleLo = 1000; S.neutralIdleHi = 2700;
    S.toLimiterLo = 0.2; S.toLimiterHi = 1.5;
    S.shakeRpm = 10000;
    S.shakeMax1st = 4000;                       // N: the bigger piston at 10,000 rpm
    S.zeroTo100What = "0-100 km/h (race, estimate)"; S.zeroTo100Lo = 5.5; S.zeroTo100Hi = 10.0;
    S.launchAccelLo = 0.4; S.launchAccelHi = 0.8;  // the light bike pulls harder off the line
    S.shiftRpm = 11500; S.shiftRpmGentle = 10000;
    S.neutralRevMin = 8000;
    S.rejoinHi = 13500;
    return S;
}

// the race pipe: a fatter, heat-blued header swelling into an expansion chamber under the crankcase,
// then a short brushed-steel can with a carbon end cap, high and tucked in
exhaustLayout mxkingTunedExhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> BLUED{0.45f, 0.42f, 0.55f, 1.f}, STEEL{0.70f, 0.69f, 0.67f, 1.f},
        CARBON{0.10f, 0.10f, 0.11f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, std::array<float, 4> rgba,
                    float rough, float metal, bool capStart, bool capEnd) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from; p.rgba = rgba;
        p.rough = rough; p.metal = metal;
        p.capStart = capStart; p.capEnd = capEnd;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    const cylinderGeom &g = e.cyls[0];
    vec3 port = cylPoint(e, 0, 0.5f * g.bore + 12.f, g.deckY - 28.f);
    vec3 out = port + cylDir(e, 0, 1.f, 0.f) * 30.f;
    int header = pipe({port, out, {out.x - 10.f, 120.f, 10.f}, {60.f, 128.f, 30.f}}, 16.f, 18.f, 0, {}, BLUED, 0.25f, 0.95f,
                      false, false);
    // the expansion chamber: swells out under the crankcase and back in
    int body = pipe({{60.f, 128.f, 30.f}, {0.f, 132.f, 55.f}, {-70.f, 126.f, 80.f}}, 18.f, 34.f, -1, {header}, BLUED,
                    0.25f, 0.95f, false, false);
    int cone = pipe({{-70.f, 126.f, 80.f}, {-140.f, 105.f, 105.f}, {-175.f, 85.f, 115.f}}, 34.f, 14.f, -1, {body}, BLUED,
                    0.25f, 0.95f, false, false);
    int can = pipe({{-175.f, 85.f, 115.f}, {-330.f, 20.f, 125.f}}, 30.f, 28.f, -1, {cone}, STEEL, 0.35f, 0.9f, true, true);
    int cap = pipe({{-325.f, 22.f, 125.f}, {-348.f, 12.f, 126.f}}, 28.f, 18.f, -1, {can}, CARBON, 0.5f, 0.2f, true, false);
    pipe({{-346.f, 13.f, 126.f}, {-358.f, 8.f, 126.f}}, 14.f, 14.f, -1, {cap}, STEEL, 0.3f, 0.9f, true, false);
    L.outlets.push_back({{-358.f, 8.f, 126.f}, {-12.f, -5.f, 0.f}, 14.f});
    return L;
}

const engineType &mxkingTunedType() {
    static const engineType T = [] {
        engineType t = mxkingType();
        t.id = "mxking-tuned";
        t.name = "Yamaha MX King race (177 cc)";
        t.summary = "Yamaha MX King road-race build: 62 mm bore-up to 177 cc, race cam, 32 mm throttle, open pipe";
        t.label = "Yamaha MX King race single, 177 cc, TUNED";
        t.windowTitle = "Yamaha MX King (race)";
        t.hudTag = "RACE";
        t.meshFile = "engines/yamaha_mxking/mxking_race.mesh";
        t.engine = mxkingTunedParams;
        t.drive = mxkingTunedDriveParams;
        t.spec = mxkingTunedSpec;
        t.exhaust = mxkingTunedExhaust;
        t.variants.clear();  // a variant has no variants of its own
        return t;
    }();
    return T;
}

REGISTER_ENGINE(mxkingType, 50);
