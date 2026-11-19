// Template engine: copy this folder to start a new one (README.md, "Adding an engine"). Replace
// `myengine` with the engine's id and `_template` with the folder name, in the file names and inside
// the files. The Makefile skips folders starting with _, so this one is neither built nor registered.
//
// As it stands it is a working but UNCALIBRATED 748 cc inline-3 bike (74 x 58 mm, built by
// build_myengine.py): it builds, starts, idles and rides, but its --phystest targets are placeholders.
// Calibrate it: set the targets in myengineSpec from the published figures, then tune the VE curve,
// friction, inertia and idle values until `./engine_sim --engine-type myengine --phystest` passes.
// The VE_*, FMEP_*, IDLE_*, INERTIA... environment variables (src/calibration.hpp) override values
// while you try them.
#pragma once
#include "../../src/engine_type.hpp"
#include "../../src/presets.hpp"

// ---- engine: physics and sound (src/physics.hpp engineParams; SI units, angles in rad) ---------
engineParams myengineParams() {
    engineParams P = superbikeBaseParams();  // a superbike starting point (src/presets.hpp); a car
                                             // starts from engineParams() and sets everything
    P.bore = 0.074;                 // must match BORE in build_myengine.py (the stroke comes from the mesh)
    P.compression = 12.5;
    P.ivc = 240.0 * DEGR;           // intake closes, deg of the 720 deg cycle from intake TDC
    P.evo = 485.0 * DEGR;           // exhaust opens
    P.burn = 46.0 * DEGR;           // combustion duration
    P.mRecip = 0.36;                // kg per cylinder: piston, pin, 1/3 of the rod
    P.inertia = 0.028;              // kg m^2: crank, flywheel, primary gear
    P.vManifold = 0.7e-3;           // m^3 of intake plenum
    P.throttleArea = 4.56e-3;       // m^2, all throttle bodies together (three 44 mm)
    // the preset's idle valve is sized for a 1000 cc four: about 3/4 of it for 750 cc
    P.idleBase = 1.2e-5; P.idleMin = 0.75e-5; P.idleMax = 0.75e-4;
    P.idleRpm = 1250.0;
    P.starterTorque = 38.0;         // N m at the crank (cranking should stay in 200-500 rpm)
    P.revLimit = 12500.0; P.revResume = 12100.0;
    // volumetric efficiency: veBase + vePeak * exp(-((rpm - veRpm) / veWidth)^2). The main torque
    // curve knob: calibrate against the published torque and power
    P.veBase = 0.55; P.vePeak = 0.55; P.veRpm = 10000.0; P.veWidth = 7000.0;
    P.sparkSpan = 7000.0;
    P.rpmScale = 14000.0;           // the HUD rev counter's full scale
    P.counterweight = 0.0;          // an even-firing inline-3 has no 1st-order vertical force to balance
    P.blockMass = 50.0; P.blockRollInertia = 0.9;
    // the voice (src/sound.hpp): the detailed stages, with the S 1000 RR's settings as a start
    P.sound = soundParams();
    soundParams &S = P.sound;
    S.detailed = true;
    S.pipeLowHz = 2600.0; S.pipeResHz = 220.0; S.pipeResGain = 0.6;
    S.masterHz = 12000.0;
    S.intakeGain = 1.0;
    S.popChance = 0.3;
    S.drive = 0.3;
    S.radiateHz = 300.0;
    S.burble = 3.0;
    return P;
}

// ---- drivetrain: gearbox, clutch, vehicle (drivetrainParams) -----------------------------------
drivetrainParams myengineDriveParams() {
    drivetrainParams P = superbikeDriveBase();  // a bike: six-speed sequential box, chain, quickshifter
    P.primary = 1.80;                           // crank to clutch
    P.primaryIn = 45; P.primaryOut = 81;        // the primary gears' teeth (drawn, and the whine)
    P.teethIn = {0, 15, 19, 22, 24, 26, 27};   // per gear (index 0 = neutral): gears = size - 1
    P.teethOut = {0, 38, 37, 35, 33, 32, 30};
    ratiosFromTeeth(P);
    P.finalDrive = 45.0 / 16.0;                 // rear / front sprocket
    P.sprocketTeeth = 16;
    P.wheelRadius = 0.310;                      // m, rear tyre
    P.carMass = 270.0;                          // kg, bike and rider
    P.cdA = 0.32;                               // m^2
    P.clutchCapacity = 200.0;                   // N m, well above the peak torque
    P.sportUp = 12000.0; P.sportDown = 8500.0;  // automatic sport mode, rpm
    P.upPedal = 8000.0;                         // upshift at upBase (4000) + upPedal * pedal: 12000 at full throttle
    return P;
}

// ---- --phystest targets (src/engine_type.hpp bikeSpec) -----------------------------------------
// PLACEHOLDERS: wide windows around what this uncalibrated engine does. Put the published figures
// here (with a sensible tolerance) and calibrate until they pass. The fields left out keep their
// superbike defaults (idle 1150-1350 rpm, shifts at 11000 rpm...); set them for an engine that idles
// or revs differently (the shift targets must stay below revLimit).
bikeSpec myengineSpec() {
    bikeSpec S{};
    S.name = "myengine inline-3";
    S.dispLo = 740; S.dispHi = 756;             // cc
    S.limLo = 12500; S.limHi = 13100;           // peak rpm bouncing on the limiter
    S.torqueWhat = "peak torque (placeholder)"; S.torqueLo = 50; S.torqueHi = 120;
    S.torqueRpmWhat = "peak torque rpm (placeholder)"; S.torqueRpmLo = 6000; S.torqueRpmHi = 12000;
    S.powerWhat = "peak power (placeholder)"; S.powerLo = 60; S.powerHi = 130;
    S.powerRpmWhat = "peak power rpm (placeholder)"; S.powerRpmLo = 8000; S.powerRpmHi = 12500;
    S.sweepTo = 12500;                          // dyno sweep top, rpm
    S.pattern = {240.0, 240.0, 240.0};          // firing intervals, deg: must match the mesh (selftest checks)
    S.patternWhat = "even 240-240-240 (max error)";
    S.bikeWhat = "myengine gearing, 270 kg with rider";
    S.upshiftLo = 10000; S.upshiftHi = 12500;   // rpm at the 1 -> 2 upshift, full throttle
    S.topWhat = "top speed after 30 s, 6th gear (placeholder)"; S.topLo = 200; S.topHi = 320;
    S.neutralSettle = 2.5;                      // s for the revs to fall back to idle after N at speed
    // where this engine is not a 1000 cc superbike (the defaults); see bikeSpec for all of them
    S.zeroTo100What = "0-100 km/h (placeholder)"; S.zeroTo100Lo = 3.0; S.zeroTo100Hi = 5.0;
    S.zeroTo200What = "0-200 km/h (placeholder)"; S.zeroTo200Lo = 8.0; S.zeroTo200Hi = 16.0;
    S.takeoffMinRpm = 700;                      // lowest rpm in the gentle manual take-off
    // S.stallsOnClutchDump = false;            // for a crank heavy enough to pull away from a clutch dump
    S.liftsFront = false;                       // ~65 kW: not enough to wheelie, so no wheelie-control checks
    S.downshiftMaxMs = 300;                     // revs up slower than a superbike for the downshift blip
    return S;
}

// ---- exhaust (src/exhaust.hpp): one header per cylinder, any count --------------------------------
// From each port at the front of the inclined head, forward and down in front of the crankcase into
// one collector under the engine, then a link pipe (with an exhaust flap) and a silencer rising on the
// right behind the gearbox. Points are mesh space (mm, front +x, "up" -y); cylPoint / cylDir work in
// a cylinder's own frame (bore toward -y). See src/exhaust_layouts.hpp for a 4-2-1.
exhaustLayout myengineExhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> STEEL{0.62f, 0.60f, 0.58f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, bool caps = false) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from; p.rgba = STEEL;
        p.rough = 0.3f; p.metal = 0.9f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    std::vector<int> headers;
    for (int c = 0; c < static_cast<int>(e.cyls.size()); c++) {
        const cylinderGeom &g = e.cyls[c];
        vec3 port = cylPoint(e, c, 0.5f * g.bore + 20.f, g.deckY - 41.f);  // front face of the head
        vec3 out = port + cylDir(e, c, 1.f, 0.f) * 40.f;
        float z = g.zCentre;
        headers.push_back(pipe({port, out, {220.f, -60.f, z}, {235.f, 60.f, z}, {200.f, 180.f, 0.6f * z},
                                {120.f, 225.f, 0.15f * z}, {80.f, 235.f, 0.f}}, 17.f, 18.f, c, {}));
    }
    int col = pipe({{80.f, 235.f, 0.f}, {20.f, 242.f, 0.f}, {-20.f, 245.f, 0.f}}, 26.f, 28.f, -1, headers);
    int link = pipe({{-20.f, 245.f, 0.f}, {-150.f, 240.f, 40.f}, {-230.f, 215.f, 110.f}, {-262.f, 199.f, 143.f}}, 28.f, 28.f,
                    -1, {col});
    pipe({{-250.f, 205.f, 140.f}, {-400.f, 110.f, 165.f}}, 48.f, 46.f, -1, {link}, true);  // the silencer
    L.outlets.push_back({{-400.f, 110.f, 165.f}, {-40.f, -23.f, 5.f}, 30.f});              // where the crackle comes out
    L.flap.on = true;                                                                      // the exhaust valve
    L.flap.at = {-190.f, 230.f, 75.f};
    L.flap.axis = normalize(vec3{-1.f, -0.25f, 0.8f});
    L.flap.shaft = {0.f, -1.f, 0.f};
    L.flap.pipeR = 28.f;
    return L;
}

// ---- the engine type: what --engine-type myengine selects ---------------------------------------
const engineType &myengineType() {
    static const engineType T = [] {
        engineType t;
        t.id = "myengine";                  // --engine-type value; must be unique
        t.aliases = "";                     // other names, space separated
        t.name = "myengine inline-3";       // tests and messages
        t.summary = "uncalibrated 748 cc inline-3 bike, made from the engine template";  // --engine-type list
        t.label = "myengine inline-3, 748 cc";  // HUD
        t.windowTitle = "myengine";
        t.credit = "built from \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";  // the model's licence wants this
        t.meshFile = "engines/_template/myengine.mesh";
        t.buildHint = "python3 -I engines/_template/build_myengine.py";
        t.bike = true;                      // bike drivetrain, gearbox drawing and physics test
        t.engine = myengineParams;
        t.drive = myengineDriveParams;
        t.physTest = physicsTestBike;       // a car writes its own (see engines/inline4_fit/test_inline4.hpp)
        t.spec = myengineSpec;
        t.exhaust = myengineExhaust;
        t.gearboxSide = -1.f;               // crank turning forward, front at +x: the box sits toward -x
        t.primaryModule = 2.6f * 84.f / (45.f + 81.f);  // primary gear module, mm (fits the shaft spacing)
        // optional: the --wav demo ride (see engines/bmw_s1000rr/bmw_s1000rr.hpp); empty: the stand script
        // optional: other versions in t.variants = {{"tuned", myengineTunedType}} (--variant tuned)
        return t;
    }();
    return T;
}

// the key orders --engine-type list (inline4 10, v4 20, s1000rr 30, h2 40): pick a free one. The type
// function must not look other engines up (findEngineType) while it builds its engineType
REGISTER_ENGINE(myengineType, 100);
