// The original engine: the Sketchfab "Inline-4 Engine" model (a 2.3 L inline-4 bottom end) driving a
// car with 2nd-generation (2009-2013) Honda Fit 5-speed manual gearing. Mesh: extract_parts.py
// splits scene.gltf / scene.bin into engine.mesh.
#pragma once
#include "../../src/engine_type.hpp"
#include "gearbox_5mt.hpp"
#include "test_inline4.hpp"

// engine: 85 mm bore, ~103 mm mean stroke (the model's throws are uneven: 108.4 mm on 1/4, 97.8 mm
// on 2/3, measured from the mesh), 10:1, 160 mm rods, flat-plane crank firing 1-3-4-2. Calibrated
// with --phystest: idle 800 rpm, 203 N m at 3500 rpm, 110 kW at 6500-7000 rpm, limiter 6800 rpm.
engineParams inline4FitParams() {
    engineParams P;
    P.bore = 0.085;
    P.compression = 10.0;
    P.ivc = 220.0 * DEGR;           // intake closes 40 deg after BDC
    P.evo = 500.0 * DEGR;           // exhaust opens 40 deg before BDC
    P.burn = 50.0 * DEGR;
    P.mRecip = 0.6;                 // piston + pin + 1/3 rod
    P.inertia = 0.13;               // crank + flywheel + 2/3 rods
    P.vManifold = 3.0e-3;
    P.throttleArea = 2.83e-3;       // 60 mm throttle bore
    P.idleRpm = 800.0;
    P.starterTorque = 140.0;
    P.starterMaxRpm = 300.0;
    P.revLimit = 6800.0; P.revResume = 6500.0;
    P.fmepA = 1.0e5; P.fmepB = 0.12e5; P.fmepC = 0.04e5;
    P.veBase = 0.80; P.vePeak = 0.12; P.veRpm = 4500.0; P.veWidth = 2800.0;
    P.sparkLow = 8.0; P.sparkHigh = 30.0; P.sparkFrom = 800.0; P.sparkSpan = 3500.0;
    P.rpmScale = 8000.0;
    P.blockMass = 150.0; P.blockRollInertia = 5.0;
    P.mountFreqY = 12.0; P.mountFreqRoll = 9.0;
    return P;
}

// Honda Fit 5MT (Honda's 2010 spec sheet): 3.308 / 1.870 / 1.303 / 0.949 / 0.727, reverse 3.308,
// final drive 4.62, 0.305 m tyres, 1300 kg car
drivetrainParams inline4FitDriveParams() {
    drivetrainParams P;
    P.ratios = {0.0, 3.308, 1.870, 1.303, 0.949, 0.727, 0.0};
    P.gears = 5;
    P.reverseRatio = 3.308;
    P.finalDrive = 4.62;
    P.wheelRadius = 0.305;
    P.carMass = 1300.0;
    P.cdA = 0.65;
    P.clutchCapacity = 350.0;       // well above the 203 N m peak
    return P;
}

// exhaust: a tubular 4-2-1 manifold (1+4 and 2+3, 360 deg apart in the 1-3-4-2 order), a close-coupled
// catalyst, a mid pipe, the silencer and a tailpipe, run back beside the gearbox. The model has no
// head, so the runners leave the block's side face just under the deck (mesh +x, "up" is -y, rear +z).
exhaustLayout inline4FitExhaust(const engineModel &e) {
    exhaustLayout L;
    const std::array<float, 4> STEEL{0.50f, 0.48f, 0.45f, 1.f}, ALU{0.62f, 0.62f, 0.60f, 1.f}, CHROME{0.80f, 0.80f, 0.82f, 1.f};
    const float PORT_Y = -232.f, PORT_X = 176.f;  // block side face is at x = 171, deck at y = -270
    auto runner = [&](int c, std::vector<vec3> mid) {
        pipeSpec p;
        float z = e.cyls[c].zCentre;
        p.pts = {{PORT_X, PORT_Y, z}, {PORT_X + 36.f, PORT_Y + 4.f, z}};
        p.pts.insert(p.pts.end(), mid.begin(), mid.end());
        p.r0 = 19.f; p.r1 = 19.f; p.cyl = c; p.rgba = STEEL; p.rough = 0.35f; p.metal = 0.85f;
        L.pipes.push_back(p);
    };
    // pair 1+4 merges outboard (x 265), pair 2+3 inboard (x 215)
    runner(0, {{248.f, -170.f, -110.f}, {264.f, -105.f, -12.f}, {265.f, -60.f, 15.f}});
    runner(3, {{248.f, -170.f, 110.f}, {264.f, -105.f, 42.f}, {265.f, -60.f, 15.f}});
    runner(1, {{226.f, -165.f, -32.f}, {216.f, -105.f, -18.f}, {215.f, -60.f, -5.f}});
    runner(2, {{226.f, -165.f, 32.f}, {216.f, -105.f, 8.f}, {215.f, -60.f, -5.f}});
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, std::vector<int> from, std::array<float, 4> c, bool caps = false) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.from = from; p.rgba = c; p.rough = 0.4f; p.metal = 0.8f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    // cast flange plate the four runners are welded to, bolted along the block's top edge
    L.boxes.push_back({{171.f, PORT_Y - 24.f, -192.f}, {182.f, PORT_Y + 24.f, 192.f}, {0.30f, 0.29f, 0.28f, 1.f}, 0.7f, 0.5f});
    int a = pipe({{265.f, -60.f, 15.f}, {265.f, -15.f, 15.f}, {250.f, 55.f, 45.f}, {240.f, 100.f, 60.f}}, 22.f, 22.f, {0, 1}, STEEL);
    int b = pipe({{215.f, -60.f, -5.f}, {215.f, -15.f, -5.f}, {232.f, 55.f, 40.f}, {240.f, 100.f, 60.f}}, 22.f, 22.f, {2, 3}, STEEL);
    int col = pipe({{240.f, 100.f, 60.f}, {240.f, 140.f, 110.f}, {240.f, 165.f, 170.f}, {240.f, 165.f, 210.f}}, 27.f, 30.f, {a, b}, STEEL);
    int cat = pipe({{240.f, 165.f, 200.f}, {240.f, 165.f, 360.f}}, 55.f, 55.f, {col}, ALU, true);
    int mid = pipe({{240.f, 165.f, 355.f}, {240.f, 165.f, 480.f}}, 25.f, 25.f, {cat}, ALU);
    int muf = pipe({{240.f, 165.f, 475.f}, {240.f, 165.f, 665.f}}, 66.f, 66.f, {mid}, ALU, true);
    pipe({{240.f, 165.f, 660.f}, {240.f, 165.f, 730.f}}, 22.f, 22.f, {muf}, CHROME);
    L.outlets.push_back({{240.f, 165.f, 730.f}, {0.f, 0.f, 1.f}, 22.f});
    return L;
}

const engineType &inline4FitType() {
    static const engineType T = [] {
        engineType t;
        t.id = "inline4";
        t.aliases = "fit honda-fit inline4-fit i4";
        t.name = "inline-4";
        t.summary = "2.3 L inline-4 (the Sketchfab model) in a car with Honda Fit 5-speed gearing";
        t.label = "inline-4, 2.3 L";
        t.firing = "1-3-4-2";
        t.windowTitle = "inline-4 engine";
        t.credit = "model: \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/inline4_fit/engine.mesh";
        t.buildHint = "python3 -I engines/inline4_fit/extract_parts.py";
        t.bike = false;
        t.engine = inline4FitParams;
        t.drive = inline4FitDriveParams;
        t.physTest = physicsTestInline4;
        t.exhaust = inline4FitExhaust;
        return t;
    }();
    return T;
}
