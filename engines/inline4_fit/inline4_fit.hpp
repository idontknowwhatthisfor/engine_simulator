// The original engine: the Sketchfab "Inline-4 Engine" model (a 2.3 L inline-4 bottom end) driving a
// car with 2nd-generation (2009-2013) Honda Fit 5-speed manual gearing. Mesh: extract_parts.py
// splits scene.gltf / scene.bin into engine.mesh.
#pragma once
#include "../../src/engine_type.hpp"
#include "gearbox_manual.hpp"
#include "test_inline4.hpp"

// engine: 85 mm bore, ~103 mm mean stroke (the model's throws are uneven: 108.4 mm on 1/4, 97.8 mm
// on 2/3, measured from the mesh), 160 mm rods, flat-plane crank firing 1-3-4-2: 2.34 L. Tuned to
// match a Honda K24Z3 (2008-12 Accord, 87 x 99 mm, 2.35 L, 10.5:1, 177 hp at 6500 rpm, 218 N m at
// 4300 rpm). Calibrated with --phystest: idle 800 rpm, 223 N m at 4000-4500 rpm, 131 kW (176 hp)
// at 6500 rpm, limiter 6800 rpm. The bore stays at the mesh's 85 mm, so the swept volume matches.
engineParams inline4FitParams() {
    engineParams P;
    P.bore = 0.085;
    P.compression = 10.5;
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
    P.veBase = 0.765; P.vePeak = 0.25; P.veRpm = 5200.0; P.veWidth = 3300.0;  // broad, high peak (K24 intake VTC)
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
    P.ratios = {0.0, 3.308, 1.870, 1.303, 0.949, 0.727};
    P.gears = 5;
    P.reverseRatio = 3.308;
    P.finalDrive = 4.62;
    P.wheelRadius = 0.305;
    P.carMass = 1300.0;
    P.cdA = 0.65;
    P.clutchCapacity = 350.0;       // well above the 223 N m peak
    return P;
}

// exhaust: a tubular 4-2-1 manifold (1+4 and 2+3, 360 deg apart in the 1-3-4-2 order), a close-coupled
// catalyst, a mid pipe, the silencer and a tailpipe, run back beside the gearbox. The model has no
// head, so the runners leave the block's side face just under the deck (mesh +x, "up" is -y, rear +z).
//
// With --turbo yes the two pairs merge above a turbocharger hung beside the block, shaft along the car:
// the turbine inlet faces up into the merge, the turbine outlet faces forward and the downpipe turns
// down and back under the turbo to the catalyst. The compressor (rear) draws through a cone filter
// and blows out sideways into a charge pipe that runs forward to a front-mounted intercooler; the
// cooled charge comes back up the other side toward the (unmodelled) throttle body.
exhaustLayout inline4FitExhaustWith(const engineModel &e, bool turbo) {
    exhaustLayout L;
    const std::array<float, 4> STEEL{0.50f, 0.48f, 0.45f, 1.f}, ALU{0.62f, 0.62f, 0.60f, 1.f}, CHROME{0.80f, 0.80f, 0.82f, 1.f};
    const float PORT_Y = -232.f, PORT_X = 88.f;  // the block's side wall under the deck is at x = 76-83 (deck y = -270)
    auto runner = [&](int c, std::vector<vec3> mid) {
        pipeSpec p;
        float z = e.cyls[c].zCentre;
        p.pts = {{PORT_X, PORT_Y, z}, {PORT_X + 50.f, PORT_Y + 2.f, z}, {212.f, PORT_Y + 4.f, z}};
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
    // cast flange plate the four runners are welded to, bolted flat on the block's side wall under the deck
    L.boxes.push_back({{78.f, PORT_Y - 24.f, -192.f}, {90.f, PORT_Y + 24.f, 192.f}, {0.30f, 0.29f, 0.28f, 1.f}, 0.7f, 0.5f});
    float lineY = 165.f;  // height of the catalyst / silencer line
    int col;
    if (!turbo) {
        int a = pipe({{265.f, -60.f, 15.f}, {265.f, -15.f, 15.f}, {250.f, 55.f, 45.f}, {240.f, 100.f, 60.f}}, 22.f, 22.f, {0, 1}, STEEL);
        int b = pipe({{215.f, -60.f, -5.f}, {215.f, -15.f, -5.f}, {232.f, 55.f, 40.f}, {240.f, 100.f, 60.f}}, 22.f, 22.f, {2, 3}, STEEL);
        col = pipe({{240.f, 100.f, 60.f}, {240.f, 140.f, 110.f}, {240.f, 165.f, 170.f}, {240.f, 165.f, 210.f}}, 27.f, 30.f, {a, b}, STEEL);
    } else {
        // turbo frame (src/turbo.hpp): shaft +x -> mesh +z (compressor at the rear), y -> y, z -> -x.
        // Ports, measured from turbo.mesh in that frame (mm): turbine inlet (45.7, -96.6, 64.1) facing -y,
        // turbine outlet (0, 0, 0) facing -x, compressor inlet (196.8, 0, 0) facing +x, compressor
        // outlet (153.3, 68.5, -85.5) facing -z
        L.turbo.on = true;
        L.turbo.at = {300.f, 115.f, -5.f};
        const vec3 T = L.turbo.at, IN{T.x - 64.1f, T.y - 96.6f, T.z + 45.7f};
        const vec3 MERGE{IN.x, IN.y - 38.f, IN.z};
        int a = pipe({{265.f, -60.f, 15.f}, {262.f, -40.f, 26.f}, {MERGE.x + 8.f, MERGE.y - 8.f, MERGE.z - 4.f}, MERGE}, 22.f, 22.f, {0, 1}, STEEL);
        int b = pipe({{215.f, -60.f, -5.f}, {218.f, -40.f, 16.f}, {MERGE.x - 6.f, MERGE.y - 8.f, MERGE.z - 4.f}, MERGE}, 22.f, 22.f, {2, 3}, STEEL);
        int feed = pipe({MERGE, {IN.x, IN.y + 6.f, IN.z}}, 27.f, 26.f, {a, b}, STEEL);
        // downpipe: forward out of the turbine, down, and back under the turbo
        lineY = 245.f;
        col = pipe({{T.x, T.y, T.z + 4.f}, {T.x, T.y, T.z - 40.f}, {T.x - 10.f, T.y + 75.f, T.z - 70.f},
                    {T.x - 35.f, lineY, T.z + 10.f}, {250.f, lineY, 150.f}, {240.f, lineY, 210.f}}, 30.f, 30.f, {feed}, STEEL);
        // intake side, cold: a hose and cone filter on the compressor inlet, a charge pipe off its outlet
        const std::array<float, 4> RUBBER{0.06f, 0.06f, 0.07f, 1.f}, FILTER{0.55f, 0.07f, 0.05f, 1.f};
        const vec3 CIN{T.x, T.y, T.z + 196.8f}, COUT{T.x + 85.5f, T.y + 68.5f, T.z + 153.3f};
        int hose = pipe({{CIN.x, CIN.y, CIN.z - 4.f}, {CIN.x, CIN.y, CIN.z + 40.f}}, 37.f, 37.f, {}, RUBBER);
        int filt = pipe({{CIN.x, CIN.y, CIN.z + 36.f}, {CIN.x, CIN.y, CIN.z + 150.f}}, 52.f, 64.f, {}, FILTER, true);
        L.pipes[hose].rough = L.pipes[filt].rough = 0.9f;
        L.pipes[hose].metal = L.pipes[filt].metal = 0.f;
        // front-mounted intercooler ahead of the crank snout: a finned core between two end tanks
        const float IC_Z0 = -430.f, IC_Z1 = -375.f, IC_Y0 = 40.f, IC_Y1 = 240.f, IC_L = -230.f, IC_R = 470.f;
        const std::array<float, 4> CORE{0.10f, 0.10f, 0.11f, 1.f};
        L.boxes.push_back({{IC_L + 40.f, IC_Y0, IC_Z0 + 5.f}, {IC_R - 40.f, IC_Y1, IC_Z1 - 5.f}, CORE, 0.8f, 0.4f});
        L.boxes.push_back({{IC_R - 40.f, IC_Y0 - 10.f, IC_Z0}, {IC_R, IC_Y1 + 10.f, IC_Z1}, ALU, 0.4f, 0.7f});
        L.boxes.push_back({{IC_L, IC_Y0 - 10.f, IC_Z0}, {IC_L + 40.f, IC_Y1 + 10.f, IC_Z1}, ALU, 0.4f, 0.7f});
        // hot side: compressor outlet forward into the right end tank
        pipe({{COUT.x - 4.f, COUT.y, COUT.z}, {COUT.x + 30.f, COUT.y, COUT.z}, {COUT.x + 50.f, COUT.y - 5.f, COUT.z - 40.f},
              {COUT.x + 50.f, COUT.y - 20.f, -60.f}, {COUT.x + 50.f, COUT.y - 20.f, -300.f}, {IC_R - 20.f, 150.f, IC_Z1 + 30.f},
              {IC_R - 20.f, 150.f, IC_Z1 - 10.f}}, 30.f, 30.f, {}, ALU);
        // cold side: out of the left end tank and up the intake side of the block
        pipe({{IC_L + 20.f, 150.f, IC_Z1 - 10.f}, {IC_L + 20.f, 150.f, IC_Z1 + 40.f}, {IC_L + 30.f, 40.f, -300.f},
              {IC_L + 40.f, -120.f, -240.f}, {IC_L + 45.f, -200.f, -200.f}}, 30.f, 30.f, {}, ALU, true);
    }
    int cat = pipe({{240.f, lineY, 200.f}, {240.f, lineY, 360.f}}, 55.f, 55.f, {col}, ALU, true);
    int mid = pipe({{240.f, lineY, 355.f}, {240.f, lineY, 480.f}}, 25.f, 25.f, {cat}, ALU);
    int muf = pipe({{240.f, lineY, 475.f}, {240.f, lineY, 665.f}}, 66.f, 66.f, {mid}, ALU, true);
    pipe({{240.f, lineY, 660.f}, {240.f, lineY, 730.f}}, 22.f, 22.f, {muf}, CHROME);
    L.outlets.push_back({{240.f, lineY, 730.f}, {0.f, 0.f, 1.f}, 22.f});
    return L;
}
exhaustLayout inline4FitExhaust(const engineModel &e) { return inline4FitExhaustWith(e, false); }
exhaustLayout inline4FitTurboExhaust(const engineModel &e) { return inline4FitExhaustWith(e, true); }

const engineType &inline4FitType() {
    static const engineType T = [] {
        engineType t;
        t.id = "inline4";
        t.aliases = "fit honda-fit inline4-fit i4";
        t.name = "inline-4";
        t.summary = "2.3 L inline-4 (the Sketchfab model) in a car with Honda Fit 5-speed gearing";
        t.label = "inline-4, 2.3 L";
        t.windowTitle = "inline-4 engine";
        t.credit = "model: \"Inline-4 Engine\" by rafaelaraujo55, CC-BY-4.0";
        t.meshFile = "engines/inline4_fit/engine.mesh";
        t.buildHint = "python3 -I engines/inline4_fit/extract_parts.py";
        t.bike = false;
        t.engine = inline4FitParams;
        t.drive = inline4FitDriveParams;
        t.physTest = physicsTestInline4;
        t.physTestTurbo = physicsTestInline4Turbo;
        t.exhaust = inline4FitExhaust;
        t.exhaustTurbo = inline4FitTurboExhaust;
        t.turboMeshFile = "engines/inline4_fit/turbo.mesh";
        t.turboBuildHint = "python3 -I engines/inline4_fit/build_turbo.py";
        t.turboCredit = "turbo: \"Car Turbocharger\" by ml-design, CC-BY-4.0";
        t.hudFiringLine = false;   // the car's HUD lists the strokes without the order's name
        return t;
    }();
    return T;
}

REGISTER_ENGINE(inline4FitType, 10);
