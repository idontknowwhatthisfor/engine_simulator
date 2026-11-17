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
        return t;
    }();
    return T;
}
