// starting points for engines of a class: a generic superbike's engine and drivetrain values. An engine
// starts from a preset and sets its own values on top, so retuning one engine never moves another.
// Nothing here is any one bike's: these are the values the superbikes here share.
#pragma once
#include "physics.hpp"

// a 1000 cc class four-cylinder superbike engine: light crank, short intake, quick twist grip, a
// high-revving cam's spark map, big valve overlap; the voice and everything that sets one engine
// apart (bore, cams, VE curve, limiter, inertia, friction) are each engine's own
engineParams superbikeBaseParams() {
    engineParams P;
    P.blowdown = 14.0 * DEGR;
    P.idleBase = 1.6e-5; P.idleMin = 1.0e-5; P.idleMax = 1.0e-4;
    P.idleBand = 400.0;
    P.starterMaxRpm = 600.0;
    P.fmepA = 0.75e5; P.fmepB = 0.06e5;
    P.pedalRate = 10.0;             // twist grip
    P.residualMin = 0.55;           // big valve overlap: lots of exhaust left in at idle
    P.sparkLow = 10.0; P.sparkHigh = 36.0; P.sparkFrom = 1300.0;
    P.mapMin = 35e3;
    P.mountFreqY = 18.0; P.mountFreqRoll = 14.0;
    P.fuelCutRpm = 2500.0;
    return P;
}

// a superbike's drivetrain rules: sequential six-speed box, chain drive, wet slipper clutch, up/down
// quickshifter, launch and wheelie control, the rider's mass and a sporty automatic. Gearing (primary,
// gears, sprockets, tyre), mass, aero and the clutch capacity are each bike's own
drivetrainParams superbikeDriveBase() {
    drivetrainParams P;
    P.bike = true;
    P.gears = 6;
    P.ratios.assign(7, 0.0);        // each bike sets its own
    P.reverseRatio = 0.0;
    P.efficiency = 0.93;
    P.wheelInertia = 1.4;           // both wheels, discs and the tyres
    P.crr = 0.018;
    P.inputInertia = 0.004;         // clutch hub and gearbox shafts, referred to the crank
    P.brakeDecel = 10.0;
    P.shiftOpen = 0.04; P.shiftClose = 0.07;
    P.normalDown = 3000.0;
    P.matchHold = 0.3;
    P.clutchPressRate = 8.0; P.clutchReleaseRate = 1.2;
    P.openRpm = 1500.0; P.idleOpenRpm = 1700.0; P.biteSpan = 2000.0;
    P.launchPedal = 4500.0;         // launch control: slips the clutch at ~6000-8000 rpm at full throttle
    P.upBase = 4000.0; P.upMargin = 500.0;
    P.downMax = 12000.0; P.kickRpm = 9000.0; P.kickMax = 12500.0; P.loadDownMax = 10000.0;
    P.pickRpm = 6000.0; P.pickMax = 12000.0;
    P.slipper = 0.35;
    P.quickshiftTime = 0.05;
    P.wheelieAccel = 1.05 * 9.81;   // about where the front lifts with a rider on
    return P;
}

// gear ratios from the tooth counts (ratios[g] = teethOut[g] / teethIn[g], neutral 0)
void ratiosFromTeeth(drivetrainParams &P) {
    P.ratios.assign(P.teethIn.size(), 0.0);
    for (std::size_t g = 1; g < P.teethIn.size(); g++) P.ratios[g] = double(P.teethOut[g]) / P.teethIn[g];
}
