// calibration overrides: environment variables that replace engine parameters while you calibrate an
// engine, for any engine type. Read by every --phystest and by --wav (not by the interactive window).
// Unset variables leave the parameter alone, so without them nothing changes.
#pragma once
#include "physics.hpp"

void applyCalibrationEnv(engineParams &P) {
    auto env = [](const char *name, double &v) {
        if (const char *s = std::getenv(name)) v = std::atof(s);
    };
    // breathing: the main VE curve, and the low-cam lobe (BMW ShiftCam)
    env("VE_BASE", P.veBase); env("VE_PEAK", P.vePeak); env("VE_RPM", P.veRpm); env("VE_WIDTH", P.veWidth);
    env("VEL_BASE", P.veLowBase); env("VEL_PEAK", P.veLowPeak); env("VEL_RPM", P.veLowRpm); env("VEL_WIDTH", P.veLowWidth);
    // pressure-wave tuning: intake ram (runner length, bore, sound speed, gain, decay), exhaust scavenging
    env("RAM_LEN", P.ramLen); env("RAM_DIA", P.ramDia); env("RAM_C", P.ramSound); env("RAM_GAIN", P.ramGain); env("RAM_DECAY", P.ramDecay);
    env("SCAV_LEN", P.scavLen); env("SCAV_C", P.scavSound); env("SCAV_GAIN", P.scavGain); env("SCAV_DECAY", P.scavDecay);
    // friction, combustion, crank, starter
    env("FMEP_A", P.fmepA); env("FMEP_B", P.fmepB); env("FMEP_C", P.fmepC);
    env("CYC_IDLE", P.cycleCovIdle); env("CYC_FULL", P.cycleCovFull);  // cycle-to-cycle variation
    env("RESID", P.residualMin); env("ETA", P.etaHeat); env("INERTIA", P.inertia); env("STARTER", P.starterTorque);
    // idle control
    env("IDLE_KP", P.idleKp); env("IDLE_KI", P.idleKi); env("IDLE_SG", P.idleSparkGain);
    env("IDLE_BASE", P.idleBase); env("IDLE_SPARK", P.idleSparkBase); env("IDLE_MIN", P.idleMin);
    // supercharger / turbo compressor map and relief valve
    env("SC_PR", P.turbo.prRef); env("SC_FLOW", P.turbo.flowPeakRef); env("SC_CHOKE", P.turbo.chokeFall);
    env("SC_RELIEF", P.turbo.reliefP);
    // exhaust turbo: turbine nozzle, wastegate, boost target (Pa absolute), shaft inertia
    env("TURBO_AREA", P.turbo.turbineArea); env("TURBO_WG", P.turbo.wastegateArea); env("TURBO_BOOST", P.turbo.boostMap);
    env("TURBO_INERTIA", P.turbo.inertia); env("TURBO_KNOCK", P.turbo.knockRetard);
    // the ECU's boost target over rpm and the charge (torque) limit
    env("TURBO_FALL", P.turbo.boostFallRpm); env("TURBO_TOPRPM", P.turbo.boostTopRpm); env("TURBO_TOP", P.turbo.boostTop);
    env("TURBO_CHARGE", P.turbo.chargeMax);
    // sound (heard in --wav)
    soundParams &S = P.sound;
    env("SND_LOW", S.pipeLowHz); env("SND_OPEN", S.flapOpenHz); env("SND_DRIVE", S.drive);
    env("SND_REFLECT", S.pipeReflect); env("SND_DAMP", S.pipeDampHz); env("SND_LEN", S.pipeLength);
    env("SND_INTAKE", S.intakeGain); env("SND_RES", S.pipeResGain); env("SND_RAD", S.radiateHz);
    env("SND_BURBLE", S.burble); env("SND_BFROM", S.burbleFadeFrom); env("SND_BSPAN", S.burbleFadeSpan);
    env("SND_BHIGH", S.burbleHigh); env("SND_TICK", S.tick); env("SND_WHINE", S.whineGain);
    env("SND_POP", S.popChance); env("SND_TURBO", S.turboGain); env("SND_RASP", S.raspGain);
    env("SND_C", S.gasSpeed); env("SND_CHAMBER", S.chamberHz); env("SND_CLOSS", S.chamberLoss); env("SND_WALL", S.wallLoss);
    if (const char *v = std::getenv("SND_LAYOUT")) S.pipeFromLayout = std::atoi(v) != 0;  // 0: the single typed pipe
}
