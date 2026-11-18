// --phystest for the superbikes, checked against each engine's bikeSpec (engines/<type>/)
#pragma once
#include "engines.hpp"

// headless physics checks for the superbike engines (--engine-type v4 | s1000rr --phystest): start, idle,
// free rev, dyno curve, balance, firing order, then on the bike
int physicsTestBike(const engineModel &e, const engineType &type) {
    const bikeSpec S = type.spec();
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0, true);  // engine checks on the stand; the bike comes after
    const double REV_LIMIT = sim.P.revLimit;
    if (const char *v = std::getenv("IDLE_KP")) sim.P.idleKp = std::atof(v);
    if (const char *v = std::getenv("IDLE_KI")) sim.P.idleKi = std::atof(v);
    if (const char *v = std::getenv("IDLE_SG")) sim.P.idleSparkGain = std::atof(v);
    // calibration hooks
    auto env = [](const char *name, double &v) { if (const char *s = std::getenv(name)) v = std::atof(s); };
    env("VEL_BASE", sim.P.veLowBase); env("VEL_PEAK", sim.P.veLowPeak); env("VEL_RPM", sim.P.veLowRpm); env("VEL_WIDTH", sim.P.veLowWidth);
    env("VE_BASE", sim.P.veBase); env("VE_PEAK", sim.P.vePeak); env("VE_RPM", sim.P.veRpm); env("VE_WIDTH", sim.P.veWidth);
    env("FMEP_A", sim.P.fmepA); env("FMEP_B", sim.P.fmepB); env("FMEP_C", sim.P.fmepC);
    env("RESID", sim.P.residualMin); env("ETA", sim.P.etaHeat); env("STARTER", sim.P.starterTorque);
    env("SC_PR", sim.P.turbo.prRef); env("SC_FLOW", sim.P.turbo.flowPeakRef); env("SC_CHOKE", sim.P.turbo.chokeFall); env("SC_RELIEF", sim.P.turbo.reliefP); env("INERTIA", sim.P.inertia);  // supercharger
    env("IDLE_BASE", sim.P.idleBase); env("IDLE_SPARK", sim.P.idleSparkBase); env("IDLE_MIN", sim.P.idleMin);
    auto run = [&](double seconds, auto &&each) {
        int n = static_cast<int>(seconds / PHYS_DT);
        for (int k = 0; k < n; k++) {
            sim.step(PHYS_DT);
            each();
        }
    };
    auto none = [] {};
    bool ok = true;
    auto check = [&](const char *what, double v, double lo, double hi, const char *unit) {
        bool pass = v >= lo && v <= hi;
        ok &= pass;
        std::printf("  %-42s %9.1f %-5s [%g .. %g] %s\n", what, v, unit, lo, hi, pass ? "ok" : "FAIL");
    };
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    std::printf("%s physics test (dt = 1/%.0f s), displacement %.0f cc\n", S.name, 1.0 / PHYS_DT, sim.vDispTotal * 1e6);
    check("displacement", sim.vDispTotal * 1e6, S.dispLo, S.dispHi, "cc");

    // 1. cranking with the ignition off
    sim.ignition = false;
    sim.starter = true;
    run(1.0, none);
    double cMin = 1e9, cMax = -1e9, cSum = 0.0;
    int cN = 0;
    run(1.0, [&] { cMin = std::min(cMin, sim.rpm()); cMax = std::max(cMax, sim.rpm()); cSum += sim.rpm(); cN++; });
    check("cranking speed, mean", cSum / cN, 200, 500, "rpm");
    check("cranking ripple (max - min)", cMax - cMin, 30, 600, "rpm");

    // 2. start
    sim.ignition = true;
    double tStart = sim.time;
    while (sim.rpm() < 800.0 && sim.time - tStart < 3.0) run(0.001, none);
    check("time to catch with the starter", sim.time - tStart, 0.0, 1.0, "s");
    sim.starter = false;
    double flare = 0.0;
    run(1.5, [&] { flare = std::max(flare, sim.rpm()); });
    check("start flare (peak rpm)", flare, 1300, 3500, "rpm");

    // 3. idle
    if (std::getenv("PHYS_TRACE")) {
        for (int k = 0; k < 60; k++) {
            run(0.05, none);
            std::printf("    t %.2f  rpm %7.1f  pMan %5.1f kPa  idleArea %.2e  brake %6.1f\n", sim.time, sim.rpm(),
                        sim.pMan / 1000.0, sim.idleArea, sim.brakeAvg);
        }
    } else {
        run(3.0, none);
    }
    double iMin = 1e9, iMax = -1e9, iSum = 0.0;
    int iN = 0;
    run(2.0, [&] { iMin = std::min(iMin, sim.rpm()); iMax = std::max(iMax, sim.rpm()); iSum += sim.rpm(); iN++; });
    check("idle speed, mean", iSum / iN, 1150, 1350, "rpm");
    check("idle ripple (max - min)", iMax - iMin, 5, 300, "rpm");
    check("idle manifold pressure", sim.pMan / 1000.0, 25, 60, "kPa");

    // 4. full throttle in neutral: a light crank flies to the limiter
    sim.pedalTarget = 1.0;
    double t0 = sim.time;
    while (sim.rpm() < sim.P.revLimit && sim.time - t0 < 5.0) run(0.0005, none);
    check("idle -> rev limiter, full throttle in neutral", sim.time - t0, 0.25, 1.5, "s");
    double limMax = 0.0;
    run(1.0, [&] { limMax = std::max(limMax, sim.rpm()); });
    check("peak rpm on the limiter", limMax, S.limLo, S.limHi, "rpm");

    // 5. snap shut: back to idle
    sim.pedalTarget = 0.0;
    run(4.0, none);
    check("rpm 4 s after closing the throttle", sim.rpm(), 1100, 1500, "rpm");

    // 6. ignition off from 8000 rpm
    sim.omega = 8000.0 / 60.0 * TWO_PI;
    sim.ignition = false;
    t0 = sim.time;
    while (sim.rpm() > 30.0 && sim.time - t0 < 20.0) run(0.001, none);
    check("coast down 8000 rpm -> stop, ignition off", sim.time - t0, 0.5, 6.0, "s");

    // 7. full-throttle dyno sweep. Boosted charge: measured as hot as it gets in a long pull, like the
    // D chart (src/dyno.hpp): the charge cooler's heat soak (icTau, seconds) would otherwise leave the
    // charge at the intake temperature for the short settle at each point, and the dyno's fan gives
    // the cooler moving-air cooling. Restored after the sweep. Without a turbo tCharge stays at tIntake.
    const double icTauWas = sim.P.turbo.icTau, icEffStillWas = sim.P.turbo.icEffStill;
    sim.P.turbo.icTau = 0.02;
    sim.P.turbo.icEffStill = sim.P.turbo.icEffMoving;
    std::printf("\n  WOT dyno sweep (speed locked)\n     rpm   torque Nm   power kW   power hp   BMEP bar   manifold kPa   peak cyl bar   charge C\n");
    double peakT = 0.0, peakTRpm = 0.0, peakP = 0.0, peakPRpm = 0.0;
    for (double rr = 2000.0; rr <= S.sweepTo; rr += rr < 10000.0 ? 1000.0 : 500.0) {
        sim.reset(0.0);
        sim.P.revLimit = 1e9;
        sim.ignition = true;
        sim.lockSpeed = true;
        sim.pedal = sim.pedalTarget = 1.0;
        sim.omega = rr / 60.0 * TWO_PI;
        run(0.3, none);
        double cycleTime = 2.0 * TWO_PI / sim.omega;
        double sum = 0.0, pk = 0.0;
        int n = 0;
        run(6.0 * cycleTime, [&] {
            sum += sim.gasTorque - sim.frictionTorque;
            n++;
            for (auto &c : sim.cyl) pk = std::max(pk, c.p);
        });
        double t = sum / n, kw = t * sim.omega / 1000.0;
        double bmep = t * 2.0 * TWO_PI / sim.vDispTotal / 1e5;
        std::printf("    %5.0f   %8.1f   %8.1f   %8.1f   %8.2f   %12.1f   %12.1f   %8.0f\n", rr, t, kw, kw * 1.341, bmep, sim.pMan / 1000.0, pk / 1e5,
                    sim.tCharge - 273.15);
        if (t > peakT) { peakT = t; peakTRpm = rr; }
        if (kw > peakP) { peakP = kw; peakPRpm = rr; }
    }
    sim.P.turbo.icTau = icTauWas;
    sim.P.turbo.icEffStill = icEffStillWas;
    std::printf("\n");
    check(S.torqueWhat, peakT, S.torqueLo, S.torqueHi, "N m");
    check(S.torqueRpmWhat, peakTRpm, S.torqueRpmLo, S.torqueRpmHi, "rpm");
    check(S.powerWhat, peakP, S.powerLo, S.powerHi, "kW");
    check(S.powerRpmWhat, peakPRpm, S.powerRpmLo, S.powerRpmHi, "rpm");

    // 8. shaking at a locked 10000 rpm: counterweights cancel the V pairs' 1st order
    {
        sim.reset(0.0);
        sim.lockSpeed = true;
        sim.ignition = false;
        sim.omega = 10000.0 / 60.0 * TWO_PI;
        run(1.5, none);
        double unbalanced = 2.0 * sim.P.mRecip * sim.cyl[0].r * sim.omega * sim.omega;  // two pairs, no counterweight
        std::printf("  shaking at 10000 rpm: 1st %.0f N (%.0f N without counterweights), 2nd %.0f N\n",
                    sim.shake1(), unbalanced, sim.shake2());
        check("1st-order vertical force / unbalanced", sim.shake1() / unbalanced, 0.0, 0.05, "");
    }

    // 9. firing intervals: the Twin Pulse 90-200-90-340 pattern, measured from the spark events
    {
        sim.reset(0.0);
        sim.lockSpeed = true;
        sim.ignition = true;
        sim.P.revLimit = 1e9;
        sim.pedal = sim.pedalTarget = 0.5;
        sim.omega = 3000.0 / 60.0 * TWO_PI;
        run(0.2, none);
        std::vector<double> fires;
        std::array<bool, 4> was{};
        for (int c = 0; c < 4; c++) was[c] = sim.cyl[c].sparked;
        double crank = 0.0;
        run(0.2, [&] {
            crank += sim.omega * PHYS_DT;
            for (int c = 0; c < 4; c++) {
                if (sim.cyl[c].sparked && !was[c]) fires.push_back(crank / DEGR);
                was[c] = sim.cyl[c].sparked;
            }
        });
        std::printf("  firing intervals (deg):");
        double worst = 0.0;
        const std::array<double, 4> &PATTERN = S.pattern;
        int start = -1;
        for (std::size_t k = 1; k < fires.size(); k++) {
            double d = fires[k] - fires[k - 1];
            if (k < 9) std::printf(" %.0f", d);
            if (start < 0 && std::fabs(d - PATTERN[0]) < 2.0 && k + 1 < fires.size() && std::fabs(fires[k + 1] - fires[k] - PATTERN[1]) < 2.0) start = static_cast<int>(k);
            if (start >= 0) worst = std::max(worst, std::fabs(d - PATTERN[(k - start) % 4]));
        }
        std::printf("\n");
        check(S.patternWhat, start < 0 ? 999.0 : worst, 0.0, 1.0, "deg");
    }

    // 10. on the bike: start, idle in gear, full-throttle launch in automatic
    auto startBike = [&](bool automatic) {
        sim.lockSpeed = false;
        sim.standalone = false;
        sim.P.revLimit = REV_LIMIT;
        sim.reset(e.cyls[0].theta0);
        sim.drive.reset(automatic);
        sim.drive.grade = sim.drive.cargo = 0.0;
        sim.pedalTarget = 0.0;
        sim.ignition = true;
        sim.starter = true;
        run(0.5, none);
        sim.starter = false;
        run(2.0, none);
    };
    std::printf("\n  bike: %s\n", S.bikeWhat);
    startBike(true);
    check("idling in 1st (automatic), rpm", sim.rpm(), 1100, 1450, "rpm");
    check("  ... not creeping", sim.drive.v * 3.6, 0.0, 1.0, "km/h");
    sim.pedalTarget = 1.0;
    t0 = sim.time;
    double t100 = -1, t200 = -1, firstShiftRpm = -1, minTrim = 1.0, peakAccel = 0.0;
    int lastGear = sim.drive.gear;
    run(30.0, [&] {
        double kmh = sim.drive.v * 3.6;
        if (t100 < 0 && kmh >= 100.0) t100 = sim.time - t0;
        if (t200 < 0 && kmh >= 200.0) t200 = sim.time - t0;
        if (firstShiftRpm < 0 && sim.drive.shiftPhase == 1 && lastGear == 1) firstShiftRpm = sim.rpm();
        lastGear = sim.drive.gear;
        minTrim = std::min(minTrim, sim.tcTrim);
        peakAccel = std::max(peakAccel, sim.drive.accel);
        static int k = 0;
        if (std::getenv("PHYS_TRACE") && sim.time - t0 < 4.0 && ++k % 2000 == 0)
            std::printf("    t %.2f  %5.1f km/h  rpm %6.0f  gear %d  clutch %.2f  slip %6.0f  wheel %5.0f N  trim %.2f\n",
                        sim.time - t0, sim.drive.v * 3.6, sim.rpm(), sim.drive.gear, sim.drive.clutch,
                        (sim.omega - sim.drive.omegaIn) * 9.549, sim.drive.wheelForce, sim.tcTrim);
    });
    check("0-100 km/h, full throttle (tests ~3.0-3.4 s)", t100, 2.6, 3.7, "s");
    check("0-200 km/h (road tests ~5.5-7 s)", t200, 5.0, 8.0, "s");
    check("rpm at the 1 -> 2 upshift", firstShiftRpm, S.upshiftLo, S.upshiftHi, "rpm");
    check("wheelie control cut in (lowest fuel share)", minTrim, 0.3, 0.95, "");
    check("peak smoothed acceleration", peakAccel / 9.81, 0.8, 1.15, "g");
    check(S.topWhat, sim.drive.v * 3.6, S.topLo, S.topHi, "km/h");
    check("  ... in gear", sim.drive.gear, 6, 6, "");

    // 11. manual: clutch in, 1st, gentle take-off, quickshift up and down
    startBike(false);
    sim.drive.clutchPedal = true;
    run(0.3, none);
    sim.drive.requestShift(-1, sim.time);  // N -> 1st
    check("manual: N -> 1st with the clutch in", sim.drive.gear, 1, 1, "");
    sim.pedalTarget = 0.25;
    sim.drive.clutchPedal = false;  // lever released at its slow rate
    double minRpm = 1e9;
    run(2.5, [&] { minRpm = std::min(minRpm, sim.rpm()); });
    check("manual take-off: lowest rpm (no stall)", minRpm, S.takeoffMinRpm, 5000, "rpm");
    sim.pedalTarget = 0.7;
    while (sim.rpm() < 11000.0 && sim.time < 60.0) run(0.001, none);
    double vBefore = sim.drive.v, rpmBefore = sim.rpm();
    sim.drive.requestShift(+1, sim.time);
    bool cut = false;
    run(0.2, [&] { cut |= sim.qsCut; });
    check("quickshift 1 -> 2 without the clutch", sim.drive.gear, 2, 2, "");
    check("  ... ignition cut during the shift", cut ? 1.0 : 0.0, 1, 1, "");
    double want = rpmBefore * sim.drive.P.ratios[2] / sim.drive.P.ratios[1] * sim.drive.v / vBefore;
    check("  ... rpm after / ratio step", sim.rpm() / want, 0.93, 1.07, "");
    sim.pedalTarget = 0.0;
    run(0.5, none);
    // quickshift down, rev-matched: blip, the old gear drops out, the engine is revved to the lower
    // gear's speed and the new gear goes in when they agree. Measured: how close the engine is to that
    // speed at the moment the gear engages, how long the shift takes, and the slip right after it
    auto downshift = [&](int to) {
        double t0 = sim.time, blipMax = 0.0, engRpm = -1.0, engMatch = 1.0, engT = -1.0, slipAfter = 0.0;
        bool wentFree = false;
        sim.drive.requestShift(-1, sim.time);
        run(0.5, [&] {
            blipMax = std::max(blipMax, sim.blip);
            wentFree |= sim.drive.dogsFree;
            static int k = 0;
            if (std::getenv("PHYS_TRACE") && sim.time - t0 < 0.3 && ++k % 200 == 0)
                std::printf("    qs t %5.1f ms  rpm %6.0f  match %6.0f  in %6.0f  gear %d  free %d  blip %.2f  thr %.2f  pMan %3.0f  Tcl %6.1f  cut %d %d\n",
                            (sim.time - t0) * 1000.0, sim.rpm(), sim.drive.matchRpm, sim.drive.omegaIn * 60.0 / TWO_PI, sim.drive.gear,
                            sim.drive.dogsFree, sim.blip, sim.throttleOpen, sim.pMan / 1000.0, sim.drive.clutchTorque, sim.revCut, sim.fuelCut);
            if (engT < 0.0 && sim.drive.gear == to) { engT = sim.time - t0; engRpm = sim.rpm(); engMatch = sim.drive.matchRpm; }
            if (engT >= 0.0 && sim.time - t0 < engT + 0.05)
                slipAfter = std::max(slipAfter, std::fabs(sim.omega - sim.drive.omegaIn) * 60.0 / TWO_PI);
        });
        std::printf("  quickshift %d -> %d: engaged after %.0f ms at %.0f rpm (wanted %.0f)\n", to + 1, to, engT * 1000.0, engRpm, engMatch);
        check("  ... gear went in", sim.drive.gear, to, to, "");
        check("  ... throttle blip opened", blipMax, 0.05, 1.0, "");
        check("  ... old gear dropped out first (dogs unloaded)", wentFree ? 1.0 : 0.0, 1, 1, "");
        check("  ... engine speed at engagement vs the gear's", engRpm / engMatch, 0.96, 1.04, "");
        check("  ... shift time", engT * 1000.0, 10, 200, "ms");
        check("  ... clutch slip just after (no lurch)", slipAfter, 0, 400, "rpm");
    };
    std::printf("  quickshift down from 2nd on the overrun:\n");
    downshift(1);
    // and further up the box: 4th -> 3rd at speed, throttle shut
    sim.pedalTarget = 0.8;
    for (int k = 0; k < 2; k++) {
        while (sim.rpm() < 11000.0 && sim.time < 200.0) run(0.001, none);
        sim.drive.requestShift(+1, sim.time);
        run(0.3, none);
    }
    sim.pedalTarget = 0.0;
    run(0.6, none);
    std::printf("  quickshift down from %dth at %.0f km/h:\n", sim.drive.gear, sim.drive.v * 3.6);
    downshift(sim.drive.gear - 1);

    // 12. dumping the clutch in 1st at idle stalls a light-crank V4
    startBike(false);
    sim.drive.clutchPedal = true;
    run(0.3, none);
    sim.drive.requestShift(-1, sim.time);
    sim.drive.clutch = 1.0;  // lever let go at once
    sim.drive.clutchPedal = false;
    run(1.0, none);
    check("clutch dump at idle in 1st: rpm after 1 s", std::fabs(sim.rpm()), 0, 150, "rpm");

    // 13. neutral: in automatic at a stop (revs freely, no creep), at speed (coasts), and back into gear
    startBike(true);
    sim.drive.bikeShift(0, sim.time, true);
    check("automatic, stopped: N selected", sim.drive.gear, 0, 0, "");
    sim.pedalTarget = 0.4;
    double nPeak = 0.0;
    run(1.0, [&] { nPeak = std::max(nPeak, sim.rpm()); });
    check("  ... revving in N: peak rpm", nPeak, 8000, S.limHi, "rpm");
    check("  ... bike stays put", sim.drive.v * 3.6, 0.0, 0.5, "km/h");
    sim.pedalTarget = 0.0;
    run(2.0, none);
    sim.drive.bikeShift(+1, sim.time, false);  // E in automatic from N: drive
    check("automatic: E from N selects drive (1st)", sim.drive.gear, 1, 1, "");
    sim.pedalTarget = 0.6;
    {
        int k = 0;
        run(6.0, [&] {
            if (std::getenv("PHYS_TRACE") && ++k % 8000 == 0)
                std::printf("    t %.2f  %5.1f km/h  rpm %6.0f  gear %d  clutch %.2f  stalled %d  cut %d\n", sim.time, sim.drive.v * 3.6,
                            sim.rpm(), sim.drive.gear, sim.drive.clutch, sim.stalled, sim.qsCut);
        });
    }
    int gearBefore = sim.drive.gear;
    double vN = sim.drive.v;
    sim.pedalTarget = 0.0;
    sim.drive.bikeShift(0, sim.time, true);
    {
        int k = 0;
        run(S.neutralSettle, [&] {
            if (std::getenv("PHYS_TRACE") && ++k % 4000 == 0)
                std::printf("    N t %.2f  rpm %6.0f  gear %d  clutch %.2f  pedal %.2f  blip %.2f  idle %.1e  pMan %.0f\n", sim.time,
                            sim.rpm(), sim.drive.gear, sim.drive.clutch, sim.pedal, sim.blip, sim.idleArea, sim.pMan / 1000.0);
        });
    }
    check("automatic at speed: N from a higher gear", gearBefore >= 3 && sim.drive.gear == 0 ? 1.0 : 0.0, 1, 1, "");
    check("  ... coasting in N (speed kept)", sim.drive.v / vN, 0.85, 1.0, "");
    check("  ... engine back near idle", sim.rpm(), 1000, 2200, "rpm");
    sim.drive.bikeShift(0, sim.time, true);  // N again: back into a gear for this speed
    int back = sim.drive.gear;
    run(2.0, none);
    check("  ... N again picks a gear for the speed", back, 3, 6, "");
    check("  ... no stall rejoining", sim.rpm(), 3000, 14000, "rpm");

    // manual: N from 4th at speed
    startBike(false);
    sim.drive.clutchPedal = true;
    run(0.3, none);
    sim.drive.requestShift(-1, sim.time);
    sim.drive.clutchPedal = false;
    sim.pedalTarget = 0.5;
    run(2.5, none);
    for (int k = 0; k < 3; k++) {
        while (sim.rpm() < 10000.0 && sim.time < 200.0) run(0.001, none);
        sim.drive.requestShift(+1, sim.time);
        run(0.3, none);
    }
    int g4 = sim.drive.gear;
    sim.drive.bikeShift(0, sim.time, true);
    check("manual: N straight from 4th", g4 == 4 && sim.drive.gear == 0 ? 1.0 : 0.0, 1, 1, "");

    // 14. wheelie control off: the launch is no longer held to 1.05 g
    startBike(true);
    sim.drive.wheelieControl = false;
    sim.pedalTarget = 1.0;
    t0 = sim.time;
    double t100off = -1, accelOff = 0.0, trimOff = 1.0;
    int kk = 0;
    run(6.0, [&] {
        if (std::getenv("PHYS_TRACE") && ++kk % 8000 == 0)
            std::printf("    off t %.2f  %5.1f km/h  rpm %6.0f  gear %d  clutch %.2f  cut %d  acc %.2f\n", sim.time - t0, sim.drive.v * 3.6,
                        sim.rpm(), sim.drive.gear, sim.drive.clutch, sim.qsCut, sim.drive.accel / 9.81);
        if (t100off < 0 && sim.drive.v * 3.6 >= 100.0) t100off = sim.time - t0;
        accelOff = std::max(accelOff, sim.drive.accel);
        trimOff = std::min(trimOff, sim.tcTrim);
    });
    sim.drive.wheelieControl = true;
    check("wheelie control OFF: no fuel trim", trimOff, 1.0, 1.0, "");
    check("  ... peak smoothed acceleration", accelOff / 9.81, 1.15, 3.0, "g");
    check("  ... 0-100 km/h (front would be in the air)", t100off, 1.5, t100 - 0.2, "s");
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

