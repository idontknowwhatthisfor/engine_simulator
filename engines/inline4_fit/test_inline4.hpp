// --engine-type inline4 --tuned yes --phystest: the turbocharged inline-4 (its tuned version). (The plain
// engine and car run the shared car test, src/test_car.hpp, against inline4FitCar)
#pragma once
#include "../../src/engines.hpp"

// --engine-type inline4 --tuned yes --phystest: the same engine with the turbocharger. Start and idle
// (no boost), the WOT dyno sweep with the turbo settled at each speed, spool-up from part load, the
// wastegate holding the target, back to idle after a lift-off, and 0-100 km/h in the car.
int physicsTestInline4Turbo(const engineModel &e, const engineType &type) {
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0);
    applyCalibrationEnv(sim.P);
    sim.drive.reset(false);
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
        std::printf("  %-42s %9.2f %-5s [%g .. %g] %s\n", what, v, unit, lo, hi, pass ? "ok" : "FAIL");
    };
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    const double TARGET = (sim.P.turbo.boostMap - sim.P.pAtm) / 1e5;  // bar
    std::printf("turbo physics test (dt = 1/%.0f s), boost target %.2f bar\n", 1.0 / PHYS_DT, TARGET);

    // 1. start and idle: the turbo barely turns, no boost
    sim.starter = true;
    double t0 = sim.time;
    while (sim.rpm() < 550.0 && sim.time - t0 < 3.0) run(0.001, none);
    check("time to catch with the starter", sim.time - t0, 0.0, 1.5, "s");
    sim.starter = false;
    run(5.0, none);
    double iSum = 0.0;
    int iN = 0;
    run(2.0, [&] { iSum += sim.rpm(); iN++; });
    check("idle speed, mean", iSum / iN, 750, 850, "rpm");
    check("idle: turbo shaft speed", sim.turboRpm() / 1000.0, 0, 40, "krpm");
    check("idle: boost", sim.boostBar(), -1.0, 0.0, "bar");

    // 2. WOT dyno sweep, speed locked, turbo settled at each speed
    std::printf("\n  WOT dyno sweep (speed locked, 3 s to spool)\n"
                "     rpm   torque Nm   power kW   boost bar   turbo krpm   wastegate   back kPa   peak cyl bar"
                "   charge kPa   comp kg/s   charge C\n");
    double peakT = 0.0, peakTRpm = 0.0, peakP = 0.0, peakPRpm = 0.0, boostAt[15] = {};
    double maxCyl = 0.0, maxTurbo = 0.0, maxSurge = 0.0;
    for (int k = 0; k <= 12; k++) {
        double rr = 1000.0 + 500.0 * k;
        sim.reset(0.0);
        sim.P.revLimit = 1e9;
        sim.lockSpeed = true;
        sim.pedal = sim.pedalTarget = 1.0;
        sim.omega = rr / 60.0 * TWO_PI;
        run(2.0, none);
        run(1.0, [&] { maxSurge = std::max(maxSurge, sim.surge); });  // settled: the compressor must not surge
        double cycleTime = 2.0 * TWO_PI / sim.omega, sum = 0.0, pk = 0.0;
        int n = 0;
        run(4.0 * cycleTime, [&] {
            sum += sim.gasTorque - sim.frictionTorque;
            n++;
            for (auto &s : sim.cyl) pk = std::max(pk, s.p);
        });
        double t = sum / n, kw = t * sim.omega / 1000.0;
        boostAt[k] = sim.boostBar();
        maxCyl = std::max(maxCyl, pk);
        maxTurbo = std::max(maxTurbo, sim.turboRpm());
        std::printf("    %5.0f   %8.1f   %8.1f   %9.2f   %10.0f   %8.0f%%   %8.1f   %12.1f   %10.1f   %9.3f   %8.0f\n", rr, t, kw,
                    sim.boostBar(), sim.turboRpm() / 1000.0, sim.wastegate * 100.0, sim.pExh / 1000.0, pk / 1e5,
                    sim.pCharge / 1000.0, sim.compFlow, sim.tCharge - 273.15);
        if (t > peakT) { peakT = t; peakTRpm = rr; }
        if (kw > peakP) { peakP = kw; peakPRpm = rr; }
    }
    std::printf("\n");
    check("peak torque", peakT, 320, 400, "N m");
    check("peak torque rpm", peakTRpm, 3000, 4500, "rpm");
    check("peak power", peakP, 180, 230, "kW");
    check("peak power rpm", peakPRpm, 6000, 7000, "rpm");
    check("boost at 1500 rpm (below the spool point)", boostAt[1], -0.1, 0.15, "bar");
    check("boost at 4000 rpm (held by the wastegate)", boostAt[6], TARGET - 0.08, TARGET + 0.05, "bar");
    check("boost at 6500 rpm", boostAt[11], TARGET - 0.08, TARGET + 0.05, "bar");
    check("highest turbo shaft speed", maxTurbo / 1000.0, 80, 170, "krpm");
    check("full throttle: no compressor surge (dyno)", maxSurge, 0.0, 0.05, "");
    check("highest cylinder pressure", maxCyl / 1e5, 80, 140, "bar");

    // 3. spool-up: cruising at 3000 rpm, light pedal, then floor it
    sim.reset(0.0);
    sim.lockSpeed = true;
    sim.omega = 3000.0 / 60.0 * TWO_PI;
    sim.pedal = sim.pedalTarget = 0.25;
    run(3.0, none);
    check("3000 rpm, 25% pedal: boost (vacuum)", sim.boostBar(), -0.8, 0.0, "bar");
    sim.pedalTarget = 1.0;
    t0 = sim.time;
    double tHalf = -1.0, t90 = -1.0, over = -1.0;
    while (sim.time - t0 < 6.0) {
        run(0.001, none);
        double b = sim.boostBar();
        over = std::max(over, b);
        if (tHalf < 0.0 && b >= 0.5 * TARGET) tHalf = sim.time - t0;
        if (t90 < 0.0 && b >= 0.9 * TARGET) t90 = sim.time - t0;
    }
    check("3000 rpm floored: time to half boost", tHalf, 0.3, 1.5, "s");
    check("3000 rpm floored: time to 90% boost", t90, 0.8, 3.5, "s");
    check("boost overshoot over the target", over - TARGET, -0.05, 0.12, "bar");
    check("boost after 6 s (wastegate holds it)", sim.boostBar(), TARGET - 0.08, TARGET + 0.05, "bar");
    // intercooler, standing still (half effective): the compressor heats the air, the core takes
    // half of that back out
    check("compressor outlet temperature", sim.tComp - 273.15, 70, 120, "C");
    check("charge temperature after the intercooler", sim.tCharge - 273.15, 35, 75, "C");
    check("pressure drop across the intercooler", (sim.pCharge - sim.pBoost) / 1000.0, 0.5, 12, "kPa");

    // 4. lift off at 3000 rpm: boost and shaft fall away
    sim.pedalTarget = 0.0;
    run(2.0, none);
    check("2 s after lifting off: boost (vacuum)", sim.boostBar(), -1.0, -0.3, "bar");
    check("2 s after lifting off: turbo shaft", sim.turboRpm() / 1000.0, 10, 80, "krpm");

    // 5. throttle snapped shut on full boost at 4000 rpm, with the blow-off valve and without it.
    // With it the valve vents the charge and the compressor keeps flowing; without it the trapped
    // charge pushes back through the compressor and it surges (flow reverses in bursts)
    auto snap = [&](bool bov, double &surgeMax, int &bursts, double &bovPeak, double &shaftAfter, double &chargePeak) {
        sim.reset(0.0);
        sim.P.turbo.bov = bov;
        sim.lockSpeed = true;
        sim.omega = 4000.0 / 60.0 * TWO_PI;
        sim.pedal = sim.pedalTarget = 1.0;
        run(3.0, none);
        sim.pedalTarget = 0.0;
        surgeMax = bovPeak = chargePeak = 0.0;
        bursts = 0;
        bool back = false;
        std::FILE *trace = std::getenv("SURGE_TRACE") ? std::fopen(bov ? "surge_bov.csv" : "surge_nobov.csv", "w") : nullptr;
        int k = 0;
        run(1.0, [&] {
            surgeMax = std::max(surgeMax, sim.surge);
            bovPeak = std::max(bovPeak, sim.bovFlow);
            chargePeak = std::max(chargePeak, sim.pCharge);
            if (!back && sim.compFlow < -0.002) bursts++;
            back = sim.compFlow < -0.002 ? true : sim.compFlow > 0.0 ? false : back;
            if (trace && k++ % 20 == 0)
                std::fprintf(trace, "%.4f,%.4f,%.1f,%.1f,%.0f,%.3f\n", sim.time, sim.compFlow, sim.pCharge / 1000.0, sim.pMan / 1000.0,
                             sim.turboRpm(), sim.bovOpen);
        });
        if (trace) std::fclose(trace);
        shaftAfter = sim.turboRpm();
        sim.P.turbo.bov = true;
    };
    {
        double sB, bB, cB, sN, bN, cN, hB, hN;
        int nB, nN;
        snap(true, sB, nB, bB, hB, cB);
        snap(false, sN, nN, bN, hN, cN);
        std::printf("  lift-off on boost at 4000 rpm      surge   backflow bursts   BOV peak kg/s   charge peak kPa   shaft after 1 s\n");
        std::printf("    with the blow-off valve        %6.2f   %15d   %13.3f   %15.1f   %11.0f krpm\n", sB, nB, bB, cB / 1000.0, hB / 1000.0);
        std::printf("    without it                     %6.2f   %15d   %13.3f   %15.1f   %11.0f krpm\n", sN, nN, bN, cN / 1000.0, hN / 1000.0);
        check("BOV: vents the charge (peak flow)", bB, 0.02, 0.2, "kg/s");
        check("BOV: no surge", sB, 0.0, 0.1, "");
        check("no BOV: surges", sN, 0.5, 1.0, "");
        check("no BOV: backflow bursts in 1 s (flutter)", nN, 5, 60, "");
    }

    // 6. the car: 0-100 km/h in the automatic
    sim.lockSpeed = false;
    sim.P.revLimit = 6800.0;
    sim.reset(e.cyls[0].theta0);
    sim.drive.reset(true);
    sim.starter = true;
    run(0.6, none);
    sim.starter = false;
    run(2.5, none);
    sim.pedalTarget = 1.0;
    t0 = sim.time;
    double t100 = -1.0, slipMax = 0.0, lastShift = sim.time;
    int lastGear = sim.drive.gear;
    while (sim.time - t0 < 20.0 && t100 < 0.0) {
        run(0.001, none);
        if (sim.drive.gear != lastGear || sim.drive.shiftPhase != 0) {
            lastGear = sim.drive.gear;
            lastShift = sim.time;
        }
        // slip under full boost in a gear, away from the shifts (the clutch closing after a shift slips on purpose)
        if (sim.drive.clutch > 0.99 && sim.drive.coupled() && sim.time - lastShift > 0.5 && sim.drive.v > 5.0)
            slipMax = std::max(slipMax, std::fabs(sim.rpm() - sim.drive.omegaIn * 60.0 / TWO_PI));
        if (sim.drive.v * 3.6 >= 100.0) t100 = sim.time - t0;
    }
    check("auto: 0-100 km/h at full throttle (6.8 s without)", t100, 4.0, 7.0, "s");
    check("clutch slip away from the shifts", slipMax, 0, 150, "rpm");
    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
