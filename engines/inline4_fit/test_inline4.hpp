// --engine-type inline4 --phystest: start, idle, rev, coast-down, WOT torque curve, vibration, and the
// Honda Fit drivetrain (automatic and manual shifting, hills, cargo, reverse)
#pragma once
#include "../../src/engines.hpp"

// headless physics checks (--phystest): cranking, start, idle, rev-up, coast-down, WOT torque curve
int physicsTestInline4(const engineModel &e, const engineType &type) {
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0);
    sim.drive.reset(false);  // engine tests run in neutral with the clutch out
    applyCalibrationEnv(sim.P);  // VE_*, FMEP_*, IDLE_*... (src/calibration.hpp)
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
    std::printf("physics test (dt = 1/%.0f s)\n", 1.0 / PHYS_DT);

    // 1. cranking with the ignition off
    sim.ignition = false;
    sim.starter = true;
    run(1.0, none);
    double cMin = 1e9, cMax = -1e9, cSum = 0.0;
    int cN = 0;
    run(1.0, [&] { cMin = std::min(cMin, sim.rpm()); cMax = std::max(cMax, sim.rpm()); cSum += sim.rpm(); cN++; });
    check("cranking speed, mean", cSum / cN, 150, 350, "rpm");
    check("cranking ripple (max - min)", cMax - cMin, 30, 400, "rpm");

    // 2. start: ignition on while cranking, release the starter once it runs
    sim.ignition = true;
    double tStart = sim.time;
    while (sim.rpm() < 550.0 && sim.time - tStart < 3.0) run(0.001, none);
    double catchTime = sim.time - tStart;
    sim.starter = false;
    check("time to catch with the starter", catchTime, 0.0, 1.5, "s");
    double flare = 0.0;
    run(1.5, [&] { flare = std::max(flare, sim.rpm()); });
    check("start flare (peak rpm)", flare, 850, 2000, "rpm");

    // 3. idle (PHYS_TRACE=1 prints the controller every 50 ms)
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
    check("idle speed, mean", iSum / iN, 750, 850, "rpm");
    check("idle ripple (max - min)", iMax - iMin, 5, 150, "rpm");
    check("idle manifold pressure", sim.pMan / 1000.0, 20, 50, "kPa");

    // 4. full throttle, no load: time to the rev limiter
    sim.pedalTarget = 1.0;
    double t0 = sim.time;
    while (sim.rpm() < sim.P.revLimit && sim.time - t0 < 5.0) run(0.001, none);
    check("idle -> rev limiter at full throttle", sim.time - t0, 0.5, 2.5, "s");
    double limMax = 0.0;
    run(1.0, [&] { limMax = std::max(limMax, sim.rpm()); });
    check("peak rpm on the limiter", limMax, 6800, 7200, "rpm");

    // 5. lift off: back to idle
    sim.pedalTarget = 0.0;
    run(4.0, none);
    check("rpm 4 s after lifting off", sim.rpm(), 700, 950, "rpm");

    // 6. ignition off from 3000 rpm
    sim.omega = 3000.0 / 60.0 * 2.0 * 3.14159265358979323846;
    sim.ignition = false;
    t0 = sim.time;
    while (sim.rpm() > 30.0 && sim.time - t0 < 20.0) run(0.001, none);
    check("coast down 3000 rpm -> stop, ignition off", sim.time - t0, 0.8, 6.0, "s");

    // 7. full-throttle torque curve on a speed-locked dyno
    std::printf("\n  WOT dyno sweep (speed locked)\n     rpm   torque Nm   power kW   BMEP bar   manifold kPa   peak cyl bar\n");
    double peakT = 0.0, peakTRpm = 0.0, peakP = 0.0, peakPRpm = 0.0;
    for (double rr = 1000.0; rr <= 7000.0; rr += 500.0) {
        sim.reset(0.0);
        sim.P.revLimit = 1e9;
        sim.ignition = true;
        sim.lockSpeed = true;
        sim.pedal = sim.pedalTarget = 1.0;
        sim.omega = rr / 60.0 * 2.0 * 3.14159265358979323846;
        run(0.4, none);
        double cycleTime = 4.0 * 3.14159265358979323846 / sim.omega;
        double sum = 0.0, pk = 0.0;
        int n = 0;
        run(4.0 * cycleTime, [&] {
            sum += sim.gasTorque - sim.frictionTorque;
            n++;
            for (auto &s : sim.cyl) pk = std::max(pk, s.p);
        });
        double t = sum / n, kw = t * sim.omega / 1000.0;
        double bmep = t * 4.0 * 3.14159265358979323846 / sim.vDispTotal / 1e5;
        std::printf("    %5.0f   %8.1f   %8.1f   %8.2f   %12.1f   %12.1f\n", rr, t, kw, bmep, sim.pMan / 1000.0, pk / 1e5);
        if (t > peakT) { peakT = t; peakTRpm = rr; }
        if (kw > peakP) { peakP = kw; peakPRpm = rr; }
    }
    std::printf("\n");
    check("peak torque", peakT, 180, 230, "N m");

    // 8. shaking forces at a locked 6000 rpm against the textbook slider-crank harmonics
    {
        sim.reset(0.0);
        sim.lockSpeed = true;
        sim.ignition = false;
        sim.omega = 6000.0 / 60.0 * 2.0 * 3.14159265358979323846;
        run(1.5, none);
        double w2 = sim.omega * sim.omega, L = sim.cyl[0].L, r14 = sim.cyl[0].r, r23 = sim.cyl[1].r;
        double want2 = 2.0 * sim.P.mRecip * w2 * (r14 * r14 + r23 * r23) / L;
        double want1 = 2.0 * sim.P.mRecip * w2 * std::fabs(r14 - r23);
        std::printf("\n  shaking at 6000 rpm: 1st %.0f N (expect %.0f, from the uneven throws), 2nd %.0f N (expect %.0f)\n",
                    sim.shake1(), want1, sim.shake2(), want2);
        check("2nd-order force / textbook", sim.shake2() / want2, 0.97, 1.03, "");
        check("1st-order force / textbook", sim.shake1() / want1, 0.95, 1.05, "");
        check("pitching moment (mirror symmetric)", sim.pitchAmp(), 0.0, 5.0, "N m");
    }
    check("peak torque rpm", peakTRpm, 3000, 5500, "rpm");
    check("peak power", peakP, 85, 140, "kW");
    check("peak power rpm", peakPRpm, 5000, 7000, "rpm");
    // 9. drivetrain: a fresh engine idling in an automatic car
    auto startCar = [&](bool automatic) {
        sim.lockSpeed = false;
        sim.P.revLimit = 6800.0;
        sim.reset(e.cyls[0].theta0);
        sim.drive.reset(automatic);
        sim.drive.grade = sim.drive.cargo = 0.0;  // the road and load are not part of reset()
        sim.pedalTarget = 0.0;
        sim.ignition = true;
        sim.starter = true;
        run(0.6, none);
        sim.starter = false;
        run(2.5, none);
    };
    std::printf("\n  drivetrain (1300 kg car, Honda Fit 5 speed, final drive 4.62)\n");
    {
        startCar(true);
        check("auto: idling at a standstill, clutch open", sim.drive.clutch, 0.0, 0.05, "");
        check("auto: car held still at idle", sim.drive.v * 3.6, -0.1, 0.1, "km/h");
        sim.pedalTarget = 1.0;
        double t0 = sim.time, t100 = -1.0;
        std::vector<std::pair<int, double>> shifts;
        int lastGear = sim.drive.gear;
        double lastRpm = sim.rpm();
        while (sim.time - t0 < 20.0 && t100 < 0.0) {
            run(0.001, [&] {
                if (sim.drive.shiftPhase == 1 && sim.drive.targetGear != lastGear) {
                    shifts.push_back({sim.drive.targetGear, lastRpm});
                    lastGear = sim.drive.targetGear;
                }
                if (sim.drive.shiftPhase == 0) lastRpm = sim.rpm();
            });
            if (sim.drive.v * 3.6 >= 100.0) t100 = sim.time - t0;
        }
        std::printf("  WOT shifts:");
        for (auto &sh : shifts) std::printf("  ->%d at %.0f rpm", sh.first, sh.second);
        std::printf("\n");
        check("auto: 0-100 km/h at full throttle", t100, 6.0, 12.0, "s");
        double minShiftRpm = 1e9;
        for (auto &sh : shifts) minShiftRpm = std::min(minShiftRpm, sh.second);
        check("auto: WOT upshift rpm (lowest)", shifts.empty() ? 0.0 : minShiftRpm, 5800, 6800, "rpm");
    }
    {
        startCar(true);
        sim.pedalTarget = 0.2;
        double firstShift = -1.0, lastRpm = 0.0;
        double t0 = sim.time;
        while (sim.time - t0 < 20.0 && firstShift < 0.0) {
            run(0.001, [&] {
                if (sim.drive.shiftPhase == 0) lastRpm = sim.rpm();
                else if (firstShift < 0.0) firstShift = lastRpm;
            });
        }
        check("auto: 20% throttle, first upshift rpm", firstShift, 2300, 3500, "rpm");
    }
    {
        // cruise at 35% pedal on the flat, then a 12% hill: the box should change down for the load
        startCar(true);
        sim.pedalTarget = 0.35;
        run(15.0, none);
        int flatGear = sim.drive.gear;
        double flatKmh = sim.drive.v * 3.6;
        sim.drive.grade = 0.12;
        run(15.0, none);
        std::printf("  35%% pedal: gear %d at %.0f km/h on the flat -> gear %d at %.0f km/h, %.0f rpm on a 12%% hill\n",
                    flatGear, flatKmh, sim.drive.gear, sim.drive.v * 3.6, sim.rpm());
        check("auto: gear after 15 s at 35% pedal", flatGear, 3, 5, "");
        check("auto: gears dropped for the 12% hill", flatGear - sim.drive.gear, 1, 4, "");
        check("auto: engine still running on the hill", sim.rpm(), 1500, 6800, "rpm");
    }
    {
        // normal mode: coasting down from speed with the throttle shut changes down only near 1200 rpm
        startCar(true);
        sim.pedalTarget = 0.5;
        run(15.0, none);
        sim.pedalTarget = 0.0;
        int g0 = sim.drive.gear;
        double downAt = -1.0, lastRpm = sim.rpm();
        while (downAt < 0.0 && sim.drive.v > 2.5) {
            run(0.001, [&] {
                const drivetrain &d = sim.drive;
                if (d.shiftPhase == 0) lastRpm = sim.rpm();
                else if (downAt < 0.0 && d.targetGear < d.gear) downAt = lastRpm;  // lifting off may upshift first
            });
            if (sim.time > 200.0) break;
        }
        std::printf("  normal coast from gear %d: first downshift at %.0f rpm\n", g0, downAt);
        check("normal: coast-down downshift rpm", downAt, 1050, 1400, "rpm");
    }
    {
        // sport mode: at full throttle shifts up near the limiter; braking changes down as soon as
        // the lower gear fits, and each downshift is rev matched (engine at the new gear's speed)
        startCar(true);
        sim.drive.sport = true;
        sim.pedalTarget = 1.0;
        run(14.0, none);
        int topGear = sim.drive.gear;
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        struct down { int to; double before, match, after; };
        std::vector<down> downs;
        int phase = 0, from = sim.drive.gear;
        double before = 0.0, match = 0.0;
        run(6.0, [&] {
            const drivetrain &d = sim.drive;
            if (phase == 0 && d.shiftPhase == 1 && d.revMatch) { phase = 1; before = sim.rpm(); from = d.gear; }
            if (phase == 1 && d.shiftPhase == 2 && d.clutch > 0.5) { match = d.matchRpm; phase = 2; downs.push_back({d.targetGear, before, match, sim.rpm()}); }
            if (phase == 2 && d.shiftPhase == 0) phase = 0;
        });
        sim.drive.brake = false;
        std::printf("  sport: gear %d after 14 s WOT; braking downshifts:", topGear);
        double worst = 0.0, lowestBefore = 1e9;
        for (auto &d : downs) {
            std::printf("  ->%d %.0f->%.0f (need %.0f)", d.to, d.before, d.after, d.match);
            worst = std::max(worst, std::fabs(d.after - d.match));
            lowestBefore = std::min(lowestBefore, d.before);
        }
        std::printf("\n");
        check("sport: downshifts while braking", static_cast<double>(downs.size()), 2, 5, "");
        check("sport: rev match error at clutch-in (worst)", worst, 0, 400, "rpm");
        check("sport: downshift waits until the lower gear fits", lowestBefore > 0.0 && !downs.empty() ? downs[0].match : 0.0, 3000, 5600, "rpm");
        sim.drive.sport = false;
    }
    for (bool sp : {false, true}) {
        // braking to a stop from a cruise: in 1st by 10 km/h, engine still running, pulls away in 1st
        startCar(true);
        sim.drive.sport = sp;
        sim.pedalTarget = 0.4;
        run(15.0, none);
        int g0 = sim.drive.gear;
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        double gearAt10 = -1.0, minRpm = 1e9;
        while (sim.drive.v > 0.01 && sim.time < 300.0)
            run(0.001, [&] {
                if (gearAt10 < 0.0 && sim.drive.v < 9.0 / 3.6) gearAt10 = sim.drive.gear;
                minRpm = std::min(minRpm, sim.rpm());
            });
        run(1.0, none);
        int stoppedGear = sim.drive.gear;
        sim.drive.brake = false;
        sim.pedalTarget = 0.3;
        run(3.0, none);
        std::printf("  %s: braked from gear %d, gear %d at 9 km/h, gear %d stopped, lowest %.0f rpm, %.0f km/h after pulling away\n",
                    sp ? "sport" : "normal", g0, int(gearAt10), stoppedGear, minRpm, sim.drive.v * 3.6);
        check(sp ? "sport: in 1st below 10 km/h" : "normal: in 1st below 10 km/h", gearAt10, 1, 1, "");
        check(sp ? "sport: no stall braking to the stop" : "normal: no stall braking to the stop", minRpm, 500, 7000, "rpm");
        check(sp ? "sport: pulls away after the stop" : "normal: pulls away after the stop", sim.drive.v * 3.6, 5, 60, "km/h");
        sim.drive.sport = false;
    }
    {
        // manual: dumping the clutch in 1st at idle stalls; a gentle release with some throttle does not
        startCar(false);
        sim.drive.clutchPedal = true;
        run(0.3, none);
        sim.drive.requestShift(+1, sim.time);
        sim.drive.P.clutchReleaseRate = 20.0;  // drop it
        sim.drive.clutchPedal = false;
        double dumpMin = 1e9;
        run(1.5, [&] { dumpMin = std::min(dumpMin, sim.rpm()); });
        std::printf("  clutch dump at idle in 1st: lowest %.0f rpm, %.0f rpm after 1.5 s\n", dumpMin, sim.rpm());
        check("manual: clutch dumped at idle in 1st -> stall", sim.rpm(), -50, 100, "rpm");

        startCar(false);
        sim.drive.P.clutchReleaseRate = 1.6;
        sim.drive.clutchPedal = true;
        run(0.3, none);
        sim.drive.requestShift(+1, sim.time);
        sim.pedalTarget = 0.3;
        run(0.5, none);
        sim.drive.clutchPedal = false;
        double minRpm = 1e9;
        run(3.0, [&] { minRpm = std::min(minRpm, sim.rpm()); });
        check("manual: gentle take-off, lowest rpm", minRpm, 700, 4000, "rpm");
        check("manual: moving after the take-off", sim.drive.v * 3.6, 5, 60, "km/h");
        sim.drive.requestShift(+1, sim.time);  // clutch is in: must be refused
        check("manual: shift refused without the clutch", sim.drive.gear, 1, 1, "");
        sim.drive.clutchPedal = true;
        run(0.3, none);
        sim.drive.requestShift(-1, sim.time);  // 1 -> N is fine
        sim.drive.requestShift(-1, sim.time);  // N -> R while rolling forward: refused
        check("manual: reverse refused while moving", sim.drive.gear, 0, 0, "");
    }
    {
        // automatic reverse: Q at a standstill selects R, the car backs up, D is refused while rolling
        startCar(true);
        sim.drive.requestShift(-1, sim.time);
        check("auto: Q at a stop selects reverse", sim.drive.gear, -1, -1, "");
        sim.pedalTarget = 0.3;
        run(4.0, none);
        double back = sim.drive.v * 3.6;
        std::printf("  reverse at 30%% pedal for 4 s: %.1f km/h, %.0f rpm\n", back, sim.rpm());
        check("auto: reversing speed", back, -40, -5, "km/h");
        sim.drive.requestShift(+1, sim.time);
        check("auto: drive refused while rolling backwards", sim.drive.gear, -1, -1, "");
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        run(3.0, none);
        sim.drive.brake = false;
        sim.drive.requestShift(+1, sim.time);
        check("auto: back to drive once stopped", sim.drive.gear, 1, 1, "");
    }

    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}


// --engine-type inline4 --turbo yes --phystest: the same engine with the turbocharger. Start and idle
// (no boost), the WOT dyno sweep with the turbo settled at each speed, spool-up from part load, the
// wastegate holding the target, back to idle after a lift-off, and 0-100 km/h in the car.
int physicsTestInline4Turbo(const engineModel &e, const engineType &type) {
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0, false, true);
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
