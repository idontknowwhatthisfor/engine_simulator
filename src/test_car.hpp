// --phystest for the cars: one test for every car, checked against each car's carSpec (engines/<type>/),
// as test_bike.hpp does for the bikes with bikeSpec. On the stand: cranking, start, idle, rev to the
// limiter, coast-down; the WOT dyno; for a boosted engine the turbo (spool, boost against the ECU's
// target, no surge, blow-off valve); the balance and the firing order against the engine's own
// geometry; then the car: automatic (launch, 0-100 / 0-200, shifts, a hill, coasting, sport-mode
// rev-matched downshifts, braking to a stop), manual NORMAL and DCT, reverse.
#pragma once
#include "engines.hpp"

int physicsTestCar(const engineModel &e, const engineType &type) {
    if (!type.car) {
        std::fprintf(stderr, "%s: physicsTestCar needs engineType::car (its carSpec)\n", type.id);
        return 1;
    }
    const carSpec S = type.car();
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0);
    applyCalibrationEnv(sim.P);  // VE_*, FMEP_*, TURBO_*... (src/calibration.hpp)
    sim.drive.reset(false);      // engine tests in neutral
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    const double REV_LIMIT = sim.P.revLimit;
    const bool boosted = sim.P.turbo.on && !sim.P.turbo.supercharger;
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
        std::printf("  %-46s %9.2f %-5s [%g .. %g] %s\n", what, v, unit, lo, hi, pass ? "ok" : "FAIL");
    };
    // the ECU's boost target at full pedal (bar over atmospheric) at this rpm (engineSim::boostTarget)
    auto target = [&](double rpm) { return (sim.boostTarget(rpm) - sim.P.pAtm) / 1e5; };
    std::printf("%s physics test (dt = 1/%.0f s), %.0f cc%s\n", S.name, 1.0 / PHYS_DT, sim.vDispTotal * 1e6,
                boosted ? ", turbocharged" : "");
    check("displacement", sim.vDispTotal * 1e6, S.dispLo, S.dispHi, "cc");

    // 1. cranking with the ignition off
    sim.ignition = false;
    sim.starter = true;
    run(1.0, none);
    double cMin = 1e9, cMax = -1e9, cSum = 0.0;
    int cN = 0;
    run(1.0, [&] { cMin = std::min(cMin, sim.rpm()); cMax = std::max(cMax, sim.rpm()); cSum += sim.rpm(); cN++; });
    check("cranking speed, mean", cSum / cN, S.crankLo, S.crankHi, "rpm");
    check("cranking ripple (max - min)", cMax - cMin, 20, 400, "rpm");

    // 2. start: ignition on while cranking, release the starter once it runs
    sim.ignition = true;
    double t0 = sim.time;
    while (sim.rpm() < 550.0 && sim.time - t0 < 3.0) run(0.001, none);
    check("time to catch with the starter", sim.time - t0, 0.0, 1.5, "s");
    sim.starter = false;
    double flare = 0.0;
    run(1.5, [&] { flare = std::max(flare, sim.rpm()); });
    check("start flare (peak rpm)", flare, S.flareLo, S.flareHi, "rpm");

    // 3. idle (PHYS_TRACE=1 prints the idle controller every 50 ms)
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
    check("idle speed, mean", iSum / iN, S.idleLo, S.idleHi, "rpm");
    check("idle ripple (max - min)", iMax - iMin, 2, 150, "rpm");
    check("idle manifold pressure", sim.pMan / 1000.0, S.idleMapLo, S.idleMapHi, "kPa");
    if (boosted) {
        check("idle: turbo shaft speed", sim.turboRpm() / 1000.0, 0, 40, "krpm");
        check("idle: boost (vacuum)", sim.boostBar(), -1.0, 0.0, "bar");
    }

    // 4. full throttle in neutral: to the limiter, bounce on it, lift off back to idle
    sim.pedalTarget = 1.0;
    t0 = sim.time;
    while (sim.rpm() < REV_LIMIT && sim.time - t0 < 5.0) run(0.001, none);
    check("idle -> rev limiter at full throttle", sim.time - t0, S.toLimiterLo, S.toLimiterHi, "s");
    double limMax = 0.0;
    run(1.0, [&] { limMax = std::max(limMax, sim.rpm()); });
    check("peak rpm on the limiter", limMax, REV_LIMIT, REV_LIMIT + S.limiterOver, "rpm");
    sim.pedalTarget = 0.0;
    run(4.0, none);
    check("rpm 4 s after lifting off", sim.rpm(), S.liftLo, S.liftHi, "rpm");

    // 5. ignition off from 3000 rpm
    sim.omega = 3000.0 / 60.0 * TWO_PI;
    sim.ignition = false;
    t0 = sim.time;
    while (sim.rpm() > 30.0 && sim.time - t0 < 20.0) run(0.001, none);
    check("coast down 3000 rpm -> stop, ignition off", sim.time - t0, S.coastLo, S.coastHi, "s");

    // 6. WOT dyno sweep, speed locked. A boosted engine gets 3 s to spool at each speed, with its charge
    // as hot as in a long pull and air through the intercooler (as the HUD's dyno, src/dyno.hpp)
    std::vector<double> rpms, tq, kw;
    for (double rr = 1000.0; rr <= S.sweepTo + 1.0; rr += 500.0) rpms.push_back(rr);
    tq.assign(rpms.size(), 0.0);
    kw.assign(rpms.size(), 0.0);
    std::vector<double> boostAt(rpms.size(), 0.0), targetAt(rpms.size(), 0.0);
    double maxCyl = 0.0, maxTurbo = 0.0, maxSurge = 0.0;
    const double icTau = sim.P.turbo.icTau, icStill = sim.P.turbo.icEffStill;
    if (boosted) {
        sim.P.turbo.icTau = 0.02;
        sim.P.turbo.icEffStill = sim.P.turbo.icEffMoving;
    }
    std::printf("\n  WOT dyno sweep (speed locked)\n     rpm   torque Nm   power kW   BMEP bar   manifold kPa   peak cyl bar%s\n",
                boosted ? "   boost bar   turbo krpm   wastegate   back kPa   charge C" : "");
    for (std::size_t k = 0; k < rpms.size(); k++) {
        sim.reset(0.0);
        sim.P.revLimit = 1e9;
        sim.ignition = true;
        sim.lockSpeed = true;
        sim.pedal = sim.pedalTarget = 1.0;
        sim.omega = rpms[k] / 60.0 * TWO_PI;
        if (boosted) {
            run(2.0, none);
            run(1.0, [&] { maxSurge = std::max(maxSurge, sim.surge); });  // settled: the compressor must not surge
        } else {
            run(0.4, none);
        }
        double cycleTime = 2.0 * TWO_PI / sim.omega, sum = 0.0, pk = 0.0;
        int n = 0;
        run(4.0 * cycleTime, [&] {
            sum += sim.gasTorque - sim.frictionTorque;
            n++;
            for (auto &s : sim.cyl) pk = std::max(pk, s.p);
        });
        tq[k] = sum / n;
        kw[k] = tq[k] * sim.omega / 1000.0;
        boostAt[k] = sim.boostBar();
        targetAt[k] = target(rpms[k]);
        maxCyl = std::max(maxCyl, pk);
        maxTurbo = std::max(maxTurbo, sim.turboRpm());
        std::printf("    %5.0f   %8.1f   %8.1f   %8.2f   %12.1f   %12.1f", rpms[k], tq[k], kw[k],
                    tq[k] * 2.0 * TWO_PI / sim.vDispTotal / 1e5, sim.pMan / 1000.0, pk / 1e5);
        if (boosted)
            std::printf("   %9.2f   %10.0f   %8.0f%%   %8.1f   %8.0f", sim.boostBar(), sim.turboRpm() / 1000.0,
                        sim.wastegate * 100.0, sim.pExh / 1000.0, sim.tCharge - 273.15);
        std::printf("\n");
    }
    if (boosted) {
        sim.P.turbo.icTau = icTau;
        sim.P.turbo.icEffStill = icStill;
    }
    std::printf("\n");
    std::size_t iT = 0, iP = 0;
    for (std::size_t k = 1; k < rpms.size(); k++) {
        if (tq[k] > tq[iT]) iT = k;
        if (kw[k] > kw[iP]) iP = k;
    }
    auto at = [&](double rpm) {  // index of a sweep point
        for (std::size_t k = 0; k < rpms.size(); k++)
            if (std::fabs(rpms[k] - rpm) < 1.0) return static_cast<int>(k);
        return -1;
    };
    check(S.torqueWhat, tq[iT], S.torqueLo, S.torqueHi, "N m");
    check("peak torque rpm", rpms[iT], S.torqueRpmLo, S.torqueRpmHi, "rpm");
    check(S.powerWhat, kw[iP], S.powerLo, S.powerHi, "kW");
    check("peak power rpm", rpms[iP], S.powerRpmLo, S.powerRpmHi, "rpm");
    char what[96];
    for (const rpmTarget &p : S.torqueAt) {
        int k = at(p.rpm);
        std::snprintf(what, sizeof what, "torque at %.0f rpm", p.rpm);
        check(what, k < 0 ? -1.0 : tq[k], p.lo, p.hi, "N m");
    }
    for (const rpmTarget &p : S.powerAt) {
        int k = at(p.rpm);
        std::snprintf(what, sizeof what, "power at %.0f rpm", p.rpm);
        check(what, k < 0 ? -1.0 : kw[k], p.lo, p.hi, "kW");
    }
    if (S.plateauMax > 0.0) {
        double lo = 1e9, hi = 0.0;
        for (std::size_t k = 0; k < rpms.size(); k++)
            if (rpms[k] >= S.plateauFrom - 1.0 && rpms[k] <= S.plateauTo + 1.0) {
                lo = std::min(lo, tq[k]);
                hi = std::max(hi, tq[k]);
            }
        std::snprintf(what, sizeof what, "torque plateau %.0f-%.0f rpm, max - min", S.plateauFrom, S.plateauTo);
        check(what, hi - lo, 0, S.plateauMax, "N m");
    }

    // 7. a boosted engine: the turbo against the ECU's target, spool, blow-off valve
    if (boosted) {
        int mid = at(S.boostCheckRpm), top = static_cast<int>(rpms.size()) - 1;
        std::snprintf(what, sizeof what, "boost at %.0f rpm vs the ECU's target", S.boostCheckRpm);
        check(what, mid < 0 ? 9.0 : boostAt[mid] - targetAt[mid], -0.08, 0.05, "bar");
        std::snprintf(what, sizeof what, "boost at %.0f rpm vs the ECU's target", rpms[top]);
        check(what, boostAt[top] - targetAt[top], -0.08, 0.05, "bar");
        check("full throttle: no compressor surge (dyno)", maxSurge, 0.0, 0.05, "");
        check("highest turbo shaft speed", maxTurbo / 1000.0, S.turboMaxLo, S.turboMaxHi, "krpm");
        check("highest cylinder pressure", maxCyl / 1e5, S.cylMaxLo, S.cylMaxHi, "bar");
        // spool-up: light pedal at a held speed, then floored
        sim.reset(0.0);
        sim.lockSpeed = true;
        sim.omega = S.spoolRpm / 60.0 * TWO_PI;
        sim.pedal = sim.pedalTarget = 0.2;
        run(3.0, none);
        check("20% pedal: boost (vacuum)", sim.boostBar(), -0.8, 0.0, "bar");
        sim.pedalTarget = 1.0;
        t0 = sim.time;
        double t90 = -1.0, over = -1.0;
        while (sim.time - t0 < 5.0) {
            run(0.001, none);
            over = std::max(over, sim.boostBar());
            if (t90 < 0.0 && sim.boostBar() >= 0.9 * target(sim.rpm())) t90 = sim.time - t0;
        }
        std::snprintf(what, sizeof what, "%.0f rpm floored: time to 90%% boost", S.spoolRpm);
        check(what, t90, S.spoolLo, S.spoolHi, "s");
        check("boost overshoot over the target", over - target(sim.rpm()), -0.05, 0.15, "bar");
        // lift-off on boost at 4000 rpm: the blow-off valve vents, no surge
        sim.omega = 4000.0 / 60.0 * TWO_PI;
        run(2.0, none);
        sim.pedalTarget = 0.0;
        double bovPeak = 0.0, surgeMax = 0.0;
        run(1.0, [&] { bovPeak = std::max(bovPeak, sim.bovFlow); surgeMax = std::max(surgeMax, sim.surge); });
        check("BOV: vents the charge (peak flow)", bovPeak, 0.02, 0.4, "kg/s");
        check("BOV: no surge", surgeMax, 0.0, 0.1, "");
        sim.lockSpeed = false;
    }

    // 8. balance at a locked 6000 rpm, ignition off, against the slider-crank textbook for this crank:
    // one cylinder's 1st- and 2nd-order force m r w^2 and m r^2 / L w^2 at its crank angle (its firing
    // lag round: the cycle angle is 720 deg periodic), its vertical share (cos bank) for a V, summed over
    // the cylinders as forces and, with each cylinder's place along the crank, as rocking couples
    {
        sim.reset(0.0);
        sim.lockSpeed = true;
        sim.ignition = false;
        sim.omega = 6000.0 / 60.0 * TWO_PI;
        run(1.5, none);
        const double w2 = sim.omega * sim.omega, m = sim.P.mRecip;
        double f1x = 0, f1y = 0, f2x = 0, f2y = 0, c1x = 0, c1y = 0, c2x = 0, c2y = 0, one1 = 0, one2 = 0;
        for (int c = 0; c < sim.cylCount(); c++) {
            const cylinderState &s = sim.cyl[c];
            // the vertical share of the force along the bore (a V engine's bores lean by their bank)
            double up = std::cos(e.cyls[c].bank);
            double a1 = m * s.r * w2 * up, a2 = m * s.r * s.r / s.L * w2 * up, lag = sim.lag[c];
            one1 = std::max(one1, a1);
            one2 = std::max(one2, a2);
            f1x += a1 * std::cos(lag); f1y += a1 * std::sin(lag);
            f2x += a2 * std::cos(2.0 * lag); f2y += a2 * std::sin(2.0 * lag);
            c1x += a1 * s.z * std::cos(lag); c1y += a1 * s.z * std::sin(lag);
            c2x += a2 * s.z * std::cos(2.0 * lag); c2y += a2 * s.z * std::sin(2.0 * lag);
        }
        double f1 = std::hypot(f1x, f1y), f2 = std::hypot(f2x, f2y), couple = std::hypot(std::hypot(c1x, c1y), std::hypot(c2x, c2y));
        std::printf("\n  balance at 6000 rpm: 1st %.0f N (textbook %.0f), 2nd %.0f N (textbook %.0f); rocking %.0f N m "
                    "(textbook %.0f: 1st order %.0f, 2nd order %.0f)\n",
                    sim.shake1(), f1, sim.shake2(), f2, sim.pitchAmp(), couple, std::hypot(c1x, c1y), std::hypot(c2x, c2y));
        // a force the crank cancels is checked against one cylinder's (it must stay near zero), one it
        // does not against the textbook
        auto force = [&](const char *name, double simV, double book, double one) {
            if (book < 0.02 * one) {
                std::snprintf(what, sizeof what, "%s-order force / one cylinder's (cancels)", name);
                check(what, simV / one, 0.0, 0.03, "");
            } else {
                std::snprintf(what, sizeof what, "%s-order force / textbook", name);
                check(what, simV / book, 0.95, 1.05, "");
            }
        };
        force("1st", sim.shake1(), f1, one1);
        force("2nd", sim.shake2(), f2, one2);
        if (couple < 5.0) check("rocking moment (mirror symmetric: none)", sim.pitchAmp(), 0.0, 5.0, "N m");
        else check("rocking moment / textbook couples", sim.pitchAmp() / couple, 0.95, 1.05, "");
        sim.lockSpeed = false;
    }

    // 9. firing: at a held 3000 rpm the cylinders fire in the mesh's order, at its intervals
    {
        sim.reset(0.0);
        sim.lockSpeed = true;
        sim.ignition = true;
        sim.P.revLimit = 1e9;
        sim.pedal = sim.pedalTarget = 0.5;
        sim.omega = 3000.0 / 60.0 * TWO_PI;
        run(0.3, none);
        std::vector<bool> was(sim.cyl.size());
        for (int c = 0; c < sim.cylCount(); c++) was[c] = sim.cyl[c].sparked;
        std::vector<std::pair<double, int>> fires;
        double crank = 0.0;
        run(4.0 * 720.0 * DEGR / sim.omega, [&] {
            crank += sim.omega * PHYS_DT;
            for (int c = 0; c < sim.cylCount(); c++) {
                if (sim.cyl[c].sparked && !was[c]) fires.push_back({crank / DEGR, c});
                was[c] = sim.cyl[c].sparked;
            }
        });
        // expected interval from each cylinder to the next to fire, from the lags
        double worst = fires.size() < static_cast<std::size_t>(3 * sim.cylCount()) ? 999.0 : 0.0;
        for (std::size_t k = 1; k < fires.size(); k++) {
            double want = std::fmod((sim.lag[fires[k].second] - sim.lag[fires[k - 1].second]) / DEGR + 720.0, 720.0);
            worst = std::max(worst, std::fabs(fires[k].first - fires[k - 1].first - want));
        }
        std::string order;
        std::size_t from = 0;
        while (from < fires.size() && fires[from].second != 0) from++;
        for (std::size_t k = from; k < fires.size() && k < from + sim.cyl.size(); k++)
            order += (order.empty() ? "" : "-") + std::to_string(fires[k].second + 1);
        std::printf("  firing order %s (mesh: %s), %zu sparks in 4 cycles\n", order.c_str(), firingOrder(e).c_str(), fires.size());
        check("firing interval error vs the mesh's lags (max)", worst, 0.0, 1.0, "deg");
        std::string mesh = firingOrder(e);  // "1-2-4-5-3, 144 deg apart": the order is the part before the comma
        check("firing order matches the mesh's", order == mesh.substr(0, mesh.find(',')) ? 1.0 : 0.0, 1, 1, "");
        sim.lockSpeed = false;
        sim.P.revLimit = REV_LIMIT;
    }

    // 10. the car
    auto startCar = [&](bool automatic) {
        sim.lockSpeed = false;
        sim.P.revLimit = REV_LIMIT;
        sim.reset(e.cyls[0].theta0);
        sim.drive.reset(automatic);
        sim.drive.grade = sim.drive.cargo = 0.0;  // the road and load are not part of reset()
        sim.drive.sport = sim.drive.dct = false;
        sim.pedalTarget = 0.0;
        sim.ignition = true;
        sim.starter = true;
        run(0.6, none);
        sim.starter = false;
        run(3.0, none);
    };
    std::printf("\n  the car: %s\n", S.carWhat);
    {
        startCar(true);
        check("auto: idling at a standstill, clutch open", sim.drive.clutch, 0.0, 0.05, "");
        check("auto: car held still at idle", sim.drive.v * 3.6, -0.1, 0.1, "km/h");
        sim.pedalTarget = 1.0;
        t0 = sim.time;
        double t100 = -1.0, t200 = -1.0, launchRpm = 0.0;
        std::vector<std::pair<int, double>> shifts;
        int lastGear = sim.drive.gear;
        double lastRpm = sim.rpm();
        bool want200 = S.zeroTo200What != nullptr;
        while (sim.time - t0 < 30.0 && (t100 < 0.0 || (want200 && t200 < 0.0))) {
            run(0.001, [&] {
                // a change starting: the clutch opening (1) or a seamless dual-clutch handover (3)
                if ((sim.drive.shiftPhase == 1 || sim.drive.shiftPhase == 3) && sim.drive.targetGear != lastGear) {
                    shifts.push_back({sim.drive.targetGear, lastRpm});
                    lastGear = sim.drive.targetGear;
                }
                if (sim.drive.shiftPhase == 0) lastRpm = sim.rpm();
                if (sim.time - t0 < 0.6) launchRpm = std::max(launchRpm, sim.rpm());
            });
            if (t100 < 0.0 && sim.drive.v * 3.6 >= 100.0) t100 = sim.time - t0;
            if (t200 < 0.0 && sim.drive.v * 3.6 >= 200.0) t200 = sim.time - t0;
        }
        std::printf("  WOT shifts:");
        for (auto &sh : shifts) std::printf("  ->%d at %.0f", sh.first, sh.second);
        std::printf("\n");
        if (S.launchHi > 0.0) check("auto: launch rpm (first 0.6 s, peak)", launchRpm, S.launchLo, S.launchHi, "rpm");
        check(S.zeroTo100What, t100, S.zeroTo100Lo, S.zeroTo100Hi, "s");
        if (want200) check(S.zeroTo200What, t200, S.zeroTo200Lo, S.zeroTo200Hi, "s");
        double lo = 1e9;
        for (auto &sh : shifts) lo = std::min(lo, sh.second);
        check("auto: WOT upshift rpm (lowest)", shifts.empty() ? 0.0 : lo, S.wotUpLo, REV_LIMIT + 100.0, "rpm");
    }
    {
        startCar(true);
        sim.pedalTarget = 0.2;
        double firstShift = -1.0, lastRpm = 0.0;
        t0 = sim.time;
        while (sim.time - t0 < 20.0 && firstShift < 0.0)
            run(0.001, [&] {
                if (sim.drive.shiftPhase == 0) lastRpm = sim.rpm();
                else if (firstShift < 0.0) firstShift = lastRpm;
            });
        check("auto: 20% pedal, first upshift rpm", firstShift, S.lightUpLo, S.lightUpHi, "rpm");
    }
    {
        // cruise, then a hill: the box changes down for the load (it does above 30% pedal)
        startCar(true);
        sim.pedalTarget = S.hillPedal;
        run(15.0, none);
        int flatGear = sim.drive.gear;
        double flatKmh = sim.drive.v * 3.6;
        sim.drive.grade = S.hillGrade;
        run(15.0, none);
        std::printf("  %.0f%% pedal: gear %d at %.0f km/h on the flat -> gear %d at %.0f km/h, %.0f rpm on a %.0f%% hill\n",
                    S.hillPedal * 100.0, flatGear, flatKmh, sim.drive.gear, sim.drive.v * 3.6, sim.rpm(), S.hillGrade * 100.0);
        check("auto: gear after 15 s cruising", flatGear, S.flatGearLo, S.flatGearHi, "");
        check("auto: gears dropped for the hill", flatGear - sim.drive.gear, 1, 5, "");
        check("auto: engine still running on the hill", sim.rpm(), 1300, REV_LIMIT, "rpm");
    }
    if (S.coastDownHi > 0.0) {
        // normal mode: coasting down from speed with the throttle shut changes down only near normalDown
        startCar(true);
        sim.pedalTarget = 0.5;
        run(15.0, none);
        sim.pedalTarget = 0.0;
        int g0 = sim.drive.gear;
        double downAt = -1.0, lastRpm = sim.rpm();
        while (downAt < 0.0 && sim.drive.v > 2.5 && sim.time < 300.0)
            run(0.001, [&] {
                const drivetrain &d = sim.drive;
                if (d.shiftPhase == 0) lastRpm = sim.rpm();
                else if (downAt < 0.0 && d.targetGear < d.gear) downAt = lastRpm;  // lifting off may upshift first
            });
        std::printf("  normal coast from gear %d: first downshift at %.0f rpm\n", g0, downAt);
        check("normal: coast-down downshift rpm", downAt, S.coastDownLo, S.coastDownHi, "rpm");
    }
    {
        // sport mode: up near the limiter at full throttle; braking changes down as soon as the lower
        // gear fits (below sportDown), each downshift rev-matched
        startCar(true);
        sim.drive.sport = true;
        sim.pedalTarget = 1.0;
        run(14.0, none);
        int topGear = sim.drive.gear;
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        struct down { int to; double before, match, after; };
        std::vector<down> downs;
        int phase = 0;
        double before = 0.0;
        run(6.0, [&] {
            const drivetrain &d = sim.drive;
            if (phase == 0 && d.shiftPhase == 1 && d.revMatch) { phase = 1; before = sim.rpm(); }
            if (phase == 1 && d.shiftPhase == 2 && d.clutch > 0.5) { phase = 2; downs.push_back({d.targetGear, before, d.matchRpm, sim.rpm()}); }
            if (phase == 2 && d.shiftPhase == 0) phase = 0;
        });
        sim.drive.brake = false;
        std::printf("  sport: gear %d after 14 s WOT; braking downshifts:", topGear);
        double worst = 0.0;
        for (auto &d : downs) {
            std::printf("  ->%d %.0f->%.0f (need %.0f)", d.to, d.before, d.after, d.match);
            worst = std::max(worst, std::fabs(d.after - d.match));
        }
        std::printf("\n");
        check("sport: downshifts while braking", static_cast<double>(downs.size()), 2, 6, "");
        check("sport: rev match error at clutch-in (worst)", worst, 0, 400, "rpm");
        double sd = sim.drive.P.sportDown;
        check("sport: downshift waits until the lower gear fits", downs.empty() ? 0.0 : downs[0].match, sd - 2500.0, sd + 100.0, "rpm");
        sim.drive.sport = false;
    }
    for (bool sp : {false, true}) {
        // braking to a stop from a cruise: in 1st by 9 km/h, engine still running, pulls away again
        startCar(true);
        sim.drive.sport = sp;
        sim.pedalTarget = 0.4;
        run(12.0, none);
        int g0 = sim.drive.gear;
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        double gearAt9 = -1.0, minRpm = 1e9;
        while (sim.drive.v > 0.01 && sim.time < 300.0)
            run(0.001, [&] {
                if (gearAt9 < 0.0 && sim.drive.v < 9.0 / 3.6) gearAt9 = sim.drive.gear;
                minRpm = std::min(minRpm, sim.rpm());
            });
        run(1.0, none);
        sim.drive.brake = false;
        sim.pedalTarget = 0.3;
        run(3.0, none);
        std::printf("  %s: braked from gear %d, gear %d at 9 km/h, lowest %.0f rpm, %.0f km/h after pulling away\n",
                    sp ? "sport" : "normal", g0, int(gearAt9), minRpm, sim.drive.v * 3.6);
        check(sp ? "sport: in 1st below 9 km/h" : "normal: in 1st below 9 km/h", gearAt9, 1, 1, "");
        check(sp ? "sport: no stall braking to the stop" : "normal: no stall braking to the stop", minRpm, 500, REV_LIMIT, "rpm");
        check(sp ? "sport: pulls away after the stop" : "normal: pulls away after the stop", sim.drive.v * 3.6, 5, 80, "km/h");
        sim.drive.sport = false;
    }
    if (S.launchTo100What) {
        // launch control: brake and full throttle together. The box only lets the clutch carry what the
        // brakes hold, the engine works against them at the launch rpm and a turbo builds boost; let go
        // of the brake and it launches
        startCar(true);
        sim.drive.brake = true;
        sim.pedalTarget = 1.0;
        double creep = 0.0, rpmSum = 0.0;
        int n = 0;
        run(1.0, none);
        run(1.0, [&] { creep = std::max(creep, std::fabs(sim.drive.v)); rpmSum += sim.rpm(); n++; });
        double boost = sim.boostBar();
        std::printf("  launch control: held at %.0f rpm, boost %+.2f bar (target %+.2f), clutch %.2f\n", rpmSum / n, boost,
                    target(sim.rpm()), sim.drive.clutch);
        check("launch control: car held still by the brakes", creep * 3.6, 0.0, 0.5, "km/h");
        double lr = sim.drive.launchRpm(1.0);
        check("launch control: revs held near the launch rpm", rpmSum / n, lr - 800.0, lr + 800.0, "rpm");
        if (boosted) check("launch control: boost built against the brakes", boost / target(sim.rpm()), 0.7, 1.2, "x tgt");
        sim.drive.brake = false;
        t0 = sim.time;
        double t100 = -1.0;
        while (t100 < 0.0 && sim.time - t0 < 20.0) {
            run(0.001, none);
            if (sim.drive.v * 3.6 >= 100.0) t100 = sim.time - t0;
        }
        check(S.launchTo100What, t100, S.launchTo100Lo, S.launchTo100Hi, "s");
    }
    {
        // reverse at a standstill, and drive refused while rolling backwards
        startCar(true);
        sim.drive.requestShift(-1, sim.time);
        check("auto: Q at a stop selects reverse", sim.drive.gear, -1, -1, "");
        sim.pedalTarget = 0.25;
        run(4.0, none);
        check("auto: reversing speed", sim.drive.v * 3.6, S.reverseLo, S.reverseHi, "km/h");
        sim.drive.requestShift(+1, sim.time);
        check("auto: drive refused while rolling backwards", sim.drive.gear, -1, -1, "");
    }
    {
        // manual NORMAL: the clutch pedal for every change, moving or not
        startCar(false);
        if (S.dumpStalls) {
            // dumping the clutch in 1st at idle stalls it
            sim.drive.clutchPedal = true;
            run(0.3, none);
            sim.drive.requestShift(+1, sim.time);
            const double rate = sim.drive.P.clutchReleaseRate;
            sim.drive.P.clutchReleaseRate = 20.0;  // drop it
            sim.drive.clutchPedal = false;
            run(1.5, none);
            sim.drive.P.clutchReleaseRate = rate;
            check("manual normal: clutch dumped at idle in 1st -> stall", sim.rpm(), -50, 100, "rpm");
            startCar(false);
        }
        sim.drive.requestShift(+1, sim.time);
        check("manual normal: 1st from N without C refused", sim.drive.gear, 0, 0, "");
        sim.drive.clutchPedal = true;
        run(0.3, none);
        sim.drive.requestShift(+1, sim.time);
        sim.pedalTarget = 0.3;
        run(0.5, none);
        sim.drive.clutchPedal = false;
        double minRpm = 1e9;
        run(3.0, [&] { minRpm = std::min(minRpm, sim.rpm()); });
        check("manual normal: gentle take-off, lowest rpm", minRpm, 600, 4000, "rpm");
        check("manual normal: moving in 1st after the take-off", sim.drive.v * 3.6, 5, 70, "km/h");
        sim.drive.requestShift(+1, sim.time);
        check("manual normal: shift on the move without C refused", sim.drive.gear, 1, 1, "");
        sim.drive.clutchPedal = true;
        run(0.3, none);
        sim.drive.requestShift(-1, sim.time);  // 1 -> N is fine
        sim.drive.requestShift(-1, sim.time);  // N -> R while rolling forward: refused
        check("manual normal: reverse refused while moving", sim.drive.gear, 0, 0, "");
        sim.drive.requestShift(+1, sim.time);
        sim.drive.requestShift(+1, sim.time);
        sim.drive.clutchPedal = false;
        run(1.0, none);
        check("manual normal: 2nd with C", sim.drive.gear, 2, 2, "");
        // braking to a stop in gear without C stalls, as with a clutch pedal
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        run(8.0, none);
        sim.drive.brake = false;
        check("manual normal: stopped in gear without C -> stall", sim.rpm(), -50, 100, "rpm");
    }
    {
        // manual DCT: the driver picks the gears, the box works the clutch: take-off, each change
        // (blipping on a downshift), anti-stall
        startCar(false);
        sim.drive.setDct(true, sim.time);
        sim.drive.requestShift(+1, sim.time);
        run(1.0, none);
        check("manual DCT: 1st from N without C", sim.drive.gear, 1, 1, "");
        check("manual DCT: idling in 1st, clutch open", sim.drive.clutch, 0.0, 0.05, "");
        sim.pedalTarget = 0.3;
        run(3.0, none);
        check("manual DCT: pulls away without C", sim.drive.v * 3.6, 10, 70, "km/h");
        // up through four gears, each at 80% of the limiter, at 60% pedal
        const double shiftAt = 0.8 * REV_LIMIT;
        const int topUp = std::min(5, sim.drive.P.gears);
        sim.pedalTarget = 0.6;
        double minRpm = 1e9, worstSlip = 0.0;
        int shifts = 0;
        for (int g = 2; g <= topUp; g++) {
            double t1 = sim.time;
            while (sim.rpm() < shiftAt && sim.time - t1 < 15.0) run(0.001, none);
            sim.drive.requestShift(+1, sim.time);
            run(1.0, [&] { minRpm = std::min(minRpm, sim.rpm()); });
            if (sim.drive.gear == g) shifts++;
            worstSlip = std::max(worstSlip, std::fabs(sim.rpm() - sim.drive.inputRpm()));
        }
        std::printf("  manual DCT up 1 -> %d at %.0f rpm: %d shifts, %.0f km/h, lowest %.0f rpm\n", topUp, shiftAt, shifts,
                    sim.drive.v * 3.6, minRpm);
        check("manual DCT: upshifts without C", shifts, topUp - 1, topUp - 1, "");
        check("manual DCT: no automatic shifts in between", sim.drive.gear, topUp, topUp, "");
        check("manual DCT: clutch locked again after the shifts", worstSlip, 0, 100, "rpm");
        check("manual DCT: no bog through the shifts", minRpm / REV_LIMIT, 0.35, 1.0, "x lim");
        // down a gear, off the throttle: the box blips to the lower gear's speed
        sim.pedalTarget = 0.0;
        run(0.5, none);
        int from = sim.drive.gear;
        sim.drive.requestShift(-1, sim.time);
        double need = 0.0, atRpm = 0.0;
        bool caught = false;
        run(0.6, [&] {
            const drivetrain &d = sim.drive;
            if (!caught && d.shiftPhase == 2 && d.clutch > 0.5) { caught = true; need = d.matchRpm; atRpm = sim.rpm(); }
        });
        std::printf("  manual DCT down %d -> %d: %.0f rpm at clutch-in (needs %.0f)\n", from, sim.drive.gear, atRpm, need);
        check("manual DCT: downshift without C", from - sim.drive.gear, 1, 1, "");
        check("manual DCT: downshift rev match error", caught ? std::fabs(atRpm - need) : 9999.0, 0, 400, "rpm");
        // flat out, Q held: every downshift that would over-rev is refused
        sim.pedalTarget = 1.0;
        run(4.0, none);
        int g0 = sim.drive.gear;
        double peak = 0.0;
        for (int k = 0; k < 6; k++) {
            sim.drive.requestShift(-1, sim.time);
            run(0.5, [&] { peak = std::max(peak, sim.rpm()); });
        }
        std::printf("  manual DCT: from gear %d at %.0f km/h, Q held: gear %d, peak %.0f rpm\n", g0, sim.drive.v * 3.6,
                    sim.drive.gear, peak);
        check("manual DCT: no over-revving downshift", peak, 0, REV_LIMIT + 400.0, "rpm");
        // anti-stall: braking to a stop in gear, no C: the clutch opens, it drops to 1st, and pulls away
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        double lowest = 1e9;
        while (sim.drive.v > 0.01 && sim.time < 400.0) run(0.001, [&] { lowest = std::min(lowest, sim.rpm()); });
        run(1.5, [&] { lowest = std::min(lowest, sim.rpm()); });
        int stoppedGear = sim.drive.gear;
        sim.drive.brake = false;
        sim.pedalTarget = 0.3;
        run(3.0, none);
        std::printf("  manual DCT: braked to a stop: gear %d, lowest %.0f rpm, %.0f km/h after pulling away\n", stoppedGear,
                    lowest, sim.drive.v * 3.6);
        check("manual DCT: anti-stall braking to the stop", lowest, 500, REV_LIMIT, "rpm");
        check("manual DCT: in 1st once stopped", stoppedGear, 1, 1, "");
        check("manual DCT: pulls away after the stop", sim.drive.v * 3.6, 5, 70, "km/h");
        // hill-hold: stopped on a 15% hill in 1st with the brake released and the throttle shut, it stays
        // put; then the throttle pulls it up the hill
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        while (sim.drive.v > 0.01 && sim.time < 500.0) run(0.001, none);
        sim.drive.grade = 0.15;
        run(1.0, none);
        sim.drive.brake = false;
        double rolled = 0.0;
        run(2.0, [&] { rolled = std::min(rolled, sim.drive.v); });
        check("manual DCT: hill-hold on 15% (most roll back)", -rolled * 3.6, 0.0, 0.5, "km/h");
        sim.pedalTarget = 0.4;
        run(3.0, none);
        check("manual DCT: pulls away up the hill", sim.drive.v * 3.6, 5, 70, "km/h");
        sim.drive.grade = 0.0;
    }
    {
        // manual NORMAL has no hill-hold: on the same hill in neutral with the brake released, it rolls back
        startCar(false);
        sim.drive.grade = 0.15;
        sim.drive.brake = true;
        run(0.5, none);
        sim.drive.brake = false;
        run(1.0, none);
        check("manual normal: no hill-hold, rolls back on 15%", sim.drive.v * 3.6, -40, -1, "km/h");
        sim.drive.grade = 0.0;
    }

    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
