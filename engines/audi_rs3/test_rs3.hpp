// --engine-type rs3 --phystest: the Audi RS 3's 2.5 TFSI and its car. Start, idle, free rev, coast-down,
// the WOT dyno sweep with the turbo settled at each speed against Audi's figures, spool-up, the
// blow-off valve, the inline-5's balance and firing order, then the car: launch, 0-100 and 0-200 km/h,
// the S tronic's shifts, a hill, braking to a stop and reverse.
#pragma once
#include "../../src/engines.hpp"

int physicsTestRs3(const engineModel &e, const engineType &type) {
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0);
    applyCalibrationEnv(sim.P);  // VE_*, FMEP_*, IDLE_*... (src/calibration.hpp)
    sim.drive.reset(false);      // engine tests in neutral
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    const double REV_LIMIT = sim.P.revLimit;
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
    // the ECU's boost target at full pedal (bar), at this rpm and charge temperature (engineSim::boostTarget)
    auto target = [&](double rpm) { return (sim.boostTarget(rpm) - sim.P.pAtm) / 1e5; };
    const double TARGET = (sim.P.turbo.boostMap - sim.P.pAtm) / 1e5;  // the ceiling
    std::printf("Audi RS 3 2.5 TFSI physics test (dt = 1/%.0f s), %.0f cc, boost ceiling %.2f bar\n", 1.0 / PHYS_DT,
                sim.vDispTotal * 1e6, TARGET);
    check("displacement (Audi: 2480 cc)", sim.vDispTotal * 1e6, 2470, 2490, "cc");

    // 1. cranking with the ignition off
    sim.ignition = false;
    sim.starter = true;
    run(1.0, none);
    double cMin = 1e9, cMax = -1e9, cSum = 0.0;
    int cN = 0;
    run(1.0, [&] { cMin = std::min(cMin, sim.rpm()); cMax = std::max(cMax, sim.rpm()); cSum += sim.rpm(); cN++; });
    check("cranking speed, mean", cSum / cN, 150, 350, "rpm");
    check("cranking ripple (max - min)", cMax - cMin, 20, 400, "rpm");

    // 2. start: ignition on while cranking, release the starter once it runs; the cold-start flare
    sim.ignition = true;
    double t0 = sim.time;
    while (sim.rpm() < 550.0 && sim.time - t0 < 3.0) run(0.001, none);
    check("time to catch with the starter", sim.time - t0, 0.0, 1.5, "s");
    sim.starter = false;
    double flare = 0.0;
    run(1.5, [&] { flare = std::max(flare, sim.rpm()); });
    check("start flare (peak rpm)", flare, 850, 2200, "rpm");

    // 3. idle
    run(4.0, none);
    double iMin = 1e9, iMax = -1e9, iSum = 0.0;
    int iN = 0;
    run(2.0, [&] { iMin = std::min(iMin, sim.rpm()); iMax = std::max(iMax, sim.rpm()); iSum += sim.rpm(); iN++; });
    check("idle speed, mean", iSum / iN, 700, 820, "rpm");
    check("idle ripple (max - min)", iMax - iMin, 2, 150, "rpm");
    check("idle manifold pressure", sim.pMan / 1000.0, 20, 55, "kPa");
    check("idle: turbo shaft speed", sim.turboRpm() / 1000.0, 0, 40, "krpm");
    check("idle: boost (vacuum)", sim.boostBar(), -1.0, 0.0, "bar");

    // 4. full throttle in neutral: to the limiter, bounce on it, lift off back to idle
    sim.pedalTarget = 1.0;
    t0 = sim.time;
    while (sim.rpm() < REV_LIMIT && sim.time - t0 < 5.0) run(0.001, none);
    check("idle -> rev limiter at full throttle", sim.time - t0, 0.4, 3.0, "s");
    double limMax = 0.0;
    run(1.0, [&] { limMax = std::max(limMax, sim.rpm()); });
    check("peak rpm on the limiter", limMax, REV_LIMIT, REV_LIMIT + 400, "rpm");
    sim.pedalTarget = 0.0;
    run(4.0, none);
    check("rpm 4 s after lifting off", sim.rpm(), 650, 1000, "rpm");

    // 5. ignition off from 3000 rpm
    sim.omega = 3000.0 / 60.0 * TWO_PI;
    sim.ignition = false;
    t0 = sim.time;
    while (sim.rpm() > 30.0 && sim.time - t0 < 20.0) run(0.001, none);
    check("coast down 3000 rpm -> stop, ignition off", sim.time - t0, 0.8, 6.0, "s");

    // 6. WOT dyno sweep, speed locked, the turbo settled at each speed. The charge as hot as in a long
    // pull with air through the intercooler (as the HUD's dyno, src/dyno.hpp)
    std::printf("\n  WOT dyno sweep (speed locked, 3 s to spool)\n"
                "     rpm   torque Nm   power kW   boost bar   turbo krpm   wastegate   back kPa   peak cyl bar   charge C\n");
    double peakT = 0.0, peakTRpm = 0.0, peakP = 0.0, peakPRpm = 0.0, maxCyl = 0.0, maxTurbo = 0.0, maxSurge = 0.0;
    double tAt[16] = {}, pAt[16] = {}, boostAt[16] = {}, targetAt[16] = {};
    const double icTau = sim.P.turbo.icTau, icStill = sim.P.turbo.icEffStill;
    sim.P.turbo.icTau = 0.02;
    sim.P.turbo.icEffStill = sim.P.turbo.icEffMoving;
    for (int k = 0; k <= 12; k++) {
        double rr = 1000.0 + 500.0 * k;
        sim.reset(0.0);
        sim.P.revLimit = 1e9;
        sim.ignition = true;
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
        tAt[k] = t; pAt[k] = kw; boostAt[k] = sim.boostBar(); targetAt[k] = target(rr);
        maxCyl = std::max(maxCyl, pk);
        maxTurbo = std::max(maxTurbo, sim.turboRpm());
        std::printf("    %5.0f   %8.1f   %8.1f   %9.2f   %10.0f   %8.0f%%   %8.1f   %12.1f   %8.0f\n", rr, t, kw, sim.boostBar(),
                    sim.turboRpm() / 1000.0, sim.wastegate * 100.0, sim.pExh / 1000.0, pk / 1e5, sim.tCharge - 273.15);
        if (t > peakT) { peakT = t; peakTRpm = rr; }
        if (kw > peakP) { peakP = kw; peakPRpm = rr; }
    }
    sim.P.turbo.icTau = icTau;
    sim.P.turbo.icEffStill = icStill;
    std::printf("\n");
    check("peak torque (Audi: 500 N m)", peakT, 475, 530, "N m");
    std::printf("  (peak torque at %.0f rpm)\n", peakTRpm);
    check("torque at 2000 rpm (Audi: 500 from 2250)", tAt[2], 460, 530, "N m");
    check("torque at 2500 rpm (Audi: 500 from 2250)", tAt[3], 480, 525, "N m");
    check("torque at 5500 rpm (Audi: 500 to 5600)", tAt[9], 465, 525, "N m");
    {
        double lo = 1e9, hi = 0.0;  // the plateau, 2500-5000 rpm: the charge limit keeps it flat
        for (int k = 3; k <= 8; k++) { lo = std::min(lo, tAt[k]); hi = std::max(hi, tAt[k]); }
        check("torque plateau 2500-5000 rpm, max - min", hi - lo, 0, 30, "N m");
        double pLo = std::min({pAt[10], pAt[11], pAt[12]}), pHi = std::max({pAt[10], pAt[11], pAt[12]});
        check("power 6000-7000 rpm, lowest (Audi: 294 kW)", pLo, 280, 310, "kW");
        check("power 6000-7000 rpm, highest (Audi: 294 kW)", pHi, 280, 310, "kW");
    }
    check("peak power (Audi: 294 kW / 400 PS)", peakP, 280, 310, "kW");
    check("peak power rpm (Audi: 5600-7000)", peakPRpm, 5500, 7000, "rpm");
    check("power at 7000 rpm (Audi: 294 kW to 7000)", pAt[12], 265, 310, "kW");
    check("boost at 1000 rpm (below the spool point)", boostAt[0], -0.2, 0.6, "bar");
    check("boost at 3000 rpm vs the ECU's target", boostAt[4] - targetAt[4], -0.08, 0.05, "bar");
    check("boost at 7000 rpm vs the ECU's target (falls at the top)", boostAt[12] - targetAt[12], -0.08, 0.05, "bar");
    check("boost target falls at the top (7000 vs 4000 rpm)", targetAt[12] - targetAt[6], -0.5, -0.05, "bar");
    check("highest turbo shaft speed", maxTurbo / 1000.0, 80, 180, "krpm");
    check("full throttle: no compressor surge (dyno)", maxSurge, 0.0, 0.05, "");
    check("highest cylinder pressure", maxCyl / 1e5, 90, 150, "bar");

    // 7. spool-up: 2500 rpm at light pedal, then floored
    sim.reset(0.0);
    sim.lockSpeed = true;
    sim.omega = 2500.0 / 60.0 * TWO_PI;
    sim.pedal = sim.pedalTarget = 0.2;
    run(3.0, none);
    check("2500 rpm, 20% pedal: boost (vacuum)", sim.boostBar(), -0.8, 0.0, "bar");
    sim.pedalTarget = 1.0;
    t0 = sim.time;
    double t90 = -1.0, over = -1.0;
    while (sim.time - t0 < 5.0) {
        run(0.001, none);
        over = std::max(over, sim.boostBar());
        if (t90 < 0.0 && sim.boostBar() >= 0.9 * target(sim.rpm())) t90 = sim.time - t0;
    }
    check("2500 rpm floored: time to 90% boost", t90, 0.3, 2.5, "s");
    check("boost overshoot over the target", over - target(sim.rpm()), -0.05, 0.15, "bar");

    // 8. lift-off on boost at 4000 rpm: the blow-off valve vents, no surge
    sim.omega = 4000.0 / 60.0 * TWO_PI;
    run(2.0, none);
    sim.pedalTarget = 0.0;
    double bovPeak = 0.0, surgeMax = 0.0;
    run(1.0, [&] { bovPeak = std::max(bovPeak, sim.bovFlow); surgeMax = std::max(surgeMax, sim.surge); });
    check("BOV: vents the charge (peak flow)", bovPeak, 0.02, 0.4, "kg/s");
    check("BOV: no surge", surgeMax, 0.0, 0.1, "");
    sim.lockSpeed = false;

    // 9. balance at a locked 6000 rpm, ignition off: the inline-5's 72 deg crank cancels the 1st- and
    // 2nd-order forces, but leaves a rocking couple (the pitching moment)
    {
        sim.reset(0.0);
        sim.lockSpeed = true;
        sim.ignition = false;
        sim.omega = 6000.0 / 60.0 * TWO_PI;
        run(1.5, none);
        double w2 = sim.omega * sim.omega, r = sim.cyl[0].r, L = sim.cyl[0].L;
        double one1 = sim.P.mRecip * r * w2, one2 = sim.P.mRecip * r * r / L * w2;  // one cylinder's
        // the rocking couples: one cylinder's force times |sum of z_c e^(i k pin angle)|, k = 1, 2 (the crank
        // angle in the cycle is 720 deg periodic, so cylinder c's pin sits its firing lag round). Textbook
        // for an inline-5 at pitch a: 0.449 a (1st order) and 4.98 a (2nd order)
        double c1x = 0.0, c1y = 0.0, c2x = 0.0, c2y = 0.0;
        for (int c = 0; c < sim.cylCount(); c++) {
            c1x += sim.cyl[c].z * std::cos(sim.lag[c]);
            c1y += sim.cyl[c].z * std::sin(sim.lag[c]);
            c2x += sim.cyl[c].z * std::cos(2.0 * sim.lag[c]);
            c2y += sim.cyl[c].z * std::sin(2.0 * sim.lag[c]);
        }
        double couple1 = one1 * std::hypot(c1x, c1y), couple2 = one2 * std::hypot(c2x, c2y);
        double pitchM = std::fabs(sim.cyl[1].z - sim.cyl[0].z);
        std::printf("\n  balance at 6000 rpm: 1st %.0f N, 2nd %.0f N (one cylinder alone: %.0f / %.0f N)\n"
                    "  rocking couples: 1st order %.0f N m (%.3f x pitch), 2nd order %.0f N m (%.2f x pitch); sim %.0f N m\n",
                    sim.shake1(), sim.shake2(), one1, one2, couple1, std::hypot(c1x, c1y) / pitchM, couple2,
                    std::hypot(c2x, c2y) / pitchM, sim.pitchAmp());
        check("1st-order force / one cylinder's", sim.shake1() / one1, 0.0, 0.03, "");
        check("2nd-order force / one cylinder's", sim.shake2() / one2, 0.0, 0.05, "");
        check("1st-order couple / pitch (textbook 0.449)", std::hypot(c1x, c1y) / pitchM, 0.44, 0.46, "");
        check("rocking couple, sim / the two couples", sim.pitchAmp() / std::hypot(couple1, couple2), 0.95, 1.05, "");
        sim.lockSpeed = false;
    }

    // 10. firing order 1-2-4-5-3, one spark every 144 deg
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
                if (sim.cyl[c].sparked && !was[c]) fires.push_back({crank / DEGR, c + 1});
                was[c] = sim.cyl[c].sparked;
            }
        });
        double worst = fires.size() < 10 ? 999.0 : 0.0;
        for (std::size_t k = 1; k < fires.size(); k++) worst = std::max(worst, std::fabs(fires[k].first - fires[k - 1].first - 144.0));
        std::string order;
        std::size_t from = 0;
        while (from < fires.size() && fires[from].second != 1) from++;
        for (std::size_t k = from; k < fires.size() && k < from + 5; k++) order += (order.empty() ? "" : "-") + std::to_string(fires[k].second);
        std::printf("  firing order %s, %zu sparks in 4 cycles\n", order.c_str(), fires.size());
        check("firing interval error vs 144 deg (max)", worst, 0.0, 1.0, "deg");
        check("firing order is 1-2-4-5-3", order == "1-2-4-5-3" ? 1.0 : 0.0, 1, 1, "");
        sim.lockSpeed = false;
        sim.P.revLimit = REV_LIMIT;
    }

    // 11. the car
    auto startCar = [&](bool automatic) {
        sim.lockSpeed = false;
        sim.P.revLimit = REV_LIMIT;
        sim.reset(e.cyls[0].theta0);
        sim.drive.reset(automatic);
        sim.drive.grade = sim.drive.cargo = 0.0;
        sim.pedalTarget = 0.0;
        sim.ignition = true;
        sim.starter = true;
        run(0.6, none);
        sim.starter = false;
        run(3.0, none);
    };
    std::printf("\n  the car: 1645 kg with driver, 7-speed S tronic, quattro\n");
    {
        startCar(true);
        check("auto: idling at a standstill, clutch open", sim.drive.clutch, 0.0, 0.05, "");
        sim.pedalTarget = 1.0;
        t0 = sim.time;
        double t100 = -1.0, t200 = -1.0, launchRpm = 0.0;
        std::vector<std::pair<int, double>> shifts;
        int lastGear = sim.drive.gear;
        double lastRpm = sim.rpm();
        while (sim.time - t0 < 25.0 && t200 < 0.0) {
            run(0.001, [&] {
                if (sim.drive.shiftPhase == 1 && sim.drive.targetGear != lastGear) {
                    shifts.push_back({sim.drive.targetGear, lastRpm});
                    lastGear = sim.drive.targetGear;
                }
                if (sim.drive.shiftPhase == 0) lastRpm = sim.rpm();
                if (sim.time - t0 < 0.6) launchRpm = std::max(launchRpm, sim.rpm());
            });
            if (t100 < 0.0 && sim.drive.v * 3.6 >= 100.0) t100 = sim.time - t0;
            if (sim.drive.v * 3.6 >= 200.0) t200 = sim.time - t0;
        }
        std::printf("  WOT shifts:");
        for (auto &sh : shifts) std::printf("  ->%d at %.0f", sh.first, sh.second);
        std::printf("\n");
        check("auto: launch rpm (first 0.6 s, peak)", launchRpm, 2500, 5000, "rpm");
        check("auto: 0-100 km/h (Audi: 3.8 s)", t100, 3.4, 4.4, "s");
        check("auto: 0-200 km/h (tests: ~12-13 s)", t200, 10.5, 14.5, "s");
        double lo = 1e9;
        for (auto &sh : shifts) lo = std::min(lo, sh.second);
        check("auto: WOT upshift rpm (lowest)", shifts.empty() ? 0.0 : lo, 6400, REV_LIMIT + 100, "rpm");
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
        check("auto: 20% pedal, first upshift rpm", firstShift, 2000, 3500, "rpm");
    }
    {
        // cruise at 35% pedal, then a 25% hill (the RS 3 holds its speed up 15% in 4th): the box changes down for the load (it does above 30% pedal)
        startCar(true);
        sim.pedalTarget = 0.35;
        run(15.0, none);
        int flatGear = sim.drive.gear;
        double flatKmh = sim.drive.v * 3.6;
        sim.drive.grade = 0.25;
        run(15.0, none);
        std::printf("  35%% pedal: gear %d at %.0f km/h on the flat -> gear %d at %.0f km/h, %.0f rpm on a 25%% hill\n",
                    flatGear, flatKmh, sim.drive.gear, sim.drive.v * 3.6, sim.rpm());
        check("auto: gear after 15 s at 35% pedal", flatGear, 4, 7, "");
        check("auto: gears dropped for the 25% hill", flatGear - sim.drive.gear, 1, 5, "");
        check("auto: engine still running on the hill", sim.rpm(), 1300, REV_LIMIT, "rpm");
    }
    for (bool sp : {false, true}) {
        // braking to a stop from a cruise: in 1st by 9 km/h, no stall, pulls away again
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
    {
        // reverse at a standstill, and drive refused while rolling backwards
        startCar(true);
        sim.drive.requestShift(-1, sim.time);
        check("auto: Q at a stop selects reverse", sim.drive.gear, -1, -1, "");
        sim.pedalTarget = 0.25;
        run(4.0, none);
        double back = sim.drive.v * 3.6;
        check("auto: reversing speed", back, -45, -5, "km/h");
        sim.drive.requestShift(+1, sim.time);
        check("auto: drive refused while rolling backwards", sim.drive.gear, -1, -1, "");
    }
    {
        // manual NORMAL: the clutch pedal for every change, moving or not
        startCar(false);
        sim.drive.dct = false;
        sim.drive.requestShift(+1, sim.time);
        check("manual normal: 1st from N without C refused", sim.drive.gear, 0, 0, "");
        sim.drive.clutchPedal = true;
        run(0.3, none);
        sim.drive.requestShift(+1, sim.time);
        sim.pedalTarget = 0.3;
        run(0.3, none);
        sim.drive.clutchPedal = false;
        run(3.0, none);
        check("manual normal: moving in 1st after the take-off", sim.drive.v * 3.6, 10, 70, "km/h");
        sim.drive.requestShift(+1, sim.time);
        check("manual normal: shift on the move without C refused", sim.drive.gear, 1, 1, "");
        sim.drive.clutchPedal = true;
        run(0.2, none);
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
        // (blipping on a downshift) and anti-stall
        startCar(false);
        sim.drive.setDct(true, sim.time);
        sim.drive.requestShift(+1, sim.time);
        run(1.0, none);
        check("manual DCT: 1st from N without C", sim.drive.gear, 1, 1, "");
        check("manual DCT: idling in 1st, clutch open", sim.drive.clutch, 0.0, 0.05, "");
        sim.pedalTarget = 0.3;
        run(3.0, none);
        check("manual DCT: pulls away without C", sim.drive.v * 3.6, 10, 70, "km/h");
        // up through the box, each at ~5500 rpm at 60% pedal
        sim.pedalTarget = 0.6;
        double minRpm = 1e9, worstSlip = 0.0;
        int shifts = 0;
        for (int g = 2; g <= 5; g++) {
            double t1 = sim.time;
            while (sim.rpm() < 5500.0 && sim.time - t1 < 15.0) run(0.001, none);
            sim.drive.requestShift(+1, sim.time);
            run(1.0, [&] { minRpm = std::min(minRpm, sim.rpm()); });
            if (sim.drive.gear == g) shifts++;
            worstSlip = std::max(worstSlip, std::fabs(sim.rpm() - sim.drive.inputRpm()));
        }
        std::printf("  manual DCT up 1 -> 5: %d shifts, %.0f km/h, lowest %.0f rpm\n", shifts, sim.drive.v * 3.6, minRpm);
        check("manual DCT: upshifts without C", shifts, 4, 4, "");
        check("manual DCT: no automatic shifts in between", sim.drive.gear, 5, 5, "");
        check("manual DCT: clutch locked again after the shifts", worstSlip, 0, 100, "rpm");
        check("manual DCT: no bog through the shifts", minRpm, 2500, REV_LIMIT, "rpm");
        // down a gear, off the throttle: the box blips to the lower gear's speed
        sim.pedalTarget = 0.0;
        run(0.5, none);
        int from = sim.drive.gear;
        sim.drive.requestShift(-1, sim.time);
        double need = 0.0, at = 0.0;
        bool caught = false;
        run(0.6, [&] {
            const drivetrain &d = sim.drive;
            if (!caught && d.shiftPhase == 2 && d.clutch > 0.5) { caught = true; need = d.matchRpm; at = sim.rpm(); }
        });
        std::printf("  manual DCT down %d -> %d: %.0f rpm at clutch-in (needs %.0f)\n", from, sim.drive.gear, at, need);
        check("manual DCT: downshift without C", from - sim.drive.gear, 1, 1, "");
        check("manual DCT: downshift rev match error", caught ? std::fabs(at - need) : 9999.0, 0, 400, "rpm");
        // a downshift that would over-rev is refused
        sim.pedalTarget = 1.0;
        run(4.0, none);
        int g0 = sim.drive.gear;
        for (int k = 0; k < 6; k++) {
            sim.drive.requestShift(-1, sim.time);
            run(0.5, none);
        }
        std::printf("  manual DCT: from gear %d at %.0f km/h, Q held: gear %d\n", g0, sim.drive.v * 3.6, sim.drive.gear);
        check("manual DCT: over-revving downshift refused", sim.drive.gear, 2, 7, "");
        // anti-stall: braking to a stop in a high gear, no C: the clutch opens, the box drops to 1st,
        // and it pulls away again on the throttle
        sim.pedalTarget = 0.0;
        sim.drive.brake = true;
        double lowest = 1e9;
        while (sim.drive.v > 0.01 && sim.time < 400.0) run(0.001, [&] { lowest = std::min(lowest, sim.rpm()); });
        run(1.5, [&] { lowest = std::min(lowest, sim.rpm()); });
        int stoppedGear = sim.drive.gear;
        sim.drive.brake = false;
        sim.pedalTarget = 0.3;
        run(3.0, none);
        std::printf("  manual DCT: braked from gear %d to a stop: gear %d, lowest %.0f rpm, %.0f km/h after pulling away\n", g0,
                    stoppedGear, lowest, sim.drive.v * 3.6);
        check("manual DCT: anti-stall braking to the stop", lowest, 500, REV_LIMIT, "rpm");
        check("manual DCT: in 1st once stopped", stoppedGear, 1, 1, "");
        check("manual DCT: pulls away after the stop", sim.drive.v * 3.6, 5, 70, "km/h");
        sim.drive.dct = false;
    }

    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
