// --drag-stats (hidden): one vehicle's specs, its dyno peaks and a drag run, printed as key=value
// lines for scripts/gen_stats.py, which builds stats.md from every engine and version.
//
// The run is the same for every vehicle: started and idling in automatic (normal mode, not sport),
// wheelie control off, a flat road. Launch control: brake and full throttle together for 3 s, then
// the brake let go with the throttle held flat for 60 s. The simulator has no tyre grip limit, so
// launches are traction-unlimited. "run_end" is where that minute ends; "vmax" the highest seen.
// The top speed is not taken from the run (the automatic's schedule and the time limit both cut it
// short): see topSpeed.
#pragma once
#include "engines.hpp"
#include "dyno.hpp"

// full-throttle brake torque at rpm, with the vehicle moving at v m/s (a ram-air intake gains from the
// oncoming air): the --dyno measurement (src/dyno.hpp) for one point, on a copy of base
double wotTorque(const engineSim &base, double rpm, double v) {
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    engineSim s = base;
    s.standalone = true;  // the engine alone; drive.v only feeds the intake
    s.P.revLimit = 1e9;
    s.P.turbo.icTau = 0.02;  // the intercooler soaked, as in a long pull (dynoSweep::run)
    s.reset(0.0);
    s.drive.v = v;
    s.ignition = true;
    s.lockSpeed = true;
    s.pedal = s.pedalTarget = 1.0;
    s.omega = rpm / 60.0 * TWO_PI;
    double settle = s.P.turbo.on && !s.P.turbo.supercharger ? 1.2 : 0.3;
    for (double t = 0.0; t < settle; t += PHYS_DT) s.step(PHYS_DT);
    double cycleTime = 2.0 * TWO_PI / s.omega, sum = 0.0;
    int n = 0;
    for (double t = 0.0; t < 6.0 * cycleTime; t += PHYS_DT, n++) {
        s.step(PHYS_DT);
        sum += s.gasTorque - s.frictionTorque;
    }
    return n ? sum / n : 0.0;
}

// flat out held in gear (manual, clutch in, no shifting), accelerating from v0 m/s the way a real run
// gets there, until the speed settles: the mean over 6 s windows changes by under 0.05 km/h (at most
// 240 s). Returns the last window's mean speed and rpm. For a gear that ends on the limiter: the hard
// cut (revLimit, back on at revResume) holds the speed under the cut, by how hard the load pulls it
// down. Started well under the limiter: dropped in at full load right below it, the driveline's
// shuffle (the clutch is a spring) spikes the revs into the cut, and on a turbo each cut collapses the
// boost, a cycle that keeps going and holds the speed ~2 km/h lower than a run that creeps up to it
std::pair<double, double> heldInGear(const engineSim &base, int gear, double v0) {
    engineSim s = base;
    s.standalone = false;
    s.lockSpeed = false;
    s.reset(0.0);
    s.drive.reset(false);
    s.drive.automatic = s.drive.dct = false;
    s.drive.wheelieControl = false;
    s.drive.gear = s.drive.targetGear = gear;
    s.drive.clutch = 1.0;
    s.drive.v = v0;
    s.drive.omegaIn = s.drive.omegaFromWheels();
    s.omega = s.drive.omegaIn;
    s.ignition = true;
    s.pedal = s.pedalTarget = 1.0;
    const double window = 6.0;
    double prev = -1.0, vMean = 0.0, rMean = 0.0;
    for (double t = 0.0; t < 240.0; t += window) {
        double vSum = 0.0, rSum = 0.0;
        int n = 0;
        for (double w = 0.0; w < window; w += PHYS_DT, n++) {
            s.step(PHYS_DT);
            vSum += s.drive.v;
            rSum += s.rpm();
        }
        vMean = vSum / n;
        rMean = rSum / n;
        if (prev >= 0.0 && std::fabs(vMean - prev) * 3.6 < 0.05) break;
        prev = vMean;
    }
    return {vMean, rMean};
}

// top speed: in each gear, the speed where the full-throttle drive force at the wheels (measured at
// that rpm and road speed, wotTorque) meets the rolling and air resistance (the terms
// drivetrain::integrate uses); if the engine still has force to spare at the rev limiter, the speed it
// holds riding the limiter (heldInGear). The best gear wins: where the vehicle settles held flat out
// in that gear, whatever gear the automatic's schedule would pick and however long it takes to get
// there. Checked against long runs in auto (the mean of the last 20 s of 200-300 s flat out): the tuned
// Mustang V6 settles in 6th at 263.2 km/h, 4357 rpm, this gives 263.4 km/h, 4356 rpm (5th, on the
// limiter, is faster: 286 km/h, a speed the automatic shifts out of); the tuned inline-4 on the limiter
// 227.5 / 227.5 km/h; the LFA on the limiter 317.8 / 318.3; the RS 3 299.6 (still creeping) / 300.0.
// Gears top down; a gear that can't beat the best so far even at the cut is not worked out.
// Prints top_kmh / top_gear / top_rpm / top_limit (drag or limiter), and for each gear worked out
// gear_N_kmh / gear_N_rpm / gear_N_limit.
void topSpeed(const engineSim &base) {
    const double TWO_PI = 2.0 * 3.14159265358979323846, g = 9.81;
    const drivetrainParams &D = base.drive.P;
    const double limit = base.P.revLimit, m = D.carMass;
    auto resist = [&](double v) { return D.crr * m * g * std::tanh(v / 0.3) + 0.5 * D.airDensity * D.cdA * v * v; };
    auto kRatio = [&](int gear) { return D.primary * D.ratios[gear] * D.finalDrive / D.wheelRadius; };
    auto speedAt = [&](int gear, double rpm) { return rpm / 60.0 * TWO_PI / kRatio(gear); };
    // wheel force minus resistance, flat out in gear at rpm
    auto net = [&](int gear, double rpm) {
        double v = speedAt(gear, rpm);
        return wotTorque(base, rpm, v) * kRatio(gear) * D.efficiency - resist(v);
    };
    double best = 0.0, bestRpm = 0.0;
    int bestGear = 0;
    bool bestAtLimit = false;
    for (int gear = D.gears; gear >= 1; gear--) {
        if (speedAt(gear, limit) <= best) continue;  // even at the cut it is slower
        // coarse scan up from 1000 rpm: the first point where the force runs out
        const double step = limit <= 8000.0 ? 250.0 : 500.0;
        double lo = 1000.0, hi = -1.0;
        if (net(gear, lo) <= 0.0) continue;  // can't even pull 1000 rpm in this gear
        for (double r = lo + step; r < limit + step; r += step) {
            double rr = std::min(r, limit);
            if (net(gear, rr) <= 0.0) { hi = rr; break; }
            lo = rr;
            if (rr >= limit) break;
        }
        double rpm, v;
        if (hi < 0.0) {
            // force to spare at the limiter: it rides the limiter, measured in the full sim
            auto [vHeld, rHeld] = heldInGear(base, gear, speedAt(gear, 0.85 * limit));
            v = vHeld;
            rpm = rHeld;
        } else {
            for (int k = 0; k < 12; k++) {  // bisect to ~0.1 % of the step
                double mid = 0.5 * (lo + hi);
                (net(gear, mid) > 0.0 ? lo : hi) = mid;
            }
            rpm = 0.5 * (lo + hi);
            v = speedAt(gear, rpm);
        }
        std::printf("gear_%d_kmh=%.1f\ngear_%d_rpm=%.0f\ngear_%d_limit=%s\n", gear, v * 3.6, gear, rpm, gear,
                    hi < 0.0 ? "limiter" : "drag");
        if (v > best) { best = v; bestGear = gear; bestRpm = rpm; bestAtLimit = hi < 0.0; }
    }
    std::printf("top_kmh=%.1f\ntop_gear=%d\ntop_rpm=%.0f\ntop_limit=%s\n", best * 3.6, bestGear, bestRpm,
                bestAtLimit ? "limiter" : "drag");
}

// DRAG_SECONDS env (checking topSpeed against a longer run): how long the run is held flat, s
const double DRAG_SECONDS = std::getenv("DRAG_SECONDS") ? std::atof(std::getenv("DRAG_SECONDS")) : 60.0;

int dragStats(const engineModel &e, const engineType &type) {
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0);
    applyCalibrationEnv(sim.P);

    // ---- specs and the dyno peaks (the --dyno sweep: 1000 rpm to the limiter)
    dynoSweep dyno;
    dyno.grid(sim);
    {
        engineSim s = sim;
        dyno.run(s);
    }
    std::size_t kT = 0, kP = 0;
    for (std::size_t k = 0; k < dyno.rpm.size(); k++) {
        if (dyno.torque[k] > dyno.torque[kT]) kT = k;
        if (dyno.kw[k] > dyno.kw[kP]) kP = k;
    }
    const drivetrainParams &D = sim.drive.P;
    std::printf("name=%s\n", type.name);
    std::printf("summary=%s\n", type.summary);
    std::printf("kind=%s\n", type.bike ? "bike" : "car");
    std::printf("boost=%s\n", boostName(type.boost));
    std::printf("cylinders=%zu\n", e.cyls.size());
    std::printf("cc=%.0f\n", sim.vDispTotal * 1e6);
    std::printf("rev_limit=%.0f\n", sim.P.revLimit);
    std::printf("peak_kw=%.1f\npeak_kw_rpm=%.0f\n", dyno.kw[kP], dyno.rpm[kP]);
    std::printf("peak_nm=%.1f\npeak_nm_rpm=%.0f\n", dyno.torque[kT], dyno.rpm[kT]);
    std::printf("mass_kg=%.0f\n", D.carMass);
    std::printf("gears=%d\n", D.gears);
    std::printf("final_drive=%.3f\n", D.finalDrive);
    std::printf("cda=%.3f\n", D.cdA);
    topSpeed(sim);

    // ---- start and idle in automatic
    auto run = [&](double seconds, auto &&each) {
        int n = static_cast<int>(seconds / PHYS_DT);
        for (int k = 0; k < n; k++) {
            sim.step(PHYS_DT);
            each();
        }
    };
    auto none = [] {};
    sim.lockSpeed = false;
    sim.standalone = false;
    sim.reset(e.cyls[0].theta0);
    sim.drive.reset(true);
    sim.drive.grade = sim.drive.cargo = 0.0;
    sim.drive.sport = sim.drive.dct = false;
    sim.drive.wheelieControl = false;
    sim.pedalTarget = 0.0;
    sim.ignition = true;
    sim.starter = true;
    run(0.6, none);
    sim.starter = false;
    run(3.0, none);

    // ---- launch control: brake and full throttle for 3 s (revs and boost over the last second)
    sim.drive.brake = true;
    sim.pedalTarget = 1.0;
    run(2.0, none);
    double rpmSum = 0.0;
    int n = 0;
    run(1.0, [&] { rpmSum += sim.rpm(); n++; });
    std::printf("launch_rpm=%.0f\n", rpmSum / n);
    std::printf("launch_boost_bar=%.2f\n", sim.P.turbo.on ? sim.boostBar() : 0.0);

    // ---- let go of the brake: 60 s flat out
    sim.drive.brake = false;
    const double t0 = sim.time, x0 = sim.drive.distance;
    struct mark { const char *key; double at; double t = -1.0; };
    mark speeds[] = {{"t_60mph", 60.0 * 0.44704}, {"t_100kmh", 100.0 / 3.6}, {"t_200kmh", 200.0 / 3.6},
                     {"t_300kmh", 300.0 / 3.6}};
    struct distMark { const char *key; double at; double t = -1.0, v = 0.0; };
    distMark dists[] = {{"eighth", 201.168}, {"quarter", 402.336}, {"km", 1000.0}};
    double vmax = 0.0, peakG = 0.0;
    run(DRAG_SECONDS, [&] {
        double t = sim.time - t0, v = sim.drive.v, x = sim.drive.distance - x0;
        for (mark &m : speeds)
            if (m.t < 0.0 && v >= m.at) m.t = t;
        for (distMark &m : dists)
            if (m.t < 0.0 && x >= m.at) { m.t = t; m.v = v; }
        vmax = std::max(vmax, v);
        peakG = std::max(peakG, sim.drive.accel / 9.81);
    });
    for (const mark &m : speeds) std::printf("%s=%.2f\n", m.key, m.t);
    for (const distMark &m : dists) std::printf("%s_s=%.2f\n%s_kmh=%.1f\n", m.key, m.t, m.key, m.v * 3.6);
    std::printf("run_end_kmh=%.1f\n", sim.drive.v * 3.6);
    std::printf("run_end_gear=%d\n", sim.drive.gear);
    std::printf("run_end_rpm=%.0f\n", sim.rpm());
    std::printf("vmax_kmh=%.1f\n", vmax * 3.6);
    std::printf("peak_g=%.2f\n", peakG);
    return 0;
}
