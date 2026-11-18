// --synthtest N (hidden): an N-cylinder inline engine built in memory, no mesh needed, run on a stand
// through a start, idle, a firing check at a held speed and a rev to the limiter. It proves the
// simulation has no 4-cylinder assumption left; `make test` runs it for 1, 2, 3, 4, 6 and 8 cylinders.
#pragma once
#include "sound.hpp"
#include "engines.hpp"

// an inline engine with the inline-4's dimensions (85 mm bore, its mean 103 mm stroke, 160 mm rods,
// 95 mm bore spacing), firing evenly in cylinder order 1-2-..-N, every 720/N degrees. Only what the
// physics reads is filled in, plus a crankpin / wrist pin pose consistent with each cylinder's lag.
engineModel synthInlineEngine(int n) {
    engineModel e;
    e.root = identity();
    e.crankAxis = {0.f, 0.f, 0.f};
    const float R = 51.5f, L = 160.f, PITCH = 95.f, BORE = 85.f;
    for (int c = 0; c < n; c++) {
        cylinderGeom g{};
        g.firingLagDeg = 720.f * static_cast<float>(c) / static_cast<float>(n);
        g.theta0 = -g.firingLagDeg * PI / 180.f;  // at rest cylinder 1 is at TDC, the others their lag behind
        g.zCentre = (static_cast<float>(c) - 0.5f * static_cast<float>(n - 1)) * PITCH;
        g.throw_ = R;
        g.rodL = L;
        g.crankpinX = R * std::sin(g.theta0);     // bore toward -y: TDC puts the crankpin at (0, -r)
        g.crankpinY = -R * std::cos(g.theta0);
        g.gudgeonX = 0.f;
        g.gudgeonY = g.crankpinY - std::sqrt(L * L - g.crankpinX * g.crankpinX);
        g.bore = BORE;
        g.deckY = -(R + L + 60.f);
        e.cyls.push_back(g);
    }
    return e;
}

// the inline-4's parameters, with what is sized for four cylinders' worth of air scaled to N: throttle
// and idle valve areas (and the idle controller's gains, which are areas per rpm), manifold volume,
// crank inertia and starter torque. Displacement, friction and pumping already follow the cylinder
// count through vDispTotal. A single still keeps half the inline-4's inertia: it has a flywheel.
engineParams synthParams(int n) {
    const engineType *base = findEngineType("inline4");
    if (!base) throw std::runtime_error("--synthtest: no inline4 engine type to take the parameters from");
    engineParams P = base->engine();
    const double k = n / 4.0;
    P.throttleArea *= k;
    P.idleBase *= k;
    P.idleMin *= k;
    P.idleMax *= k;
    P.idleKp *= k;
    P.idleKi *= k;
    P.vManifold *= k;
    P.inertia *= std::max(k, 0.5);
    P.starterTorque *= std::max(k, 0.5);
    P.turbo.on = false;
    return P;
}

int synthTest(int n) {
    if (n < 1 || n > static_cast<int>(MAX_CYLINDERS)) {
        std::fprintf(stderr, "--synthtest: 1 to %zu cylinders\n", MAX_CYLINDERS);
        return 1;
    }
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    engineModel e = synthInlineEngine(n);
    engineSim sim;
    sim.P = synthParams(n);
    sim.drive.P = findEngineType("inline4")->drive();
    sim.standalone = true;  // on a stand: nothing downstream of the crank
    sim.init(e, e.cyls[0].theta0);
    engineSound snd(1.0 / PHYS_DT, sim.P.sound);

    bool ok = true;
    auto check = [&](const char *what, double v, double lo, double hi, const char *unit) {
        bool pass = v >= lo && v <= hi;
        ok &= pass;
        std::printf("  %-42s %9.1f %-5s [%g .. %g] %s\n", what, v, unit, lo, hi, pass ? "ok" : "FAIL");
    };
    // every step: the state and the sound stay finite
    long bad = 0;
    double badAt = -1.0;
    auto run = [&](double seconds, auto &&each) {
        int steps = static_cast<int>(seconds / PHYS_DT);
        for (int k = 0; k < steps; k++) {
            sim.step(PHYS_DT);
            float s = snd.sample(sim);
            bool fin = std::isfinite(sim.omega) && std::isfinite(sim.pMan) && std::isfinite(sim.gasTorque) &&
                       std::isfinite(sim.shakeForce) && std::isfinite(sim.rollMoment) && std::isfinite(sim.mountY) &&
                       std::isfinite(s);
            for (const cylinderState &c : sim.cyl) fin &= std::isfinite(c.p) && std::isfinite(c.mAir);
            if (!fin && bad++ == 0) badAt = sim.time;
            each();
        }
    };
    auto none = [] {};
    std::printf("synthetic %d-cylinder inline engine (%.0f cc, even firing every %.1f deg), dt = 1/%.0f s\n", n,
                sim.vDispTotal * 1e6, 720.0 / n, 1.0 / PHYS_DT);

    // 1. start: crank with the ignition on, release the starter once it runs
    sim.ignition = true;
    sim.starter = true;
    double t0 = sim.time;
    while (sim.rpm() < 550.0 && sim.time - t0 < 3.0) run(0.001, none);
    check("time to catch with the starter", sim.time - t0, 0.0, 2.0, "s");
    sim.starter = false;

    // 2. idle: settled once two successive seconds average the same, near the target
    run(4.0, none);
    double sum1 = 0.0, sum2 = 0.0, lo = 1e9, hi = -1e9;
    int cnt = 0;
    run(1.0, [&] { sum1 += sim.rpm(); cnt++; });
    run(1.0, [&] { sum2 += sim.rpm(); lo = std::min(lo, sim.rpm()); hi = std::max(hi, sim.rpm()); });
    double idle1 = sum1 / cnt, idle2 = sum2 / cnt;
    check("idle, mean rpm", idle2, sim.P.idleRpm - 100.0, sim.P.idleRpm + 150.0, "rpm");
    check("idle drift between two seconds", std::fabs(idle2 - idle1), 0.0, 25.0, "rpm");
    check("idle stays running (min rpm)", lo, 0.5 * sim.P.idleRpm, 1e9, "rpm");
    std::printf("    idle ripple over a cycle: %.0f .. %.0f rpm\n", lo, hi);

    // 3. firing at a held 3000 rpm: over 10 cycles every cylinder burns once per 720 deg, every 720/N deg
    {
        sim.lockSpeed = true;
        sim.omega = 3000.0 / 60.0 * TWO_PI;
        sim.pedalTarget = 0.5;
        run(0.2, none);
        std::vector<bool> was(sim.cyl.size());
        for (int c = 0; c < sim.cylCount(); c++) was[c] = sim.cyl[c].sparked;
        std::vector<int> fired(sim.cyl.size(), 0);
        std::vector<double> at;  // crank angle of each burning spark, deg
        double crank = 0.0;
        const double CYCLES = 10.0;
        run(CYCLES * 720.0 * DEGR / sim.omega + 0.01, [&] {
            crank += sim.omega * PHYS_DT;
            for (int c = 0; c < sim.cylCount(); c++) {
                if (sim.cyl[c].sparked && !was[c] && sim.cyl[c].qTot > 0.0 && crank < CYCLES * 720.0 * DEGR) {
                    fired[c]++;
                    at.push_back(crank / DEGR);
                }
                was[c] = sim.cyl[c].sparked;
            }
        });
        int fewest = *std::min_element(fired.begin(), fired.end()), most = *std::max_element(fired.begin(), fired.end());
        check("firings per cylinder in 10 cycles, fewest", fewest, CYCLES, CYCLES, "");
        check("firings per cylinder in 10 cycles, most", most, CYCLES, CYCLES, "");
        double worst = at.size() < 2 ? 999.0 : 0.0;
        for (std::size_t k = 1; k < at.size(); k++) worst = std::max(worst, std::fabs(at[k] - at[k - 1] - 720.0 / n));
        check("firing interval error vs 720/N (max)", worst, 0.0, 1.0, "deg");
        sim.lockSpeed = false;
        sim.pedalTarget = 0.0;
    }

    // 4. full throttle up to the limiter, then bounce on it for a second
    {
        run(2.0, none);  // back to idle
        sim.pedalTarget = 1.0;
        double tUp = sim.time;
        while (!sim.revCut && sim.time - tUp < 6.0) run(0.001, none);
        check("time from idle to the limiter", sim.time - tUp, 0.0, 5.0, "s");
        double peak = 0.0, low = 1e9;
        run(1.0, [&] { peak = std::max(peak, sim.rpm()); low = std::min(low, sim.rpm()); });
        check("peak rpm on the limiter", peak, sim.P.revLimit, sim.P.revLimit + 600.0, "rpm");
        check("lowest rpm on the limiter", low, sim.P.revResume - 600.0, sim.P.revLimit, "rpm");
        sim.pedalTarget = 0.0;
        run(2.0, none);
    }

    check("steps with NaN or inf", static_cast<double>(bad), 0.0, 0.0, "");
    if (bad) std::printf("    first at t = %.4f s\n", badAt);
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
