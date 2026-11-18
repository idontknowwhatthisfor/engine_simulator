// full-throttle dyno curve for the HUD (D): torque and power against rpm, measured on a copy of the
// engine with its speed held at each point, like the --phystest sweep. It runs on a worker thread so
// the window opens at once; the chart fills in as the points arrive.
#pragma once
#include "physics.hpp"
#include <atomic>
#include <thread>
#include <tuple>
#include <vector>

struct dynoSweep {
    std::vector<float> rpm, torque, kw;  // sized up front; the worker fills them in order
    std::atomic<int> done{0};            // points measured so far
    std::atomic<bool> stop{false};
    std::thread worker;

    // engine: a freshly set-up sim (its parameters and geometry are what matter, not its state)
    void start(const engineSim &engine) {
        double top = engine.P.revLimit;
        double stepRpm = top <= 8000.0 ? 250.0 : 500.0;
        for (double r = 1000.0; r <= top + 1.0; r += stepRpm) rpm.push_back(static_cast<float>(r));
        torque.assign(rpm.size(), 0.f);
        kw.assign(rpm.size(), 0.f);
        worker = std::thread([this, s = engine]() mutable { run(s); });
    }
    ~dynoSweep() {
        stop = true;
        if (worker.joinable()) worker.join();
    }

    void run(engineSim &s) {
        const double TWO_PI = 2.0 * 3.14159265358979323846;
        s.standalone = true;  // on the dyno: no clutch or vehicle
        s.P.revLimit = 1e9;
        // boosted charge: measure with the intake as hot as it gets in a long pull. The intercooler
        // core soaks heat over seconds (icTau), far longer than each point settles, so let the charge
        // reach that temperature at once; and a dyno's fan gives the intercooler moving-air cooling
        s.P.turbo.icTau = 0.02;
        s.P.turbo.icEffStill = s.P.turbo.icEffMoving;
        // an exhaust turbo needs time to spool up at each speed; the rest settles in a few cycles
        double settle = s.P.turbo.on && !s.P.turbo.supercharger ? 1.2 : 0.3;
        for (std::size_t k = 0; k < rpm.size() && !stop; k++) {
            s.reset(0.0);
            s.ignition = true;
            s.lockSpeed = true;
            s.pedal = s.pedalTarget = 1.0;
            s.omega = rpm[k] / 60.0 * TWO_PI;
            for (double t = 0.0; t < settle && !stop; t += PHYS_DT) s.step(PHYS_DT);
            // average the brake torque over whole cycles
            double cycleTime = 2.0 * TWO_PI / s.omega, sum = 0.0;
            int n = 0;
            for (double t = 0.0; t < 6.0 * cycleTime && !stop; t += PHYS_DT, n++) {
                s.step(PHYS_DT);
                sum += s.gasTorque - s.frictionTorque;
            }
            if (n == 0) break;
            torque[k] = static_cast<float>(sum / n);
            kw[k] = static_cast<float>(sum / n * s.omega / 1000.0);
            done.store(static_cast<int>(k) + 1, std::memory_order_release);
        }
    }
};
