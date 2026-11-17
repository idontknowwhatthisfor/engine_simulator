// engine physics (4-stroke pressure model per cylinder, crank dynamics, ECU) and the drivetrain
// (clutch, gearbox, vehicle). The parameter structs' defaults are a generic baseline; each engine
// sets its own values in engines/<type>/.
#pragma once
#include "model.hpp"

// ---------------------------------------------------------------------------------------------
// engine physics, SI units, double precision
// ---------------------------------------------------------------------------------------------
// State: cycle angle `theta` of cylinder 1 (0..720 deg, 0 = TDC at the start of intake) and crank
// speed `omega`. Each cylinder runs its own 4-stroke pressure model; gas, reciprocating inertia,
// friction, starter and load torques drive the crank through the exact slider-crank geometry.

const double DEGR = 3.14159265358979323846 / 180.0;
const double CYCLE = 720.0 * DEGR;
// how far each cylinder's cycle lags cylinder 1 comes from the model (cylinderGeom::firingLagDeg):
// inline-4 1-3-4-2 = 0, 540, 180, 360; Ducati V4 Twin Pulse = 0, 90, 290, 380

double wrapCycle(double a) {
    a = std::fmod(a, CYCLE);
    return a < 0.0 ? a + CYCLE : a;
}

// how an engine sounds (see engineSound). The defaults are the original voice the inline-4 and
// the V4 use; `superbike` switches on the extra stages tuned for the BMW S 1000 RR.
struct soundParams {
    double pipeLowHz = 700.0, pipeResHz = 110.0, pipeResGain = 1.5, masterHz = 7000.0;
    bool superbike = false;
    // 4-2-1 exhaust as a quarter-wave pipe: valve end closed, tailpipe open, so it rings at the odd
    // multiples of c / 4L. The reflection loses highs every round trip.
    double pipeLength = 1.15;       // m, header + link pipe + silencer
    double gasSpeed = 520.0;        // m/s, speed of sound in ~670 K exhaust gas
    double pipeReflect = 0.55, pipeDampHz = 3500.0;
    // silencer flap: shut at idle and light load (muffled), open from ~5000 rpm or with the throttle up
    double flapOpenRpm = 5000.0, flapSpan = 1500.0;
    double flapShutHz = 1600.0, flapOpenHz = 6500.0, flapShutGain = 0.55;
    // airbox: tonal honk from the four intake pulses, loud at full throttle
    double airboxHz = 190.0, intakeHowlHz = 750.0, intakeGain = 0.0;
    // overrun crackle (lift-off above popRpm), quickshifter bark, limiter crackle
    double popRpm = 3500.0, popChance = 0.0;
    // straight-cut gear whine: primary pair and the engaged gearbox pair
    double whineGain = 0.0;
    double drive = 0.6;             // soft-clip drive into the output
    // what leaves the tailpipe is heard as the rate of change of the gas flow (an open pipe end
    // radiates like a monopole): +6 dB per octave, so the voice brightens and swells with the revs.
    // radiateHz is the frequency where the derivative has unity gain
    double radiateHz = 0.0;         // 0 = off (the original voice)
    double burble = 0.0;            // the low exhaust body under 300 Hz (the idle burble), heard directly
    double tick = 0.0;              // valvetrain: a click as each cam lobe snaps a valve shut
};

struct engineParams {
    soundParams sound;
    double bore = 0.085;            // m
    double compression = 10.0;
    double gamma = 1.3;             // polytropic exponent, compression and expansion
    double ivc = 220.0 * DEGR;      // intake valve closes 40 deg after BDC
    double evo = 500.0 * DEGR;      // exhaust valve opens 40 deg before BDC
    double blowdown = 12.0 * DEGR;  // exhaust pressure decay length after EVO
    double pAtm = 101325.0;
    double pExhaust = 105000.0;
    double pCrankcase = 101325.0;
    double tIntake = 300.0;         // K
    double rGas = 287.0;
    double afr = 14.7;
    double lhv = 44e6;              // J/kg petrol
    double etaHeat = 0.85;          // fuel energy that ends up as gas pressure (combustion + wall losses)
    double burn = 50.0 * DEGR;      // Wiebe burn duration
    double wiebeA = 5.0, wiebeM = 2.0;
    double leakTau = 1.0;           // s, ring blow-by time constant: ~1.5% of the charge at 3000 rpm, ~18% when cranking
    double mRecip = 0.6;            // kg per cylinder: piston + pin + 1/3 rod
    double inertia = 0.13;          // kg m^2: crank + flywheel + 2/3 rods
    double vManifold = 3.0e-3;      // m^3
    double throttleArea = 2.83e-3;  // m^2, 60 mm throttle bore
    double throttleCd = 0.8;
    double idleBase = 2.0e-5, idleMin = 1.6e-5, idleMax = 1.2e-4;  // m^2 idle bypass valve (held open on overrun)
    double idleKp = 1e-8, idleKi = 1.5e-8;       // slow air PI gains on filtered rpm, m^2 per rpm (per rpm s)
    double idleBand = 300.0;                     // rpm: air trim only integrates this close to idle
    double idleSparkBase = 8.0, idleSparkGain = 0.03;  // deg, deg per rpm: fast idle control through spark
    double idleRpm = 800.0;
    double starterTorque = 140.0;   // N m at the crank when stalled
    double starterMaxRpm = 300.0;
    double revLimit = 6800.0, revResume = 6500.0;
    double fmepA = 1.0e5, fmepB = 0.12e5, fmepC = 0.04e5;  // Pa, Pa per krpm, Pa per krpm^2 (A includes ~8 N m of accessories)
    double pedalRate = 4.0;         // throttle travel per second
    // breathing and spark maps
    double veBase = 0.80, vePeak = 0.12, veRpm = 4500.0, veWidth = 2800.0;  // VE = base + peak * gauss(rpm)
    // variable intake cam (BMW ShiftCam): a second, mild cam lobe with its own VE curve. The ECU
    // runs whichever lobe breathes better at this rpm. lowCam = false: one fixed cam (the curve above)
    bool lowCam = false;
    double veLowBase = 0.0, veLowPeak = 0.0, veLowRpm = 0.0, veLowWidth = 1.0;
    double sparkLow = 8.0, sparkHigh = 30.0, sparkFrom = 800.0, sparkSpan = 3500.0;  // deg BTDC ramp
    double mapMin = 30e3;           // Pa, manifold pressure the drive-by-wire asks for at zero pedal
    double residualMin = 0.7;       // combustion effectiveness at very low load (residual gas dilution)
    // crank counterweights cancel this share of each cylinder's 1st-order reciprocating force. The
    // inline-4's flat-plane crank cancels it by symmetry (0); on a 90 deg V the pair on one pin
    // makes a constant rotating force that a counterweight opposite the pin cancels fully (0.5 each)
    double counterweight = 0.0;
    double rpmScale = 8000.0;       // HUD rpm bar full scale
    double fuelCutRpm = 1400.0;     // overrun fuel cut above this (throttle shut, in gear)
    // engine on its mounts (rigid block on spring-dampers), for the vibration readout and shake view
    double blockMass = 150.0;       // kg
    double blockRollInertia = 5.0;  // kg m^2 about the crank axis
    double mountFreqY = 12.0, mountFreqRoll = 9.0;  // Hz
    double mountZeta = 0.15;
};

struct cylinderState {
    double r = 0, L = 0, area = 0, vDisp = 0, vClear = 0;
    double z = 0;           // cylinder position along the crank, m (for the pitching moment)
    double p = 101325.0;
    double mAir = 0.0;      // trapped air at IVC, kg
    double qTot = 0.0;      // heat to release this cycle, J
    double xbDone = 0.0;    // Wiebe fraction already released
    double phiSpark = 0.0;
    double heatRate = 0.0;  // J/s
    double glowEvo = 0.0;   // glow when the exhaust valve opened, fades through blowdown
    bool sparked = false;
    int region = 0;         // 0 intake, 1 closed (compression/power), 2 exhaust
};

// ---------------------------------------------------------------------------------------------
// drivetrain: clutch, 5-speed gearbox, final drive and the car it pushes
// ---------------------------------------------------------------------------------------------
// The clutch is a friction coupling: it passes slip * stiffness up to engagement * capacity, so it
// slips on take-off, locks once the speeds match, and can drag the engine down (stall) when dumped.
// Automatic mode works the clutch from engine rpm (like a centrifugal clutch) and shifts on rpm,
// throttle and load: early upshifts at light throttle, late at full throttle, kickdown when floored,
// and a downshift when the engine labours (throttle open but the car is not accelerating, e.g. a hill).

struct drivetrainParams {
    // 2nd-generation (2009-2013) Honda Fit 5-speed manual, from Honda's 2010 spec sheet
    std::array<double, 7> ratios{0.0, 3.308, 1.870, 1.303, 0.949, 0.727, 0.0};  // index 0 = neutral
    int gears = 5;
    bool bike = false;              // sequential box: 1-N-2-3..., no reverse, quickshifter, slipper clutch
    double primary = 1.0;           // crank -> gearbox input reduction (bikes have a primary gear pair)
    double reverseRatio = 3.308;    // gear -1, through an idler so the output turns backwards
    double finalDrive = 4.62;
    double wheelRadius = 0.305;     // 175/65R15 / 185/55R16 tyres, ~610 mm diameter
    double efficiency = 0.92;
    double carMass = 1300.0, wheelInertia = 3.6;  // kg, kg m^2 for all four wheels
    double crr = 0.012, cdA = 0.65, airDensity = 1.2;
    double clutchCapacity = 350.0;  // N m when fully engaged (well above the 203 N m peak)
    double clutchStiffness = 400.0; // N m per rad/s of slip while below capacity
    double inputInertia = 0.02;     // clutch disc + input shaft + countershaft, kg m^2
    double brakeDecel = 8.0;        // m/s^2 with the brake held
    double shiftOpen = 0.12, shiftClose = 0.25;            // automatic shift: clutch out / in, s
    double normalDown = 1200.0;     // normal mode: change down when the engine falls to this, rpm
    double sportUp = 6400.0;        // sport mode: change up here, just under the 6500-6800 limiter, rpm
    double sportDown = 5500.0;      // sport mode: change down whenever the lower gear lands below this, rpm
    double matchHold = 0.6;         // sport mode: longest the clutch waits for the rev-match blip, s
    double firstGearSpeed = 10.0 / 3.6;  // automatic: below this (m/s) go straight back to 1st
    double clutchPressRate = 6.0, clutchReleaseRate = 1.6; // manual pedal travel per s
    // automatic: clutch and shift rpm thresholds
    double openRpm = 900.0;         // clutch fully open below this
    double idleOpenRpm = 1000.0;    // ... or below this with the throttle shut
    double biteSpan = 1300.0;       // take-off: engagement rises from openRpm over this many rpm
    double launchPedal = 0.0;       // take-off: the bite point moves up this many rpm at full throttle
    double upBase = 2000.0, upPedal = 4300.0;  // upshift at upBase + upPedal * pedal
    double upMargin = 250.0;        // the next gear must land this far above normalDown
    double downMax = 6000.0;        // never change down into more than this
    double kickRpm = 4500.0, kickMax = 5600.0, loadDownMax = 4800.0;
    double pickRpm = 2500.0, pickMax = 6000.0;  // switching to automatic: gear nearest pickRpm
    // bikes
    double slipper = 1.0;           // share of the clutch capacity that passes engine braking
    double quickshiftTime = 0.0;    // s of ignition cut (up) or blip (down) per clutchless shift; 0 = none
    // quickshift down, rev-matched: blip until the drive torque through the dogs crosses zero (or
    // quickshiftTime runs out), the old gear drops out, the ECU revs the engine to the lower gear's
    // speed, and the new gear goes in once the speeds agree (or after qsMatchTimeout)
    double qsUnloadTorque = 6.0;    // N m at the crank: below this the dogs are free to slide out
    double qsMatchTol = 0.015;      // the new gear engages within this share of its speed
    double qsMatchTimeout = 0.25;   // s between gears at most
    double qsMaxRpm = 1e9;          // a downshift that would land above this is refused (set to the limiter)
    double wheelieAccel = 0.0;      // m/s^2 the wheelie control allows, 0 = none fitted
    // bike gearbox drawing: real tooth counts (0 = derive from the ratio), primary pair, sprocket
    std::array<int, 7> teethIn{}, teethOut{};
    int primaryIn = 30, primaryOut = 54, sprocketTeeth = 16;
};

struct drivetrain {
    drivetrainParams P;
    bool automatic = true;
    bool sport = false;            // automatic sub-mode: hold low gears, rev-matched downshifts
    int gear = 1, targetGear = 1;  // -1 = reverse, 0 = neutral, 1..gears
    int qsPhase = 0;               // quickshift under way: 1 up (ignition cut), -1 down (blip)
    bool dogsFree = false;         // quickshift down: the old gear is out, engine and wheels are apart
    bool coupled() const { return gear != 0 && !dogsFree; }
    bool wheelieControl = true;    // rider setting (V key); only matters when P.wheelieAccel > 0
    bool wheelieOn() const { return wheelieControl && P.wheelieAccel > 0.0; }
    double qsTimer = 0.0;
    bool revMatch = false;         // this shift is a downshift the engine blips the throttle for
    bool toFirst = false;          // this shift is the drop to 1st when (nearly) stopped
    double matchRpm = 0.0;         // engine rpm the target gear needs at the current road speed
    double clutch = 0.0;           // engagement 0..1
    bool clutchPedal = false;      // manual: held = disengaged
    bool brake = false;
    double v = 0.0;                // road speed, m/s
    double omegaIn = 0.0;          // gearbox input shaft, rad/s
    double grade = 0.0;            // rise / run
    double cargo = 0.0;            // extra kg
    double clutchTorque = 0.0, wheelForce = 0.0, accel = 0.0, distance = 0.0;
    int shiftPhase = 0;            // 0 none, 1 clutch opening, 2 clutch closing
    double lastShift = -10.0;
    double inputAngle = 0.0, outputAngle = 0.0;  // for drawing
    std::string message;
    double messageTime = -10.0;

    void reset(bool autoMode) {
        automatic = autoMode;
        gear = targetGear = autoMode ? 1 : 0;
        clutch = 0.0;
        clutchPedal = brake = false;
        v = omegaIn = clutchTorque = wheelForce = accel = distance = 0.0;
        shiftPhase = 0;
        qsPhase = 0;
        qsTimer = 0.0;
        dogsFree = false;
        revMatch = toFirst = false;
        lastShift = -10.0;
        inputAngle = outputAngle = 0.0;
        message.clear();
    }

    double ratio() const { return P.primary * (gear >= 0 ? P.ratios[gear] : -P.reverseRatio); }  // signed, crank to output
    double mass() const { return P.carMass + cargo; }
    double outputOmega() const { return v / P.wheelRadius * P.finalDrive; }  // gearbox output shaft
    double omegaFromWheels() const { return outputOmega() * ratio(); }
    double inputRpm() const { return omegaIn * 60.0 / (2.0 * 3.14159265358979323846); }
    void say(const std::string &m, double now) { message = m; messageTime = now; }

    // shifting: +1 up, -1 down. Manual: a gear only goes in with the clutch pressed, and reverse
    // (below neutral) only with the car stopped. Automatic: Q at a standstill selects R, E goes back to D.
    // sequential bike box: 1 - N - 2 - 3 - ... A full click skips neutral, N selects it from 1st or 2nd.
    // Moving in gear with the clutch home, the quickshifter changes without the lever: an ignition cut
    // to unload the dogs going up, a throttle blip going down.
    // gear that suits the current road speed (1st when stopped)
    int pickGear() const {
        int best = 1;
        double bestErr = 1e9;
        for (int g = 1; g <= P.gears; g++) {
            double r = outputOmega() * P.primary * P.ratios[g] * 60.0 / (2.0 * 3.14159265358979323846);
            if (std::fabs(r - P.pickRpm) < bestErr && r < P.pickMax) { best = g; bestErr = std::fabs(r - P.pickRpm); }
        }
        return best;
    }

    // N toggles neutral from any gear, in automatic and manual. Leaving neutral picks the gear for the
    // road speed (1st when stopped); in manual that needs the clutch lever pulled.
    void bikeShift(int dir, double now, bool toNeutral) {
        if (toNeutral && gear != 0) {
            gear = targetGear = 0;
            qsPhase = shiftPhase = 0;
            dogsFree = false;
            revMatch = toFirst = false;
            say("N", now);
            return;
        }
        if (qsPhase != 0 || shiftPhase != 0) return;
        if (automatic) {
            if (gear != 0) { say("automatic: press T for manual, N for neutral", now); return; }
            gear = targetGear = pickGear();
            lastShift = now;
            if (std::fabs(v) > P.firstGearSpeed) {
                // rolling: blip up to the gear's speed, then close the clutch (the rev-matched shift path)
                clutch = 0.0;
                shiftPhase = 2;
                revMatch = true;
                matchRpm = outputOmega() * P.primary * P.ratios[gear] * 60.0 / (2.0 * 3.14159265358979323846);
            }
            say("D (" + std::to_string(gear) + ")", now);
            return;
        }
        int g;
        if (toNeutral) {
            g = std::fabs(v) < 1.0 ? 1 : pickGear();  // N again: back into gear
        } else if (gear == 0) {
            g = dir > 0 ? 2 : 1;
        } else {
            g = std::clamp(gear + dir, 1, P.gears);
        }
        if (g == gear) return;
        bool moving = std::fabs(v) > 1.0;
        if (P.quickshiftTime > 0.0 && clutch > 0.3 && gear >= 1 && g >= 1 && moving) {
            double landing = outputOmega() * P.primary * P.ratios[g] * 60.0 / (2.0 * 3.14159265358979323846);
            if (g < gear && landing > P.qsMaxRpm) {  // over-rev protection, like the real ECU
                say("downshift refused: " + std::to_string(g) + " would be " +
                        std::to_string(static_cast<int>(std::lround(landing / 100.0) * 100)) + " rpm", now);
                return;
            }
            targetGear = g;
            qsPhase = g > gear ? 1 : -1;
            qsTimer = P.quickshiftTime;
            dogsFree = false;
            matchRpm = outputOmega() * P.primary * P.ratios[g] * 60.0 / (2.0 * 3.14159265358979323846);
            say(std::string("quickshift ") + std::to_string(gear) + " -> " + std::to_string(g) +
                    (g < gear ? " (rev match " + std::to_string(static_cast<int>(std::lround(matchRpm / 100.0) * 100)) + " rpm)" : ""),
                now);
            lastShift = now;
            return;
        }
        if ((gear == 0 || !moving) && clutch > 0.3 && g != 0) {
            say("pull C (clutch) to select a gear", now);
            return;
        }
        gear = targetGear = g;
        say(g == 0 ? "N" : std::to_string(g), now);
    }

    void requestShift(int dir, double now) {
        if (P.bike) { bikeShift(dir, now, false); return; }
        bool stopped = std::fabs(v) < 0.5;
        if (automatic) {
            if (shiftPhase != 0) return;
            if (dir < 0 && gear >= 1) {
                if (!stopped) { say("stop the car to select reverse", now); return; }
                gear = targetGear = -1;
                say("R", now);
            } else if (dir > 0 && gear == -1) {
                if (!stopped) { say("stop the car to select drive", now); return; }
                gear = targetGear = 1;
                say("D", now);
            }
            return;
        }
        int g = std::clamp(gear + dir, -1, P.gears);
        if (g == gear) return;
        if (g != 0 && clutch > 0.3) {
            say("hold C (clutch) to change gear", now);
            return;
        }
        if (g == -1 && !stopped) {
            say("stop the car before selecting reverse", now);
            return;
        }
        gear = targetGear = g;
        if (g == -1) say("R", now);
    }

    void setSport(bool on, double now) {
        sport = on;
        if (automatic) say(on ? "SPORT: holds gears, rev-matched downshifts" : "NORMAL", now);
        else say(on ? "SPORT (active in automatic, press T)" : "NORMAL", now);
    }

    void setAutomatic(bool a, double rpmNow, double now) {
        automatic = a;
        shiftPhase = 0;
        revMatch = toFirst = false;
        if (!a) {
            // hand over in neutral with the clutch where it is, so nothing lurches or stalls
            gear = targetGear = 0;
            clutchPedal = false;
            say(P.bike ? "MANUAL: neutral. Hold C, press Q for 1st, release C gently"
                       : "MANUAL: neutral. Hold C, press E for 1st, release C gently", now);
        } else {
            // pick the gear that puts the engine nearest pickRpm for this road speed
            int best = pickGear();
            if (gear != -1 || v > 0.5) gear = targetGear = best;  // stay in R if reversing
            say(gear == -1 ? "AUTOMATIC (R)" : "AUTOMATIC", now);
        }
        (void)rpmNow;
    }

    void control(double dt, double pedal, double rpmFilt, double now) {
        if (qsPhase > 0) {  // quickshift up: the gear goes in once the ignition cut has unloaded the dogs
            qsTimer -= dt;
            if (qsTimer <= 0.0) {
                gear = targetGear;
                qsPhase = 0;
            }
        } else if (qsPhase < 0) {  // quickshift down, rev-matched
            qsTimer -= dt;
            matchRpm = outputOmega() * P.primary * P.ratios[targetGear] * 60.0 / (2.0 * 3.14159265358979323846);
            if (!dogsFree) {
                // the blip takes the load off the dogs: once the torque through them crosses zero, out
                if (std::fabs(clutchTorque) < P.qsUnloadTorque || qsTimer <= 0.0) {
                    dogsFree = true;
                    qsTimer = P.qsMatchTimeout;
                }
            } else {
                // between gears: in when the input shaft runs at the new gear's speed
                double inRpm = omegaIn * 60.0 / (2.0 * 3.14159265358979323846);
                if (std::fabs(inRpm - matchRpm) < P.qsMatchTol * matchRpm || qsTimer <= 0.0) {
                    gear = targetGear;
                    qsPhase = 0;
                    dogsFree = false;
                }
            }
        }
        if (!automatic) {
            clutch = std::clamp(clutch + (clutchPedal ? -P.clutchPressRate : P.clutchReleaseRate) * dt, 0.0, 1.0);
            return;
        }
        if (shiftPhase != 0 && revMatch) matchRpm = outputOmega() * P.primary * P.ratios[targetGear] * 60.0 / (2.0 * 3.14159265358979323846);
        if (shiftPhase == 1) {  // clutch out, then swap gears
            clutch -= dt / P.shiftOpen;
            if (clutch <= 0.0) {
                clutch = 0.0;
                gear = targetGear;
                shiftPhase = 2;
                // dropping to 1st near a stop: let the take-off clutch logic below bring it back in
                // gently, a full engagement at walking pace would stall the engine
                if (toFirst) shiftPhase = 0, toFirst = false;
            }
            return;
        }
        if (shiftPhase == 2) {  // clutch back in; a rev-matched downshift waits for the blip to get there
            bool waiting = revMatch && rpmFilt < matchRpm - 200.0 && now - lastShift < P.matchHold;
            if (!waiting) clutch += dt / P.shiftClose;
            if (clutch >= 1.0) {
                clutch = 1.0;
                shiftPhase = 0;
                revMatch = false;
            }
            return;
        }
        // automatic clutch: open near idle so the engine can't stall, slip on take-off
        double target;
        // braking to a stop with the throttle shut: open early, in 1st the wheels would drag the engine down
        bool brakingToStop = brake && pedal < 0.03 && std::fabs(v) < P.firstGearSpeed;
        bool launch = false;
        if (rpmFilt < P.openRpm || (pedal < 0.03 && rpmFilt < P.idleOpenRpm) || brakingToStop || gear == 0) target = 0.0;  // open in N
        else if (P.launchPedal > 0.0 && gear <= 2 && launching(rpmFilt)) {
            // launch control: the engine is held at launchRpm (ignition cut, see engineSim) and the
            // clutch closes as far as the revs allow, but never past what would lift the front wheel
            double kRatio = ratio() * P.finalDrive / P.wheelRadius;
            double engMax = wheelieOn() ? std::min(1.0, mass() * P.wheelieAccel / (P.clutchCapacity * kRatio * P.efficiency)) : 1.0;
            // a fixed rpm -> engagement map settles where the clutch carries what the engine makes
            target = std::clamp((rpmFilt - launchRpm(pedal) + 0.5 * P.biteSpan) / P.biteSpan, 0.0, engMax);
            launch = true;
        }
        else if (std::fabs(v) < 3.0 && gear <= 2) target = std::clamp((rpmFilt - P.openRpm - P.launchPedal * pedal) / P.biteSpan, 0.0, 1.0);
        else target = 1.0;
        double rate = target > clutch || launch ? 2.5 : 8.0;  // launch: ease both ways so it doesn't hunt
        clutch += std::clamp(target - clutch, -rate * dt, rate * dt);

        // stopped or nearly: go straight to 1st (skipping the gears between), ready to pull away
        if (gear > 1 && std::fabs(v) < P.firstGearSpeed) {
            say("downshift " + std::to_string(gear) + " -> 1 (slow)", now);
            lastShift = now;
            revMatch = false;
            if (clutch < 0.05) {
                gear = targetGear = 1;  // clutch already open: just select it
            } else {
                targetGear = 1;
                toFirst = true;
                shiftPhase = 1;
            }
            return;
        }

        // shift schedule
        if (gear < 1 || now - lastShift < 1.0 || v < 2.0 || clutch < 0.99) return;  // no shifting in R
        auto rpmIn = [&](int g) { return rpmFilt * P.ratios[g] / P.ratios[gear]; };
        double up = P.upBase + P.upPedal * pedal, down = P.normalDown;
        // asking for power but losing speed (hill, cargo): change down; don't change up while not gaining
        bool losing = pedal > 0.3 && accel < -0.1;
        bool notGaining = pedal > 0.3 && accel < 0.05;
        int want = gear;
        const char *why = "";
        bool slow = std::fabs(v) < P.firstGearSpeed + 1.0;  // no upshift until clear of the drop-to-1st speed
        if (sport) {
            // keep the engine in the lowest gear that stays clear of the limiter
            if (gear < P.gears && rpmFilt > P.sportUp && !slow) { want = gear + 1; why = "upshift"; }
            else if (gear > 1 && rpmIn(gear - 1) < P.sportDown) { want = gear - 1; why = "downshift"; }
        } else if (gear < P.gears && rpmFilt > up && rpmIn(gear + 1) > down + P.upMargin && !notGaining && !slow) { want = gear + 1; why = "upshift"; }
        else if (gear > 1 && rpmFilt < down && rpmIn(gear - 1) < P.downMax) { want = gear - 1; why = "downshift"; }
        else if (gear > 1 && pedal > 0.85 && rpmFilt < P.kickRpm && rpmIn(gear - 1) < P.kickMax) { want = gear - 1; why = "kickdown"; }
        else if (gear > 1 && losing && rpmIn(gear - 1) < P.loadDownMax) { want = gear - 1; why = "downshift (load)"; }
        if (want != gear) {
            targetGear = want;
            shiftPhase = 1;
            lastShift = now;
            revMatch = sport && want < gear;
            matchRpm = rpmIn(want);
            say(std::string(why) + (revMatch ? " (rev match) " : " ") + std::to_string(gear) + " -> " + std::to_string(want), now);
        }
    }

    // bikes with launch control keep slipping the clutch until the engine and wheels meet, instead of
    // closing it at walking pace and dragging a light crank out of its power band
    double launchRpm(double pedal) const { return P.openRpm + P.launchPedal * pedal + 0.5 * P.biteSpan; }
    bool launching(double rpmEngine) const {
        double wheelRpm = omegaFromWheels() * 60.0 / (2.0 * 3.14159265358979323846);
        return P.launchPedal > 0.0 && gear >= 1 && std::fabs(v) < 25.0 && rpmEngine - wheelRpm > 300.0;
    }

    // torque the clutch puts on the engine (positive = load on the engine)
    double clutchTorqueFor(double omegaEngine, double engineInertia, double dt) {
        double side = coupled() ? omegaFromWheels() : omegaIn;
        double kRatio = coupled() ? ratio() * P.finalDrive / P.wheelRadius : 0.0;
        double sideInertia = coupled() ? mass() / (kRatio * kRatio) + P.inputInertia : P.inputInertia;
        // stiff but explicit-stable: the slip time constant stays a few steps long
        double k = std::min(P.clutchStiffness, 0.4 * std::min(engineInertia, sideInertia) / dt);
        double cap = clutch * P.clutchCapacity;
        clutchTorque = std::clamp(k * (omegaEngine - side), -cap * P.slipper, cap);  // a slipper clutch slips on overrun
        return clutchTorque;
    }

    void integrate(double dt, double pedal) {
        const double g = 9.81;
        double th = std::atan(grade), m = mass();
        double kRatio = ratio() * P.finalDrive / P.wheelRadius;
        wheelForce = coupled() ? clutchTorque * kRatio * P.efficiency : 0.0;
        double resist = P.crr * m * g * std::cos(th) * std::tanh(v / 0.3) +
                        0.5 * P.airDensity * P.cdA * v * std::fabs(v) + m * g * std::sin(th);
        double mEff = m + P.wheelInertia / (P.wheelRadius * P.wheelRadius) +
                      (coupled() ? P.inputInertia * kRatio * kRatio : 0.0);
        // brake, plus hill-hold in automatic when stopped with the throttle closed
        bool hold = brake || (automatic && pedal < 0.03 && std::fabs(v) < 0.5);
        double brakeMax = hold ? m * P.brakeDecel : 0.0;
        double fNet = wheelForce - resist;
        double a;
        if (brakeMax > 0.0 && std::fabs(v) < 0.05 && std::fabs(fNet) <= brakeMax) {
            v = 0.0;  // brakes hold the car still
            a = 0.0;
        } else {
            fNet -= brakeMax * std::tanh(v / 0.05);
            a = fNet / mEff;
            v += a * dt;
        }
        accel += (a - accel) * std::min(1.0, dt / 0.5);
        if (!coupled()) omegaIn += (clutchTorque - 0.3 * std::tanh(omegaIn)) / P.inputInertia * dt;
        else omegaIn = omegaFromWheels();
        inputAngle += omegaIn * dt;
        outputAngle += outputOmega() * dt;
        distance += v * dt;
    }
};

struct engineSim {
    engineParams P;
    std::array<cylinderState, 4> cyl;
    std::array<double, 4> lag{};   // firing lag of each cylinder behind cylinder 1, rad
    std::array<double, 4> bankCos{1, 1, 1, 1}, bankSin{};  // bore direction: up = cos, sideways = sin
    bool standalone = false;       // engine on a stand: no clutch, gearbox or vehicle (--v4 --engine-only)
    bool qsCut = false;            // quickshifter ignition cut
    double tcTrim = 1.0;           // wheelie control: share of the fuel the ECU lets burn
    double wheelForceFilt = 0.0;   // N, smoothed over a few firings for the wheelie control
    double theta = 0.0, omega = 0.0;
    double pMan = 101325.0;
    double idleArea = 0.0, idleInt = 0.0;
    double pedal = 0.0, pedalTarget = 0.0;
    bool ignition = true, starter = false, revCut = false;
    bool fuelCut = false;     // overrun fuel cut: throttle shut while the wheels drive the engine
    bool stalled = true;      // ECU has lost the engine: no fuel below 120 rpm unless cranking (or bump-started past 300)
    drivetrain drive;
    bool idleActive = false;  // idle speed control engaged
    double rpmFilt = 0.0;     // rpm averaged over ~a firing interval, what the ECU controls on
    bool lockSpeed = false;  // dyno sweep: hold omega fixed
    double vDispTotal = 0.0;
    // outputs
    double gasTorque = 0.0, frictionTorque = 0.0, starterTorque = 0.0, loadTorque = 0.0;
    double brakeAvg = 0.0;   // smoothed gas - friction torque
    double time = 0.0;
    double alpha = 0.0;      // crank angular acceleration this step
    double alphaFilt = 0.0;  // ... averaged over ~a firing interval (the quickshift blip predicts with it)
    double throttleFlow = 0.0;  // kg/s through throttle + idle valve (intake hiss)
    double throttleOpen = 0.0;  // throttle plate opening 0..1 set by the drive-by-wire
    double blip = 0.0;          // rev-match throttle the gearbox asks for on a sport downshift, 0..1

    // vibration: forces the moving parts put into the block
    double shakeForce = 0.0;   // N, vertical, from piston accelerations
    double rollMoment = 0.0;   // N m about the crank axis, reaction to the crank torque
    double pitchMoment = 0.0;  // N m, vertical forces times their position along the crank
    double h1c = 0, h1s = 0, h2c = 0, h2s = 0;   // 1st / 2nd order components (running Fourier)
    double rollMean = 0, rollVar = 0, pitchVar = 0;
    double mountY = 0, mountVy = 0, mountRoll = 0, mountVr = 0;  // block motion on its mounts

    // cylinder 1 p-V loop, one sample per crank degree over the last 720 degrees
    std::array<float, 720> pvV{}, pvP{};
    int pvLast = -1;

    double shake1() const { return 2.0 * std::hypot(h1c, h1s); }
    double shake2() const { return 2.0 * std::hypot(h2c, h2s); }
    double rollAmp() const { return std::sqrt(2.0 * rollVar); }
    double pitchAmp() const { return std::sqrt(2.0 * pitchVar); }

    void init(const engineModel &e, double theta1) {
        for (int c = 0; c < 4; c++) {
            cylinderState &s = cyl[c];
            s.r = e.cyls[c].throw_ * 1e-3;
            s.L = e.cyls[c].rodL * 1e-3;
            s.area = 0.25 * 3.14159265358979323846 * P.bore * P.bore;
            s.vDisp = s.area * 2.0 * s.r;
            s.vClear = s.vDisp / (P.compression - 1.0);
            s.z = e.cyls[c].zCentre * 1e-3;
            lag[c] = e.cyls[c].firingLagDeg * DEGR;  // exact in double, like the old constant table
            bankCos[c] = std::cos(e.cyls[c].bank);
            bankSin[c] = std::sin(e.cyls[c].bank);
        }
        vDispTotal = 0.0;
        for (auto &s : cyl) vDispTotal += s.vDisp;
        reset(theta1);
    }

    void reset(double theta1) {
        theta = wrapCycle(theta1);
        omega = 0.0;
        pMan = P.pAtm;
        idleArea = P.idleBase;
        idleInt = 0.0;
        rpmFilt = 0.0;
        pedal = pedalTarget = 0.0;
        revCut = false;
        stalled = true;
        qsCut = false;
        tcTrim = 1.0;
        wheelForceFilt = 0.0;
        brakeAvg = 0.0;
        time = 0.0;
        alpha = throttleFlow = 0.0;
        shakeForce = rollMoment = pitchMoment = 0.0;
        h1c = h1s = h2c = h2s = rollMean = rollVar = pitchVar = 0.0;
        mountY = mountVy = mountRoll = mountVr = 0.0;
        for (int d = 0; d < 720; d++) {
            pvV[d] = static_cast<float>(volume(cyl[0], d * DEGR));
            pvP[d] = static_cast<float>(P.pAtm);
        }
        pvLast = -1;
        drive.reset(drive.automatic);
        for (int c = 0; c < 4; c++) {
            cylinderState &s = cyl[c];
            double phi = phase(c);
            s.region = regionOf(phi);
            s.p = P.pAtm;
            s.mAir = s.region == 1 ? trappedAir(s, 0.0) : 0.0;
            s.qTot = s.xbDone = s.heatRate = s.glowEvo = 0.0;
            // a cylinder already past its spark point has nothing left to burn
            s.sparked = s.region == 1 && phi > 340.0 * DEGR;
        }
    }

    double rpm() const { return omega * 60.0 / (2.0 * 3.14159265358979323846); }
    double phase(int c) const { return wrapCycle(theta - lag[c]); }
    int regionOf(double phi) const { return phi < P.ivc ? 0 : (phi < P.evo ? 1 : 2); }

    // slider-crank: piston travel from TDC and its first two derivatives with crank angle
    static void geom(double phi, double r, double L, double &x, double &dx, double &d2x) {
        double s = std::sin(phi), c = std::cos(phi);
        double k = std::sqrt(L * L - r * r * s * s);
        x = r * (1.0 - c) + L - k;
        dx = r * s * (1.0 + r * c / k);
        d2x = r * c + r * r * (c * c - s * s) / k + r * r * r * r * s * s * c * c / (k * k * k);
    }

    double volume(const cylinderState &s, double phi) const {
        double x, dx, d2x;
        geom(phi, s.r, s.L, x, dx, d2x);
        return s.vClear + s.area * x;
    }

    double volEffHigh(double r) const { return P.veBase + P.vePeak * std::exp(-std::pow((r - P.veRpm) / P.veWidth, 2.0)); }
    double volEffLow(double r) const {
        return P.veLowBase + P.veLowPeak * std::exp(-std::pow((r - P.veLowRpm) / P.veLowWidth, 2.0));
    }
    // ShiftCam: the partial-load lobe while it breathes better (low and mid rpm), then the race lobe
    bool onLowCam(double r) const { return P.lowCam && volEffLow(r) > volEffHigh(r); }
    double volEff(double r) const { return onLowCam(r) ? volEffLow(r) : volEffHigh(r); }

    double trappedAir(const cylinderState &s, double r) const {
        return volEff(r) * pMan * s.vDisp / (P.rGas * P.tIntake);
    }

    double sparkAdvance(double r) const {
        if (r < 400.0) return 5.0 * DEGR;
        // at idle the ECU trims spark around a retarded base: slower rpm -> more advance -> more torque
        if (idleActive)
            return std::clamp(P.idleSparkBase + P.idleSparkGain * (P.idleRpm - rpmFilt), -5.0, 25.0) * DEGR;
        return (P.sparkLow + (P.sparkHigh - P.sparkLow) * std::clamp((r - P.sparkFrom) / P.sparkSpan, 0.0, 1.0)) * DEGR;
    }

    // residual-gas dilution makes low-load combustion less effective (and is why idle needs ~30 kPa)
    double loadFactor() const { return P.residualMin + (1.0 - P.residualMin) * std::clamp((pMan - 25e3) / 60e3, 0.0, 1.0); }

    // compressible flow function through the throttle, pr = downstream / upstream
    static double flowFunction(double pr) {
        const double g = 1.4;
        pr = std::clamp(pr, 0.0, 1.0);
        if (pr < 0.528) pr = 0.528;  // choked
        return std::sqrt(2.0 * g / (g - 1.0) * (std::pow(pr, 2.0 / g) - std::pow(pr, (g + 1.0) / g)));
    }

    // advance one cylinder's gas state from phiOld to phiNew (dphi is the signed change)
    void advanceCylinder(cylinderState &s, double phiOld, double phiNew, double dphi, double dt, double r) {
        int region = regionOf(phiNew);
        s.heatRate = 0.0;
        if (region != s.region) {
            bool forward = dphi > 0.0;
            if (region == 0) {
                s.p = pMan;
            } else if (region == 1) {
                s.qTot = s.xbDone = 0.0;
                if (forward) {  // intake valve just closed: trap this cycle's charge
                    s.p = pMan;
                    s.mAir = trappedAir(s, r);
                    s.sparked = false;
                } else {        // turned backwards out of the exhaust stroke
                    s.p = P.pExhaust;
                    s.mAir = 0.0;
                    s.sparked = true;
                }
            } else if (forward) {   // exhaust valve just opened: remember how hot the gas was
                s.glowEvo = burnGlow(s, phiOld);
            } else {                // backwards out of intake into exhaust
                s.p = P.pExhaust;
                s.glowEvo = 0.0;
            }
            s.region = region;
            return;
        }
        if (region == 0) {
            s.p = pMan;
        } else if (region == 1) {
            s.p *= std::pow(volume(s, phiOld) / volume(s, phiNew), P.gamma);
            double phiS = 360.0 * DEGR - sparkAdvance(r);
            if (!s.sparked && dphi > 0.0 && phiOld < phiS && phiNew >= phiS) {
                s.sparked = true;
                s.phiSpark = phiS;
                s.xbDone = 0.0;
                if (ignition && !revCut && !fuelCut && !stalled && !qsCut && s.mAir > 0.0)
                    s.qTot = tcTrim * P.etaHeat * loadFactor() * s.mAir / P.afr * P.lhv;
            }
            if (s.qTot > 0.0 && dphi > 0.0) {
                double u = std::clamp((phiNew - s.phiSpark) / P.burn, 0.0, 1.0);
                double xb = 1.0 - std::exp(-P.wiebeA * std::pow(u, P.wiebeM + 1.0));
                double dq = s.qTot * (xb - s.xbDone);
                if (dq > 0.0) {
                    s.p += (P.gamma - 1.0) / volume(s, phiNew) * dq;
                    s.xbDone = xb;
                    if (dt > 0.0) s.heatRate = dq / dt;
                }
            }
            if (dt > 0.0) s.p = P.pCrankcase + (s.p - P.pCrankcase) * std::exp(-dt / P.leakTau);
        } else {
            s.p = P.pExhaust + (s.p - P.pExhaust) * std::exp(-std::fabs(dphi) / P.blowdown);
        }
    }

    // 0..1 glow from the burned gas temperature T = pV / (m R): ~800 K dark, ~2000 K full
    double burnGlow(const cylinderState &s, double phi) const {
        if (s.qTot <= 0.0 || s.mAir <= 0.0 || phi < s.phiSpark) return 0.0;
        double t = s.p * volume(s, phi) / (s.mAir * (1.0 + 1.0 / P.afr) * P.rGas);
        return std::clamp((t - 800.0) / 1200.0, 0.0, 1.0);
    }

    double glow(int c) const {
        const cylinderState &s = cyl[c];
        double phi = phase(c);
        if (s.region == 1) return burnGlow(s, phi);
        if (s.region == 2 && s.qTot > 0.0) return s.glowEvo * std::exp(-(phi - P.evo) / (15.0 * DEGR));
        return 0.0;
    }

    // move the crank by dphi without dynamics (paused single-step), keeping the gas states consistent
    void nudge(double dphi) {
        double old = theta;
        theta = wrapCycle(theta + dphi);
        for (int c = 0; c < 4; c++)
            advanceCylinder(cyl[c], wrapCycle(old - lag[c]), phase(c), dphi, 0.0, rpm());
    }

    void step(double dt) {
        double r = rpm();
        time += dt;

        pedal += std::clamp(pedalTarget - pedal, -P.pedalRate * dt, P.pedalRate * dt);
        if (r > P.revLimit) revCut = true;
        if (r < P.revResume) revCut = false;

        // idle air PI: holds idle rpm with the throttle closed once the engine has started
        rpmFilt += (r - rpmFilt) * std::min(1.0, dt / 0.1);
        if (!standalone) drive.control(dt, pedal, rpmFilt, time);
        if (std::fabs(r) < 120.0 && !starter) stalled = true;
        if (starter || std::fabs(r) > 300.0) stalled = false;
        // rev-match blip: once the clutch is mostly open, drive the engine up to the lower gear's rpm
        if (drive.revMatch && drive.shiftPhase != 0 && drive.clutch < 0.4)
            blip = std::clamp((drive.matchRpm + 150.0 - r) / 1000.0, 0.0, 1.0);
        else if (drive.qsPhase < 0) {
            // quickshift down: blip to unload the dogs, then rev to the lower gear's speed. A light crank
            // spins up fast, so the throttle eases off early to land on the target instead of past it
            // (on the averaged acceleration: each firing alone kicks the crank far harder)
            double lead = std::max(0.0, alphaFilt) * 60.0 / (2.0 * 3.14159265358979323846) * 0.02;  // rpm the next 20 ms add
            blip = std::clamp((drive.matchRpm - r - lead) / 800.0 + 0.05, 0.0, 1.0);
        } else
            blip = 0.0;
        double demand = std::max(pedal, blip);
        // overrun fuel cut only while the wheels really drive the engine (not between gears)
        fuelCut = !standalone && drive.coupled() && drive.clutch > 0.9 && demand < 0.02 && rpmFilt > P.fuelCutRpm;
        qsCut = !standalone && drive.qsPhase > 0;
        // launch control holds the revs while the clutch slips (and the plain bite logic when not)
        if (!standalone && drive.automatic && drive.P.launchPedal > 0.0 && drive.launching(r) && r > drive.launchRpm(demand) + 300.0)
            qsCut = true;
        // wheelie control: trim the fuel while the drive force would lift the front wheel
        // (the smoothed drive force is also what the HUD shows)
        if (!standalone) wheelForceFilt += (drive.wheelForce - wheelForceFilt) * std::min(1.0, dt / 0.05);
        if (!drive.wheelieOn()) tcTrim = 1.0;
        else if (!standalone && std::fabs(omega - drive.omegaIn) < 20.0) {  // clutch locked
            double lift = drive.mass() * drive.P.wheelieAccel;
            // proportional: trims down fast when the front would lift, eases back as it settles
            tcTrim = std::clamp(tcTrim + 4.0 * dt * (1.0 - wheelForceFilt / lift), 0.3, 1.0);
        }
        idleActive = ignition && demand < 0.02 && r > 400.0;
        if (idleActive) {
            double err = P.idleRpm - rpmFilt;
            // anti-windup: stop integrating while the valve is pinned at a limit in the same direction
            double unclamped = P.idleBase + P.idleKp * err + P.idleKi * idleInt;
            bool pinned = (unclamped >= P.idleMax && err > 0.0) || (unclamped <= P.idleMin && err < 0.0);
            if (!pinned && std::fabs(err) < P.idleBand) idleInt += err * dt;
            idleArea = std::clamp(P.idleBase + P.idleKp * err + P.idleKi * idleInt, P.idleMin, P.idleMax);
        } else if (!ignition || r <= 400.0) {
            // stopped or cranking: park the valve at its start position
            idleInt = 0.0;
            idleArea = P.idleBase;
        }

        // torques at the current angle
        double inertia = P.inertia, dInertia = 0.0;
        double dxs[4], d2xs[4];
        gasTorque = 0.0;
        for (int c = 0; c < 4; c++) {
            cylinderState &s = cyl[c];
            double x, dx, d2x;
            geom(phase(c), s.r, s.L, x, dx, d2x);
            dxs[c] = dx;
            d2xs[c] = d2x;
            gasTorque += (s.p - P.pCrankcase) * s.area * dx;
            inertia += P.mRecip * dx * dx;
            dInertia += 2.0 * P.mRecip * dx * d2x;
        }
        double fmep = P.fmepA + P.fmepB * std::fabs(r) / 1000.0 + P.fmepC * r * r / 1e6;
        frictionTorque = fmep * vDispTotal / (4.0 * 3.14159265358979323846) * std::tanh(omega / 1.0);
        starterTorque = starter ? P.starterTorque * std::clamp(1.0 - r / P.starterMaxRpm, 0.0, 1.0) : 0.0;
        loadTorque = standalone ? 0.0 : drive.clutchTorqueFor(omega, inertia, dt);  // what the clutch takes from the crank

        // Lagrange with angle-dependent inertia: I(theta) w' + 1/2 I'(theta) w^2 = sum of torques
        alpha = 0.0;
        if (!lockSpeed) {
            alpha = (gasTorque - frictionTorque + starterTorque - loadTorque - 0.5 * dInertia * omega * omega) / inertia;
            omega += alpha * dt;
        }
        alphaFilt += (alpha - alphaFilt) * std::min(1.0, dt / 0.015);

        // shaking: each piston needs m*a along its bore, the block supplies it and feels the reaction.
        // The block also feels the reaction of the torque the pistons put into the crank.
        {
            double fUp = 0.0, recipTorque = 0.0, pitch = 0.0;
            for (int c = 0; c < 4; c++) {
                double a = d2xs[c] * omega * omega + dxs[c] * alpha;  // piston acceleration away from TDC
                fUp += P.mRecip * a * bankCos[c];  // vertical part (a V engine's bores lean)
                // counterweight opposite this cylinder's crankpin (pin angle in mesh space = phase + bank)
                double pinAngle = phase(c) + std::atan2(bankSin[c], bankCos[c]);
                fUp -= P.counterweight * P.mRecip * cyl[c].r * omega * omega * std::cos(pinAngle);
                recipTorque += P.mRecip * a * dxs[c];
                pitch += P.mRecip * a * bankCos[c] * cyl[c].z;
            }
            shakeForce = fUp;
            rollMoment = -(gasTorque - recipTorque);
            pitchMoment = pitch;
            double k = std::min(1.0, dt / 0.3);
            double psi = phase(0);  // cylinder 1 crank angle; cos/sin are 360-degree periodic
            h1c += (fUp * std::cos(psi) - h1c) * k;
            h1s += (fUp * std::sin(psi) - h1s) * k;
            h2c += (fUp * std::cos(2.0 * psi) - h2c) * k;
            h2s += (fUp * std::sin(2.0 * psi) - h2s) * k;
            rollMean += (rollMoment - rollMean) * k;
            rollVar += ((rollMoment - rollMean) * (rollMoment - rollMean) - rollVar) * k;
            pitchVar += (pitch * pitch - pitchVar) * k;

            // block on its mounts: vertical and roll spring-damper, semi-implicit Euler
            const double TWO_PI = 2.0 * 3.14159265358979323846;
            double kY = P.blockMass * std::pow(TWO_PI * P.mountFreqY, 2.0);
            double cY = 2.0 * P.mountZeta * std::sqrt(kY * P.blockMass);
            mountVy += (fUp - kY * mountY - cY * mountVy) / P.blockMass * dt;
            mountY += mountVy * dt;
            double kR = P.blockRollInertia * std::pow(TWO_PI * P.mountFreqRoll, 2.0);
            double cR = 2.0 * P.mountZeta * std::sqrt(kR * P.blockRollInertia);
            mountVr += (rollMoment - rollMean - kR * mountRoll - cR * mountVr) / P.blockRollInertia * dt;
            mountRoll += mountVr * dt;
        }
        double dphi = omega * dt;
        double old = theta;
        theta = wrapCycle(theta + dphi);
        for (int c = 0; c < 4; c++)
            advanceCylinder(cyl[c], wrapCycle(old - lag[c]), phase(c), dphi, dt, r);

        // intake manifold filling: throttle + idle valve in, cylinders out
        // drive-by-wire: the pedal asks for a manifold pressure (so torque follows the pedal roughly
        // linearly at any rpm) and the throttle opens to whatever area passes that flow at this rpm,
        // inverting the flow model below. Full pedal is always wide open.
        double pWant = P.mapMin + demand * (P.pAtm - P.mapMin);
        double flowWant = volEff(r) * pWant * vDispTotal * std::fabs(omega) / (4.0 * 3.14159265358979323846) / (P.rGas * P.tIntake);
        double flux = P.throttleCd * P.pAtm / std::sqrt(P.rGas * P.tIntake) * flowFunction(std::min(pWant / P.pAtm, 0.995));
        double etc = demand > 0.001 ? std::min(P.throttleArea, flowWant / flux) : 0.0;
        throttleOpen = std::max(etc, P.throttleArea * demand * demand * demand) / P.throttleArea;
        double area = P.throttleArea * throttleOpen + idleArea;
        double mIn = P.throttleCd * area * P.pAtm / std::sqrt(P.rGas * P.tIntake) * flowFunction(pMan / P.pAtm);
        double mOut = volEff(r) * pMan * vDispTotal * std::fabs(omega) / (4.0 * 3.14159265358979323846) / (P.rGas * P.tIntake);
        pMan = std::clamp(pMan + P.rGas * P.tIntake / P.vManifold * (mIn - mOut) * dt, 5000.0, P.pAtm);
        throttleFlow = mIn;
        if (!standalone) drive.integrate(dt, pedal);

        // record cylinder 1 for the p-V diagram, filling any degrees skipped this step
        int idx = std::min(719, static_cast<int>(phase(0) / DEGR));
        float vNow = static_cast<float>(volume(cyl[0], phase(0))), pNow = static_cast<float>(cyl[0].p);
        int gap = pvLast < 0 ? 0 : (idx - pvLast + 720) % 720;
        if (dphi > 0.0 && gap > 0 && gap < 60) {
            for (int d = 1; d <= gap; d++) {
                int j = (pvLast + d) % 720;
                pvV[j] = static_cast<float>(volume(cyl[0], j * DEGR));
                pvP[j] = pNow;
            }
        }
        pvV[idx] = vNow;
        pvP[idx] = pNow;
        pvLast = idx;

        brakeAvg += (gasTorque - frictionTorque - brakeAvg) * std::min(1.0, dt / 0.25);
    }

    // stroke name for cylinder c
    const char *stroke(int c) const {
        double phi = phase(c) / DEGR;
        if (phi < 180.0) return "INTAKE";
        if (phi < 360.0) return "COMP";
        if (phi < 540.0) return "POWER";
        return "EXHAUST";
    }
};

const double PHYS_DT = 1.0 / 40000.0;

