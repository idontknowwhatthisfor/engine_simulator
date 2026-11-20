// engine sound: synthesised from the simulation at the physics rate, streamed to the sound card
#pragma once
#include "physics.hpp"

// ---------------------------------------------------------------------------------------------
// engine sound: synthesised from the simulation at the physics rate
// ---------------------------------------------------------------------------------------------
// The main voice is exhaust blowdown: when an exhaust valve opens, gas rushes out at a rate that
// grows with the cylinder pressure, giving one pulse per firing (half the cylinder count per revolution). Those pulses
// go through a crude exhaust pipe (resonance + low-pass) with flow turbulence on top. Intake
// pulsing, throttle hiss, combustion clatter and the starter's gear whine are mixed in quietly.

struct biquad {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    enum type { LOWPASS, HIGHPASS, BANDPASS };
    // RBJ audio EQ cookbook; BANDPASS has 0 dB peak gain
    static biquad make(type t, double fs, double f0, double q) {
        double w0 = 2.0 * 3.14159265358979323846 * f0 / fs, cs = std::cos(w0), al = std::sin(w0) / (2.0 * q);
        biquad f;
        double a0 = 1.0 + al;
        if (t == LOWPASS) { f.b0 = (1 - cs) / 2; f.b1 = 1 - cs; f.b2 = (1 - cs) / 2; }
        if (t == HIGHPASS) { f.b0 = (1 + cs) / 2; f.b1 = -(1 + cs); f.b2 = (1 + cs) / 2; }
        if (t == BANDPASS) { f.b0 = al; f.b1 = 0; f.b2 = -al; }
        f.a1 = -2.0 * cs;
        f.a2 = 1.0 - al;
        f.b0 /= a0; f.b1 /= a0; f.b2 /= a0; f.a1 /= a0; f.a2 /= a0;
        return f;
    }
    double run(double x) {
        double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

// The exhaust signal is a train of positive pulses, so it carries a DC offset that a high-pass takes
// out. Its corner must stay well under the slowest firing rate the engine makes (idle) or it eats the
// fundamental and reshapes each pulse: 0.6 of the idle firing frequency, where the fundamental loses
// about 0.5 dB, capped at 15 Hz (what every engine used before; all of them idle at 26.7 Hz or more,
// so they still get exactly 15) and floored at 1 Hz. A single idling at 800 rpm fires at 6.7 Hz and
// gets 4 Hz. soundParams::dcBlockHz > 0 sets it by hand.
double dcBlockCutoff(const soundParams &sp, double idleRpm, int cylinders) {
    if (sp.dcBlockHz > 0.0) return sp.dcBlockHz;
    double idleFiringHz = idleRpm / 60.0 * cylinders / 2.0;  // one firing per cylinder per two turns
    return std::clamp(0.6 * idleFiringHz, 1.0, 15.0);
}

struct engineSound {
    double fs;
    soundParams sp;
    biquad dcBlock, pipeLow, pipeRes, turbulence, intake, clatter, master;
    std::uint32_t rng = 0x12345678u;
    double gearPhase = 0.0, armaturePhase = 0.0;
    // the detailed voice's stages (soundParams::detailed)
    std::vector<double> pipeBuf;      // round trip of the exhaust pipe
    std::size_t pipePos = 0;
    double pipeFrac = 0.0, pipeDampZ = 0.0, pipeDampA = 0.0;
    biquad flapShut, flapOpen, airbox, howl, popBand, popThump, whineBand, burbleLow;
    double flap = 0.0;                // 0 shut .. 1 open, follows rpm and throttle with a servo lag
    double lastTube = 0.0;
    cylArray<int> lastRegion;         // per cylinder, to spot each exhaust valve opening (-1: not seen yet)
    double popEnv = 0.0, popAmp = 0.0, tickEnv = 0.0;
    int popCount = 0;                 // crackles so far (the exhaust draws a flame and sparks for each)
    double popLastAmp = 0.0;
    biquad tickBand;
    double sinceLift = 1e9;           // s since the throttle was snapped shut
    bool lastQs = false;
    double primaryPhase = 0.0, gearMeshPhase = 0.0;
    // turbo: compressor whistle, intake whoosh, blow-off valve, surge flutter
    biquad whoosh, bovBand, bovHigh, flutterBand, flutterThump;
    double whistlePhase = 0.0, splitterPhase = 0.0;

    // for this simulation's engine (its sound parameters, idle speed and cylinder count)
    engineSound(double sampleRate, const engineSim &sim) : fs(sampleRate), sp(sim.P.sound) {
        dcBlock = biquad::make(biquad::HIGHPASS, fs, dcBlockCutoff(sp, sim.P.idleRpm, sim.cylCount()), 0.707);
        pipeLow = biquad::make(biquad::LOWPASS, fs, sp.pipeLowHz, 0.9);
        pipeRes = biquad::make(biquad::BANDPASS, fs, sp.pipeResHz, 3.0);
        turbulence = biquad::make(biquad::BANDPASS, fs, 1200.0, 0.8);
        intake = biquad::make(biquad::BANDPASS, fs, 350.0, 1.5);
        clatter = biquad::make(biquad::HIGHPASS, fs, 2500.0, 0.707);
        master = biquad::make(biquad::LOWPASS, fs, sp.masterHz, 0.707);
        whoosh = biquad::make(biquad::BANDPASS, fs, 2500.0, 0.8);
        bovBand = biquad::make(biquad::BANDPASS, fs, 3200.0, 0.6);
        bovHigh = biquad::make(biquad::HIGHPASS, fs, 1200.0, 0.707);
        flutterBand = biquad::make(biquad::BANDPASS, fs, 900.0, 1.2);
        flutterThump = biquad::make(biquad::LOWPASS, fs, 160.0, 0.707);
        if (sp.detailed) {
            // the quarter-wave pipe: one round trip (2L / c) of delay with an inverting, lossy
            // reflection at the open end
            double delay = 2.0 * sp.pipeLength / sp.gasSpeed * fs;
            pipeBuf.assign(static_cast<std::size_t>(delay) + 2, 0.0);
            pipeFrac = delay - std::floor(delay);
            pipeDampA = std::exp(-2.0 * 3.14159265358979323846 * sp.pipeDampHz / fs);
            flapShut = biquad::make(biquad::LOWPASS, fs, sp.flapShutHz, 0.707);
            flapOpen = biquad::make(biquad::LOWPASS, fs, sp.flapOpenHz, 0.707);
            airbox = biquad::make(biquad::BANDPASS, fs, sp.airboxHz, 2.0);
            howl = biquad::make(biquad::BANDPASS, fs, sp.intakeHowlHz, 1.2);
            popBand = biquad::make(biquad::BANDPASS, fs, 1100.0, 0.9);
            popThump = biquad::make(biquad::LOWPASS, fs, 180.0, 0.707);
            whineBand = biquad::make(biquad::HIGHPASS, fs, 400.0, 0.707);
            burbleLow = biquad::make(biquad::LOWPASS, fs, 300.0, 0.707);
            tickBand = biquad::make(biquad::BANDPASS, fs, sp.tickHz, 1.5);
        }
    }

    double noise() {  // xorshift32, uniform in [-1, 1]
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return rng / 2147483648.0 - 1.0;
    }

    // quarter-wave exhaust pipe: y = x - r * damped(y delayed by one round trip)
    double pipe(double x) {
        // the write head walks backwards, so the sample written k steps ago sits at pipePos + k
        std::size_t n = pipeBuf.size(), d = n - 2;  // whole samples of the round trip
        double old = pipeBuf[(pipePos + d) % n] * (1.0 - pipeFrac) + pipeBuf[(pipePos + d + 1) % n] * pipeFrac;
        pipeDampZ = (1.0 - pipeDampA) * old + pipeDampA * pipeDampZ;
        double y = x - sp.pipeReflect * pipeDampZ;
        pipeBuf[pipePos] = y;
        pipePos = (pipePos + n - 1) % n;
        return y;
    }

    // one output sample for the current simulation state; call once per physics step
    float sample(const engineSim &sim) {
        const double TWO_PI = 2.0 * 3.14159265358979323846;
        double exhaust = 0.0, intakeFlow = 0.0, heat = 0.0;
        int opened = 0;  // exhaust valves that opened this sample
        if (sp.detailed && lastRegion.size() != sim.cyl.size()) lastRegion.assign(sim.cyl.size(), -1);
        for (int c = 0; c < sim.cylCount(); c++) {
            const cylinderState &s = sim.cyl[c];
            double x, dx, d2x;
            engineSim::geom(sim.phase(c), s.r, s.L, x, dx, d2x);
            double pistonSpeed = dx * sim.omega / 15.0;  // relative to a brisk 15 m/s
            if (s.region == 2) {
                // blowdown (signed: an over-expanded cylinder at light load sucks back a weaker pulse)
                // plus the gas the piston pushes out on the exhaust stroke
                double dp = (s.p - sim.pExh) / 1e5;
                // the displaced gas is scaled by how full the cylinder was (tiny charge on overrun)
                double fullCharge = sim.P.pAtm * s.vDisp / (sim.P.rGas * sim.P.tIntake);
                double charge = std::sqrt(std::clamp(s.mAir / fullCharge, 0.0, 1.0));
                double blow = (dp >= 0.0 ? 1.0 : -0.5) * std::sqrt(std::fabs(dp));
                if (sp.detailed) {
                    // the valve takes ~30 deg to lift: the blowdown swells in instead of stepping
                    // (a step would radiate as a click on every opening)
                    double since = std::fmod(sim.phase(c) - sim.P.evo + 4.0 * CYCLE, CYCLE) / DEGR;
                    double u = std::clamp(since / 30.0, 0.0, 1.0);
                    blow *= u * u * (3.0 - 2.0 * u);
                }
                exhaust += blow + 1.2 * charge * std::max(0.0, -pistonSpeed);
            }
            if (s.region == 0) intakeFlow += std::max(0.0, pistonSpeed) * sim.pMan / sim.P.pAtm;
            heat += s.heatRate;
            if (sp.detailed) {
                if (s.region == 2 && lastRegion[c] != 2 && lastRegion[c] >= 0) opened++;
                lastRegion[c] = s.region;
            }
        }
        double ex = dcBlock.run(exhaust);
        if (!sp.detailed) {
            double body = pipeLow.run(ex) + sp.pipeResGain * pipeRes.run(ex);
            double rasp = 0.6 * sp.raspGain * turbulence.run(noise() * exhaust);
            double induction = intake.run(0.4 * intakeFlow + 3.0 * sim.throttleFlow * noise());
            double knock = 0.15 * clatter.run(noise() * std::min(1.0, heat / 5e6));
            double mix = master.run(body + rasp + induction + knock + starterWhine(sim)) + turboVoice(sim);
            return output(mix);
        }

        // ---- detailed voice (soundParams::detailed) ----------------------------------------------
        double rpm = std::fabs(sim.rpm());
        double demand = std::max(sim.pedal, sim.blip);
        // exhaust flap: opens with revs or throttle (a servo, ~0.3 s)
        double want = std::clamp((rpm - sp.flapOpenRpm) / sp.flapSpan + 0.5, 0.0, 1.0);
        want = std::max(want, std::clamp((demand - 0.25) / 0.25, 0.0, 1.0) * std::clamp((rpm - 2500.0) / 1500.0, 0.0, 1.0));
        flap += (want - flap) * std::min(1.0, 1.0 / (0.3 * fs));

        // header: the firing pulses ring in the pipe; the quarter-wave modes give the hollow howl
        double tube = pipe(ex);
        double radiated = (tube - lastTube) * fs / (2.0 * 3.14159265358979323846 * sp.radiateHz);
        lastTube = tube;
        double body = pipeLow.run(radiated) + sp.pipeResGain * pipeRes.run(tube) + burbleGain(rpm) * burbleLow.run(tube);
        // turbulent rasp rides on the gas flow, brighter as the revs climb
        // turbulence noise climbs steeply with the gas speed (jet noise goes as a high power of it), so it
        // follows the outflow squared: a full charge at high revs roars, the thin overrun flow barely hisses
        double outflow = std::max(0.0, exhaust);
        double rasp = 0.5 * sp.raspGain * turbulence.run(noise() * outflow * std::min(outflow, 3.0) / 1.5);
        double raw = body + rasp;
        // silencer: shut flap = dull and quieter, open = bright and loud (crossfade, both always running)
        double shut = sp.flapShutGain * flapShut.run(raw), open = flapOpen.run(raw);
        double exhaustOut = shut + (open - shut) * flap;

        // airbox: tonal honk from the intake pulses plus the howl of the bellmouths; it mostly
        // speaks with the throttle open
        double open3 = sim.throttleOpen * std::sqrt(sim.throttleOpen);
        double ipulse = intakeFlow * (0.15 + open3);
        double induction = sp.intakeGain * (0.9 * airbox.run(ipulse) + 0.5 * howl.run(ipulse + 0.6 * open3 * noise())) +
                           intake.run(1.5 * sim.throttleFlow * noise());

        // overrun crackle: a share of the exhaust valve openings on a closed throttle light up the
        // unburnt mixture in the hot header; the quickshifter cut ends in a bark and the limiter crackles
        bool overrun = demand < 0.02 && rpm > sp.popRpm && sim.ignition;
        sinceLift = demand < 0.02 ? sinceLift + 1.0 / fs : 0.0;
        double fade = std::exp(-sinceLift / 1.0);  // the ECU leans the overrun out after a second or so
        bool cut = sim.revCut || sim.qsCut;
        for (int k = 0; k < opened; k++) {
            double chance = overrun ? sp.popChance * fade * std::clamp((rpm - sp.popRpm) / 3000.0, 0.2, 1.0) : cut ? 0.5 : 0.0;
            if (0.5 * (noise() + 1.0) < chance) trigger(0.6 + 0.6 * std::fabs(noise()));
        }
        if (lastQs && !sim.qsCut) trigger(1.3);  // power comes back after the shift: bark
        lastQs = sim.qsCut;
        double pn = noise() * popEnv;
        double pops = popAmp * (1.4 * popBand.run(pn) + 2.5 * popThump.run(pn));
        popEnv *= std::exp(-1.0 / (0.008 * fs));

        // straight-cut gears: the primary pair whines all the time, the engaged pair under load
        double crankHz = rpm / 60.0;
        const drivetrainParams &dp = sim.drive.P;
        primaryPhase = std::fmod(primaryPhase + TWO_PI * crankHz * dp.primaryIn / fs, TWO_PI);
        double whine = 0.4 * std::sin(primaryPhase);
        int g = sim.drive.gear;
        if (g > 0 && g < static_cast<int>(dp.teethIn.size()) && dp.teethIn[g] > 0 && !sim.standalone) {
            double mainHz = std::fabs(sim.drive.omegaIn) / TWO_PI / dp.primary;
            gearMeshPhase = std::fmod(gearMeshPhase + TWO_PI * mainHz * dp.teethIn[g] / fs, TWO_PI);
            double load = std::clamp(std::fabs(sim.drive.clutchTorque) / 80.0, 0.0, 1.0);
            whine += load * (std::sin(gearMeshPhase) + 0.3 * std::sin(2.0 * gearMeshPhase));
        }
        whine = sp.whineGain * whineBand.run(whine) * std::clamp(crankHz / 40.0, 0.0, 1.0);

        double knock = 0.08 * clatter.run(noise() * std::min(1.0, heat / 5e6));
        // valvetrain: each exhaust opening is also an intake closing on another cylinder; the clicks
        // matter at idle and drown in the exhaust once the revs are up
        if (opened) tickEnv = 1.0;
        double ticks = sp.tick * tickBand.run(noise() * tickEnv);
        tickEnv *= std::exp(-1.0 / (0.0015 * fs));
        double mix = master.run(exhaustOut + induction + pops + whine + knock + ticks + starterWhine(sim)) + turboVoice(sim);
        return output(mix);
    }

    // the output level: linear (sp.drive is the gain, set per engine so its loudest moments stay
    // clearly under full scale), then a limiter that only acts on what would still go over: below
    // LIMIT_KNEE nothing is touched, above it the excess is folded softly into the last 1 - knee so
    // the output never reaches 1. A shaping stage here (the old 0.8 tanh(drive x)) would flatten
    // the loud pulses and add harmonics the engine does not make.
    static constexpr double LIMIT_KNEE = 0.85;
    double peakOut = 0.0;      // loudest output before the limiter (|drive * mix|), for --wav's report
    long limitedSamples = 0;   // samples the limiter had to touch
    float output(double mix) {
        double y = sp.drive * mix, a = std::fabs(y);
        peakOut = std::max(peakOut, a);
        if (a <= LIMIT_KNEE) return static_cast<float>(y);
        limitedSamples++;
        double room = 1.0 - LIMIT_KNEE;
        return static_cast<float>(std::copysign(LIMIT_KNEE + room * std::tanh((a - LIMIT_KNEE) / room), y));
    }

private:
    // the turbo's own sounds, after the master low-pass so the whistle and the hiss stay bright:
    //   - whistle: the compressor's blade pass (6 full blades, 6 splitters) at the shaft speed,
    //     louder with the air it moves
    //   - whoosh: broadband rush of the intake air through the compressor
    //   - blow-off valve: the vented charge hisses out ("pssh"), following the flow through it
    //   - surge flutter: each backflow burst through the stalled compressor chuffs ("stu-tu-tu")
    double turboVoice(const engineSim &sim) {
        if (!sim.P.turbo.on) return 0.0;
        const double TWO_PI = 2.0 * 3.14159265358979323846;
        double shaftHz = sim.turboOmega / TWO_PI;
        double flow = std::clamp(sim.compFlow / 0.15, 0.0, 1.5);
        double spin = std::clamp(sim.turboRpm() / 60000.0, 0.0, 1.0);
        double whistle = 0.0;
        whistlePhase = std::fmod(whistlePhase + TWO_PI * shaftHz * 6.0 / fs, TWO_PI);
        splitterPhase = std::fmod(splitterPhase + TWO_PI * shaftHz * 12.0 / fs, TWO_PI);
        if (shaftHz * 6.0 < 0.45 * fs) whistle += std::sin(whistlePhase);
        if (shaftHz * 12.0 < 0.45 * fs) whistle += 0.35 * std::sin(splitterPhase);
        whistle *= 0.035 * spin * (0.25 + flow);
        double pr = std::max(0.0, sim.pCharge / sim.P.pAtm - 1.0);
        double rush = 0.35 * whoosh.run(noise() * flow * (0.3 + pr));
        double vent = std::pow(std::clamp(sim.bovFlow / 0.05, 0.0, 1.5), 1.5);  // a trickle at idle vacuum is near silent
        double pssh = 0.9 * bovHigh.run(bovBand.run(noise() * vent));
        double back = std::clamp(-sim.compFlow / 0.03, 0.0, 1.5);
        double flutter = sim.surge * (0.8 * flutterBand.run(noise() * back) + 2.0 * flutterThump.run(back));
        return sp.turboGain * (whistle + rush + pssh + flutter);
    }

    double burbleGain(double rpm) const {
        if (sp.burbleFadeSpan <= 0.0) return sp.burble;
        double u = std::clamp((rpm - sp.burbleFadeFrom) / sp.burbleFadeSpan, 0.0, 1.0);
        return sp.burble * (1.0 + (sp.burbleHigh - 1.0) * u);
    }

    void trigger(double amp) {
        popEnv = 1.0;
        popAmp = amp;
        popCount++;
        popLastAmp = amp;
    }

    double starterWhine(const engineSim &sim) {
        if (!sim.starter) return 0.0;
        const double TWO_PI = 2.0 * 3.14159265358979323846;
        double crankHz = std::fabs(sim.rpm()) / 60.0;
        gearPhase = std::fmod(gearPhase + TWO_PI * crankHz * 130.0 / fs, TWO_PI);          // 130-tooth ring gear
        armaturePhase = std::fmod(armaturePhase + TWO_PI * crankHz * 13.0 / fs, TWO_PI);   // 13:1 reduction
        return 0.07 * (std::sin(gearPhase) + 0.4 * std::sin(2.0 * gearPhase)) + 0.05 * std::sin(armaturePhase);
    }
};

// single-producer single-consumer ring of samples: main thread writes, audio thread reads
class audioRing {
public:
    bool push(float v) {
        std::size_t w = wr.load(std::memory_order_relaxed), next = (w + 1) & MASK;
        if (next == rd.load(std::memory_order_acquire)) return false;  // full: drop
        buf[w] = v;
        wr.store(next, std::memory_order_release);
        return true;
    }
    bool pop(float &v) {
        std::size_t r = rd.load(std::memory_order_relaxed);
        if (r == wr.load(std::memory_order_acquire)) return false;
        v = buf[r];
        rd.store((r + 1) & MASK, std::memory_order_release);
        return true;
    }
    std::size_t size() const {
        return (wr.load(std::memory_order_acquire) - rd.load(std::memory_order_acquire)) & MASK;
    }

private:
    static constexpr std::size_t N = 1u << 16, MASK = N - 1;
    std::array<float, N> buf{};
    std::atomic<std::size_t> wr{0}, rd{0};
};

// band-limited resampling from the physics rate (40 kHz) to the sound card's: windowed-sinc
// interpolation (64 taps, Kaiser window, beta 9: ~90 dB stopband) read from a polyphase table with
// 1024 phases, linearly interpolated between them. The pass band ends at 0.9 of the input's Nyquist
// (18 kHz from 40 kHz), so the images that linear interpolation leaves above 20 kHz (which fold back
// at 44.1 kHz) are gone and the top octave is not dulled. Reading faster than the input (step > 1)
// moves the cutoff down with the step, so a downsample does not alias either.
// Feed it input samples with push(); at(frac) is the signal frac (0..1) of the way from the
// HALF-th newest sample to the next one (a fixed latency of HALF samples: 0.8 ms at 40 kHz).
class sincResampler {
public:
    static constexpr int HALF = 32, TAPS = 2 * HALF, PHASES = 1024;

    sincResampler() { build(0.9); }

    void push(float v) {
        // each sample is written twice, so the newest TAPS always sit contiguous at hist[pos .. pos+TAPS)
        hist[pos] = hist[pos + TAPS] = v;
        pos = (pos + 1) % TAPS;
    }

    // the step between output samples, in input samples: above 1 the cutoff follows it down
    void setStep(double step) {
        double want = 0.9 * std::min(1.0, 1.0 / std::max(step, 1e-6));
        if (std::fabs(want - cutoff) > 0.01 * cutoff) build(want);
    }

    float at(double frac) const {
        double ph = std::clamp(frac, 0.0, 1.0) * PHASES;
        int p = std::min(static_cast<int>(ph), PHASES - 1);
        float f = static_cast<float>(ph - p);
        const float *a = &table[p * TAPS], *b = &table[(p + 1) * TAPS], *x = &hist[pos];
        float ya = 0.f, yb = 0.f;
        for (int j = 0; j < TAPS; j++) {
            ya += a[j] * x[j];
            yb += b[j] * x[j];
        }
        return ya + (yb - ya) * f;
    }

private:
    std::array<float, 2 * TAPS> hist{};
    int pos = 0;  // hist[pos] is the oldest of the newest TAPS samples, hist[pos + TAPS - 1] the newest
    std::vector<float> table = std::vector<float>((PHASES + 1) * TAPS);
    double cutoff = 0.0;

    static double besselI0(double x) {
        double sum = 1.0, term = 1.0;
        for (int k = 1; k < 30; k++) {
            term *= (x / (2.0 * k)) * (x / (2.0 * k));
            sum += term;
        }
        return sum;
    }

    // row p: the taps for an output frac = p / PHASES past hist[pos + HALF - 1]; tap j weighs
    // hist[pos + j], which sits d = HALF - 1 + frac - j samples before the output point
    void build(double fc) {
        const double PI = 3.14159265358979323846, BETA = 9.0, norm = besselI0(BETA);
        cutoff = fc;
        for (int p = 0; p <= PHASES; p++) {
            double frac = static_cast<double>(p) / PHASES, sum = 0.0;
            float *row = &table[p * TAPS];
            for (int j = 0; j < TAPS; j++) {
                double d = HALF - 1 + frac - j;
                double r = d / HALF;
                double w = std::fabs(r) < 1.0 ? besselI0(BETA * std::sqrt(1.0 - r * r)) / norm : 0.0;
                double sinc = std::fabs(d) < 1e-9 ? 1.0 : std::sin(PI * fc * d) / (PI * fc * d);
                row[j] = static_cast<float>(fc * sinc * w);
                sum += row[j];
            }
            for (int j = 0; j < TAPS; j++) row[j] = static_cast<float>(row[j] / sum);  // unity gain at DC
        }
    }
};

// streams the synthesised samples to the sound card, resampling from the physics rate. In slow
// motion fewer samples arrive per second and they are stretched, so the pitch drops like a slowed
// recording. The read rate is nudged to keep about 50 ms queued, which absorbs frame jitter.
class engineAudio : public sf::SoundStream {
public:
    static constexpr unsigned RATE = 44100;
    audioRing ring;
    std::atomic<double> simPerOut{40000.0 / RATE};  // simulation samples per output sample
    std::atomic<bool> muted{false};

    engineAudio() { initialize(1, RATE, {sf::SoundChannel::Mono}); }
    ~engineAudio() override { stop(); }

private:
    std::vector<std::int16_t> out = std::vector<std::int16_t>(512);
    double frac = 0.0;
    float last = 0.f, vol = 1.f;
    sincResampler rs;

    bool onGetData(Chunk &data) override {
        double base = simPerOut.load();
        double target = std::max(64.0, 0.05 * RATE * base);
        double fill = static_cast<double>(ring.size());
        double step = base * std::clamp(1.0 + 0.05 * (fill - target) / target, 0.9, 1.1);
        rs.setStep(step);
        for (auto &o : out) {
            frac += step;
            while (frac >= 1.0) {
                frac -= 1.0;
                float v;
                if (ring.pop(v)) last = v;
                else last *= 0.995f;  // starved (paused): fade out instead of clicking
                rs.push(last);
            }
            float x = rs.at(frac);
            vol += ((muted.load() ? 0.f : 1.f) - vol) * 0.002f;
            o = static_cast<std::int16_t>(std::clamp(x * vol, -1.f, 1.f) * 30000.f);
        }
        data.samples = out.data();
        data.sampleCount = out.size();
        return true;
    }
    void onSeek(sf::Time) override {}
};

