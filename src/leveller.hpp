#pragma once
// Playback leveller: a slow compressor that rides the gain of what goes to the sound card, so the
// quiet stretches (idle, cruising, a single's gaps between firings) come up toward a target loudness
// while the loud ones are left about where they are. It does not shape the waveform (one gain for
// both channels, gliding), so it adds no harmonics.
//
// Only the live window's stream uses it (engineAudio::onGetData, at the output rate): the engines'
// levels (soundParams::drive), --wav, --stats and the tests are all upstream and untouched.
//
// Per frame:
//   - loudness: the mean square of both channels through a one-pole (RMS_MS)
//   - peak: instant attack, PEAK_RELEASE_MS release, measured on the input before the delay
//   - wanted gain = min((TARGET / rms)^(1 - 1/RATIO), CEILING / peak), clamped to [1, maxGain]; 1
//     below GATE (the silence before the starter, the fade when paused is not pumped up) and when
//     switched off. The ratio keeps the loudness order: output level = TARGET^(1-1/R) x rms^(1/R)
//     rises with the input, so idle stays under load (a 20 dB gap becomes 20 / RATIO). Pulling
//     everything to TARGET (an infinite ratio) put the tuned V4's idle over its full throttle.
//   - the gain falls in ATTACK_MS and rises in RELEASE_MS; the audio is delayed by LOOKAHEAD frames
//     so the gain is already down when a sudden pulse reaches the output
// Cost: a handful of multiply-adds per output frame on the audio thread.
#include <algorithm>
#include <array>
#include <cmath>

struct playbackLeveller {
    static constexpr double TARGET = 0.126;          // -18 dBFS RMS: no boost from here up
    static constexpr double RATIO = 2.0;             // upward compression below TARGET
    static constexpr double CEILING = 0.70;          // where the peaks are kept (the engines' own peak)
    static constexpr double MAX_GAIN = 5.62;         // +15 dB
    static constexpr double MAX_GAIN_SPEAKER = 10.0; // +20 dB with the speaker voicing on (its high-pass
                                                     // took out the bass that counted as loudness)
    static constexpr double GATE = 5.6e-4;           // -65 dBFS RMS: below this, no boost
    static constexpr double RMS_MS = 300.0, PEAK_RELEASE_MS = 200.0;
    static constexpr double ATTACK_MS = 2.0, RELEASE_MS = 1000.0;
    static constexpr int LOOKAHEAD = 256;            // frames (5.8 ms at 44.1 kHz), a power of two

    bool on = true;
    double maxGain = MAX_GAIN;

    explicit playbackLeveller(double rate)
        : aRms(coef(RMS_MS, rate)), aPeak(coef(PEAK_RELEASE_MS, rate)),
          aAtt(coef(ATTACK_MS, rate)), aRel(coef(RELEASE_MS, rate)) {}

    // the current gain (1 = untouched), for the HUD
    double gain() const { return g; }

    void process(float &l, float &r) {
        double ms2 = 0.5 * (double(l) * l + double(r) * r);
        ms = aRms * ms + (1.0 - aRms) * ms2;
        double pk = std::max(std::fabs(l), std::fabs(r));
        peak = pk > peak ? pk : aPeak * peak + (1.0 - aPeak) * pk;

        double want = 1.0;
        double rms = std::sqrt(ms);
        if (on && rms > GATE) {
            want = std::pow(TARGET / rms, 1.0 - 1.0 / RATIO);
            if (peak > 0.0) want = std::min(want, CEILING / peak);
            want = std::clamp(want, 1.0, maxGain);
        }
        double a = want < g ? aAtt : aRel;
        g = a * g + (1.0 - a) * want;

        int i = pos;
        pos = (pos + 1) & (LOOKAHEAD - 1);
        float dl = bufL[i], dr = bufR[i];
        bufL[i] = l;
        bufR[i] = r;
        l = static_cast<float>(dl * g);
        r = static_cast<float>(dr * g);
    }

private:
    static double coef(double ms, double rate) { return std::exp(-1000.0 / (ms * rate)); }
    double aRms, aPeak, aAtt, aRel;
    double ms = 0.0, peak = 0.0, g = 1.0;
    std::array<float, LOOKAHEAD> bufL{}, bufR{};
    int pos = 0;
};
