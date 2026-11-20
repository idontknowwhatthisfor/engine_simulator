#pragma once
// Speaker voicing: makes the low end heard on small speakers (laptops, phones, desk speakers), which
// play almost nothing below ~150 Hz. Playback only (engineAudio::onGetData, before the leveller), off
// by default (O in the window): headphones and full-range speakers keep the honest sound.
//
//   - EQ (linear): a 2nd-order high-pass at HP_HZ takes out the bass the speaker cannot play, which
//     otherwise eats the leveller's level budget (it measures that bass as loudness and boosts less),
//     plus a +MID_DB peak at MID_HZ, where small speakers are most efficient.
//   - Virtual bass ("missing fundamental"): the band under BASS_HZ is full-wave rectified, which makes
//     its 2nd, 4th... harmonics at a level proportional to the input (no level-dependent distortion),
//     then band-passed to HARM_LO..HARM_HI and mixed in at HARM_GAIN. The ear hears the harmonics and
//     fills in the fundamental, so a single's thump and the pipe resonance come through as punch.
//     This adds harmonics the engine does not make: that is why it is playback-only and switchable.
//     HARM_GAIN 4: at 1.2 the MX King's idle gained only ~2 dB above 150 Hz; with 4 (and the leveller's
//     cap raised to +20 dB while this is on) about +10 dB.
// Switching glides over 50 ms (both paths always run), so it does not click. Cost: eight biquads per
// channel per output frame.
#include <algorithm>
#include <cmath>

struct speakerVoicing {
    static constexpr double HP_HZ = 110.0;
    static constexpr double MID_HZ = 400.0, MID_DB = 3.0, MID_Q = 0.7;
    static constexpr double BASS_HZ = 150.0;
    static constexpr double HARM_LO = 150.0, HARM_HI = 600.0, HARM_GAIN = 4.0;

    bool on = false;

    explicit speakerVoicing(double rate) {
        for (channel *c : {&L, &R}) {
            c->hp = filter::highpass(rate, HP_HZ);
            c->mid = filter::peaking(rate, MID_HZ, MID_Q, MID_DB);
            c->low1 = c->low2 = filter::lowpass(rate, BASS_HZ);
            c->hlo1 = c->hlo2 = filter::highpass(rate, HARM_LO);
            c->hhi1 = c->hhi2 = filter::lowpass(rate, HARM_HI);
        }
        glide = 1.0 - std::exp(-1000.0 / (50.0 * rate));
    }

    void process(float &l, float &r) {
        mix += ((on ? 1.0 : 0.0) - mix) * glide;
        l = static_cast<float>(L.run(l, mix));
        r = static_cast<float>(R.run(r, mix));
    }

private:
    // RBJ audio EQ cookbook biquad (transposed direct form II)
    struct filter {
        double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
        static filter make(double b0, double b1, double b2, double a0, double a1, double a2) {
            filter f;
            f.b0 = b0 / a0; f.b1 = b1 / a0; f.b2 = b2 / a0; f.a1 = a1 / a0; f.a2 = a2 / a0;
            return f;
        }
        static double w(double fs, double f0) { return 2.0 * 3.14159265358979323846 * f0 / fs; }
        static filter lowpass(double fs, double f0, double q = 0.7071) {
            double c = std::cos(w(fs, f0)), al = std::sin(w(fs, f0)) / (2.0 * q);
            return make((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
        }
        static filter highpass(double fs, double f0, double q = 0.7071) {
            double c = std::cos(w(fs, f0)), al = std::sin(w(fs, f0)) / (2.0 * q);
            return make((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
        }
        static filter peaking(double fs, double f0, double q, double db) {
            double A = std::pow(10.0, db / 40.0), c = std::cos(w(fs, f0)), al = std::sin(w(fs, f0)) / (2.0 * q);
            return make(1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A);
        }
        double run(double x) {
            double y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }
    };
    struct channel {
        filter hp, mid, low1, low2, hlo1, hlo2, hhi1, hhi2;
        double run(double x, double mix) {
            double eq = mid.run(hp.run(x));
            double bass = low2.run(low1.run(x));
            double harm = hhi2.run(hhi1.run(hlo2.run(hlo1.run(std::fabs(bass)))));
            double voiced = eq + HARM_GAIN * harm;
            return x + mix * (voiced - x);
        }
    };
    channel L, R;
    double mix = 0.0, glide = 0.0;
};
