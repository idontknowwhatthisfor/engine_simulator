// engine sound: synthesised from the simulation at the physics rate, streamed to the sound card
#pragma once
#include "physics.hpp"
#include "exhaust.hpp"

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

// a delay line read at a fractional delay (linear interpolation), for the exhaust's pipes. Read
// before writing each sample: read(d) is what was written d samples ago (d >= 1)
struct fracDelay {
    std::vector<double> buf;
    std::size_t pos = 0;  // where the next sample goes
    void init(double maxDelay) { buf.assign(static_cast<std::size_t>(maxDelay) + 4, 0.0); pos = 0; }
    double read(double d) const {
        std::size_t n = buf.size();
        d = std::clamp(d, 1.0, static_cast<double>(n - 2));
        std::size_t k = static_cast<std::size_t>(d);
        double f = d - k;
        double a = buf[(pos + n - k) % n], b = buf[(pos + n - k - 1) % n];
        return a + (b - a) * f;
    }
    void write(double v) { buf[pos] = v; pos = (pos + 1) % buf.size(); }
};

// the exhaust's gas temperature, for the speed of sound in it (c = sqrt(gamma R T)):
//   - the gas leaving the cylinders: the burned-gas temperature at exhaust valve opening (from the
//     combustion, ~800-2000 K) cooled by the blowdown's expansion to ~0.7 of it (so ~600-1100 K in
//     the headers, as measured on real engines), or ~450 K when nothing burned (overrun, fuel cut), averaged over the
//     cylinders and smoothed over ~0.2 s; it falls to the air's 300 K when the engine stops
//   - the pipe walls (0 cold .. 1 at the exhaustModel's full heat), heating over seconds and cooling
//     slower, from the same heat input the drawn pipes glow with (exhaustModel::heatTarget)
//   - along the pipe the gas settles to the walls' temperature: cold pipes chill it, hot ones keep
//     even the overrun's cooler air warm
// So a cold start rings lower (slower sound, longer delays), the pitch of the pipe modes climbs as
// the system heats up over a long pull, and drops a little on the overrun.
struct exhaustGasTemp {
    double gas = 300.0, wall = 0.0;
    bool started = false;
    void update(const engineSim &sim, double dt) {
        double exitT = 300.0, wallT = 0.0;
        int n = sim.cylCount();
        if (n > 0 && std::fabs(sim.rpm()) > 50.0) {
            exitT = 0.0;
            for (int c = 0; c < n; c++) {
                const cylinderState &s = sim.cyl[c];
                // blowdown expands the gas from ~4-6 bar to the pipe's pressure before it gets there:
                // isentropic, (1/5)^((1.35 - 1) / 1.35) = 0.66, so ~0.7 of the in-cylinder temperature
                exitT += s.qTot > 0.0 ? 0.7 * (800.0 + 1200.0 * s.glowEvo) : 450.0;
                wallT += exhaustModel::heatTarget(sim, c);
            }
            exitT /= n;
            wallT /= n;
        }
        if (!started) {  // a run that starts with the engine already going (--simtime) starts settled
            started = true;
            if (sim.time > 1.0) { gas = exitT; wall = wallT; return; }
        }
        gas += (exitT - gas) * (1.0 - std::exp(-dt / 0.2));
        wall += (wallT - wall) * (1.0 - std::exp(-dt / (wallT > wall ? 4.0 : 10.0)));
    }
    // K, d metres down the pipes from the ports: the gas trades heat with the walls, settling to
    // their temperature over ~1 m; the walls are as hot as the drawn pipes (exhaustModel: 40 +
    // 860 * heat deg C at the port, cooling along the pipe over ~1.4 m)
    double wallAt(double d) const { return 313.0 + 860.0 * wall * std::exp(-d / 1.4); }
    double at(double d) const {
        double w = wallAt(d);
        return w + (gas - w) * std::exp(-d / 1.0);
    }
    static double soundSpeed(double T) { return std::sqrt(1.35 * 287.0 * T); }
};

// the drawn exhaust (exhaustLayout) as an acoustic network, so the sound follows the pipes on screen.
// Every pipe the cylinders' gas reaches is a pair of delay lines, one each way, as long as the sound
// takes to travel its centreline at the local gas temperature. Where pipes meet (a merge, a split,
// an X-pipe) a scattering junction shares each arriving wave between them by area: a wave going into
// a bigger pipe partly reflects inverted, into a smaller one partly reflects upright. So:
//   - each cylinder's pulse reaches the merges after its own runner (unequal headers: uneven timing)
//   - every section rings at its own length; two banks ring separately until they meet
//   - a silencer (pipeSpec::acoustic, or automatically a pipe that widens abruptly to 1.6x the radius
//     feeding it and is at least as long as it is wide): besides the reflections its area steps give,
//     its packing damps and dulls what passes through (soundParams::chamberHz, chamberLoss); a drawn
//     exhaust valve, as it opens, lets the gas past the packing (setValve)
//   - a catalyst (tagged in the layout): its area steps, and its monolith takes a little off the top
//   - the turbine (exhaustLayout::turbo) passes a damped, dulled share of the pulses and reflects
//     much of them back into the manifold
//   - every pass along a pipe loses soundParams::wallLoss per metre (at 20 mm radius; narrower pipes
//     lose more): friction, heat to the walls, the eddies at bends and joints
//   - each header starts at its exhaust port: a closed valve reflects almost all, an open one less
//     (the cylinder's volume behind it); the cylinder's pulse enters there
//   - every pipe end nothing continues from is open: it reflects inverted and lossy
//     (soundParams::pipeReflect, pipeDampHz) and radiates the flow that leaves it
// step() returns the summed outflow at the open ends, in units of a header's flow.
// Pipes the gas does not reach (intake runners, ducts) are left out.
class exhaustNetwork {
public:
    bool ok = false;

    // false (and nothing built) when the layout cannot carry the sound: a cylinder without a header
    bool build(const exhaustLayout &L, int nCyl, double fs_, const soundParams &sp_) {
        fs = fs_;
        sp = sp_;
        ok = false;
        const std::size_t n = L.pipes.size();
        std::vector<char> reach(n, 0);
        std::vector<std::vector<float>> dist(n, std::vector<float>(nCyl, -1.f));
        std::vector<double> len(n, 0.0);
        port.assign(nCyl, -1);
        for (std::size_t k = 0; k < n; k++) {
            const pipeSpec &ps = L.pipes[k];
            std::vector<vec3> c = exhaustModel::centreline(ps.pts);
            for (std::size_t j = 1; j < c.size(); j++) len[k] += length(c[j] - c[j - 1]);
            if (ps.cyl >= 0 && ps.cyl < nCyl) {
                reach[k] = 1;
                dist[k][ps.cyl] = 0.f;
                if (port[ps.cyl] < 0) port[ps.cyl] = static_cast<int>(k);
            }
            for (int f : ps.from)
                if (f >= 0 && f < static_cast<int>(k) && reach[f]) {
                    reach[k] = 1;
                    for (int cy = 0; cy < nCyl; cy++)
                        if (dist[f][cy] >= 0.f) dist[k][cy] = std::max(dist[k][cy], dist[f][cy] + static_cast<float>(len[f]));
                }
        }
        for (int c = 0; c < nCyl; c++)
            if (port[c] < 0) return false;
        // the pipes, renumbered
        std::vector<int> idx(n, -1);
        pipes.clear();
        double headerArea = 0.0;
        for (std::size_t k = 0; k < n; k++) {
            if (!reach[k]) continue;
            const pipeSpec &ps = L.pipes[k];
            pipeAc p;
            p.len = std::max(len[k], 1.0) * 1e-3;
            p.a0 = ps.r0 * ps.r0;
            p.a1 = ps.r1 * ps.r1;
            p.r = 0.5e-3 * (ps.r0 + ps.r1);
            float d = 1e9f;
            for (int cy = 0; cy < nCyl; cy++)
                if (dist[k][cy] >= 0.f) d = std::min(d, dist[k][cy]);
            p.mid = (d + 0.5 * len[k]) * 1e-3;
            double upR = 0.0;
            for (int f : ps.from)
                if (f >= 0 && idx[f] >= 0) upR = std::max(upR, static_cast<double>(L.pipes[f].r1));
            // a chamber widens abruptly and is at least as long as it is wide (a short wide tip is not)
            if (ps.acoustic == pipeAcoustic::automatic)
                p.kind = upR > 0.0 && ps.r0 >= 1.6 * upR && len[k] >= 2.0 * ps.r0 ? pipeAcoustic::silencer : pipeAcoustic::plain;
            else
                p.kind = ps.acoustic;
            if (ps.cyl >= 0) headerArea = std::max(headerArea, p.a0);
            double maxDelay = p.len / exhaustGasTemp::soundSpeed(280.0) * fs + 2.0;
            p.fwd.init(maxDelay);
            p.bwd.init(maxDelay);
            idx[k] = static_cast<int>(pipes.size());
            pipes.push_back(std::move(p));
        }
        aRef = headerArea > 0.0 ? headerArea : 1.0;
        for (int &c : port) c = idx[c];
        // terminals 2k (start of pipe k) and 2k + 1 (its end) that meet are one node (union-find)
        const int T = 2 * static_cast<int>(pipes.size());
        std::vector<int> parent(T);
        for (int t = 0; t < T; t++) parent[t] = t;
        auto find = [&](int t) {
            while (parent[t] != t) t = parent[t] = parent[parent[t]];
            return t;
        };
        for (std::size_t k = 0; k < n; k++) {
            if (idx[k] < 0) continue;
            for (int f : L.pipes[k].from)
                if (f >= 0 && f < static_cast<int>(k) && idx[f] >= 0) parent[find(2 * idx[f] + 1)] = find(2 * idx[k]);
        }
        std::vector<std::vector<int>> groups(T);
        for (int t = 0; t < T; t++) groups[find(t)].push_back(t);
        junctions.clear();
        outlets.clear();
        for (int t = 0; t < T; t++) {
            const std::vector<int> &g = groups[t];
            if (g.empty()) continue;
            if (g.size() == 1) {
                int k = g[0] / 2;
                if (g[0] % 2 == 1) outlets.push_back({k});
                continue;  // a lone start is a header's port
            }
            junction j;
            j.terms = g;
            // the turbine: the junction where a pipe starts at the turbine's outlet face
            if (L.turbo.on && g.size() == 2)
                for (std::size_t k = 0; k < n; k++)
                    if (idx[k] >= 0 && (g[0] == 2 * idx[k] || g[1] == 2 * idx[k]) && length(L.pipes[k].pts.front() - L.turbo.at) < 60.f)
                        j.turbine = true;
            junctions.push_back(j);
        }
        double a = std::exp(-2.0 * 3.14159265358979323846 * sp.pipeDampHz / fs);
        openA = a;
        chamberA = std::exp(-2.0 * 3.14159265358979323846 * sp.chamberHz / fs);
        catalystA = std::exp(-2.0 * 3.14159265358979323846 * 4000.0 / fs);
        turbineA = std::exp(-2.0 * 3.14159265358979323846 * 1500.0 / fs);
        ok = !outlets.empty();
        setValve(0.0);
        return ok;
    }

    // the delays from the gas temperature (or soundParams::gasSpeed when it is set)
    void setTemperature(const exhaustGasTemp &gt) {
        for (pipeAc &p : pipes) {
            double c = sp.gasSpeed > 0.0 ? sp.gasSpeed : exhaustGasTemp::soundSpeed(gt.at(p.mid));
            p.delay = std::max(1.0, p.len / c * fs);
            p.loss = std::exp(-sp.wallLoss * 0.02 / p.r * p.len);  // per pass; narrow pipes lose more
        }
    }

    // the exhaust valve (0 shut .. 1 open) opens a way past the silencers' packing: their corner rises
    // to 4x soundParams::chamberHz and their loss falls toward 0.95 per pass
    void setValve(double v) {
        if (std::fabs(v - valve) < 1e-3 && silA > 0.0) return;
        valve = v;
        silA = std::pow(chamberA, 1.0 + 3.0 * v);
        silG = sp.chamberLoss + (0.95 - sp.chamberLoss) * v;
    }

    // src: each cylinder's pulse; open: its exhaust valve is open
    double step(const double *src, const int *open) {
        // what arrives at both ends of every pipe this sample
        for (pipeAc &p : pipes) {
            p.inStart = p.loss * p.bwd.read(p.delay);
            p.inEnd = p.loss * p.fwd.read(p.delay);
            if (p.kind == pipeAcoustic::silencer || p.kind == pipeAcoustic::catalyst) {
                // a silencer's packing: duller and weaker on the way through; a catalyst's monolith
                // only takes a little off the top
                bool sil = p.kind == pipeAcoustic::silencer;
                double a = sil ? silA : catalystA, g = sil ? silG : 0.92;
                p.lpS = (1.0 - a) * p.inStart + a * p.lpS;
                p.lpE = (1.0 - a) * p.inEnd + a * p.lpE;
                p.inStart = g * p.lpS;
                p.inEnd = g * p.lpE;
            }
            p.outStart = p.outEnd = 0.0;
        }
        for (std::size_t c = 0; c < port.size(); c++) {
            pipeAc &p = pipes[port[c]];
            p.outStart = (open[c] ? 0.5 : 0.9) * p.inStart + src[c];
        }
        for (junction &j : junctions) {
            auto in = [&](int t) { return t % 2 ? pipes[t / 2].inEnd : pipes[t / 2].inStart; };
            auto out = [&](int t) -> double & { return t % 2 ? pipes[t / 2].outEnd : pipes[t / 2].outStart; };
            auto area = [&](int t) { return t % 2 ? pipes[t / 2].a1 : pipes[t / 2].a0; };
            if (j.turbine) {
                int up = j.terms[0] % 2 ? j.terms[0] : j.terms[1], down = up == j.terms[0] ? j.terms[1] : j.terms[0];
                j.lp = (1.0 - turbineA) * in(up) + turbineA * j.lp;
                out(up) = 0.6 * in(up);
                out(down) = 0.45 * j.lp + 0.6 * in(down);
                continue;
            }
            double sy = 0.0, syp = 0.0;
            for (int t : j.terms) { sy += area(t); syp += area(t) * in(t); }
            double pj = 2.0 * syp / sy;
            for (int t : j.terms) out(t) = pj - in(t);
        }
        double flow = 0.0;
        for (outlet &o : outlets) {
            pipeAc &p = pipes[o.pipe];
            o.lp = (1.0 - openA) * p.inEnd + openA * o.lp;
            p.outEnd = -sp.pipeReflect * o.lp;
            flow += p.a1 / aRef * (p.inEnd - p.outEnd);
        }
        for (pipeAc &p : pipes) {
            p.fwd.write(p.outStart);
            p.bwd.write(p.outEnd);
        }
        return flow;
    }

    std::size_t pipeCount() const { return pipes.size(); }
    std::size_t outletCount() const { return outlets.size(); }
    std::size_t count(pipeAcoustic k) const { return std::count_if(pipes.begin(), pipes.end(), [k](const pipeAc &p) { return p.kind == k; }); }
    bool hasTurbine() const { return std::any_of(junctions.begin(), junctions.end(), [](const junction &j) { return j.turbine; }); }

private:
    struct pipeAc {
        double len = 0.0, a0 = 1.0, a1 = 1.0, mid = 0.0;  // m, mm^2 at the start / end, m from the ports to its middle
        double r = 0.02;                                   // mean radius, m
        pipeAcoustic kind = pipeAcoustic::plain;
        double delay = 1.0, loss = 1.0;
        fracDelay fwd, bwd;                                // toward the tail / back toward the ports
        double inStart = 0, inEnd = 0, outStart = 0, outEnd = 0, lpS = 0, lpE = 0;
    };
    struct junction { std::vector<int> terms; bool turbine = false; double lp = 0.0; };
    struct outlet { int pipe; double lp = 0.0; };
    double fs = 40000.0, aRef = 1.0, openA = 0.0, chamberA = 0.0, catalystA = 0.0, turbineA = 0.0;
    double valve = -1.0, silA = 0.0, silG = 0.8;  // the silencers' packing as the exhaust valve has it
    soundParams sp;
    std::vector<pipeAc> pipes;
    std::vector<int> port;           // per cylinder: its header
    std::vector<junction> junctions;
    std::vector<outlet> outlets;
};

// where the listener is: the 3D camera, its two ears 18 cm apart along the camera's right vector
// (world space, mm)
struct listenerPose {
    vec3 eye{0.f, 0.f, 1000.f}, right{1.f, 0.f, 0.f};
};

// the listener from the camera (src/common.hpp viewMatrix): the eye sits at target + dist * (row 2 of
// the view rotation), the right vector is row 0
listenerPose listenerFromCamera(const camera &cam, vec3 target) {
    float cy = std::cos(cam.yaw), sy = std::sin(cam.yaw), cp = std::cos(cam.pitch), sp = std::sin(cam.pitch);
    listenerPose l;
    l.eye = target + vec3{-sy * cp, sp, cy * cp} * cam.dist;
    l.right = {cy, 0.f, sy};
    return l;
}

// stereo from the engine's separate sound sources (engineSound's groups), each placed in the scene:
//   - the tailpipes (at the drawn outlets' centre), louder from behind the pipe than from in front of
//     it (an open pipe end beams its higher harmonics along its axis)
//   - the intake (airbox, bellmouths, throttle hiss) above the heads
//   - the engine block (combustion clatter, valvetrain ticks, gear whine, the starter)
//   - the turbo or supercharger, where it is drawn (on the heads when it is not)
// Each reaches each ear after its distance at the speed of sound (so the far ear hears it a little
// later), panned by its direction (a constant-power pan: ~7.5 dB between the ears at 90 deg), at a
// level that follows its distance (1 / r) relative to the starting view, where every source plays at
// its mono level (calibrate), and the ear facing away hears it duller (the head's shadow). Delay
// and level glide to the camera's new place over ~50 ms, so orbiting the camera pans smoothly (with
// the slight Doppler shift a moving listener really hears).
class spatialMixer {
public:
    enum group { EXHAUST, INTAKE, BLOCK, TURBO, GROUPS };

    void init(double fs_) {
        fs = fs_;
        shadowA = std::exp(-2.0 * 3.14159265358979323846 * 1800.0 / fs);
        glide = 1.0 - std::exp(-1.0 / (0.05 * fs));
    }
    // a source of `share` of group g at pos (world mm); dir: the way a tailpipe points (0: all round)
    void add(int g, vec3 pos, vec3 dir, double share) {
        source s;
        s.g = g; s.pos = pos; s.dir = dir; s.share = share;
        s.line.init(0.04 * fs);  // up to ~13 m away
        src.push_back(std::move(s));
    }
    bool empty() const { return src.empty(); }

    // the reference view: from here every source plays at exactly its mono level (so the starting
    // camera hears the mono balance, in stereo); elsewhere its level is relative to this
    void calibrate(const listenerPose &l) {
        for (source &s : src) s.ref = level(s, l);
    }

    void setListener(const listenerPose &l) {
        for (source &s : src) {
            double rel = std::min(level(s, l) / s.ref, 1.6);  // closer than the reference: at most +4 dB
            vec3 toSrc = normalize(s.pos - l.eye);
            double pan = dot(toSrc, normalize(l.right));       // -1 hard left .. +1 hard right
            for (int e = 0; e < 2; e++) {
                double side = e == 0 ? -1.0 : 1.0;
                vec3 ear = l.eye + l.right * static_cast<float>(90.0 * side);
                vec3 v = s.pos - ear;
                double d = std::max(static_cast<double>(length(v)), 150.0);
                s.wantDelay[e] = d * 1e-3 / 343.0 * fs;
                // constant-power pan (about 7.5 dB between the ears for a source at 90 deg)
                s.wantGain[e] = rel * s.share * std::sqrt(std::max(0.0, 1.0 + 0.7 * side * pan));
                // full for a source ahead or on this ear's side, down to 0.4 with the head in the way
                s.wantBright[e] = 1.0 - 0.6 * std::max(0.0, -side * pan);
                if (!s.placed) { s.delay[e] = s.wantDelay[e]; s.gain[e] = s.wantGain[e]; s.bright[e] = s.wantBright[e]; }
            }
            s.placed = true;
        }
    }

    // this sample's groups in, the two ears out
    void mix(const double *groups, double &left, double &right) {
        double out[2] = {0.0, 0.0};
        for (source &s : src) {
            s.line.write(groups[s.g]);
            for (int e = 0; e < 2; e++) {
                s.delay[e] += (s.wantDelay[e] - s.delay[e]) * glide;
                s.gain[e] += (s.wantGain[e] - s.gain[e]) * glide;
                s.bright[e] += (s.wantBright[e] - s.bright[e]) * glide;
                double x = s.line.read(std::max(1.0, s.delay[e]));
                s.lp[e] = (1.0 - shadowA) * x + shadowA * s.lp[e];
                out[e] += s.gain[e] * (s.lp[e] + (x - s.lp[e]) * s.bright[e]);
            }
        }
        left = out[0];
        right = out[1];
    }

private:
    struct source;
    // a source's level heard from l: 1 / r from the listener's head, and a tailpipe's beam (x0.5
    // straight in front of the pipe, x1 behind it)
    static double level(const source &s, const listenerPose &l) {
        double d = std::max(static_cast<double>(length(s.pos - l.eye)), 150.0);
        double g = 1000.0 / d;
        if (length(s.dir) > 0.f) g *= 0.75 + 0.25 * dot(normalize(s.dir), normalize(l.eye - s.pos));
        return g;
    }
    struct source {
        double ref = 1.0;  // level() from the reference view
        int g = 0;
        vec3 pos{}, dir{};
        double share = 1.0;
        fracDelay line;
        double delay[2] = {1, 1}, gain[2] = {0, 0}, bright[2] = {1, 1}, lp[2] = {0, 0};
        double wantDelay[2] = {1, 1}, wantGain[2] = {0, 0}, wantBright[2] = {1, 1};
        bool placed = false;
    };
    std::vector<source> src;
    double fs = 40000.0, shadowA = 0.0, glide = 0.0;
};

struct engineSound {
    double fs;
    soundParams sp;
    biquad dcBlock, pipeLow, pipeRes, turbulence, intake, clatter, master, masterIntake, masterBlock;
    // this sample's sound split by where it comes from (spatialMixer::group), each after its own copy
    // of the master low-pass; the mono output is their sum
    double groups[spatialMixer::GROUPS] = {0, 0, 0, 0};
    spatialMixer space;  // stereo (setScene, setListener, stereo())
    std::uint32_t rng = 0x12345678u;
    double gearPhase = 0.0, armaturePhase = 0.0;
    // the detailed voice's stages (soundParams::detailed)
    // the exhaust: the drawn layout as a network of pipes (exhaustNetwork), or without one a single
    // quarter-wave pipe of soundParams::pipeLength (the round trip in pipeLine)
    exhaustNetwork net;
    biquad netBlock;                  // the DC out of the network's outflow (as dcBlock for the single pipe)
    exhaustGasTemp gasTemp;
    fracDelay pipeLine;
    double pipeDelay = 1.0, pipeDampZ = 0.0, pipeDampA = 0.0;
    int tempCountdown = 0;            // samples to the next delay update from the gas temperature
    bool hasFlap = true;              // the layout draws an exhaust valve (or there is no layout)
    cylArray<double> cylSrc;          // per cylinder: this sample's exhaust pulse (for the network)
    cylArray<int> cylOpen;            // ... and whether its exhaust valve is open
    biquad flapShut, flapOpen, airbox, howl, popBand, popThump, whineBand, burbleLow;
    double flap = 0.0;                // 0 shut .. 1 open, follows rpm and throttle with a servo lag
    double lastTube = 0.0;
    cylArray<int> lastRegion;         // per cylinder, to spot each exhaust valve opening (-1: not seen yet)
    double popEnv = 0.0, popAmp = 0.0, tickEnv = 0.0;
    int popCount = 0;                 // crackles so far (the exhaust draws a flame and sparks for each)
    double popLastAmp = 0.0;
    biquad tickBand;
    double sinceLift = 1e9;           // s since the throttle was snapped shut
    double unburnt = 0.0;             // unburnt fuel in the header, full charges' worth (afterfire)
    cylArray<int> openedCyl;          // the cylinders whose exhaust valve opened this sample
    // per cylinder, from its exhaust valve's opening (valveFlow): the gas's mass (kg), internal energy
    // (J) and the cylinder's volume at the opening (m^3)
    cylArray<double> bdM, bdU, bdV;
    // the valve-flow blowdown's weight against the piston's push: 0.3 keeps the two in the balance
    // the earlier sqrt(pressure) pulses had (median full-throttle level over the engines within 2 dB)
    static constexpr double blowScale = 0.3;
    double primaryPhase = 0.0, gearMeshPhase = 0.0;
    // turbo: compressor whistle, intake whoosh, blow-off valve, surge flutter
    biquad whoosh, bovBand, bovHigh, flutterBand, flutterThump;
    double whistlePhase = 0.0, splitterPhase = 0.0;

    // for this simulation's engine (its sound parameters, idle speed and cylinder count) and, when
    // given, its drawn exhaust: the detailed voice's pipes then follow the layout (unless
    // soundParams::pipeFromLayout is off), and its exhaust valve exists only if the layout has one
    engineSound(double sampleRate, const engineSim &sim, const exhaustLayout *layout = nullptr) : fs(sampleRate), sp(sim.P.sound) {
        dcBlock = biquad::make(biquad::HIGHPASS, fs, dcBlockCutoff(sp, sim.P.idleRpm, sim.cylCount()), 0.707);
        pipeLow = biquad::make(biquad::LOWPASS, fs, sp.pipeLowHz, 0.9);
        pipeRes = biquad::make(biquad::BANDPASS, fs, sp.pipeResHz, 3.0);
        turbulence = biquad::make(biquad::BANDPASS, fs, 1200.0, 0.8);
        intake = biquad::make(biquad::BANDPASS, fs, 350.0, 1.5);
        clatter = biquad::make(biquad::HIGHPASS, fs, 2500.0, 0.707);
        master = biquad::make(biquad::LOWPASS, fs, sp.masterHz, 0.707);
        masterIntake = masterBlock = master;
        space.init(fs);
        whoosh = biquad::make(biquad::BANDPASS, fs, 2500.0, 0.8);
        bovBand = biquad::make(biquad::BANDPASS, fs, 3200.0, 0.6);
        bovHigh = biquad::make(biquad::HIGHPASS, fs, 1200.0, 0.707);
        flutterBand = biquad::make(biquad::BANDPASS, fs, 900.0, 1.2);
        flutterThump = biquad::make(biquad::LOWPASS, fs, 160.0, 0.707);
        if (sp.detailed) {
            // the quarter-wave pipe: one round trip (2L / c) of delay with an inverting, lossy
            // reflection at the open end
            if (layout && sp.pipeFromLayout) net.build(*layout, sim.cylCount(), fs, sp);
            netBlock = dcBlock;
            if (layout) hasFlap = layout->flap.on;
            pipeLine.init(2.0 * sp.pipeLength / exhaustGasTemp::soundSpeed(280.0) * fs + 2.0);
            cylSrc.assign(sim.cylCount(), 0.0);
            cylOpen.assign(sim.cylCount(), 0);
            openedCyl.assign(sim.cylCount(), 0);
            bdM.assign(sim.cylCount(), 0.0); bdU.assign(sim.cylCount(), 0.0); bdV.assign(sim.cylCount(), 0.0);
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

    // quarter-wave exhaust pipe (no layout): y = x - r * damped(y delayed by one round trip)
    double pipe(double x) {
        double old = pipeLine.read(pipeDelay);
        pipeDampZ = (1.0 - pipeDampA) * old + pipeDampA * pipeDampZ;
        double y = x - sp.pipeReflect * pipeDampZ;
        pipeLine.write(y);
        return y;
    }

    // every 32 samples: the gas temperature, and from it the pipes' delays
    void updateTemperature(const engineSim &sim) {
        if (--tempCountdown > 0) return;
        tempCountdown = 32;
        gasTemp.update(sim, 32.0 / fs);
        if (net.ok) net.setTemperature(gasTemp);
        double c = sp.gasSpeed > 0.0 ? sp.gasSpeed : exhaustGasTemp::soundSpeed(gasTemp.at(0.5 * sp.pipeLength));
        pipeDelay = std::max(1.0, 2.0 * sp.pipeLength / c * fs);
    }

    // one output sample for the current simulation state; call once per physics step
    float sample(const engineSim &sim) {
        const double TWO_PI = 2.0 * 3.14159265358979323846;
        double exhaust = 0.0, intakeFlow = 0.0, heat = 0.0, intakeJet = 0.0;
        int opened = 0;  // exhaust valves that opened this sample
        if (sp.detailed && lastRegion.size() != sim.cyl.size()) lastRegion.assign(sim.cyl.size(), -1);
        for (int c = 0; c < sim.cylCount(); c++) {
            const cylinderState &s = sim.cyl[c];
            double x, dx, d2x;
            engineSim::geom(sim.phase(c), s.r, s.L, x, dx, d2x);
            double pistonSpeed = dx * sim.omega / 15.0;  // relative to a brisk 15 m/s
            bool justOpened = sp.detailed && s.region == 2 && lastRegion[c] != 2 && lastRegion[c] >= 0;
            if (s.region == 2 && sp.detailed) {
                // the blowdown through the valve, plus the gas the piston pushes out on the exhaust
                // stroke, scaled by how full the cylinder was (tiny charge on overrun)
                double fullCharge = sim.P.pAtm * s.vDisp / (sim.P.rGas * sim.P.tIntake);
                double charge = std::sqrt(std::clamp(s.mAir / fullCharge, 0.0, 1.0));
                double pulse = blowScale * valveFlow(sim, c, justOpened) + 1.2 * charge * std::max(0.0, -pistonSpeed);
                exhaust += pulse;
                cylSrc[c] = pulse;
            } else if (s.region == 2) {
                // the simple voice: blowdown (signed: an over-expanded cylinder at light load sucks back a
                // weaker pulse) plus the gas the piston pushes out, scaled by how full the cylinder was
                double dp = (s.p - sim.pExh) / 1e5;
                double fullCharge = sim.P.pAtm * s.vDisp / (sim.P.rGas * sim.P.tIntake);
                double charge = std::sqrt(std::clamp(s.mAir / fullCharge, 0.0, 1.0));
                double blow = (dp >= 0.0 ? 1.0 : -0.5) * std::sqrt(std::fabs(dp));
                exhaust += blow + 1.2 * charge * std::max(0.0, -pistonSpeed);
            } else if (sp.detailed) {
                cylSrc[c] = 0.0;
                bdM[c] = 0.0;
            }
            if (sp.detailed) cylOpen[c] = s.region == 2;
            if (s.region == 0 && sp.detailed) intakeJet += intakeVelocity(sim, c, pistonSpeed);
            if (s.region == 0) intakeFlow += std::max(0.0, pistonSpeed) * sim.pMan / sim.P.pAtm;
            heat += s.heatRate;
            if (sp.detailed) {
                if (justOpened) openedCyl[opened++] = c;
                lastRegion[c] = s.region;
            }
        }
        double ex = dcBlock.run(exhaust);
        if (!sp.detailed) {
            double body = pipeLow.run(ex) + sp.pipeResGain * pipeRes.run(ex);
            double rasp = 0.6 * sp.raspGain * turbulence.run(noise() * exhaust);
            double induction = intake.run(0.4 * intakeFlow + 3.0 * sim.throttleFlow * noise());
            double knock = 0.15 * clatter.run(noise() * std::min(1.0, heat / 5e6));
            return mixGroups(body + rasp, induction, knock + starterWhine(sim), turboVoice(sim));
        }

        // ---- detailed voice (soundParams::detailed) ----------------------------------------------
        double rpm = std::fabs(sim.rpm());
        double demand = std::max(sim.pedal, sim.blip);
        // exhaust flap: opens with revs or throttle (a servo, ~0.3 s)
        double want = std::clamp((rpm - sp.flapOpenRpm) / sp.flapSpan + 0.5, 0.0, 1.0);
        want = std::max(want, std::clamp((demand - 0.25) / 0.25, 0.0, 1.0) * std::clamp((rpm - 2500.0) / 1500.0, 0.0, 1.0));
        if (!hasFlap) want = 1.0;  // no valve drawn: nothing to shut
        flap += (want - flap) * std::min(1.0, 1.0 / (0.3 * fs));

        // the exhaust: the firing pulses ring in the pipes (the drawn network, or the single
        // quarter-wave pipe); their modes give the hollow howl
        updateTemperature(sim);
        if (net.ok && hasFlap) net.setValve(flap);
        double tube = net.ok ? netBlock.run(net.step(cylSrc.begin(), cylOpen.begin())) : pipe(ex);
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
        // the bellmouths' roar: noise riding on the air's speed through the intake valves (intakeVelocity)
        double roar = std::min(intakeJet, 3.0);
        double induction = sp.intakeGain * (0.9 * airbox.run(ipulse) + 0.5 * howl.run(ipulse + 0.6 * open3 * roar * noise())) +
                           intake.run(1.5 * sim.throttleFlow * noise());

        // crackle: unburnt fuel lit in the header (afterfire)
        sinceLift = demand < 0.02 ? sinceLift + 1.0 / fs : 0.0;
        for (int k = 0; k < opened; k++) afterfire(sim, openedCyl[k], rpm, demand);
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
        return mixGroups(exhaustOut + pops, induction, whine + knock + ticks + starterWhine(sim), turboVoice(sim));
    }

    // the groups through their master low-pass (the turbo's after it, to stay bright); the mono sum out
    float mixGroups(double exhaustPart, double intakePart, double blockPart, double turboPart) {
        groups[spatialMixer::EXHAUST] = master.run(exhaustPart);
        groups[spatialMixer::INTAKE] = masterIntake.run(intakePart);
        groups[spatialMixer::BLOCK] = masterBlock.run(blockPart);
        groups[spatialMixer::TURBO] = turboPart;
        return output(groups[0] + groups[1] + groups[2] + groups[3]);
    }

    // place the sources in the scene (world space, mm): root is the engine's mesh -> world matrix.
    // The tailpipes at the layout's outlets (without any: under the engine), the intake above the
    // heads, the block between the deck and the crank, the turbo where the layout puts it
    void setScene(const engineModel &e, const exhaustLayout *layout) {
        space = spatialMixer();
        space.init(fs);
        auto world = [&](vec3 p) { return transformPoint(e.root, p); };
        auto worldDir = [&](vec3 d) { return transformPoint(e.root, d) - transformPoint(e.root, {0.f, 0.f, 0.f}); };
        vec3 top{}, mid{};
        int n = static_cast<int>(e.cyls.size());
        for (int c = 0; c < n; c++) {
            top = top + world(cylPoint(e, c, 0.f, e.cyls[c].deckY - 80.f)) * (1.f / n);
            mid = mid + world(cylPoint(e, c, 0.f, 0.5f * e.cyls[c].deckY)) * (1.f / n);
        }
        if (layout && !layout->outlets.empty()) {
            // one source at the tailpipes' centre, pointing their mean way: tips a few cm apart give no
            // width heard from metres away, and separate copies would comb against each other
            vec3 at{}, dir{};
            for (const outletSpec &o : layout->outlets) {
                at = at + world(o.at) * (1.f / layout->outlets.size());
                dir = dir + normalize(worldDir(o.dir));
            }
            space.add(spatialMixer::EXHAUST, at, normalize(dir), 1.0);
        } else {
            space.add(spatialMixer::EXHAUST, mid - vec3{0.f, 300.f, 0.f}, {}, 1.0);
        }
        space.add(spatialMixer::INTAKE, top, {}, 1.0);
        space.add(spatialMixer::BLOCK, mid, {}, 1.0);
        space.add(spatialMixer::TURBO, layout && layout->turbo.on ? world(layout->turbo.at) : top, {}, 1.0);
    }
    // reference: the starting view (each source at its mono level there); l: where the camera is now
    void calibrateListener(const listenerPose &reference) { space.calibrate(reference); }
    void setListener(const listenerPose &l) { space.setListener(l); }

    // the last sample in stereo (call after sample()): each source from where it sits, heard from the
    // listener; the same level and limiter as the mono output
    void stereo(float &left, float &right) {
        double l, r;
        space.mix(groups, l, r);
        left = limit(sp.drive * l);
        right = limit(sp.drive * r);
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
        if (a > LIMIT_KNEE) limitedSamples++;
        return limit(y);
    }
    static float limit(double y) {
        double a = std::fabs(y);
        if (a <= LIMIT_KNEE) return static_cast<float>(y);
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

    // the exhaust valve's blowdown (detailed voice): from the opening, the cylinder's gas escapes
    // through the valves' open area as they lift, by compressible flow through a nozzle (choked while
    // the cylinder is above ~1.9 x the pipe's pressure), as a filling-and-emptying volume: from the
    // physics' pressure at opening, the gas's mass and energy change with the enthalpy carried out
    // until it is down to the pipe's pressure. A light-load charge that has over-expanded below the
    // pipe's pressure instead draws the pipe's hot gas back in (reverse flow: the valve flows worse
    // backwards, discharge coefficient 0.45 against 0.65) until it is filled. The pulse's rise and
    // length come from the valve's lift and the pressure ratio: how sharp or deep each engine sounds.
    // The piston's push on the exhaust stroke is added separately (sample()). Returned: the volume
    // flow at the pipe's pressure (hot gas takes more room), relative to the piston sweeping its
    // bore at 15 m/s.
    //   valves: 2 per cylinder, 0.3 x bore across, lifting to 0.28 x their diameter (the curtain,
    //   capped by the port); open from EVO to 20 deg after TDC on a smooth (1 - cos) lift curve
    double valveFlow(const engineSim &sim, int c, bool justOpened) {
        const cylinderState &s = sim.cyl[c];
        const double R = sim.P.rGas, G = 1.35, PI_ = 3.14159265358979323846;
        const double CV = R / (G - 1.0), CP = G * CV;
        double phi = sim.phase(c);
        if (justOpened) {
            double burnedMass = s.mAir * (1.0 + (s.qTot > 0.0 ? 1.0 / sim.P.afr : 0.0));
            bdV[c] = sim.volume(s, phi);
            bdM[c] = burnedMass;
            bdU[c] = s.p * bdV[c] / (G - 1.0);  // internal energy m cv T = p V / (gamma - 1)
        }
        double m = bdM[c], V = bdV[c];
        if (m <= 0.0) return 0.0;  // not seen open yet (the first cycle), or nothing in it
        double p = (G - 1.0) * bdU[c] / V, T = std::max(200.0, p * V / (m * R));
        double since = std::fmod(phi - sim.P.evo + 4.0 * CYCLE, CYCLE);
        double u = std::clamp(since / (CYCLE + 20.0 * DEGR - sim.P.evo), 0.0, 1.0);
        double dv = 0.3 * sim.P.bore;
        double curtain = PI_ * dv * 0.28 * dv * (0.5 - 0.5 * std::cos(2.0 * PI_ * u));
        double area = 2.0 * std::min(curtain, 0.25 * PI_ * dv * dv);
        double pe = sim.pExh, mdot, tOut, hFlow;
        if (p >= pe) {
            double pr = pe / p;
            mdot = 0.65 * area * p / std::sqrt(R * T) * engineSim::flowFunction(pr);
            tOut = T * std::pow(pr, (G - 1.0) / G);
            hFlow = CP * T;            // the enthalpy leaving
        } else {                       // the pipe's gas back into the cylinder
            double tPipe = gasTemp.at(0.1);
            mdot = -0.45 * area * pe / std::sqrt(R * tPipe) * engineSim::flowFunction(p / pe);
            tOut = tPipe;
            hFlow = CP * tPipe;        // the enthalpy coming in
        }
        double dt = 1.0 / fs;
        bdU[c] = std::max(bdU[c] - mdot * hFlow * dt, 1e-6);
        bdM[c] = std::max(m - mdot * dt, 1e-6 * s.vDisp);
        double q = mdot * R * tOut / pe;  // m^3/s at the pipe's pressure
        return q / (s.area * 15.0);
    }

    // the air's speed through cylinder c's intake valves while they are open (from ~20 deg before
    // TDC to IVC, same valve sizes as the exhaust's, a little bigger: 0.35 x bore), relative to
    // 100 m/s and squared (turbulent roar grows steeply with the jet's speed), scaled by the air's
    // density (manifold pressure). The piston sets the flow; the valve's small opening at either end
    // of the stroke makes the jet fast: the roar peaks as each valve opens and closes.
    double intakeVelocity(const engineSim &sim, int c, double pistonSpeed) {
        const cylinderState &s = sim.cyl[c];
        const double PI_ = 3.14159265358979323846;
        double phi = sim.phase(c);
        double u = std::clamp((phi + 20.0 * DEGR) / (sim.P.ivc + 20.0 * DEGR), 0.0, 1.0);
        double dv = 0.35 * sim.P.bore;
        double curtain = PI_ * dv * 0.28 * dv * (0.5 - 0.5 * std::cos(2.0 * PI_ * u));
        double area = 0.65 * 2.0 * std::max(std::min(curtain, 0.25 * PI_ * dv * dv), 1e-6);
        double v = std::min(std::max(0.0, pistonSpeed) * 15.0 * s.area / area, 340.0);
        return (v / 100.0) * (v / 100.0) * sim.pMan / sim.P.pAtm;
    }

    // afterfire, at cylinder c's exhaust valve opening. Unburnt fuel collects in the header:
    //   - the overrun map: with the throttle shut above soundParams::popRpm the ECU keeps injecting for
    //     a moment with the spark retarded, so a share of each charge's fuel (popChance, more as
    //     the revs rise) leaves unburnt and still burning; it leans out over about a second
    //   - an ignition cut (rev limiter, quickshifter): the spark is cut, not the fuel, so the whole
    //     charge's fuel goes into the pipe
    // Each exhaust stroke flushes some of it out of the tailpipe (15%), and the pulse arriving lights
    // what is there with a chance set by how hot it is: a burning or freshly burned slug (~1000 K and
    // up), or failing that the header walls (the gas temperature model's, ~300-1170 K), from nothing
    // at 700 K to certain at 1100 K, and by how much fuel has gathered. A pop burns most of it, louder
    // the more there was. So the limiter crackles once the pipes are hot, the overrun pops die away as
    // the ECU leans out, and the first firing after a quickshift lights the fuel the cut cylinders
    // dumped: the bark.
    void afterfire(const engineSim &sim, int c, double rpm, double demand) {
        const cylinderState &s = sim.cyl[c];
        if (s.mAir <= 0.0 || sim.stalled || !sim.ignition) { unburnt *= 0.85; return; }
        double fullCharge = sim.P.pAtm * s.vDisp / (sim.P.rGas * sim.P.tIntake);
        double fuel = s.mAir / fullCharge;  // this charge's fuel, in full charges' worth
        bool burned = s.qTot > 0.0, cut = sim.revCut || sim.qsCut;
        bool overrunMap = demand < 0.02 && rpm > sp.popRpm;
        double add = 0.0, source = 0.0;     // unburnt fuel added; the igniting gas's temperature (K)
        if (cut && !burned) {
            add = fuel;
        } else if (overrunMap) {
            double fade = std::exp(-sinceLift / 1.0);
            add = fuel * sp.popChance * fade * std::clamp((rpm - sp.popRpm) / 3000.0, 0.2, 1.0);
            source = 1100.0 * fade;         // the late-burning charge itself
        }
        if (burned) source = std::max(source, 0.7 * (800.0 + 1200.0 * s.glowEvo));
        source = std::max(source, gasTemp.wallAt(0.3));
        unburnt = 0.85 * unburnt + add;
        double chance = std::clamp((source - 700.0) / 400.0, 0.0, 1.0) * std::clamp(unburnt / 0.05, 0.0, 1.0);
        if (0.5 * (noise() + 1.0) < chance) {
            double lit = 0.8 * unburnt;
            unburnt -= lit;
            trigger(std::clamp(0.45 + 2.0 * std::sqrt(lit), 0.4, 1.5));
        }
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

// one stereo sample
struct audioFrame { float l = 0.f, r = 0.f; };

// single-producer single-consumer ring of stereo frames: main thread writes, audio thread reads
class audioRing {
public:
    bool push(audioFrame v) {
        std::size_t w = wr.load(std::memory_order_relaxed), next = (w + 1) & MASK;
        if (next == rd.load(std::memory_order_acquire)) return false;  // full: drop
        buf[w] = v;
        wr.store(next, std::memory_order_release);
        return true;
    }
    bool pop(audioFrame &v) {
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
    std::array<audioFrame, N> buf{};
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
// recording. The read rate is nudged to keep about 50 ms queued, which absorbs frame jitter. Always
// a stereo stream: the mono sound arrives as the same sample on both sides.
class engineAudio : public sf::SoundStream {
public:
    static constexpr unsigned RATE = 44100;
    audioRing ring;
    std::atomic<double> simPerOut{40000.0 / RATE};  // simulation samples per output sample
    std::atomic<bool> muted{false};

    engineAudio() { initialize(2, RATE, {sf::SoundChannel::FrontLeft, sf::SoundChannel::FrontRight}); }
    ~engineAudio() override { stop(); }

private:
    std::vector<std::int16_t> out = std::vector<std::int16_t>(1024);  // 512 frames, interleaved
    double frac = 0.0;
    audioFrame last;
    float vol = 1.f;
    sincResampler rsL, rsR;

    bool onGetData(Chunk &data) override {
        double base = simPerOut.load();
        double target = std::max(64.0, 0.05 * RATE * base);
        double fill = static_cast<double>(ring.size());
        double step = base * std::clamp(1.0 + 0.05 * (fill - target) / target, 0.9, 1.1);
        rsL.setStep(step);
        rsR.setStep(step);
        for (std::size_t k = 0; k < out.size(); k += 2) {
            frac += step;
            while (frac >= 1.0) {
                frac -= 1.0;
                audioFrame v;
                if (ring.pop(v)) last = v;
                else { last.l *= 0.995f; last.r *= 0.995f; }  // starved (paused): fade out instead of clicking
                rsL.push(last.l);
                rsR.push(last.r);
            }
            vol += ((muted.load() ? 0.f : 1.f) - vol) * 0.002f;
            out[k] = static_cast<std::int16_t>(std::clamp(rsL.at(frac) * vol, -1.f, 1.f) * 30000.f);
            out[k + 1] = static_cast<std::int16_t>(std::clamp(rsR.at(frac) * vol, -1.f, 1.f) * 30000.f);
        }
        data.samples = out.data();
        data.sampleCount = out.size();
        return true;
    }
    void onSeek(sf::Time) override {}
};

