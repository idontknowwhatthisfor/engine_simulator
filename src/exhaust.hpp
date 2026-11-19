// the exhaust system, drawn beside the engine: headers from each port, merges, collector, cat or
// pre-silencer, silencer and tailpipe. Each engine gives its own layout (engines/<type>/*.hpp); this
// file builds the swept tubes and animates them from the simulation:
//   - heat: each cylinder's runner heats toward the burned-gas temperature at exhaust valve opening,
//     scaled by how much gas flows (rpm), with a lag of a few seconds; it glows dull red, then orange,
//     at sustained high load and cools down the pipe and after a lift-off
//   - pulses: when a cylinder's exhaust valve opens, a slug of hot gas runs down its runner and on
//     through the merges at the gas flow speed (best seen in slow motion, [ )
//   - the bikes' exhaust valve (flap): a servo lever on the pipe, shut at idle, open with revs or throttle
//   - crackle: every pop the sound makes (overrun, limiter, quickshifter bark) shoots a flame out of
//     the tailpipes and throws a spray of sparks that fall under gravity
#pragma once
#include "render.hpp"
#include "physics.hpp"

// one tube: a smooth centreline through control points (mesh space, mm), radius tapering r0 -> r1
struct pipeSpec {
    std::vector<vec3> pts;
    float r0 = 20.f, r1 = 20.f;
    int cyl = -1;                 // a runner from this cylinder's port (0 .. cylinders - 1); -1: fed by `from`
    std::vector<int> from;        // upstream pipes (earlier indices) that flow into this one
    std::array<float, 4> rgba{0.5f, 0.5f, 0.5f, 1.f};
    float rough = 0.4f, metal = 0.8f;
    float segLen = 40.f;          // heat segments along the tube, mm
    bool capStart = false, capEnd = false;
    int sides = 20;
};

// exhaust valve in the pipe, worked by a lever on a shaft that pokes out of the pipe wall
struct flapSpec {
    bool on = false;
    vec3 at{};                    // on the pipe centreline
    vec3 axis{1.f, 0.f, 0.f};     // pipe direction there
    vec3 shaft{0.f, 0.f, 1.f};    // the shaft, across the pipe (the lever is on this side)
    float pipeR = 25.f;
};

// a fixed part: a flange plate, a bracket (mesh space, axis-aligned)
struct boxSpec {
    vec3 lo, hi;
    std::array<float, 4> rgba{0.3f, 0.3f, 0.3f, 1.f};
    float rough = 0.6f, metal = 0.5f;
};

// a tailpipe exit: where the crackle flames come out
struct outletSpec {
    vec3 at;                      // centre of the opening, mesh mm
    vec3 dir;                     // pointing out of the pipe
    float r = 20.f;
};

// a turbocharger (src/turbo.hpp) in the exhaust: where its frame sits in mesh space. The turbo's own
// frame has the shaft along +x from the turbine outlet face (x = 0) toward the compressor
struct turboSpec {
    bool on = false;
    vec3 at{};                                                  // turbine outlet face, on the shaft
    vec3 x{0.f, 0.f, 1.f}, y{0.f, 1.f, 0.f}, z{-1.f, 0.f, 0.f};  // the turbo's axes in mesh space (a rotation)
};

struct exhaustLayout {
    std::vector<pipeSpec> pipes;
    std::vector<boxSpec> boxes;
    std::vector<outletSpec> outlets;
    flapSpec flap;
    turboSpec turbo;
};

// a point / direction in cylinder c's own frame (bore toward -y, x across the head) in mesh space
vec3 cylPoint(const engineModel &e, int c, float lx, float ly) {
    return transformPoint(bankMatrix(e, c), {e.cyls[c].gudgeonX + lx, ly, e.cyls[c].zCentre});
}
vec3 cylDir(const engineModel &e, int c, float lx, float ly) {
    float a = e.cyls[c].bank, cs = std::cos(a), sn = std::sin(a);
    return normalize(vec3{lx * cs - ly * sn, lx * sn + ly * cs, 0.f});
}

// model matrix whose x, y, z axes are the given (orthonormal) directions, placed at o
mat4 basisMatrix(vec3 o, vec3 x, vec3 y, vec3 z) {
    mat4 r = identity();
    vec3 cols[4] = {x, y, z, o};
    for (int c = 0; c < 4; c++) {
        r.at(0, c) = cols[c].x;
        r.at(1, c) = cols[c].y;
        r.at(2, c) = cols[c].z;
    }
    return r;
}

struct exhaustModel {
    struct segment {
        meshBuf mesh;
        float s = 0.f;            // mid of the segment, mm from the pipe start
    };
    struct pipe {
        pipeSpec spec;
        std::vector<segment> segs;
        float length = 0.f;       // mm along the centreline
        std::vector<float> dist;  // per cylinder: mm from cylinder c's port to this pipe's start (-1: not fed)
    };
    std::vector<pipe> pipes;
    flapSpec flapAt;
    meshBuf lever, pulley;
    std::vector<boxSpec> boxSpecs;
    std::vector<meshBuf> boxes;
    vec3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};  // mesh-space bounds
    std::vector<float> runnerR;  // runner radius per cylinder, mm (20 unless a header says otherwise)

    // crackle: flame at the outlets, sparks thrown out of them
    std::vector<outletSpec> outlets;
    meshBuf flameMesh, sparkMesh;
    struct spark {
        vec3 p, v;                // mesh mm, mm/s
        float age, life;          // s
    };
    std::vector<spark> sparks;
    double fireAge = 1e9, fireAmp = 0.0;  // s since the last pop, its strength
    int lastPops = -1;
    std::uint32_t rng = 0x9e3779b9u;
    float rand01() {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return static_cast<float>(rng) / 4294967296.f;
    }

    // animated state
    std::vector<double> heat;      // 0..1 per cylinder's gas path
    double flap = 0.0;              // 0 shut .. 1 open
    double lastTime = -1.0;

    // centreline: uniform Catmull-Rom through the control points, sampled every ~5 mm
    static std::vector<vec3> centreline(const std::vector<vec3> &p) {
        std::vector<vec3> out;
        if (p.size() < 2) return out;
        for (std::size_t k = 0; k + 1 < p.size(); k++) {
            vec3 p0 = k > 0 ? p[k - 1] : p[k] * 2.f - p[k + 1];
            vec3 p1 = p[k], p2 = p[k + 1];
            vec3 p3 = k + 2 < p.size() ? p[k + 2] : p[k + 1] * 2.f - p[k];
            int n = std::max(2, static_cast<int>(std::ceil(length(p2 - p1) / 5.f)));
            for (int j = 0; j < n; j++) {
                float t = static_cast<float>(j) / n, t2 = t * t, t3 = t2 * t;
                vec3 q = (p1 * 2.f + (p2 - p0) * t + (p0 * 2.f - p1 * 5.f + p2 * 4.f - p3) * t2 +
                          (p1 * 3.f - p0 - p2 * 3.f + p3) * t3) * 0.5f;
                out.push_back(q);
            }
        }
        out.push_back(p.back());
        return out;
    }

    // a tube along a sampled centreline, split into heat segments; smooth normals, parallel-transport frames
    static void sweep(pipe &pp) {
        const pipeSpec &sp = pp.spec;
        std::vector<vec3> c = centreline(sp.pts);
        std::size_t n = c.size();
        std::vector<float> s(n, 0.f);
        for (std::size_t k = 1; k < n; k++) s[k] = s[k - 1] + length(c[k] - c[k - 1]);
        pp.length = s.back();
        std::vector<vec3> t(n), nor(n), bin(n);
        for (std::size_t k = 0; k < n; k++) {
            vec3 d = c[std::min(k + 1, n - 1)] - c[k > 0 ? k - 1 : 0];
            t[k] = normalize(d);
        }
        vec3 ref = std::fabs(t[0].y) < 0.9f ? vec3{0.f, 1.f, 0.f} : vec3{1.f, 0.f, 0.f};
        nor[0] = normalize(cross(cross(t[0], ref), t[0]));
        for (std::size_t k = 0; k < n; k++) {
            if (k > 0) {
                vec3 m = nor[k - 1] - t[k] * dot(nor[k - 1], t[k]);
                nor[k] = length(m) > 1e-6f ? normalize(m) : nor[k - 1];
            }
            bin[k] = cross(t[k], nor[k]);
        }
        auto radius = [&](std::size_t k) { return sp.r0 + (sp.r1 - sp.r0) * (pp.length > 0.f ? s[k] / pp.length : 0.f); };
        int nSeg = sp.segLen > 0.f ? std::max(1, static_cast<int>(std::lround(pp.length / sp.segLen))) : 1;
        nSeg = std::min<int>(nSeg, static_cast<int>(n) - 1);
        const int SIDES = sp.sides;
        for (int g = 0; g < nSeg; g++) {
            std::size_t k0 = (n - 1) * g / nSeg, k1 = (n - 1) * (g + 1) / nSeg;
            segment seg;
            meshBuf &m = seg.mesh;
            auto vert = [&](vec3 p, vec3 nn) {
                m.v.insert(m.v.end(), {p.x, p.y, p.z, nn.x, nn.y, nn.z});
                return static_cast<std::uint32_t>(m.v.size() / 6 - 1);
            };
            for (std::size_t k = k0; k <= k1; k++) {
                float r = radius(k);
                for (int j = 0; j < SIDES; j++) {
                    float a = 2.f * PI * j / SIDES;
                    vec3 dir = nor[k] * std::cos(a) + bin[k] * std::sin(a);
                    vert(c[k] + dir * r, dir);
                }
            }
            for (std::size_t k = 0; k < k1 - k0; k++)
                for (int j = 0; j < SIDES; j++) {
                    std::uint32_t a = static_cast<std::uint32_t>(k * SIDES + j);
                    std::uint32_t b = static_cast<std::uint32_t>(k * SIDES + (j + 1) % SIDES);
                    m.i.insert(m.i.end(), {a, b, b + SIDES, a, b + SIDES, a + SIDES});
                }
            auto cap = [&](std::size_t k, float sign) {
                vec3 nn = t[k] * sign;
                std::uint32_t mid = vert(c[k], nn), first = 0;
                for (int j = 0; j < SIDES; j++) {
                    float a = 2.f * PI * j / SIDES;
                    std::uint32_t id = vert(c[k] + (nor[k] * std::cos(a) + bin[k] * std::sin(a)) * radius(k), nn);
                    if (j == 0) first = id;
                }
                for (int j = 0; j < SIDES; j++) {
                    std::uint32_t a = first + j, b = first + (j + 1) % SIDES;
                    m.i.insert(m.i.end(), {mid, a, b});
                }
            };
            if (g == 0 && sp.capStart) cap(k0, -1.f);
            if (g == nSeg - 1 && sp.capEnd) cap(k1, 1.f);
            seg.s = 0.5f * (s[k0] + s[k1]);
            pp.segs.push_back(std::move(seg));
        }
    }

    // nCyl: the engine's cylinder count (the headers' pipeSpec::cyl index them)
    void build(const exhaustLayout &L, int nCyl) {
        pipes.clear();
        runnerR.assign(nCyl, 20.f);
        heat.assign(nCyl, 0.0);
        for (std::size_t k = 0; k < L.pipes.size(); k++) {
            pipe pp;
            pp.spec = L.pipes[k];
            if (pp.spec.cyl >= nCyl) throw std::runtime_error("exhaust: a header names cylinder " + std::to_string(pp.spec.cyl + 1) +
                                                              " of a " + std::to_string(nCyl) + "-cylinder engine");
            pp.dist.assign(nCyl, -1.f);
            sweep(pp);
            if (pp.spec.cyl >= 0) {
                pp.dist[pp.spec.cyl] = 0.f;
                runnerR[pp.spec.cyl] = pp.spec.r0;
            }
            for (int f : pp.spec.from) {
                if (f < 0 || f >= static_cast<int>(k)) throw std::runtime_error("exhaust: a pipe must come after the pipes feeding it");
                const pipe &up = pipes[f];
                for (int c = 0; c < nCyl; c++)
                    if (up.dist[c] >= 0.f) pp.dist[c] = std::max(pp.dist[c], up.dist[c] + up.length);
            }
            for (const segment &sg : pp.segs)
                for (std::size_t v = 0; v < sg.mesh.v.size(); v += 6) {
                    vec3 p{sg.mesh.v[v], sg.mesh.v[v + 1], sg.mesh.v[v + 2]};
                    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
                    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
                }
            pipes.push_back(std::move(pp));
        }
        boxSpecs = L.boxes;
        boxes.clear();
        for (const boxSpec &b : boxSpecs) {
            boxes.push_back(makeBox(b.lo, b.hi));
            lo = {std::min(lo.x, b.lo.x), std::min(lo.y, b.lo.y), std::min(lo.z, b.lo.z)};
            hi = {std::max(hi.x, b.hi.x), std::max(hi.y, b.hi.y), std::max(hi.z, b.hi.z)};
        }
        outlets = L.outlets;
        for (outletSpec &o : outlets) o.dir = normalize(o.dir);
        // flame plume: a round body along +y from the opening (y = 0) to the tip (y = 1), widest a third
        // of the way out; the flame shader brightens it toward the opening
        {
            const int SIDES = 20, RINGS = 14;
            for (int k = 0; k <= RINGS; k++) {
                float y = static_cast<float>(k) / RINGS;
                float r = (1.f + 1.2f * y) * (1.f - y * y);
                for (int j = 0; j < SIDES; j++) {
                    float a = 2.f * PI * j / SIDES;
                    flameMesh.v.insert(flameMesh.v.end(), {r * std::cos(a), y, r * std::sin(a), std::cos(a), 0.3f, std::sin(a)});
                }
            }
            for (int k = 0; k < RINGS; k++)
                for (int j = 0; j < SIDES; j++) {
                    std::uint32_t a = k * SIDES + j, b = k * SIDES + (j + 1) % SIDES;
                    flameMesh.i.insert(flameMesh.i.end(), {a, b, b + SIDES, a, b + SIDES, a + SIDES});
                }
        }
        sparkMesh = makeBox({-0.5f, -0.5f, 0.f}, {0.5f, 0.5f, 1.f});  // a streak along +z
        flapAt = L.flap;
        if (flapAt.on) {
            lever = makeBox({0.f, -4.f, -3.f}, {30.f, 4.f, 3.f});
            pulley = makeCylinder(13.f, -3.f, 3.f, 24);
        }
    }

    void upload(renderer &ren) {
        for (pipe &pp : pipes)
            for (segment &sg : pp.segs) ren.upload(sg.mesh);
        for (meshBuf &b : boxes) ren.upload(b);
        if (flapAt.on) {
            ren.upload(lever);
            ren.upload(pulley);
        }
        if (!outlets.empty()) {
            ren.upload(flameMesh);
            ren.upload(sparkMesh);
        }
    }

    // mesh -> world bounds, to make room for the pipes in the camera framing and the floor
    void growBounds(engineModel &e) const {
        for (int k = 0; k < 8; k++) {
            vec3 p = transformPoint(e.root, {k & 1 ? hi.x : lo.x, k & 2 ? hi.y : lo.y, k & 4 ? hi.z : lo.z});
            e.lo = {std::min(e.lo.x, p.x - 5.f), std::min(e.lo.y, p.y - 5.f), std::min(e.lo.z, p.z - 5.f)};
            e.hi = {std::max(e.hi.x, p.x + 5.f), std::max(e.hi.y, p.y + 5.f), std::max(e.hi.z, p.z + 5.f)};
        }
    }

    // heat input of cylinder c's gas: hot when it left the cylinder hot and lots of it flows
    static double heatTarget(const engineSim &sim, int c) {
        double flow = std::clamp(std::fabs(sim.rpm()) / (0.7 * sim.P.revLimit), 0.0, 1.0);
        return std::clamp(1.6 * sim.cyl[c].glowEvo * std::sqrt(flow), 0.0, 1.0);
    }

    // advance the thermal state by the simulated time since the last call
    // pops: the sound's crackle counter and the strength of the last one (engineSound::popCount)
    void update(const engineSim &sim, int pops = 0, double popAmp = 1.0) {
        bool first = lastTime < 0.0;
        double dt = first ? 0.0 : std::clamp(sim.time - lastTime, 0.0, 0.5);
        lastTime = sim.time;
        if (first && sim.time > 1.0) dt = 1e3;  // after a headless --simtime run: start settled
        if (heat.size() != sim.cyl.size()) heat.assign(sim.cyl.size(), 0.0);
        for (int c = 0; c < sim.cylCount(); c++) {
            double want = heatTarget(sim, c);
            double tau = want > heat[c] ? 4.0 : 10.0;  // heats in seconds, cools slower
            heat[c] += (want - heat[c]) * (1.0 - std::exp(-dt / tau));
        }
        // the flap servo, as in engineSound (the detailed voice)
        const soundParams &sp = sim.P.sound;
        double rpm = std::fabs(sim.rpm()), demand = std::max(sim.pedal, sim.blip);
        double want = std::clamp((rpm - sp.flapOpenRpm) / sp.flapSpan + 0.5, 0.0, 1.0);
        want = std::max(want, std::clamp((demand - 0.25) / 0.25, 0.0, 1.0) * std::clamp((rpm - 2500.0) / 1500.0, 0.0, 1.0));
        flap += (want - flap) * (1.0 - std::exp(-dt / 0.3));
        crackle(std::min(dt, 0.05), lastPops < 0 ? 0 : std::max(0, pops - lastPops), popAmp);
        lastPops = pops;
    }

    // flame and sparks from each new pop, then the sparks fly (simulated time, so slow motion slows them)
    void crackle(double dt, int newPops, double amp) {
        if (outlets.empty()) return;
        fireAge += dt;
        if (newPops > 0) {
            fireAge = 0.0;
            fireAmp = std::clamp(amp, 0.3, 1.4);
            // most overrun pops only flame; some, and every hard bark, also spit a few sparks (glowing
            // carbon flakes torn off the pipe)
            int n = 0;
            for (int k = 0; k < std::min(newPops, 3); k++)
                if (fireAmp > 1.0 || rand01() < 0.3f) n += static_cast<int>(3 + 6 * fireAmp);
            for (const outletSpec &o : outlets) {
                vec3 ref = std::fabs(o.dir.y) < 0.9f ? vec3{0.f, 1.f, 0.f} : vec3{1.f, 0.f, 0.f};
                vec3 u = normalize(cross(o.dir, ref)), w = cross(o.dir, u);
                for (int k = 0; k < n && sparks.size() < 400; k++) {
                    float a = 2.f * PI * rand01(), spread = 0.45f * rand01();
                    vec3 d = normalize(o.dir + (u * std::cos(a) + w * std::sin(a)) * spread);
                    float speed = (1500.f + 3000.f * rand01()) * static_cast<float>(fireAmp);
                    vec3 off = (u * std::cos(a) + w * std::sin(a)) * (o.r * 0.6f * rand01());
                    sparks.push_back({o.at + off, d * speed, 0.f, 0.12f + 0.3f * rand01()});
                }
            }
        }
        const float G = 9810.f;  // mm/s^2, mesh +y is down
        for (spark &s : sparks) {
            float h = static_cast<float>(dt);
            s.v = s.v * std::exp(-1.5f * h);  // air drag on a tiny glowing fleck
            s.v.y += G * h;
            s.p = s.p + s.v * h;
            s.age += h;
        }
        sparks.erase(std::remove_if(sparks.begin(), sparks.end(), [](const spark &s) { return s.age >= s.life; }), sparks.end());
    }

    // a rough header temperature for the HUD, deg C (the hottest runner, at the port)
    int headerC() const {
        double h = heat.empty() ? 0.0 : *std::max_element(heat.begin(), heat.end());
        return static_cast<int>(std::lround(40.0 + 860.0 * h));
    }

    // gas slug speed in a runner: the exhaust stroke pushes the bore's gas through the smaller pipe;
    // blowdown makes it quicker still. Clamped so a slug is still seen at the starter's rpm
    double slugSpeed(const engineSim &sim, int c) const {
        double stroke = 2.0 * sim.cyl[c].r, pistonSpeed = 2.0 * stroke * std::fabs(sim.rpm()) / 60.0;
        double areaRatio = std::pow(sim.P.bore / (2.0 * runnerR[c] * 1e-3), 2.0);
        return std::max(15.0, 1.6 * pistonSpeed * areaRatio) * 1000.0;  // mm/s
    }

    // where the last pulse of cylinder c is now (mm from its port) and how strong it is
    void pulse(const engineSim &sim, int c, double &front, double &amp) const {
        front = -1e9;
        amp = 0.0;
        if (std::fabs(sim.omega) < 1.0) return;
        double since = wrapCycle(sim.phase(c) - sim.P.evo);  // crank angle since the valve opened
        front = slugSpeed(sim, c) * since / std::fabs(sim.omega);
        amp = sim.cyl[c].glowEvo;
    }

    void items(const engineSim &sim, const mat4 &world, std::vector<drawItem> &out) const {
        const int n = static_cast<int>(heat.size());  // build() sized it from the engine
        std::vector<double> front(n), amp(n);
        for (int c = 0; c < n; c++) pulse(sim, c, front[c], amp[c]);
        for (const pipe &pp : pipes) {
            for (const segment &sg : pp.segs) {
                double hot = 0.0, flash = 0.0;
                // the slug thins out where the pipe gets wider (collector, silencer)
                double spread = std::min(1.0, std::pow(runnerR[0] / (0.5 * (pp.spec.r0 + pp.spec.r1)), 2.0));
                for (int c = 0; c < n; c++) {
                    if (pp.dist[c] < 0.f) continue;
                    double d = pp.dist[c] + sg.s;                     // mm from the port
                    hot = std::max(hot, heat[c] * std::exp(-d / 1400.0));  // cools along the pipe
                    double w = 70.0 + 0.12 * std::max(0.0, front[c]);       // the slug spreads as it goes
                    double x = (d - front[c]) / w;
                    flash = std::max(flash, spread * amp[c] * std::exp(-x * x) * std::exp(-d / 900.0));
                }
                // steel shows a dull red from ~500 deg C (heat 0.5) up to orange at full heat; the passing
                // gas slug flashes brighter on top
                float t = static_cast<float>(std::clamp(0.65 * std::max((hot - 0.45) / 0.55, 0.0) + 0.9 * flash, 0.0, 1.0));
                const pipeSpec &sp = pp.spec;
                float dim = 1.f - 0.65f * t;  // glowing steel reads as red light on a dark oxide, not tinted chrome
                drawItem it{&sg.mesh, world, {sp.rgba[0] * dim, sp.rgba[1] * dim, sp.rgba[2] * dim, sp.rgba[3]}, sp.rough, sp.metal};
                // blackbody-ish: cherry red, then orange (linear light; the shader adds gamma)
                it.glow[0] = 1.2f * t;
                it.glow[1] = 0.28f * t * t;
                it.glow[2] = 0.03f * t * t * t;
                out.push_back(it);
            }
        }
        for (std::size_t k = 0; k < boxes.size(); k++) {
            const boxSpec &b = boxSpecs[k];
            out.push_back({&boxes[k], world, {b.rgba[0], b.rgba[1], b.rgba[2], b.rgba[3]}, b.rough, b.metal});
        }
        // crackle flame: an orange plume with a hot yellow core, flaring for ~50 ms after each pop
        double fire = fireAmp * std::exp(-fireAge / 0.05);
        if (fire > 0.02 && !outlets.empty()) {
            // the flicker comes from the pop counter and the age, so it is the same every time it is drawn
            float f = static_cast<float>(fire);
            float wob = 0.85f + 0.3f * static_cast<float>(std::fabs(std::sin(lastPops * 12.9898 + fireAge * 211.0)));
            for (const outletSpec &o : outlets) {
                vec3 ref = std::fabs(o.dir.y) < 0.9f ? vec3{0.f, 1.f, 0.f} : vec3{1.f, 0.f, 0.f};
                vec3 u = normalize(cross(o.dir, ref)), w = cross(o.dir, u);
                auto plume = [&](float len, float rad, std::array<float, 3> c) {
                    drawItem it{&flameMesh, world * basisMatrix(o.at, u * rad, o.dir * len, w * rad), {c[0], c[1], c[2], 1.f}, 1.f, 0.f};
                    it.additive = 2;
                    out.push_back(it);
                };
                float len = (90.f + 220.f * f) * wob, rad = o.r * (0.9f + 0.5f * f);
                plume(len, rad, {1.0f * f, 0.32f * f, 0.05f * f});
                plume(0.55f * len, 0.6f * rad, {0.9f * f, 0.75f * f, 0.35f * f});
            }
        }
        // sparks: white-hot streaks along their flight, cooling to orange and red as they fall
        for (const spark &s : sparks) {
            float a = s.age / s.life, speed = length(s.v);
            if (speed < 1.f) continue;
            vec3 d = s.v * (1.f / speed);
            vec3 ref = std::fabs(d.y) < 0.9f ? vec3{0.f, 1.f, 0.f} : vec3{1.f, 0.f, 0.f};
            vec3 u = normalize(cross(d, ref)), w = cross(d, u);
            float len = std::max(4.f, speed * 0.006f), wid = 1.6f;
            float k = 1.6f * std::sqrt(1.f - a);
            drawItem it{&sparkMesh, world * basisMatrix(s.p - d * len, u * wid, w * wid, d * len),
                        {k, k * (0.85f - 0.6f * a), k * (0.45f - 0.45f * a), 1.f}, 1.f, 0.f};
            it.additive = 1;
            out.push_back(it);
        }
        if (flapAt.on) {
            // lever on the shaft end, swinging ~80 degrees from shut to open
            vec3 sh = normalize(flapAt.shaft), u = normalize(flapAt.axis - sh * dot(flapAt.axis, sh));
            vec3 v = cross(sh, u);
            vec3 o = flapAt.at + sh * (flapAt.pipeR + 5.f);
            float a = -0.7f + 1.4f * static_cast<float>(flap);
            mat4 base = world * basisMatrix(o, u, v, sh);
            const std::array<float, 4> DARK{0.12f, 0.12f, 0.13f, 1.f}, ALU{0.62f, 0.62f, 0.64f, 1.f};
            out.push_back({&pulley, base * rotateZ(a), {DARK[0], DARK[1], DARK[2], DARK[3]}, 0.5f, 0.4f});
            out.push_back({&lever, base * translate({0.f, 0.f, 4.f}) * rotateZ(a), {ALU[0], ALU[1], ALU[2], ALU[3]}, 0.35f, 0.8f});
        }
    }
};
