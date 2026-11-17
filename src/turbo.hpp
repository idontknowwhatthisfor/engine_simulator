// the turbocharger (--turbo yes): the Sketchfab "Car Turbocharger" model, split by
// engines/inline4_fit/build_turbo.py into turbo.mesh. The housings are drawn see-through like the
// engine block (hidden with it, G), so the compressor wheel, shaft, bearing and turbine wheel show.
// The rotor spins up from the exhaust gas: the shaft speed follows the gas flow (revs x manifold
// pressure) with spool lag. It is drawn only; the engine physics has no boost.
#pragma once
#include "exhaust.hpp"

struct turboModel {
    enum : std::uint32_t { HOUSING = 0, ROTOR = 1, BEARING = 2 };
    struct part {
        std::uint32_t kind;
        float rgba[4];
        float rough, metal;
        meshBuf mesh;
    };
    std::vector<part> parts;
    mat4 place = identity();       // turbo frame -> mesh space
    vec3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};  // mesh-space bounds

    double shaftRpm = 0.0;         // simulated turbo shaft speed
    double angle = 0.0;            // drawn rotor angle, rad
    double lastTime = -1.0;

    static constexpr double MAX_RPM = 160000.0;
    static constexpr double IDLE_RPM = 12000.0;
    // the drawn rotor turns at most this fast: the real 100k+ rpm would just strobe at the frame rate
    static constexpr double DRAW_MAX_REV_S = 12.0;

    void load(const std::string &path, const turboSpec &s) {
        reader rd(path);
        char magic[8];
        rd.getArray(magic, 8);
        if (std::memcmp(magic, "TURBOv1", 8) != 0) throw std::runtime_error("bad turbo mesh magic");
        place = basisMatrix(s.at, s.x, s.y, s.z);
        parts.resize(rd.get<std::uint32_t>());
        for (part &p : parts) {
            p.kind = rd.get<std::uint32_t>();
            rd.getArray(p.rgba, 4);
            p.rough = rd.get<float>();
            p.metal = rd.get<float>();
            std::uint32_t nv = rd.get<std::uint32_t>(), ni = rd.get<std::uint32_t>();
            p.mesh.v.resize(nv * 6);
            p.mesh.i.resize(ni);
            rd.getArray(p.mesh.v.data(), p.mesh.v.size());
            rd.getArray(p.mesh.i.data(), p.mesh.i.size());
            for (std::uint32_t idx : p.mesh.i)
                if (idx >= nv) throw std::runtime_error("turbo mesh: bad index");
            for (std::uint32_t k = 0; k < nv; k++) {
                vec3 q = transformPoint(place, {p.mesh.v[k * 6], p.mesh.v[k * 6 + 1], p.mesh.v[k * 6 + 2]});
                lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
                hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
            }
        }
        if (!rd.atEnd()) throw std::runtime_error("turbo mesh has trailing data");
    }

    void upload(renderer &ren) {
        for (part &p : parts) ren.upload(p.mesh);
    }

    // mesh -> world bounds, so the camera framing and the floor make room for it
    void growBounds(engineModel &e) const {
        for (int k = 0; k < 8; k++) {
            vec3 p = transformPoint(e.root, {k & 1 ? hi.x : lo.x, k & 2 ? hi.y : lo.y, k & 4 ? hi.z : lo.z});
            e.lo = {std::min(e.lo.x, p.x - 5.f), std::min(e.lo.y, p.y - 5.f), std::min(e.lo.z, p.z - 5.f)};
            e.hi = {std::max(e.hi.x, p.x + 5.f), std::max(e.hi.y, p.y + 5.f), std::max(e.hi.z, p.z + 5.f)};
        }
    }

    // spool: the shaft speed chases a target set by the exhaust gas flow (revs and manifold
    // pressure), quicker to spin up than to run down
    void update(const engineSim &sim) {
        bool first = lastTime < 0.0;
        double dt = first ? 0.0 : std::clamp(sim.time - lastTime, 0.0, 0.5);
        lastTime = sim.time;
        double flow = std::clamp(std::fabs(sim.rpm()) / sim.P.revLimit, 0.0, 1.0) * std::clamp(sim.pMan / sim.P.pAtm, 0.0, 1.2);
        double want = std::fabs(sim.rpm()) < 100.0 ? 0.0 : IDLE_RPM + (MAX_RPM - IDLE_RPM) * std::pow(std::min(flow * 1.25, 1.0), 1.6);
        if (first) shaftRpm = sim.time > 1.0 ? want : 0.0;  // after a headless --simtime run: start spooled
        double tau = want > shaftRpm ? 0.7 : 1.8;
        shaftRpm += (want - shaftRpm) * (1.0 - std::exp(-dt / tau));
        // drawn speed in proportion to the real one (full drawn speed from half the top shaft speed), so spool-up shows
        double revS = DRAW_MAX_REV_S * std::min(1.0, shaftRpm / (0.5 * MAX_RPM));
        angle = std::fmod(angle + 2.0 * PI * revS * dt, 2.0 * PI);
    }

    // ghost: as the engine block (0 = see-through housings, 1 = housings hidden)
    void items(const mat4 &world, int ghost, std::vector<drawItem> &out) const {
        mat4 base = world * place;
        for (const part &p : parts) {
            if (p.kind == HOUSING && ghost != 0) continue;
            mat4 m = p.kind == ROTOR ? base * rotateX(static_cast<float>(angle)) : base;
            out.push_back({&p.mesh, m, {p.rgba[0], p.rgba[1], p.rgba[2], p.rgba[3]}, p.rough, p.metal});
        }
    }
};
