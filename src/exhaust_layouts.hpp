// exhaust layouts more than one engine uses (src/exhaust.hpp draws them). An engine's exhaust function
// calls one with its own options, or builds its own layout from scratch.
#pragma once
#include "exhaust.hpp"

// what differs between the engines that use the inline superbike 4-2-1 layout below
struct superbike421Options {
    std::array<float, 4> canRgba{0.68f, 0.68f, 0.70f, 1.f};  // the silencer
    float canR0 = 52.f, canR1 = 50.f;                        // its radius at the link pipe / at the end
};

// an inline-4 superbike's 4-2-1 system, for a block inclined forward with the front at mesh +x and
// "up" -y: titanium headers from the front of the head, curving down in front of the crankcase and
// under the engine; 1+4 and 2+3 merge (360 deg apart in a 1-2-4-3 order), then a collector, a link
// pipe with the exhaust flap, and a short silencer rising on the right (+z) behind the gearbox, with
// a carbon end cap. Pipes in this order: the four headers, the two pair merges, collector, link
// pipe, silencer, end cap.
exhaustLayout superbike421Exhaust(const engineModel &e, const superbike421Options &o = {}) {
    exhaustLayout L;
    const std::array<float, 4> TI{0.60f, 0.57f, 0.52f, 1.f}, CARBON{0.10f, 0.10f, 0.11f, 1.f};
    auto pipe = [&](std::vector<vec3> pts, float r0, float r1, int cyl, std::vector<int> from, std::array<float, 4> c,
                    bool caps = false) {
        pipeSpec p;
        p.pts = pts; p.r0 = r0; p.r1 = r1; p.cyl = cyl; p.from = from; p.rgba = c;
        p.rough = c == CARBON ? 0.5f : 0.3f;
        p.metal = c == CARBON ? 0.2f : 0.9f;
        p.capStart = p.capEnd = caps;
        L.pipes.push_back(p);
        return static_cast<int>(L.pipes.size()) - 1;
    };
    auto port = [&](int c) { return cylPoint(e, c, 0.5f * e.cyls[c].bore + 20.f, e.cyls[c].deckY - 41.f); };
    auto out = [&](int c) { return port(c) + cylDir(e, c, 1.f, 0.f) * 40.f; };
    // outer pair 1+4 runs wide and low to its merge (x 175); inner pair 2+3 merges higher (x 120)
    for (int c : {0, 3}) {
        float z = e.cyls[c].zCentre, sg = z < 0.f ? -1.f : 1.f;
        pipe({port(c), out(c), {235.f, -60.f, z}, {245.f, 50.f, z}, {225.f, 160.f, 110.f * sg}, {200.f, 225.f, 50.f * sg},
              {175.f, 235.f, 0.f}}, 18.f, 19.f, c, {}, TI);
    }
    for (int c : {1, 2}) {
        float z = e.cyls[c].zCentre, sg = z < 0.f ? -1.f : 1.f;
        pipe({port(c), out(c), {235.f, -60.f, z}, {245.f, 50.f, z}, {215.f, 140.f, 40.f * sg}, {165.f, 185.f, 20.f * sg},
              {120.f, 195.f, 0.f}}, 18.f, 19.f, c, {}, TI);
    }
    int a = pipe({{175.f, 235.f, 0.f}, {120.f, 250.f, 0.f}, {40.f, 243.f, 0.f}}, 24.f, 25.f, -1, {0, 1}, TI);
    int b = pipe({{120.f, 195.f, 0.f}, {75.f, 215.f, 0.f}, {40.f, 238.f, 0.f}}, 24.f, 25.f, -1, {2, 3}, TI);
    int col = pipe({{40.f, 240.f, 0.f}, {-20.f, 245.f, 0.f}}, 28.f, 30.f, -1, {a, b}, TI);
    int link = pipe({{-20.f, 245.f, 0.f}, {-150.f, 240.f, 40.f}, {-230.f, 215.f, 110.f}, {-262.f, 199.f, 143.f}}, 30.f, 30.f,
                    -1, {col}, TI);
    int can = pipe({{-250.f, 205.f, 140.f}, {-400.f, 110.f, 165.f}}, o.canR0, o.canR1, -1, {link}, TI, true);
    L.pipes[can].rgba = o.canRgba;  // finished like the titanium (rough / metal), in the silencer's colour
    pipe({{-395.f, 113.f, 165.f}, {-435.f, 90.f, 170.f}}, 20.f, 20.f, -1, {can}, CARBON, true);
    L.outlets.push_back({{-435.f, 90.f, 170.f}, {-40.f, -23.f, 5.f}, 20.f});
    L.flap.on = true;
    L.flap.at = {-190.f, 230.f, 75.f};
    L.flap.axis = normalize(vec3{-1.f, -0.25f, 0.8f});
    L.flap.shaft = {0.f, -1.f, 0.f};
    L.flap.pipeR = 30.f;
    return L;
}
