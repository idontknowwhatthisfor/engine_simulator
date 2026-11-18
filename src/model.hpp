// engine meshes (engines/<type>/*.mesh), exact slider-crank kinematics and the kinematics self-test
//
// Engine mesh format ENGMSHv3 (little endian), written by tools/meshlib.py's write_engine for every
// engine (engines/*/build_*.py, engines/inline4_fit/extract_parts.py):
//   char[8]  magic "ENGMSHv3"
//   f32[16]  root matrix (column major, as in glTF)
//   u32      material count, then per material: f32 rgba[4], f32 metallic, f32 roughness, u32 blend
//   f32[3]   crank axis point (x, y, z); the axis direction is +Z
//   u32      cylinder count (any number), then per cylinder 12 x f32: z_centre, gudgeon x / y,
//            crankpin x / y, throw, theta0 (rad, rest crankpin angle from the bore axis), rod length,
//            bank (rad), deck y (cylinder frame), bore (mm), firing lag (deg of the 720 deg cycle
//            behind cylinder 1)
//   u32      metadata count, then per entry: u32 name length, char[length] name, u32 value count,
//            f32[value count]. Read here: banked (1: built by meshlib, pivots and piston / rod
//            vertices in the cylinder frame; 0: the original inline-4 model), zPrimary (the crank's
//            primary gear along the crank, for the bike gearbox), supercharger_at (3 values, mesh space)
//   u32      group count, then per group: u32 kind, i32 cylinder (-1: none), u32 material,
//            u32 vertex count, u32 index count, f32[6 * vertex count] (px py pz nx ny nz), u32[index count]
// Still read: "I4ENGv1" (the inline-4 before v3: 8 values per cylinder, no metadata; the deck is the
// see-through block's lowest point and the firing order 1-3-4-2) and "ENGMSHv2" / "BMWS1KRR" (the
// banked builders before v3: 12 values per cylinder, no metadata).
#pragma once
#include "common.hpp"

// ---------------------------------------------------------------------------------------------
// engine.mesh
// ---------------------------------------------------------------------------------------------
enum partKind : std::uint32_t { STATIC = 0, CRANK = 1, PISTON = 2, ROD = 3, BLOCK = 4 };

struct material {
    float rgba[4];
    float metallic;
    float roughness;
    bool blend;
};

struct cylinderGeom {
    float zCentre;
    float gudgeonX, gudgeonY;   // wrist pin centre at rest
    float crankpinX, crankpinY; // crankpin centre at rest
    float throw_;               // crank radius r
    float theta0;               // rest crankpin angle from the bore axis (rad)
    float rodL;
    // v2 meshes (V engines): pivots above and the piston / rod vertices are in the cylinder's own
    // frame (bore toward -Y), turned into mesh space by R_z(bank) about the crank axis
    float bank = 0.f;           // rad
    float deckY = 0.f;          // block deck, local frame
    float bore = 85.f;          // mm
    float firingLagDeg = 0.f;   // degrees of the 720 deg cycle this cylinder runs behind cylinder 1
};

struct group {
    partKind kind;
    int cyl;
    std::uint32_t mat;
    std::vector<float> verts;  // px py pz nx ny nz
    std::vector<std::uint32_t> idx;
    GLuint vbo = 0, ibo = 0;
};

struct engineModel {
    // banked: a v2 mesh (engines/ducati_v4/build_v4.py, engines/bmw_s1000rr/build_s1000rr.py) with a
    // bank angle, deck, bore and firing lag per cylinder, and piston / rod vertices in the cylinder's frame
    bool banked = false;
    mat4 root;
    std::vector<material> mats;
    vec3 crankAxis;
    std::vector<cylinderGeom> cyls;
    std::vector<group> groups;
    vec3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};  // world-space bounds
    std::vector<std::pair<std::string, std::vector<float>>> meta;  // v3 named values

    // a named value from the mesh (nullptr when the file has none)
    const std::vector<float> *metaValues(const std::string &name) const {
        for (const auto &m : meta)
            if (m.first == name) return &m.second;
        return nullptr;
    }
};

class reader {
public:
    explicit reader(const std::string &path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("cannot open " + path);
        data.assign(std::istreambuf_iterator<char>(f), {});
    }
    template <class T> T get() {
        T v;
        need(sizeof(T));
        std::memcpy(&v, data.data() + pos, sizeof(T));
        pos += sizeof(T);
        return v;
    }
    template <class T> void getArray(T *out, std::size_t n) {
        need(sizeof(T) * n);
        std::memcpy(out, data.data() + pos, sizeof(T) * n);
        pos += sizeof(T) * n;
    }
    bool atEnd() const { return pos == data.size(); }
    std::string getString(std::size_t n) {
        need(n);
        std::string s(data.data() + pos, n);
        pos += n;
        return s;
    }

private:
    void need(std::size_t n) const {
        if (pos + n > data.size()) throw std::runtime_error("engine.mesh truncated");
    }
    std::vector<char> data;
    std::size_t pos = 0;
};

// cylinder frame -> mesh space: turn about the crank axis by the bank angle (identity for the inline-4)
mat4 bankMatrix(const engineModel &e, int c) {
    vec3 axis{e.crankAxis.x, e.crankAxis.y, 0.f};
    return translate(axis) * rotateZ(e.cyls[c].bank) * translate(axis * -1.f);
}

engineModel loadEngine(const std::string &path) {
    reader rd(path);
    char magic[8];
    rd.getArray(magic, 8);
    engineModel e;
    const bool v3 = std::memcmp(magic, "ENGMSHv3", 8) == 0;
    const bool v2 = std::memcmp(magic, "ENGMSHv2", 8) == 0 || std::memcmp(magic, "BMWS1KRR", 8) == 0;  // same v2 layout
    if (!v3 && !v2 && std::memcmp(magic, "I4ENGv1", 8) != 0) throw std::runtime_error("bad engine mesh magic");
    e.banked = v2;  // v3: from its metadata below

    rd.getArray(e.root.m.data(), 16);
    e.mats.resize(rd.get<std::uint32_t>());
    for (auto &m : e.mats) {
        rd.getArray(m.rgba, 4);
        m.metallic = rd.get<float>();
        m.roughness = rd.get<float>();
        m.blend = rd.get<std::uint32_t>() != 0;
    }
    rd.getArray(&e.crankAxis.x, 3);
    e.cyls.resize(rd.get<std::uint32_t>());
    for (auto &c : e.cyls) {
        rd.getArray(&c.zCentre, 8);
        if (v2 || v3) {
            rd.getArray(&c.bank, 3);
            c.firingLagDeg = rd.get<float>();
        }
    }
    if (v3) {
        e.meta.resize(rd.get<std::uint32_t>());
        for (auto &m : e.meta) {
            m.first = rd.getString(rd.get<std::uint32_t>());
            m.second.resize(rd.get<std::uint32_t>());
            rd.getArray(m.second.data(), m.second.size());
        }
        const std::vector<float> *b = e.metaValues("banked");
        e.banked = b && !b->empty() && (*b)[0] != 0.f;
    }
    if (e.cyls.size() != 4) throw std::runtime_error("the simulator needs 4 cylinders");
    if (!v2 && !v3) {
        // the inline-4 before v3: firing order 1-3-4-2
        const float LAG[4] = {0.f, 540.f, 180.f, 360.f};
        for (int c = 0; c < 4; c++) e.cyls[c].firingLagDeg = LAG[c];
    }
    e.groups.resize(rd.get<std::uint32_t>());
    for (auto &g : e.groups) {
        g.kind = static_cast<partKind>(rd.get<std::uint32_t>());
        g.cyl = rd.get<std::int32_t>();
        g.mat = rd.get<std::uint32_t>();
        std::uint32_t nv = rd.get<std::uint32_t>(), ni = rd.get<std::uint32_t>();
        g.verts.resize(nv * 6);
        g.idx.resize(ni);
        rd.getArray(g.verts.data(), g.verts.size());
        rd.getArray(g.idx.data(), g.idx.size());
        if (g.mat >= e.mats.size()) throw std::runtime_error("bad material index");
        if ((g.kind == PISTON || g.kind == ROD) && (g.cyl < 0 || g.cyl >= 4)) throw std::runtime_error("bad cylinder index");
        mat4 toWorld = e.root;
        if (g.kind == PISTON || g.kind == ROD) toWorld = e.root * bankMatrix(e, g.cyl);
        for (std::uint32_t k = 0; k < nv; k++) {
            vec3 p = transformPoint(toWorld, {g.verts[k * 6], g.verts[k * 6 + 1], g.verts[k * 6 + 2]});
            e.lo = {std::min(e.lo.x, p.x), std::min(e.lo.y, p.y), std::min(e.lo.z, p.z)};
            e.hi = {std::max(e.hi.x, p.x), std::max(e.hi.y, p.y), std::max(e.hi.z, p.z)};
        }
    }
    if (!rd.atEnd()) throw std::runtime_error("engine.mesh has trailing data");
    if (!v2 && !v3) {
        // the inline-4 deck is the lowest point of the see-through block
        float deck = 1e9f;
        for (const group &g : e.groups)
            if (g.kind == BLOCK && e.mats[g.mat].blend)
                for (std::size_t v = 0; v < g.verts.size(); v += 6) deck = std::min(deck, g.verts[v + 1]);
        for (auto &c : e.cyls) c.deckY = deck;
    }
    return e;
}

// the gaps between successive firings, in firing order from cylinder 1, deg (they add up to 720)
std::vector<double> firingIntervals(const engineModel &e) {
    std::vector<double> lags;
    for (const cylinderGeom &c : e.cyls) lags.push_back(c.firingLagDeg);
    std::sort(lags.begin(), lags.end());
    std::vector<double> gaps;
    for (std::size_t k = 0; k < lags.size(); k++)
        gaps.push_back(k + 1 < lags.size() ? lags[k + 1] - lags[k] : 720.0 - lags[k] + lags[0]);
    return gaps;
}

// where the crank's primary gear sits along the crank (mesh z, mm), for the bike gearbox: the mesh's
// zPrimary, or for a mesh from before v3 the middle of the primary hub, 11 mm in from the crank's
// right-hand end (tools/meshlib.py build_crank)
float primaryGearZ(const engineModel &e) {
    if (const std::vector<float> *z = e.metaValues("zPrimary"); z && !z->empty()) return (*z)[0];
    float hi = -1e9f;
    for (const group &g : e.groups)
        if (g.kind == CRANK)
            for (std::size_t v = 0; v < g.verts.size(); v += 6) hi = std::max(hi, g.verts[v + 2]);
    return hi - 11.f;
}

// a named point from the mesh (mesh space); throws, naming the point, when the mesh has none
vec3 meshPoint(const engineModel &e, const std::string &name) {
    const std::vector<float> *p = e.metaValues(name);
    if (!p || p->size() != 3) throw std::runtime_error("the engine mesh has no " + name + ": rebuild it (make)");
    return {(*p)[0], (*p)[1], (*p)[2]};
}

// the firing order from the cylinders' lags, e.g. "1-2-4-3", with ", 180 deg apart" when they are
// evenly spaced over the 720 deg cycle
std::string firingOrder(const engineModel &e) {
    std::vector<int> order(e.cyls.size());
    for (std::size_t c = 0; c < order.size(); c++) order[c] = static_cast<int>(c);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return e.cyls[a].firingLagDeg < e.cyls[b].firingLagDeg; });
    std::string s;
    for (int c : order) s += (s.empty() ? "" : "-") + std::to_string(c + 1);
    std::vector<double> gaps = firingIntervals(e);
    bool even = !gaps.empty();
    for (double g : gaps) even &= std::fabs(g - gaps[0]) < 0.01;
    if (even && gaps.size() > 1) {
        char buf[48];
        std::snprintf(buf, sizeof buf, ", %g deg apart", gaps[0]);
        s += buf;
    }
    return s;
}

// ---------------------------------------------------------------------------------------------
// kinematics: exact slider-crank, all in mesh space (crank axis = Z, bores point toward -Y)
// ---------------------------------------------------------------------------------------------
// The model's rest pose is not TDC, so everything is a delta from rest: `delta` is the crank
// rotation away from the rest pose. R_z(+delta) in mesh space turns the crank clockwise seen
// from the front (timing snout), because the root matrix flips Y and Z.

struct cylinderPose {
    vec3 crankpin;   // crankpin centre now
    vec3 gudgeon;    // wrist pin centre now
    float rodAngle;  // direction of gudgeon -> crankpin, rad
};

cylinderPose solveCylinder(const engineModel &e, const cylinderGeom &c, float delta) {
    float ax = e.crankAxis.x, ay = e.crankAxis.y;
    float dx = c.crankpinX - ax, dy = c.crankpinY - ay;
    float cs = std::cos(delta), sn = std::sin(delta);
    vec3 pin{ax + dx * cs - dy * sn, ay + dx * sn + dy * cs, c.zCentre};
    // the wrist pin slides on the bore line x = gudgeonX and stays rod length L from the crankpin
    float ox = c.gudgeonX - pin.x;
    vec3 g{c.gudgeonX, pin.y - std::sqrt(c.rodL * c.rodL - ox * ox), c.zCentre};
    return {pin, g, std::atan2(pin.y - g.y, pin.x - g.x)};
}

// crank angle of cylinder 1 measured from its TDC, in degrees 0..360
float crankDegrees(const engineModel &e, float delta) {
    float d = (e.cyls[0].theta0 + delta) * 180.f / PI;
    d = std::fmod(d, 360.f);
    return d < 0.f ? d + 360.f : d;
}

// model matrix for every draw group at crank offset `delta`
void poseEngine(const engineModel &e, float delta, std::vector<mat4> &modelOf) {
    vec3 axis{e.crankAxis.x, e.crankAxis.y, 0.f};
    mat4 crank = translate(axis) * rotateZ(delta) * translate(axis * -1.f);
    std::vector<mat4> piston(e.cyls.size()), rod(e.cyls.size());
    for (std::size_t c = 0; c < e.cyls.size(); c++) {
        cylinderPose rest = solveCylinder(e, e.cyls[c], 0.f);
        cylinderPose now = solveCylinder(e, e.cyls[c], delta);
        mat4 bank = bankMatrix(e, static_cast<int>(c));
        piston[c] = bank * translate(now.gudgeon - rest.gudgeon);
        rod[c] = bank * translate(now.gudgeon) * rotateZ(now.rodAngle - rest.rodAngle) * translate(rest.gudgeon * -1.f);
    }
    modelOf.resize(e.groups.size());
    for (std::size_t k = 0; k < e.groups.size(); k++) {
        const group &g = e.groups[k];
        mat4 local = identity();
        if (g.kind == CRANK) local = crank;
        else if (g.kind == PISTON) local = piston[g.cyl];
        else if (g.kind == ROD) local = rod[g.cyl];
        modelOf[k] = e.root * local;
    }
}

// numeric checks over two full turns, no window needed (--selftest). pattern: the published firing
// intervals (bikeSpec::pattern) the mesh's lags must give, in any rotation; empty: not checked
int selfTest(const engineModel &e, const char *name, const std::vector<double> &pattern = {}) {
    vec3 axis{e.crankAxis.x, e.crankAxis.y, 0.f};
    std::vector<mat4> modelOf;
    float worstBigEnd = 0.f, worstSmallEnd = 0.f, worstPistonX = 0.f, worstRestErr = 0.f, worstPair = 0.f;
    std::vector<float> yMin(e.cyls.size(), 1e9f), yMax(e.cyls.size(), -1e9f);

    // rest pose must be the stored geometry for every group (cylinder parts sit in their bank frame)
    poseEngine(e, 0.f, modelOf);
    for (std::size_t k = 0; k < modelOf.size(); k++) {
        const group &g = e.groups[k];
        mat4 want = (g.kind == PISTON || g.kind == ROD) ? e.root * bankMatrix(e, g.cyl) : e.root;
        for (int j = 0; j < 16; j++) worstRestErr = std::max(worstRestErr, std::fabs(modelOf[k].m[j] - want.m[j]));
    }

    // the firing lags must agree with the geometry: every cylinder's crank angle from its own TDC is
    // cylinder 1's minus its lag (mod 360), or the gas forces would push on the wrong strokes
    float worstLag = 0.f;
    for (const cylinderGeom &cg : e.cyls) {
        float d = std::remainder(e.cyls[0].theta0 - cg.firingLagDeg * PI / 180.f - cg.theta0, 2.f * PI);
        worstLag = std::max(worstLag, std::fabs(d) * 180.f / PI);
    }

    // crank vertices (rest pose, mesh space) for the skirt clearance sweep
    std::vector<vec3> crankPts;
    for (const group &g : e.groups)
        if (g.kind == CRANK)
            for (std::size_t v = 0; v < g.verts.size(); v += 6) crankPts.push_back({g.verts[v], g.verts[v + 1], g.verts[v + 2]});
    // piston skirt bottom (max local y) and crown (min local y) at rest, per cylinder
    std::vector<float> skirtRest(e.cyls.size(), -1e9f), crownRest(e.cyls.size(), 1e9f);
    for (const group &g : e.groups)
        if (g.kind == PISTON)
            for (std::size_t v = 0; v < g.verts.size(); v += 6) {
                crownRest[g.cyl] = std::min(crownRest[g.cyl], g.verts[v + 1]);
                skirtRest[g.cyl] = std::max(skirtRest[g.cyl], g.verts[v + 1]);
            }
    float worstSkirt = -1e9f;  // > 0 means the crank pokes into a piston's footprint below the skirt

    bool inline4 = true;  // all bores in one bank (the S 1000 RR's leans forward, still inline)
    for (const cylinderGeom &cg : e.cyls) inline4 &= cg.bank == e.cyls[0].bank;

    for (int step = 0; step <= 1440; step++) {
        float delta = step * PI / 360.f;  // 0.5 degree steps over 720 degrees
        mat4 crank = translate(axis) * rotateZ(delta) * translate(axis * -1.f);
        std::vector<vec3> gNow(e.cyls.size());
        for (std::size_t c = 0; c < e.cyls.size(); c++) {
            const cylinderGeom &cg = e.cyls[c];
            cylinderPose rest = solveCylinder(e, cg, 0.f);
            cylinderPose now = solveCylinder(e, cg, delta);
            mat4 piston = translate(now.gudgeon - rest.gudgeon);
            mat4 rod = translate(now.gudgeon) * rotateZ(now.rodAngle - rest.rodAngle) * translate(rest.gudgeon * -1.f);
            // big end carried by the rod must sit on the crankpin carried by the crank (both in the
            // cylinder frame: the crank turns about the same axis as the bank rotation)
            vec3 bigEndOnRod = transformPoint(rod, rest.crankpin);
            vec3 pinOnCrank = transformPoint(crank, rest.crankpin);
            worstBigEnd = std::max(worstBigEnd, length(bigEndOnRod - pinOnCrank));
            // small end carried by the rod must sit on the wrist pin carried by the piston
            vec3 smallEndOnRod = transformPoint(rod, rest.gudgeon);
            vec3 pinOnPiston = transformPoint(piston, rest.gudgeon);
            worstSmallEnd = std::max(worstSmallEnd, length(smallEndOnRod - pinOnPiston));
            // the piston only slides along the bore
            worstPistonX = std::max(worstPistonX, std::fabs(now.gudgeon.x - rest.gudgeon.x));
            yMin[c] = std::min(yMin[c], now.gudgeon.y);
            yMax[c] = std::max(yMax[c], now.gudgeon.y);
            gNow[c] = now.gudgeon;

            // crank vs this piston's skirt, every 2 degrees, in the cylinder frame
            if (step % 4 == 0) {
                float skirt = skirtRest[c] + (now.gudgeon.y - rest.gudgeon.y);
                float r = 0.5f * cg.bore;
                float cb = std::cos(-cg.bank), sb = std::sin(-cg.bank);
                for (const vec3 &p0 : crankPts) {
                    vec3 p = transformPoint(crank, p0) - axis;
                    float lx = p.x * cb - p.y * sb, ly = p.x * sb + p.y * cb;
                    float dz = p.z - cg.zCentre;
                    if (lx * lx + dz * dz < r * r && ly < 0.f) worstSkirt = std::max(worstSkirt, skirt - (ly + axis.y));
                }
            }
        }
        // flat-plane crank: 1 and 4 move together, 2 and 3 move together
        if (inline4) {
            worstPair = std::max(worstPair, std::fabs(gNow[0].y - gNow[3].y));
            worstPair = std::max(worstPair, std::fabs(gNow[1].y - gNow[2].y));
        }
    }
    float sweep = 0.f;
    for (const vec3 &p : crankPts) sweep = std::max(sweep, std::hypot(p.x - axis.x, p.y - axis.y));

    auto line = [](const char *name, float v, float limit, const char *unit) {
        bool ok = v <= limit;
        std::printf("  %-44s %10.5f %-3s %s\n", name, v, unit, ok ? "ok" : "FAIL");
        return ok;
    };
    std::printf("kinematics self-test, %s (720 deg, 0.5 deg steps)\n", name);
    bool ok = true;
    ok &= line("rest pose == original model (max matrix err)", worstRestErr, 1e-5f, "");
    ok &= line("rod big end vs crankpin (max gap)", worstBigEnd, 0.1f, "mm");
    ok &= line("rod small end vs wrist pin (max gap)", worstSmallEnd, 0.1f, "mm");
    ok &= line("piston sideways motion (max)", worstPistonX, 1e-3f, "mm");
    ok &= line("firing lags vs crank geometry (max)", worstLag, 0.1f, "deg");
    if (!pattern.empty()) {
        // the mesh's intervals against the expected ones, best rotation (the pattern may start at any firing)
        std::vector<double> gaps = firingIntervals(e);
        double best = 1e9;
        if (gaps.size() == pattern.size())
            for (std::size_t r = 0; r < gaps.size(); r++) {
                double worst = 0.0;
                for (std::size_t k = 0; k < gaps.size(); k++) worst = std::max(worst, std::fabs(gaps[(k + r) % gaps.size()] - pattern[k]));
                best = std::min(best, worst);
            }
        ok &= line("firing intervals vs the published pattern (max)", static_cast<float>(best), 0.1f, "deg");
    }
    if (inline4) ok &= line("cyl 1/4 and 2/3 position mismatch (max)", worstPair, 1e-3f, "mm");
    // the inline-4 skirts sit between the counterweights, so the footprint test does not apply there
    if (e.banked) ok &= line("crank inside a piston skirt (worst, < 0 = clear)", worstSkirt, -1.f, "mm");

    for (std::size_t c = 0; c < e.cyls.size(); c++) {
        const cylinderGeom &cg = e.cyls[c];
        // bore points toward -Y: TDC is the most negative wrist pin y
        float crownTdc = crownRest[c] + (yMin[c] - cg.gudgeonY);
        float skirtBdc = skirtRest[c] + (yMax[c] - cg.gudgeonY);
        float stroke = yMax[c] - yMin[c];
        std::printf("  cyl %zu: bank %+5.1f  stroke %.2f (2r = %.2f)  crown at TDC %.1f below deck  skirt at BDC %.1f from axis (crank sweep %.1f)\n",
                    c + 1, cg.bank * 180.f / PI, stroke, 2.f * cg.throw_, crownTdc - cg.deckY, axis.y - skirtBdc, sweep);
        ok &= std::fabs(stroke - 2.f * cg.throw_) < 0.05f;
        ok &= crownTdc >= cg.deckY;  // the crown never pokes out of the block
    }
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// pixel comparison of two PNGs (--compare a.png b.png)
int compareImages(const std::string &a, const std::string &b) {
    sf::Image ia, ib;
    if (!ia.loadFromFile(a) || !ib.loadFromFile(b)) return 2;
    if (ia.getSize() != ib.getSize()) {
        std::printf("size differs\n");
        return 1;
    }
    std::size_t diff = 0;
    int worst = 0;
    unsigned x0 = ~0u, y0 = ~0u, x1 = 0, y1 = 0;
    const std::uint8_t *pa = ia.getPixelsPtr(), *pb = ib.getPixelsPtr();
    std::size_t n = std::size_t(ia.getSize().x) * ia.getSize().y;
    for (std::size_t k = 0; k < n; k++) {
        // ignore the HUD text rows (top-left stats, bottom help / credit line)
        unsigned row = unsigned(k / ia.getSize().x);
        if (row < 140 || row + 35 >= ia.getSize().y) continue;
        int d = 0;
        // RGB only: older screenshots carried blend alpha in the framebuffer, which is not what is shown
        for (int ch = 0; ch < 3; ch++) d = std::max(d, std::abs(int(pa[k * 4 + ch]) - int(pb[k * 4 + ch])));
        if (d) {
            diff++;
            unsigned x = unsigned(k % ia.getSize().x), y = unsigned(k / ia.getSize().x);
            x0 = std::min(x0, x); y0 = std::min(y0, y); x1 = std::max(x1, x); y1 = std::max(y1, y);
        }
        worst = std::max(worst, d);
    }
    std::printf("%zu of %zu pixels differ, max channel diff %d", diff, n, worst);
    if (diff) std::printf(", in box x %u..%u y %u..%u", x0, x1, y0, y1);
    std::printf("\n");
    return diff == 0 ? 0 : 1;
}

