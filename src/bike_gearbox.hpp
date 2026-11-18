// the superbikes' gearbox, drawn beside the crank: primary pair, clutch, the gears, sprocket, drum
#pragma once
#include "engine_type.hpp"
#include "render.hpp"

// ---------------------------------------------------------------------------------------------
// bike gearbox geometry for the superbikes (mesh space, mm, crank axis = z through the origin)
// ---------------------------------------------------------------------------------------------
// Layout of a motorcycle box: a gear on the crank drives the clutch basket (primary pair 30/54) on the
// right-hand end of the main shaft, behind and below the crank. The clutch hub turns the main shaft.
// One always-meshed pair per gear (the real tooth counts when known) sit between the main shaft and the output
// shaft below and behind it; the engaged pair is brass. The chain sprocket is on the left-hand end
// of the output shaft, and a shift drum turns one notch per gear. The Ducati's crank counter-rotates
// with the front at -x; the BMW's turns forward with the front at +x, so its box is mirrored (side -1).
struct bikeGearboxModel {
    int N_PC = 30, N_PB = 54;     // primary: crank gear, clutch basket gear
    float MOD_P = 2.6f;           // primary module
    static constexpr float D = 69.f;            // main shaft to output shaft
    float Z_PRIMARY = 119.f;      // the crank's primary gear (engines/<type>/build_*.py)
    int N_SPROCKET = 16;
    float side = 1.f;             // +1: box toward +x (Ducati), -1: toward -x (BMW)

    void configure(const engineType &t, const drivetrainParams &dp, const engineModel &e) {
        N_PC = dp.primaryIn;
        N_PB = dp.primaryOut;
        N_SPROCKET = dp.sprocketTeeth;
        MOD_P = t.primaryModule;
        Z_PRIMARY = primaryGearZ(e);
        side = t.gearboxSide;
    }
    float mx = 0.f, my = 0.f, ox = 0.f, oy = 0.f, drumX = 0.f, drumY = 0.f;  // shaft centres
    // per gear, index 1..gears (sized by build from drivetrainParams::gears)
    std::vector<int> nIn, nOut;
    std::vector<float> zGear;
    meshBuf crankGear, basket, clutchPack, mainShaft, outShaft, sprocket, drum, housing, cover;
    std::vector<meshBuf> inGear, outGear;

    static float meshPhase(float theta, int nA, int nB) { return gearMeshPhase(theta, nA, nB); }

    // shaft centres: the main shaft behind (+x) and below (+y) the crank, clear of the rear bank
    void layout() {
        float rPc = MOD_P * N_PC / 2.f, rPb = MOD_P * N_PB / 2.f;
        float a = std::atan2(0.5f, 1.f);
        mx = side * (rPc + rPb) * std::cos(a);
        my = (rPc + rPb) * std::sin(a);
        float b = std::atan2(0.8f, 0.6f);
        ox = mx + side * D * std::cos(b);
        oy = my + D * std::sin(b);
        // shift drum beside the shafts, on the side away from the crank, clear of the 1st gear pair
        drumX = mx + side * (0.5f * D * std::cos(b) + 75.f * std::sin(b));
        drumY = my + 0.5f * D * std::sin(b) - 75.f * std::cos(b);
    }

    void build(const drivetrainParams &dp, renderer &ren) {
        layout();
        float rPc = MOD_P * N_PC / 2.f, rPb = MOD_P * N_PB / 2.f;
        crankGear = makeGear(rPc, N_PC, 14.f, 14.f);
        basket = makeGear(rPb, N_PB, 14.f, 46.f);
        clutchPack = makeGear(52.f, 12, 30.f, 16.f);  // the plate stack's tabs show it turning
        mainShaft = makeCylinder(11.f, -112.f, Z_PRIMARY + 47.f);
        outShaft = makeCylinder(13.f, -134.f, 72.f);
        // chain sprocket: 16 teeth on 5/8" (525) chain
        float rS = 15.875f / (2.f * std::sin(PI / N_SPROCKET));
        sprocket = makeGear(rS, N_SPROCKET, 8.f, 16.f);
        drum = makeGear(15.f, 6, 150.f, 6.f);
        // in the real box the tooth counts don't all add up the same and the gears are profile
        // shifted; here each pair just gets pitch radii in its ratio at the common centre distance
        std::size_t n = static_cast<std::size_t>(dp.gears) + 1;
        nIn.assign(n, 0);
        nOut.assign(n, 0);
        zGear.assign(n, 0.f);
        inGear.assign(n, meshBuf());
        outGear.assign(n, meshBuf());
        for (int g = 1; g <= dp.gears; g++) {
            double r = dp.ratios[g];
            // the real tooth counts when they give this ratio, else ~52 teeth per pair from the ratio
            bool known = g < static_cast<int>(dp.teethIn.size()) && dp.teethIn[g] > 0;
            if (known && std::fabs(r - double(dp.teethOut[g]) / dp.teethIn[g]) < 1e-9) {
                nIn[g] = dp.teethIn[g];
                nOut[g] = dp.teethOut[g];
            } else {
                nIn[g] = std::max(12, static_cast<int>(std::lround(52.0 / (1.0 + r))));
                nOut[g] = std::max(12, static_cast<int>(std::lround(nIn[g] * r)));
            }
            float rin = D * nIn[g] / float(nIn[g] + nOut[g]);
            inGear[g] = makeGear(rin, nIn[g], 13.f, 13.f);
            outGear[g] = makeGear(D - rin, nOut[g], 13.f, 15.f);
            zGear[g] = -92.f + 27.f * (g - 1);
        }
        float hx0 = side > 0.f ? mx - 42.f : ox - 56.f, hx1 = side > 0.f ? ox + 56.f : mx + 42.f;
        housing = makeBox({hx0, my - 40.f, -112.f}, {hx1, oy + 58.f, Z_PRIMARY - 7.f});
        cover = makeBox({mx - rPb - 6.f, my - rPb - 6.f, Z_PRIMARY - 7.f}, {mx + rPb + 6.f, my + rPb + 6.f, Z_PRIMARY + 51.f});
        for (meshBuf *m : {&crankGear, &basket, &clutchPack, &mainShaft, &outShaft, &sprocket, &drum, &housing, &cover})
            ren.upload(*m);
        for (int g = 1; g <= dp.gears; g++) {
            ren.upload(inGear[g]);
            ren.upload(outGear[g]);
        }
    }

    // farthest extent behind / below the crank and to the right, for the camera and floor
    float maxX() const { return ox + side * 56.f; }  // signed: the far edge of the box in x
    float maxY() const { return oy + 58.f; }
    float maxZ() const { return Z_PRIMARY + 51.f; }

    void items(const engineSim &sim, float crankDelta, const mat4 &world, int ghost, std::vector<drawItem> &out) const {
        const drivetrain &d = sim.drive;
        auto place = [&](float x, float y, float z, float angle) { return world * translate({x, y, z}) * rotateZ(angle); };
        auto add = [&](const meshBuf &m, const mat4 &model, std::array<float, 4> c, float rough, float metal) {
            out.push_back({&m, model, {c[0], c[1], c[2], c[3]}, rough, metal});
        };
        const std::array<float, 4> STEEL{0.42f, 0.43f, 0.45f, 1.f}, SHAFT{0.55f, 0.55f, 0.58f, 1.f},
            BRASS{0.80f, 0.58f, 0.22f, 1.f}, FRICTION{0.24f, 0.17f, 0.11f, 1.f}, DARK{0.16f, 0.16f, 0.17f, 1.f};
        float thP = std::atan2(my, mx);       // crank -> main shaft
        float thG = std::atan2(oy - my, ox - mx);  // main shaft -> output shaft
        // the crank gear turns with the crank; the basket is driven by it; the clutch hub and main
        // shaft turn as if driven by a crank at the input angle, so slip shows between basket and pack
        float basketA = -crankDelta * N_PC / N_PB + meshPhase(thP, N_PC, N_PB);
        float mainA = -static_cast<float>(d.inputAngle) * N_PC / N_PB + meshPhase(thP, N_PC, N_PB);
        float outA = -static_cast<float>(d.outputAngle);  // turns against the main shaft
        add(crankGear, place(0.f, 0.f, Z_PRIMARY, crankDelta), STEEL, 0.35f, 0.7f);
        add(basket, place(mx, my, Z_PRIMARY, basketA), STEEL, 0.35f, 0.7f);
        float lift = static_cast<float>(1.0 - d.clutch) * 4.f;  // the pack backs off when the lever is pulled
        add(clutchPack, place(mx, my, Z_PRIMARY + 25.f + lift, mainA), FRICTION, 0.8f, 0.f);
        add(mainShaft, place(mx, my, 0.f, mainA), SHAFT, 0.3f, 0.8f);
        add(outShaft, place(ox, oy, 0.f, outA), SHAFT, 0.3f, 0.8f);
        add(sprocket, place(ox, oy, -124.f, outA), DARK, 0.5f, 0.6f);
        for (int g = 1; g <= d.P.gears; g++) {
            bool engaged = d.gear == g;
            float og = -mainA * nIn[g] / nOut[g] + meshPhase(thG, nIn[g], nOut[g]);
            add(inGear[g], place(mx, my, zGear[g], mainA), engaged ? BRASS : STEEL, 0.35f, 0.7f);
            add(outGear[g], place(ox, oy, zGear[g], og), engaged ? BRASS : STEEL, 0.35f, 0.7f);
        }
        // shift drum: one notch per position, neutral between 1st and 2nd
        float pos = d.gear <= 0 ? 0.5f : (d.gear == 1 ? 0.f : float(d.gear));
        add(drum, place(drumX, drumY, -20.f, pos * PI / 3.f), SHAFT, 0.3f, 0.8f);
        if (ghost == 0) {
            add(housing, world, {0.37f, 0.36f, 0.33f, 0.16f}, 0.65f, 0.f);
            add(cover, world, side > 0.f ? std::array<float, 4>{0.55f, 0.03f, 0.03f, 0.22f} : std::array<float, 4>{0.05f, 0.20f, 0.55f, 0.22f}, 0.5f, 0.f);
        }
    }
};

