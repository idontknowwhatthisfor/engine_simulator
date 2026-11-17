// the car's 5-speed manual gearbox, drawn behind the flywheel (Honda Fit ratios and reverse idler)
#pragma once
#include "../../src/render.hpp"
#include "../../src/physics.hpp"

// ---------------------------------------------------------------------------------------------
// gearbox geometry (mesh space, mm): clutch, input shaft, countershaft, 5 gear pairs, output shaft
// ---------------------------------------------------------------------------------------------
// Layout of a classic in-line manual box: the input shaft drives the countershaft through a
// constant-mesh head pair, every countershaft gear is always meshed with a gear that idles on the
// output shaft, and a sliding dog collar locks the selected one to the output shaft. Gear sizes come
// from the real ratios at a fixed centre distance; tooth phases are set so the teeth actually mesh.
struct gearboxModel {
    static constexpr float D = 105.f;     // main axis to countershaft
    static constexpr float H = 1.6f;      // head pair reduction
    static constexpr float MODULE = 3.2f; // tooth size
    float ax = 0.f, ay = 0.f;
    int nIn = 0, nHead = 0;
    std::array<int, 6> nOut{}, nCnt{};
    std::array<float, 6> zGear{};
    meshBuf inGear, headGear, clutchDisc, pressurePlate, inShaft, outShaft, cntShaft, flange, collar, housing;
    std::array<meshBuf, 6> outGear, cntGear;
    // reverse: countershaft gear -> idler (on its own short shaft, off to the side) -> output gear
    static constexpr float Z_REV = 485.f;
    int nCr = 0, nId = 0, nOr = 0;
    float idX = 0.f, idY = 0.f;  // idler centre relative to the main axis
    meshBuf revCnt, revIdler, revOut, idlerShaft;

    static float meshPhase(float theta, int nA, int nB) { return gearMeshPhase(theta, nA, nB); }

    static int teethFor(float r) { return std::max(10, static_cast<int>(std::lround(2.f * r / MODULE))); }
    // for a top gear (nTop) over a bottom gear (nBot): bottom angle = -top * nTop / nBot + offset puts a
    // tooth of one into a gap of the other where they touch (top at -y... bottom at +y in mesh space)
    static float meshOffset(int nTop, int nBot) { return -PI / 2.f + PI / nBot + (PI / 2.f) * nTop / nBot; }

    void build(const engineModel &e, const drivetrainParams &dp, renderer &ren) {
        ax = e.crankAxis.x;
        ay = e.crankAxis.y;
        float rIn = D / (1.f + H);
        nIn = teethFor(rIn);
        nHead = teethFor(D - rIn);
        inGear = makeGear(rIn, nIn, 16.f, 12.f);
        headGear = makeGear(D - rIn, nHead, 16.f, 11.f);
        for (int g = 1; g <= 5; g++) {
            float rc = D / (1.f + static_cast<float>(dp.ratios[g]) / H);
            nCnt[g] = teethFor(rc);
            nOut[g] = teethFor(D - rc);
            cntGear[g] = makeGear(rc, nCnt[g], 18.f, 11.f);
            outGear[g] = makeGear(D - rc, nOut[g], 18.f, 14.f);
            zGear[g] = 335.f + 30.f * (g - 1);
        }
        clutchDisc = makeGear(60.f, 18, 8.f, 12.f);  // the damper-spring bumps show it turning
        pressurePlate = makeGear(68.f, 6, 12.f, 20.f);
        inShaft = makeCylinder(12.f, 262.f, 312.f);
        outShaft = makeCylinder(14.f, 316.f, 532.f);
        cntShaft = makeCylinder(11.f, 288.f, 500.f);
        // reverse set: counter gear and output gear don't touch; the idler sits where it meshes with both
        {
            float rc = 30.f, ro = rc * static_cast<float>(dp.reverseRatio) / H, ri = 22.f;
            nCr = teethFor(rc); nOr = teethFor(ro); nId = teethFor(ri);
            float dOut = ro + ri, dCnt = rc + ri;  // idler centre distances to the main axis / countershaft
            idY = (dOut * dOut - dCnt * dCnt + D * D) / (2.f * D);
            idX = std::sqrt(std::max(0.f, dOut * dOut - idY * idY));
            revCnt = makeGear(rc, nCr, 16.f, 11.f);
            revOut = makeGear(ro, nOr, 16.f, 14.f);
            revIdler = makeGear(ri, nId, 16.f, 8.f);
            idlerShaft = makeCylinder(8.f, Z_REV - 14.f, Z_REV + 14.f, 16);
        }
        flange = makeGear(32.f, 4, 10.f, 14.f);
        collar = makeCylinder(24.f, -5.f, 5.f, 24);
        housing = makeBox({ax - 92.f, ay - 92.f, 256.f}, {ax + 92.f, ay + D + 84.f, 540.f});
        for (meshBuf *m : {&inGear, &headGear, &clutchDisc, &pressurePlate, &inShaft, &outShaft, &cntShaft, &flange,
                           &collar, &housing, &revCnt, &revOut, &revIdler, &idlerShaft})
            ren.upload(*m);
        for (int g = 1; g <= 5; g++) {
            ren.upload(cntGear[g]);
            ren.upload(outGear[g]);
        }
    }

    // lowest point in mesh y (bottom of the housing) and the far end in mesh z, for the floor/camera
    float bottomY() const { return ay + D + 84.f; }
    static float rearZ() { return 540.f; }

    void items(const engineSim &sim, float crankDelta, const mat4 &world, int ghost, std::vector<drawItem> &out) const {
        const drivetrain &d = sim.drive;
        auto place = [&](float x, float y, float z, float angle) { return world * translate({x, y, z}) * rotateZ(angle); };
        auto add = [&](const meshBuf &m, const mat4 &model, std::array<float, 4> c, float rough, float metal) {
            out.push_back({&m, model, {c[0], c[1], c[2], c[3]}, rough, metal});
        };
        const std::array<float, 4> STEEL{0.42f, 0.43f, 0.45f, 1.f}, SHAFT{0.55f, 0.55f, 0.58f, 1.f},
            BRASS{0.80f, 0.58f, 0.22f, 1.f}, FRICTION{0.24f, 0.17f, 0.11f, 1.f}, PLATE{0.30f, 0.31f, 0.33f, 1.f};
        float inA = static_cast<float>(d.inputAngle);
        float cntA = -inA * nIn / nHead + meshOffset(nIn, nHead);
        float outA = static_cast<float>(d.outputAngle);
        float cy = ay + D;  // countershaft sits below the main axis (mesh +y is down)

        // clutch: the pressure plate turns with the crank, the disc with the input shaft, so you can see
        // them slip; the plate backs off a little when the clutch is released
        float lift = static_cast<float>(1.0 - d.clutch) * 5.f;
        add(pressurePlate, place(ax, ay, 281.f + lift, crankDelta), PLATE, 0.4f, 0.6f);
        add(clutchDisc, place(ax, ay, 266.f, inA), FRICTION, 0.8f, 0.f);
        add(inShaft, place(ax, ay, 0.f, inA), SHAFT, 0.3f, 0.8f);
        add(inGear, place(ax, ay, 300.f, inA), STEEL, 0.35f, 0.7f);
        add(headGear, place(ax, cy, 300.f, cntA), STEEL, 0.35f, 0.7f);
        add(cntShaft, place(ax, cy, 0.f, cntA), SHAFT, 0.3f, 0.8f);
        add(outShaft, place(ax, ay, 0.f, outA), SHAFT, 0.3f, 0.8f);
        add(flange, place(ax, ay, 527.f, outA), PLATE, 0.4f, 0.6f);
        for (int g = 1; g <= 5; g++) {
            bool engaged = d.gear == g;
            float og = (meshOffset(nOut[g], nCnt[g]) - cntA) * nCnt[g] / nOut[g];
            add(cntGear[g], place(ax, cy, zGear[g], cntA), engaged ? BRASS : STEEL, 0.35f, 0.7f);
            add(outGear[g], place(ax, ay, zGear[g], og), engaged ? BRASS : STEEL, 0.35f, 0.7f);
        }
        // reverse: counter gear -> idler -> output gear, each contact phased for its own direction
        {
            bool rev = d.gear == -1;
            float thCI = std::atan2(idY - D, idX);  // countershaft centre -> idler centre
            float thIO = std::atan2(-idY, -idX);    // idler centre -> main axis
            float idA = -cntA * nCr / nId + meshPhase(thCI, nCr, nId);
            float orA = -idA * nId / nOr + meshPhase(thIO, nId, nOr);
            add(revCnt, place(ax, cy, Z_REV, cntA), rev ? BRASS : STEEL, 0.35f, 0.7f);
            add(revIdler, place(ax + idX, ay + idY, Z_REV, idA), rev ? BRASS : STEEL, 0.35f, 0.7f);
            add(idlerShaft, place(ax + idX, ay + idY, 0.f, idA), SHAFT, 0.3f, 0.8f);
            add(revOut, place(ax, ay, Z_REV, orA), rev ? BRASS : STEEL, 0.35f, 0.7f);
        }
        // dog collar beside the engaged gear; parked between 2nd and 3rd in neutral
        float collarZ = d.gear > 0 ? zGear[d.gear] - 15.f : d.gear < 0 ? Z_REV - 15.f : (zGear[2] + zGear[3]) / 2.f;
        add(collar, place(ax, ay, collarZ, outA), d.gear != 0 ? BRASS : STEEL, 0.3f, 0.8f);
        if (ghost == 0) add(housing, world, {0.37f, 0.36f, 0.33f, 0.16f}, 0.65f, 0.f);
    }
};

