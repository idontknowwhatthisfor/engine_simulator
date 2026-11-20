// --dump-params (hidden): every parameter the selected engine runs with, one `name = value` per line,
// doubles with %.17g and floats with %.9g so they round-trip exactly. scripts/regress.sh diffs this
// output before and after a change to prove nothing was recalibrated. The values are the effective
// ones, after setupSim (for the version --tuned / --variant picks). When a field is added to one of the parameter
// structs, add it here too: the sizeof checks below fail to compile until the list is updated.
#pragma once
#include "engines.hpp"
#include "sound.hpp"

struct paramDumper {
    std::string prefix;
    void put(const char *n, double v) const { std::printf("%s%s = %.17g\n", prefix.c_str(), n, v); }
    void put(const char *n, float v) const { std::printf("%s%s = %.9g\n", prefix.c_str(), n, static_cast<double>(v)); }
    void put(const char *n, int v) const { std::printf("%s%s = %d\n", prefix.c_str(), n, v); }
    void put(const char *n, bool v) const { std::printf("%s%s = %s\n", prefix.c_str(), n, v ? "true" : "false"); }
    void put(const char *n, const char *v) const { std::printf("%s%s = \"%s\"\n", prefix.c_str(), n, v ? v : "(null)"); }
    void put(const char *n, const std::string &v) const { put(n, v.c_str()); }
    void put(const char *n, vec3 v) const {
        std::printf("%s%s = (%.9g, %.9g, %.9g)\n", prefix.c_str(), n, double(v.x), double(v.y), double(v.z));
    }
    template <class T, std::size_t N> void put(const char *n, const std::array<T, N> &a) const {
        for (std::size_t k = 0; k < N; k++) put((std::string(n) + "[" + std::to_string(k) + "]").c_str(), a[k]);
    }
    template <class T> void put(const char *n, const std::vector<T> &a) const {
        for (std::size_t k = 0; k < a.size(); k++) put((std::string(n) + "[" + std::to_string(k) + "]").c_str(), a[k]);
    }
    template <class F> void set(const char *n, F *f) const { std::printf("%s%s = %s\n", prefix.c_str(), n, f ? "set" : "none"); }
    paramDumper sub(const std::string &p) const { return {prefix + p}; }
};
#define DUMP(d, obj, f) (d).put(#f, (obj).f)

// a field added to a parameter struct must be added to its dump below; these trip when one is not
static_assert(sizeof(soundParams) == 248, "soundParams changed: update dumpSound()");
static_assert(sizeof(turboParams) == 328, "turboParams changed: update dumpTurbo()");
static_assert(sizeof(engineParams) == 1176, "engineParams changed: update dumpEngine()");
static_assert(sizeof(drivetrainParams) == 448, "drivetrainParams changed: update dumpDrive()");
static_assert(sizeof(bikeSpec) == 544, "bikeSpec changed: update dumpBikeSpec()");
static_assert(sizeof(carSpec) == 552, "carSpec changed: update dumpCarSpec()");

void dumpSound(const paramDumper &d, const soundParams &S) {
    DUMP(d, S, pipeLowHz); DUMP(d, S, pipeResHz); DUMP(d, S, pipeResGain); DUMP(d, S, masterHz);
    DUMP(d, S, detailed); DUMP(d, S, pipeLength); DUMP(d, S, gasSpeed); DUMP(d, S, pipeReflect);
    DUMP(d, S, pipeDampHz); DUMP(d, S, flapOpenRpm); DUMP(d, S, flapSpan); DUMP(d, S, flapShutHz);
    DUMP(d, S, flapOpenHz); DUMP(d, S, flapShutGain); DUMP(d, S, airboxHz); DUMP(d, S, intakeHowlHz);
    DUMP(d, S, intakeGain); DUMP(d, S, popRpm); DUMP(d, S, popChance); DUMP(d, S, whineGain);
    DUMP(d, S, drive); DUMP(d, S, radiateHz); DUMP(d, S, burble); DUMP(d, S, burbleFadeFrom);
    DUMP(d, S, burbleFadeSpan); DUMP(d, S, burbleHigh); DUMP(d, S, tick); DUMP(d, S, tickHz);
    DUMP(d, S, turboGain); DUMP(d, S, raspGain); DUMP(d, S, dcBlockHz);
}

void dumpTurbo(const paramDumper &d, const turboParams &T) {
    DUMP(d, T, on); DUMP(d, T, supercharger); DUMP(d, T, driveRatio); DUMP(d, T, driveEff);
    DUMP(d, T, ramGain); DUMP(d, T, reliefP); DUMP(d, T, reliefSpan); DUMP(d, T, reliefArea);
    DUMP(d, T, bov); DUMP(d, T, inertia); DUMP(d, T, maxRpm); DUMP(d, T, prRef); DUMP(d, T, rpmRef);
    DUMP(d, T, flowPeakRef); DUMP(d, T, shutoffHead); DUMP(d, T, chokeFall); DUMP(d, T, ductLA);
    DUMP(d, T, vCharge); DUMP(d, T, icDrop); DUMP(d, T, icEffStill); DUMP(d, T, icEffMoving);
    DUMP(d, T, icTau); DUMP(d, T, bovArea); DUMP(d, T, bovCrack); DUMP(d, T, bovSpan); DUMP(d, T, etaC);
    DUMP(d, T, etaT); DUMP(d, T, turbineArea); DUMP(d, T, wastegateArea); DUMP(d, T, vExhaust);
    DUMP(d, T, bearingLoss); DUMP(d, T, boostMap); DUMP(d, T, boostFallRpm); DUMP(d, T, boostTopRpm); DUMP(d, T, boostTop); DUMP(d, T, chargeMax); DUMP(d, T, wgKp); DUMP(d, T, wgKi);
    DUMP(d, T, knockRetard); DUMP(d, T, knockRetardHot); DUMP(d, T, tExhLight); DUMP(d, T, tExhFull);
}

void dumpEngine(const paramDumper &d, const engineParams &P) {
    dumpSound(d.sub("sound."), P.sound);
    dumpTurbo(d.sub("turbo."), P.turbo);
    DUMP(d, P, bore); DUMP(d, P, compression); DUMP(d, P, gamma); DUMP(d, P, ivc); DUMP(d, P, evo);
    DUMP(d, P, blowdown); DUMP(d, P, pAtm); DUMP(d, P, pExhaust); DUMP(d, P, pCrankcase); DUMP(d, P, tIntake);
    DUMP(d, P, rGas); DUMP(d, P, afr); DUMP(d, P, lhv); DUMP(d, P, etaHeat); DUMP(d, P, burn);
    DUMP(d, P, wiebeA); DUMP(d, P, wiebeM); DUMP(d, P, cycleCovIdle); DUMP(d, P, cycleCovFull); DUMP(d, P, leakTau); DUMP(d, P, mRecip); DUMP(d, P, inertia);
    DUMP(d, P, vManifold); DUMP(d, P, throttleArea); DUMP(d, P, throttleCd); DUMP(d, P, idleBase);
    DUMP(d, P, idleMin); DUMP(d, P, idleMax); DUMP(d, P, idleKp); DUMP(d, P, idleKi); DUMP(d, P, idleBand);
    DUMP(d, P, idleSparkBase); DUMP(d, P, idleSparkGain); DUMP(d, P, idleRpm); DUMP(d, P, starterTorque);
    DUMP(d, P, starterMaxRpm); DUMP(d, P, revLimit); DUMP(d, P, revResume); DUMP(d, P, fmepA);
    DUMP(d, P, fmepB); DUMP(d, P, fmepC); DUMP(d, P, pedalRate); DUMP(d, P, veBase); DUMP(d, P, vePeak);
    DUMP(d, P, veRpm); DUMP(d, P, veWidth); DUMP(d, P, lowCam); DUMP(d, P, veLowBase); DUMP(d, P, veLowPeak);
    DUMP(d, P, veLowRpm); DUMP(d, P, veLowWidth); DUMP(d, P, ramLen); DUMP(d, P, ramDia); DUMP(d, P, ramSound);
    DUMP(d, P, ramGain); DUMP(d, P, ramDecay); DUMP(d, P, scavLen); DUMP(d, P, scavPortLen); DUMP(d, P, scavSound); DUMP(d, P, scavGain);
    DUMP(d, P, scavDecay); DUMP(d, P, sparkLow); DUMP(d, P, sparkHigh);
    DUMP(d, P, sparkFrom); DUMP(d, P, sparkSpan); DUMP(d, P, mapMin); DUMP(d, P, residualMin);
    DUMP(d, P, counterweight); DUMP(d, P, rpmScale); DUMP(d, P, fuelCutRpm); DUMP(d, P, blockMass);
    DUMP(d, P, blockRollInertia); DUMP(d, P, mountFreqY); DUMP(d, P, mountFreqRoll); DUMP(d, P, mountZeta);
}

void dumpDrive(const paramDumper &d, const drivetrainParams &P) {
    DUMP(d, P, gears);
    // the gear tables only up to the gears fitted (index 0 = neutral)
    for (int g = 0; g <= P.gears; g++) d.put(("ratios[" + std::to_string(g) + "]").c_str(), P.ratios[g]);
    DUMP(d, P, bike); DUMP(d, P, primary); DUMP(d, P, reverseRatio); DUMP(d, P, finalDrive);
    DUMP(d, P, wheelRadius); DUMP(d, P, efficiency); DUMP(d, P, carMass); DUMP(d, P, wheelInertia);
    DUMP(d, P, crr); DUMP(d, P, cdA); DUMP(d, P, airDensity); DUMP(d, P, clutchCapacity);
    DUMP(d, P, clutchStiffness); DUMP(d, P, inputInertia); DUMP(d, P, brakeDecel); DUMP(d, P, shiftOpen);
    DUMP(d, P, shiftClose); DUMP(d, P, normalDown); DUMP(d, P, sportUp); DUMP(d, P, sportDown);
    DUMP(d, P, matchHold); DUMP(d, P, firstGearSpeed); DUMP(d, P, clutchPressRate);
    DUMP(d, P, clutchReleaseRate); DUMP(d, P, openRpm); DUMP(d, P, idleOpenRpm); DUMP(d, P, biteSpan);
    DUMP(d, P, launchPedal); DUMP(d, P, upBase); DUMP(d, P, upPedal); DUMP(d, P, upMargin);
    DUMP(d, P, downMax); DUMP(d, P, kickRpm); DUMP(d, P, kickMax); DUMP(d, P, loadDownMax);
    DUMP(d, P, pickRpm); DUMP(d, P, pickMax); DUMP(d, P, slipper); DUMP(d, P, quickshiftTime); DUMP(d, P, dualClutch);
    DUMP(d, P, qsUnloadTorque); DUMP(d, P, qsMatchTol); DUMP(d, P, qsMatchTimeout); DUMP(d, P, qsMaxRpm);
    DUMP(d, P, wheelieAccel);
    for (int g = 1; g <= P.gears; g++) {
        auto tooth = [&](const std::vector<int> &t) { return g < static_cast<int>(t.size()) ? t[g] : 0; };  // none: 0
        d.put(("teethIn[" + std::to_string(g) + "]").c_str(), tooth(P.teethIn));
        d.put(("teethOut[" + std::to_string(g) + "]").c_str(), tooth(P.teethOut));
    }
    DUMP(d, P, primaryIn); DUMP(d, P, primaryOut); DUMP(d, P, sprocketTeeth);
}

void dumpCarSpec(const paramDumper &d, const carSpec &S) {
    DUMP(d, S, name); DUMP(d, S, dispLo); DUMP(d, S, dispHi); DUMP(d, S, crankLo); DUMP(d, S, crankHi);
    DUMP(d, S, flareLo); DUMP(d, S, flareHi); DUMP(d, S, idleLo); DUMP(d, S, idleHi); DUMP(d, S, idleMapLo);
    DUMP(d, S, idleMapHi); DUMP(d, S, toLimiterLo); DUMP(d, S, toLimiterHi); DUMP(d, S, limiterOver);
    DUMP(d, S, liftLo); DUMP(d, S, liftHi); DUMP(d, S, coastLo); DUMP(d, S, coastHi); DUMP(d, S, sweepTo); DUMP(d, S, torqueWhat); DUMP(d, S, torqueLo);
    DUMP(d, S, torqueHi); DUMP(d, S, torqueRpmLo); DUMP(d, S, torqueRpmHi); DUMP(d, S, powerWhat);
    DUMP(d, S, powerLo); DUMP(d, S, powerHi); DUMP(d, S, powerRpmLo); DUMP(d, S, powerRpmHi);
    for (std::size_t k = 0; k < S.torqueAt.size(); k++)
        d.put(("torqueAt[" + std::to_string(k) + "]").c_str(), std::vector<double>{S.torqueAt[k].rpm, S.torqueAt[k].lo, S.torqueAt[k].hi});
    for (std::size_t k = 0; k < S.powerAt.size(); k++)
        d.put(("powerAt[" + std::to_string(k) + "]").c_str(), std::vector<double>{S.powerAt[k].rpm, S.powerAt[k].lo, S.powerAt[k].hi});
    DUMP(d, S, plateauFrom); DUMP(d, S, plateauTo); DUMP(d, S, plateauMax); DUMP(d, S, cylMaxLo); DUMP(d, S, cylMaxHi);
    DUMP(d, S, turboMaxLo); DUMP(d, S, turboMaxHi); DUMP(d, S, boostCheckRpm); DUMP(d, S, spoolRpm); DUMP(d, S, spoolLo); DUMP(d, S, spoolHi);
    DUMP(d, S, carWhat); DUMP(d, S, zeroTo100What); DUMP(d, S, zeroTo100Lo); DUMP(d, S, zeroTo100Hi);
    DUMP(d, S, zeroTo200What); DUMP(d, S, zeroTo200Lo); DUMP(d, S, zeroTo200Hi); DUMP(d, S, launchLo);
    DUMP(d, S, launchHi); DUMP(d, S, launchTo100What); DUMP(d, S, launchTo100Lo); DUMP(d, S, launchTo100Hi); DUMP(d, S, wotUpLo); DUMP(d, S, lightUpLo); DUMP(d, S, lightUpHi); DUMP(d, S, hillPedal);
    DUMP(d, S, hillGrade); DUMP(d, S, flatGearLo); DUMP(d, S, flatGearHi); DUMP(d, S, coastDownLo);
    DUMP(d, S, coastDownHi); DUMP(d, S, dumpStalls); DUMP(d, S, reverseLo); DUMP(d, S, reverseHi);
}

void dumpBikeSpec(const paramDumper &d, const bikeSpec &S) {
    DUMP(d, S, name); DUMP(d, S, dispLo); DUMP(d, S, dispHi); DUMP(d, S, limLo); DUMP(d, S, limHi);
    DUMP(d, S, torqueWhat); DUMP(d, S, torqueRpmWhat); DUMP(d, S, powerWhat); DUMP(d, S, powerRpmWhat);
    DUMP(d, S, torqueLo); DUMP(d, S, torqueHi); DUMP(d, S, torqueRpmLo); DUMP(d, S, torqueRpmHi);
    DUMP(d, S, powerLo); DUMP(d, S, powerHi); DUMP(d, S, powerRpmLo); DUMP(d, S, powerRpmHi);
    DUMP(d, S, sweepTo); DUMP(d, S, pattern); DUMP(d, S, patternWhat); DUMP(d, S, bikeWhat);
    DUMP(d, S, upshiftLo); DUMP(d, S, upshiftHi); DUMP(d, S, topWhat); DUMP(d, S, topLo); DUMP(d, S, topHi);
    DUMP(d, S, neutralSettle); DUMP(d, S, takeoffMinRpm);
    DUMP(d, S, crankLo); DUMP(d, S, crankHi); DUMP(d, S, catchRpm); DUMP(d, S, flareLo); DUMP(d, S, flareHi);
    DUMP(d, S, idleLo); DUMP(d, S, idleHi); DUMP(d, S, idleMapLo); DUMP(d, S, idleMapHi);
    DUMP(d, S, toLimiterLo); DUMP(d, S, toLimiterHi); DUMP(d, S, closedLo); DUMP(d, S, closedHi);
    DUMP(d, S, coastFromRpm); DUMP(d, S, shakeRpm); DUMP(d, S, shakeMax1st); DUMP(d, S, firingRpm);
    DUMP(d, S, idleGearLo); DUMP(d, S, idleGearHi); DUMP(d, S, zeroTo100What); DUMP(d, S, zeroTo100Lo);
    DUMP(d, S, zeroTo100Hi); DUMP(d, S, zeroTo200What); DUMP(d, S, zeroTo200Lo); DUMP(d, S, zeroTo200Hi);
    DUMP(d, S, launchAccelLo); DUMP(d, S, launchAccelHi); DUMP(d, S, shiftRpm); DUMP(d, S, shiftRpmGentle);
    DUMP(d, S, stallsOnClutchDump); DUMP(d, S, dumpMinRpm); DUMP(d, S, neutralRevMin); DUMP(d, S, neutralIdleLo); DUMP(d, S, neutralIdleHi);
    DUMP(d, S, rejoinLo); DUMP(d, S, rejoinHi); DUMP(d, S, liftsFront); DUMP(d, S, downshiftMaxMs);
}

void dumpExhaust(const paramDumper &d, const exhaustLayout &L) {
    d.put("pipes", static_cast<int>(L.pipes.size()));
    for (std::size_t k = 0; k < L.pipes.size(); k++) {
        const pipeSpec &p = L.pipes[k];
        paramDumper q = d.sub("pipe[" + std::to_string(k) + "].");
        DUMP(q, p, pts); DUMP(q, p, r0); DUMP(q, p, r1); DUMP(q, p, cyl); DUMP(q, p, from); DUMP(q, p, rgba);
        DUMP(q, p, rough); DUMP(q, p, metal); DUMP(q, p, segLen); DUMP(q, p, capStart); DUMP(q, p, capEnd);
        DUMP(q, p, sides);
    }
    for (std::size_t k = 0; k < L.boxes.size(); k++) {
        const boxSpec &b = L.boxes[k];
        paramDumper q = d.sub("box[" + std::to_string(k) + "].");
        DUMP(q, b, lo); DUMP(q, b, hi); DUMP(q, b, rgba); DUMP(q, b, rough); DUMP(q, b, metal);
    }
    for (std::size_t k = 0; k < L.outlets.size(); k++) {
        const outletSpec &o = L.outlets[k];
        paramDumper q = d.sub("outlet[" + std::to_string(k) + "].");
        DUMP(q, o, at); DUMP(q, o, dir); DUMP(q, o, r);
    }
    paramDumper f = d.sub("flap.");
    DUMP(f, L.flap, on); DUMP(f, L.flap, at); DUMP(f, L.flap, axis); DUMP(f, L.flap, shaft); DUMP(f, L.flap, pipeR);
    paramDumper t = d.sub("turbo.");
    DUMP(t, L.turbo, on); DUMP(t, L.turbo, at); DUMP(t, L.turbo, x); DUMP(t, L.turbo, y); DUMP(t, L.turbo, z);
}

// the geometry the simulation takes from the mesh, and a hash of everything drawn
void dumpMesh(const paramDumper &d, const engineModel &e) {
    DUMP(d, e, banked);
    DUMP(d, e, crankAxis);
    d.put("cylinders", static_cast<int>(e.cyls.size()));
    for (std::size_t c = 0; c < e.cyls.size(); c++) {
        const cylinderGeom &g = e.cyls[c];
        paramDumper q = d.sub("cyl[" + std::to_string(c) + "].");
        DUMP(q, g, zCentre); DUMP(q, g, gudgeonX); DUMP(q, g, gudgeonY); DUMP(q, g, crankpinX); DUMP(q, g, crankpinY);
        DUMP(q, g, throw_); DUMP(q, g, theta0); DUMP(q, g, rodL); DUMP(q, g, bank); DUMP(q, g, deckY); DUMP(q, g, bore);
        DUMP(q, g, firingLagDeg);
    }
    // FNV-1a over the root matrix, materials and every group as loaded: the same drawn mesh whatever
    // the file format
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](const void *p, std::size_t n) {
        const unsigned char *b = static_cast<const unsigned char *>(p);
        for (std::size_t k = 0; k < n; k++) h = (h ^ b[k]) * 1099511628211ull;
    };
    mix(e.root.m.data(), sizeof(float) * 16);
    for (const material &m : e.mats) {
        mix(m.rgba, sizeof(m.rgba));
        mix(&m.metallic, sizeof(float));
        mix(&m.roughness, sizeof(float));
        std::uint8_t b = m.blend;
        mix(&b, 1);
    }
    for (const group &g : e.groups) {
        std::uint32_t head[3] = {g.kind, static_cast<std::uint32_t>(g.cyl), g.mat};
        mix(head, sizeof(head));
        mix(g.verts.data(), g.verts.size() * sizeof(float));
        mix(g.idx.data(), g.idx.size() * sizeof(std::uint32_t));
    }
    d.put("materials", static_cast<int>(e.mats.size()));
    d.put("groups", static_cast<int>(e.groups.size()));
    std::printf("%scontentHash = %016llx\n", d.prefix.c_str(), static_cast<unsigned long long>(h));
    d.put("lo", e.lo);
    d.put("hi", e.hi);
}

void dumpType(const paramDumper &d, const engineType &t, const engineModel &e) {
    DUMP(d, t, id); DUMP(d, t, aliases); DUMP(d, t, name); DUMP(d, t, summary); DUMP(d, t, label);
    d.put("firing", hudFiring(t, e)); DUMP(d, t, windowTitle); DUMP(d, t, hudTag); DUMP(d, t, credit); DUMP(d, t, meshFile);
    DUMP(d, t, buildHint); DUMP(d, t, bike); d.put("boost", boostName(t.boost));
    d.set("engine", t.engine); d.set("drive", t.drive); d.set("physTest", t.physTest);
    d.set("spec", t.spec); d.set("exhaust", t.exhaust);
    if (t.bike) d.put("zPrimary", primaryGearZ(e));  // from the mesh
    DUMP(d, t, gearboxSide); DUMP(d, t, primaryModule);
    DUMP(d, t, turboMeshFile); DUMP(d, t, turboFallbackMeshFile); DUMP(d, t, turboBuildHint); DUMP(d, t, turboCredit); DUMP(d, t, hudFiringLine);
    d.put("ride", static_cast<int>(t.ride.size()));
    for (std::size_t k = 0; k < t.ride.size(); k++) {
        const rideStep &r = t.ride[k];
        paramDumper q = d.sub("ride[" + std::to_string(k) + "].");
        DUMP(q, r, until); DUMP(q, r, starter); DUMP(q, r, throttle); DUMP(q, r, shift); DUMP(q, r, what); DUMP(q, r, brake);
    }
    d.put("tuned", findVariant(t, "tuned") != nullptr);
    for (std::size_t k = 0; k < t.variants.size(); k++) d.put(("variants[" + std::to_string(k) + "]").c_str(), t.variants[k].name);
}

// everything for one engine type, as setupSim leaves it
int dumpParams(const engineModel &e, const engineType &t, bool bov) {
    engineSim sim;
    setupSim(sim, e, t, e.cyls[0].theta0);
    sim.P.turbo.bov = bov;
    paramDumper root;
    dumpType(root.sub("type."), t, e);
    dumpEngine(root.sub("engine."), sim.P);
    root.put("engine.sound.dcBlockCutoff", dcBlockCutoff(sim.P.sound, sim.P.idleRpm, sim.cylCount()));  // effective, Hz
    dumpDrive(root.sub("drive."), sim.drive.P);
    if (t.spec) dumpBikeSpec(root.sub("spec."), t.spec());
    if (t.car) dumpCarSpec(root.sub("car."), t.car());
    dumpMesh(root.sub("mesh."), e);
    if (t.exhaust) {
        try {
            dumpExhaust(root.sub("exhaust."), t.exhaust(e));
        } catch (const std::exception &ex) {
            root.put("exhaust.error", ex.what());  // e.g. a mesh from before v3 without supercharger_at
        }
    }
    return 0;
}
