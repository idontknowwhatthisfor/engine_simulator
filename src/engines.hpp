// the engines --engine-type can pick (every engine that registered itself, src/engine_type.hpp), and
// setting the simulation up for one of them
#pragma once
#include "engine_type.hpp"
#include "calibration.hpp"

// the registered engines in key order. Two engines with the same key, id or alias are a mistake in
// the engines' headers: reported and the program stops.
const std::vector<const engineType *> &engineTypes() {
    static const std::vector<const engineType *> all = [] {
        std::vector<engineRegistration> r = engineRegistry();
        std::stable_sort(r.begin(), r.end(), [](const engineRegistration &a, const engineRegistration &b) { return a.key < b.key; });
        std::vector<const engineType *> out;
        std::vector<std::pair<std::string, const char *>> names;  // every id and alias, with its engine's id
        for (std::size_t k = 0; k < r.size(); k++) {
            const engineType &t = r[k].type();
            if (k > 0 && r[k].key == r[k - 1].key) {
                std::fprintf(stderr, "engine registry: %s and %s both use key %d\n", r[k - 1].type().id, t.id, r[k].key);
                std::exit(2);
            }
            std::istringstream aliases(t.aliases);
            std::vector<std::string> mine{t.id};
            for (std::string a; aliases >> a;) mine.push_back(a);
            for (const std::string &n : mine) {
                for (const auto &[other, owner] : names)
                    if (n == other) {
                        std::fprintf(stderr, "engine registry: \"%s\" names both %s and %s\n", n.c_str(), owner, t.id);
                        std::exit(2);
                    }
                names.push_back({n, t.id});
            }
            out.push_back(&t);
        }
        return out;
    }();
    return all;
}

// by id or alias, case-insensitive; nullptr when unknown
const engineType *findEngineType(std::string name) {
    for (char &ch : name) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    for (const engineType *t : engineTypes()) {
        if (name == t->id) return t;
        std::istringstream aliases(t->aliases);
        for (std::string a; aliases >> a;)
            if (name == a) return t;
    }
    return nullptr;
}

// load the engine's parameter sets, then init. stand: a bike engine on a stand, nothing downstream
// of the crank (manual, neutral). turbo: --turbo yes (the clutch gets a stronger spring for the boost)
void setupSim(engineSim &sim, const engineModel &e, const engineType &type, double theta1, bool stand = false, bool turbo = false) {
    sim.P = type.engine();
    sim.drive.P = type.drive();
    validateDrivetrain(sim.drive.P);
    // an engine built with a turbo or a supercharger always has it
    if ((type.boost == builtInBoost::supercharger) != sim.P.turbo.supercharger)
        throw std::runtime_error(std::string(type.id) + ": engineType::boost (" + boostName(type.boost) +
                                 ") disagrees with engineParams::turbo.supercharger");
    sim.P.turbo.on = turbo || type.boost != builtInBoost::none;
    // the exhaust's pressure-wave tuning: its header length from the drawn layout (the one the window
    // shows: --turbo yes has its own), unless the engine gives one
    if (sim.P.scavGain != 0.0 && sim.P.scavLen <= 0.0) {
        exhaustLayout L;
        if (turbo && type.exhaustTurbo) L = type.exhaustTurbo(e);
        else if (type.exhaust) L = type.exhaust(e);
        double r = 0.0, h = headerLength(L, static_cast<int>(e.cyls.size()), 1.6f, &r);
        if (h > 0.0) sim.P.scavLen = h + sim.P.scavPortLen + 0.6 * r;
    }
    if (turbo && !sim.P.turbo.supercharger) sim.drive.P.clutchCapacity *= 1.5;  // --turbo yes on an engine built without
    sim.drive.P.qsMaxRpm = sim.P.revLimit;  // no quickshift / DCT shift down past the limiter
    if (type.bike) {
        sim.standalone = stand;
    }
    sim.init(e, theta1);
    if (stand) sim.drive.reset(false);
}
