// the engines --engine-type can pick, and setting the simulation up for one of them
#pragma once
#include "engine_type.hpp"

const engineType &inline4FitType();   // engines/inline4_fit/inline4_fit.hpp
const engineType &ducatiV4Type();     // engines/ducati_v4/ducati_v4.hpp
const engineType &bmwS1000rrType();   // engines/bmw_s1000rr/bmw_s1000rr.hpp
const engineType &kawasakiH2Type();   // engines/kawasaki_h2/kawasaki_h2.hpp

const std::vector<const engineType *> &engineTypes() {
    static const std::vector<const engineType *> all{&inline4FitType(), &ducatiV4Type(), &bmwS1000rrType(), &kawasakiH2Type()};
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
    sim.P.turbo.on = turbo || sim.P.turbo.supercharger;  // a supercharged engine always has it
    if (turbo && !sim.P.turbo.supercharger) sim.drive.P.clutchCapacity *= 1.5;
    if (type.bike) {
        sim.drive.P.qsMaxRpm = sim.P.revLimit;
        sim.standalone = stand;
    }
    sim.init(e, theta1);
    if (stand) sim.drive.reset(false);
}
