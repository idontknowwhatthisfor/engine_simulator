// what makes one engine different from another: its specs, gearing, mesh, labels and test targets.
// Each engine lives in engines/<type>/ and fills one engineType; src/engines.hpp lists them for
// --engine-type.
#pragma once
#include "physics.hpp"
#include "exhaust.hpp"

// what the superbike physics test expects from an engine (published figures, with tolerance)
struct bikeSpec {
    const char *name;
    double dispLo, dispHi;
    double limLo, limHi;                   // peak rpm bouncing on the limiter
    const char *torqueWhat, *torqueRpmWhat, *powerWhat, *powerRpmWhat;
    double torqueLo, torqueHi, torqueRpmLo, torqueRpmHi;
    double powerLo, powerHi, powerRpmLo, powerRpmHi;
    double sweepTo;                        // dyno sweep top, rpm
    std::vector<double> pattern;           // firing intervals, deg, one per cylinder (they add up to 720)
    const char *patternWhat;
    const char *bikeWhat;
    double upshiftLo, upshiftHi;           // rpm at the 1 -> 2 upshift, full throttle
    const char *topWhat;
    double topLo, topHi;
    double neutralSettle;                  // s for the revs to fall back to idle after N at speed
    double takeoffMinRpm = 900.0;          // lowest rpm in the gentle (25%) manual take-off
};

// one step of a scripted ride for --wav (bikes): hold these controls until `until` seconds
struct rideStep {
    double until;
    bool starter;
    double throttle;
    int shift;                             // +1 into gear from N, -1 quickshift down (rider takes over)
    const char *what;
    bool brake = false;
};

struct engineType;
int physicsTestBike(const engineModel &e, const engineType &t);  // src/test_bike.hpp

struct engineType {
    const char *id = "";                   // --engine-type value
    const char *aliases = "";              // other accepted names, space separated
    const char *name = "";                 // short name for tests and messages
    const char *summary = "";              // one line for --engine-type list
    const char *label = "";                // HUD: what is running
    const char *firingLabel = "";          // HUD: a descriptive firing order (e.g. "Twin Pulse 0-90-290-380");
                                           // empty: the order from the mesh's lags (firingOrder, "1-2-4-3")
    const char *windowTitle = "";
    const char *hudTag = "";               // shown after the rpm on the riding HUD (e.g. TUNED)
    const char *credit = "";               // model licence line (CC-BY needs it on screen)
    const char *meshFile = "";             // relative to the project root
    const char *buildHint = "";            // how to make the mesh when it is missing
    bool bike = false;                     // bike drivetrain, bike gearbox, superbike tests
    bool supercharged = false;             // always boosted (engine().turbo.supercharger): its exhaust() places
                                           // turboMeshFile, drawn and spun like --turbo yes
    engineParams (*engine)() = nullptr;    // engine physics and sound
    drivetrainParams (*drive)() = nullptr; // clutch, gearbox and vehicle
    int (*physTest)(const engineModel &, const engineType &) = nullptr;  // --phystest
    int (*physTestTurbo)(const engineModel &, const engineType &) = nullptr;  // --turbo yes --phystest
    bikeSpec (*spec)() = nullptr;          // bikes: --phystest targets
    // bike gearbox drawing (src/bike_gearbox.hpp): which side of the crank the box goes (+1 toward +x,
    // -1 toward -x) and the primary gear module. Where the crank's primary gear sits along the crank
    // comes from the mesh (zPrimary, see primaryGearZ)
    float gearboxSide = 1.f, primaryModule = 2.6f;
    // the exhaust system (src/exhaust.hpp): headers from the ports to the tailpipe, in mesh space
    exhaustLayout (*exhaust)(const engineModel &) = nullptr;
    // --turbo yes: the exhaust with a turbocharger in it (its turboSpec places src/turbo.hpp's model)
    exhaustLayout (*exhaustTurbo)(const engineModel &) = nullptr;
    const char *turboMeshFile = "";        // relative to the project root
    const char *turboBuildHint = "";
    std::vector<rideStep> ride;            // bikes: the --wav demo ride (empty: the stand script)
    // --tuned yes: the tuned version of this engine, a whole engineType of its own (params, test
    // targets, labels); nullptr when the engine has none
    const engineType &(*tuned)() = nullptr;
};

// the firing order the HUD shows: the engine's label, or the order from the mesh
std::string hudFiring(const engineType &t, const engineModel &e) { return *t.firingLabel ? t.firingLabel : firingOrder(e); }
