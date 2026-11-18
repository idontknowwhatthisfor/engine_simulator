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

    // The rest have defaults for a 1000 cc superbike (idle ~1250 rpm, limiter ~14000+, 6 gears): an
    // engine that idles, revs or launches differently sets its own.
    // on the stand
    double crankLo = 200.0, crankHi = 500.0;           // cranking speed with the ignition off, mean rpm
    double catchRpm = 800.0;                           // it has caught once the starter gets it past this
    double flareLo = 1300.0, flareHi = 3500.0;         // start flare, peak rpm
    double idleLo = 1150.0, idleHi = 1350.0;           // idle speed, mean rpm
    double idleMapLo = 25.0, idleMapHi = 60.0;         // idle manifold pressure, kPa
    double toLimiterLo = 0.25, toLimiterHi = 1.5;      // s from idle to the limiter, full throttle in neutral
    double closedLo = 1100.0, closedHi = 1500.0;       // rpm 4 s after snapping the throttle shut
    double coastFromRpm = 8000.0;                      // coast-down (ignition off) starts here
    double shakeRpm = 10000.0;                         // the balance check runs at this locked speed
    double shakeMax1st = 0.0;                          // N, most 1st-order vertical force there; 0: 5% of
                                                       // half the pistons moving together without counterweights
    double firingRpm = 3000.0;                         // the firing-interval check runs at this locked speed
    // on the bike
    double idleGearLo = 1100.0, idleGearHi = 1450.0;   // idling in 1st (automatic), rpm
    const char *zeroTo100What = "0-100 km/h, full throttle (tests ~3.0-3.4 s)";
    double zeroTo100Lo = 2.6, zeroTo100Hi = 3.7;       // s
    const char *zeroTo200What = "0-200 km/h (road tests ~5.5-7 s)";
    double zeroTo200Lo = 5.0, zeroTo200Hi = 8.0;       // s
    double launchAccelLo = 0.8, launchAccelHi = 1.15;  // peak smoothed acceleration with wheelie control, g
    double shiftRpm = 11000.0;                         // manual tests: quickshift up at this rpm (70-80% pedal)
    double shiftRpmGentle = 10000.0;                   // ... and at this one (50% pedal, up to 4th)
                                                       // (both must be below the rev limiter)
    bool stallsOnClutchDump = true;                    // dumping the clutch in 1st at idle stalls it (a light
                                                       // crank); false: it must pull away without a lurch
    double neutralRevMin = 8000.0;                     // revving in N at 40% pedal reaches at least this
    double neutralIdleLo = 1000.0, neutralIdleHi = 2200.0;  // rpm after coasting in N from speed
    double rejoinLo = 3000.0, rejoinHi = 14000.0;      // rpm once N picks a gear again at speed
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
