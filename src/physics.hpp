// engine physics (4-stroke pressure model per cylinder, crank dynamics, ECU) and the drivetrain
// (clutch, gearbox, vehicle). The parameter structs' defaults are a generic baseline; each engine
// sets its own values in engines/<type>/.
#pragma once
#include "model.hpp"

// ---------------------------------------------------------------------------------------------
// engine physics, SI units, double precision
// ---------------------------------------------------------------------------------------------
// State: cycle angle `theta` of cylinder 1 (0..720 deg, 0 = TDC at the start of intake) and crank
// speed `omega`. Each cylinder runs its own 4-stroke pressure model; gas, reciprocating inertia,
// friction, starter and load torques drive the crank through the exact slider-crank geometry.

const double DEGR = 3.14159265358979323846 / 180.0;
const double CYCLE = 720.0 * DEGR;
// how far each cylinder's cycle lags cylinder 1 comes from the model (cylinderGeom::firingLagDeg):
// inline-4 1-3-4-2 = 0, 540, 180, 360; Ducati V4 Twin Pulse = 0, 90, 290, 380

double wrapCycle(double a) {
    a = std::fmod(a, CYCLE);
    return a < 0.0 ? a + CYCLE : a;
}

// how an engine sounds (see engineSound). The defaults are the original, simpler voice the inline-4
// uses; `detailed` switches on the extra stages, first tuned for the BMW S 1000 RR and now used by
// every other engine, bike or car: valve-lift shaped blowdown, the exhaust as a ringing quarter-wave
// pipe heard through its radiation, the burble, the exhaust flap, airbox honk, overrun crackle (with
// the flames), gear whine and valvetrain ticks.
struct soundParams {
    double pipeLowHz = 700.0, pipeResHz = 110.0, pipeResGain = 1.5, masterHz = 7000.0;
    bool detailed = false;
    // the exhaust pipes. With the engine's drawn exhaust (engineType::exhaust) the sound runs through
    // its pipes (exhaustNetwork, src/sound.hpp): lengths, merges, areas and silencers from the drawing.
    // pipeFromLayout = false, or no drawing, falls back to one quarter-wave pipe of pipeLength: valve
    // end closed, tailpipe open, so it rings at the odd multiples of c / 4L.
    bool pipeFromLayout = true;
    double pipeLength = 1.15;       // m, header + link pipe + silencer (the single pipe only)
    // the speed of sound in the gas: 0 = from the exhaust gas temperature (exhaustGasTemp: ~500 m/s
    // in ~670 K gas, slower in cold pipes); > 0 fixes it (m/s)
    double gasSpeed = 0.0;
    // the open tailpipe end(s): reflection (inverted) and the corner of its high-frequency loss
    double pipeReflect = 0.55, pipeDampHz = 3500.0;
    // a chamber in the drawn exhaust (a pipe 1.6x or more wider than the one feeding it: silencer,
    // catalyst): the packing's low-pass corner and the share that gets through each pass
    double chamberHz = 900.0, chamberLoss = 0.8;
    // the drawn pipes' losses (friction, heat to the walls, eddies at bends and joints): amplitude
    // e-folds per metre at 20 mm radius. 0.2 keeps every engine's full throttle over its overrun
    double wallLoss = 0.2;
    // silencer flap: shut at idle and light load (muffled), open from ~5000 rpm or with the throttle up
    double flapOpenRpm = 5000.0, flapSpan = 1500.0;
    double flapShutHz = 1600.0, flapOpenHz = 6500.0, flapShutGain = 0.55;
    // airbox: tonal honk from the intake pulses, loud at full throttle
    double airboxHz = 190.0, intakeHowlHz = 750.0, intakeGain = 0.0;
    // overrun crackle (lift-off above popRpm), quickshifter bark, limiter crackle
    double popRpm = 3500.0, popChance = 0.0;
    // straight-cut gear whine: primary pair and the engaged gearbox pair
    double whineGain = 0.0;
    // output gain (linear; a limiter near full scale is only a safety net, src/sound.hpp output()).
    // Set per engine from the --wav "level:" line: the loudest moment of the script well under full
    // scale (peak ~0.7) and the full-throttle loudness like the other engines' (tuned ones ~2 dB up)
    double drive = 0.178;
    // what leaves the tailpipe is heard as the rate of change of the gas flow (an open pipe end
    // radiates like a monopole): +6 dB per octave, so the voice brightens and swells with the revs.
    // radiateHz is the frequency where the derivative has unity gain
    double radiateHz = 0.0;         // 0 = off (the original voice)
    double burble = 0.0;            // the low exhaust body under 300 Hz (the idle burble), heard directly
    // the burble fades to burbleHigh of itself from burbleFadeFrom over burbleFadeSpan rpm, so the
    // radiated howl carries the top end (0 span = no fade)
    double burbleFadeFrom = 0.0, burbleFadeSpan = 0.0, burbleHigh = 1.0;
    double tick = 0.0;              // valvetrain: a click as each cam lobe snaps a valve shut
    double tickHz = 3200.0;         // ... centred here (a Desmo's rocker clatter sits lower)
    double turboGain = 1.0;         // the turbo's whistle, whoosh, blow-off and flutter
    // the turbulent rasp (broadband noise on the exhaust flow, around 1.2 kHz). It follows the summed
    // outflow squared, so with many cylinders whose exhaust strokes overlap (a V10: about three at
    // once) the flow hardly drops between pulses and the rasp drowns the firing note: turn it down
    double raspGain = 1.0;
    double dcBlockHz = 0.0;         // high-pass taking the DC out of the exhaust pulses; 0: from the
                                    // engine's idle firing rate (dcBlockCutoff in src/sound.hpp)
};

// turbocharger (the RS 3's, the tuned inline-4's), a mean-value model:
//   - exhaust: the gas fills the exhaust manifold and leaves through the turbine and the wastegate;
//     the turbine's power spins the shaft against the compressor's and the bearings'
//   - compressor (Greitzer): the air in the inlet duct has inertia and is pushed by the pressure
//     rise of the compressor map, which falls off on both sides of its peak. Right of the peak it
//     is stable; when the throttle shuts on boost the flow drops left of the peak and the
//     compressor surges: the flow stalls and reverses in bursts (the "flutter")
//   - charge pipes and intercooler: a volume between the compressor and the throttle, with a
//     pressure drop across the intercooler core. The compressor heats the air; the intercooler
//     takes out a share of that heat (more with air flowing through it at speed), and the warmer
//     charge fills the cylinders with less air
//   - blow-off valve: a diaphragm valve on the charge pipe, opened by manifold vacuum, that vents
//     the boost when the throttle shuts so the compressor does not surge
// The ECU works the wastegate to hold the boost the pedal asks for.
// supercharger = true (Kawasaki H2): the same compressor, charge pipes and bypass valve, but the shaft
// is geared to the crank (driveRatio) instead of driven by a turbine: no exhaust manifold, turbine or
// wastegate, the boost follows the revs, and the crank pays for the compressor's power (through a
// gear train of efficiency driveEff). ramGain: share of the dynamic pressure of the oncoming air a
// ram-air intake recovers at the compressor inlet.
struct turboParams {
    bool on = false;
    bool supercharger = false;
    double driveRatio = 9.2;        // impeller rpm per crank rpm (supercharger)
    double driveEff = 0.95;         // gear train
    double ramGain = 0.0;
    // plenum relief valve (supercharger): a spring valve that vents the charge above reliefP (Pa,
    // absolute; 0 = none), fully open reliefSpan higher. With no wastegate it is what caps the boost
    double reliefP = 0.0, reliefSpan = 10e3, reliefArea = 6.0e-4;
    bool bov = true;                // blow-off valve fitted (F toggles it, --bov no takes it off)
    double inertia = 2.0e-5;        // kg m^2, shaft + both wheels
    double maxRpm = 175000.0;       // overspeed: the shaft is held here
    double prRef = 2.4, rpmRef = 160000.0;  // compressor peak pressure ratio at rpmRef (rises with speed^2)
    double flowPeakRef = 0.116;     // kg/s at the peak of the map at rpmRef (scales with speed): the surge line
    double shutoffHead = 0.6;       // pressure rise at zero flow, share of the peak
    double chokeFall = 0.15;        // how fast the pressure rise falls right of the peak
    double ductLA = 0.6 / 1.6e-3;   // 1/m, inlet duct length / area (the air column's inertia)
    double vCharge = 5.0e-3;        // m^3, charge pipes + intercooler
    double icDrop = 2.0e5;          // Pa per (kg/s)^2: ~0.08 bar across the core at 0.2 kg/s
    double icEffStill = 0.5, icEffMoving = 0.8;  // intercooler effectiveness standing / from 30 m/s
    double icTau = 6.0;             // s, the core's heat soak
    double bovArea = 3.5e-4;        // m^2, blow-off valve open (21 mm)
    double bovCrack = 25e3;         // Pa: opens when the charge is this far over the manifold
    double bovSpan = 20e3;          // Pa more for fully open
    double etaC = 0.72, etaT = 0.68;  // compressor / turbine efficiency
    double turbineArea = 0.68e-3;   // m^2, effective flow area of the turbine nozzle (small: early spool)
    double wastegateArea = 3.0e-3;  // m^2, wastegate fully open
    double vExhaust = 2.0e-3;       // m^3, exhaust manifold + turbine housing
    double bearingLoss = 1.4e-6;    // W per (rad/s)^2: ~150 W at 100k rpm
    double boostMap = 160e3;        // Pa, manifold pressure at full pedal (~0.6 bar boost)
    // the ECU's boost target over rpm (engineSim::boostTarget): boostMap up to boostFallRpm, then
    // straight down to boostTop at boostTopRpm and held there (a real map lowers the boost at the top
    // end, for the turbine's back pressure, the exhaust heat and knock). boostFallRpm 0: flat
    double boostFallRpm = 0.0, boostTopRpm = 0.0, boostTop = 0.0;
    // torque limit (engineSim::boostTarget): the ECU caps the air per cycle, VE x manifold pressure /
    // charge temperature, at chargeMax times an atmospheric charge at tIntake (VE 1). Where the turbo
    // could give more, the boost is turned down and the torque plateaus. 0: none
    double chargeMax = 0.0;
    double wgKp = 2.0, wgKi = 10.0;  // wastegate PI: travel per bar over the target, per bar second
    double knockRetard = 8.0;       // deg of spark taken out per bar over atmospheric
    double knockRetardHot = 0.1;    // deg more per K of charge over 320 K
    double tExhLight = 850.0, tExhFull = 1150.0;  // K, exhaust gas at light and full load
};

struct engineParams {
    soundParams sound;
    turboParams turbo;
    double bore = 0.085;            // m
    double compression = 10.0;
    double gamma = 1.3;             // polytropic exponent, compression and expansion
    double ivc = 220.0 * DEGR;      // intake valve closes 40 deg after BDC
    double evo = 500.0 * DEGR;      // exhaust valve opens 40 deg before BDC
    double blowdown = 12.0 * DEGR;  // exhaust pressure decay length after EVO
    double pAtm = 101325.0;
    double pExhaust = 105000.0;
    double pCrankcase = 101325.0;
    double tIntake = 300.0;         // K
    double rGas = 287.0;
    double afr = 14.7;
    double lhv = 44e6;              // J/kg petrol
    double etaHeat = 0.85;          // fuel energy that ends up as gas pressure (combustion + wall losses)
    double burn = 50.0 * DEGR;      // Wiebe burn duration
    double wiebeA = 5.0, wiebeM = 2.0;
    // cycle-to-cycle variation: no two combustions are alike (mixture, turbulence and residual gas
    // differ from cycle to cycle), which is what makes an idle lope instead of humming. Each firing
    // draws its heat release and its burn duration at random around the mean. The coefficient of
    // variation of the heat goes from cycleCovIdle at idle-like manifold vacuum to cycleCovFull at
    // full load (real engines: IMEP CoV a few percent at idle, 1-2% at wide-open throttle); the burn
    // duration varies 1.5 times as much. 0 turns it off. Seeded: every run is the same.
    double cycleCovIdle = 0.03, cycleCovFull = 0.012;
    double leakTau = 1.0;           // s, ring blow-by time constant: ~1.5% of the charge at 3000 rpm, ~18% when cranking
    double mRecip = 0.6;            // kg per cylinder: piston + pin + 1/3 rod
    double inertia = 0.13;          // kg m^2: crank + flywheel + 2/3 rods
    double vManifold = 3.0e-3;      // m^3
    double throttleArea = 2.83e-3;  // m^2, 60 mm throttle bore
    double throttleCd = 0.8;
    double idleBase = 2.0e-5, idleMin = 1.6e-5, idleMax = 1.2e-4;  // m^2 idle bypass valve (held open on overrun)
    double idleKp = 1e-8, idleKi = 1.5e-8;       // slow air PI gains on filtered rpm, m^2 per rpm (per rpm s)
    double idleBand = 300.0;                     // rpm: air trim only integrates this close to idle
    double idleSparkBase = 8.0, idleSparkGain = 0.03;  // deg, deg per rpm: fast idle control through spark
    double idleRpm = 800.0;
    double starterTorque = 140.0;   // N m at the crank when stalled
    double starterMaxRpm = 300.0;
    double revLimit = 6800.0, revResume = 6500.0;
    double fmepA = 1.0e5, fmepB = 0.12e5, fmepC = 0.04e5;  // Pa, Pa per krpm, Pa per krpm^2 (A includes ~8 N m of accessories)
    double pedalRate = 4.0;         // throttle travel per second
    // breathing and spark maps
    double veBase = 0.80, vePeak = 0.12, veRpm = 4500.0, veWidth = 2800.0;  // VE = base + peak * gauss(rpm)
    // variable intake cam (BMW ShiftCam): a second, mild cam lobe with its own VE curve. The ECU
    // runs whichever lobe breathes better at this rpm. lowCam = false: one fixed cam (the curve above)
    bool lowCam = false;
    double veLowBase = 0.0, veLowPeak = 0.0, veLowRpm = 0.0, veLowWidth = 1.0;
    // pressure-wave (resonance) tuning: ripples on top of the smooth VE curve above, from the waves
    // that run up and down the intake runners and exhaust headers (engineSim::waveTuning explains the
    // model). The lengths are effective acoustic lengths: the pipe plus the port and the open end's
    // end correction. Gains are fractions of the trapped air; 0 switches a side off (the default).
    double ramLen = 0.0, ramDia = 0.04;   // m: intake runner (velocity stack, throttle body, port), its bore
    double ramSound = 345.0;              // m/s, speed of sound in the intake air (~ 25 C)
    double ramGain = 0.0, ramDecay = 0.5; // ram amplitude at the first wave (x = 0.5); decay per period
    // m: exhaust header, valve to the merge, collector or widening that reflects. 0 (the default): setupSim
    // measures it on the engine's drawn exhaust layout (headerLength, src/exhaust.hpp) and adds the port
    // in the head (scavPortLen) and the open end's correction (0.6 x the header's radius); an engine (or
    // SCAV_LEN) may set it outright instead
    double scavLen = 0.0;
    double scavPortLen = 0.05;            // m: the exhaust port in the head, valve seat to the head face
    double scavSound = 500.0;             // m/s, speed of the pressure waves in the hot exhaust (~ 350 C)
    double scavGain = 0.0, scavDecay = 0.5;
    double sparkLow = 8.0, sparkHigh = 30.0, sparkFrom = 800.0, sparkSpan = 3500.0;  // deg BTDC ramp
    double mapMin = 30e3;           // Pa, manifold pressure the drive-by-wire asks for at zero pedal
    double residualMin = 0.7;       // combustion effectiveness at very low load (residual gas dilution)
    // crank counterweights cancel this share of each cylinder's 1st-order reciprocating force. The
    // inline-4's flat-plane crank cancels it by symmetry (0); on a 90 deg V the pair on one pin
    // makes a constant rotating force that a counterweight opposite the pin cancels fully (0.5 each)
    double counterweight = 0.0;
    double rpmScale = 8000.0;       // HUD rpm bar full scale
    double fuelCutRpm = 1400.0;     // overrun fuel cut above this (throttle shut, in gear)
    // engine on its mounts (rigid block on spring-dampers), for the vibration readout and shake view
    double blockMass = 150.0;       // kg
    double blockRollInertia = 5.0;  // kg m^2 about the crank axis
    double mountFreqY = 12.0, mountFreqRoll = 9.0;  // Hz
    double mountZeta = 0.15;
};

struct cylinderState {
    double r = 0, L = 0, area = 0, vDisp = 0, vClear = 0;
    double z = 0;           // cylinder position along the crank, m (for the pitching moment)
    double p = 101325.0;
    double mAir = 0.0;      // trapped air at IVC, kg
    double qTot = 0.0;      // heat to release this cycle, J
    double xbDone = 0.0;    // Wiebe fraction already released
    double phiSpark = 0.0;
    double burn = 0.0;      // this cycle's burn duration, rad (engineParams::burn with its variation)
    double heatRate = 0.0;  // J/s
    double glowEvo = 0.0;   // glow when the exhaust valve opened, fades through blowdown
    bool sparked = false;
    int region = 0;         // 0 intake, 1 closed (compression/power), 2 exhaust
};

// ---------------------------------------------------------------------------------------------
// drivetrain: clutch, gearbox (any number of gears), final drive and the vehicle it pushes
// ---------------------------------------------------------------------------------------------
// The clutch is a friction coupling: it passes slip * stiffness up to engagement * capacity, so it
// slips on take-off, locks once the speeds match, and can drag the engine down (stall) when dumped.
// Automatic mode works the clutch from engine rpm (like a centrifugal clutch) and shifts on rpm,
// throttle and load: early upshifts at light throttle, late at full throttle, kickdown when floored,
// and a downshift when the engine labours (throttle open but the car is not accelerating, e.g. a hill).

struct drivetrainParams {
    // 2nd-generation (2009-2013) Honda Fit 5-speed manual, from Honda's 2010 spec sheet.
    // ratios[0] is neutral, ratios[1..gears] the gears: gears + 1 entries (validateDrivetrain checks)
    std::vector<double> ratios{0.0, 3.308, 1.870, 1.303, 0.949, 0.727};
    int gears = 5;
    bool bike = false;              // sequential box: 1-N-2-3..., no reverse, quickshifter, slipper clutch
    double primary = 1.0;           // crank -> gearbox input reduction (bikes have a primary gear pair)
    double reverseRatio = 3.308;    // gear -1, through an idler so the output turns backwards
    double finalDrive = 4.62;
    double wheelRadius = 0.305;     // 175/65R15 / 185/55R16 tyres, ~610 mm diameter
    double efficiency = 0.92;
    double carMass = 1300.0, wheelInertia = 3.6;  // kg, kg m^2 for all four wheels
    double crr = 0.012, cdA = 0.65, airDensity = 1.2;
    double clutchCapacity = 350.0;  // N m when fully engaged (well above the 203 N m peak)
    double clutchStiffness = 400.0; // N m per rad/s of slip while below capacity
    double inputInertia = 0.02;     // clutch disc + input shaft + countershaft, kg m^2
    double brakeDecel = 8.0;        // m/s^2 with the brake held
    double shiftOpen = 0.12, shiftClose = 0.25;            // automatic shift: clutch out / in, s
    double normalDown = 1200.0;     // normal mode: change down when the engine falls to this, rpm
    double sportUp = 6400.0;        // sport mode: change up here, just under the 6500-6800 limiter, rpm
    double sportDown = 5500.0;      // sport mode: change down whenever the lower gear lands below this, rpm
    double matchHold = 0.6;         // sport mode: longest the clutch waits for the rev-match blip, s
    double firstGearSpeed = 10.0 / 3.6;  // automatic: below this (m/s) go straight back to 1st
    double clutchPressRate = 6.0, clutchReleaseRate = 1.6; // manual pedal travel per s
    // automatic: clutch and shift rpm thresholds
    double openRpm = 900.0;         // clutch fully open below this
    double idleOpenRpm = 1000.0;    // ... or below this with the throttle shut
    double biteSpan = 1300.0;       // take-off: engagement rises from openRpm over this many rpm
    double launchPedal = 0.0;       // take-off: the bite point moves up this many rpm at full throttle
    double upBase = 2000.0, upPedal = 4300.0;  // upshift at upBase + upPedal * pedal
    double upMargin = 250.0;        // the next gear must land this far above normalDown
    double downMax = 6000.0;        // never change down into more than this
    double kickRpm = 4500.0, kickMax = 5600.0, loadDownMax = 4800.0;
    double pickRpm = 2500.0, pickMax = 6000.0;  // switching to automatic: gear nearest pickRpm
    // bikes
    double slipper = 1.0;           // share of the clutch capacity that passes engine braking
    double quickshiftTime = 0.0;    // s of ignition cut (up) or blip (down) per clutchless shift; 0 = none
    bool dualClutch = false;        // cars: a dual-clutch box (drawn with two clutch packs; the RS 3's S tronic)
    // quickshift down, rev-matched: blip until the drive torque through the dogs crosses zero (or
    // quickshiftTime runs out), the old gear drops out, the ECU revs the engine to the lower gear's
    // speed, and the new gear goes in once the speeds agree (or after qsMatchTimeout)
    double qsUnloadTorque = 6.0;    // N m at the crank: below this the dogs are free to slide out
    double qsMatchTol = 0.015;      // the new gear engages within this share of its speed
    double qsMatchTimeout = 0.25;   // s between gears at most
    double qsMaxRpm = 1e9;          // a downshift that would land above this is refused (set to the limiter)
    double wheelieAccel = 0.0;      // m/s^2 the wheelie control allows, 0 = none fitted
    // bike gearbox drawing: real tooth counts, indexed like ratios (empty, or 0 for a gear: derive the
    // teeth from the ratio), primary pair, sprocket
    std::vector<int> teethIn, teethOut;
    int primaryIn = 30, primaryOut = 54, sprocketTeeth = 16;
};

// the most gears the gearbox drawings and the gear tables are laid out for
const int MAX_GEARS = 10;

// a drivetrain's gear tables must fit its gear count: throws std::runtime_error saying what is wrong
void validateDrivetrain(const drivetrainParams &P) {
    auto fail = [](const std::string &what) { throw std::runtime_error("drivetrainParams: " + what); };
    if (P.gears < 1 || P.gears > MAX_GEARS)
        fail("gears = " + std::to_string(P.gears) + ", must be 1.." + std::to_string(MAX_GEARS));
    if (P.ratios.size() != static_cast<std::size_t>(P.gears) + 1)
        fail("ratios has " + std::to_string(P.ratios.size()) + " entries, needs gears + 1 = " + std::to_string(P.gears + 1) +
             " (index 0 is neutral)");
    for (int g = 1; g <= P.gears; g++)
        if (!(P.ratios[g] > 0.0) || !std::isfinite(P.ratios[g]))
            fail("ratios[" + std::to_string(g) + "] = " + std::to_string(P.ratios[g]) + ": every gear needs a ratio > 0");
    for (const std::vector<int> *teeth : {&P.teethIn, &P.teethOut}) {
        const char *name = teeth == &P.teethIn ? "teethIn" : "teethOut";
        if (!teeth->empty() && teeth->size() != P.ratios.size())
            fail(std::string(name) + " has " + std::to_string(teeth->size()) + " entries, needs none or gears + 1 = " +
                 std::to_string(P.gears + 1));
        for (int t : *teeth)
            if (t < 0) fail(std::string(name) + " has a negative tooth count");
    }
    if (P.teethIn.empty() != P.teethOut.empty()) fail("teethIn and teethOut must both be given or both be empty");
    if (!P.bike && !(P.reverseRatio > 0.0)) fail("a car needs reverseRatio > 0");
    if (!(P.primary > 0.0) || !(P.finalDrive > 0.0) || !(P.wheelRadius > 0.0))
        fail("primary, finalDrive and wheelRadius must be > 0");
}

struct drivetrain {
    drivetrainParams P;
    bool automatic = true;
    bool sport = false;            // automatic sub-mode: hold low gears, rev-matched downshifts
    // manual sub-mode (cars, Z in manual): false = NORMAL, a clutch pedal for every change and for pulling
    // away; true = DCT, the driver picks the gears (E/Q) and the box works the clutch like a dual-clutch
    // gearbox in its manual mode: take-off, each change (blipping to the lower gear's speed on a
    // downshift), and anti-stall (the clutch opens near idle and it drops to 1st as the car stops)
    bool dct = false;
    bool dctOn() const { return dct && !automatic && !P.bike; }
    bool boxWorksClutch() const { return automatic || dctOn(); }
    int gear = 1, targetGear = 1;  // -1 = reverse, 0 = neutral, 1..gears
    int qsPhase = 0;               // quickshift under way: 1 up (ignition cut), -1 down (blip)
    bool dogsFree = false;         // quickshift down: the old gear is out, engine and wheels are apart
    bool coupled() const { return gear != 0 && !dogsFree; }
    bool wheelieControl = true;    // rider setting (V key); only matters when P.wheelieAccel > 0
    bool wheelieOn() const { return wheelieControl && P.wheelieAccel > 0.0; }
    double qsTimer = 0.0;
    bool revMatch = false;         // this shift is a downshift the engine blips the throttle for
    bool toFirst = false;          // this shift is the drop to 1st when (nearly) stopped
    double matchRpm = 0.0;         // engine rpm the target gear needs at the current road speed
    double clutch = 0.0;           // engagement 0..1
    bool clutchPedal = false;      // manual: held = disengaged
    bool brake = false;
    double v = 0.0;                // road speed, m/s
    double omegaIn = 0.0;          // gearbox input shaft, rad/s
    double grade = 0.0;            // rise / run
    double cargo = 0.0;            // extra kg
    double clutchTorque = 0.0, wheelForce = 0.0, accel = 0.0, distance = 0.0;
    int shiftPhase = 0;            // 0 none, 1 clutch opening, 2 clutch closing
    double lastShift = -10.0;
    double inputAngle = 0.0, outputAngle = 0.0;  // for drawing
    std::string message;
    double messageTime = -10.0;

    void reset(bool autoMode) {
        automatic = autoMode;
        gear = targetGear = autoMode ? 1 : 0;
        clutch = 0.0;
        clutchPedal = brake = false;
        v = omegaIn = clutchTorque = wheelForce = accel = distance = 0.0;
        shiftPhase = 0;
        qsPhase = 0;
        qsTimer = 0.0;
        dogsFree = false;
        revMatch = toFirst = false;
        lastShift = -10.0;
        inputAngle = outputAngle = 0.0;
        message.clear();
    }

    double ratio() const { return P.primary * (gear >= 0 ? P.ratios[gear] : -P.reverseRatio); }  // signed, crank to output
    double mass() const { return P.carMass + cargo; }
    double outputOmega() const { return v / P.wheelRadius * P.finalDrive; }  // gearbox output shaft
    double omegaFromWheels() const { return outputOmega() * ratio(); }
    double inputRpm() const { return omegaIn * 60.0 / (2.0 * 3.14159265358979323846); }
    void say(const std::string &m, double now) { message = m; messageTime = now; }

    // shifting: +1 up, -1 down. Manual: a gear only goes in with the clutch pressed, and reverse
    // (below neutral) only with the car stopped. Automatic: Q at a standstill selects R, E goes back to D.
    // sequential bike box: 1 - N - 2 - 3 - ... A full click skips neutral, N selects it from 1st or 2nd.
    // Moving in gear with the clutch home, the quickshifter changes without the lever: an ignition cut
    // to unload the dogs going up, a throttle blip going down.
    // gear that suits the current road speed (1st when stopped)
    int pickGear() const {
        int best = 1;
        double bestErr = 1e9;
        for (int g = 1; g <= P.gears; g++) {
            double r = outputOmega() * P.primary * P.ratios[g] * 60.0 / (2.0 * 3.14159265358979323846);
            if (std::fabs(r - P.pickRpm) < bestErr && r < P.pickMax) { best = g; bestErr = std::fabs(r - P.pickRpm); }
        }
        return best;
    }

    // N toggles neutral from any gear, in automatic and manual. Leaving neutral picks the gear for the
    // road speed (1st when stopped); in manual that needs the clutch lever pulled.
    void bikeShift(int dir, double now, bool toNeutral) {
        if (toNeutral && gear != 0) {
            gear = targetGear = 0;
            qsPhase = shiftPhase = 0;
            dogsFree = false;
            revMatch = toFirst = false;
            say("N", now);
            return;
        }
        if (qsPhase != 0 || shiftPhase != 0) return;
        if (automatic) {
            if (gear != 0) { say("automatic: press T for manual, N for neutral", now); return; }
            gear = targetGear = pickGear();
            lastShift = now;
            if (std::fabs(v) > P.firstGearSpeed) {
                // rolling: blip up to the gear's speed, then close the clutch (the rev-matched shift path)
                clutch = 0.0;
                shiftPhase = 2;
                revMatch = true;
                matchRpm = outputOmega() * P.primary * P.ratios[gear] * 60.0 / (2.0 * 3.14159265358979323846);
            }
            say("D (" + std::to_string(gear) + ")", now);
            return;
        }
        int g;
        if (toNeutral) {
            g = std::fabs(v) < 1.0 ? 1 : pickGear();  // N again: back into gear
        } else if (gear == 0) {
            g = dir > 0 ? 2 : 1;
        } else {
            g = std::clamp(gear + dir, 1, P.gears);
        }
        if (g == gear) return;
        bool moving = std::fabs(v) > 1.0;
        if (P.quickshiftTime > 0.0 && clutch > 0.3 && gear >= 1 && g >= 1 && moving) {
            double landing = outputOmega() * P.primary * P.ratios[g] * 60.0 / (2.0 * 3.14159265358979323846);
            if (g < gear && landing > P.qsMaxRpm) {  // over-rev protection, like the real ECU
                say("downshift refused: " + std::to_string(g) + " would be " +
                        std::to_string(static_cast<int>(std::lround(landing / 100.0) * 100)) + " rpm", now);
                return;
            }
            targetGear = g;
            qsPhase = g > gear ? 1 : -1;
            qsTimer = P.quickshiftTime;
            dogsFree = false;
            matchRpm = outputOmega() * P.primary * P.ratios[g] * 60.0 / (2.0 * 3.14159265358979323846);
            say(std::string("quickshift ") + std::to_string(gear) + " -> " + std::to_string(g) +
                    (g < gear ? " (rev match " + std::to_string(static_cast<int>(std::lround(matchRpm / 100.0) * 100)) + " rpm)" : ""),
                now);
            lastShift = now;
            return;
        }
        if ((gear == 0 || !moving) && clutch > 0.3 && g != 0) {
            say("pull C (clutch) to select a gear", now);
            return;
        }
        gear = targetGear = g;
        say(g == 0 ? "N" : std::to_string(g), now);
    }

    void requestShift(int dir, double now) {
        if (P.bike) { bikeShift(dir, now, false); return; }
        bool stopped = std::fabs(v) < 0.5;
        if (automatic) {
            if (shiftPhase != 0) return;
            if (dir < 0 && gear >= 1) {
                if (!stopped) { say("stop the car to select reverse", now); return; }
                gear = targetGear = -1;
                say("R", now);
            } else if (dir > 0 && gear == -1) {
                if (!stopped) { say("stop the car to select drive", now); return; }
                gear = targetGear = 1;
                say("D", now);
            }
            return;
        }
        if (shiftPhase != 0) return;  // a DCT change is still going in
        int g = std::clamp(gear + dir, -1, P.gears);
        if (g == gear) return;
        if (dctOn() && g == -1 && !stopped) {
            say("stop the car before selecting reverse", now);
            return;
        }
        if (dctOn() && gear >= 1 && g >= 1 && clutch > 0.3) {
            // DCT, in gear with the clutch closed: the box opens it, swaps gears and closes it again
            double landing = outputOmega() * P.primary * P.ratios[g] * 60.0 / (2.0 * 3.14159265358979323846);
            if (g < gear && landing > P.qsMaxRpm) {  // over-rev protection
                say("downshift refused: " + std::to_string(g) + " would be " +
                        std::to_string(static_cast<int>(std::lround(landing / 100.0) * 100)) + " rpm", now);
                return;
            }
            targetGear = g;
            shiftPhase = 1;
            lastShift = now;
            revMatch = g < gear;
            matchRpm = landing;
            say(std::to_string(gear) + " -> " + std::to_string(g) +
                    (revMatch ? " (rev match " + std::to_string(static_cast<int>(std::lround(matchRpm / 100.0) * 100)) + " rpm)" : ""),
                now);
            return;
        }
        if (dctOn()) {
            // DCT, out of or into N or R: with the clutch open the gear just goes in; with it closed (as
            // it sits in N) the box opens it first. Either way the take-off logic then bites on the throttle
            say(g == 0 ? "N" : g == -1 ? "R" : std::to_string(g), now);
            revMatch = false;
            if (clutch < 0.05) {
                gear = targetGear = g;
            } else {
                targetGear = g;
                toFirst = true;  // after the swap: back to the take-off clutch, not a full engagement
                shiftPhase = 1;
                lastShift = now;
            }
            return;
        }
        if (g != 0 && clutch > 0.3) {
            say("hold C (clutch) to change gear", now);
            return;
        }
        if (g == -1 && !stopped) {
            say("stop the car before selecting reverse", now);
            return;
        }
        gear = targetGear = g;
        if (g == -1) say("R", now);
    }

    void setSport(bool on, double now) {
        sport = on;
        if (automatic) say(on ? "SPORT: holds gears, rev-matched downshifts" : "NORMAL", now);
        else say(on ? "SPORT (active in automatic, press T)" : "NORMAL", now);
    }

    // manual NORMAL <-> DCT (cars). The clutch carries on from where it is: a held pedal still opens
    // it in NORMAL, and DCT takes over from the current engagement
    void setDct(bool on, double now) {
        dct = on;
        shiftPhase = 0;
        revMatch = toFirst = false;
        say(on ? "MANUAL DCT: E/Q shift, no clutch, anti-stall" : "MANUAL NORMAL: hold C to shift and pull away", now);
    }

    // the Z key: sport in automatic; in a car's manual, NORMAL <-> DCT
    void toggleMode(double now) {
        if (!automatic && !P.bike) setDct(!dct, now);
        else setSport(!sport, now);
    }

    void setAutomatic(bool a, double rpmNow, double now) {
        automatic = a;
        shiftPhase = 0;
        revMatch = toFirst = false;
        if (!a) {
            // hand over in neutral with the clutch where it is, so nothing lurches or stalls
            gear = targetGear = 0;
            clutchPedal = false;
            say(P.bike ? "MANUAL: neutral. Hold C, press Q for 1st, release C gently"
                : dct ? "MANUAL DCT: neutral. E for 1st, then the throttle (Z: normal)"
                      : "MANUAL: neutral. Hold C, E for 1st, release C gently (Z: DCT)", now);
        } else {
            // pick the gear that puts the engine nearest pickRpm for this road speed
            int best = pickGear();
            if (gear != -1 || v > 0.5) gear = targetGear = best;  // stay in R if reversing
            say(gear == -1 ? "AUTOMATIC (R)" : "AUTOMATIC", now);
        }
        (void)rpmNow;
    }

    void control(double dt, double pedal, double rpmFilt, double now) {
        if (qsPhase > 0) {  // quickshift up: the gear goes in once the ignition cut has unloaded the dogs
            qsTimer -= dt;
            if (qsTimer <= 0.0) {
                gear = targetGear;
                qsPhase = 0;
            }
        } else if (qsPhase < 0) {  // quickshift down, rev-matched
            qsTimer -= dt;
            matchRpm = outputOmega() * P.primary * P.ratios[targetGear] * 60.0 / (2.0 * 3.14159265358979323846);
            if (!dogsFree) {
                // the blip takes the load off the dogs: once the torque through them crosses zero, out
                if (std::fabs(clutchTorque) < P.qsUnloadTorque || qsTimer <= 0.0) {
                    dogsFree = true;
                    qsTimer = P.qsMatchTimeout;
                }
            } else {
                // between gears: in when the input shaft runs at the new gear's speed
                double inRpm = omegaIn * 60.0 / (2.0 * 3.14159265358979323846);
                if (std::fabs(inRpm - matchRpm) < P.qsMatchTol * matchRpm || qsTimer <= 0.0) {
                    gear = targetGear;
                    qsPhase = 0;
                    dogsFree = false;
                }
            }
        }
        if (!boxWorksClutch() && shiftPhase == 0) {
            clutch = std::clamp(clutch + (clutchPedal ? -P.clutchPressRate : P.clutchReleaseRate) * dt, 0.0, 1.0);
            return;
        }
        // a shift the box is working the clutch for: automatic, or manual DCT
        if (shiftPhase != 0 && revMatch) matchRpm = outputOmega() * P.primary * P.ratios[targetGear] * 60.0 / (2.0 * 3.14159265358979323846);
        if (shiftPhase == 1) {  // clutch out, then swap gears
            clutch -= dt / P.shiftOpen;
            if (clutch <= 0.0) {
                clutch = 0.0;
                gear = targetGear;
                shiftPhase = 2;
                // dropping to 1st near a stop: let the take-off clutch logic below bring it back in
                // gently, a full engagement at walking pace would stall the engine
                if (toFirst) shiftPhase = 0, toFirst = false;
            }
            return;
        }
        if (shiftPhase == 2) {  // clutch back in; a rev-matched downshift waits for the blip to get there
            bool waiting = revMatch && rpmFilt < matchRpm - 200.0 && now - lastShift < P.matchHold;
            if (!waiting) clutch += dt / P.shiftClose;
            if (clutch >= 1.0) {
                clutch = 1.0;
                shiftPhase = 0;
                revMatch = false;
            }
            return;
        }
        // automatic clutch: open near idle so the engine can't stall, slip on take-off
        double target;
        // braking to a stop with the throttle shut: open early, in 1st the wheels would drag the engine down
        bool brakingToStop = brake && pedal < 0.03 && std::fabs(v) < P.firstGearSpeed;
        bool launch = false;
        if (rpmFilt < P.openRpm || (pedal < 0.03 && rpmFilt < P.idleOpenRpm) || brakingToStop || gear == 0) target = 0.0;  // open in N
        else if (P.launchPedal > 0.0 && gear <= 2 && launching(rpmFilt)) {
            // launch control: the engine is held at launchRpm (ignition cut, see engineSim) and the
            // clutch closes as far as the revs allow, but never past what would lift the front wheel
            double kRatio = ratio() * P.finalDrive / P.wheelRadius;
            double engMax = wheelieOn() ? std::min(1.0, mass() * P.wheelieAccel / (P.clutchCapacity * kRatio * P.efficiency)) : 1.0;
            // a fixed rpm -> engagement map settles where the clutch carries what the engine makes
            target = std::clamp((rpmFilt - launchRpm(pedal) + 0.5 * P.biteSpan) / P.biteSpan, 0.0, engMax);
            launch = true;
        }
        else if (std::fabs(v) < 3.0 && gear <= 2) target = std::clamp((rpmFilt - P.openRpm - P.launchPedal * pedal) / P.biteSpan, 0.0, 1.0);
        else target = 1.0;
        // brakes on at a standstill (launch control: brake and throttle together): the box lets the clutch
        // carry only what the brakes can hold, so the car stays put while the engine works against them,
        // held at the launch rpm, and the turbo builds boost. Let go of the brake and it launches on boost
        if (brake && std::fabs(v) < 0.5 && gear != 0) {
            double kRatio = std::fabs(ratio()) * P.finalDrive / P.wheelRadius;
            target = std::min(target, 0.8 * mass() * P.brakeDecel / (P.clutchCapacity * kRatio * P.efficiency));
        }
        double rate = target > clutch || launch ? 2.5 : 8.0;  // launch: ease both ways so it doesn't hunt
        clutch += std::clamp(target - clutch, -rate * dt, rate * dt);

        // stopped or nearly: go straight to 1st (skipping the gears between), ready to pull away
        if (gear > 1 && std::fabs(v) < P.firstGearSpeed) {
            say("downshift " + std::to_string(gear) + " -> 1 (slow)", now);
            lastShift = now;
            revMatch = false;
            if (clutch < 0.05) {
                gear = targetGear = 1;  // clutch already open: just select it
            } else {
                targetGear = 1;
                toFirst = true;
                shiftPhase = 1;
            }
            return;
        }
        if (!automatic) return;  // manual DCT: the driver picks the gears

        // shift schedule
        if (gear < 1 || now - lastShift < 1.0 || v < 2.0 || clutch < 0.99) return;  // no shifting in R
        auto rpmIn = [&](int g) { return rpmFilt * P.ratios[g] / P.ratios[gear]; };
        double up = P.upBase + P.upPedal * pedal, down = P.normalDown;
        // asking for power but losing speed (hill, cargo): change down; don't change up while not gaining
        bool losing = pedal > 0.3 && accel < -0.1;
        bool notGaining = pedal > 0.3 && accel < 0.05;
        int want = gear;
        const char *why = "";
        bool slow = std::fabs(v) < P.firstGearSpeed + 1.0;  // no upshift until clear of the drop-to-1st speed
        if (sport) {
            // keep the engine in the lowest gear that stays clear of the limiter
            if (gear < P.gears && rpmFilt > P.sportUp && !slow) { want = gear + 1; why = "upshift"; }
            else if (gear > 1 && rpmIn(gear - 1) < P.sportDown) { want = gear - 1; why = "downshift"; }
        } else if (gear < P.gears && rpmFilt > up && rpmIn(gear + 1) > down + P.upMargin && !notGaining && !slow) { want = gear + 1; why = "upshift"; }
        else if (gear > 1 && rpmFilt < down && rpmIn(gear - 1) < P.downMax) { want = gear - 1; why = "downshift"; }
        else if (gear > 1 && pedal > 0.85 && rpmFilt < P.kickRpm && rpmIn(gear - 1) < P.kickMax) { want = gear - 1; why = "kickdown"; }
        else if (gear > 1 && losing && rpmIn(gear - 1) < P.loadDownMax) { want = gear - 1; why = "downshift (load)"; }
        if (want != gear) {
            targetGear = want;
            shiftPhase = 1;
            lastShift = now;
            revMatch = sport && want < gear;
            matchRpm = rpmIn(want);
            say(std::string(why) + (revMatch ? " (rev match) " : " ") + std::to_string(gear) + " -> " + std::to_string(want), now);
        }
    }

    // bikes with launch control keep slipping the clutch until the engine and wheels meet, instead of
    // closing it at walking pace and dragging a light crank out of its power band
    double launchRpm(double pedal) const { return P.openRpm + P.launchPedal * pedal + 0.5 * P.biteSpan; }
    bool launching(double rpmEngine) const {
        double wheelRpm = omegaFromWheels() * 60.0 / (2.0 * 3.14159265358979323846);
        return P.launchPedal > 0.0 && gear >= 1 && std::fabs(v) < 25.0 && rpmEngine - wheelRpm > 300.0;
    }

    // torque the clutch puts on the engine (positive = load on the engine)
    double clutchTorqueFor(double omegaEngine, double engineInertia, double dt) {
        double side = coupled() ? omegaFromWheels() : omegaIn;
        double kRatio = coupled() ? ratio() * P.finalDrive / P.wheelRadius : 0.0;
        double sideInertia = coupled() ? mass() / (kRatio * kRatio) + P.inputInertia : P.inputInertia;
        // stiff but explicit-stable: the slip time constant stays a few steps long
        double k = std::min(P.clutchStiffness, 0.4 * std::min(engineInertia, sideInertia) / dt);
        double cap = clutch * P.clutchCapacity;
        clutchTorque = std::clamp(k * (omegaEngine - side), -cap * P.slipper, cap);  // a slipper clutch slips on overrun
        return clutchTorque;
    }

    void integrate(double dt, double pedal) {
        const double g = 9.81;
        double th = std::atan(grade), m = mass();
        double kRatio = ratio() * P.finalDrive / P.wheelRadius;
        wheelForce = coupled() ? clutchTorque * kRatio * P.efficiency : 0.0;
        double resist = P.crr * m * g * std::cos(th) * std::tanh(v / 0.3) +
                        0.5 * P.airDensity * P.cdA * v * std::fabs(v) + m * g * std::sin(th);
        double mEff = m + P.wheelInertia / (P.wheelRadius * P.wheelRadius) +
                      (coupled() ? P.inputInertia * kRatio * kRatio : 0.0);
        // brake, plus hill-hold when the box works the clutch (automatic, manual DCT): stopped with the
        // throttle closed, the brakes stay on until the throttle opens
        bool hold = brake || (boxWorksClutch() && pedal < 0.03 && std::fabs(v) < 0.5);
        double brakeMax = hold ? m * P.brakeDecel : 0.0;
        double fNet = wheelForce - resist;
        double a;
        if (brakeMax > 0.0 && std::fabs(v) < 0.05 && std::fabs(fNet) <= brakeMax) {
            v = 0.0;  // brakes hold the car still
            a = 0.0;
        } else {
            fNet -= brakeMax * std::tanh(v / 0.05);
            a = fNet / mEff;
            v += a * dt;
        }
        accel += (a - accel) * std::min(1.0, dt / 0.5);
        if (!coupled()) omegaIn += (clutchTorque - 0.3 * std::tanh(omegaIn)) / P.inputInertia * dt;
        else omegaIn = omegaFromWheels();
        inputAngle += omegaIn * dt;
        outputAngle += outputOmega() * dt;
        distance += v * dt;
    }
};

struct engineSim {
    engineParams P;
    // one entry per cylinder, sized from the engine mesh by init()
    cylArray<cylinderState> cyl;
    cylArray<double> lag;                // firing lag of each cylinder behind cylinder 1, rad
    cylArray<double> bankCos, bankSin;   // bore direction: up = cos, sideways = sin
    cylArray<double> dxs, d2xs;          // step() scratch: piston travel derivatives per cylinder
    int cylCount() const { return static_cast<int>(cyl.size()); }
    bool standalone = false;       // engine on a stand: no clutch, gearbox or vehicle (--v4 --engine-only)
    bool qsCut = false;            // quickshifter ignition cut
    double tcTrim = 1.0;           // wheelie control: share of the fuel the ECU lets burn
    double wheelForceFilt = 0.0;   // N, smoothed over a few firings for the wheelie control
    double theta = 0.0, omega = 0.0;
    double pMan = 101325.0;
    double idleArea = 0.0, idleInt = 0.0;
    double pedal = 0.0, pedalTarget = 0.0;
    bool ignition = true, starter = false, revCut = false;
    bool fuelCut = false;     // overrun fuel cut: throttle shut while the wheels drive the engine
    bool stalled = true;      // ECU has lost the engine: no fuel below 120 rpm unless cranking (or bump-started past 300)
    drivetrain drive;
    bool idleActive = false;  // idle speed control engaged
    double rpmFilt = 0.0;     // rpm averaged over ~a firing interval, what the ECU controls on
    bool lockSpeed = false;  // dyno sweep: hold omega fixed
    double vDispTotal = 0.0;
    std::uint32_t cycleRng = 0x9e3779b9u;  // cycle-to-cycle variation (engineParams::cycleCovIdle), seeded
    // outputs
    double gasTorque = 0.0, frictionTorque = 0.0, starterTorque = 0.0, loadTorque = 0.0;
    double brakeAvg = 0.0;   // smoothed gas - friction torque
    double time = 0.0;
    double alpha = 0.0;      // crank angular acceleration this step
    double alphaFilt = 0.0;  // ... averaged over ~a firing interval (the quickshift blip predicts with it)
    double throttleFlow = 0.0;  // kg/s through throttle + idle valve (intake hiss)
    double throttleOpen = 0.0;  // throttle plate opening 0..1 set by the drive-by-wire
    double blip = 0.0;          // rev-match throttle the gearbox asks for on a sport downshift, 0..1
    // turbo (P.turbo.on); without it pExh stays at P.pExhaust and pBoost at P.pAtm
    double pExh = 105000.0;     // Pa, exhaust manifold (back pressure on the exhaust stroke)
    double pBoost = 101325.0;   // Pa, throttle inlet (after the intercooler)
    double pCharge = 101325.0;  // Pa, compressor outlet (charge pipes)
    double compFlow = 0.0;      // kg/s through the compressor (negative: surge backflow)
    double airFlowFilt = 0.0;   // kg/s through the throttle, averaged over ~5 ms (it chatters step to step at WOT)
    double bovOpen = 0.0, bovFlow = 0.0;  // blow-off valve 0..1, kg/s vented
    double surge = 0.0;         // 0..1, how hard the compressor is surging (decays after each stall)
    double tComp = 300.0;       // K, compressor outlet
    double tCharge = 300.0;     // K, after the intercooler: what the cylinders breathe (P.tIntake without the turbo)
    double turboOmega = 0.0;    // rad/s, turbo shaft
    double wastegate = 0.0;     // 0 shut .. 1 open
    double wgInt = 0.0;         // wastegate PI integral
    double turbinePower = 0.0, compressorPower = 0.0;  // W
    double chargerTorque = 0.0; // N m the crank spends driving the supercharger (P.turbo.supercharger)
    double pInlet = 101325.0;   // Pa, compressor inlet (above atmospheric with ram air)
    double reliefOpen = 0.0, reliefFlow = 0.0;  // plenum relief valve 0..1, kg/s vented
    double turboRpm() const { return turboOmega * 60.0 / (2.0 * 3.14159265358979323846); }
    double boostBar() const { return (pMan - P.pAtm) / 1e5; }

    // vibration: forces the moving parts put into the block
    double shakeForce = 0.0;   // N, vertical, from piston accelerations
    double rollMoment = 0.0;   // N m about the crank axis, reaction to the crank torque
    double pitchMoment = 0.0;  // N m, vertical forces times their position along the crank
    double h1c = 0, h1s = 0, h2c = 0, h2s = 0;   // 1st / 2nd order components (running Fourier)
    double rollMean = 0, rollVar = 0, pitchVar = 0;
    double mountY = 0, mountVy = 0, mountRoll = 0, mountVr = 0;  // block motion on its mounts

    // cylinder 1 p-V loop, one sample per crank degree over the last 720 degrees
    std::array<float, 720> pvV{}, pvP{};
    int pvLast = -1;

    double shake1() const { return 2.0 * std::hypot(h1c, h1s); }
    double shake2() const { return 2.0 * std::hypot(h2c, h2s); }
    double rollAmp() const { return std::sqrt(2.0 * rollVar); }
    double pitchAmp() const { return std::sqrt(2.0 * pitchVar); }

    void init(const engineModel &e, double theta1) {
        const std::size_t n = e.cyls.size();
        if (n == 0) throw std::runtime_error("the engine has no cylinders");
        cyl.assign(n, cylinderState{});
        lag.assign(n, 0.0);
        bankCos.assign(n, 1.0);
        bankSin.assign(n, 0.0);
        dxs.assign(n, 0.0);
        d2xs.assign(n, 0.0);
        for (std::size_t c = 0; c < n; c++) {
            cylinderState &s = cyl[c];
            s.r = e.cyls[c].throw_ * 1e-3;
            s.L = e.cyls[c].rodL * 1e-3;
            s.area = 0.25 * 3.14159265358979323846 * P.bore * P.bore;
            s.vDisp = s.area * 2.0 * s.r;
            s.vClear = s.vDisp / (P.compression - 1.0);
            s.z = e.cyls[c].zCentre * 1e-3;
            lag[c] = e.cyls[c].firingLagDeg * DEGR;  // exact in double, like the old constant table
            bankCos[c] = std::cos(e.cyls[c].bank);
            bankSin[c] = std::sin(e.cyls[c].bank);
        }
        vDispTotal = 0.0;
        for (auto &s : cyl) vDispTotal += s.vDisp;
        reset(theta1);
    }

    void reset(double theta1) {
        theta = wrapCycle(theta1);
        omega = 0.0;
        pMan = P.pAtm;
        pExh = P.pExhaust;
        pBoost = pCharge = P.pAtm;
        compFlow = airFlowFilt = bovOpen = bovFlow = surge = 0.0;
        tComp = tCharge = P.tIntake;
        turboOmega = wastegate = wgInt = turbinePower = compressorPower = chargerTorque = 0.0;
        pInlet = P.pAtm;
        reliefOpen = reliefFlow = 0.0;
        idleArea = P.idleBase;
        idleInt = 0.0;
        rpmFilt = 0.0;
        pedal = pedalTarget = 0.0;
        revCut = false;
        stalled = true;
        qsCut = false;
        tcTrim = 1.0;
        wheelForceFilt = 0.0;
        brakeAvg = 0.0;
        time = 0.0;
        alpha = throttleFlow = 0.0;
        shakeForce = rollMoment = pitchMoment = 0.0;
        h1c = h1s = h2c = h2s = rollMean = rollVar = pitchVar = 0.0;
        mountY = mountVy = mountRoll = mountVr = 0.0;
        for (int d = 0; d < 720; d++) {
            pvV[d] = static_cast<float>(volume(cyl[0], d * DEGR));
            pvP[d] = static_cast<float>(P.pAtm);
        }
        pvLast = -1;
        drive.reset(drive.automatic);
        for (int c = 0; c < cylCount(); c++) {
            cylinderState &s = cyl[c];
            double phi = phase(c);
            s.region = regionOf(phi);
            s.p = P.pAtm;
            s.mAir = s.region == 1 ? trappedAir(s, 0.0) : 0.0;
            s.qTot = s.xbDone = s.heatRate = s.glowEvo = 0.0;
            // a cylinder already past its spark point has nothing left to burn
            s.sparked = s.region == 1 && phi > 340.0 * DEGR;
        }
    }

    double rpm() const { return omega * 60.0 / (2.0 * 3.14159265358979323846); }
    double phase(int c) const { return wrapCycle(theta - lag[c]); }
    int regionOf(double phi) const { return phi < P.ivc ? 0 : (phi < P.evo ? 1 : 2); }

    // slider-crank: piston travel from TDC and its first two derivatives with crank angle
    static void geom(double phi, double r, double L, double &x, double &dx, double &d2x) {
        double s = std::sin(phi), c = std::cos(phi);
        double k = std::sqrt(L * L - r * r * s * s);
        x = r * (1.0 - c) + L - k;
        dx = r * s * (1.0 + r * c / k);
        d2x = r * c + r * r * (c * c - s * s) / k + r * r * r * r * s * s * c * c / (k * k * k);
    }

    double volume(const cylinderState &s, double phi) const {
        double x, dx, d2x;
        geom(phi, s.r, s.L, x, dx, d2x);
        return s.vClear + s.area * x;
    }

    // Pressure-wave tuning: the dips and humps a real dyno curve has on top of its smooth hump. Both
    // sides are the same picture: a valve event starts a pressure wave in a pipe, the wave rings back
    // and forth, and what decides the filling is the sign of the pressure at the valve at one later
    // moment. With x = (time from the event to that moment) / (the ringing's period):
    //     wave(x) = gain * env(x) * -cos(2 pi x)
    // a suction at the start, the opposite sign (a help) half a period later (x = 0.5, 1.5, 2.5 ...),
    // the same sign again (a loss) at whole periods (x = 1, 2, ...). Each period the wave loses some
    // of its energy (friction, leaks at the plenum or collector): env = exp(-decay (x - 0.5)). Lower
    // rpm means more periods fit, so the higher-order humps at low rpm are weaker; below x = 0.5 the
    // wave has not come back yet and env fades it out ((2x)^2). x scales as 1/rpm, so the humps sit at
    // rpm_k = rpm(x = 0.5) / (2k + 1) and the dips halfway between, on a 1/rpm (not linear) spacing.
    //   intake (inertial ram, Engelman's Helmholtz model): with the valve open, the air column in the
    //     runner (length L, bore area A) bounces on the gas in the cylinder (volume V, taken at half
    //     stroke) like a mass on a spring, at f = c / (2 pi) * sqrt(A / (L V)), Hz. The piston's pull
    //     starts it at intake TDC (the valve opens close to it); a compression back at the valve as it
    //     closes (IVC) packs extra air in. x = f * (time from TDC to IVC). Its first help, x = 1.5,
    //     lands at about 2 bounces per crank revolution, the classic ram-tuned speed; x = 0.5 is far
    //     above any limiter.
    //   exhaust (scavenging): the blowdown pulse at EVO runs down the header at the wave speed c,
    //     reflects at the open end (the merge or collector) as a rarefaction and comes back after
    //     2 L / c: one half of the quarter-wave period 4 L / c. If it reaches the port during the valve
    //     overlap (taken at TDC, 720 deg) it pulls the last burned gas out and the fresh charge in
    //     after it; a pressure wave there instead pushes exhaust back into the cylinder (reversion).
    //     x = (time from EVO to TDC) / (4 L / c).
    // The result multiplies VE (both ShiftCam lobes, so the ECU's choice between them is unchanged).
    static double waveTerm(double x, double gain, double decay) {
        double env = x < 0.5 ? 4.0 * x * x : std::exp(-decay * (x - 0.5));
        return -gain * env * std::cos(2.0 * 3.14159265358979323846 * x);
    }
    // one-entry cache: a physics step asks for the same rpm several times (flow, trapped air, boost)
    mutable double waveAtRpm = -1.0, waveLast = 0.0;
    double waveTuning(double r) const {
        if (r == waveAtRpm) return waveLast;
        waveAtRpm = r;
        double w = std::fabs(r) * (2.0 * 3.14159265358979323846 / 60.0);  // rad/s
        if (w < 1.0) return waveLast = 0.0;
        double d = 0.0;
        if (P.ramGain != 0.0 && P.ramLen > 0.0 && !cyl.empty()) {
            double area = 0.25 * 3.14159265358979323846 * P.ramDia * P.ramDia;
            double vol = cyl[0].vClear + 0.5 * cyl[0].vDisp;
            double f = P.ramSound / (2.0 * 3.14159265358979323846) * std::sqrt(area / (P.ramLen * vol));
            d += waveTerm(f * P.ivc / w, P.ramGain, P.ramDecay);
        }
        if (P.scavGain != 0.0 && P.scavLen > 0.0)
            d += waveTerm((4.0 * 3.14159265358979323846 - P.evo) / w / (4.0 * P.scavLen / P.scavSound), P.scavGain, P.scavDecay);
        return waveLast = d;
    }
    // the smooth curves of the two cam lobes; the wave tuning multiplies whichever runs
    double veCurveHigh(double r) const { return P.veBase + P.vePeak * std::exp(-std::pow((r - P.veRpm) / P.veWidth, 2.0)); }
    double veCurveLow(double r) const {
        return P.veLowBase + P.veLowPeak * std::exp(-std::pow((r - P.veLowRpm) / P.veLowWidth, 2.0));
    }
    // ShiftCam: the partial-load lobe while it breathes better (low and mid rpm), then the race lobe
    bool onLowCam(double r) const { return P.lowCam && veCurveLow(r) > veCurveHigh(r); }
    double volEff(double r) const { return (onLowCam(r) ? veCurveLow(r) : veCurveHigh(r)) * (1.0 + waveTuning(r)); }

    // manifold pressure the ECU aims for at full pedal with the turbo: boostMap, lowered at the top end
    // (boostFallRpm .. boostTopRpm) and capped by the charge (torque) limit, never below atmospheric
    double boostTarget(double r) const {
        const turboParams &T = P.turbo;
        double p = T.boostMap;
        if (T.boostFallRpm > 0.0 && T.boostTopRpm > T.boostFallRpm)
            p += (T.boostTop - T.boostMap) * std::clamp((std::fabs(r) - T.boostFallRpm) / (T.boostTopRpm - T.boostFallRpm), 0.0, 1.0);
        if (T.chargeMax > 0.0) p = std::min(p, T.chargeMax * P.pAtm * tCharge / (P.tIntake * std::max(volEff(std::fabs(r)), 0.1)));
        return std::max(p, P.pAtm);
    }

    double trappedAir(const cylinderState &s, double r) const {
        return volEff(r) * pMan * s.vDisp / (P.rGas * tCharge);
    }

    double sparkAdvance(double r) const {
        if (r < 400.0) return 5.0 * DEGR;
        // at idle the ECU trims spark around a retarded base: slower rpm -> more advance -> more torque
        if (idleActive)
            return std::clamp(P.idleSparkBase + P.idleSparkGain * (P.idleRpm - rpmFilt), -5.0, 25.0) * DEGR;
        double adv = P.sparkLow + (P.sparkHigh - P.sparkLow) * std::clamp((r - P.sparkFrom) / P.sparkSpan, 0.0, 1.0);
        if (P.turbo.on)  // boost and a hot charge: back off for knock
            adv -= P.turbo.knockRetard * std::max(0.0, pMan - P.pAtm) / 1e5 + P.turbo.knockRetardHot * std::max(0.0, tCharge - 320.0);
        return adv * DEGR;
    }

    // cycle-to-cycle variation for the next firing: its coefficient of variation from the manifold
    // pressure (idle vacuum ~0.3 atm -> cycleCovIdle, 0.9 atm and up -> cycleCovFull). Off on the
    // dyno sweep (lockSpeed), whose curves should be the mean engine
    double cycleCov() const {
        if (lockSpeed) return 0.0;
        double light = std::clamp((0.9 - pMan / P.pAtm) / 0.6, 0.0, 1.0);
        return P.cycleCovFull + (P.cycleCovIdle - P.cycleCovFull) * light;
    }
    // about normal (sum of three uniforms: mean 0, variance 1, within +-3), from the seeded generator
    double cycleGauss() {
        double sum = 0.0;
        for (int k = 0; k < 3; k++) {
            cycleRng ^= cycleRng << 13;
            cycleRng ^= cycleRng >> 17;
            cycleRng ^= cycleRng << 5;
            sum += cycleRng / 2147483648.0 - 1.0;
        }
        return sum;
    }

    // residual-gas dilution makes low-load combustion less effective (and is why idle needs ~30 kPa)
    double loadFactor() const { return P.residualMin + (1.0 - P.residualMin) * std::clamp((pMan - 25e3) / 60e3, 0.0, 1.0); }

    // compressible flow function through the throttle, pr = downstream / upstream
    static double flowFunction(double pr) {
        const double g = 1.4;
        pr = std::clamp(pr, 0.0, 1.0);
        if (pr < 0.528) pr = 0.528;  // choked
        return std::sqrt(2.0 * g / (g - 1.0) * (std::pow(pr, 2.0 / g) - std::pow(pr, (g + 1.0) / g)));
    }

    // advance one cylinder's gas state from phiOld to phiNew (dphi is the signed change)
    void advanceCylinder(cylinderState &s, double phiOld, double phiNew, double dphi, double dt, double r) {
        int region = regionOf(phiNew);
        s.heatRate = 0.0;
        if (region != s.region) {
            bool forward = dphi > 0.0;
            if (region == 0) {
                s.p = pMan;
            } else if (region == 1) {
                s.qTot = s.xbDone = 0.0;
                if (forward) {  // intake valve just closed: trap this cycle's charge
                    s.p = pMan;
                    s.mAir = trappedAir(s, r);
                    s.sparked = false;
                } else {        // turned backwards out of the exhaust stroke
                    s.p = pExh;
                    s.mAir = 0.0;
                    s.sparked = true;
                }
            } else if (forward) {   // exhaust valve just opened: remember how hot the gas was
                s.glowEvo = burnGlow(s, phiOld);
            } else {                // backwards out of intake into exhaust
                s.p = pExh;
                s.glowEvo = 0.0;
            }
            s.region = region;
            return;
        }
        if (region == 0) {
            s.p = pMan;
        } else if (region == 1) {
            s.p *= std::pow(volume(s, phiOld) / volume(s, phiNew), P.gamma);
            double phiS = 360.0 * DEGR - sparkAdvance(r);
            // fire once the crank is at or past the spark point. Not "crossed it this step": the
            // advance moves with rpm and (boosted) manifold pressure, and when it moves back past
            // the crank between two steps a crossing test never sees it and the cylinder misfires
            // (sparked is reset when the intake valve closes, so this fires once per cycle)
            if (!s.sparked && dphi > 0.0 && phiNew >= phiS) {
                s.sparked = true;
                s.phiSpark = phiS;
                s.xbDone = 0.0;
                s.burn = P.burn;
                if (ignition && !revCut && !fuelCut && !stalled && !qsCut && s.mAir > 0.0) {
                    s.qTot = tcTrim * P.etaHeat * loadFactor() * s.mAir / P.afr * P.lhv;
                    double cov = cycleCov();
                    if (cov > 0.0) {
                        s.qTot *= std::max(0.5, 1.0 + cov * cycleGauss());
                        s.burn *= std::max(0.5, 1.0 + 1.5 * cov * cycleGauss());
                    }
                }
            }
            if (s.qTot > 0.0 && dphi > 0.0) {
                double u = std::clamp((phiNew - s.phiSpark) / s.burn, 0.0, 1.0);
                double xb = 1.0 - std::exp(-P.wiebeA * std::pow(u, P.wiebeM + 1.0));
                double dq = s.qTot * (xb - s.xbDone);
                if (dq > 0.0) {
                    s.p += (P.gamma - 1.0) / volume(s, phiNew) * dq;
                    s.xbDone = xb;
                    if (dt > 0.0) s.heatRate = dq / dt;
                }
            }
            if (dt > 0.0) s.p = P.pCrankcase + (s.p - P.pCrankcase) * std::exp(-dt / P.leakTau);
        } else {
            s.p = pExh + (s.p - pExh) * std::exp(-std::fabs(dphi) / P.blowdown);
        }
    }

    // 0..1 glow from the burned gas temperature T = pV / (m R): ~800 K dark, ~2000 K full
    double burnGlow(const cylinderState &s, double phi) const {
        if (s.qTot <= 0.0 || s.mAir <= 0.0 || phi < s.phiSpark) return 0.0;
        double t = s.p * volume(s, phi) / (s.mAir * (1.0 + 1.0 / P.afr) * P.rGas);
        return std::clamp((t - 800.0) / 1200.0, 0.0, 1.0);
    }

    double glow(int c) const {
        const cylinderState &s = cyl[c];
        double phi = phase(c);
        if (s.region == 1) return burnGlow(s, phi);
        if (s.region == 2 && s.qTot > 0.0) return s.glowEvo * std::exp(-(phi - P.evo) / (15.0 * DEGR));
        return 0.0;
    }

    // move the crank by dphi without dynamics (paused single-step), keeping the gas states consistent
    void nudge(double dphi) {
        double old = theta;
        theta = wrapCycle(theta + dphi);
        for (int c = 0; c < cylCount(); c++)
            advanceCylinder(cyl[c], wrapCycle(old - lag[c]), phase(c), dphi, 0.0, rpm());
    }

    void step(double dt) {
        double r = rpm();
        time += dt;

        pedal += std::clamp(pedalTarget - pedal, -P.pedalRate * dt, P.pedalRate * dt);
        if (r > P.revLimit) revCut = true;
        if (r < P.revResume) revCut = false;

        // idle air PI: holds idle rpm with the throttle closed once the engine has started
        rpmFilt += (r - rpmFilt) * std::min(1.0, dt / 0.1);
        if (!standalone) drive.control(dt, pedal, rpmFilt, time);
        if (std::fabs(r) < 120.0 && !starter) stalled = true;
        if (starter || std::fabs(r) > 300.0) stalled = false;
        // rev-match blip: once the clutch is mostly open, drive the engine up to the lower gear's rpm
        if (drive.revMatch && drive.shiftPhase != 0 && drive.clutch < 0.4)
            blip = std::clamp((drive.matchRpm + 150.0 - r) / 1000.0, 0.0, 1.0);
        else if (drive.qsPhase < 0) {
            // quickshift down: blip to unload the dogs, then rev to the lower gear's speed. A light crank
            // spins up fast, so the throttle eases off early to land on the target instead of past it
            // (on the averaged acceleration: each firing alone kicks the crank far harder)
            double lead = std::max(0.0, alphaFilt) * 60.0 / (2.0 * 3.14159265358979323846) * 0.02;  // rpm the next 20 ms add
            blip = std::clamp((drive.matchRpm - r - lead) / 800.0 + 0.05, 0.0, 1.0);
        } else
            blip = 0.0;
        double demand = std::max(pedal, blip);
        // overrun fuel cut only while the wheels really drive the engine (not between gears)
        fuelCut = !standalone && drive.coupled() && drive.clutch > 0.9 && demand < 0.02 && rpmFilt > P.fuelCutRpm;
        qsCut = !standalone && drive.qsPhase > 0;
        // launch control holds the revs while the clutch slips (and the plain bite logic when not)
        if (!standalone && drive.boxWorksClutch() && drive.P.launchPedal > 0.0 && drive.launching(r) && r > drive.launchRpm(demand) + 300.0)
            qsCut = true;
        // wheelie control: trim the fuel while the drive force would lift the front wheel
        // (the smoothed drive force is also what the HUD shows)
        if (!standalone) wheelForceFilt += (drive.wheelForce - wheelForceFilt) * std::min(1.0, dt / 0.05);
        if (!drive.wheelieOn()) tcTrim = 1.0;
        else if (!standalone && std::fabs(omega - drive.omegaIn) < 20.0) {  // clutch locked
            double lift = drive.mass() * drive.P.wheelieAccel;
            // proportional: trims down fast when the front would lift, eases back as it settles
            tcTrim = std::clamp(tcTrim + 4.0 * dt * (1.0 - wheelForceFilt / lift), 0.3, 1.0);
        }
        idleActive = ignition && demand < 0.02 && r > 400.0;
        if (idleActive) {
            double err = P.idleRpm - rpmFilt;
            // anti-windup: stop integrating while the valve is pinned at a limit in the same direction
            double unclamped = P.idleBase + P.idleKp * err + P.idleKi * idleInt;
            bool pinned = (unclamped >= P.idleMax && err > 0.0) || (unclamped <= P.idleMin && err < 0.0);
            if (!pinned && std::fabs(err) < P.idleBand) idleInt += err * dt;
            idleArea = std::clamp(P.idleBase + P.idleKp * err + P.idleKi * idleInt, P.idleMin, P.idleMax);
        } else if (!ignition || r <= 400.0) {
            // stopped or cranking: park the valve at its start position
            idleInt = 0.0;
            idleArea = P.idleBase;
        }

        // torques at the current angle
        double inertia = P.inertia, dInertia = 0.0;
        const int n = cylCount();
        gasTorque = 0.0;
        for (int c = 0; c < n; c++) {
            cylinderState &s = cyl[c];
            double x, dx, d2x;
            geom(phase(c), s.r, s.L, x, dx, d2x);
            dxs[c] = dx;
            d2xs[c] = d2x;
            gasTorque += (s.p - P.pCrankcase) * s.area * dx;
            inertia += P.mRecip * dx * dx;
            dInertia += 2.0 * P.mRecip * dx * d2x;
        }
        double fmep = P.fmepA + P.fmepB * std::fabs(r) / 1000.0 + P.fmepC * r * r / 1e6;
        frictionTorque = fmep * vDispTotal / (4.0 * 3.14159265358979323846) * std::tanh(omega / 1.0);
        frictionTorque += chargerTorque;  // the supercharger's drive is a parasitic load like friction
        starterTorque = starter ? P.starterTorque * std::clamp(1.0 - r / P.starterMaxRpm, 0.0, 1.0) : 0.0;
        loadTorque = standalone ? 0.0 : drive.clutchTorqueFor(omega, inertia, dt);  // what the clutch takes from the crank

        // Lagrange with angle-dependent inertia: I(theta) w' + 1/2 I'(theta) w^2 = sum of torques
        alpha = 0.0;
        if (!lockSpeed) {
            alpha = (gasTorque - frictionTorque + starterTorque - loadTorque - 0.5 * dInertia * omega * omega) / inertia;
            omega += alpha * dt;
        }
        alphaFilt += (alpha - alphaFilt) * std::min(1.0, dt / 0.015);

        // shaking: each piston needs m*a along its bore, the block supplies it and feels the reaction.
        // The block also feels the reaction of the torque the pistons put into the crank.
        {
            double fUp = 0.0, recipTorque = 0.0, pitch = 0.0;
            for (int c = 0; c < n; c++) {
                double a = d2xs[c] * omega * omega + dxs[c] * alpha;  // piston acceleration away from TDC
                fUp += P.mRecip * a * bankCos[c];  // vertical part (a V engine's bores lean)
                // counterweight opposite this cylinder's crankpin (pin angle in mesh space = phase + bank)
                double pinAngle = phase(c) + std::atan2(bankSin[c], bankCos[c]);
                fUp -= P.counterweight * P.mRecip * cyl[c].r * omega * omega * std::cos(pinAngle);
                recipTorque += P.mRecip * a * dxs[c];
                pitch += P.mRecip * a * bankCos[c] * cyl[c].z;
            }
            shakeForce = fUp;
            rollMoment = -(gasTorque - recipTorque);
            pitchMoment = pitch;
            double k = std::min(1.0, dt / 0.3);
            double psi = phase(0);  // cylinder 1 crank angle; cos/sin are 360-degree periodic
            h1c += (fUp * std::cos(psi) - h1c) * k;
            h1s += (fUp * std::sin(psi) - h1s) * k;
            h2c += (fUp * std::cos(2.0 * psi) - h2c) * k;
            h2s += (fUp * std::sin(2.0 * psi) - h2s) * k;
            rollMean += (rollMoment - rollMean) * k;
            rollVar += ((rollMoment - rollMean) * (rollMoment - rollMean) - rollVar) * k;
            pitchVar += (pitch * pitch - pitchVar) * k;

            // block on its mounts: vertical and roll spring-damper, semi-implicit Euler
            const double TWO_PI = 2.0 * 3.14159265358979323846;
            double kY = P.blockMass * std::pow(TWO_PI * P.mountFreqY, 2.0);
            double cY = 2.0 * P.mountZeta * std::sqrt(kY * P.blockMass);
            mountVy += (fUp - kY * mountY - cY * mountVy) / P.blockMass * dt;
            mountY += mountVy * dt;
            double kR = P.blockRollInertia * std::pow(TWO_PI * P.mountFreqRoll, 2.0);
            double cR = 2.0 * P.mountZeta * std::sqrt(kR * P.blockRollInertia);
            mountVr += (rollMoment - rollMean - kR * mountRoll - cR * mountVr) / P.blockRollInertia * dt;
            mountRoll += mountVr * dt;
        }
        double dphi = omega * dt;
        double old = theta;
        theta = wrapCycle(theta + dphi);
        for (int c = 0; c < n; c++)
            advanceCylinder(cyl[c], wrapCycle(old - lag[c]), phase(c), dphi, dt, r);

        // intake manifold filling: throttle + idle valve in, cylinders out
        // drive-by-wire: the pedal asks for a manifold pressure (so torque follows the pedal roughly
        // linearly at any rpm) and the throttle opens to whatever area passes that flow at this rpm,
        // inverting the flow model below. Full pedal is always wide open.
        // with the turbo, full pedal asks for the boost target and the throttle is fed from the compressor
        double pUp = P.turbo.on ? pBoost : P.pAtm;
        double pWant = P.mapMin + demand * ((P.turbo.on ? boostTarget(r) : P.pAtm) - P.mapMin);
        double flowWant = volEff(r) * pWant * vDispTotal * std::fabs(omega) / (4.0 * 3.14159265358979323846) / (P.rGas * tCharge);
        double flux = P.throttleCd * pUp / std::sqrt(P.rGas * tCharge) * flowFunction(std::min(pWant / pUp, 0.995));
        double etc = demand > 0.001 ? std::min(P.throttleArea, flowWant / flux) : 0.0;
        throttleOpen = std::max(etc, P.throttleArea * demand * demand * demand) / P.throttleArea;
        double area = P.throttleArea * throttleOpen + idleArea;
        double mIn = P.throttleCd * area * pUp / std::sqrt(P.rGas * tCharge) * flowFunction(pMan / pUp);
        double mOut = volEff(r) * pMan * vDispTotal * std::fabs(omega) / (4.0 * 3.14159265358979323846) / (P.rGas * tCharge);
        pMan = std::clamp(pMan + P.rGas * tCharge / P.vManifold * (mIn - mOut) * dt, 5000.0, pUp);
        throttleFlow = mIn;
        if (P.turbo.on) stepTurbo(dt, mIn, mOut, pWant);
        if (!standalone) drive.integrate(dt, pedal);

        // record cylinder 1 for the p-V diagram, filling any degrees skipped this step
        int idx = std::min(719, static_cast<int>(phase(0) / DEGR));
        float vNow = static_cast<float>(volume(cyl[0], phase(0))), pNow = static_cast<float>(cyl[0].p);
        int gap = pvLast < 0 ? 0 : (idx - pvLast + 720) % 720;
        if (dphi > 0.0 && gap > 0 && gap < 60) {
            for (int d = 1; d <= gap; d++) {
                int j = (pvLast + d) % 720;
                pvV[j] = static_cast<float>(volume(cyl[0], j * DEGR));
                pvP[j] = pNow;
            }
        }
        pvV[idx] = vNow;
        pvP[idx] = pNow;
        pvLast = idx;

        brakeAvg += (gasTorque - frictionTorque - brakeAvg) * std::min(1.0, dt / 0.25);
    }

    // compressor map: pressure rise (Pa) at shaft speed w (rad/s) and flow m (kg/s). Peak at the surge
    // line; falls to the shut-off head at zero flow, rises again for backflow (the wheel acts like a
    // throttle against it) and falls slowly toward choke on the right
    double compressorRise(double w, double m) const {
        const turboParams &T = P.turbo;
        double x = w / (T.rpmRef / 60.0 * 2.0 * 3.14159265358979323846);
        double peak = P.pAtm * (T.prRef - 1.0) * x * x;
        double mPk = std::max(1e-4, T.flowPeakRef * x);
        double phi = m / mPk, shape;
        if (phi < 0.0) shape = T.shutoffHead + 1.5 * phi * phi;
        else if (phi < 1.0) shape = 1.0 - (1.0 - T.shutoffHead) * (1.0 - phi) * (1.0 - phi);
        else shape = 1.0 - T.chokeFall * (phi - 1.0) * (phi - 1.0);
        return peak * shape;
    }

    // the turbo, one step: exhaust manifold filling, shaft power balance, compressor duct and charge
    // pipes (surge), blow-off valve, intercooler, wastegate. mAir: air through the throttle;
    // mCyl: what the cylinders breathe
    void stepTurbo(double dt, double mAir, double mCyl, double pWant) {
        const turboParams &T = P.turbo;
        const double CP = 1005.0, CP_EX = 1150.0, R_EX = 290.0, G_EX = 1.33, K = 0.4 / 1.4;
        bool burning = ignition && !stalled && !fuelCut && !revCut && !qsCut;
        double mExh = mCyl * (burning ? 1.0 + 1.0 / P.afr : 1.0);
        double load = std::clamp((pMan - 30e3) / (T.boostMap - 30e3), 0.0, 1.0);
        double tExh = burning ? T.tExhLight + (T.tExhFull - T.tExhLight) * load : 450.0;
        double mTurb = 0.0;
        if (!T.supercharger) {
            // exhaust manifold: gas in from the cylinders, out through the turbine and the wastegate
            double aT = T.turbineArea, aW = T.wastegateArea * wastegate;
            double mOut = (aT + aW) * pExh / std::sqrt(R_EX * tExh) * flowFunction(P.pAtm / pExh);
            pExh = std::clamp(pExh + R_EX * tExh / T.vExhaust * (mExh - mOut) * dt, P.pAtm, 4.0e5);
            mTurb = mOut * aT / (aT + aW);
        } else {
            turboOmega = std::fabs(omega) * T.driveRatio;  // geared to the crank
        }
        // ram air: the oncoming air is slowed in the intake duct and raises the inlet pressure
        double vAir = std::fabs(drive.v);
        pInlet = P.pAtm + T.ramGain * 0.5 * 1.2 * vAir * vAir;
        double inScale = pInlet / P.pAtm;

        // compressor duct (the air column accelerates on the pressure difference), then the charge
        // pipes fill with the new flow (semi-implicit, so the duct-plenum ringing stays bounded)
        compFlow += (pInlet + inScale * compressorRise(turboOmega, compFlow) - pCharge) / T.ductLA * dt;
        compFlow = std::clamp(compFlow, -0.3, 0.6);
        double pr = std::max(1.0, pCharge / pInlet);
        tComp = P.tIntake * (1.0 + (std::pow(pr, K) - 1.0) / T.etaC);
        // blow-off valve: its diaphragm sees the charge pressure against the manifold's
        double bovWant = T.bov ? std::clamp((pCharge - pMan - T.bovCrack) / T.bovSpan, 0.0, 1.0) : 0.0;
        bovOpen += (bovWant - bovOpen) * std::min(1.0, dt / (bovWant > bovOpen ? 0.015 : 0.06));
        bovFlow = pCharge > P.pAtm ? 0.8 * T.bovArea * bovOpen * pCharge / std::sqrt(P.rGas * tComp) * flowFunction(P.pAtm / pCharge) : 0.0;
        double reliefWant = T.reliefP > 0.0 ? std::clamp((pCharge - T.reliefP) / T.reliefSpan, 0.0, 1.0) : 0.0;
        reliefOpen += (reliefWant - reliefOpen) * std::min(1.0, dt / 0.01);
        reliefFlow = pCharge > P.pAtm ? 0.8 * T.reliefArea * reliefOpen * pCharge / std::sqrt(P.rGas * tComp) * flowFunction(P.pAtm / pCharge) : 0.0;
        airFlowFilt += (mAir - airFlowFilt) * std::min(1.0, dt / 0.005);
        pCharge += 1.4 * P.rGas * tComp / T.vCharge * (compFlow - airFlowFilt - bovFlow - reliefFlow) * dt;
        pCharge = std::clamp(pCharge, 0.5 * P.pAtm, 4.0e5);
        // the throttle sees the charge less the drop across the intercooler core
        pBoost = std::max(0.3 * P.pAtm, pCharge - T.icDrop * airFlowFilt * airFlowFilt);
        // surge: the flow stalls left of the map's peak and reverses; each backflow burst kicks it up
        if (compFlow < 0.0) surge = std::max(surge, std::min(1.0, -compFlow / 0.03));
        surge *= std::exp(-dt / 0.15);

        // intercooler: takes out a share of the compressor's heat (more with the car moving), the core
        // soaking up and giving back heat slowly
        double eff = T.icEffStill + (T.icEffMoving - T.icEffStill) * std::clamp(std::fabs(drive.v) / 30.0, 0.0, 1.0);
        double tWant = tComp - eff * (tComp - P.tIntake);
        tCharge += (tWant - tCharge) * std::min(1.0, dt / T.icTau);

        // shaft: turbine power against the compressor's (it works on the air either way through it)
        // and the bearings'
        turbinePower = T.etaT * mTurb * CP_EX * tExh * (1.0 - std::pow(P.pAtm / pExh, (G_EX - 1.0) / G_EX));
        compressorPower = std::fabs(compFlow) * CP * P.tIntake * (std::pow(pr, K) - 1.0) / T.etaC;
        if (T.supercharger) {
            // the crank pays for it (next step's friction); no turbine, no wastegate
            chargerTorque = compressorPower / T.driveEff / std::max(std::fabs(omega), 50.0) * (omega >= 0.0 ? 1.0 : -1.0);
            return;
        }
        double net = turbinePower - compressorPower - T.bearingLoss * turboOmega * turboOmega;
        turboOmega += net / (T.inertia * std::max(turboOmega, 200.0)) * dt;
        turboOmega = std::clamp(turboOmega, 0.0, T.maxRpm / 60.0 * 2.0 * 3.14159265358979323846);

        // wastegate: opens while there is more boost than the pedal asks for (never aiming below
        // atmospheric), shuts when there is less
        double err = (pBoost - std::max(P.pAtm, pWant)) / 1e5;  // bar over the target
        wgInt = std::clamp(wgInt + T.wgKi * err * dt, 0.0, 1.0);
        wastegate = std::clamp(wgInt + T.wgKp * err, 0.0, 1.0);
    }

    // stroke name for cylinder c
    const char *stroke(int c) const {
        double phi = phase(c) / DEGR;
        if (phi < 180.0) return "INTAKE";
        if (phi < 360.0) return "COMP";
        if (phi < 540.0) return "POWER";
        return "EXHAUST";
    }
};

const double PHYS_DT = 1.0 / 40000.0;

