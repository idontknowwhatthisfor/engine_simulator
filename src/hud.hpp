// what the window's text says: the help line and the HUD, built from what the engine has (a bike or a
// car drivetrain or a stand, wheelie control, a turbo or a supercharger and its valve, an exhaust)
// rather than from which engine it is, so a new engine with a new combination needs no edit here
#pragma once
#include "engine_type.hpp"

// the controls' words: a rider's or a driver's. The car's line has fewer keys, so it spells more out
struct controlWords {
    const char *throttle;  // what the pedal / twist grip is called on the HUD
    const char *clutch;
    const char *load;      // what L adds
    const char *shift;     // the E/Q keys on the help line
    const char *hill;      // the H/J keys
};
controlWords controlsOf(const drivetrainParams &P) {
    if (P.bike) return {"twist grip", "clutch lever", "passenger", "E/Q: shift", "H/J: hill"};
    return {"pedal", "clutch pedal", "cargo", "E/Q: shift up/down (Q at a stop: R)", "H/J: hill up/down"};
}

// the help line at the bottom of the window: the keys main() handles for this simulation
std::string helpText(const engineSim &sim) {
    const drivetrainParams &D = sim.drive.P;
    const controlWords w = controlsOf(D);
    const bool drive = !sim.standalone;  // on a stand the drivetrain keys do nothing
    std::string s = "drag: orbit  scroll: zoom  SPACE: starter  W/1-0: throttle";
    if (drive) s += "  S: brake";
    s += "  I: ignition\n";
    if (drive) {
        // Z: sport in automatic; in a car's manual, the NORMAL / DCT clutch mode
        s += std::string("T: auto/manual  ") + (!sim.drive.automatic && !D.bike ? "Z: DCT  " : "Z: sport  ") + w.shift;
        if (D.bike) s += "  N: neutral";                    // the bikes' neutral finder
        s += "  C: clutch";
        if (D.wheelieAccel > 0.0) s += "  V: wheelie ctrl";
        s += std::string("  ") + w.hill + "  L: " + w.load;
        if (sim.P.turbo.on) s += sim.P.turbo.supercharger ? "  F: bypass" : "  F: BOV";  // the boost valve
        s += "\n";
    }
    s += "[ ]: slow-mo  , .: step  G: ghost  B: blur  M: mute  D: dyno  X: shake  P: pause  R: reset";
    return s;
}

// the window's state the HUD reports besides the simulation's
struct hudView {
    float fps = 0.f;              // 0: not measured yet
    double timeScale = 1.0;
    bool paused = false;
    const char *sound = "off";    // "on", "muted" or "off"
    float shakeView = 1.f;
    bool ghost = true, blur = true;
    const exhaustModel *exhaust = nullptr;  // drawn exhaust (nullptr: none)
    bool showTurbo = false;       // the turbo or supercharger is drawn
};

std::string hudText(const engineSim &sim, const engineType &type, const engineModel &eng, const hudView &v) {
    auto f0 = [](double x) { return numToStr(static_cast<float>(x), 0); };
    const drivetrain &dv = sim.drive;
    const controlWords w = controlsOf(dv.P);
    const bool stand = sim.standalone;
    const double kw = sim.brakeAvg * sim.omega / 1000.0;

    // the cylinders in firing order: stroke and pressure
    std::string strokes;
    std::vector<int> order(sim.cyl.size());
    for (std::size_t c = 0; c < order.size(); c++) order[c] = static_cast<int>(c);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return sim.lag[a] < sim.lag[b]; });
    for (int c : order) {
        std::string name = sim.stroke(c);
        name.resize(8, ' ');
        strokes += std::string(c == 0 ? "\n" : "") + " " + std::to_string(c + 1) + " " + name.substr(0, 4) + " " +
                   numToStr(static_cast<float>(sim.cyl[c].p / 1e5), 1);
    }
    auto gname = [](int g) { return g == 0 ? std::string("N") : g < 0 ? std::string("R") : std::to_string(g); };
    std::string gearName = gname(dv.gear);
    if (dv.shiftPhase) gearName += ">" + gname(dv.targetGear);
    if (dv.qsPhase != 0) gearName += ">" + gname(dv.targetGear);  // a quickshift (bikes)
    double slipRpm = (sim.omega - dv.omegaIn) * 60.0 / (2.0 * 3.14159265358979323846);
    std::string flags = std::string(sim.ignition ? "" : "  IGNITION OFF") + (sim.revCut ? "  REV CUT" : "") +
                        (sim.fuelCut ? "  FUEL CUT" : "") + (sim.starter ? "  STARTER" : "") + (dv.brake ? "  BRAKE" : "");
    if (dv.P.wheelieAccel > 0.0) {  // wheelie control fitted
        if (sim.tcTrim < 0.98) flags += "  WHEELIE CTRL";
        if (!dv.wheelieControl) flags += "  wheelie ctrl off";
    }
    std::string camStr = sim.P.lowCam ? (sim.onLowCam(sim.rpm()) ? "   ShiftCam: partial-load lobe" : "   ShiftCam: full lobe") : "";

    std::string s = " FPS " + (v.fps > 0.f ? f0(v.fps) : std::string("--")) + "   time x" +
                    numToStr(static_cast<float>(v.timeScale), 2) + (v.paused ? "   PAUSED" : "");
    if (stand) s += "\n RPM " + f0(sim.rpm()) + "   " + type.label + ", on a stand" + camStr;
    else
        s += "\n RPM " + f0(sim.rpm()) + (*type.hudTag ? std::string("  ") + type.hudTag : std::string()) + "   gear " + gearName +
             (dv.automatic ? (dv.sport ? " SPORT" : " AUTO") : dv.dctOn() ? " MANUAL DCT" : " MANUAL") + "   " + f0(dv.v * 3.6) + " km/h" + camStr;
    s += std::string("\n ") + w.throttle + " " + f0(sim.pedal * 100.0) + "%  throttle " + f0(sim.throttleOpen * 100.0) +
         "%  manifold " + f0(sim.pMan / 1000.0) + " kPa";
    if (sim.P.turbo.on && !stand) {
        const bool sc = sim.P.turbo.supercharger;
        s += "\n boost " + std::string(sim.boostBar() >= 0.0 ? "+" : "") + numToStr(static_cast<float>(sim.boostBar()), 2) +
             (sc ? " bar  plenum " + f0(sim.pCharge / 1000.0) + " kPa  drive " + f0(sim.chargerTorque * sim.omega / 1000.0) + " kW" +
                       (sim.pInlet > sim.P.pAtm + 200.0 ? "  ram +" + f0(sim.pInlet - sim.P.pAtm) + " Pa" : "")
                 : " bar  wastegate " + f0(sim.wastegate * 100.0) + "%  back pressure " + f0(sim.pExh / 1000.0) + " kPa") +
             "\n charge " + f0(sim.tCharge - 273.15) + " C (compressor " + f0(sim.tComp - 273.15) + " C)  " +
             (sc ? "bypass valve " : "blow-off valve ") + (!sim.P.turbo.bov ? "removed" : sim.bovOpen > 0.05 ? "VENTING" : "shut") +
             (sim.surge > 0.2 ? "  SURGE" : "");
    }
    s += "\n torque " + numToStr(static_cast<float>(sim.brakeAvg), 1) + " N m  power " + numToStr(static_cast<float>(std::max(kw, 0.0)), 1);
    if (stand) s += " kW (" + f0(std::max(kw, 0.0) * 1.341) + " hp)";
    else {
        // with wheelie control the drive force shown is the smoothed one that control works on
        double wheel = dv.P.wheelieAccel > 0.0 ? sim.wheelForceFilt : dv.wheelForce;
        // the clutch: in manual the driver's pedal / lever travel; when the box works it (automatic, a
        // car's manual DCT) how far it is engaged, and who is working it
        std::string clutchStr = dv.boxWorksClutch()
            ? "clutch " + f0(dv.clutch * 100.0) + "% engaged (" + (dv.automatic ? "auto" : "DCT") + ")"
            : std::string(w.clutch) + " " + f0((1.0 - dv.clutch) * 100.0) + "%";
        s += " kW  wheels " + numToStr(static_cast<float>(wheel / 1000.0), 2) + " kN" + "\n " + clutchStr +
             "  slip " + f0(slipRpm) + " rpm" + "\n hill " + (dv.grade >= 0 ? "+" : "") +
             f0(dv.grade * 100.0) + "%  mass " + f0(dv.mass()) + " kg" + (dv.cargo > 0 ? std::string(" (") + w.load + ")" : "");
        if (v.exhaust)
            s += "\n exhaust: headers ~" + std::to_string(v.exhaust->headerC()) + " C" +
                 (v.exhaust->flapAt.on ? "  valve " + f0(v.exhaust->flap * 100.0) + "% open" : "") +
                 (v.showTurbo ? (sim.P.turbo.supercharger ? "  supercharger " : "  turbo ") + f0(sim.turboRpm() / 1000.0) + "k rpm" : "");
    }
    s += (flags.empty() ? "" : "\n" + flags) + "\n shake 1st " + numToStr(static_cast<float>(sim.shake1() / 1000.0), 2) +
         " kN  2nd " + numToStr(static_cast<float>(sim.shake2() / 1000.0), 2) + " kN  roll +-" + f0(sim.rollAmp()) + " N m" +
         "\n sound " + v.sound + "  shake view x" + f0(v.shakeView) + "  block " + (v.ghost ? "ghost" : "hidden") + "  blur " +
         (v.blur ? "on" : "off");
    if (type.hudFiringLine) s += "\n firing (" + hudFiring(type, eng) + "):";
    return s + strokes;
}
