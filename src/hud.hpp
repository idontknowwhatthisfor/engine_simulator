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
    s += "[ ]: slow-mo  , .: step  U: hud  G: ghost  B: blur  M: sound  K: leveller  O: speakers  D: dyno  X: shake  P: pause  R: reset";
    return s;
}

// the window's state the HUD reports besides the simulation's
struct hudView {
    float fps = 0.f;              // 0: not measured yet
    double timeScale = 1.0;
    bool paused = false;
    const char *sound = "off";    // "stereo", "mono", "muted" or "off"
    bool leveller = false;        // the playback leveller (K) is on
    double levelGainDb = 0.0;     // its boost now
    bool speaker = false;         // the small-speaker voicing (O) is on
    float shakeView = 1.f;
    bool ghost = true, blur = true;
    const exhaustModel *exhaust = nullptr;  // drawn exhaust (nullptr: none)
    bool showTurbo = false;       // the turbo or supercharger is drawn
};

// the stats in two blocks, which U shows one, the other, both or neither of (hudMode): the top is the
// engine and the car (rpm, gear, pedal, boost, torque, clutch, exhaust, the flags), the bottom the
// shaking, the sound and view settings and the firing order with each cylinder's stroke
struct hudParts {
    std::string top, bottom;
};
enum class hudMode { top, bottom, both, none };
hudMode nextHudMode(hudMode m) {
    return m == hudMode::top ? hudMode::bottom : m == hudMode::bottom ? hudMode::both : m == hudMode::both ? hudMode::none : hudMode::top;
}
bool hudShowsTop(hudMode m) { return m == hudMode::top || m == hudMode::both; }
bool hudShowsBottom(hudMode m) { return m == hudMode::bottom || m == hudMode::both; }

hudParts hudText(const engineSim &sim, const engineType &type, const engineModel &eng, const hudView &v) {
    auto f0 = [](double x) { return numToStr(static_cast<float>(x), 0); };
    // fixed-width fields for the numbers that change while it runs (the font is monospaced)
    auto p0 = [](double x, std::size_t w) { return numPad(static_cast<float>(x), 0, w); };
    auto p1 = [](double x, std::size_t w) { return numPad(static_cast<float>(x), 1, w); };
    auto p2 = [](double x, std::size_t w) { return numPad(static_cast<float>(x), 2, w); };
    const drivetrain &dv = sim.drive;
    const controlWords w = controlsOf(dv.P);
    const bool stand = sim.standalone;
    const double kw = sim.brakeAvg * sim.omega / 1000.0;

    // the cylinders in firing order: stroke and pressure
    std::string strokes;
    std::vector<int> order(sim.cyl.size());
    for (std::size_t c = 0; c < order.size(); c++) order[c] = static_cast<int>(c);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return sim.lag[a] < sim.lag[b]; });
    // more than six cylinders (the V10): two lines, so the list stays inside the window
    const std::size_t perLine = order.size() > 6 ? (order.size() + 1) / 2 : order.size();
    for (std::size_t k = 0; k < order.size(); k++) {
        int c = order[k];
        std::string name = sim.stroke(c);
        name.resize(8, ' ');
        strokes += std::string(k % perLine == 0 ? "\n" : "") + " " + std::to_string(c + 1) + " " + name.substr(0, 4) + " " +
                   p1(sim.cyl[c].p / 1e5, 5);
    }
    auto gname = [](int g) { return g == 0 ? std::string("N") : g < 0 ? std::string("R") : std::to_string(g); };
    std::string gearName = gname(dv.gear);
    if (dv.shiftPhase) gearName += ">" + gname(dv.targetGear);
    if (dv.qsPhase != 0) gearName += ">" + gname(dv.targetGear);  // a quickshift (bikes)
    gearName.resize(std::max<std::size_t>(gearName.size(), 3), ' ');  // "2" and "2>3" take the same room
    double slipRpm = (sim.omega - dv.omegaIn) * 60.0 / (2.0 * 3.14159265358979323846);
    std::string flags = std::string(sim.ignition ? "" : "  IGNITION OFF") + (sim.revCut ? "  REV CUT" : "") +
                        (sim.fuelCut ? "  FUEL CUT" : "") + (sim.starter ? "  STARTER" : "") + (dv.brake ? "  BRAKE" : "");
    if (dv.P.wheelieAccel > 0.0) {  // wheelie control fitted
        if (sim.tcTrim < 0.98) flags += "  WHEELIE CTRL";
        if (!dv.wheelieControl) flags += "  wheelie ctrl off";
    }
    std::string camStr = sim.P.lowCam ? (sim.onLowCam(sim.rpm()) ? "   ShiftCam: partial-load lobe" : "   ShiftCam: full lobe") : "";

    std::string s = " FPS " + padLeft(v.fps > 0.f ? f0(v.fps) : std::string("--"), 3) + "   time x" +
                    numToStr(static_cast<float>(v.timeScale), 2) + (v.paused ? "   PAUSED" : "");
    if (stand) s += "\n RPM " + p0(sim.rpm(), 5) + "   " + type.label + ", on a stand" + camStr;
    else
        s += "\n RPM " + p0(sim.rpm(), 5) + (*type.hudTag ? std::string("  ") + type.hudTag : std::string()) + "   gear " + gearName +
             (dv.automatic ? (dv.sport ? " SPORT" : " AUTO") : dv.dctOn() ? " MANUAL DCT" : " MANUAL") + "   " + p0(dv.v * 3.6, 3) + " km/h" + camStr;
    s += std::string("\n ") + w.throttle + " " + p0(sim.pedal * 100.0, 3) + "%  throttle " + p0(sim.throttleOpen * 100.0, 3) +
         "%  manifold " + p0(sim.pMan / 1000.0, 3) + " kPa";
    if (sim.P.turbo.on && !stand) {
        const bool sc = sim.P.turbo.supercharger;
        s += "\n boost " + padLeft(std::string(sim.boostBar() >= 0.0 ? "+" : "") + numToStr(static_cast<float>(sim.boostBar()), 2), 5) +
             (sc ? " bar  plenum " + p0(sim.pCharge / 1000.0, 3) + " kPa  drive " + p0(sim.chargerTorque * sim.omega / 1000.0, 3) + " kW" +
                       (sim.pInlet > sim.P.pAtm + 200.0 ? "  ram " + padLeft("+" + f0(sim.pInlet - sim.P.pAtm), 5) + " Pa" : "")
                 : " bar  wastegate " + p0(sim.wastegate * 100.0, 3) + "%  back pressure " + p0(sim.pExh / 1000.0, 3) + " kPa") +
             "\n charge " + p0(sim.tCharge - 273.15, 3) + " C (compressor " + p0(sim.tComp - 273.15, 3) + " C)  " +
             (sc ? "bypass valve " : "blow-off valve ") + (!sim.P.turbo.bov ? "removed" : sim.bovOpen > 0.05 ? "VENTING" : "shut") +
             (sim.surge > 0.2 ? "  SURGE" : "");
    }
    s += "\n torque " + p1(sim.brakeAvg, 6) + " N m  power " + p1(std::max(kw, 0.0), 5);
    if (stand) s += " kW (" + p0(std::max(kw, 0.0) * 1.341, 3) + " hp)";
    else {
        // with wheelie control the drive force shown is the smoothed one that control works on
        double wheel = dv.P.wheelieAccel > 0.0 ? sim.wheelForceFilt : dv.wheelForce;
        // the clutch: in manual the driver's pedal / lever travel; when the box works it (automatic, a
        // car's manual DCT) how far it is engaged, and who is working it
        std::string clutchStr = dv.boxWorksClutch()
            ? "clutch " + p0(dv.clutch * 100.0, 3) + "% engaged (" + (dv.automatic ? "auto" : "DCT") + ")"
            : std::string(w.clutch) + " " + p0((1.0 - dv.clutch) * 100.0, 3) + "%";
        s += " kW  wheels " + p2(wheel / 1000.0, 5) + " kN" + "\n " + clutchStr +
             "  slip " + p0(slipRpm, 5) + " rpm" + "\n hill " + (dv.grade >= 0 ? "+" : "") +
             f0(dv.grade * 100.0) + "%  mass " + f0(dv.mass()) + " kg" + (dv.cargo > 0 ? std::string(" (") + w.load + ")" : "");
        if (v.exhaust)
            s += "\n exhaust: headers " + padLeft("~" + std::to_string(v.exhaust->headerC()), 5) + " C" +
                 (v.exhaust->flapAt.on ? "  valve " + p0(v.exhaust->flap * 100.0, 3) + "% open" : "") +
                 (v.showTurbo ? (sim.P.turbo.supercharger ? "  supercharger " : "  turbo ") + p0(sim.turboRpm() / 1000.0, 3) + "k rpm" : "");
    }
    // the flags keep their line even when there are none, so the lines under it do not jump as
    // FUEL CUT, REV CUT... come and go
    hudParts out;
    out.top = s + "\n" + flags;
    std::string b = " shake 1st " + p2(sim.shake1() / 1000.0, 5) +
         " kN  2nd " + p2(sim.shake2() / 1000.0, 5) + " kN  roll +-" + p0(sim.rollAmp(), 4) + " N m" +
         "\n sound " + v.sound + (v.leveller ? " lvl +" + f0(v.levelGainDb) + " dB" : " lvl off") + (v.speaker ? " spk" : "") + "  shake view x" + f0(v.shakeView) + "  block " + (v.ghost ? "ghost" : "hidden") + "  blur " +
         (v.blur ? "on" : "off");
    if (type.hudFiringLine) b += "\n firing (" + hudFiring(type, eng) + "):";
    out.bottom = b + strokes;
    return out;
}
