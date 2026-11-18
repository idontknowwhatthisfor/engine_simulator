// engine simulator: the entry point. Pick the engine with --engine-type (inline4 | v4 | s1000rr);
// see README.md. Shared code is in src/, each engine's own files are in engines/<type>/.
// model: "Inline-4 Engine" by rafaelaraujo55, CC-BY-4.0 (engines/inline4_fit/license.txt)
#include "src/common.hpp"
#include "src/model.hpp"
#include "src/render.hpp"
#include "src/physics.hpp"
#include "src/engine_type.hpp"
#include "src/engines.hpp"
#include "engines/inline4_fit/inline4_fit.hpp"
#include "engines/ducati_v4/ducati_v4.hpp"
#include "engines/bmw_s1000rr/bmw_s1000rr.hpp"
#include "engines/kawasaki_h2/kawasaki_h2.hpp"
#include "src/sound.hpp"
#include "src/wav.hpp"
#include "src/test_bike.hpp"
#include "src/bike_gearbox.hpp"
#include "src/turbo.hpp"
#include "src/dyno.hpp"
#include "src/dump_params.hpp"

// ---------------------------------------------------------------------------------------------
// command line, for scripted test runs
// ---------------------------------------------------------------------------------------------
struct options {
    std::string screenshot;  // save a PNG after `frames` frames and quit
    int frames = 5;
    float yaw = 0.6f, pitch = 0.45f, distScale = 1.f;
    int ghost = 0;
    float crankDeg = -1.f;   // cylinder 1 crank angle from TDC; < 0 keeps the model's rest pose
    float simTime = 0.f;     // run the physics this long (headless) before the first frame
    bool start = false;      // hold the starter for the first second of --simtime
    float throttle = 0.f;    // latched throttle 0..1
    float stopAt = -1.f;     // after --simtime, run on until cylinder 1 reaches this cycle angle (0..720), then pause
    bool blur = true;
    bool hud = true;         // --nohud for pixel regression shots
    bool sound = true;
    bool engineOnly = false; // --engine-only: no gearbox, original framing (pixel baselines)
    std::string engineType = "inline4";  // --engine-type: inline4 | v4 | s1000rr (see src/engines.hpp)
    bool wheelie = true;     // --no-wheelie: a bike starts with wheelie control off
    float grade = 0.f;       // road grade in percent
    bool reverse = false;    // start in reverse (automatic)
    bool sport = false;      // automatic sport mode
    bool turbo = false;      // --turbo yes: a turbocharger in the exhaust (inline-4)
    bool bov = true;         // --bov no: the turbo without its blow-off valve (it surges on lift-off)
    bool tuned = false;      // --tuned yes: the engine's tuned version (h2)
    std::string wav;         // --wav out.wav: render a scripted run to a file and quit
    bool selftest = false, phystest = false;
    bool dumpParams = false; // --dump-params (hidden): print every parameter and quit (scripts/regress.sh)
    std::string cmpA, cmpB;
};

options parseArgs(int argc, char **argv) {
    options o;
    for (int k = 1; k < argc; k++) {
        std::string a = argv[k];
        auto next = [&]() -> std::string {
            if (k + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++k];
        };
        if (a == "--screenshot") o.screenshot = next();
        else if (a == "--frames") o.frames = std::stoi(next());
        else if (a == "--yaw") o.yaw = std::stof(next());
        else if (a == "--pitch") o.pitch = std::stof(next());
        else if (a == "--zoom") o.distScale = std::stof(next());
        else if (a == "--ghost") o.ghost = std::stoi(next());
        else if (a == "--crank") o.crankDeg = std::stof(next());
        else if (a == "--simtime") o.simTime = std::stof(next());
        else if (a == "--start") o.start = true;
        else if (a == "--throttle") o.throttle = std::clamp(std::stof(next()), 0.f, 1.f);
        else if (a == "--stopat") o.stopAt = std::stof(next());
        else if (a == "--noblur") o.blur = false;
        else if (a == "--nohud") o.hud = false;
        else if (a == "--mute") o.sound = false;
        else if (a == "--engine-only") o.engineOnly = true;
        else if (a == "--engine-type") o.engineType = next();
        else if (a.rfind("--engine-type=", 0) == 0) o.engineType = a.substr(14);
        else if (a == "--v4") o.engineType = "v4";            // older spellings
        else if (a == "--s1000rr") o.engineType = "s1000rr";
        else if (a == "--no-wheelie") o.wheelie = false;
        else if (a == "--grade") o.grade = std::stof(next());
        else if (a == "--reverse") o.reverse = true;
        else if (a == "--sport") o.sport = true;
        else if (a == "--turbo" || a.rfind("--turbo=", 0) == 0) {
            std::string v = a == "--turbo" ? next() : a.substr(8);
            if (v == "yes") o.turbo = true;
            else if (v == "no") o.turbo = false;
            else throw std::runtime_error("--turbo takes yes or no, not \"" + v + "\"");
        }
        else if (a == "--tuned" || a.rfind("--tuned=", 0) == 0) {
            std::string v = a == "--tuned" ? next() : a.substr(8);
            if (v == "yes") o.tuned = true;
            else if (v == "no") o.tuned = false;
            else throw std::runtime_error("--tuned takes yes or no, not \"" + v + "\"");
        }
        else if (a == "--bov" || a.rfind("--bov=", 0) == 0) {
            std::string v = a == "--bov" ? next() : a.substr(6);
            if (v == "yes") o.bov = true;
            else if (v == "no") o.bov = false;
            else throw std::runtime_error("--bov takes yes or no, not \"" + v + "\"");
        }
        else if (a == "--wav") o.wav = next();
        else if (a == "--selftest") o.selftest = true;
        else if (a == "--phystest") o.phystest = true;
        else if (a == "--dump-params") o.dumpParams = true;
        else if (a == "--compare") {
            o.cmpA = next();
            o.cmpB = next();
        }
        else throw std::runtime_error("unknown option " + a);
    }
    return o;
}

void listEngineTypes(std::FILE *out) {
    std::fprintf(out, "engine types (--engine-type):\n");
    for (const engineType *t : engineTypes())
        std::fprintf(out, "  %-9s %s\n            also: %s\n", t->id, t->summary, t->aliases);
}

int main(int argc, char **argv) {
    options opt;
    try {
        opt = parseArgs(argc, argv);
    } catch (const std::exception &ex) {
        std::fprintf(stderr, "%s\nusage: engine_sim [--engine-type inline4|v4|s1000rr] [--screenshot out.png] [--frames n] [--yaw r] [--pitch r]\n"
                             "                  [--zoom s] [--ghost 0|1] [--crank deg] [--simtime s] [--start] [--throttle 0..1] [--stopat deg]\n"
                             "                  [--noblur] [--nohud] [--mute] [--engine-only] [--grade pct] [--reverse] [--sport] [--no-wheelie]\n"
                             "                  [--turbo yes|no] [--bov yes|no] [--tuned yes|no]\n"
                             "       engine_sim [--engine-type t] --selftest | --phystest | --wav out.wav\n"
                             "       engine_sim --compare a.png b.png\n"
                             "       engine_sim --engine-type list\n", ex.what());
        return 1;
    }

    if (!opt.cmpA.empty()) return compareImages(opt.cmpA, opt.cmpB);
    if (opt.engineType == "list") {
        listEngineTypes(stdout);
        return 0;
    }
    const engineType *typePtr = findEngineType(opt.engineType);
    if (!typePtr) {
        std::fprintf(stderr, "unknown engine type \"%s\"\n", opt.engineType.c_str());
        listEngineTypes(stderr);
        return 1;
    }
    if (opt.tuned) {
        if (!typePtr->tuned) {
            std::fprintf(stderr, "--tuned yes: the %s has no tuned version (only h2 has one)\n", typePtr->name);
            return 1;
        }
        typePtr = &typePtr->tuned();
    }
    const engineType &type = *typePtr;
    if (opt.turbo && !type.exhaustTurbo) {
        std::fprintf(stderr, type.supercharged ? "--turbo yes: the %s is already supercharged\n"
                                               : "--turbo yes: the %s has no turbo layout (only inline4 has one)\n", type.name);
        return 1;
    }

    engineModel eng;
    try {
        eng = loadEngine(type.meshFile);
    } catch (const std::exception &ex) {
        std::fprintf(stderr, "%s (run: %s, or make)\n", ex.what(), type.buildHint);
        return 1;
    }
    if (opt.dumpParams) return dumpParams(eng, type, opt.turbo, opt.bov);
    if (opt.selftest) return selfTest(eng, type.name);
    if (opt.phystest) return opt.turbo ? type.physTestTurbo(eng, type) : type.physTest(eng, type);
    if (!opt.wav.empty()) return renderWav(eng, type, opt.wav, opt.turbo, opt.bov);

    // engine state; theta of cylinder 1 starts at the model's rest pose unless --crank says otherwise
    const double THETA0 = opt.crankDeg < 0.f ? eng.cyls[0].theta0 : opt.crankDeg * DEGR;
    engineSim sim;
    setupSim(sim, eng, type, THETA0, type.bike && opt.engineOnly, opt.turbo);
    sim.P.turbo.bov = opt.bov;
    // the full-throttle dyno curve (D), measured in the background on a copy of the engine
    dynoSweep dyno;
    dyno.start(sim);
    if (!opt.screenshot.empty() && dyno.worker.joinable()) dyno.worker.join();  // scripted shots show the whole curve
    double latchedThrottle = opt.throttle;
    sim.drive.wheelieControl = opt.wheelie;
    sim.drive.grade = opt.grade / 100.0;
    if (opt.reverse) sim.drive.gear = sim.drive.targetGear = -1;
    sim.drive.sport = opt.sport;
    if (opt.simTime > 0.f) {
        sim.pedalTarget = latchedThrottle;
        for (double t = 0.0; t < opt.simTime; t += PHYS_DT) {
            sim.starter = opt.start && t < 1.0;
            sim.step(PHYS_DT);
        }
        sim.starter = false;
    }
    if (opt.stopAt >= 0.f) {
        // run on (at most 2 s) until cylinder 1 is within 1 degree past the requested cycle angle
        double target = opt.stopAt * DEGR;
        for (double t = 0.0; t < 2.0; t += PHYS_DT) {
            if (wrapCycle(sim.theta - target) < 1.0 * DEGR) break;
            sim.step(PHYS_DT);
        }
    }

    // piston crown tops in mesh space, for the combustion effects
    std::array<float, 4> crownY;
    crownY.fill(1e9f);
    for (const group &g : eng.groups)
        for (std::size_t v = 0; v < g.verts.size(); v += 6)
            if (g.kind == PISTON) crownY[g.cyl] = std::min(crownY[g.cyl], g.verts[v + 1]);

    sf::Font font;
    if (!font.openFromFile("SpaceMono-Regular.ttf")) return 1;

    sf::ContextSettings ctx;
    ctx.depthBits = 24;
    ctx.stencilBits = 8;
    ctx.antiAliasingLevel = 4;
    ctx.majorVersion = 2;
    ctx.minorVersion = 1;
    sf::RenderWindow window(sf::VideoMode({static_cast<unsigned int>(WIDTH), static_cast<unsigned int>(HEIGHT)}),
                            type.windowTitle, sf::Style::Default, sf::State::Windowed, ctx);
    window.setFramerateLimit(144);
    if (!window.setActive(true)) return 1;

    // make room for the gearbox behind the flywheel (world z = -mesh z, world y = -mesh y)
    if (!opt.engineOnly && !type.bike) {
        eng.lo.z = std::min(eng.lo.z, -gearboxModel::rearZ() - 5.f);
        eng.lo.y = std::min(eng.lo.y, -(eng.crankAxis.y + gearboxModel::D + 84.f) - 5.f);
    }
    bikeGearboxModel bbox;
    if (!opt.engineOnly && type.bike) {
        bbox.configure(type, sim.drive.P);
        bbox.layout();
        if (bbox.side > 0.f) eng.hi.x = std::max(eng.hi.x, bbox.maxX() + 5.f);
        else eng.lo.x = std::min(eng.lo.x, bbox.maxX() - 5.f);
        eng.lo.y = std::min(eng.lo.y, -bbox.maxY() - 5.f);
        eng.lo.z = std::min(eng.lo.z, -bbox.maxZ() - 5.f);
    }
    // the exhaust runs from the ports to the tailpipe, outside the engine: make room for it too
    exhaustModel exhaust;
    turboModel turbo;
    bool showExhaust = !opt.engineOnly && type.exhaust;
    bool showTurbo = showExhaust && (opt.turbo || type.supercharged);  // the turbo, or the supercharger
    if (showExhaust) {
        exhaustLayout layout;
        try {
            layout = opt.turbo ? type.exhaustTurbo(eng) : type.exhaust(eng);
            exhaust.build(layout);
        } catch (const std::exception &ex) {
            std::fprintf(stderr, "%s\n", ex.what());
            return 1;
        }
        exhaust.growBounds(eng);
        if (showTurbo) {
            try {
                turbo.load(type.turboMeshFile, layout.turbo);
            } catch (const std::exception &ex) {
                std::fprintf(stderr, "%s (run: %s, or make)\n", ex.what(), type.turboBuildHint);
                return 1;
            }
            turbo.growBounds(eng);
        }
    }
    renderer ren;
    // the road runs along the vehicle: the car's crank points along the car (front at world +z), the
    // bikes' crank lies across the bike, so they face along x (the gearbox sits behind the crank)
    ren.travelX = type.bike;
    ren.travelSign = type.bike ? -type.gearboxSide : 1.f;
    try {
        ren.init(eng);
    } catch (const std::exception &ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        return 1;
    }

    // the sphere sim's box half-size is 300 with the camera at 1100; scale that to the engine
    vec3 centre = (eng.lo + eng.hi) * 0.5f;
    vec3 ext = (eng.hi - eng.lo) * 0.5f;
    float half = std::max({ext.x, ext.y, ext.z});
    const float DIST0 = half * 1100.f / 300.f;
    const camera CAM0{opt.yaw, opt.pitch, DIST0 * opt.distScale};
    camera cam = CAM0;

    gearboxModel gbox;
    if (!opt.engineOnly) {
        if (type.bike) {
            bbox.build(sim.drive.P, ren);
        } else {
            gbox.build(eng, sim.drive.P, ren);
        }
    }
    if (showExhaust) {
        exhaust.upload(ren);
        exhaust.update(sim);  // after a --simtime run the pipes start at their settled heat
        if (showTurbo) {
            turbo.upload(ren);
            turbo.update(sim);
        }
    }
    std::vector<drawItem> extras;

    // sound: synthesised per physics step, streamed on SFML's audio thread (off for scripted screenshots)
    std::unique_ptr<engineAudio> audio;
    engineSound snd(1.0 / PHYS_DT, sim.P.sound);
    if (opt.sound && opt.screenshot.empty()) {
        audio = std::make_unique<engineAudio>();
        audio->play();
    }
    bool showDyno = true;
    const float SHAKE_VIEWS[] = {1.f, 20.f, 100.f};
    int shakeIdx = 0;

    const double TIME_SCALES[] = {1.0, 0.5, 0.25, 0.1, 0.05, 0.02, 0.01};
    const int NUM_SCALES = 7;
    int scaleIdx = 0;
    double physAccum = 0.0;

    bool dragging = false;
    bool paused = opt.stopAt >= 0.f;
    bool blur = opt.blur;
    int ghost = opt.ghost;
    sf::Vector2i lastMouse;
    sf::Vector2u winSize = window.getSize();

    float fps = 0.f, dt;
    sf::Clock clock;
    sf::Clock clock2, clock3;
    int fpsFrames = 0;
    float prevDelta = static_cast<float>(sim.theta - eng.cyls[0].theta0);
    frameEffects fx;
    std::string textStr;
    sf::Text text(font, textStr, 16);
    sf::Text help(font, type.bike && opt.engineOnly
                            ? "drag: orbit  scroll: zoom  SPACE: starter  W/1-0: throttle  I: ignition\n"
                              "[ ]: slow-mo  , .: step  G: ghost  B: blur  M: mute  D: dyno  X: shake  P: pause  R: reset"
                            : type.bike
                            ? "drag: orbit  scroll: zoom  SPACE: starter  W/1-0: throttle  S: brake  I: ignition\n"
                              "T: auto/manual  Z: sport  E/Q: shift  N: neutral  C: clutch  V: wheelie ctrl  H/J: hill  L: passenger" +
                              std::string(type.supercharged ? "  F: bypass" : "") + "\n"
                              "[ ]: slow-mo  , .: step  G: ghost  B: blur  M: mute  D: dyno  X: shake  P: pause  R: reset"
                               : "drag: orbit  scroll: zoom  SPACE: starter  W/1-0: throttle  S: brake  I: ignition\n"
                                 "T: auto/manual  Z: sport  E/Q: shift up/down (Q at a stop: R)  C: clutch  H/J: hill up/down  L: cargo" +
                                 std::string(opt.turbo ? "  F: BOV" : "") + "\n"
                                 "[ ]: slow-mo  , .: step  G: ghost  B: blur  M: mute  D: dyno  X: shake  P: pause  R: reset", 14);
    help.setFillColor(sf::Color(150, 150, 160));
    sf::Text credit(font, std::string(type.credit) + (showTurbo && opt.turbo ? "\nturbo: \"Car Turbocharger\" by ml-design, CC-BY-4.0" : ""), 12);
    credit.setFillColor(sf::Color(110, 110, 120));

    std::vector<mat4> modelOf;
    bool firstFrame = true;
    int frame = 0;

    // keys held down, tracked from window events: sf::Keyboard::isKeyPressed reads the hardware
    // directly, which macOS blocks unless the terminal has Input Monitoring permission
    std::array<bool, sf::Keyboard::KeyCount> held{};
    auto isHeld = [&](sf::Keyboard::Key k) {
        auto i = static_cast<unsigned>(k);  // Unknown (-1) wraps past KeyCount
        return i < sf::Keyboard::KeyCount && held[i];
    };

    while (window.isOpen()) {
        while (auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
            else if (event->is<sf::Event::FocusLost>())
                held.fill(false);
            else if (auto *e = event->getIf<sf::Event::KeyReleased>()) {
                auto i = static_cast<unsigned>(e->code);
                if (i < sf::Keyboard::KeyCount) held[i] = false;
            }
            else if (auto *e = event->getIf<sf::Event::Resized>()) {
                winSize = e->size;
                glViewport(0, 0, static_cast<GLsizei>(winSize.x), static_cast<GLsizei>(winSize.y));
                window.setView(sf::View(sf::FloatRect({0.f, 0.f}, {float(winSize.x), float(winSize.y)})));
            } else if (auto *e = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (e->button == sf::Mouse::Button::Left) {
                    dragging = true;
                    lastMouse = e->position;
                }
            } else if (auto *e = event->getIf<sf::Event::MouseButtonReleased>()) {
                if (e->button == sf::Mouse::Button::Left) dragging = false;
            } else if (auto *e = event->getIf<sf::Event::MouseMoved>()) {
                if (dragging) {
                    sf::Vector2i delta = e->position - lastMouse;
                    lastMouse = e->position;
                    cam.yaw += delta.x * 0.008f;  // reversed from the sphere sim on request
                    cam.pitch = std::clamp(cam.pitch + delta.y * 0.008f, -1.4f, 1.4f);
                }
            } else if (auto *e = event->getIf<sf::Event::MouseWheelScrolled>()) {
                // macOS sends a horizontal event (often delta 0) alongside each vertical one; trackpads give
                // fractional deltas, so zoom by the amount scrolled
                if (e->wheel == sf::Mouse::Wheel::Vertical && e->delta != 0.f)
                    cam.dist = std::clamp(cam.dist * std::pow(0.9f, std::clamp(e->delta, -3.f, 3.f)), half * 2.2f, half * 8.f);
            } else if (auto *e = event->getIf<sf::Event::KeyPressed>()) {
                using K = sf::Keyboard::Key;
                {
                    auto i = static_cast<unsigned>(e->code);
                    if (i < sf::Keyboard::KeyCount) held[i] = true;
                }
                if (e->code == K::Escape) window.close();
                if (e->code == K::G) ghost = (ghost + 1) % 2;
                if (e->code == K::B) blur = !blur;
                if (e->code == K::F && sim.P.turbo.on) {
                    sim.P.turbo.bov = !sim.P.turbo.bov;
                    const char *valve = sim.P.turbo.supercharger ? "bypass valve" : "blow-off valve";
                    sim.drive.message = std::string(valve) + (sim.P.turbo.bov ? " fitted" : " removed: lift off on boost to hear it surge");
                    sim.drive.messageTime = sim.time;
                }
                if (e->code == K::M && audio) audio->muted = !audio->muted.load();
                if (e->code == K::D) showDyno = !showDyno;
                if (e->code == K::X) shakeIdx = (shakeIdx + 1) % 3;
                if (e->code == K::P) paused = !paused;
                if (e->code == K::R) {
                    cam = CAM0;
                    sim.reset(THETA0);
                    sim.drive.grade = sim.drive.cargo = 0.0;
                    latchedThrottle = 0.0;
                    sim.ignition = true;
                    scaleIdx = 0;
                }
                if (e->code == K::I) sim.ignition = !sim.ignition;
                if (sim.standalone) {
                    // no gearbox on the V4 stand: the drivetrain keys do nothing
                } else {
                if (e->code == K::T) sim.drive.setAutomatic(!sim.drive.automatic, sim.rpm(), sim.time);
                if (e->code == K::Z) sim.drive.setSport(!sim.drive.sport, sim.time);
                if (e->code == K::E) sim.drive.requestShift(+1, sim.time);
                if (e->code == K::Q) sim.drive.requestShift(-1, sim.time);
                if (e->code == K::H) sim.drive.grade = std::min(sim.drive.grade + 0.02, 0.20);
                if (e->code == K::J) sim.drive.grade = std::max(sim.drive.grade - 0.02, -0.10);
                if (e->code == K::N && sim.drive.P.bike) sim.drive.bikeShift(0, sim.time, true);
                if (e->code == K::V && sim.drive.P.wheelieAccel > 0.0) {
                    sim.drive.wheelieControl = !sim.drive.wheelieControl;
                    sim.drive.say(sim.drive.wheelieControl ? "wheelie control ON" : "wheelie control OFF", sim.time);
                }
                if (e->code == K::L) {  // cargo / trailer; a passenger on the bike
                    double c = sim.drive.cargo;
                    if (sim.drive.P.bike) sim.drive.cargo = c < 1.0 ? 80.0 : 0.0;
                    else sim.drive.cargo = c < 1.0 ? 500.0 : (c < 600.0 ? 1500.0 : 0.0);
                }
                }
                if (e->code == K::LBracket) scaleIdx = std::min(scaleIdx + 1, NUM_SCALES - 1);
                if (e->code == K::RBracket) scaleIdx = std::max(scaleIdx - 1, 0);
                if (paused && e->code == K::Comma) sim.nudge(-DEGR);
                if (paused && e->code == K::Period) sim.nudge(DEGR);
                // 1..9 -> 10..90 %, 0 -> 100 %; pressing the same key again closes the throttle
                for (int d = 0; d <= 9; d++) {
                    if (e->code == static_cast<K>(static_cast<int>(K::Num0) + d)) {
                        double v = d == 0 ? 1.0 : d / 10.0;
                        latchedThrottle = std::fabs(latchedThrottle - v) < 1e-6 ? 0.0 : v;
                    }
                }
            }
        }

        dt = std::min(clock.restart().asSeconds(), 1.f / 30.f);

        // held controls, only while the window has focus
        bool focus = window.hasFocus();
        using K = sf::Keyboard::Key;
        sim.starter = focus && isHeld(K::Space);
        bool floorIt = focus && (isHeld(K::W) || isHeld(K::Up));
        sim.pedalTarget = floorIt ? 1.0 : latchedThrottle;
        sim.drive.clutchPedal = focus && isHeld(K::C);
        sim.drive.brake = focus && (isHeld(K::S) || isHeld(K::Down));

        // fixed-step physics, slowed down by the time scale
        if (!paused) {
            physAccum += dt * TIME_SCALES[scaleIdx];
            while (physAccum >= PHYS_DT) {
                sim.step(PHYS_DT);
                float smp = snd.sample(sim);  // always: its crackles also light the tailpipe flames
                if (audio) audio->ring.push(smp);
                physAccum -= PHYS_DT;
            }
        }

        if (audio) audio->simPerOut = (1.0 / PHYS_DT) * TIME_SCALES[scaleIdx] / engineAudio::RATE;

        // FPS averaged over the last half second
        fpsFrames++;
        if (clock2.getElapsedTime().asSeconds() >= 0.5f) {
            fps = fpsFrames / clock2.restart().asSeconds();
            fpsFrames = 0;
        }
        if (firstFrame || clock3.getElapsedTime().asSeconds() >= 0.1f) {
            firstFrame = false;
            double kw = sim.brakeAvg * sim.omega / 1000.0;
            std::string strokes;
            std::array<int, 4> order{0, 1, 2, 3};
            std::sort(order.begin(), order.end(), [&](int a, int b) { return sim.lag[a] < sim.lag[b]; });
            for (int c : order) {  // firing order
                std::string name = sim.stroke(c);
                name.resize(8, ' ');
                strokes += std::string(c == 0 ? "\n" : "") + " " + std::to_string(c + 1) + " " + name.substr(0, 4) + " " +
                           numToStr(static_cast<float>(sim.cyl[c].p / 1e5), 1);
            }
            const drivetrain &dv = sim.drive;
            auto f0 = [](double v) { return numToStr(static_cast<float>(v), 0); };
            auto gname = [](int g) { return g == 0 ? std::string("N") : g < 0 ? std::string("R") : std::to_string(g); };
            std::string gearName = gname(dv.gear);
            if (dv.shiftPhase) gearName += ">" + gname(dv.targetGear);
            double slipRpm = (sim.omega - dv.omegaIn) * 60.0 / (2.0 * 3.14159265358979323846);
            std::string flags = std::string(sim.ignition ? "" : "  IGNITION OFF") + (sim.revCut ? "  REV CUT" : "") +
                                (sim.fuelCut ? "  FUEL CUT" : "") + (sim.starter ? "  STARTER" : "") +
                                (dv.brake ? "  BRAKE" : "");
            bool bike = dv.P.bike;
            std::string camStr = sim.P.lowCam ? (sim.onLowCam(sim.rpm()) ? "   ShiftCam: partial-load lobe" : "   ShiftCam: full lobe") : "";
            if (bike && dv.qsPhase != 0) gearName += ">" + gname(dv.targetGear);
            if (bike && sim.tcTrim < 0.98) flags += "  WHEELIE CTRL";
            if (bike && !dv.wheelieControl) flags += "  wheelie ctrl off";
            if (sim.standalone)
                textStr = " FPS " + (fps > 0.f ? f0(fps) : std::string("--")) + "   time x" +
                      numToStr(static_cast<float>(TIME_SCALES[scaleIdx]), 2) + (paused ? "   PAUSED" : "") +
                      "\n RPM " + f0(sim.rpm()) + "   " + type.label + ", on a stand" + camStr +
                      "\n twist grip " + f0(sim.pedal * 100.0) + "%  throttle " + f0(sim.throttleOpen * 100.0) +
                      "%  manifold " + f0(sim.pMan / 1000.0) + " kPa" +
                      "\n torque " + numToStr(static_cast<float>(sim.brakeAvg), 1) + " N m  power " +
                      numToStr(static_cast<float>(std::max(kw, 0.0)), 1) + " kW (" +
                      f0(std::max(kw, 0.0) * 1.341) + " hp)" +
                      (flags.empty() ? "" : "\n" + flags) +
                      "\n shake 1st " + numToStr(static_cast<float>(sim.shake1() / 1000.0), 2) + " kN  2nd " +
                      numToStr(static_cast<float>(sim.shake2() / 1000.0), 2) + " kN  roll +-" + f0(sim.rollAmp()) + " N m" +
                      "\n sound " + (audio ? (audio->muted ? "muted" : "on") : "off") + "  shake view x" +
                      f0(SHAKE_VIEWS[shakeIdx]) + "  block " + (ghost == 0 ? "ghost" : "hidden") + "  blur " +
                      (blur ? "on" : "off") +
                      "\n firing (" + type.firing + "):" + strokes;
            else
            textStr = " FPS " + (fps > 0.f ? f0(fps) : std::string("--")) + "   time x" +
                      numToStr(static_cast<float>(TIME_SCALES[scaleIdx]), 2) + (paused ? "   PAUSED" : "") +
                      "\n RPM " + f0(sim.rpm()) + (*type.hudTag ? std::string("  ") + type.hudTag : std::string()) + "   gear " + gearName + (dv.automatic ? (dv.sport ? " SPORT" : " AUTO") : " MANUAL") +
                      "   " + f0(dv.v * 3.6) + " km/h" + camStr +
                      (bike ? "\n twist grip " : "\n pedal ") + f0(sim.pedal * 100.0) + "%  throttle " + f0(sim.throttleOpen * 100.0) +
                      "%  manifold " + f0(sim.pMan / 1000.0) + " kPa" +
                      (sim.P.turbo.on ? "\n boost " + std::string(sim.boostBar() >= 0.0 ? "+" : "") + numToStr(static_cast<float>(sim.boostBar()), 2) +
                                            (sim.P.turbo.supercharger
                                                 ? " bar  plenum " + f0(sim.pCharge / 1000.0) + " kPa  drive " + f0(sim.chargerTorque * sim.omega / 1000.0) +
                                                       " kW" + (sim.pInlet > sim.P.pAtm + 200.0 ? "  ram +" + f0(sim.pInlet - sim.P.pAtm) + " Pa" : "")
                                                 : " bar  wastegate " + f0(sim.wastegate * 100.0) + "%  back pressure " + f0(sim.pExh / 1000.0) + " kPa") +
                                            "\n charge " + f0(sim.tCharge - 273.15) + " C (compressor " + f0(sim.tComp - 273.15) + " C)  " +
                                            (sim.P.turbo.supercharger ? "bypass valve " : "blow-off valve ") +
                                            (!sim.P.turbo.bov ? "removed" : sim.bovOpen > 0.05 ? "VENTING" : "shut") +
                                            (sim.surge > 0.2 ? "  SURGE" : "")
                                      : std::string()) +
                      "\n torque " + numToStr(static_cast<float>(sim.brakeAvg), 1) + " N m  power " +
                      numToStr(static_cast<float>(std::max(kw, 0.0)), 1) + " kW  wheels " +
                      numToStr(static_cast<float>((bike ? sim.wheelForceFilt : dv.wheelForce) / 1000.0), 2) + " kN" +
                      (bike ? "\n clutch lever " : "\n clutch pedal ") + f0((1.0 - dv.clutch) * 100.0) + "%  slip " + f0(slipRpm) + " rpm" +
                      "\n hill " + (dv.grade >= 0 ? "+" : "") + f0(dv.grade * 100.0) + "%  mass " +
                      f0(dv.mass()) + " kg" + (dv.cargo > 0 ? (bike ? " (passenger)" : " (cargo)") : "") +
                      (showExhaust ? "\n exhaust: headers ~" + std::to_string(exhaust.headerC()) + " C" +
                                         (exhaust.flapAt.on ? "  valve " + f0(exhaust.flap * 100.0) + "% open" : "") +
                                         (showTurbo ? (sim.P.turbo.supercharger ? "  supercharger " : "  turbo ") + f0(sim.turboRpm() / 1000.0) + "k rpm" : "")
                                   : std::string()) +
                      (flags.empty() ? "" : "\n" + flags) +
                      "\n shake 1st " + numToStr(static_cast<float>(sim.shake1() / 1000.0), 2) + " kN  2nd " +
                      numToStr(static_cast<float>(sim.shake2() / 1000.0), 2) + " kN  roll +-" + f0(sim.rollAmp()) + " N m" +
                      "\n sound " + (audio ? (audio->muted ? "muted" : "on") : "off") + "  shake view x" +
                      f0(SHAKE_VIEWS[shakeIdx]) + "  block " + (ghost == 0 ? "ghost" : "hidden") + "  blur " +
                      (blur ? "on" : "off") +
                      (bike ? "\n firing (" + std::string(type.firing) + "):" : std::string()) + strokes;
            text.setString(textStr);
        }

        // 3D pass
        glClearColor(14 / 255.f, 15 / 255.f, 20 / 255.f, 1.f);
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        float aspect = float(winSize.x) / float(winSize.y);
        float fovY = 2.f * std::atan((HEIGHT / 2.f) / FOCAL);
        mat4 proj = perspective(fovY, aspect, 5.f, half * 40.f);
        mat4 view = viewMatrix(cam, centre);
        // the road (and everything on it) pitches with the grade about the axis across the vehicle, so
        // the front goes up a hill: the car faces world +z, the bikes face +x or -x (see ren.travelX)
        {
            float pitch = static_cast<float>(std::atan(sim.drive.grade)) * ren.travelSign;
            mat4 tilt = ren.travelX ? rotateZ(pitch) : rotateX(-pitch);
            view = view * translate(centre) * tilt * translate(centre * -1.f);
        }
        fx.gridScroll = static_cast<float>(sim.drive.distance * 1000.0);
        float delta = static_cast<float>(sim.theta - eng.cyls[0].theta0);
        poseEngine(eng, delta, modelOf);

        // combustion glow and flame per cylinder
        for (int c = 0; c < 4; c++) {
            float t = static_cast<float>(sim.glow(c));
            fx.glow[c] = fx.flame[c] = t;
            fx.crownY[c] = crownY[c];
            float lift = solveCylinder(eng, eng.cyls[c], delta).gudgeon.y - solveCylinder(eng, eng.cyls[c], 0.f).gudgeon.y;
            float crownNow = crownY[c] + lift;
            // the flame fills the real gas volume: a column of height V / A above the crown. Near TDC
            // it reaches past the deck into where the (unmodelled) head's combustion chamber would be.
            const cylinderState &cs = sim.cyl[c];
            float gasHeight = static_cast<float>(sim.volume(cs, sim.phase(c)) / cs.area * 1000.0);
            const float flameR = 0.5f * eng.cyls[c].bore * 0.97f;  // just inside the bore
            fx.flameModel[c] = eng.root * bankMatrix(eng, c) * translate({eng.cyls[c].gudgeonX, crownNow, eng.cyls[c].zCentre}) *
                               scale({flameR, -gasHeight, flameR});
        }
        // motion blur once the crank moves more than 45 degrees between frames (not at idle: the extra
        // draws would cost frames, which makes each frame's turn bigger and asks for even more ghosts)
        float frameTurn = std::remainder(delta - prevDelta, 4.f * PI);
        prevDelta = delta;
        fx.ghostPoses.clear();
        fx.ghostAlpha.clear();
        if (blur && !paused && std::fabs(frameTurn) > 45.f * PI / 180.f) {
            float span = std::clamp(frameTurn, -2.f * PI, 2.f * PI);
            int n = std::min(6, static_cast<int>(std::ceil(std::fabs(span) / (15.f * PI / 180.f))));
            for (int k = n; k >= 1; k--) {
                float f = static_cast<float>(k) / (n + 1);
                fx.ghostPoses.emplace_back();
                poseEngine(eng, delta - span * f, fx.ghostPoses.back());
                fx.ghostAlpha.push_back(0.35f * (1.f - f));
            }
        }
        // the whole engine sits on its mounts: vertical bounce and roll about the crank axis, in mesh
        // space (bores toward -y, so "up" is -y), optionally exaggerated so it can be seen
        {
            float ex = SHAKE_VIEWS[shakeIdx];
            vec3 axis{eng.crankAxis.x, eng.crankAxis.y, 0.f};
            mat4 mountMesh = translate({0.f, -static_cast<float>(sim.mountY * 1000.0) * ex, 0.f}) * translate(axis) *
                             rotateZ(static_cast<float>(sim.mountRoll) * ex) * translate(axis * -1.f);
            mat4 mountW = eng.root * mountMesh * eng.root;  // the root matrix is its own inverse
            for (auto &m : modelOf) m = mountW * m;
            for (auto &pose : fx.ghostPoses)
                for (auto &m : pose) m = mountW * m;
            for (auto &m : fx.flameModel) m = mountW * m;
            // the gearbox is bolted to the block, so it rides on the same mounts
            extras.clear();
            if (!opt.engineOnly) {
                if (type.bike) bbox.items(sim, delta, mountW * eng.root, ghost, extras);
                else gbox.items(sim, delta, mountW * eng.root, ghost, extras);
            }
            // the exhaust hangs off the heads, so it moves with the engine too
            if (showExhaust) {
                exhaust.update(sim, snd.popCount, snd.popLastAmp);
                exhaust.items(sim, mountW * eng.root, extras);
                if (showTurbo) {
                    turbo.update(sim);
                    turbo.items(mountW * eng.root, ghost, extras);
                }
            }
        }
        ren.draw(eng, modelOf, view, proj, ghost, fx, extras);

        // 2D HUD
        window.pushGLStates();
        if (opt.hud) {
        help.setPosition({10.f, float(winSize.y) - 66.f});
        credit.setPosition({float(winSize.x) - credit.getLocalBounds().size.x - 12.f, 8.f});
        sf::Text angle(font, " crank " + numToStr(crankDegrees(eng, delta), 1) + " deg   cyl 1 cycle " +
                                 numToStr(static_cast<float>(sim.theta / DEGR), 0) + " / 720", 16);
        angle.setPosition({0.f, text.getLocalBounds().size.y + 14.f});
        window.draw(text);
        window.draw(angle);

        // rpm and throttle bars under the stats
        auto bar = [&](float y, float frac, sf::Color fill, float mark) {
            const float X = 14.f, W = 260.f, H = 9.f;
            sf::RectangleShape back({W, H});
            back.setPosition({X, y});
            back.setFillColor(sf::Color(40, 44, 56));
            window.draw(back);
            if (mark > 0.f) {  // redline zone
                sf::RectangleShape red({W * (1.f - mark), H});
                red.setPosition({X + W * mark, y});
                red.setFillColor(sf::Color(110, 35, 40));
                window.draw(red);
            }
            sf::RectangleShape f({W * std::clamp(frac, 0.f, 1.f), H});
            f.setPosition({X, y});
            f.setFillColor(fill);
            window.draw(f);
        };
        float barY = angle.getPosition().y + 36.f;
        float rpmFrac = static_cast<float>(sim.rpm() / sim.P.rpmScale);
        bar(barY, rpmFrac, sim.rpm() > sim.P.revLimit - 300.0 ? sf::Color(235, 80, 80) : sf::Color(80, 200, 120),
            static_cast<float>(sim.P.revLimit / sim.P.rpmScale));
        bar(barY + 16.f, static_cast<float>(sim.pedal), sf::Color(245, 150, 60), 0.f);
        if (!sim.standalone) bar(barY + 32.f, static_cast<float>(1.0 - sim.drive.clutch), sf::Color(80, 140, 240), 0.f);  // clutch pedal travel
        // gearbox messages (shifts, "hold C") fade out after 2.5 s
        double age = sim.time - sim.drive.messageTime;
        if (!sim.drive.message.empty() && age < 2.5 * TIME_SCALES[scaleIdx]) {
            sf::Text msg(font, sim.drive.message, 20);
            msg.setFillColor(sf::Color(240, 210, 80));
            msg.setPosition({14.f, barY + 52.f});  // under the bars, clear of the stats and the dyno panel
            window.draw(msg);
        }

        // full-throttle dyno chart: torque and power against rpm, with where the engine is right now
        if (showDyno) {
            const float PW = 330.f, PH = 230.f;
            const float px = float(winSize.x) - PW - 12.f, py = float(winSize.y) - PH - 78.f;
            sf::RectangleShape panel({PW, PH});
            panel.setPosition({px, py});
            panel.setFillColor(sf::Color(20, 22, 30, 220));
            panel.setOutlineColor(sf::Color(55, 60, 75));
            panel.setOutlineThickness(1.f);
            window.draw(panel);

            const sf::Color TQ_COL(245, 150, 60), KW_COL(80, 200, 120), AXIS(90, 95, 115), GRID(45, 50, 62),
                LABEL(150, 150, 160);
            const int n = dyno.done.load(std::memory_order_acquire);
            const float rMin = dyno.rpm.front(), rMax = dyno.rpm.back();
            float tMax = 10.f, kMax = 10.f, peakT = 0.f, peakTRpm = 0.f, peakK = 0.f, peakKRpm = 0.f;
            for (int k = 0; k < n; k++) {
                if (dyno.torque[k] > peakT) { peakT = dyno.torque[k]; peakTRpm = dyno.rpm[k]; }
                if (dyno.kw[k] > peakK) { peakK = dyno.kw[k]; peakKRpm = dyno.rpm[k]; }
            }
            // round the scales up to a tidy number so the gridlines land on round values
            auto nice = [](float v) {
                float m = std::pow(10.f, std::floor(std::log10(v)));
                for (float f : {1.f, 2.f, 2.5f, 5.f, 10.f})
                    if (f * m >= v) return f * m;
                return 10.f * m;
            };
            tMax = nice(std::max(tMax, peakT * 1.1f));
            kMax = nice(std::max(kMax, peakK * 1.1f));

            const float L = px + 40.f, R = px + PW - 40.f, T = py + 40.f, B = py + PH - 26.f;
            auto xOf = [&](float r) { return L + (r - rMin) / (rMax - rMin) * (R - L); };
            auto yOf = [&](float v, float vMax) { return B - std::clamp(v / vMax, -0.05f, 1.02f) * (B - T); };
            auto label = [&](const std::string &s, float x, float y, sf::Color c, bool right = false) {
                sf::Text t(font, s, 11);
                t.setFillColor(c);
                t.setPosition({right ? x - t.getLocalBounds().size.x : x, y});
                window.draw(t);
            };

            // grid: every 1000 rpm (2000 on a high-revving engine), quarters of the vertical scales
            sf::VertexArray grid(sf::PrimitiveType::Lines);
            float rStep = rMax > 9000.f ? 2000.f : 1000.f;
            for (float r = std::ceil(rMin / rStep) * rStep; r <= rMax + 1.f; r += rStep) {
                grid.append({{xOf(r), T}, GRID});
                grid.append({{xOf(r), B}, GRID});
                label(numToStr(r / 1000.f, 0) + "k", xOf(r) - 6.f, B + 4.f, LABEL);
            }
            for (int q = 1; q <= 4; q++) {
                float y = B - q / 4.f * (B - T);
                grid.append({{L, y}, GRID});
                grid.append({{R, y}, GRID});
            }
            window.draw(grid);
            sf::Vertex axes[] = {{{L, T}, AXIS}, {{L, B}, AXIS}, {{L, B}, AXIS}, {{R, B}, AXIS}, {{R, B}, AXIS}, {{R, T}, AXIS}};
            window.draw(axes, 6, sf::PrimitiveType::Lines);
            label(numToStr(tMax, 0), L - 4.f, T - 7.f, TQ_COL, true);
            label("N m", L - 4.f, B - 14.f, TQ_COL, true);
            label(numToStr(kMax, 0), R + 4.f, T - 7.f, KW_COL);
            label("kW", R + 4.f, B - 14.f, KW_COL);

            // the curves (thickened by drawing each twice, a pixel apart)
            for (float off : {0.f, 1.f}) {
                sf::VertexArray tq(sf::PrimitiveType::LineStrip), pw(sf::PrimitiveType::LineStrip);
                for (int k = 0; k < n; k++) {
                    tq.append({{xOf(dyno.rpm[k]), yOf(dyno.torque[k], tMax) + off}, TQ_COL});
                    pw.append({{xOf(dyno.rpm[k]), yOf(dyno.kw[k], kMax) + off}, KW_COL});
                }
                window.draw(tq);
                window.draw(pw);
            }

            // where the engine is right now: a cursor at its rpm, dots at its smoothed torque and power
            float rNow = static_cast<float>(sim.rpm());
            if (rNow > rMin * 0.5f) {
                float x = std::clamp(xOf(rNow), L, R);
                sf::Vertex cursor[] = {{{x, T}, sf::Color(200, 200, 210, 90)}, {{x, B}, sf::Color(200, 200, 210, 90)}};
                window.draw(cursor, 2, sf::PrimitiveType::Lines);
                float tNow = static_cast<float>(sim.brakeAvg), kNow = static_cast<float>(sim.brakeAvg * sim.omega / 1000.0);
                for (auto [v, vMax, col] : {std::tuple{tNow, tMax, TQ_COL}, std::tuple{kNow, kMax, KW_COL}}) {
                    sf::CircleShape dot(4.f);
                    dot.setOrigin({4.f, 4.f});
                    dot.setPosition({x, yOf(v, vMax)});
                    dot.setFillColor(col);
                    dot.setOutlineColor(sf::Color::White);
                    dot.setOutlineThickness(1.f);
                    window.draw(dot);
                }
            }

            sf::Text title(font, "WOT dyno", 13);
            title.setPosition({px + 10.f, py + 6.f});
            title.setFillColor(sf::Color(200, 200, 210));
            window.draw(title);
            if (n < static_cast<int>(dyno.rpm.size()))
                label("measuring " + numToStr(dyno.rpm[n], 0) + " rpm...", px + PW - 10.f, py + 8.f, LABEL, true);
            else {
                label(numToStr(peakT, 0) + " N m @ " + numToStr(peakTRpm, 0), px + PW - 10.f, py + 2.f, TQ_COL, true);
                label(numToStr(peakK, 0) + " kW (" + numToStr(peakK * 1.341f, 0) + " hp) @ " + numToStr(peakKRpm, 0),
                      px + PW - 10.f, py + 15.f, KW_COL, true);
            }
        }
        }
        if (opt.hud) {
            window.draw(help);
            window.draw(credit);
        }
        window.popGLStates();

        if (!opt.screenshot.empty() && ++frame >= opt.frames) {
            sf::Texture shot(winSize);
            shot.update(window);
            bool ok = shot.copyToImage().saveToFile(opt.screenshot);
            std::printf("screenshot %s: %s\n", opt.screenshot.c_str(), ok ? "saved" : "FAILED");
            window.close();
            return ok ? 0 : 1;
        }
        window.display();
    }
}
