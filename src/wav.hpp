// --wav: render a scripted run (the stand script, or the engine's demo ride) to a WAV file
#pragma once
#include "engines.hpp"
#include "sound.hpp"

// headless: run a start / idle / rev / lift-off / ignition-off sequence and write it as a WAV (--wav)
int renderWav(const engineModel &e, const engineType &type, const std::string &path, bool turbo = false, bool bov = true) {
    engineSim sim;
    setupSim(sim, e, type, e.cyls[0].theta0, type.bike, turbo);  // a bike engine revs on its stand
    sim.P.turbo.bov = bov;
    // tuning hooks for the superbike voice
    auto env = [](const char *name, double &v) { if (const char *s = std::getenv(name)) v = std::atof(s); };
    env("SND_LOW", sim.P.sound.pipeLowHz); env("SND_OPEN", sim.P.sound.flapOpenHz); env("SND_DRIVE", sim.P.sound.drive);
    env("SND_REFLECT", sim.P.sound.pipeReflect); env("SND_DAMP", sim.P.sound.pipeDampHz); env("SND_LEN", sim.P.sound.pipeLength);
    env("SND_INTAKE", sim.P.sound.intakeGain); env("SND_RES", sim.P.sound.pipeResGain); env("SND_RAD", sim.P.sound.radiateHz); env("SND_BURBLE", sim.P.sound.burble); env("SND_BFROM", sim.P.sound.burbleFadeFrom); env("SND_BSPAN", sim.P.sound.burbleFadeSpan); env("SND_BHIGH", sim.P.sound.burbleHigh); env("SND_TICK", sim.P.sound.tick);
    env("SND_WHINE", sim.P.sound.whineGain); env("SND_POP", sim.P.sound.popChance); env("SND_TURBO", sim.P.sound.turboGain);
    engineSound snd(1.0 / PHYS_DT, sim.P.sound);
    std::vector<float> raw;
    struct segment { double until; bool starter, ignition; double throttle; const char *what; };
    const segment SCRIPT[] = {
        {1.2, true, true, 0.0, "starter"},   {4.0, false, true, 0.0, "idle"},
        {5.5, false, true, 1.0, "full throttle"}, {8.0, false, true, 0.0, "lift off"},
        {9.5, false, false, 0.0, "ignition off"},
    };
    // with the turbo: pulls in the car so it spools on load, with lift-offs for the blow-off valve
    // (or, with --bov no, the surge flutter)
    const segment TURBO_SCRIPT[] = {
        {1.2, true, true, 0.0, "starter"},        {3.0, false, true, 0.0, "idle"},
        {8.0, false, true, 1.0, "full throttle pull"}, {10.0, false, true, 0.0, "lift off"},
        {13.0, false, true, 1.0, "full throttle"}, {15.0, false, true, 0.0, "lift off"},
        {16.5, false, false, 0.0, "ignition off"},
    };
    double from = 0.0;
    if (!type.ride.empty()) {
        // the engine's demo ride (engines/<type>/): on the bike, starting in neutral
        sim.standalone = false;
        sim.drive.reset(true);
        sim.drive.bikeShift(0, sim.time, true);  // neutral
        for (const rideStep &sg : type.ride) {
            std::printf("  %4.1f - %4.1f s  %s\n", from, sg.until, sg.what);
            sim.starter = sg.starter;
            sim.pedalTarget = sg.throttle;
            sim.drive.brake = sg.brake;
            if (sg.shift > 0) sim.drive.bikeShift(+1, sim.time, false);
            if (sg.shift < 0) {
                sim.drive.automatic = false;  // the rider takes over the lever (gear and clutch stay as they are)
                int before = sim.drive.gear;
                sim.drive.bikeShift(-1, sim.time, false);
                std::printf("           quickshift %d -> %d requested: %s\n", before, sim.drive.targetGear, sim.drive.message.c_str());
            }
            while (sim.time < sg.until) {
                sim.step(PHYS_DT);
                raw.push_back(snd.sample(sim));
            }
            std::printf("           at %4.1f s: %5.0f rpm  gear %d  %3.0f km/h\n", sim.time, sim.rpm(), sim.drive.gear, sim.drive.v * 3.6);
            from = sg.until;
        }
    } else {
        auto play = [&](const segment &sg) {
            std::printf("  %4.1f - %4.1f s  %s\n", from, sg.until, sg.what);
            sim.starter = sg.starter;
            sim.ignition = sg.ignition;
            sim.pedalTarget = sg.throttle;
            double bovPeak = 0.0, surgePeak = 0.0, boostPeak = -1.0;
            while (sim.time < sg.until) {
                sim.step(PHYS_DT);
                raw.push_back(snd.sample(sim));
                bovPeak = std::max(bovPeak, sim.bovFlow);
                surgePeak = std::max(surgePeak, sim.surge);
                boostPeak = std::max(boostPeak, sim.boostBar());
            }
            if (turbo)
                std::printf("           at %4.1f s: %5.0f rpm  gear %d  %3.0f km/h  peak boost %+.2f bar  BOV %.3f kg/s  surge %.2f\n",
                            sim.time, sim.rpm(), sim.drive.gear, sim.drive.v * 3.6, boostPeak, bovPeak, surgePeak);
            from = sg.until;
        };
        if (turbo) for (const segment &sg : TURBO_SCRIPT) play(sg);
        else for (const segment &sg : SCRIPT) play(sg);
    }
    // resample 40 kHz -> 44.1 kHz, linear
    const unsigned RATE = engineAudio::RATE;
    std::vector<std::int16_t> pcm;
    double ratio = (1.0 / PHYS_DT) / RATE;
    for (double t = 0.0; t + 1.0 < raw.size(); t += ratio) {
        std::size_t i = static_cast<std::size_t>(t);
        double f = t - i;
        double v = raw[i] + (raw[i + 1] - raw[i]) * f;
        pcm.push_back(static_cast<std::int16_t>(std::clamp(v, -1.0, 1.0) * 30000.0));
    }
    std::ofstream f(path, std::ios::binary);
    if (!f) return 1;
    auto u32 = [&](std::uint32_t v) { f.write(reinterpret_cast<const char *>(&v), 4); };
    auto u16 = [&](std::uint16_t v) { f.write(reinterpret_cast<const char *>(&v), 2); };
    std::uint32_t bytes = static_cast<std::uint32_t>(pcm.size() * 2);
    f.write("RIFF", 4); u32(36 + bytes); f.write("WAVEfmt ", 8);
    u32(16); u16(1); u16(1); u32(RATE); u32(RATE * 2); u16(2); u16(16);
    f.write("data", 4); u32(bytes);
    f.write(reinterpret_cast<const char *>(pcm.data()), bytes);
    std::printf("wrote %s (%.1f s, %u Hz mono)\n", path.c_str(), pcm.size() / double(RATE), RATE);
    return f ? 0 : 1;
}

