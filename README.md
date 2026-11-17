# Engine simulator

A real-time 3D engine simulator. The crank, rods and pistons move on exact slider-crank geometry, and
the crank speed comes from a physics model, not a fixed rate:

- a pressure model for each cylinder (intake, compression, Wiebe combustion, expansion, blowdown);
- the gas and reciprocating-mass torques on the crank;
- friction, a starter motor and an ECU that controls the idle, the rev limiter and the fuel cut;
- a clutch, a gearbox and the vehicle the engine pushes.

The sound is synthesised from the same simulation.

It has three engines. `engine_sim` is the single entry point, and you pick the engine with
`--engine-type`:

| `--engine-type` | Engine | Drives | Also accepted |
|---|---|---|---|
| `inline4` (default) | 2.3 L inline-4 (the Sketchfab model) | a car with Honda Fit 5-speed gearing | `fit`, `honda-fit`, `inline4-fit`, `i4` |
| `v4` | Ducati Desmosedici Stradale 90° V4, 1103 cc | a Panigale V4 (bike) | `ducati`, `ducati-v4`, `panigale` |
| `s1000rr` | BMW S 1000 RR inline-4, 999 cc | an S 1000 RR (bike) | `bmw`, `s1000`, `bmw-s1000rr` |

```bash
./engine_sim --engine-type list
```

## Quick start

You need SFML 3 (Homebrew: `brew install sfml`; on Linux the Makefile looks in `/usr/local`), a
C++20 compiler and Python 3. The mesh builders use only the standard library.

```bash
make
```

The 6 MB model data (`engines/inline4_fit/scene.bin`) is not in git. `make` downloads it from this
repo's GitHub release (`Assets-1`) on first run, verified by checksum, using only the Python
standard library. To fetch it by hand: `make assets`. If the repo is private, export a token first:
`export GITHUB_TOKEN=$(gh auth token)`. The file list lives in `assets.json`.

```bash
./engine_sim --engine-type s1000rr
```

`make` builds `engine_sim` and any missing meshes. Run it from the project root, because it loads the
meshes and `SpaceMono-Regular.ttf` from there. Shortcuts: `make run` (inline-4), `make run-v4` and
`make run-s1000rr`. `make test` runs the headless checks for every engine.

## The engines

### `inline4`: 2.3 L inline-4 with Honda Fit gearing

The original model, split into moving parts from the glTF.

| | |
|---|---|
| Engine | 85 mm bore, 160 mm rods, 10:1, flat-plane crank, firing 1-3-4-2 |
| Stroke | The model's crank throws are uneven: 108.4 mm on cylinders 1/4 and 97.8 mm on 2/3 (~2.34 L). Each cylinder uses its own measured throw, so the parts stay joined. |
| Output (sim) | 203 N·m at 3500 rpm, 110 kW at 6500–7000 rpm, idle 800 rpm, limiter 6800 rpm |
| Gearbox | 2nd-gen (2009–2013) Honda Fit 5MT: 3.308 / 1.870 / 1.303 / 0.949 / 0.727, reverse 3.308, final drive 4.62 |
| Vehicle | 1300 kg car, 0.305 m tyres, CdA 0.65 |
| Drivetrain | Friction clutch, automatic mode (shift schedule, kickdown, downshift under load, sport mode with rev-matched downshifts) or manual (clutch pedal, stalls if you dump it at idle) |
| Results (sim) | 0–100 km/h in 7.7 s |
| 3D | The see-through block, and a procedural 5-speed gearbox with a reverse idler behind the flywheel |

### `v4`: Ducati Desmosedici Stradale (Panigale V4)

Built from the inline-4's piston and rod, scaled, on a new procedural crank and crankcase.

| | |
|---|---|
| Engine | 81 × 53.5 mm = 1103 cc, 14.0:1, 90° V tilted back 42°, counter-rotating crank |
| Firing | Twin Pulse 0-90-290-380° |
| Output (sim) | 124 N·m at 10500 rpm, 155 kW (208 hp) at 13000 rpm (Ducati: 124 N·m / 157 kW), limiter 14000 rpm, idle 1250 rpm |
| Gearbox | Primary 54/30; gears 38/14, 36/17, 33/19, 32/21, 30/22, 30/24; chain 41/16; 200/55 ZR17 rear |
| Vehicle | 278 kg with rider, CdA 0.32 |
| Rider aids | Sequential 1-N-2-…-6 box, up/down quickshifter (rev-matched downshifts), slipper clutch, launch control, wheelie control |
| Results (sim) | 0–100 km/h 3.3 s, 0–200 km/h 6.8 s, top speed ~289 km/h |
| Sound | Its own Ducati voice. The Twin Pulse firing comes from the physics: pulses arrive in pairs, so the howl sits at the crank rate (rpm/60), an octave below an even-firing inline-4, with a loping, off-beat idle. On top of that:<br>• longer headers from both banks (lower, darker pipe modes)<br>• an idle burble that gives way to the radiated howl as the revs rise<br>• exhaust valve, airbox honk<br>• more overrun crackle than the BMW<br>• Desmo valvetrain clatter |

### `s1000rr`: BMW S 1000 RR (2023-, K67 ShiftCam engine)

Built the same way as the V4: an inline-4 with the block inclined 32° forward and a forward-turning
crank.

| | |
|---|---|
| Engine | 80.0 × 49.7 mm = 999 cc, 13.3:1, 103 mm rods, 180° flat-plane crank |
| Firing | 1-2-4-3 |
| ShiftCam | A partial-load intake lobe in the mid range, then the full lobe; the HUD shows which is active |
| Output (sim) | 114.5 N·m at 11000 rpm, 153 kW at 14000–14500 rpm (BMW: 113 N·m / 154 kW at 13750), limiter 14600 rpm, idle 1250 rpm |
| Gearbox | Primary 76/46 (1.652); gears 45/17, 46/22, 38/22, 33/22, 34/25, 29/23; chain 46/17; 190/55 ZR17 rear |
| Vehicle | 197 kg wet + 80 kg rider, CdA 0.31 |
| Rider aids | Same as the V4: quickshifter with rev-matched downshifts, slipper clutch, launch control, wheelie control |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 6.9 s, 307 km/h in 6th |
| Sound | Its own voice (the inline-4 keeps the original one):<br>• radiated exhaust (brightens with revs) through a 4-2-1 quarter-wave pipe<br>• exhaust flap, shut at idle and open from ~5000 rpm or under throttle<br>• airbox honk at full throttle<br>• overrun crackle that fades out<br>• quickshifter bark<br>• straight-cut gear whine<br>• valvetrain tick |

**Estimates.** These values are not published: the V4 rod length; the S 1000 RR bore spacing (92 mm);
and the crank inertia, reciprocating mass, friction and cam timing of both bikes. They are estimates,
calibrated so that the published torque and power come out of the physics.

## Controls

| Input | Action |
|---|---|
| Left-drag / scroll | Orbit / zoom |
| SPACE (hold) | Starter |
| W or Up (hold) | Throttle; `1`–`9`, `0` latch 10–90 %, 100 % |
| I | Ignition on/off |
| S (hold) | Brake |
| T | Automatic / manual |
| Z | Sport mode (automatic: holds gears, rev-matched downshifts) |
| E / Q | Shift up / down. Car: Q at a stop selects R. Bikes: the quickshifter works while moving, no clutch needed. |
| N | Neutral (bikes) |
| C (hold) | Clutch |
| V | Wheelie control on/off (bikes) |
| H / J | Hill up / down |
| L | Cargo (car) / passenger (bikes) |
| `[` / `]` | Slow motion / back toward real time (×1 … ×0.01) |
| `,` / `.` | When paused: step the crank by 1° |
| G / B / M / D / X | See-through block / motion blur / mute / p-V diagram / shake view (×1, ×20, ×100) |
| P / R / Esc | Pause / reset / quit |

## Command-line options

| Option | |
|---|---|
| `--engine-type T` (or `--engine-type=T`) | `inline4` (default), `v4`, `s1000rr` or an alias; `list` prints them. `--v4` and `--s1000rr` still work. |
| `--engine-only` | No gearbox or vehicle (a bike engine runs on a stand) |
| `--selftest` | Kinematics checks: parts stay joined, strokes, clearances, firing lags vs. crank geometry |
| `--phystest` | Physics checks against the engine's targets (start, idle, limiter, dyno curve, firing order, balance, acceleration, shifting, quickshifter rev-match, …) |
| `--wav out.wav` | Render a scripted run to a WAV file. The bikes play a ride: start, blips, limiter, launch through the gears, braking with rev-matched downshifts. The inline-4 plays a start / rev / lift-off script. |
| `--screenshot out.png [--frames n]` | Save a frame and quit. Framing: `--yaw r --pitch r --zoom s`. State: `--crank deg`, `--simtime s --start --throttle x`, `--stopat deg` |
| `--nohud --noblur --mute --ghost 0\|1` | Options for regression shots |
| `--grade pct --reverse --sport --no-wheelie` | Starting conditions |
| `--compare a.png b.png` | Pixel diff of two screenshots |

## Project layout

```
engine_sim.cpp              entry point: command line, window, event loop, HUD
src/
  common.hpp                includes, vec3/mat4, orbit camera
  model.hpp                 mesh loading (v1 and banked v2), slider-crank kinematics, --selftest
  render.hpp                OpenGL 2.1 renderer, combustion glow, motion blur, gear meshes
  physics.hpp               engine model, ECU, drivetrain, vehicle (parameter structs + engineSim)
  engine_type.hpp           engineType: what an engine provides (params, mesh, labels, tests, demo)
  engines.hpp               registry for --engine-type, setupSim()
  sound.hpp, wav.hpp        sound synthesis + audio stream, --wav rendering
  bike_gearbox.hpp          the bikes' 6-speed gearbox (3D)
  test_bike.hpp             --phystest for the bikes, checked against each engine's bikeSpec
engines/
  inline4_fit/              inline4_fit.hpp (engine + Honda Fit gearing + type), gearbox_5mt.hpp (3D car
                            gearbox), test_inline4.hpp, extract_parts.py, scene.gltf/.bin, license.txt,
                            engine.mesh, tests/ (pixel baselines)
  ducati_v4/                ducati_v4.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_v4.py, v4.mesh
  bmw_s1000rr/              bmw_s1000rr.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_s1000rr.py, s1000rr.mesh
tools/meshlib.py            mesh helpers shared by build_v4.py and build_s1000rr.py
plan.md                     development log (design notes and measurements, phase by phase)
```

The C++ builds as a single translation unit: `engine_sim.cpp` includes the headers.

**Meshes.** `extract_parts.py` splits the glTF into moving groups (crank, pistons, rods, static parts,
block) and measures the pivots from the geometry. `build_v4.py` and `build_s1000rr.py` reuse the
inline-4's piston and rod from that mesh, and add a procedural crank, crankcase, cylinders and head.

**Coordinates.** Mesh units are mm, and the crank axis is Z. The V4's front is −X and its crank
counter-rotates. The S 1000 RR's front is +X and its crank turns forward, so its gearbox is mirrored to
sit behind the crank.

## Adding an engine

1. **Folder.** Make `engines/<name>/` with a mesh builder. Copy `build_s1000rr.py` as a starting point:
   it writes the banked v2 mesh format with a bank angle, deck, bore and firing lag per cylinder.
2. **Header.** Write `<name>.hpp` with:
   - an `engineParams` function (geometry, cam timing, VE curve, limiter, and `sound` for the voice);
   - a `drivetrainParams` function (gearing and vehicle);
   - a `bikeSpec` function for the test targets (bikes);
   - a function returning its `engineType`, with the id, labels, mesh path, physics test, and for bikes
     the gearbox placement and an optional `ride` for `--wav`.
3. **Register it.** Add it to `engineTypes()` in `src/engines.hpp`, include the header in
   `engine_sim.cpp`, and add the mesh rule to the `Makefile`.
4. **Calibrate.** Run `./engine_sim --engine-type <name> --phystest`. The `VE_*`, `FMEP_*`, `IDLE_*` and
   `SND_*` environment variables override parameters while you calibrate.

## Credits

The 3D model is ["Inline-4 Engine" by rafaelaraujo55](engines/inline4_fit/license.txt), licensed
CC-BY-4.0. The credit line is shown on screen. The V4 and S 1000 RR are built from its parts.

Published figures come from Ducati, BMW Motorrad (technical data and press material) and Honda's 2010
Fit specifications. This is a simulation, not manufacturer data.
