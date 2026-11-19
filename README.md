# Engine simulator

A real-time 3D engine simulator. The crank, rods and pistons move on exact slider-crank geometry, and
the crank speed comes from a physics model, not a fixed rate:

- a pressure model for each cylinder (intake, compression, Wiebe combustion, expansion, blowdown);
- the gas and reciprocating-mass torques on the crank;
- friction, a starter motor and an ECU that controls the idle, the rev limiter and the fuel cut;
- a clutch, a gearbox and the vehicle the engine pushes.

The sound is synthesised from the same simulation.

It has seven engines. `engine_sim` is the single entry point, and you pick the engine with
`--engine-type`:

| `--engine-type` | Engine | Drives | Also accepted |
|---|---|---|---|
| `inline4` (default) | 2.3 L inline-4 (the Sketchfab model) | a car with Honda Fit 5-speed gearing | `fit`, `honda-fit`, `inline4-fit`, `i4` |
| `rs3` | Audi RS 3 2.5 TFSI turbocharged inline-5, 2480 cc | an RS 3 (8Y) with the 7-speed S tronic | `audi`, `audi-rs3`, `rs-3`, `inline5`, `i5`, `2.5tfsi`, `daza` |
| `v4` | Ducati Desmosedici Stradale 90° V4, 1103 cc | a Panigale V4 (bike) | `ducati`, `ducati-v4`, `panigale` |
| `s1000rr` | BMW S 1000 RR inline-4, 999 cc | an S 1000 RR (bike) | `bmw`, `s1000`, `bmw-s1000rr` |
| `h2` | Kawasaki Ninja H2 supercharged inline-4, 998 cc | a Ninja H2 (bike) | `kawasaki`, `kawasaki-h2`, `ninja-h2` |
| `zx10r` | Kawasaki Ninja ZX-10R inline-4, 998 cc | a ZX-10R (bike) | `zx-10r`, `ninja-zx10r`, `ninja-zx-10r`, `zx10`, `kawasaki-zx10r` |
| `mxking` | Yamaha MX King 150 single, 149.7 cc | an MX King underbone (bike) | `mx-king`, `jupiter-mx`, `t150`, `yamaha150` |

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
`export GITHUB_TOKEN=$(gh auth token)`. The file list lives in `assets.json`. The turbocharger's
14 MB `engines/inline4_fit/turbocharger/scene.bin` comes the same way from release `Assets-2`; it is
optional, so without it everything builds and only `--turbo yes` is unavailable.

```bash
./engine_sim --engine-type s1000rr
```

`make` builds `engine_sim` and any missing meshes. Run it from the project root, because it loads the
meshes and `SpaceMono-Regular.ttf` from there. Shortcuts: `make run` (inline-4), `make run ENGINE=<id>`
or `make run-<id>` (`make run-rs3`, `make run-v4`, `make run-s1000rr`, `make run-h2`, `make run-zx10r`, `make run-mxking`; aliases work too). `make test` runs
the headless checks for every engine and version of one (`./engine_sim --engine-type list --ids`), and
fails if any of them fails.

## The engines

### `inline4`: 2.3 L inline-4 with Honda Fit gearing

The original model, split into moving parts from the glTF.

| | |
|---|---|
| Engine | 85 mm bore, 160 mm rods, 10.5:1, flat-plane crank, firing 1-3-4-2. Tuned to match a Honda K24Z3 (2008–12 Accord: 2.35 L, 177 hp at 6500 rpm, 218 N·m at 4300 rpm) |
| Stroke | The model's crank throws are uneven: 108.4 mm on cylinders 1/4 and 97.8 mm on 2/3 (~2.34 L). Each cylinder uses its own measured throw, so the parts stay joined. |
| Output (sim) | 223 N·m at 4000–4500 rpm, 131 kW (176 hp) at 6500 rpm, idle 800 rpm, limiter 6800 rpm |
| Gearbox | 2nd-gen (2009–2013) Honda Fit 5MT: 3.308 / 1.870 / 1.303 / 0.949 / 0.727, reverse 3.308, final drive 4.62 |
| Vehicle | 1300 kg car, 0.305 m tyres, CdA 0.65 |
| Drivetrain | Friction clutch, automatic mode (shift schedule, kickdown, downshift under load, sport mode with rev-matched downshifts) or manual in two modes (Z): NORMAL (clutch pedal for every change, stalls if you dump it at idle) and DCT (see the RS 3) |
| Results (sim) | 0–100 km/h in 6.8 s |
| 3D | The see-through block, and a procedural 5-speed gearbox with a reverse idler behind the flywheel (the car gearbox drawing moves back, `gearboxModel::shiftFor`, for an engine whose crank or crankcase reaches past the inline-4's) |
| Exhaust (3D) | Tubular 4-2-1 manifold (1+4, 2+3) on a cast flange, close-coupled catalyst, mid pipe, silencer and tailpipe running back beside the gearbox |

### `rs3`: Audi RS 3 (8Y, 2021-), 2.5 TFSI turbocharged inline-5

```bash
./engine_sim --engine-type rs3
```

The five-cylinder, built like the superbikes from the inline-4's piston and rod (scaled to the 82.5 mm
bore and the 144 mm rod) on a procedural crank with its pins 72° apart, with a red RS cam cover. It is
the only engine that is always turbocharged (`engineType::boost = builtInBoost::turbo`; the H2 is
`supercharger`): the turbo is on without `--turbo yes` and is drawn with the inline-4's turbo model,
or a procedural one when that is not there (below).

| | |
|---|---|
| Engine | 82.5 × 92.8 mm = 2480 cc, 10.0:1, 144 mm rods, five cylinders 88 mm apart (estimate), the bank upright, 74 mm throttle |
| Firing | 1-2-4-5-3, every 144°. No free 1st- or 2nd-order force; the rocking couples the test checks against the textbook (0.449 and 4.98 × the bore pitch) |
| Turbo | One turbo: a small turbine nozzle so it spools from ~2000 rpm, a big wastegate, a high-pressure-ratio compressor with a wide map (its surge line well left of the engine's flow at 2000 rpm, no choke at 7000; the test checks it never surges at full throttle). Blow-off valve, front-mounted intercooler |
| ECU boost | Up to 1.5 bar, but a charge (torque) limit holds the 500 N·m plateau with ~1.2–1.4 bar, and the target falls from 5500 rpm to ~1.1 bar at 7000 for the flat top-end power (`turboParams::chargeMax`, `boostFallRpm`…) |
| Output (sim) | 492 N·m at 2000, 500–507 N·m at 2500–4500, 485 at 5500; 299–300 kW (~402 hp) at 6000–6500 and 289 kW at 7000 rpm (Audi: 500 N·m at 2250–5600, 294 kW / 400 PS at 5600–7000), idle 770 rpm, limiter 7100 rpm, peak cylinder pressure 150 bar |
| Gearbox | 7-speed S tronic (DQ500 dual clutch): 3.563 / 2.526 / 1.679 / 1.022 / 0.788 / 0.761 / 0.635, reverse 2.789, final drives 4.059 (1st, 4th, 5th, R) and 3.450 (2nd, 3rd, 6th, 7th): Audi's 8V figures, the same box. The sim has one final drive (3.450), so 1st, 4th, 5th and reverse carry the 4.059 / 3.450 step in their ratio. Shifts take 0.03 s out and 0.08 s in. Shifts the same in manual's DCT mode (any car: T for manual, then Z). Drawn as a dual-clutch box (`drivetrainParams::dualClutch`): two wet clutch packs nested in a crank-driven drum, the outer one for the odd gears, the inner one for the even gears and reverse, on two coaxial input shafts; the pack carrying the drive closes (its apply plate brass), and a shift hands over from one to the other |
| Vehicle | 1570 kg + 75 kg driver, 265/30 R19 (0.325 m), CdA 0.72, quattro (the car model has no tyre grip limit, so all-wheel drive needs nothing extra) |
| Exhaust (3D) | Five cast runners into a manifold over the turbine, the downpipe back under the car, catalyst, mid pipe with the sport exhaust's flap, rear silencer and two big tailpipes. Intake: airbox behind the turbo, charge pipe to a front-mounted intercooler, back up the other side to the throttle and a plenum along the head |
| Results (sim) | 0–100 km/h 3.7 s with launch control (S and full throttle, then let go of S: held at ~4500 rpm on 1.2 bar; Audi: 3.8 s), 4.3 s floored from idle (the turbo spools on the way), 0–200 km/h 14.0 s, full-throttle upshifts at ~7000 rpm |
| Sound | The detailed voice (`soundParams::detailed`, the bikes' stages) on a car: a long pipe for the deep five-cylinder note and its idle burble, the sport exhaust's flaps opening from ~3500 rpm, overrun crackle (with flames), the turbo's whistle and the blow-off valve |

`--turbo yes` is refused (it already has one). Without the downloaded turbo model (the optional
`Assets-2` download) the RS 3 draws a procedural turbo instead (`build_rs3_turbo.py` ->
`rs3_turbo.mesh`, `engineType::turboFallbackMeshFile`): turbine and compressor wheels on their shaft,
the centre housing, and see-through scrolls with their four ports where the downloaded model has them,
so the same exhaust and intake pipes fit either.

#### Tuned (`--tuned yes`): big turbo

```bash
./engine_sim --engine-type rs3 --tuned yes
```

A typical big-turbo build (estimates, not one tuner's car) on the stock 82.5 × 92.8 mm, 2480 cc and
10.0:1: only what bolts on, the map and forged internals of the same dimensions change.

| | |
|---|---|
| Turbo | A larger hybrid: a bigger compressor wheel (pressure ratio 4.6 at the reference speed, more flow, still clear of surge at full throttle) and a bigger turbine housing, so it breathes to 7400 rpm but comes on boost at ~3000–3500 rpm |
| ECU | Ethanol blend: up to ~1.9 bar, the torque limit raised to ~610 N·m, 5.5° of knock retard per bar (stock 7), a little more advance; the boost eases off above 6000 rpm |
| Engine | Forged rods and pistons (same dimensions): limiter 7400 rpm. 80 mm throttle and intake, bigger intercooler, catless downpipe, valved cat-back |
| Output (sim) | 613 N·m at 3500, ~600 N·m to 5500, 417 kW (559 hp) at 7000 rpm (stock: 507 N·m, 300 kW); peak cylinder pressure 179 bar |
| Gearbox | Stock ratios; a gearbox tune: more clutch clamping (950 N·m), full-throttle upshifts at ~7300 rpm, launch control a little higher (~4900 rpm on 1.5 bar) |
| Results (sim) | 0–100 km/h 3.0 s with launch control (3.6 s floored from idle), 0–200 km/h 10.8 s |
| 3D | The downpipe a size up with a straight pipe where the catalyst was, bigger blued titanium tips |
| Sound | Louder, flaps open from 2500 rpm, more crackle, more turbo |


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
| Exhaust (3D) | Front-bank headers wrap down in front of the crankcase, rear-bank headers drop behind the gearbox; each bank's pair merges and both feed the black pre-silencer under the engine, with an exhaust valve and two short outlets on the right |
| Results (sim) | 0–100 km/h 3.3 s, 0–200 km/h 6.8 s, top speed ~289 km/h |
| Sound | Its own Ducati voice. The Twin Pulse firing comes from the physics: pulses arrive in pairs, so the howl sits at the crank rate (rpm/60), an octave below an even-firing inline-4, with a loping, off-beat idle. On top of that:<br>• longer headers from both banks (lower, darker pipe modes)<br>• an idle burble that gives way to the radiated howl as the revs rise<br>• exhaust valve, airbox honk<br>• more overrun crackle than the BMW<br>• Desmo valvetrain clatter |

#### Tuned (`--tuned yes`)

```bash
./engine_sim --engine-type v4 --tuned yes
```

Ducati's own accessory route to more power: the full titanium racing exhaust (made by Akrapovič) and
the ECU map that comes with it, quoted at about 226 hp (166 kW) against the stock 214. On top of that,
for top speed, a higher limiter and taller gearing (estimates, not Ducati's kit). The engine and the
gearbox stay stock. The changes:

- the freer system and the map: more torque everywhere, from ~+1% at 4000–6000 rpm growing to ~+7% at
  the top, and the map adds a little spark advance;
- the map's limiter is 500 rpm up, at 14500 rpm, and the full-throttle upshift moves to ~14100 rpm;
- a 38-tooth rear sprocket in place of the 41 (16/38): every gear ~8% taller, so 6th now runs out where
  the power and the drag balance instead of on the stock limiter. The taller 1st makes a gentle 25%
  take-off bog to ~780 rpm (`takeoffMinRpm = 700`);
- the exhaust: heat-blued titanium headers a size up on the stock route, a titanium collector under the
  engine in place of the black pre-silencer, and two short titanium cans stacked on the right with
  carbon end caps. No exhaust valve;
- the voice: louder, with a harder idle lope, more crackle and a slightly higher pipe note.

| | Stock | Tuned (sim) |
|---|---|---|
| Peak torque | 124 N·m at 10500 rpm | 130 N·m at 11000 rpm |
| Peak power | 155 kW (208 hp) at 13000 rpm | 164.5 kW (221 hp) at 13000 rpm |
| Limiter | 14000 rpm | 14500 rpm |
| Chain | 16/41 | 16/38 |
| Top of 1st / 2nd / 3rd (from the gearing) | ~136 / 174 / 212 km/h | ~152 / 194 / 237 km/h |
| 0–100 km/h | 3.3 s | 3.3 s |
| 0–200 km/h | 6.8 s | 6.7 s |
| After 30 s in 6th | ~290 km/h (on the 14000 limiter) | ~326 km/h |

The HUD shows TUNED after the rpm.

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
| Exhaust (3D) | 4-2-1 titanium headers from the front of the inclined head, under the engine (1+4 and 2+3 merge), collector, link pipe with the exhaust flap, short silencer rising on the right behind the gearbox |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 6.9 s, 307 km/h in 6th |
| Sound | Its own voice (the inline-4 keeps the original one):<br>• radiated exhaust (brightens with revs) through a 4-2-1 quarter-wave pipe<br>• exhaust flap, shut at idle and open from ~5000 rpm or under throttle<br>• airbox honk at full throttle<br>• overrun crackle that fades out<br>• quickshifter bark<br>• straight-cut gear whine<br>• valvetrain tick |

#### Tuned (`--tuned yes`)

```bash
./engine_sim --engine-type s1000rr --tuned yes
```

BMW's own tune of this engine: the M 1000 RR's (2021-), which BMW rates at 156 kW (212 hp) at
14500 rpm and 113 N·m at 11000 rpm, with a 15100 rpm limiter. The block, the ShiftCam partial-load
lobe, the gearbox and the gearing stay stock. The changes:

- titanium rods and lighter two-ring pistons (less reciprocating mass), 13.5:1;
- reworked ports and a longer race lobe (intake closes 5° later, exhaust opens 5° earlier), so the
  engine breathes to the higher limiter. The mid range below 9000 rpm is unchanged;
- the limiter raised from 14600 to 15100 rpm, and the full-throttle upshift moved to ~14700 rpm;
- a titanium race exhaust: the stock 4-2-1 route without the exhaust valve, and a short titanium can
  with a carbon end cap in place of the silencer. It's louder, with more crackle and a higher note.

| | Stock | Tuned (sim) |
|---|---|---|
| Peak torque | 114.5 N·m at 11000 rpm | 115 N·m at 11000 rpm |
| Peak power | 153 kW (205 hp) at 14500 rpm | 156 kW (209 hp) at 14500 rpm |
| Limiter | 14600 rpm | 15100 rpm |
| 0–200 km/h | 6.9 s | 6.8 s |
| After 30 s in 6th | ~307 km/h | ~319 km/h |

The sim's figures come from `--phystest` (no ram air), like the stock engine's. The HUD shows TUNED
after the rpm.

### `h2`: Kawasaki Ninja H2 (2015-), supercharged

Built like the S 1000 RR, plus a procedural centrifugal supercharger behind the cylinders.

| | |
|---|---|
| Engine | 76.0 × 55.0 mm = 998 cc, 8.5:1, 107 mm rods (estimate), 180° flat-plane crank, block inclined 30° (estimate), forward-turning crank |
| Firing | 1-2-4-3 |
| Supercharger | Gear-driven from the crank at 9.2× crank speed (~129k rpm at 14000), 69 mm impeller with 6 blades and 6 splitters, no intercooler, ram air, a 4 L plenum over the throttle bodies with a relief valve that holds ~+1.0 bar from ~10500 rpm, and a bypass valve that vents the charge when the throttles snap shut |
| Output (sim) | 133 N·m at 10500 rpm, 147.5 kW at 11000–11500 rpm with the charge hot (~100 °C) and no ram air (Kawasaki: 133.5 N·m at 10500 / 147.2 kW at 11000), limiter 14000 rpm, idle 1200 rpm |
| Gearbox | Primary 74/50 (1.480); gears 51/16, 48/19, 45/22, 38/22, 32/21, 31/23; chain 44/18; 200/55 ZR17 rear |
| Vehicle | 238 kg wet + 80 kg rider, CdA 0.33 |
| Rider aids | Same as the other bikes; launch control holds ~7500–9500 rpm, where the boost is |
| Exhaust (3D) | The S 1000 RR's 4-2-1 layout into a bigger black silencer |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 7.0 s, ~312 km/h after 30 s in 6th (Kawasaki limits it to 299) |
| Sound | The S 1000 RR's voice with a longer pipe, less airbox honk, more gear whine, and the supercharger's whine (impeller blade pass) and rush on top, with the bypass valve's chirp on each snap shut |

**How the supercharger is modelled.** It uses the turbo's compressor model (`turboParams` with
`supercharger = true`): the shaft is geared to the crank instead of driven by a turbine, so there is
no exhaust back pressure, turbine lag or wastegate. The boost follows the revs (roughly with rpm²),
and the crank pays for the compressor's power (~18–22 kW at the top), which counts as friction.
The relief valve is what caps the boost. The `--phystest` dyno sweep and the D chart both measure
with the charge as hot as a long pull leaves it, which is what the H2 is calibrated against; the HUD shows boost, plenum pressure, the drive power, the
charge temperature (no intercooler: ~100 °C in a long full-throttle pull) and the impeller speed. `F` takes the
bypass valve off, so the compressor surges on lift-offs.

The 3D supercharger (`h2_supercharger.mesh`, written by `build_h2.py` with the engine) is drawn by
`src/turbo.hpp`: the volute, ram-air duct, plenum and throttle bodies are see-through like the block
(`G` hides them), the impeller spins in proportion to its speed.

#### Tuned (`--tuned yes`)

```bash
./engine_sim --engine-type h2 --tuned yes
```

A top-end tune that moves the peak up toward the 14000 rpm redline. The block, supercharger,
gearing and limiter stay stock; the changes are estimates of a race kit, not a published one:

- higher-lift cams: the engine breathes best near 12000 rpm instead of 9000. It trades mid range for
  top end: stock up to ~3000 rpm, then weaker up to ~10700 rpm (16–18% less torque at 6000–8000 rpm),
  and stronger above it (+20–30% from 12000 rpm). It feels flat out of slow corners until ~11000 rpm;
- the exhaust opens 10° earlier, so it blows down in time at 14000 rpm;
- a stiffer spring in the plenum relief valve: about +1.2 bar at the top instead of +1.0;
- race fuel: less spark taken out for boost and for the hot charge;
- its own race exhaust: a full titanium 4-2-1 with heat-blued headers a size up (41 mm), no
  exhaust valve, and a short titanium can with a carbon end cap in place of the stock silencer.
  It's louder (~3 dB), with more crackle and a higher, shorter-pipe note;
- the full-throttle upshift moved to ~13800 rpm.

| | Stock | Tuned (sim) |
|---|---|---|
| Peak torque | 133 N·m at 10500 rpm | 142 N·m at 12000 rpm |
| Torque at 6000 / 8000 rpm | 96 / 117 N·m | 80 / 96 N·m |
| Peak power | 147.5 kW (198 hp) at 11000 rpm | 186 kW (250 hp) at 13500 rpm |
| 0–200 km/h | 7.0 s | 6.8 s |
| After 30 s in 6th | ~312 km/h | ~340 km/h |

The HUD shows TUNED after the rpm. `--tuned yes` works with `--phystest` (its own targets),
`--wav` and screenshots; `make test` checks both versions.

### `zx10r`: Kawasaki Ninja ZX-10R (2021-)

Built like the S 1000 RR: the inline-4's piston and rod scaled to the 76 mm bore (the H2 shares the
bore and stroke, without the supercharger), on a procedural flat-plane crank, with a green cam cover.

| | |
|---|---|
| Engine | 76.0 × 55.0 mm = 998 cc, 13.0:1, 107 mm rods (estimate), 180° flat-plane crank, block inclined 30° (estimate), forward-turning crank, four 47 mm throttle bodies, finger-follower valvetrain, one cam (no variable timing) |
| Firing | 1-2-4-3 |
| Output (sim) | 115 N·m at 11000 rpm, 149 kW (200 hp) at 13500 rpm, no ram air (Kawasaki: 114.9 N·m at 11400 / 149.3 kW at 13200), limiter 14000 rpm, idle 1250 rpm |
| Gearbox | Primary 79/47 (1.681); gears 39/15, 41/19, 32/17, 33/20, 31/21, 30/23; chain 41/17 (the 2021 bike's bigger rear sprocket); 190/55 ZR17 rear |
| Vehicle | 207 kg wet + 80 kg rider, CdA 0.31 |
| Rider aids | Same as the other bikes; launch control holds the clutch higher (~7500–9500 rpm) |
| Exhaust (3D) | The S 1000 RR's 4-2-1 layout into a bigger dark silencer |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 7.0 s, ~317 km/h after 30 s in 6th (on the limiter; the real bike is limited to 299) |
| Sound | The inline-4 superbike voice with a longer pipe, a lower note, more airbox honk (the ram-air box sits under the rider) and a little more gear whine than the BMW |

Its 1st gear is taller than the BMW's (10.5:1 overall against 11.8:1), so in the sim it peaks at
~1.05 g on a launch, short of the test's 1.15 g "front in the air" mark. It sets
`bikeSpec::liftsFront = false` for that check, and its wheelie control still cuts in. The same tall 1st makes a
25% take-off bog to ~640 rpm (`takeoffMinRpm = 600`).

#### Tuned (`--tuned yes`): race kit

```bash
./engine_sim --engine-type zx10r --tuned yes
```

A Superstock-style race build on the ZX-10RR's engine, the version Kawasaki homologates for racing.
It is no one team's engine, and every number beyond the RR's parts is an estimate. The block, throttle
bodies, gearbox and gearing stay stock. The changes:

- the RR's Pankl titanium rods and lighter pistons and pins (less reciprocating mass);
- Kawasaki's race kit ECU and kit cams: more lift and duration (intake closes 10° later, exhaust opens
  10° earlier), so the engine breathes best near 13000 rpm, with the limiter at 15500 rpm (stock 14000,
  RR 14700). The full-throttle upshift moves to ~15200 rpm;
- high-compression kit pistons (13.6:1) on race fuel, with more spark at the top. The mid range is no
  weaker than stock, unlike the road-going RR;
- a titanium race exhaust: the stock 4-2-1 route with heat-blued headers a size up (41 mm), no exhaust
  valve, and a short titanium can with a carbon end cap in place of the dark silencer. It's louder, with
  more crackle, a higher note and more intake honk from the open kit airbox.

| | Stock | Race kit (sim) |
|---|---|---|
| Peak torque | 115 N·m at 11000 rpm | 118 N·m at 12000 rpm |
| Torque at 6000 / 8000 rpm | 90 / 103 N·m | 93 / 104 N·m |
| Peak power | 149 kW (200 hp) at 13500 rpm | 171 kW (229 hp) at 15000–15500 rpm |
| Limiter | 14000 rpm | 15500 rpm |
| Top of 1st / 2nd / 3rd (from the gearing) | ~161 / 194 / 223 km/h | ~179 / 215 / 247 km/h |
| 0–100 km/h | 3.4 s | 3.4 s (the tall 1st and the wheelie limit cap it) |
| 0–200 km/h | 7.0 s | 6.8 s |
| After 30 s in 6th | ~317 km/h | ~334 km/h |

The HUD shows RACE KIT after the rpm.

### `mxking`: Yamaha MX King 150 (2015-2020)

The Indonesian underbone (Jupiter MX King, T150 engine), made from the engine template: a single
from the inline-4's scaled parts, the barrel laid 70° forward, a forward-turning crank.

| | |
|---|---|
| Engine | 57.0 × 58.7 mm = 149.7 cc, 10.4:1, SOHC four valves, liquid cooled, fuel injected, 98 mm rod (estimate) |
| Firing | Single, every 720° |
| Output (sim) | 13.9 N·m at 7000 rpm, 11.2 kW at 8000 rpm in the 1000 rpm sweep (the curve peaks ~8400) (Yamaha: 13.8 N·m at 7000 / 11.3 kW at 8500), limiter ~10300 rpm (estimate), idle 1400 rpm |
| Gearbox | Five speeds: primary 73/24 (3.042); gears 34/12, 30/16, 30/22, 24/21, 22/23; chain 40/14; 120/70-17 rear |
| Vehicle | 117 kg wet + 70 kg rider, CdA 0.42 |
| Rider aids | None on the real bike. The sim's bike drivetrain still gives it the quickshifter and the automatic; no slipper clutch, launch or wheelie control |
| Exhaust (3D) | One header out of the underside of the head, back under the crankcase into a black-shielded silencer on the right |
| Results (sim) | 0–100 km/h 13.6 s, ~117 km/h after 30 s in 5th |
| Sound | The superbike stages tuned for a single: one thump every other turn, a longer pipe, a small silencer with no exhaust valve, little crackle |

The heavier crank (0.012 kg·m², flywheel included) idles with a ~200 rpm ripple, pulls away from a
clutch dump at idle after nearly stalling (`bikeSpec::dumpMinRpm`), and needs up to ~400 ms to blip up
for the rev-matched 2 → 1 downshift. `--phystest` checks the 1st-order shake against a fixed limit
(`shakeMax1st`): the test measures only vertical force, which a counterweight on a laid-down single
adds to rather than cancels. It never reaches 200 km/h, so it sets `zeroTo200What = nullptr` to skip
that check. The superbike-sized gearbox drawing looks large next to this engine.

#### Tuned (`--tuned yes`): race build

```bash
./engine_sim --engine-type mxking --tuned yes
```

A typical Indonesian underbone road-race build of the T150 engine. It is no one team's engine, and
every number is an estimate. The changes:

- a bore-up to 177 cc: a 62 mm forged piston in a replated barrel on the stock crank and rod. It has
  its own mesh (`mxking_race.mesh`, written by `build_mxking.py` with the stock one), so the bigger
  piston shows. The cam cover is red;
- 13.2:1, a race cam with more lift and overlap (intake closes ~70° after BDC, exhaust opens ~65°
  before BDC), stiffer valve springs and a 13000 rpm limiter;
- a 32 mm throttle body, an open race filter and a race-fuel map;
- a lighter magneto rotor. It idles higher (1800 rpm) and lumpier, and needs a stronger starter for
  the compression;
- an open race pipe: a fatter, heat-blued header that swells into an expansion chamber under the
  crankcase, then a short steel can with a carbon end cap. It's louder, with more crackle and intake
  honk;
- a stripped bike (~100 kg) and a 60 kg rider tucked in, with race clutch springs. The gearbox and
  sprockets are stock.

| | Stock | Race (sim) |
|---|---|---|
| Engine | 57.0 × 58.7 mm, 149.7 cc, 10.4:1 | 62.0 × 58.7 mm, 177 cc, 13.2:1 |
| Peak torque | 13.9 N·m at 7000 rpm | 18.8 N·m at 11000 rpm |
| Peak power | 11.2 kW (15 hp) at 8000 rpm | 22.0 kW (29.5 hp) at 11500 rpm |
| Limiter | ~10300 rpm | 13000 rpm |
| Idle | 1400 rpm | 1800 rpm |
| 0–100 km/h | 13.6 s | 7.3 s |
| After 30 s in 5th | ~117 km/h | ~163 km/h |

It is peaky: below ~8000 rpm it makes less torque than the stock engine. The HUD shows RACE after
the rpm.

### Exhaust

Each engine draws its exhaust beside the block (not with `--engine-only`). The pipes are swept tubes
built from the engine's layout (`src/exhaust.hpp`), and they move with the engine on its mounts.

- **Heat.** Each cylinder's gas path heats toward the burned-gas temperature at exhaust valve opening,
  scaled by the gas flow (rpm), over a few seconds, and cools more slowly after a lift-off. It cools
  along the pipe, so the headers glow cherry red, then orange, at sustained full throttle while the
  silencer stays darker. The HUD shows the header temperature (a rough estimate).
- **Pulses.** When a cylinder's exhaust valve opens, a slug of hot gas runs down its runner and on
  through the merges at the gas flow speed. Slow motion (`[`) shows the firing order travelling down
  the pipes.
- **Exhaust valve** (bikes). A lever on the pipe, driven like the sound's flap: shut at idle and light
  load, open with revs or throttle. The HUD shows how far it is open.
- **Crackle** (bikes). Every pop the sound makes (overrun crackle, limiter, quickshifter bark) shoots a
  short flame out of the tailpipes. Some pops, and every bark, also throw a few sparks that arc down
  under gravity. They run on simulated time, so slow motion slows them too. The inline-4 makes no pops, so
  it has no flames.

### Turbocharger (`--turbo yes`, inline-4)

This section is the inline-4's optional turbo; the RS 3 (`rs3`) uses the same model, always on, with
its own sizing.

```bash
./engine_sim --turbo yes
```

The inline-4's exhaust then feeds a turbocharger (the Sketchfab "Car Turbocharger" model) hung beside
the block with its shaft along the car. The two header pairs merge above the turbine inlet, the
downpipe leaves the turbine forward and turns back under the turbo to the catalyst, and the
compressor draws through a cone filter and blows through a charge pipe into a front-mounted
intercooler ahead of the crank snout; the cooled charge comes back up the intake side.

- **See-through.** The compressor, centre and turbine housings, flanges, clamps and bolts are drawn
  like the engine block, so the compressor wheel, shaft, bearing and turbine wheel show through.
  `G` hides them with the block.
- **Boost.** The turbo is part of the engine physics (`engineSim::stepTurbo`, a mean-value model):
  - the exhaust gas fills the exhaust manifold and leaves through the turbine and the wastegate, so
    the back pressure on the exhaust stroke rises with the flow;
  - the turbine's power spins the shaft against the compressor's and the bearings', so it spools
    up with lag;
  - full pedal asks for 0.6 bar, and the ECU works the wastegate (PI) to hold it. Part pedal asks
    for less, and the throttle shuts as usual. An engine can also give its ECU a boost target that
    falls over the top of the rev range (`boostFallRpm`, `boostTopRpm`, `boostTop`) and a charge limit
    (`chargeMax`: the most air per cycle, so the torque plateaus where the turbo could give more), as
    the RS 3 does; both are off here;
  - the spark backs off 8° per bar of boost, and 0.1° per K of charge over 320 K, against knock;
    the clutch is made 1.5× stronger.
- **Compressor and surge.** The air in the compressor's inlet duct has inertia and is pushed by
  the compressor map (a Greitzer model): the pressure rise peaks at the surge line and falls away
  on both sides. In normal running the compressor sits right of the peak and is stable. Shut the
  throttle on boost without a way out for the charge, and the flow drops left of the peak: the
  compressor stalls, the charge pushes back through it, it recovers, and it stalls again, about
  20 times a second (the "flutter").
- **Blow-off valve.** A diaphragm valve on the charge pipe, opened by manifold vacuum: when the
  throttle shuts on boost it vents the charge to the air, so the compressor keeps flowing and does
  not surge. `F` takes it off and puts it back while driving; `--bov no` starts without it. With
  the valve the compressor keeps working against the open valve, so the shaft actually slows a
  little faster than in surge (25k vs 42k rpm one second after a lift-off at 4000 rpm).
- **Intercooler.** The compressor heats the air (to ~90 °C at full boost). The charge pipes and
  intercooler are a 5 L volume between the compressor and the throttle, with a pressure drop
  across the core. The core takes out half of the compressor's heat standing still and 80% from
  30 m/s of air through it, and soaks heat over a few seconds. The cylinders fill with fewer
  kilograms of the warmer charge.
- **Sound.** On top of the engine's voice: the compressor whistle (its 6 blades and 6 splitters
  at the shaft speed, louder with the air it moves), the rush of air through it, the blow-off
  valve's hiss on each lift-off, and the surge flutter's chuffs without it. `SND_TURBO` scales them
  when rendering a WAV. `--turbo yes --wav out.wav` drives the car through two full-throttle pulls
  with lift-offs (add `--bov no` to hear the flutter).
- **What it does (sim).** Boost comes in at ~3000 rpm and is held at 0.58 bar to the limiter:
  365 N·m at 4000–5000 rpm, 226 kW at 7000 rpm (223 N·m / 131 kW without). Floored at 3000 rpm, it
  reaches 90% boost in 1.0 s. 0–100 km/h takes 4.7 s (6.8 s without); the car model has no
  wheelspin. The HUD shows the boost, wastegate, back pressure, charge and compressor-outlet
  temperatures, the blow-off valve and surge, and the shaft speed (~105–125k rpm on boost). The
  wheels are drawn turning at a readable speed in proportion to the shaft, since the real speed
  would just strobe. `./engine_sim --turbo yes --phystest` checks all this.

`engines/inline4_fit/build_turbo.py` splits the model (`engines/inline4_fit/turbocharger/`) into
housing, rotor and bearing by geometry and writes `turbo.mesh`. `make` builds it when the model is
there.

**Estimates.** These values are not published: the V4 rod length; the S 1000 RR bore spacing (92 mm);
the H2's rod length, bore spacing (84 mm), cylinder incline and supercharger compressor map; the same
three for the ZX-10R; the RS 3's bore spacing (88 mm), turbo map and its friction, inertia and cam timing;
and the crank inertia, reciprocating mass, friction and cam timing of both bikes. They are estimates,
calibrated so that the published torque and power come out of the physics.

## Controls

| Input | Action |
|---|---|
| Left-drag / scroll | Orbit / zoom |
| SPACE (hold) | Starter |
| W or Up (hold) | Throttle; `1`–`9`, `0` latch 10–90 %, 100 % |
| I | Ignition on/off |
| S (hold) | Brake. With the throttle too, at a standstill (automatic or manual DCT): launch control. The box lets the clutch carry only what the brakes hold, so the car stays put while the engine works against them at the launch rpm and a turbo builds boost; let go of S to launch. Stopped with the throttle shut, the brakes also hold the car on a hill (hill-hold) until the throttle opens, in automatic and manual DCT |
| T | Automatic / manual |
| Z | Automatic: sport mode (holds gears, rev-matched downshifts). Cars in manual: NORMAL / DCT. NORMAL needs the clutch (C) for every change and to pull away, and stalls if you stop in gear without it. DCT works like a dual-clutch box's manual mode: you pick the gears with E/Q and the box works the clutch: take-off (with launch control), each change in a few hundredths of a second (blipping to the lower gear's speed on a downshift, refusing one that would over-rev), anti-stall (the clutch opens near idle and it drops to 1st as you stop), hill-hold, and launch control with the brake (S). The HUD shows MANUAL or MANUAL DCT |
| E / Q | Shift up / down. Car: Q at a stop selects R. Bikes: the quickshifter works while moving, no clutch needed. Cars in manual DCT mode: no clutch needed either. |
| N | Neutral (bikes) |
| C (hold) | Clutch |
| V | Wheelie control on/off (bikes) |
| H / J | Hill up / down |
| L | Cargo (car) / passenger (bikes) |
| F | Blow-off valve on/off (`--turbo yes`); bypass valve on/off (`h2`) |
| `[` / `]` | Slow motion / back toward real time (×1 … ×0.01) |
| `,` / `.` | When paused: step the crank by 1° |
| G / B / M / D / X | See-through block / motion blur / mute / full-throttle dyno chart / shake view (×1, ×20, ×100) |
| P / R / Esc | Pause / reset / quit |

## Command-line options

| Option | |
|---|---|
| `--engine-type T` (or `--engine-type=T`) | `inline4` (default), `rs3`, `v4`, `s1000rr`, `h2`, `zx10r`, `mxking` or an alias; `list` prints them, `list --ids` one runnable configuration per line (for scripts). `--v4` and `--s1000rr` still work. |
| `--engine-only` | No gearbox or vehicle (a bike engine runs on a stand) |
| `--selftest` | Kinematics checks: parts stay joined, strokes, clearances, firing lags vs. crank geometry |
| `--phystest` | Physics checks against the engine's targets (start, idle, limiter, dyno curve, firing order, balance, acceleration, shifting, quickshifter rev-match, …) |
| `--wav out.wav` | Render a scripted run to a WAV file. The bikes play a ride: start, blips, limiter, launch through the gears, braking with rev-matched downshifts. The inline-4 plays a start / rev / lift-off script; the RS 3 (and the inline-4 with `--turbo yes`) two full-throttle pulls in the car with lift-offs. |
| `--screenshot out.png [--frames n]` | Save a frame and quit. Framing: `--yaw r --pitch r --zoom s`. State: `--crank deg`, `--simtime s --start --throttle x`, `--stopat deg` |
| `--nohud --noblur --mute --ghost 0\|1` | Options for regression shots |
| `--grade pct --reverse --sport --manual --dct --no-wheelie` | Starting conditions (`--manual`: start in manual, in neutral; `--dct`: a car's manual mode is DCT) |
| `--turbo yes\|no` (or `--turbo=yes`) | Fit the turbocharger to the inline-4 (default `no`; see below; the RS 3 always has its own). With `--phystest`, runs the turbo checks; with `--wav`, plays the turbo drive |
| `--bov yes\|no` | With the turbo: fit the blow-off valve (default `yes`). `no` lets the compressor surge on lift-offs |
| `--variant name` | Another version of the engine (`rs3`, `v4`, `s1000rr`, `h2`, `zx10r` and `mxking` have one: `tuned`) |
| `--tuned yes\|no` | Same as `--variant tuned` (default `no`) |
| `--compare a.png b.png` | Pixel diff of two screenshots |
| `--synthtest N` | Hidden: test an N-cylinder inline engine built in memory (see [Testing](#testing)) |
| `--dump-params` | Hidden: print every parameter of the selected engine, one `name = value` per line |

## Project layout

```
engine_sim.cpp              entry point: command line, window, event loop
src/
  common.hpp                includes, vec3/mat4, orbit camera, cylArray (per-cylinder storage)
  model.hpp                 mesh loading (ENGMSHv3, still v1/v2), slider-crank kinematics, --selftest
  render.hpp                OpenGL 2.1 renderer, combustion glow, motion blur, gear meshes
  exhaust.hpp               exhaust systems (3D): swept tubes from each engine's layout, heat, gas pulses, valve, crackle fire
  exhaust_layouts.hpp       layouts more than one engine uses (the inline superbike 4-2-1)
  turbo.hpp                 the turbocharger (--turbo yes): see-through housings, spinning rotor, spool
  physics.hpp               engine model, ECU, drivetrain, vehicle (parameter structs + engineSim)
  presets.hpp               starting points for a class of engine (the superbike engine and drivetrain)
  engine_type.hpp           engineType: what an engine provides; bikeSpec / carSpec (test targets); REGISTER_ENGINE
  engines.hpp               the registered engines for --engine-type, setupSim()
  calibration.hpp           VE_*, FMEP_*, IDLE_*, SND_*... overrides for calibrating (every --phystest, --wav)
  hud.hpp                   the HUD and help line, built from what the engine has
  sound.hpp, wav.hpp        sound synthesis + audio stream, --wav rendering
  dyno.hpp                  the WOT dyno curve (D), measured in the background
  bike_gearbox.hpp          the bikes' sequential gearbox (3D)
  test_bike.hpp             --phystest for the bikes, checked against each engine's bikeSpec
  test_car.hpp              --phystest for the cars, checked against each car's carSpec
  test_synth.hpp            --synthtest N
  dump_params.hpp           --dump-params
engines/
  all.hpp                   generated by the Makefile: includes every engine's header (not in git)
  <type>/engine.mk          each engine's part of the Makefile: its header, meshes and mesh rule
  inline4_fit/              inline4_fit.hpp (engine + Honda Fit gearing + car spec + type), gearbox_manual.hpp
                            (3D car gearbox, single or dual clutch), test_inline4.hpp (--turbo yes test), extract_parts.py, scene.gltf/.bin, license.txt,
                            engine.mesh, tests/ (pixel baselines), build_turbo.py, turbocharger/
                            (turbo model), turbo.mesh
  audi_rs3/                 audi_rs3.hpp (engine, turbo, sound, S tronic gearing, exhaust, car spec, type),
                            build_rs3.py, rs3.mesh, build_rs3_turbo.py, rs3_turbo.mesh (procedural turbo)
  ducati_v4/                ducati_v4.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_v4.py, v4.mesh
  bmw_s1000rr/              bmw_s1000rr.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_s1000rr.py, s1000rr.mesh
  kawasaki_h2/              kawasaki_h2.hpp (engine, supercharger, sound, gearing, test targets, demo ride,
                            type), build_h2.py, h2.mesh, h2_supercharger.mesh
  kawasaki_zx10r/            kawasaki_zx10r.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_zx10r.py, zx10r.mesh
  yamaha_mxking/            mxking.hpp (engine, sound, gearing, test targets, type, race version),
                            build_mxking.py, mxking.mesh, mxking_race.mesh
  _template/                a new engine to copy (not built): myengine.hpp, build_myengine.py, engine.mk
tools/meshlib.py            the mesh builders' shared steps and the ENGMSHv3 writer
scripts/
  fetch_assets.py           downloads the large model files (make assets)
  regress.sh, pngdiff.py    the behaviour regression check (see Testing)
plan.md                     development log (design notes and measurements, phase by phase)
```

The C++ builds as a single translation unit: `engine_sim.cpp` includes the headers, and the engines
through `engines/all.hpp`.

**Meshes.** `extract_parts.py` splits the glTF into moving groups (crank, pistons, rods, static parts,
block) and measures the pivots from the geometry. The other engines' builders reuse the inline-4's
piston and rod from that mesh, scaled to their bore, stroke and rod length, and add a procedural
crank, crankcase, cylinders and head (`tools/meshlib.py`: `inline_engine` for an inline engine of any
cylinder count, `Donor`, `build_crank`, `build_block` and `write_engine` for other layouts).

**Coordinates.** Mesh units are mm, and the crank axis is Z. The V4's front is −X and its crank
counter-rotates. The S 1000 RR's front is +X and its crank turns forward, so its gearbox is mirrored to
sit behind the crank.

### Mesh format (ENGMSHv3)

Every builder writes ENGMSHv3 through `tools/meshlib.py` `write_engine`; the full layout is documented
at the top of `src/model.hpp` and in `meshlib.py`. In short: the root matrix, the materials, the crank
axis, then **any number of cylinders** with their geometry (pivots, throw, rod length, bank angle, deck,
bore) and **firing lag** (degrees of the 720° cycle behind cylinder 1), a block of **named values**
(`banked`, `zPrimary` where the bike gearbox's primary gear sits on the crank, `supercharger_at`...)
and the draw groups (crank, pistons, rods, block, static parts). The mesh is the single source for
what Python and C++ share: the simulator takes the firing order (shown on the HUD as e.g.
`1-2-4-3, 180 deg apart`), the cylinder count and the named values from it, and `--selftest` checks
the mesh's firing intervals against the engine's published `bikeSpec::pattern`. Older files
(`I4ENGv1`, `ENGMSHv2`, `BMWS1KRR`) still load.

## Adding an engine

1. **Folder.** Copy `engines/_template/` to `engines/<name>/` and replace `myengine` with `<name>` and
   `_template` with the folder name, in the file names and inside the files:

   ```bash
   cp -R engines/_template engines/mybike && cd engines/mybike && for f in *myengine*; do mv "$f" "${f/myengine/mybike}"; done && sed -i '' 's/myengine/mybike/g; s/_template/mybike/g' *
   ```

   As copied, it is a working but uncalibrated 748 cc inline-3 bike: `make` builds its mesh and it
   passes `--selftest` and `--phystest` against placeholder targets.
2. **Builder.** In `build_<name>.py` set the bore, stroke, rod length, bore spacing, bank incline and
   firing order (any cylinder count). For another layout, start from `build_v4.py` (a V) or `build_h2.py`
   (extra parts). Run `./engine_sim --engine-type <name> --selftest` after each change.
3. **Header.** In `<name>.hpp`:
   - `engineParams`: bore (as in the builder), compression, cam timing, VE curve, limiter, inertia,
     idle, and `sound` for the voice. A bike starts from `superbikeBaseParams()` (`src/presets.hpp`);
   - `drivetrainParams`: gearing (any number of gears, from the tooth counts) and vehicle;
   - `bikeSpec` (bikes, `physicsTestBike`) or `carSpec` (cars, `physicsTestCar` in `src/test_car.hpp`):
     the `--phystest` targets. A car's test checks the balance and the firing order against the
     engine's own geometry (no targets needed), runs the turbo checks when the engine is boosted, and
     drives automatic, manual NORMAL and DCT and reverse. Besides the published figures, every check's window has a
     field with a superbike default (idle, cranking, flare, coast-down, balance and firing-test speeds,
     0-100 / 0-200, the manual tests' shift rpm, `stallsOnClutchDump`, `liftsFront`, `shakeMax1st`,
     `downshiftMaxMs`...; see `src/engine_type.hpp`). Set the ones where the engine differs; the shift
     targets must be below the rev limiter (`carSpec` likewise: `src/engine_type.hpp`);
   - `exhaustLayout`: the pipes (control points in mesh space, radii, which cylinder or upstream pipes
     feed each one), optional flange boxes, the tailpipe outlets (for the crackle flames) and an exhaust
     valve. `cylPoint` and `cylDir` place the ports on the head in the cylinder's own frame. The
     template's works for any cylinder count;
   - the `engineType`: id, labels, mesh path, physics test, for bikes the gearbox placement and an
     optional `ride` for `--wav`; other versions of the engine go in `variants` (`--variant <name>`).
4. **Register.** The header ends with `REGISTER_ENGINE(<type function>, <key>)`; the key orders
   `--engine-type list` (inline4 10, rs3 15, v4 20, s1000rr 30, h2 40, zx10r 45, mxking 50), pick a free one. `engine.mk` lists the
   header and the mesh rule. Nothing outside the folder needs an edit: the Makefile includes every
   `engine.mk` and generates `engines/all.hpp`, `make test` picks the engine up, and the help line
   and HUD follow from what it has (bike or car, boost, wheelie control).
5. **Calibrate.** Put the published figures in `bikeSpec` or `carSpec` and run
   `./engine_sim --engine-type <name> --phystest` until it passes. Environment variables override
   parameters while you try values, for every engine: `VE_BASE VE_PEAK VE_RPM VE_WIDTH` (and `VEL_*`
   for a low-cam lobe), `FMEP_A FMEP_B FMEP_C`, `RESID ETA INERTIA STARTER`,
   `IDLE_KP IDLE_KI IDLE_SG IDLE_BASE IDLE_SPARK IDLE_MIN`, `SC_PR SC_FLOW SC_CHOKE SC_RELIEF`,
   `TURBO_AREA TURBO_WG TURBO_BOOST TURBO_INERTIA TURBO_KNOCK TURBO_FALL TURBO_TOPRPM TURBO_TOP
   TURBO_CHARGE` and, for
   `--wav`, `SND_*` (`src/calibration.hpp` lists them).

## Testing

- `make test` runs `--selftest` and `--phystest` for every engine and version of one
  (`./engine_sim --engine-type list --ids`), plus `--synthtest` for 1, 2, 3, 4, 5, 6 and 8 cylinders, and
  fails if any check fails.
- `./engine_sim --synthtest N` builds an N-cylinder inline engine in memory (the inline-4's dimensions,
  even firing, no mesh) and checks that it starts, idles, burns once per cylinder every 720° at
  720/N intervals, revs to the limiter, and never produces NaN or inf.
- `scripts/regress.sh baseline` then, after a change, `scripts/regress.sh check` proves a refactor
  changed nothing: for every engine and version (from `--engine-type list --ids`) it compares `--dump-params`, the full `--selftest`
  and `--phystest` output, the `--wav` render (sha256), screenshots (engine, scene, HUD, running, on a
  stand, with options, in manual and manual DCT) pixel by pixel, the meshes' sha256 and the engine
  list. A screenshot within `JITTER_PIXELS` pixels and `JITTER_LEVELS` levels (50, 8) passes as GPU
  jitter; a shot the baseline does not have yet is listed as NEW. Everything it prints is also saved
  in `regress/<baseline|check>/report.txt`. Output goes to `regress/` (not in git).

## Credits

The 3D model is ["Inline-4 Engine" by rafaelaraujo55](engines/inline4_fit/license.txt), licensed
CC-BY-4.0. The credit line is shown on screen. The V4, S 1000 RR, H2, ZX-10R, MX King and RS 3 are built from its parts.

The turbocharger is ["Car Turbocharger" by ml-design](engines/inline4_fit/turbocharger/license.txt),
licensed CC-BY-4.0; its credit line is shown on screen with `--turbo yes` and on the RS 3.

Published figures come from Ducati, BMW Motorrad (technical data and press material), Audi (RS 3 press
and technical data) and Honda's 2010 Fit specifications. This is a simulation, not manufacturer data.
