# Engine simulator

A real-time 3D engine simulator. The crank, rods and pistons move on exact slider-crank geometry, and
the crank speed comes from a physics model, not a fixed rate:

- a pressure model for each cylinder (intake, compression, Wiebe combustion, expansion, blowdown),
  with no two combustions alike: each firing's heat release and burn duration vary a few percent at
  random (3% at idle, 1.2% at full load: `engineParams::cycleCovIdle`, `cycleCovFull`), seeded so
  every run is the same;
- how well each engine breathes over its rev range, with the pressure waves in its intake runners and
  exhaust headers putting real humps and dips on the torque curve ([wave tuning](#wave-tuning));
- the gas and reciprocating-mass torques on the crank;
- friction, a starter motor and an ECU that controls the idle, the rev limiter and the fuel cut;
- a clutch, a gearbox and the vehicle the engine pushes.

The sound is synthesised from the same simulation, at the physics rate (40 kHz), and resampled to
the sound card's 44.1 kHz through a windowed-sinc filter. Each engine's level (`soundParams::drive`)
is linear and set so its loudest moments stay under full scale; a limiter near full scale is only a
safety net, so the waveform is not flattened.

In the window only, a playback leveller (`src/leveller.hpp`, K toggles it, on by default) rides the
gain of what goes to the sound card: quiet stretches (idle, cruising, a single's gaps between firings)
are compressed 2:1 upward toward -18 dBFS RMS (by up to +15 dB), while full throttle stays about
where it is and peaks stay near 0.70. The ratio keeps the loudness order: a 20 dB idle-to-load gap
becomes about 10 dB, never reversed. One gliding gain for both channels with a 6 ms lookahead, so it adds no harmonics.
It runs on the audio thread after resampling; `--wav`, `--stats` and the tests are upstream of it and
unchanged. The HUD's sound line shows its boost (`lvl +N dB`).

Small speakers (laptops, phones) play almost nothing below ~150 Hz, where a single's idle lives. O
turns on a speaker voicing (`src/speaker.hpp`, off by default, window only): a 110 Hz high-pass and a
+3 dB lift at 400 Hz, plus virtual bass: the band under 150 Hz is rectified into its 2nd, 4th...
harmonics, band-passed to 150-600 Hz and mixed in, so the ear fills in the fundamental it cannot hear.
With it on the leveller may boost up to +20 dB. Measured above 150 Hz (over the leveller alone): idle
+4-8 dB, load +2-5 dB. It adds harmonics the engine does not make, so leave it
off on headphones. The HUD shows `spk` when it is on.

Each cylinder's exhaust pulse is its gas escaping through the exhaust valves as they lift: compressible
flow through the valves' open area (choked at high pressure ratios), the cylinder emptying from the
pressure the combustion left, or, on the overrun, drawing the pipe's hot gas back in; the piston's push
on the exhaust stroke rides on top. How fast the valve opens against the pressure ratio sets how sharp
or deep each engine sounds. The intake roar follows the air's speed through the intake valves, peaking
as each opens and closes. (The inline-4's simpler voice keeps the original pulses.)

The exhaust you see is the exhaust you hear (every engine but the inline-4, whose simpler voice has no
pipe stage): each drawn pipe the gas reaches is a pair of delay lines as long as sound takes to cross
it, joined where the drawing joins them. So each cylinder's pulse reaches the merges after its own
runner, every section rings at its own length, area changes reflect part of each wave, a silencer
(a section that widens abruptly, or one tagged `pipeAcoustic::silencer`) damps and dulls what passes,
a catalyst (tagged) takes a little off the top, the turbine passes a dulled share (the turbulent rasp of
the gas flow goes out through the same silencers, catalysts and turbine, so straight pipes keep it and
silenced or turbo exhausts lose most of it), and the drawn
exhaust valve both opens a way past the silencer and switches the voice's valve stage (no valve drawn:
always open). The speed of sound follows the gas temperature: hot gas from combustion, cooled by the
blowdown and settling to the pipe walls' temperature, so a cold start rings lower and the pipe modes
climb as the pipes heat up. `--wav` prints what it found (pipes, silencers, catalysts, open ends,
turbine). `SND_LAYOUT=0` falls back to the single typed pipe (`soundParams::pipeLength`).

Everything plays in mono (the window, `--wav`, the tests). In the window, M switches to stereo, heard
from the camera (M cycles mono, stereo, muted). The sound is split
by where it comes from: the tailpipes (at the drawn outlets), the intake (above the heads), the
engine block (clatter, valvetrain, gear whine, starter) and the turbo or supercharger (where it is
drawn). Each reaches each ear after its distance at the speed of sound, panned by its direction, with
the ear facing away hearing it duller, and louder as the camera gets closer (up to +4 dB). The
tailpipe beams backward: orbit behind a car and the exhaust comes up, in front of it the intake and
the engine do; a bike's far side, away from its silencer, is quieter. From the starting view every
source plays at its mono level, so the balance there is the mono one.

It has eleven engines. `engine_sim` is the single entry point, and you pick the engine with
`--engine-type`:

| `--engine-type` | Engine | Drives | Also accepted |
|---|---|---|---|
| `inline4` (default) | 2.3 L inline-4 (the Sketchfab model); `--tuned yes`: turbocharged, straight pipe | a car with Honda Fit 5-speed gearing; tuned: a close-ratio 6-speed dual-clutch | `fit`, `honda-fit`, `inline4-fit`, `i4` |
| `rs3` | Audi RS 3 2.5 TFSI turbocharged inline-5, 2480 cc | an RS 3 (8Y) with the 7-speed S tronic | `audi`, `audi-rs3`, `rs-3`, `inline5`, `i5`, `2.5tfsi`, `daza` |
| `lfa` | Lexus LFA 1LR-GUE 72° V10, 4805 cc | an LFA with the 6-speed ASG | `lexus`, `lexus-lfa`, `v10`, `1lr-gue`, `1lr` |
| `mustang-v6` | 2014 Ford Mustang V6: 3.7 L Cyclone 60° V6, 3726 cc; `--tuned yes`: twin-screw supercharger, straight pipe and an ECU tune | a Mustang V6 with the MT82 6-speed manual; tuned: close-ratio gears on a 3.73 axle | `mustang`, `ford`, `ford-mustang`, `mustang-3.7`, `cyclone`, `duratec37`, `v6`, `3.7` |
| `ca2006` | Cosworth CA2006 2.4 L 90° Formula 1 V8, 2399 cc, 20,000 rpm | a Williams FW28 with the 7-speed seamless sequential box | `cosworth`, `cosworth-ca2006`, `ca`, `f1`, `f1-v8`, `fw28`, `williams-fw28` |
| `v4` | Ducati Desmosedici Stradale 90° V4, 1103 cc | a Panigale V4 (bike) | `ducati`, `ducati-v4`, `panigale` |
| `rsv4` | Aprilia RSV4 RR 65° V4, 999.6 cc; `--tuned yes`: Akrapovic Evolution titanium system, race map, 16/40 gearing | an RSV4 RR (bike) | `aprilia`, `aprilia-rsv4`, `rsv4rr`, `rsv4-rr`, `aprilia-rsv4rr`, `aprilia-rsv4-rr` |
| `s1000rr` | BMW S 1000 RR inline-4, 999 cc | an S 1000 RR (bike) | `bmw`, `s1000`, `bmw-s1000rr` |
| `h2` | Kawasaki Ninja H2 supercharged inline-4, 998 cc | a Ninja H2 (bike) | `kawasaki`, `kawasaki-h2`, `ninja-h2` |
| `zx10r` | Kawasaki Ninja ZX-10R inline-4, 998 cc | a ZX-10R (bike) | `zx-10r`, `ninja-zx10r`, `ninja-zx-10r`, `zx10`, `kawasaki-zx10r` |
| `f4rr` | MV Agusta F4 RR inline-4, 998 cc; `--tuned yes`: the F4 RC with its racing kit (212 hp) | an F4 RR (bike) | `mv-agusta`, `mvagusta`, `mv-agusta-f4`, `mv-f4`, `f4`, `f4-rr`, `mv-f4rr` |
| `mxking` | Yamaha MX King 150 single, 149.7 cc | an MX King underbone (bike) | `mx-king`, `jupiter-mx`, `t150`, `yamaha150` |

[stats.md](stats.md) compares every engine and its tuned version: power, torque, weight and one drag run
each (launch control, 0–100 km/h, ⅛ and ¼ mile, top speed after a minute flat out). It is generated:
`make stats` (`scripts/gen_stats.py`, which runs `engine_sim --drag-stats` for each).

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
optional: without it everything builds, and the tuned inline-4 and the RS 3 draw a procedural turbo
in its place.

```bash
./engine_sim --engine-type s1000rr
```

`make` builds `engine_sim` and any missing meshes. Run it from the project root, because it loads the
meshes and `SpaceMono-Regular.ttf` from there. Shortcuts: `make run` (inline-4), `make run ENGINE=<id>`
or `make run-<id>` (`make run-rs3`, `make run-lfa`, `make run-mustang-v6`, `make run-ca2006`, `make run-v4`, `make run-rsv4`, `make run-s1000rr`, `make run-h2`, `make run-zx10r`, `make run-f4rr`, `make run-mxking`; aliases work too). `make test` runs
the headless checks for every engine and version of one (`./engine_sim --engine-type list --ids`), and
fails if any of them fails.

## The engines

### `inline4`: 2.3 L inline-4 with Honda Fit gearing

The original model, split into moving parts from the glTF.

| | |
|---|---|
| Engine | 85 mm bore, 160 mm rods, 10.5:1, flat-plane crank, firing 1-3-4-2. Tuned to match a Honda K24Z3 (2008–12 Accord: 2.35 L, 177 hp at 6500 rpm, 218 N·m at 4300 rpm) |
| Stroke | The model's crank throws are uneven: 108.4 mm on cylinders 1/4 and 97.8 mm on 2/3 (~2.34 L). Each cylinder uses its own measured throw, so the parts stay joined. |
| Output (sim) | 226 N·m at 4500 rpm, 128 kW (172 hp) at 6500 rpm, idle 800 rpm, limiter 6800 rpm |
| Wave tuning | ~0.42 m runners (45 mm bore) ram at ~4300 rpm; the 4-2-1 header to its last merge, 0.57 m acoustically (measured on the drawn exhaust), scavenges weakly. A dip at 3000–3500 rpm, a hump at 4500–5000 (+3%) and a fall toward the limiter (−5%) (runner: estimate) |
| Gearbox | 2nd-gen (2009–2013) Honda Fit 5MT: 3.308 / 1.870 / 1.303 / 0.949 / 0.727, reverse 3.308, final drive 4.62 |
| Vehicle | 1300 kg car, 0.305 m tyres, CdA 0.65 |
| Drivetrain | Friction clutch, automatic mode (shift schedule, kickdown, downshift under load, sport mode with rev-matched downshifts) or manual in two modes (Z): NORMAL (clutch pedal for every change, stalls if you dump it at idle) and DCT (see the RS 3) |
| Results (sim) | 0–100 km/h in 6.9 s |
| 3D | The see-through block, and a procedural 5-speed gearbox with a reverse idler behind the flywheel (the car gearbox drawing moves back, `gearboxModel::shiftFor`, for an engine whose crank or crankcase reaches past the inline-4's) |
| Exhaust (3D) | Tubular 4-2-1 manifold (1+4, 2+3) on a cast flange, close-coupled catalyst, mid pipe, silencer and tailpipe running back beside the gearbox |

#### Tuned (`--tuned yes`): turbo, straight pipe

```bash
./engine_sim --tuned yes
```

The same engine with a turbocharger and a straight pipe: everything under
[Turbocharger](#turbocharger-the-tuned-inline-4) below (this was `--turbo yes` before). After the
turbine the exhaust is a straight pipe: no catalyst or silencer, the 3 in downpipe carried straight
back to a big burnt-titanium tip. The physics is unchanged by it (past the turbine the gas goes to the
air either way in the model, and the header's waves reflect at the merge over the turbine), so the
output is the turbo's: 366 N·m, 226 kW, 0–100 km/h in 4.5 s. The voice is louder and brighter with the
pipe's ring left in (~3 dB at idle and on lift-off, ~2 dB at full throttle); the turbine takes the edge
off the pulses, so it is a loud, raw drone rather than a scream. The clutch is 1.5× the stock one.

The gearbox is changed for racing: a close-ratio 6-speed dual-clutch in place of the Fit's 5-speed.

| | |
|---|---|
| Ratios | 3.308 / 1.870 / 1.391 / 1.039 / 0.776 / 0.580, final drive 4.62 (the Fit's 1st, 2nd and final drive). 2nd–6th step evenly (×1.34) |
| Power band | Full-throttle upshifts at 6400 rpm (98% of peak power, clear of the 6800 rpm cut, whose fuel cut would dump the boost). Every change from 2nd up lands at ~4800 rpm, on the torque plateau and on boost. 6th reaches its drag-limited 286 km/h at ~6660 rpm, on the power peak (the Fit's 5th held it on the limiter at 228 km/h) |
| Shifts | Power-on upshifts are seamless (`drivetrainParams::seamlessUp`), in automatic and manual DCT: the next gear is already selected on the other input shaft, so the oncoming clutch takes the drive at the engine's torque while the ECU cuts the engine, and the slip pulls the revs down to the new gear in ~0.06 s (~0.1 s for 1st to 2nd) with no gap in the drive and the boost held. Downshifts and pulling away open and close the clutch in 0.03 s and 0.08 s. Drawn as a dual-clutch box; the RS 3 shifts the same way |
| Results (sim) | 0–100 km/h 3.8 s with launch control, 0–200 km/h 14.9 s, ¼ mile 12.05 s; see [stats.md](stats.md). The ratios and shift point came from sweeping them with `--drag-stats` |

### `rs3`: Audi RS 3 (8Y, 2021-), 2.5 TFSI turbocharged inline-5

```bash
./engine_sim --engine-type rs3
```

The five-cylinder, built like the superbikes from the inline-4's piston and rod (scaled to the 82.5 mm
bore and the 144 mm rod) on a procedural crank with its pins 72° apart, with a red RS cam cover. It is
the only engine that is always turbocharged (`engineType::boost = builtInBoost::turbo`; the H2 is
`supercharger`; the tuned inline-4 is turbocharged too): the turbo is always on and is drawn with the
inline-4's turbo model, or a procedural one when that is not there (below).

| | |
|---|---|
| Engine | 82.5 × 92.8 mm = 2480 cc, 10.0:1, 144 mm rods, five cylinders 88 mm apart (estimate), the bank upright, 74 mm throttle |
| Firing | 1-2-4-5-3, every 144°. No free 1st- or 2nd-order force; the rocking couples the test checks against the textbook (0.449 and 4.98 × the bore pitch) |
| Turbo | One turbo: a small turbine nozzle so it spools from ~2000 rpm, a big wastegate, a high-pressure-ratio compressor with a wide map (its surge line well left of the engine's flow at 2000 rpm, no choke at 7000; the test checks it never surges at full throttle). Blow-off valve, front-mounted intercooler |
| ECU boost | Up to 1.5 bar, but a charge (torque) limit holds the 500 N·m plateau with ~1.2–1.4 bar, and the target falls from 5500 rpm to ~1.1 bar at 7000 for the flat top-end power (`turboParams::chargeMax`, `boostFallRpm`…) |
| Output (sim) | 492 N·m at 2000, 500–507 N·m at 2500–4500, 485 at 5500; 299–301 kW (~403 hp) at 6000–6500 and 286 kW at 7000 rpm (Audi: 500 N·m at 2250–5600, 294 kW / 400 PS at 5600–7000), idle 770 rpm, limiter 7100 rpm, peak cylinder pressure 150 bar |
| Wave tuning | Weak on a turbo engine: ~0.30 m runners from the charge-air plenum ram at ~5800 rpm, and the short manifold primaries end at the merge over the turbine, 0.25 m acoustically (measured on the drawn exhaust), which reflects little. The charge limit takes the ripple out of the 500 N·m plateau, so it shows only off boost (−1 to −2% below 2000 rpm) and as a trace at the top (runner: estimate) |
| Gearbox | 7-speed S tronic (DQ500 dual clutch): 3.563 / 2.526 / 1.679 / 1.022 / 0.788 / 0.761 / 0.635, reverse 2.789, final drives 4.059 (1st, 4th, 5th, R) and 3.450 (2nd, 3rd, 6th, 7th): Audi's 8V figures, the same box. The sim has one final drive (3.450), so 1st, 4th, 5th and reverse carry the 4.059 / 3.450 step in their ratio. Power-on upshifts are seamless (`drivetrainParams::seamlessUp`): the oncoming clutch takes the drive at the engine's torque while the ECU cuts the engine, and the slip pulls the revs down to the new gear with no gap in the drive. Downshifts and pulling away take 0.03 s out and 0.08 s in. Shifts the same in manual's DCT mode (any car: T for manual, then Z). Drawn as a dual-clutch box (`drivetrainParams::dualClutch`): two wet clutch packs nested in a crank-driven drum, the outer one for the odd gears, the inner one for the even gears and reverse, on two coaxial input shafts; the pack carrying the drive closes (its apply plate brass), and a shift hands over from one to the other |
| Vehicle | 1570 kg + 75 kg driver, 265/30 R19 (0.325 m), CdA 0.72, quattro (the car model has no tyre grip limit, so all-wheel drive needs nothing extra) |
| Exhaust (3D) | Five cast runners into a manifold over the turbine, the downpipe back under the car, catalyst, mid pipe with the sport exhaust's flap, rear silencer and two big tailpipes. Intake: airbox behind the turbo, charge pipe to a front-mounted intercooler, back up the other side to the throttle and a plenum along the head |
| Results (sim) | 0–100 km/h 3.7 s with launch control (S and full throttle, then let go of S: held at ~4500 rpm on 1.2 bar; Audi: 3.8 s), 4.3 s floored from idle (the turbo spools on the way), 0–200 km/h 14.0 s, full-throttle upshifts at ~7000 rpm |
| Sound | The detailed voice (`soundParams::detailed`, the bikes' stages) on a car: a long pipe for the deep five-cylinder note and its idle burble, the sport exhaust's flaps opening under throttle or from ~5000 rpm (shut while coasting down), overrun crackle (with flames), the turbo's whistle and the blow-off valve |

Without the downloaded turbo model (the optional `Assets-2` download) the RS 3 (and the tuned
inline-4) draws a procedural turbo instead (`build_rs3_turbo.py` ->
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
| Engine | Forged rods and pistons (same dimensions): limiter 7400 rpm. 80 mm throttle and intake, bigger intercooler, catless downpipe and a straight pipe (no silencer, no flap) |
| Output (sim) | 613 N·m at 3500, ~600 N·m to 5500, 417 kW (559 hp) at 7000 rpm (stock: 507 N·m, 300 kW); peak cylinder pressure 179 bar |
| Gearbox | Stock ratios; a gearbox tune: more clutch clamping (950 N·m), full-throttle upshifts at ~7300 rpm, launch control a little higher (~4900 rpm on 1.5 bar) |
| Results (sim) | 0–100 km/h 3.0 s with launch control (3.6 s floored from idle), 0–200 km/h 10.8 s |
| 3D | Straight pipe: the downpipe and the mid pipe a size up, straight pipes where the catalyst and the rear silencer were, no flap, bigger blued titanium tips. The physics is unchanged by it (the header's waves reflect at the manifold's merge over the turbine, upstream) |
| Sound | Straight pipe: always open (no flap), the pipe rings harder; louder (~9 dB at idle, ~4 dB at full throttle, which stays the loudest, 1–3 dB on lift-off, ~7 dB with the ignition off), more crackle, more turbo |


### `lfa`: Lexus LFA (2010-2012), 1LR-GUE 72° V10

```bash
./engine_sim --engine-type lfa
```

A V like the Ducati (the inline-4's piston and rod scaled to the 88 mm bore, on a procedural crank,
crankcase, two banks, heads and red cam covers) in a car like the RS 3. Five crankpins 144° apart,
each shared by one rod from each bank: each bank fires as an even inline-5, and the 72° V puts the
other bank's cylinder half way between, so the ten fire evenly every 72°.

| | |
|---|---|
| Engine | 88 × 79 mm = 4805 cc, 12.0:1, 72° V, banks 36° either side of vertical, five shared crankpins 104 mm apart (estimate), 136 mm rods (estimate), ten individual throttle bodies. Cylinders 1-5 are bank A (−x), 6-10 bank B (+x), n+5 sharing n's pin |
| Firing | 1-6-5-10-2-7-3-8-4-9, every 72° (the usual V10 order; with shared pins each pin's two cylinders fire 72° apart). No free 1st- or 2nd-order force (each bank is an even inline-5), only rocking couples; the test checks them against the textbook, with each bore's vertical share for the V |
| Output (sim) | 480 N·m at 6500–7000 rpm, 408 kW (547 hp) at 9000 (Toyota: 480 N·m at 6800, 412 kW / 560 PS at 8700, 90% of the torque from 3700 to 9000); idle 1000 rpm, limiter 9000, idle to the limiter in 0.42 s in neutral (Toyota: 0.6 s) |
| Wave tuning | ~0.33 m from the surge tank through each throttle body ram at ~7000 rpm; the equal-length 5-into-1 headers reflect at the catalysts (measured on the drawn exhaust). A dip at ~3000 rpm and a sag of 2–3% at 7500–8500 (runner: estimate) |
| Gearbox | 6-speed ASG (Aisin automated single-clutch manual, rear transaxle): 3.231 / 2.188 / 1.609 / 1.233 / 0.970 / 0.795, final drive 4.556, reverse 3.25 (estimate). 6th at the limiter is ~323 km/h, the LFA's 325 km/h top speed. Shifts 0.05 s out and 0.10 s in (the ASG at its quickest), drawn as a single-clutch box |
| Vehicle | 1480 kg + 75 kg driver, 305/30 ZR20 rears (0.345 m), CdA 0.59 (Cd 0.31), rear-wheel drive |
| Exhaust (3D) | Exhaust ports on the outer sides of the heads: each bank's equal-length 5-into-1 header drops down the block to a collector and a catalyst, two titanium mid pipes (the left one with the exhaust valve) into a transverse silencer, and the LFA's three tailpipes in a triangle in the middle of the rear. Intake: a carbon surge tank over the V, ten runners with their throttle bodies into the valley-side ports, the inlet duct from an airbox ahead of the engine |
| Results (sim) | 0–100 km/h 3.6 s floored (Lexus: 3.7 s), 3.2 s with launch control (optimistic: the simulator has no tyre grip limit), 0–200 km/h 9.9 s, full-throttle upshifts at ~8900 rpm |
| Sound | The detailed voice on a car: the drawn exhaust, whose two banks of five ring apart until they meet in the silencer (the full firing rate is 10 × rpm / 120, 750 Hz at 9000 rpm), the intake howl, the exhaust valve opening from ~3700 rpm, a light crackle on the overrun. The turbulent rasp is turned down to an eighth (`soundParams::raspGain`): it follows the summed exhaust flow squared, and with ten cylinders' exhaust strokes overlapping (about three at once) the flow hardly drops between pulses, so at full strength the roar was ~7 dB over the firing note |

The light crank (0.14 kg·m²) against a 4.8 L's pumping losses stops quicker than a road car's from
3000 rpm with the ignition off, so the LFA sets its own coast-down window (`carSpec::coastLo`/`coastHi`).
#### Tuned (`--tuned yes`): straight pipe

```bash
./engine_sim --engine-type lfa --tuned yes
```

A typical straight-pipe build with an ECU tune (estimates, not one tuner's car) on the stock 88 × 79 mm,
4805 cc, 12.0:1 and internals.

| | |
|---|---|
| Exhaust | Catalysts, silencer and exhaust valve removed: the stock headers and collectors, the two banks crossed in an X-pipe under the gearbox, two straight titanium pipes to two big blued tips. Almost no back pressure (1.025 bar). The wave tuning is unchanged: each bank's pulses still reflect at its 5-into-1 collector (0.78 m acoustically, measured on the drawn exhaust) |
| ECU | Limiter 9500 rpm (stock 9000), a little more advance at the top; the VE curve peaks higher up |
| Output (sim) | 479 N·m at 7000 rpm, 425 kW (570 hp) at 9500 (stock: 480 N·m, 408 kW at 9000) |
| Gearbox | Stock ratios; full-throttle upshifts at ~9400 rpm |
| Results (sim) | 0–100 km/h 3.6 s floored, 3.2 s with launch control (no grip limit), 0–200 km/h 9.7 s |
| Sound | No silencer or valve: always open, the pipe rings harder (more reflection at the tips), brighter and louder (about the same at idle, +4 dB at full throttle, +2 dB on lift-off), more crackle on the overrun; the low burble held under the stock car's so the decel boom stays below full throttle, and the rasp at the stock eighth |

### `mustang-v6`: Ford Mustang V6 (2014), 3.7 L Cyclone 60° V6

```bash
./engine_sim --engine-type mustang-v6
```

A V built like the LFA (the inline-4's piston and rod scaled to the 95.5 mm bore, a procedural
crankcase, two banks, heads and black cam covers) in a front-engined, rear-drive car. A 60° V6 fires
evenly only with a split-pin crank: each of the three pairs carries one rod per bank on its own half
pin, the two halves 60° apart and joined by a thin flying web, so the builder makes its own crank
(four main bearings) instead of meshlib's `build_crank`.

| | |
|---|---|
| Engine | 95.5 × 86.7 mm = 3726 cc, 10.5:1, 60° V, banks 30° either side of vertical, split crankpins 60° apart, pairs 108 mm apart (estimate), 150 mm rods (estimate), DOHC 24 valves with Ti-VCT, port injection, ~70 mm throttle. Ford numbering: 1-3 the right bank (−x, forward), 4-6 the left bank (+x) |
| Firing | 1-4-2-5-3-6, every 120°. No free 1st- or 2nd-order force, only rocking couples (the test checks them against the textbook) |
| Output (sim) | 389 N·m at 4500 rpm, 225 kW (302 hp) at 6500 (Ford: 380 N·m / 280 lb·ft at 4250, 227 kW / 305 hp at 6500); idle ~720 rpm, limiter 7000, idle to the limiter in 0.61 s in neutral |
| Wave tuning | ~0.42 m composite-manifold runners ram in the mid range; short manifolds into close-coupled catalysts give little scavenging (estimates) |
| Gearbox | Getrag MT82 6-speed manual: 3.66 / 2.43 / 1.69 / 1.32 / 1.00 / 0.65, reverse 3.42 (estimate), 3.31 axle. Driven by the simulator's automatic clutch like every car, with a manual's slower changes (0.08 s out, 0.22 s in) |
| Vehicle | 1600 kg + 75 kg driver, P235/55 R17 (0.345 m), CdA 0.77 (Cd 0.35), rear-wheel drive |
| Exhaust (3D) | Exhaust ports on the outer sides: each bank's short 3-into-1 manifold runs back along the head to a close-coupled catalyst, two mid pipes back under the gearbox joined by an H-pipe, two rear mufflers and a chrome tip at each rear corner. No exhaust valve. Intake: the composite plenum over the V with a runner into each valley-side port, the throttle body at its front and the duct forward to the airbox |
| Results (sim) | 0–100 km/h 5.8 s (road tests: 0–60 mph 5.4–5.9 s with the manual), 0–200 km/h 19.8 s, full-throttle upshifts at ~6850–6950 rpm |
| Sound | The detailed voice: three firings a revolution (350 Hz at 7000 rpm) through true dual exhaust that meets only at the H-pipe, a hollow intake moan, a mild burble and the odd crackle on the overrun |

The idle air floor (`idleMin`) sits close to what the idle needs, so braking hard to a stop in 2nd
(sport) does not pull the revs under 500 rpm before the clutch opens.

#### Tuned (`--tuned yes`): twin-screw supercharger, straight pipe

```bash
./engine_sim --engine-type mustang-v6 --tuned yes
```

What a typical blown, straight-piped 3.7 Mustang does (estimates, not one tuner's kit), on the stock
engine, with the gearing changed for racing:

- **Supercharger:** a twin-screw blower on top of the V, about 1.5 L per turn of the male rotor,
  belted from the crank at 1.9:1 (~14k rotor rpm at the limiter). It sits on an intercooled lower
  manifold (an air-to-water core) in place of the composite plenum, fed through a throttle body on its
  rear inlet. A bypass valve, opened by manifold vacuum, lets it idle against no pressure off throttle
  (F toggles it, like a turbo's blow-off valve).
- **Exhaust:** the catalysts and both mufflers out, the H-pipe swapped for an X-pipe, two 2.5 in pipes
  straight back to two big burnt tips.
- **ECU:** more advance off boost and 6° per bar taken out with it, the cams re-phased for the top
  end, an 800 rpm idle, the limiter from 7000 to 7400 rpm, full-throttle changes at ~7000-7300 rpm.

The blower is positive displacement (`turboParams::screw`, `src/physics.hpp`): each turn of the male
rotor pushes `screwDisp` of inlet air into the charge pipes whatever the pressure, less what leaks back
through the rotor clearances (`screwLeak`, a nozzle driven by the pressure rise). Unlike a turbo or the
H2's centrifugal impeller, the boost is there from just off idle and holds nearly flat to the limiter;
the pulley ratio (`driveRatio`) sets how much. The crank pays for the compression through the belt.
`SC_DRIVE`, `SC_DISP` and `SC_LEAK` override the three while calibrating.

| | |
|---|---|
| Output (sim) | ~0.2 bar by 2000 rpm, 0.2–0.3 bar to the limiter; 488 N·m at 5000 rpm (391 N·m already at 2000), 341 kW (458 hp) at 7000 (stock: 389 N·m, 225 kW) |
| Gearing | Close-ratio MT82: 3.66 / 2.71 / 2.00 / 1.48 / 1.10 / 0.81 on a 3.73 axle (the stock 1st; the usual drag swap from the 3.31). Every step is ×1.35 |
| Power band | Full-throttle upshifts at 7000 rpm (98% of peak power, clear of the 7400 cut, which the manual's ~0.3 s change would flare into from 7200). Every change lands at ~5200 rpm, on the 485 N·m torque peak (stock ratios dropped to ~4800–5000 rpm after 1st and 2nd). 6th reaches its drag-limited 310 km/h at ~7190 rpm, on the power peak (on the stock ratios 5th held it on the limiter at 286 km/h, and 6th was a 4400 rpm overdrive) |
| Results (sim) | 0–100 km/h 4.6 s floored from idle, 4.1 s with launch control, 0–200 km/h 13.5 s, ¼ mile 12.15 s; see [stats.md](stats.md). The ratios, axle and shift point came from sweeping them with `--drag-stats` |
| Drawn | `mustang_v6_blower.mesh` (built with the engine by `build_mustang_v6.py`, placed at the mesh's `supercharger_at`): a see-through figure-8 rotor case, the 4-lobe male rotor with its pulley and the 6-lobe female rotor of the other hand, each turning about its own axis (`turboModel::ROTOR_A`, `ROTOR_B`), the female the other way at 4/6 speed, and the belt from the crank damper |
| Sound | The straight pipe as before, plus the blower's whine: the male rotor's lobe pass (4 × rotor speed, ~900 Hz at the limiter) and its harmonics, ~12 dB under the exhaust at full throttle and growing with the boost. The bypass's steady recirculation is near silent; only its snap open is heard |

### `ca2006`: Cosworth CA2006 (2006), 2.4 L Formula 1 V8, in the Williams FW28

```bash
./engine_sim --engine-type ca2006
```

Built like the LFA (the inline-4's piston and rod scaled to the 98 mm bore, on a procedural crank,
crankcase, two banks, heads and blue cam covers), on a flat-plane crank: four pins, the outer two at
0° and the inner two at 180°, each shared by one rod from each bank. Each bank is an inline-4 firing
every 180°, and the 90° V puts the other bank half way between, so the eight fire evenly every 90°.
Cosworth's figures come from Race Engine Technology (issues 20, 21 and 73), as collected in
[Grand Prix Engines Note 108](https://www.grandprixengines.co.uk/Note_108.pdf); the rest are estimates.

| | |
|---|---|
| Engine | 98 × 39.75 mm = 2399 cc (the 2006 rules: a 90° V8 of at most 2.4 L with a bore of at most 98 mm), 13.3:1, rods 2.570 × the stroke = 102.2 mm, 41.3 mm inlet valves with 16 mm lift, pneumatic valve springs, a fixed inlet (the rules), eight throttle barrels under a carbon airbox, dry sump. Bore spacing 106 mm (estimate). Cylinders 1-4 are bank A (−x), 5-8 bank B (+x), n+4 sharing n's pin |
| Firing | 1-5-3-7-4-8-2-6, every 90° (the usual flat-plane V8 order: estimate). No free 1st- or 2nd-order force: the 1st-order forces cancel in each bank and the two banks' 2nd-order forces add up sideways, which the vertical check does not see. There is a small 2nd-order rocking couple, and the test checks it against the textbook |
| Output (sim) | 294 N·m at 18,000 rpm, 560 kW (752 hp) at 19,000. Cosworth: 290.7 N·m at 17,000, 563 kW / 755 bhp at 19,250, and on its power curve 614 / 731 / 711 bhp at 16,000 / 18,000 / 20,000 (sim: 612 / 742 / 715). Idle 4000 rpm, limiter 20,000 (Cosworth ran it to just over 20,000, the first F1 engine to get there). Idle to the limiter takes 0.42 s in neutral: there is no flywheel (0.032 kg·m² estimate) |
| Wave tuning | Fixed trumpets ~0.20 m from bell mouth to valve, and equal-length 4-into-1 headers per bank, measured on the drawn exhaust (estimates) |
| Gearbox | Williams' 7-speed seamless-shift sequential box. The ratios are not published, so these are estimates: a medium-downforce set, 4.78 / 3.67 / 2.98 / 2.51 / 2.16 / 1.90 / 1.72 with a final drive of 4.40 (1st to ~115 km/h, 7th to ~320 km/h at 19,500 rpm). Power-on upshifts are seamless (`seamlessUp`), with a 0.02 s / 0.04 s clutch for downshifts. 4-plate carbon clutch, launch at ~12,000 rpm |
| Vehicle | 605 kg with the driver (the 2006 minimum) plus ~35 kg of fuel, 0.330 m rears, CdA 1.25 (estimate, medium downforce), rear-wheel drive, carbon brakes |
| Exhaust (3D) | Exhaust ports on the outer sides of the heads: each bank's four equal-length primaries curl down and back to a 4-into-1 collector beside the gearbox, then a short blued tailpipe rises through the bodywork, its open end pointing up and back. No catalyst, silencer or valve. Intake: a carbon airbox over the V with the eight trumpets standing in it, and the snorkel forward and up to the roll-hoop scoop |
| Results (sim) | 0–100 km/h 2.2 s floored and 1.9 s with launch control (optimistic: the simulator has no tyre grip limit, and the real car manages ~2.5 s), 0–200 km/h 4.9 s (2006 F1: ~5 s). Full-throttle upshifts at ~19,800 rpm |
| Sound | The detailed voice with nothing to muffle it: two open 4-into-1 systems at 8 × rpm / 120 (1333 Hz at 20,000 rpm), the airbox howl over the driver's head, straight-cut gear whine, almost no valvetrain tick (pneumatic springs), crackle on the overrun. At full throttle it is ~2 dB louder than the stock LFA |

An F1 car is far outside the road-car windows `src/test_car.hpp` was written for, so the CA2006 sets
its own: an idle at 4000 rpm and its start flare, the coast-down, reverse, and `carSpec::pullAwayHi`
(a 640 kg car at 30–40% pedal reaches ~70–85 km/h in 3 s, over the 70 km/h road-car default). It also
sets `dumpStalls = false`: dumping the clutch in 1st at idle makes it bog, then it drives away.

### `v4`: Ducati Desmosedici Stradale (Panigale V4)

Built from the inline-4's piston and rod, scaled, on a new procedural crank and crankcase.

| | |
|---|---|
| Engine | 81 × 53.5 mm = 1103 cc, 14.0:1, 90° V tilted back 42°, counter-rotating crank |
| Firing | Twin Pulse 0-90-290-380° |
| Output (sim) | 128 N·m at 10500 rpm, 155 kW (208 hp) at 12500 rpm (Ducati: 124 N·m / 157 kW), limiter 14000 rpm, idle 1250 rpm |
| Wave tuning | Stack, oval throttle body and port, ~0.31 m, ram at ~10000 rpm; the headers and pair merges reach the pre-silencer under the engine at 0.57 m acoustically (measured on the drawn exhaust). A hump at ~6000 rpm (+4%), a mid-range hole at 7500–8500 (−4%) and a hump at 10500–11000 (runner: estimate) |
| Gearbox | Primary 54/30; gears 38/14, 36/17, 33/19, 32/21, 30/22, 30/24; chain 41/16; 200/55 ZR17 rear |
| Vehicle | 278 kg with rider, CdA 0.32 |
| Rider aids | Sequential 1-N-2-…-6 box, up/down quickshifter (rev-matched downshifts), slipper clutch, launch control, wheelie control |
| Exhaust (3D) | Front-bank headers wrap down in front of the crankcase, rear-bank headers drop behind the gearbox; each bank's pair merges and both feed the black pre-silencer under the engine, with an exhaust valve and two short outlets on the right |
| Results (sim) | 0–100 km/h 3.3 s, 0–200 km/h 6.8 s, top speed ~293 km/h |
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
  the top, and the map adds a little spark advance (same runners and header lengths, so the same humps
  and dips);
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
| Peak torque | 128 N·m at 10500 rpm | 130 N·m at 11000 rpm |
| Peak power | 155 kW (208 hp) at 12500 rpm | 161 kW (215 hp) at 13000 rpm |
| Limiter | 14000 rpm | 14500 rpm |
| Chain | 16/41 | 16/38 |
| Top of 1st / 2nd / 3rd (from the gearing) | ~136 / 174 / 212 km/h | ~152 / 194 / 237 km/h |
| 0–100 km/h | 3.3 s | 3.3 s |
| 0–200 km/h | 6.8 s | 6.8 s |
| After 30 s in 6th | ~293 km/h (on the 14000 limiter) | ~323 km/h |

The HUD shows TUNED after the rpm.

### `rsv4`: Aprilia RSV4 RR (2015-2020)

Built like the Ducati V4: the inline-4's piston and rod scaled to the 78 mm bore, on a procedural
two-pin crank with a front and a rear rod side by side on each pin, with black cam covers. Unlike the
Ducati, the V is a narrow 65° and the crank turns forward. The published figures are from Aprilia's
RSV4 RR spec sheet.

| | |
|---|---|
| Engine | 78 × 52.3 mm = 999.6 cc, 13.6:1, longitudinal 65° V4, DOHC with four titanium valves per cylinder (33 mm intakes), Pankl rods on 36 mm crankpins, forward-turning crank, four 48 mm Marelli ride-by-wire throttle bodies with eight injectors. Estimates: 105 mm rods, front bank 35° above horizontal (the rear one 10° back of vertical), 92 mm cylinder pitch |
| Firing | 1-4-2-3 at 0-115-295-540°: two crankpins 180° apart in a 65° V give the uneven 115-180-245-180 beat |
| Variable intake | Aprilia's ECU-controlled variable-length intake ducts, modelled like MV's TSS with the two-curve model: long ducts for the mid range, short ducts for the top end, blended by rpm from ~7950 to ~10850 rpm (`camSwitchRpm` 9400, `camBlend` 2900); the HUD shows which is in use |
| Output (sim) | 116 N·m at 10500–11000 rpm, 147.6 kW (198 hp) at 13000–13500 rpm, no ram air (Aprilia: 115 N·m at 10500 / 148 kW (201 hp) at 13000), limiter 14000 rpm (estimate), idle 1250 rpm |
| Wave tuning | One effective runner, ~0.30 m (estimate) |
| Gearbox | Cassette six-speed. Primary 73/44 (1.659), straight-cut; gears 39/15, 33/16, 34/20, 31/21, 31/23, 34/27; chain 16/42; 200/55 ZR17 rear (all Aprilia's figures) |
| Vehicle | 204 kg wet (Aprilia, 18.5 l tank) + 80 kg rider, CdA 0.32 |
| Rider aids | Same as the other bikes (APRC): quickshifter with rev-matched downshifts, slipper clutch, launch control held near the torque peak (~10000 rpm, for the tall 1st), wheelie control |
| Exhaust (3D) | 4-2-1: the front bank's headers drop in front of the crankcase, the rear bank's behind the cylinders, each bank's pair merges under the engine into a collector, then a link pipe with the butterfly valve to the short side silencer on the right, angled up, with a titanium end cap |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 7.0 s, ~307 km/h after 30 s in 6th, on the limiter (Aprilia: ~291 km/h) |
| Sound | The 65° V4's hard-edged, uneven howl from the physics, between the Ducati's rumble and the inline-4s' shriek, through medium headers into a short side silencer with a valve, a big airbox, crackle on the overrun and straight-cut primary whine |

Not published by Aprilia, so estimated: rod length, bank angles, cam timing, crank inertia, friction,
the limiter, the runner length, the exhaust's route and valve.

#### Tuned (`--tuned yes`): Akrapovic Evolution and a race map

```bash
./engine_sim --engine-type rsv4 --tuned yes
```

Akrapovic's Evolution full titanium system for the RSV4 RR (S-A10E8-RC), with an ECU race map. On
Akrapovic's own dyno (at the wheel) the system gives +5.2 kW at 13500 rpm (140.8 → 145.8 kW, +3.7%) and
+3 N·m at ~10800 rpm; it is 3.2 kg lighter than stock (4.7 against 7.9 kg) and 2 dB louder. The engine
itself is stock. The changes:

- the system: titanium headers on the stock route, heat-blued and a size up, a plain 2-1 merge with no
  catalyst, no exhaust valve, and Akrapovic's shorter hexagonal titanium muffler with a carbon end cap;
- the map (estimates): a little more spark, and the limiter 500 rpm up at 14500;
- the gearing: tuned, on the stock 16/42 chain 6th gear runs into the 14500 limiter at ~321 km/h, so it is
  gearing-limited. It runs Aprilia's own alternative 16/40 rear sprocket (listed for the RSV4 RR beside the
  16/42), every ratio 5% taller: 6th now tops out at ~13900 rpm, at the power peak and short of the
  limiter, drag-limited. The gearbox is stock;
- 281 kg with the rider (3.2 kg off).

| | Stock | Tuned (sim) |
|---|---|---|
| Peak torque | 116 N·m at 11000 rpm | 120 N·m at 11000 rpm (+3 N·m) |
| Peak power | 147.6 kW (198 hp) at 13500 rpm | 154.8 kW (208 hp) at 13500 rpm (+5%; Akrapovic's +3.7% plus the map) |
| Limiter | 14000 rpm | 14500 rpm |
| Gearing | 16/42 | 16/40 |
| 0–200 km/h | 7.0 s | 6.9 s |
| After 30 s in 6th | ~307 km/h, on the limiter | ~321–324 km/h, drag-limited at ~13900 rpm |

The gain grows with rpm: about +1% at 5000, +3% at 10800 and +5% at 13500. The HUD shows TUNED after the rpm.

### `s1000rr`: BMW S 1000 RR (2023-, K67 ShiftCam engine)

Built the same way as the V4: an inline-4 with the block inclined 32° forward and a forward-turning
crank.

| | |
|---|---|
| Engine | 80.0 × 49.7 mm = 999 cc, 13.3:1, 103 mm rods, 180° flat-plane crank |
| Firing | 1-2-4-3 |
| ShiftCam | A partial-load intake lobe in the mid range, then the full lobe; the HUD shows which is active |
| Output (sim) | 116 N·m at 11000–11500 rpm, 151.5 kW at 14000 rpm (BMW: 113 N·m / 154 kW at 13750), limiter 14600 rpm, idle 1250 rpm |
| Wave tuning | On both cam lobes: funnel, throttle body and port, ~0.23 m, ram at ~11000 rpm; the 4-2-1 headers to the final collector, 0.65 m acoustically (measured on the drawn exhaust). Their dips meet in the mid-range hole superbikes are known for, 7500–9000 rpm (−5%), with humps at 4500–5000 (+2%) and 11000–12000 (+4%) (runner: estimate) |
| Gearbox | Primary 76/46 (1.652); gears 45/17, 46/22, 38/22, 33/22, 34/25, 29/23; chain 46/17; 190/55 ZR17 rear |
| Vehicle | 197 kg wet + 80 kg rider, CdA 0.31 |
| Rider aids | Same as the V4: quickshifter with rev-matched downshifts, slipper clutch, launch control, wheelie control |
| Exhaust (3D) | 4-2-1 titanium headers from the front of the inclined head, under the engine (1+4 and 2+3 merge), collector, link pipe with the exhaust flap, short silencer rising on the right behind the gearbox |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 6.9 s, 308 km/h in 6th |
| Sound | Its own voice (the inline-4 keeps the original one):<br>• radiated exhaust (brightens with revs) through its drawn 4-2-1: headers, pair merges, collector and silencer<br>• exhaust flap, shut at idle and open from ~5000 rpm or under throttle<br>• airbox honk at full throttle<br>• overrun crackle that fades out<br>• quickshifter bark<br>• straight-cut gear whine<br>• valvetrain tick |

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
| Peak torque | 116 N·m at 11500 rpm | 116.5 N·m at 11500 rpm |
| Peak power | 151.5 kW (203 hp) at 14000 rpm | 154.5 kW (207 hp) at 14000 rpm |
| Limiter | 14600 rpm | 15100 rpm |
| 0–200 km/h | 6.9 s | 6.8 s |
| After 30 s in 6th | ~308 km/h | ~321 km/h |

The sim's figures come from `--phystest` (no ram air), like the stock engine's. The HUD shows TUNED
after the rpm.

### `h2`: Kawasaki Ninja H2 (2015-), supercharged

Built like the S 1000 RR, plus a procedural centrifugal supercharger behind the cylinders.

| | |
|---|---|
| Engine | 76.0 × 55.0 mm = 998 cc, 8.5:1, 107 mm rods (estimate), 180° flat-plane crank, block inclined 30° (estimate), forward-turning crank |
| Firing | 1-2-4-3 |
| Supercharger | Gear-driven from the crank at 9.2× crank speed (~129k rpm at 14000), 69 mm impeller with 6 blades and 6 splitters, no intercooler, ram air, a 4 L plenum over the throttle bodies with a relief valve that holds ~+1.0 bar from ~10500 rpm, and a bypass valve that vents the charge when the throttles snap shut |
| Output (sim) | 132.5 N·m at 10500 rpm, 148 kW at 11000–11500 rpm with the charge hot (~100 °C) and no ram air (Kawasaki: 133.5 N·m at 10500 / 147.2 kW at 11000), limiter 14000 rpm, idle 1200 rpm |
| Wave tuning | Smaller than on the unblown bikes (the supercharger's plenum damps the runners' waves): ~0.22 m from the plenum through the throttle bodies rams at ~10500 rpm; the 4-2-1 headers to the collector, 0.66 m acoustically (measured on the drawn exhaust). A dip at 7000–8500 rpm (−3%), a hump at 11000 (+3%) (runner: estimate) |
| Gearbox | Primary 74/50 (1.480); gears 51/16, 48/19, 45/22, 38/22, 32/21, 31/23; chain 44/18; 200/55 ZR17 rear |
| Vehicle | 238 kg wet + 80 kg rider, CdA 0.33 |
| Rider aids | Same as the other bikes; launch control holds ~7500–9500 rpm, where the boost is |
| Exhaust (3D) | The S 1000 RR's 4-2-1 layout into a bigger black silencer |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 7.0 s, ~312 km/h after 30 s in 6th (Kawasaki limits it to 299) |
| Sound | The S 1000 RR's voice through its own drawn exhaust (a bigger silencer), less airbox honk, more gear whine, and the supercharger's whine (impeller blade pass) and rush on top, with the bypass valve's chirp on each snap shut |

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
  top end: stock up to ~3000 rpm, then weaker up to ~10700 rpm (13–15% less torque at 6000–8000 rpm),
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
| Peak torque | 132.5 N·m at 10500 rpm | 144 N·m at 12000 rpm |
| Torque at 6000 / 8000 rpm | 92 / 110 N·m | 80.5 / 93 N·m |
| Peak power | 148 kW (199 hp) at 11500 rpm | 187 kW (251 hp) at 13500 rpm |
| 0–200 km/h | 7.0 s | 6.8 s |
| After 30 s in 6th | ~312 km/h | ~338 km/h |

The HUD shows TUNED after the rpm. `--tuned yes` works with `--phystest` (its own targets),
`--wav` and screenshots; `make test` checks both versions.

### `zx10r`: Kawasaki Ninja ZX-10R (2021-)

Built like the S 1000 RR: the inline-4's piston and rod scaled to the 76 mm bore (the H2 shares the
bore and stroke, without the supercharger), on a procedural flat-plane crank, with a green cam cover.

| | |
|---|---|
| Engine | 76.0 × 55.0 mm = 998 cc, 13.0:1, 107 mm rods (estimate), 180° flat-plane crank, block inclined 30° (estimate), forward-turning crank, four 47 mm throttle bodies, finger-follower valvetrain, one cam (no variable timing) |
| Firing | 1-2-4-3 |
| Output (sim) | 116 N·m at 11500 rpm, 148.5 kW (199 hp) at 13000 rpm, no ram air (Kawasaki: 114.9 N·m at 11400 / 149.3 kW at 13200), limiter 14000 rpm, idle 1250 rpm |
| Wave tuning | Stack, throttle body and port, ~0.20 m, ram at ~11500 rpm; the 4-2-1 headers to the collector, 0.66 m acoustically (measured on the drawn exhaust). A hole at 7500–9000 rpm (−4.5%), humps at 5000 (+3%) and 12000 (+5%) (runner: estimate) |
| Gearbox | Primary 79/47 (1.681); gears 39/15, 41/19, 32/17, 33/20, 31/21, 30/23; chain 41/17 (the 2021 bike's bigger rear sprocket); 190/55 ZR17 rear |
| Vehicle | 207 kg wet + 80 kg rider, CdA 0.31 |
| Rider aids | Same as the other bikes; launch control holds the clutch higher (~7500–9500 rpm) |
| Exhaust (3D) | The S 1000 RR's 4-2-1 layout into a bigger dark silencer |
| Results (sim) | 0–100 km/h 3.5 s, 0–200 km/h 7.0 s, ~315 km/h after 30 s in 6th (on the limiter; the real bike is limited to 299) |
| Sound | The inline-4 superbike voice through its own drawn exhaust (a bigger silencer), a lower note, more airbox honk (the ram-air box sits under the rider) and a little more gear whine than the BMW |

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
| Peak torque | 116 N·m at 11500 rpm | 120.5 N·m at 12000 rpm |
| Torque at 6000 / 8000 rpm | 85.5 / 95 N·m | 90 / 98 N·m |
| Peak power | 148.5 kW (199 hp) at 13000 rpm | 168 kW (226 hp) at 14500 rpm |
| Limiter | 14000 rpm | 15500 rpm |
| Top of 1st / 2nd / 3rd (from the gearing) | ~161 / 194 / 223 km/h | ~179 / 215 / 247 km/h |
| 0–100 km/h | 3.5 s | 3.5 s (the tall 1st and the wheelie limit cap it) |
| 0–200 km/h | 7.0 s | 6.8 s |
| After 30 s in 6th | ~315 km/h | ~333 km/h |

The HUD shows RACE KIT after the rpm.

### `f4rr`: MV Agusta F4 RR (2012-)

Built like the S 1000 RR and the ZX-10R: the inline-4's piston and rod scaled to the 79 mm bore, on a
procedural flat-plane crank, with an MV red cam cover. The published figures are from MV Agusta's F4 RR
spec sheet and brochure (the Euro 3 bike).

| | |
|---|---|
| Engine | 79.0 × 50.9 mm = 998 cc (the shortest stroke here), 13.4:1, DOHC with four radial valves per cylinder, titanium rods, 180° flat-plane crank, forward-turning crank, Mikuni ride-by-wire throttle bodies with eight injectors, Eldor EM2.0 ECU. Estimates: 104 mm rods, block inclined 25°, ~49 mm throttle bores |
| Firing | 1-2-4-3 |
| TSS | MV's Torque Shift System: two-position variable-length intake runners, lifted off the throttle bodies near 10000 rpm. Modelled with the two-curve variable-intake model (as the BMW's ShiftCam): a long-runner curve for the mid range, a short-runner one for the top end. Unlike ShiftCam's instant switch, the ECU schedules the change by rpm and blends the two curves from ~9650 to ~11950 rpm (`camSwitchRpm` 10800, `camBlend` 2300), so the torque curve has no step; the HUD shows which is in use |
| Output (sim) | 111 N·m at 9500–10000 rpm, 147.8 kW (198 hp) at 13500 rpm, no ram air (MV Agusta: 111 N·m at 9600 / 147.7 kW (201 hp) at 13600), limiter 14000 rpm (MV: 14000), idle 1250 rpm |
| Wave tuning | One effective runner, ~0.30 m, ram at ~10000–11000 rpm (+3%); a dip at 7000–8000 (−4%) (runner: estimate) |
| Gearbox | Cassette six-speed. Primary 48/82 (1.708); gears 14/37, 16/33, 18/31, 20/30, 22/29, 21/25; chain 15/41; 200/55 ZR17 rear (all MV's figures) |
| Vehicle | 190 kg dry (MV), 17 l tank: ~208 kg wet (estimate) + 80 kg rider, CdA 0.31 |
| Rider aids | Same as the other bikes: EAS 2.0 quickshifter with rev-matched downshifts, slipper clutch, launch control, wheelie control |
| Exhaust (3D) | 4-2-1-4: headers under the engine, a link pipe with the exhaust valve climbing behind the gearbox into a dark silencer under the seat, ending in the F4's four stacked "organ pipe" tailpipes (four open ends for the sound) |
| Results (sim) | 0–100 km/h 3.4 s, 0–200 km/h 7.1 s, ~304 km/h after 30 s in 6th (MV: 298 km/h) |
| Sound | The inline-4 superbike voice through its own drawn exhaust: a short silencer split into four outlets, a higher, rawer note than the Japanese bikes, a big airbox honk and the radial valvetrain's tick |

Not published by MV, so estimated: rod length, throttle bore, cam timing, crank inertia, friction, the
runner length, the exhaust valve and the wet weight.

#### Tuned (`--tuned yes`): F4 RC with the racing kit

```bash
./engine_sim --engine-type f4rr --tuned yes
```

MV's own tune of this engine: the F4 RC (Reparto Corse, 2015), with the racing kit fitted. MV's F4 RC spec
sheet rates it at 158.0 kW (212 hp) at 13600 rpm and 115 N·m at 9300 rpm (151 kW / 205 hp in street
trim). Bore, stroke, 13.4:1 and the gearbox stay as on the RR. The changes MV lists:

- a lighter crankshaft and lighter pistons and rods (sim: crank inertia 0.034 → 0.029 kg·m², reciprocating
  mass 0.30 → 0.27 kg per cylinder, both estimates);
- a cylinder head with new intake ducts and four upper Magneti Marelli injectors with more fuel flow: both
  TSS curves breathe more, most at the top;
- the racing kit: a Termignoni titanium exhaust with its mapped ECU. Drawn as the stock route in titanium
  with heat-blued headers a size up, no exhaust valve and a shorter titanium can with four titanium tips
  (the kit's real route is an estimate). It's louder, with more crackle and a higher note;
- 175 kg dry with the kit (MV), ~193 kg wet (estimate);
- not from MV: taller track sprockets, 15/38 for the road 15/41 (estimate). On road gearing the RC runs
  into the 14000 limiter in 6th at ~303 km/h (MV quotes 302); with them it pulls to ~330 km/h,
  drag-limited just under the limiter, like the other tuned bikes.

The limiter is not published for the RC; it stays at 14000 rpm.

| | Stock (F4 RR) | F4 RC kit (sim) | MV (F4 RC kit) |
|---|---|---|---|
| Peak torque | 111 N·m at 9500–10000 rpm | 115–116 N·m at 9500–10000 rpm | 115 N·m at 9300 |
| Peak power | 147.8 kW (198 hp) at 13500 rpm | 157.9 kW (212 hp) at 13500–14000 rpm | 158.0 kW (212 hp) at 13600 |
| Mass with rider | 288 kg | 273 kg | 175 kg dry |
| 0–200 km/h | 7.1 s | 6.9 s | |
| After 30 s in 6th | ~304 km/h | ~326 km/h (15/38 sprockets) | 302 km/h (road gearing) |

Both versions blend the TSS curves across the same ~2300 rpm span, fitted together with the curves so
that torque stays within a few N·m of its peak through the change and power climbs without a step (at
most 7–8.5 kW per 500 rpm, against 12 kW with an instant switch). From 4500 rpm up the kit's torque is at
or above stock; below 4000 it is about 1 N·m under. The kit's power is still rising slightly at the
14000 limiter (156.8 kW at 13500, 157.9 at 14000), a little past MV's 13600. The HUD shows F4 RC after the rpm.

### `mxking`: Yamaha MX King 150 (2015-2020)

The Indonesian underbone (Jupiter MX King, T150 engine), made from the engine template: a single
from the inline-4's scaled parts, the barrel laid 70° forward, a forward-turning crank.

| | |
|---|---|
| Engine | 57.0 × 58.7 mm = 149.7 cc, 10.4:1, SOHC four valves, liquid cooled, fuel injected, 98 mm rod (estimate) |
| Firing | Single, every 720° |
| Output (sim) | 13.9 N·m at 7000–7500 rpm, 11.2 kW at 8000 rpm (Yamaha: 13.8 N·m at 7000 / 11.3 kW at 8500), limiter ~10300 rpm (estimate), idle 1400 rpm |
| Gearbox | Five speeds: primary 73/24 (3.042); gears 34/12, 30/16, 30/22, 24/21, 22/23; chain 40/14; 120/70-17 rear |
| Vehicle | 117 kg wet + 70 kg rider, CdA 0.42 |
| Rider aids | None on the real bike. The sim's bike drivetrain still gives it the quickshifter and the automatic; no slipper clutch, launch or wheelie control |
| Exhaust (3D) | One header out of the underside of the head, back under the crankcase into a black-shielded silencer on the right |
| Wave tuning | Strong on a single (one big pulse per cycle): airbox boot, throttle body and port, ~0.27 m, ram at ~7000 rpm, the quoted torque peak; the header into the silencer, 0.58 m acoustically (measured on the drawn exhaust). A hump at 6000–7000 rpm (+3%) and a fall toward the limiter (−6% at 9500–10000) (runner: estimate) |
| Results (sim) | 0–100 km/h 14.1 s, ~117 km/h after 30 s in 5th |
| Sound | The superbike stages tuned for a single: one thump every other turn, a long header into a small silencer with no exhaust valve, little crackle |

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
- a 32 mm throttle body on a short velocity stack (~0.16 m to the valve: it rams at ~10800 rpm), an
  open race filter and a race-fuel map;
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
| Peak torque | 13.9 N·m at 7000 rpm | 17.8 N·m at 11000 rpm |
| Peak power | 11.2 kW (15 hp) at 8000 rpm | 20.8 kW (27.9 hp) at 11500 rpm |
| Limiter | ~10300 rpm | 13000 rpm |
| Idle | 1400 rpm | 1800 rpm |
| 0–100 km/h | 14.1 s | 7.7 s |
| After 30 s in 5th | ~117 km/h | ~163 km/h |

It is peaky: from ~4000 to ~8500 rpm it makes less torque than the stock engine, with a deep hole at
7500–8500 rpm where the expansion chamber's and the stack's dips meet, then it comes on hard. The HUD shows RACE after
the rpm.

### Wave tuning

Each engine's breathing (volumetric efficiency: how full the cylinders get) is a smooth curve over
rpm, calibrated to the published torque and power, with the pressure waves of its intake and exhaust
on top (`engineSim::waveTuning`, `src/physics.hpp`). A valve event starts a wave in a pipe, the wave
rings back and forth, and what counts is the sign of the pressure at the valve at one moment. With x
the number of the ringing's periods that fit in between, the share of the filling it adds is
`gain · env(x) · −cos(2πx)`: a help at x = 0.5, 1.5, 2.5…, a loss at whole periods, each period a
little weaker (`env = exp(−decay (x − 0.5))`). x goes as 1/rpm, so the humps sit at rpm / 3, rpm / 5…
below the first one, closer together and weaker toward idle.

- **Intake ram** (Engelman's Helmholtz model). With the valve open, the air in the runner (length L,
  bore area A) bounces on the gas in the cylinder (volume V at half stroke) like a mass on a spring,
  at f = c / 2π · √(A / (L V)). The piston starts it at TDC; a compression back at the valve as it
  closes (IVC) packs extra air in. Its main hump (x = 1.5, about two bounces per crank turn) is the
  classic ram-tuned speed, and the engines' runner lengths put it at the published torque peak.
- **Exhaust scavenging.** The blowdown pulse at exhaust valve opening runs down the header, comes back
  from the open end (the merge or collector) as a suction after 2L / c, and pulls the cylinder clean
  if it arrives during the valve overlap; a pressure wave there pushes exhaust back in (x = time from
  EVO to TDC over 4L / c). The header length is not typed in: `setupSim` measures it on the engine's
  drawn exhaust (`headerLength`, `src/exhaust.hpp`), the mean over the cylinders of the distance along
  the pipes from each port to the first pipe that carries every cylinder's gas (the final collector,
  a turbo manifold's merge) or widens to 1.6× the header (a silencer, catalyst, expansion chamber),
  plus the port in the head (`scavPortLen`, 0.05 m) and the open end's correction (0.6 × the header's
  radius). Reshape an exhaust and its scavenging moves with it; the tuned versions measure their own
  exhausts.

The ripple multiplies both of ShiftCam's lobes and passes through the throttle like the rest of the
breathing, so it also shows in the sound (each pulse scales with the air trapped). The effective
lengths and gains are estimates (each engine's table lists them): a few percent on the unblown bikes
and the inline-4, less on the boosted engines, and on the RS 3 the charge limit takes it out of the
plateau. Where the ram's and the exhaust's dips meet the superbikes get their mid-range hole at
~7000–9000 rpm. The HUD's dyno chart (D) shows the result; `--dyno` prints the same curve with the
wave share at each speed. `engineParams::ramLen ramDia ramSound ramGain ramDecay` and
`scavLen scavPortLen scavSound scavGain scavDecay` set it (`scavLen` 0, the default, measures the drawn
header; a value of its own overrides that, as does `SCAV_LEN`); a gain of 0 switches a side off.

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
- **Crackle** (bikes). The pops are afterfire: unburnt fuel lit in the hot header. The overrun map
  leaves a share of each closed-throttle charge unburnt for about a second (`soundParams::popChance`),
  the limiter and quickshifter cuts dump whole charges, each exhaust stroke flushes some out, and the
  next hot pulse (a burning slug, or hot enough walls) lights what has gathered, louder the more there
  was. So a cold pipe barely pops, the limiter crackles once the pipes are hot, and a quickshift ends in
  a bark when the first firing lights the cut cylinders' fuel. `--wav` counts them per phase. Every pop
  the sound makes shoots a short flame out of the tailpipes. Some pops, and every bark, also throw a few sparks that arc down
  under gravity. They run on simulated time, so slow motion slows them too. The inline-4 makes no pops, so
  it has no flames.

### Turbocharger (the tuned inline-4)

This section is the tuned inline-4's turbo (`--tuned yes`; it was `--turbo yes` before); the RS 3
(`rs3`) uses the same model with its own sizing.

```bash
./engine_sim --tuned yes
```

The inline-4's exhaust feeds a turbocharger (the Sketchfab "Car Turbocharger" model) hung beside
the block with its shaft along the car. The two header pairs merge above the turbine inlet, the
downpipe leaves the turbine forward and turns back under the turbo into the straight pipe, and the
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
    the clutch is 1.5× the stock one.
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
  when rendering a WAV. `--tuned yes --wav out.wav` drives the car through two full-throttle pulls
  with lift-offs (add `--bov no` to hear the flutter).
- **What it does (sim).** Boost comes in at ~3000 rpm and is held at 0.58 bar to the limiter:
  366 N·m at 4000–4500 rpm, 226 kW at 7000 rpm (225 N·m / 132 kW without). Floored at 3000 rpm, it
  reaches 90% boost in 1.3 s (the intake's dip at 3000–3250 rpm starves the turbine a little).
  0–100 km/h takes 4.5 s floored from idle, 3.8 s with launch control (6.9 s without the turbo,
  on the Fit's gearing); the car model has no
  wheelspin. The HUD shows the boost, wastegate, back pressure, charge and compressor-outlet
  temperatures, the blow-off valve and surge, and the shaft speed (~105–125k rpm on boost). The
  wheels are drawn turning at a readable speed in proportion to the shaft, since the real speed
  would just strobe. `./engine_sim --tuned yes --phystest` checks all this.

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
| Z | Automatic: sport mode (holds gears, rev-matched downshifts). Cars in manual: NORMAL / DCT. NORMAL needs the clutch (C) for every change and to pull away, and stalls if you stop in gear without it. DCT works like a dual-clutch box's manual mode: you pick the gears with E/Q and the box works the clutch: take-off (with launch control), each change at the car's own shift speed (a few hundredths of a second on the RS 3 and the tuned inline-4, whose power-on upshifts are seamless too, with no gap in the drive; ~0.15 s on the LFA; ~0.3 s on the stock inline-4 and the Mustang) (blipping to the lower gear's speed on a downshift, refusing one that would over-rev), anti-stall (the clutch opens near idle and it drops to 1st as you stop), hill-hold, and launch control with the brake (S). The HUD shows MANUAL or MANUAL DCT |
| E / Q | Shift up / down. Car: Q at a stop selects R. Bikes: the quickshifter works while moving, no clutch needed. Cars in manual DCT mode: no clutch needed either. |
| N | Neutral (bikes) |
| C (hold) | Clutch |
| V | Wheelie control on/off (bikes) |
| H / J | Hill up / down |
| L | Cargo (car) / passenger (bikes) |
| F | Blow-off valve on/off (`rs3`, the tuned `inline4`); bypass valve on/off (`h2`, the tuned `mustang-v6`) |
| `[` / `]` | Slow motion / back toward real time (×1 … ×0.01) |
| `,` / `.` | When paused: step the crank by 1° |
| G / B / M / D / X | See-through block / motion blur / sound: mono → stereo → muted / full-throttle dyno chart / shake view (×1, ×20, ×100) |
| K | Playback leveller on / off (lifts the quiet stretches in the window; `--wav` is never levelled) |
| O | Speaker voicing on / off (virtual bass for laptop / phone speakers; window only) |
| U | HUD: top block (engine and car: rpm, gear, pedal, boost, torque, clutch, exhaust) → bottom block (shaking, sound and view settings, firing order and strokes, crank angle) → both → none (only the credit and the dyno chart, if D shows it). Starts on the top block; the rpm / pedal / clutch bars sit under whichever block is shown |
| P / R / Esc | Pause / reset / quit |

## Command-line options

| Option | |
|---|---|
| `--engine-type T` (or `--engine-type=T`) | `inline4` (default), `rs3`, `lfa`, `mustang-v6`, `ca2006`, `v4`, `rsv4`, `s1000rr`, `h2`, `zx10r`, `f4rr`, `mxking` or an alias; `list` prints them, `list --ids` one runnable configuration per line (for scripts). `--v4` and `--s1000rr` still work. |
| `--engine-only` | No gearbox or vehicle (a bike engine runs on a stand) |
| `--selftest` | Kinematics checks: parts stay joined, strokes, clearances, firing lags vs. crank geometry |
| `--phystest` | Physics checks against the engine's targets (start, idle, limiter, dyno curve, firing order, balance, acceleration, shifting, quickshifter rev-match, …) |
| `--wav out.wav` | Render a scripted run to a WAV file. The bikes play a ride: start, blips, limiter, launch through the gears, braking with rev-matched downshifts. The inline-4 plays a start / rev / lift-off script; the RS 3 (and the tuned inline-4) two full-throttle pulls in the car with lift-offs. Mono. |
| `--save-audio yes\|no` | Record what the window plays (stereo, 44.1 kHz, after the leveller and speaker voicing, so M / K / O and slow motion are all heard as played) to `<engine>-<date>-<time>.wav` in the current folder. Default `no`. The file is finished when the window closes; a run killed from the terminal still leaves a playable file (up to the last second). `--mute` records nothing. |
| `--screenshot out.png [--frames n]` | Save a frame and quit. Framing: `--yaw r --pitch r --zoom s`. State: `--crank deg`, `--simtime s --start --throttle x`, `--stopat deg` |
| `--nohud --noblur --mute --ghost 0\|1` | Options for regression shots |
| `--hud top\|bottom\|both\|none` | The HUD blocks shown at start (U cycles them; default `top`) |
| `--grade pct --reverse --sport --manual --dct --no-wheelie` | Starting conditions (`--manual`: start in manual, in neutral; `--dct`: a car's manual mode is DCT) |
| `--bov yes\|no` | A turbo engine (`rs3`, the tuned `inline4`): fit the blow-off valve (default `yes`). `no` lets the compressor surge on lift-offs |
| `--variant name` | Another version of the engine (every engine has one: `tuned`) |
| `--tuned yes\|no` | Same as `--variant tuned` (default `no`) |
| `--compare a.png b.png` | Pixel diff of two screenshots |
| `--synthtest N` | Hidden: test an N-cylinder inline engine built in memory (see [Testing](#testing)) |
| `--dump-params` | Hidden: print every parameter of the selected engine, one `name = value` per line |
| `--dyno` | Hidden: print the full-throttle dyno curve (the D chart's) and the wave tuning's share at each speed, with the calibration variables applied |

## Project layout

```
engine_sim.cpp              entry point: command line, window, event loop
src/
  common.hpp                includes, vec3/mat4, orbit camera, cylArray (per-cylinder storage)
  model.hpp                 mesh loading (ENGMSHv3, still v1/v2), slider-crank kinematics, --selftest
  render.hpp                OpenGL 2.1 renderer, combustion glow, motion blur, gear meshes
  exhaust.hpp               exhaust systems (3D): swept tubes from each engine's layout, heat, gas pulses, valve, crackle fire
  exhaust_layouts.hpp       layouts more than one engine uses (the inline superbike 4-2-1)
  turbo.hpp                 the turbocharger (rs3, the tuned inline4): see-through housings, spinning rotor, spool
  physics.hpp               engine model, ECU, drivetrain, vehicle (parameter structs + engineSim)
  presets.hpp               starting points for a class of engine (the superbike engine and drivetrain)
  engine_type.hpp           engineType: what an engine provides; bikeSpec / carSpec (test targets); REGISTER_ENGINE
  engines.hpp               the registered engines for --engine-type, setupSim()
  calibration.hpp           VE_*, RAM_*, SCAV_*, FMEP_*, IDLE_*, SND_*... overrides for calibrating (every --phystest, --wav)
  hud.hpp                   the HUD and help line, built from what the engine has
  sound.hpp, wav.hpp        sound synthesis + audio stream, --wav rendering
  leveller.hpp              playback leveller (the window's stream only)
  speaker.hpp               small-speaker voicing: EQ + virtual bass (the window's stream only)
  dyno.hpp                  the WOT dyno curve (D), measured in the background (and --dyno)
  bike_gearbox.hpp          the bikes' sequential gearbox (3D)
  test_bike.hpp             --phystest for the bikes, checked against each engine's bikeSpec
  test_car.hpp              --phystest for the cars, checked against each car's carSpec
  test_synth.hpp            --synthtest N
  dump_params.hpp           --dump-params
engines/
  all.hpp                   generated by the Makefile: includes every engine's header (not in git)
  <type>/engine.mk          each engine's part of the Makefile: its header, meshes and mesh rule
  inline4_fit/              inline4_fit.hpp (engine + Honda Fit gearing + car spec + type), gearbox_manual.hpp
                            (3D car gearbox, single or dual clutch), test_inline4.hpp (the tuned version's turbo test), extract_parts.py, scene.gltf/.bin, license.txt,
                            engine.mesh, tests/ (pixel baselines), build_turbo.py, turbocharger/
                            (turbo model), turbo.mesh
  audi_rs3/                 audi_rs3.hpp (engine, turbo, sound, S tronic gearing, exhaust, car spec, type),
                            build_rs3.py, rs3.mesh, build_rs3_turbo.py, rs3_turbo.mesh (procedural turbo)
  lexus_lfa/                lexus_lfa.hpp (engine, sound, ASG gearing, exhaust and intake, car spec, type),
                            build_lfa.py, lfa.mesh
  cosworth_ca2006/          cosworth_ca2006.hpp (engine, sound, seamless gearing, exhaust and intake, car spec,
                            type), build_ca2006.py, ca2006.mesh
  ducati_v4/                ducati_v4.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_v4.py, v4.mesh
  aprilia_rsv4/             aprilia_rsv4.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_rsv4.py, rsv4.mesh
  bmw_s1000rr/              bmw_s1000rr.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_s1000rr.py, s1000rr.mesh
  kawasaki_h2/              kawasaki_h2.hpp (engine, supercharger, sound, gearing, test targets, demo ride,
                            type), build_h2.py, h2.mesh, h2_supercharger.mesh
  kawasaki_zx10r/            kawasaki_zx10r.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_zx10r.py, zx10r.mesh
  mv_agusta_f4rr/           mv_agusta_f4rr.hpp (engine, sound, gearing, test targets, demo ride, type),
                            build_f4rr.py, f4rr.mesh
  yamaha_mxking/            mxking.hpp (engine, sound, gearing, test targets, type, race version),
                            build_mxking.py, mxking.mesh, mxking_race.mesh
  _template/                a new engine to copy (not built): myengine.hpp, build_myengine.py, engine.mk
tools/meshlib.py            the mesh builders' shared steps and the ENGMSHv3 writer
scripts/
  fetch_assets.py           downloads the large model files (make assets)
  regress.sh, pngdiff.py    the behaviour regression check (see Testing)
  spectrogram.py            a WAV as a spectrogram PNG, over a reference recording; --stats for levels
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
   `--engine-type list` (inline4 10, rs3 15, lfa 17, mustang-v6 18, ca2006 19, v4 20, rsv4 22, s1000rr 30, h2 40, zx10r 45, f4rr 47, mxking 50), pick a free one. `engine.mk` lists the
   header and the mesh rule. Nothing outside the folder needs an edit: the Makefile includes every
   `engine.mk` and generates `engines/all.hpp`, `make test` picks the engine up, and the help line
   and HUD follow from what it has (bike or car, boost, wheelie control).
5. **Calibrate.** Put the published figures in `bikeSpec` or `carSpec` and run
   `./engine_sim --engine-type <name> --phystest` until it passes. Environment variables override
   parameters while you try values, for every engine: `VE_BASE VE_PEAK VE_RPM VE_WIDTH` (and `VEL_*`
   for a low-cam lobe, `CAM_SWITCH CAM_BLEND` for an rpm-scheduled blend between the two), the wave tuning `RAM_LEN RAM_DIA RAM_C RAM_GAIN RAM_DECAY SCAV_LEN SCAV_C
   SCAV_GAIN SCAV_DECAY`, `FMEP_A FMEP_B FMEP_C`, `RESID ETA INERTIA STARTER`,
   `IDLE_KP IDLE_KI IDLE_SG IDLE_BASE IDLE_SPARK IDLE_MIN`, the cycle-to-cycle variation `CYC_IDLE CYC_FULL`, `SC_PR SC_FLOW SC_CHOKE SC_RELIEF`,
   `TURBO_AREA TURBO_WG TURBO_BOOST TURBO_INERTIA TURBO_KNOCK TURBO_FALL TURBO_TOPRPM TURBO_TOP
   TURBO_CHARGE` and, for
   `--wav`, `SND_*` (`src/calibration.hpp` lists them). `--dyno` (hidden) prints the full-throttle
   curve with them applied, every 250 rpm (cars) or 500 rpm (bikes), in under a second.

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
- Sound: `--wav out.wav` ends with a `level:` line, the loudest output before the limiter and how many
  samples the limiter touched (it should be none: lower `soundParams::drive` if not).
  `python3 -I scripts/spectrogram.py out.wav` draws its spectrogram (log frequency, 20 Hz–12 kHz);
  `--ref real.wav --ref-offset S` draws a recording under it on the same time scale, lined up by
  skipping S seconds of the recording, and `--stats` prints the peak, the clipping and the loudness
  of every half second. Keep recordings out of git (`*.wav` is ignored) or check their licence.

## Credits

The 3D model is ["Inline-4 Engine" by rafaelaraujo55](engines/inline4_fit/license.txt), licensed
CC-BY-4.0. The credit line is shown on screen. The V4, S 1000 RR, H2, ZX-10R, MX King and RS 3 are built from its parts.

The turbocharger is ["Car Turbocharger" by ml-design](engines/inline4_fit/turbocharger/license.txt),
licensed CC-BY-4.0; its credit line is shown on screen on the tuned inline-4 and on the RS 3.

Published figures come from Ducati, BMW Motorrad (technical data and press material), Audi (RS 3 press
and technical data) and Honda's 2010 Fit specifications. This is a simulation, not manufacturer data.
