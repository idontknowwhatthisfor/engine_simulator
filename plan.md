# Inline-4 Engine Simulator — Plan

> **Layout update:** the code was later split into `src/` (shared) and `engines/<type>/` (one folder
> per engine), with `./engine_sim --engine-type inline4|v4|s1000rr` as the entry point. Paths and flags
> below (`tools/*.py`, `engine.mesh` at the top, `--v4`, `--s1000rr`) are from before the split; see
> README.md for the current layout. Behaviour is unchanged.

Goal: load the Sketchfab "Inline-4 Engine" glTF (`scene.gltf` + `scene.bin`) and make it run like a real
engine. The crankshaft turns, the connecting rods swing on exact slider-crank kinematics, and the pistons
move up and down. Crank speed comes from a physics model (gas pressure, inertia, friction, throttle), not
from a fixed speed. The camera and controls work the same way as in `sphere_simulation_3d.cpp`.

---

## 1. What the model actually contains (measured)

The glTF has **no separate named parts and no animations**. Sketchfab merged everything into 8 meshes,
one per material. Each mesh does contain separate pieces (connected triangle islands), so parts can be
recovered by splitting each mesh into its connected components. I already ran that analysis. The
results below are in raw mesh units (mm).

Root node matrix = `diag(1, -1, -1)`, so Y and Z are flipped when drawn. In **mesh space**:
- the crank axis is the **Z axis** through (x=0, y=0)
- the cylinders point toward **−Y**, so they point up (+Y) after the root flip
- the cylinder centres are at **z = −142.5 (cyl 1), −47.5 (cyl 2), +47.5 (cyl 3), +142.5 (cyl 4)**

| Mesh | Material | Components | What they are | Motion group |
|---|---|---|---|---|
| 0 | 0 (grey, **alpha 0.25, BLEND**) | 1 big + 1 bolt | Engine block (see-through) | static |
| 1 | 0 (translucent, like the block) | 101 small | Bolt threads / nuts on the main caps (x≈22) | static |
| 2 | 1 | 10 × n=216 | Freeze plugs on the block sides and ends | static |
| 2 | 1 | 5 × n=192 on the axis (z=−190,−95,0,95,190) | Main bearing shells | static |
| 2 | 1 | 4 × n=192 off the axis | Rod big-end bearing shells | rod *i* |
| 3 | 2 (dark) | 1 | **Crankshaft** (snout, journals, pins, counterweights, flywheel) | crank |
| 4 | 3 | 4 × n=1390 | **Pistons** (Ø85, x∈[−42.5,42.5]) | piston *i* |
| 4 | 3 | 4 × n=2491 | **Connecting rods** | rod *i* |
| 4 | 3 | 8 × n=2587, 8 × n=586 | Rod-cap bolts and nuts (2 per rod) | rod *i* |
| 5 | 4 | 4 × n=162 | **Gudgeon (wrist) pins** | piston *i* |
| 6 | 5 | 3 | Main bearing caps | static |
| 7 | 6 | 10 | Main-cap bolts | static |

Note: there is no cylinder head, valvetrain or camshaft in the model. The simulation shows the bottom
end only, and the strokes and combustion are shown through the HUD and colour (see §5).

### Rest-pose kinematics (from the mesh)

| | Cyl 1 & 4 | Cyl 2 & 3 |
|---|---|---|
| Gudgeon pin centre (x, y) | (0, −203.0) | (0, −116.5) |
| Crankpin centre (x, y) | (29.0, −45.75) | (−26.25, 41.2) |
| Crank throw r | **53.97** | **49.03** |
| Pin angle from cylinder axis θ₀ | **32.4°** | **212.4°** (180° opposite) |
| Rod length L (pin to pin) | **160.0** | **160.0** |
| Crankpin journal radius | 17.0 | 17.0 |
| Main journal radius | ≈19.2 | |
| Crank axis (x, y) | (0.01, −0.03) | |

- Pairs 1/4 and 2/3 sit 180° apart. This is the standard flat-plane I4 crank, and firing order **1-3-4-2**
  fits it.
- **The artist's throws differ (53.97 vs 49.03 mm).** Each cylinder is self-consistent (L = 160 for both),
  so we use **per-cylinder r_i measured at load time**. That way the rods stay attached exactly to the
  modelled crankpins. Pairs 1/4 will have a slightly longer stroke (108.4 mm) than 2/3 (97.8 mm). This
  can't be seen, and it is the only way to keep the parts joined without editing the mesh.
- Bore 85 mm, mean stroke ≈ 103 mm, so displacement ≈ **2.34 L**.

Every value above must be **computed at load time** (component centroids and circle fits), not
hard-coded. The numbers here are used to check the results (see §8).

---

## 2. Tech stack

- **C++20, SFML 3.0.0** (already installed in `/usr/local`; the same API as the sphere sim)
- **OpenGL 2.1 + GLSL 1.20** through `sf::RenderWindow` (depth buffer 24, MSAA 4). There are about 290k
  triangles, so the sphere sim's painter's-algorithm 2D projection can't be used. We need a real depth
  buffer. Use `GL_GLEXT_PROTOTYPES` with Mesa's `libGL` (headers are present in `/usr/include/GL`), so no
  loader library is needed.
- HUD text: SFML `sf::Text` drawn between `window.pushGLStates()` / `popGLStates()`
- A small hand-written `vec3` / `mat4` (no GLM installed)
- Font: copy `SpaceMono-Regular.ttf` from `/home/debian/gitrepo/code/c/sfml/` into this folder

### Asset pipeline (recommended: offline split)
`tools/extract_parts.py` uses only the Python standard library (no numpy; the analysis above already
works this way):
1. Parse `scene.gltf` and read POSITION / NORMAL / indices from `scene.bin`.
2. Weld vertices by position, then union-find over triangles to get connected components.
3. Classify each component into a **group** using the rules in the §1 table (mesh index + vertex count +
   bounding box). Assign it to cylinder *i* by nearest z-centre.
4. Compute pivots: crank axis, per-cylinder crankpin centre (circle fit on crank vertices in the pin's
   z-band), gudgeon pin centre (centroid of the mesh 5 component), r_i, θ₀ᵢ, L_i.
5. Write `engine.mesh`: a header, then per group: group id, cylinder index, material index, pivot data,
   and interleaved `pos(3f) normal(3f)` + `uint32` indices. Also write the material colours/alpha.

C++ then only needs to `fread` a flat binary. (Alternative: vendor `cgltf.h` and do the split in C++ at
startup. The union-find takes milliseconds. Use this only if a Python step is unwanted.)

---

## 3. Kinematics (exact slider-crank)

For cylinder *i* with crank angle θ (0 = pin pointing at the cylinder, i.e. TDC), throw rᵢ and rod L:

```
pin distance from crank axis along the bore:   s(θ) = r cosθ + sqrt(L² − r² sin²θ)
rod angle from bore axis:                      φ(θ) = asin(r sinθ / L)
crankpin position:                             P(θ) = R_z(θ) · P₀   (rotated about the crank axis)
```

Per-frame transforms in mesh space (then multiply by the root `diag(1,-1,-1)` matrix):

| Group | Model matrix |
|---|---|
| crank | `R_z(θ − θ_rest)` about the crank axis |
| piston *i* (+ gudgeon pin) | `T(0, −(s(θᵢ) − s(θ₀ᵢ)), 0)`; the bore is along −Y in mesh space |
| rod *i* (+ bolts, nuts, big-end shell) | `T(pin_now) · R_z(φ(θᵢ) − φ(θ₀ᵢ)) · T(−pin_rest)`, rotating about the gudgeon pin centre |
| static | identity |

θᵢ = θ + θ₀ᵢ − θ_rest. At θ = θ_rest the scene must reproduce the original model **exactly**
(regression check). Watch the sign conventions: rotation sense about Z versus the side of the bore the
pin swings to. Check that `R_z` applied to P₀ gives the same pin x-sign as the rod tilt in the mesh.

Direction of rotation: clockwise seen from the front (timing snout, mesh z = −332 end), as on most
engines. Provide a constant to flip it.

---

## 4. Engine physics ("moves like real life")

State: crank angle `θ` (wrapped to 0..720° for the 4-stroke cycle), angular velocity `ω`.

```
I_total · dω/dt = Σ T_gas,i + Σ T_inertia,i − T_friction(ω) − T_load + T_starter
```

- **Fixed-step integrator**: semi-implicit Euler at **dt = 1/20000 s** with a time accumulator. This is
  the same idea as `SUBSTEPS` in the sphere sim, but it stays deterministic at any frame rate. Clamp the
  frame dt to 1/30 s, as the sphere sim does.
- **Geometry**: A = π·(85 mm)²/4, cylinder volume V(θ) = V_c + A·(s_max − s(θ)), compression ratio 10:1
  → V_c.
- **Phasing** (firing order 1-3-4-2): cycle offsets cyl1 = 0°, cyl3 = 180°, cyl4 = 360°, cyl2 = 540°.
  This matches the mesh, where 1/4 are at the same crank position and 2/3 are 180° away.
- **Cylinder pressure per stroke** (φ = local cycle angle, 0 = TDC at the start of intake):
  - Intake 0–180°: p = p_manifold(throttle, rpm). Around 30 kPa at idle, up to about 100 kPa at full
    throttle (a simple throttle-plate flow model with a manifold filling lag τ ≈ 50 ms).
  - Compression 180–360°: p = p_ivc · (V_ivc / V)^1.32
  - Combustion: **Wiebe function** heat release from the spark angle (advance 10–35° BTDC, rising with
    rpm) over about 50°. Fuel energy ∝ trapped air mass (∝ p_manifold · VE(rpm)) × AFR 14.7 × LHV,
    η_comb ≈ 0.9. Integrate dp = ((γ−1)/V)·dQ − γ·(p/V)·dV.
  - Expansion 360–540°: same ODE, without heat release after the burn ends
  - Exhaust 540–720°: p blends to about 105 kPa
- **Gas torque**: `T_gas,i = (p_i − p_crankcase) · A · ds/dθ` (exact derivative from §3).
- **Reciprocating inertia**: piston + pin + about ⅓ of the rod (≈ 0.5 kg) per cylinder.
  `T_inertia,i = −m_rec · s̈ · ds/dθ`. This gives the real 2nd-order shake and the speed ripple at idle.
- **Rotating inertia**: crank + flywheel I ≈ 0.12 kg·m² (tuneable).
- **Friction**: Chen–Flynn FMEP `= a + b·p_max + c·rpm + d·rpm²`, converted to torque.
- **Starter motor**: about 80 N·m stall, falling to 0 at about 300 rpm. Active while its key is held.
- **Ignition off** → no combustion, and the engine coasts down through compression ("engine braking").
- **Rev limiter**: cut spark above 6800 rpm. Optional **dyno load** knob for T_load.
- **Idle stability**: a small idle-air control (PI on throttle bypass) holds about 800 rpm with the
  throttle closed. Turning it off lets the engine stall, which is realistic.

The result: the engine cranks slowly and unevenly on the starter, catches, settles to a slightly lumpy
idle, revs up quickly when you blip the throttle, and coasts down when the ignition is off.

### Seeing the motion (temporal aliasing)
At 3000 rpm the crank turns 50 times per second. At 144 fps that strobes (the wagon-wheel effect).
- **Time scale** control (×1 … ×0.01): the physics clock is scaled, so the motion is still
  physically correct, only slowed down.
- **Single-step** when paused: advance the crank 1° per key press.
- Optional: draw a few faint "ghost" frames of the crank and rods (motion blur) when ω·dt_frame > 20°.

---

## 5. Rendering

- One VBO + IBO per group. Draw with a per-group model matrix uniform.
- Shader: Blinn-Phong with the **same light as the sphere sim** (dir (−0.45, −0.55, 0.7), specular
  power 60, ambient 0.18, diffuse 0.85), so both programs look alike. Use the material base colours from
  the glTF.
- **Pass order**: opaque groups first (crank, rods, pistons, caps, bolts), then the block last with
  alpha 0.25, depth write off and back-face culling. This keeps the internals visible through the
  translucent block.
- Background `(14,15,20)`. Floor quad with grid lines under the engine (as `drawFloor`). Optional soft
  contact shadow.
- **Combustion cue**: tint the piston crown orange or white from the cylinder's heat-release rate.
  Optional faint glow quad in the bore above the piston during the power stroke.
- Camera: reuse `struct camera {yaw, pitch, dist}`, building a view matrix equal to `toView()`: orbit
  around the engine's bounding-box centre. Projection: perspective with `fovY = 2·atan((HEIGHT/2)/FOCAL)`
  so FOCAL = 700 frames the same way as before.

---

## 6. Controls (taken from `sphere_simulation_3d.cpp`)

Kept **identical** (same constants, same feel):

| Input | Action | Same as sphere sim |
|---|---|---|
| Left-drag | Orbit: `yaw += dx·0.008` (**horizontal reversed vs the sphere sim, as you asked**), `pitch = clamp(pitch + dy·0.008, −1.4, 1.4)` | ✔ / reversed |
| Mouse wheel | Zoom: `dist *= (delta>0 ? 0.9 : 1.1)`, clamped to [2.2, 8] × model radius | ✔ (clamp rescaled to model size) |
| P | Pause / resume | ✔ |
| R | Reset (engine stopped, θ = rest pose, default camera) | ✔ |
| Esc | Quit | ✔ |
| G | Toggle **ghost block** (translucent ↔ hidden) — was "gravity" toggle | key kept, new meaning |
| SPACE (hold) | **Starter motor** — was "kick" (adds energy) | key kept, matching meaning |

New engine controls:

| Input | Action |
|---|---|
| W / Up (hold) | Open throttle (eases up at about 3 s from 0 to 100%; releases back to 0) |
| 1–9, 0 | Fixed throttle 10%…90%, 100% |
| I | Ignition on/off |
| `[` / `]` | Time scale ÷2 / ×2 (×0.01 … ×1) |
| `,` / `.` | When paused: step crank −1° / +1° |
| L | Toggle dyno load |

HUD (top-left, SpaceMono 18, updated every 0.5 s like the original):
`FPS`, `RPM`, `throttle %`, `crank angle`, `torque N·m`, `power kW`, `ignition`, `time scale`, `PAUSED`.
Per-cylinder stroke list (e.g. `1:POWER 3:COMP 4:INTAKE 2:EXH`). The help line at the bottom uses the
same style (14 px, grey). Add the CC-BY credit line from `license.txt` (required by the licence).

---

## 7. File layout and build

```
inline4_sim/
  plan.md
  engine_sim.cpp          # single-file program, structured like sphere_simulation_3d.cpp
  tools/extract_parts.py  # glTF → engine.mesh (stdlib only)
  engine.mesh             # generated
  SpaceMono-Regular.ttf   # copied
  Makefile
  scene.gltf, scene.bin, license.txt  (unchanged)
```

```
g++ -std=c++20 -O2 engine_sim.cpp -o engine_sim -DGL_GLEXT_PROTOTYPES \
    -I/usr/local/include -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
    -lsfml-graphics -lsfml-window -lsfml-system -lGL
```

Code sections in `engine_sim.cpp`: config constants → math (vec3/mat4) → mesh loading → kinematics →
engine physics → GL renderer → HUD → `main()` event loop (the event handling is lifted from the sphere sim).

---

## 8. Implementation phases and checks

1. **Extract parts.** ✅ DONE. `python3 tools/extract_parts.py` writes `engine.mesh` (6 MB, 21 draw
   groups, all 187,414 triangles kept) and prints the classification and kinematics report.
   ✔ The counts match §1 (4 pistons, 4 rods, 8+8 rod fasteners, 4 pins, 4 big-end shells, 1 crank).
   ✔ r, L and θ₀ come within 0.5 mm / 0.5° of the §1 table.
2. **Static viewer.** ✅ DONE. `make && ./engine_sim`. The OpenGL 2.1 renderer, sphere-sim camera controls
   and translucent block all work. The freeze plugs got their own `block` kind so they hide with the
   block. Test flags: `--screenshot out.png --frames n --yaw r --pitch r --zoom s --ghost 0|1`. Baselines
   for the phase 3 check are in `tests/baseline_rest*.png`.
   ✔ It looks like the Sketchfab model, and orbit/zoom feel the same as the sphere sim.
3. **Kinematics.** ✅ DONE. `./engine_sim --selftest` passes: big-end gap ≤ 0.00003 mm, crowns 3.9/8.9 mm
   below the deck, skirts clear the counterweights by 2.7 mm. The rest pose matches the phase 2 baseline
   pixel for pixel (`--compare`, HUD rows masked). Debug motor: C turns it on/off, `[ ]` change speed,
   `, .` step 1°. `--crank deg` / `--spin rpm` for scripted shots.
   Drive θ at a constant speed from a debug key.
   ✔ At θ_rest the result is pixel-identical to step 2.
   ✔ Over a full rotation, |crankpin_now − rod big-end centre| < 0.1 mm.
   ✔ The piston never leaves the bore (the x stays at 0, the crown stays below the deck).
   ✔ 1/4 move together, 2/3 move opposite.
4. **Physics.** ✅ DONE. `./engine_sim --phystest` passes every check. Measured: cranking 211 rpm (lumpy),
   catches on the first compression, start flare to about 1700 rpm, idle 803 rpm ±20 at 25 kPa, idle →
   limiter in 0.6 s, lift-off back to 780 rpm, coast-down from 3000 rpm in 1.0 s. WOT torque 181 N·m at
   1000 rpm, peak 203 N·m at 3500 rpm, 110 kW at 6500–7000 rpm (BMEP 10.9 bar, peak cylinder pressure
   about 79 bar).
   Calibration notes: etaHeat 0.85; FMEP constant 1.0 bar (includes about 8 N·m of accessories);
   blow-by τ = 1 s; residual-gas load factor 0.7→1.0 with MAP. Idle control works like a real ECU: fast
   spark trim plus slow air PI, both on rpm filtered over 0.1 s, active only within 300 rpm of idle, and
   the idle valve is held open on overrun. Without the filter, the controllers chased the firing-pulse
   ripple and the idle hunted by ±150 rpm. The C debug motor is gone. `[ ]` is now slow-motion, and
   `, .` step only while paused. Scripted runs: `--start --simtime s --throttle x`. Tuning hooks for
   --phystest: `IDLE_KP IDLE_KI IDLE_SG INERTIA PHYS_TRACE`.
   Original plan for this step: pressure model, torque, starter, throttle, ignition, friction.
   ✔ Cranking speed is about 200–300 rpm and visibly uneven.
   ✔ Idle holds at about 800 rpm.
   ✔ Full throttle with no load reaches the limiter in about 1–2 s.
   ✔ With the ignition off it coasts down in a few seconds.
   ✔ Peak torque comes out at about 180–230 N·m for 2.3 L (sanity check against a real BMEP of about
      10–12 bar).
5. **HUD and polish.** ✅ DONE.
   - Combustion: piston crowns glow from the gas temperature T = pV/(mR) (800 K dark → 2000 K
     yellow-white). An additive flame column fills the real gas volume (height V/A above the crown), so
     near TDC it reaches into where the missing head's combustion chamber would be. It fades through
     expansion and blowdown.
   - Motion blur (B): up to 8 faded poses across the frame once the crank turns > 20° per frame. It costs
     about 20% FPS at high rpm.
   - HUD: rpm bar with redline, throttle bar, per-cylinder stroke + pressure in firing order, FPS averaged
     over 0.5 s.
   - Fix: blending used to write alpha < 1 into the framebuffer, so PNG screenshots looked washed out
     (the window itself was fine). Now `glBlendFuncSeparate` keeps alpha at 1. `--compare` checks RGB
     only. The baselines were regenerated (RGB unchanged).
   - Test flags: `--stopat deg` (pause at a cylinder 1 cycle angle), `--nohud`, `--noblur`.
6. **Extras.** ✅ DONE.
   - **Sound** (M mutes): synthesised from the simulation at 40 kHz. Exhaust blowdown pulses (signed
     √Δp) plus displaced exhaust scaled by charge go through a pipe model (15 Hz DC block, 110 Hz
     resonance, 700 Hz low-pass), with turbulence noise, intake pulsing and throttle hiss, combustion
     clatter and starter gear whine on top. The audio is streamed through a lock-free ring to an
     `sf::SoundStream` that resamples to 44.1 kHz and holds about 50 ms queued, so slow-mo lowers the
     pitch. `--wav out.wav` renders a scripted run. Checked offline: dominant 26.5 Hz at idle (= 800/30)
     and 222 Hz on the limiter. Level order: WOT 0.37 rms > overrun 0.14 > idle 0.03. No clipping.
   - **p–V diagram** (D): cylinder 1, one sample per degree, coloured by stroke. Degrees not yet reached
     this cycle are dimmed, because they still hold the last cycle (this avoids a confusing jump on
     rev-cut cycles). IMEP comes from the loop area (idle ≈ 1.1 bar, the friction it has to overcome).
     The HUD also shows the 0–720° cycle angle.
   - **Vibration**: vertical shaking force split into 1st/2nd order, roll reaction torque, and pitch
     moment. In `--phystest` at 6000 rpm: 2nd order 16.06 kN vs textbook 15.73 kN (+2% from
     higher-order terms), 1st order 2.32 kN vs 2.34 kN. The 1st order is not zero because the model's
     throws are uneven. Pitch is 0 (mirror symmetry). The block sits on spring-damper mounts (150 kg,
     12 Hz vertical, 9 Hz roll, ζ 0.15). X cycles the shake view ×1 / ×20 / ×100.
   - Blur now starts at 45°/frame (max 6 ghosts). At 20° it triggered at idle and fed back into lower
     FPS.

7. **Transmission.** ✅ DONE (added on request after phase 6).
   - **Drivetrain physics:** a friction clutch (350 N·m capacity; the slip torque is capped by
     engagement) drives a 5-speed box (3.50/2.10/1.40/1.00/0.80) and a 4.1 final drive. The car is
     1300 kg on 0.31 m wheels, with rolling resistance, CdA 0.65 aero drag, a grade, brakes, and
     cargo (L: +500 / +1500 kg). The clutch torque is the engine's load torque. There is an overrun
     fuel cut while the wheels drive the engine.
   - **Automatic (T toggles):**
     - The clutch engages from engine rpm, like a centrifugal clutch, so the engine never stalls. It
       holds the car on a hill while stopped.
     - Shift points depend on throttle: up at 2000 + 4300·pedal rpm, down at 1200 + 2000·pedal rpm.
       Kickdown above 85% pedal.
     - Downshift under load: when the pedal is over 40% but the car isn't accelerating (a hill or
       cargo), and upshifts are blocked while that lasts.
   - **Manual:**
     - C is the clutch pedal: pressing is fast, releasing is slow. E/Q shift up/down.
     - A gear only goes in with the clutch pressed; otherwise a message says so.
     - Dumping the clutch at idle stalls the engine.
   - **Drive-by-wire throttle:** the pedal sets a manifold-pressure target, and the throttle opens to
     whatever area passes that flow at the current rpm (the flow model inverted). Torque follows the
     pedal roughly linearly. The old fixed pedal curve gave near full-load air at 20–40% pedal. Phase
     4 tests are unchanged.
   - **3D gearbox:** procedural, behind the flywheel, in mesh space, riding on the engine mounts.
     - Pressure plate (turns with the crank) and clutch disc (turns with the input shaft), so slip is
       visible.
     - Input shaft, a constant-mesh head pair, a countershaft below, and 5 gear pairs sized from the
       real ratios at a 105 mm centre distance, with tooth phases set so they mesh.
     - Output shaft and flange, and a brass dog collar plus brass highlight on the engaged pair.
     - See-through housing (G hides it).
   - **Road:** the floor grid scrolls with road distance and the scene tilts with the grade (H/J).
   - **Checks** (`--phystest`):
     - 0–100 km/h in 7.8 s at WOT, shifting at about 6450 rpm. At 20% pedal the first upshift is at
       3146 rpm.
     - At 35% pedal the car cruises in 4th at 73 km/h; on a 12% hill it drops to 3rd and holds
       2470 rpm.
     - Manual: a clutch dump at idle stalls; a gentle take-off bottoms out at 911 rpm, with no stall.
       Shifting without the clutch is refused.
   - **Honda Fit gearing** (2nd gen 2009–2013 5MT, Honda's 2010 spec sheet): 3.308 / 1.870 /
     1.303 / 0.949 / 0.727, reverse 3.308, final drive 4.62, tyre radius 0.305 m. This replaces the
     generic 3.50/2.10/1.40/1.00/0.80 × 4.1 and 0.31 m. The gearbox geometry resizes itself from the
     ratios.
     - Re-tuned with it: the auto "downshift under load" rule is now *losing speed with > 30% pedal*
       (accel < −0.1 m/s²), and upshifts are blocked while not gaining speed. The old "not
       accelerating with > 40% pedal" rule also fired when cruising steadily, and missed the hill at
       35% pedal.
     - ECU stall rule: below 120 rpm without the starter, fuelling stops until the starter is used or
       the engine is spun past 300 rpm (a bump start). Before this, a clutch dump bogged to 51 rpm and
       recovered, which a real engine can't do.
     - Results: 0–100 km/h in 7.7 s. On a 12% hill at 35% pedal, 4th → 2nd at 62 km/h. A clutch dump
       at idle stalls; a gentle take-off bottoms out at 1006 rpm.
   - **Reverse** (now 3.308, the Fit's): countershaft gear → idler on its own shaft → output gear. The idler
     position is solved so it meshes with both, and the tooth phases use a general contact-direction
     formula. It only engages with the car stopped (|v| < 0.5 m/s):
     - Manual: Q from N, clutch pressed.
     - Automatic: Q at a stop gives R, E gives D. There is no auto-shifting in R.
     - Checks: R at 30% pedal reaches −29.6 km/h in 4 s. D is refused while rolling back, and R is
       refused while moving forward.
   - **HUD:** the blue bar is now clutch *pedal* travel (1 − engagement), so holding C fills it.
     `--nohud` now really hides the help and credit lines; the baselines were refreshed, with the 3D
     region unchanged.
   - **Flags and keys:** `--grade pct` and `--engine-only` (no gearbox; the old pixel baselines still
     match). L now cycles cargo; the old fan load is gone.

8. **Ducati V4.** ✅ DONE (added on request). `make run-v4`, or `./engine_sim --v4`.
   - **Model:** `tools/build_v4.py` turns `engine.mesh` into `v4.mesh`. It reuses cylinder 1's piston,
     gudgeon pin, rod, cap bolts/nuts and big-end shell, scaled from 85 mm to 81 mm bore. The rod beam is
     shortened from 152.5 to 110 mm (estimate; Ducati doesn't publish it). The small eye and the big end
     with its bolts stay rigid, so both eyes stay round. New procedural parts: a twin-pin crank
     (pins 70° apart, a front and a rear rod side by side on each pin, three mains, counterweighted
     webs, alternator rotor, primary gear) and a see-through crankcase, barrels and heads with red
     cam covers.
   - **Layout:** Desmosedici Stradale. 81 × 53.5 mm = 1103 cc, 90° V tilted back so the front bank is
     42° above horizontal, counter-rotating crank. Cylinders 1 front-left, 2 rear-left (pin A),
     3 front-right, 4 rear-right (pin B). Twin Pulse firing at 0/90/290/380°.
   - **Format v2** (`ENGMSHv2`): each cylinder also stores bank, deck, bore and firing lag. Piston and
     rod vertices are kept in the cylinder's own frame (bore toward −Y) and turned by R_z(bank) at draw
     time, so the slider-crank code, crown glow and flame work unchanged. The inline-4 stays on v1.
   - **Physics:** `ducatiV4Params()`: 14:1, 250°/475° IVC/EVO, 0.36 kg reciprocating mass, I = 0.030 kg·m²,
     8.5e-3 m² throttle, VE curve peaking ≈114% at 10–12.5k rpm, idle 1250 rpm, limiter 14000 rpm. The
     engine runs on a stand (no clutch, gearbox or car) — see 9 for the bike gearbox. Counterweights cancel each V pair's 1st order.
   - **Checks:** `--v4 --selftest` adds firing-lag-vs-geometry and crank-vs-skirt clearance (6 mm).
     `--v4 --phystest`: cranks ~465 rpm, catches at once, idle 1255 rpm at 29 kPa, idle → limiter in
     0.5 s, 124 N·m at 10–11k, 155 kW (208 hp) at 13k (Ducati claims 124 N·m / 157 kW), firing intervals
     90-200-90-340, 1st-order shake 0 N. The inline-4 tests and renders are unchanged (pixel-identical).

9. **V4 bike gearbox.** ✅ DONE (added on request). `./engine_sim --v4` now rides; `--v4 --engine-only`
   is the old engine stand.
   - **Gearing:** Panigale V4. Primary 54/30 (1.80), gears 38/14 36/17 33/19 32/21 30/22 30/24, chain
     41/16, 200/55 ZR17 rear (0.322 m). 278 kg with an 80 kg rider, CdA 0.32. `ducatiDriveParams()`.
   - **Drivetrain:** the car code is generalised. The rpm thresholds, gear count and primary ratio are now
     parameters, with car defaults unchanged. Bike additions:
     - Sequential 1-N-2-…-6, with N as a half click from 1st or 2nd (N key).
     - Up/down quickshifter: 50 ms ignition cut going up, throttle blip going down, no clutch needed
       while moving.
     - Slipper clutch: passes 35% of its capacity on overrun.
     - Launch control in automatic: holds about 7000 rpm with an ignition cut, and slips the clutch,
       capped at the wheelie limit, until the engine and wheel speeds meet.
     - Wheelie control: once the clutch locks, trims fuel to keep about 1.05 g.
   - **3D:** a gear on the crank drives the clutch basket on the main shaft (behind and below the
     crank). The clutch pack turns with the main shaft, so slip shows. Six pairs with Ducati's tooth
     counts mesh to the output shaft; the engaged pair is brass. There is a 16-tooth chain sprocket on
     the left, a shift drum, a see-through gearbox case and a red clutch cover. The crank's primary
     disc in `build_v4.py` is now a 30 mm hub under the toothed gear.
   - **Keys:** E/Q up/down (quickshifter), N neutral, C clutch, T auto/manual, Z sport, S brake, H/J hill,
     L passenger.
   - **Checks** (`--v4 --phystest`): idles in gear without creeping. 0–100 km/h in 3.4 s and 0–200 in
     6.8 s, at ≤1.05 g. Shifts 1→2 at 13600 rpm. Top speed 289 km/h in 6th. Manual take-off bottoms at
     909 rpm. Quickshift up (ignition cut, rpm drop = ratio step) and down (blip, +30% rpm). A clutch dump
     at idle stalls. The inline-4 tests and renders are unchanged (pixel-identical to before).

10. **Neutral and wheelie-control switch.** ✅ DONE (on request).
   - **N** toggles neutral from any gear, in automatic and manual. Pressing N again (or E/Q in
     automatic) selects the gear for the road speed. When rolling in automatic, it blips to that gear's
     rpm and closes the clutch, using the rev-matched shift path. In manual, leaving N needs the clutch.
     In automatic the clutch stays open in N; before this it could drag at a high idle and stall the
     engine when 1st went in.
   - **V** (or `--no-wheelie`) switches wheelie control off. Then there's no fuel trim and no
     engagement cap on launch.
   - Launch control now maps rpm to clutch engagement directly instead of integrating, which stopped
     the hunting without the wheelie cap.
   - **Checks:**
     - In automatic N: revs to the limiter without moving, and coasts at speed with the engine back
       at idle.
     - Rejoining from N lands without a stall (7100 rpm in 6th).
     - Manual: N works straight from 4th.
     - Wheelie control off: 1.3 g and 0–100 km/h in 2.6 s (3.3 s with it on). The sim doesn't model
       the front lifting, so that's the "wheel in the air" figure.

11. **BMW S 1000 RR.** ✅ DONE (on request). `make run-s1000rr`, or `./engine_sim --s1000rr`.
   - **Model:** `tools/build_s1000rr.py` turns `engine.mesh` into `s1000rr.mesh` (magic `BMWS1KRR`, the v2
     layout). It reuses the V4 builder's helpers and cylinder 1's piston/rod, scaled to the 80 mm bore,
     rod beam shortened to BMW's 103 mm. New: a flat-plane crank with four throws on a 92 mm bore pitch
     (pitch is an estimate), five mains, counterweights, alternator rotor left, primary gear right
     (z 195.2); a see-through crankcase, one-piece block and head, blue cam cover.
   - **Layout:** 80.0 × 49.7 mm = 999 cc, 13.3:1, block inclined 32° forward, firing 1-2-4-3
     (lags 0/180/540/360). The crank turns forward, so the front of the bike is **+X** here (the V4's is
     −X), and the bike gearbox is mirrored behind it (`bikeGearboxModel::side = −1`).
   - **Physics:** `bmwS1000rrParams()` (starts from the Ducati set). **ShiftCam**: a second, mild intake
     lobe with its own VE curve; the ECU runs whichever lobe breathes better (`engineParams::lowCam`).
     The HUD shows the active lobe. 48 mm throttle bodies, limiter 14600, idle 1250.
   - **Gearing:** `bmwDriveParams()`: primary 76/46, gears 45/17 46/22 38/22 33/22 34/25 29/23, chain
     46/17, 190/55 ZR17 (0.320 m), 197 kg + 80 kg rider. Tooth counts and primary/sprocket teeth are now
     `drivetrainParams` fields (the Ducati's moved there unchanged).
   - **Checks** (`--s1000rr --phystest`, generalised from the V4 test via `bikeSpec`): 114.5 N·m at
     11000 rpm, 153 kW at 14000–14500 (BMW: 113 N·m / 154 kW at 13750), even 180° firing, 1st-order
     shake 0, idle 1255 rpm, 0–100 km/h 3.4 s, 0–200 6.9 s, 1→2 at 14150 rpm, 307 km/h in 6th.
     `--s1000rr --selftest` passes. The inline-4 and V4 outputs and renders are unchanged; the V4 dyno
     sweep now steps 500 rpm above 10000 (same peaks).

12. **S 1000 RR sound.** ✅ DONE (on request). The inline-4 and V4 voices are byte-identical to before;
   `soundParams` (in `engineParams`) holds each engine's voice, and `superbike = true` adds:
   - **Tailpipe radiation:** the heard pressure is the rate of change of the pipe's gas flow (an open
     pipe end radiates as a monopole), +6 dB/octave, so the note brightens and swells with rpm.
   - **Quarter-wave 4-2-1 pipe:** a 1.15 m delay-line resonator (c = 520 m/s, inverting lossy reflection),
     odd modes of 113 Hz, plus a direct low "burble" path for the idle.
   - **Valve lift:** blowdown swells in over 30° after EVO. A pressure step at a closed-throttle opening
     radiated as a click and made the overrun hiss.
   - **Exhaust flap:** shut (1.6 kHz, quieter) at idle and light load, open (6.5 kHz) from ~5000 rpm or
     with the throttle up, with a 0.3 s servo lag.
   - **Airbox honk** (190 Hz) and bellmouth howl (750 Hz) from the intake pulses, scaled by throttle opening.
   - **Turbulent rasp** that follows outflow squared, so it is loud at full charge and quiet on the overrun.
   - **Crackle:** on lift-off above 3500 rpm, fading over about 1 s. Also a bark when the quickshifter
     cut ends and crackle on the limiter.
   - **Straight-cut gear whine:** the 46-tooth primary gear, plus the engaged pair under load.
   - **Valvetrain tick:** heard at idle.
   - `--s1000rr --wav out.wav` renders a ride: start, idle, blips, limiter, launch through the gears,
     overrun, cruise. Tuning hooks: `SND_*` env vars in `renderWav`.
   - **Checked offline:** firing note = rpm/30 (44 Hz idle, 483 Hz limiter); 61% of the energy is above
     1 kHz on the limiter vs 7% at idle; idle is 24 dB below the limiter; overrun is 86% below 500 Hz
     (burble + crackle); no clipping.

13. **Rev-matched quickshift down.** ✅ DONE (on request), for both bikes. A downshift is now three stages:
   - **Unload:** blip until the torque through the dogs falls under 6 N·m (or 50 ms).
   - **Free:** the old gear is out (`drivetrain::dogsFree`, `coupled()` false), so the engine runs free
     and the ECU revs it toward the lower gear's rpm at the current road speed. The blip eases off ahead
     using the crank acceleration averaged over 15 ms; the raw value swings with every firing and made
     the blip chatter.
   - **Engage:** the new gear goes in within 1.5% of that speed, or after 250 ms.
   - Overrun fuel cut is off while the gear is out. A downshift that would land above the limiter is
     refused, as the real ECU does (`qsMaxRpm`). The HUD message shows the target rpm.
   - Before this, the blip ran 50 ms with the old gear still locked, so the engine couldn't rise and the
     new gear went in with the engine at the old gear's speed.
   - **Checks** (`--phystest`, 2→1 on the overrun and 3→2 at speed): engaged in 98–118 ms at 98.5% of
     the target rpm, ≤ 230 rpm clutch slip after (≤ 400 required). The `--s1000rr --wav` demo now brakes
     and quickshifts 5→4→3→2.

---

## 9. Risks / open points

- **Uneven crank throws** in the model (53.97 vs 49.03). Handled with per-cylinder r. An alternative is to
  move the 2/3 pins out to 53.97 and accept a small misfit with the crank mesh. Not recommended.
- **Rest pose is not TDC.** All transforms are deltas from the rest pose, so this doesn't matter. Just
  never assume θ = 0 at load.
- **Block transparency sorting.** One translucent mesh drawn last with back-face culling is usually
  enough. If inner block walls show artifacts, add a depth pre-pass for the block.
- **Units.** The mesh is in mm and the physics uses SI. Convert in exactly one place.
