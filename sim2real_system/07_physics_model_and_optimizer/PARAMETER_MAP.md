# Sim ↔ Real Parameter Map

**What this is.** An exhaustive map between every tunable parameter of the *simulated*
UGV Waveshare Jetson rover (`jetson_rover_v2.usd`, driven by `ugv_controller_v2`) and the
*observable behaviour* of the *real* rover on the 1 m × 1 m grids. For each parameter it
says: what it physically means, the current baked value, the **one real-rover test that
isolates it**, the **direction to move it** given a real-vs-sim symptom, **why** (the
mechanism), a **sensible starting range** and how to **narrow** it, and what it **couples
with** (so the optimiser doesn't fight itself).

This is the design reference for the automated calibration loop: the real rover + overhead
camera produce ground truth, the sim is driven with the *same* command stream, and an
optimiser nudges the parameters below until sim motion matches real motion — using a
**single overhead webcam + ArUco marker as cheap ground truth** instead of a motion-capture
rig (OptiTrack/Vicon). That substitution is the budget story: the loop below needs only
pose-vs-time at ~cm accuracy, which the grid homography already delivers.

Companion files:
- Sim-side authoring: `ugv_controller_v2/calibration/` (`sim_calibration.yaml`,
  `apply_sim_calibration.py`, `CALIBRATION.md`).
- Real-side capture + compare: `sim2real_calib/` (`trials/record_traj.py`,
  `trials/compare_paths.py`, `parallel/cmd_sequence.py`, `viz/metric_overlay.py`,
  `calib/grid_calib.json`).

---

## Table of contents

1. [How to read the map (the closed loop)](#1-the-closed-loop)
2. [What we can observe](#2-what-we-can-observe)
3. [Master parameter inventory (grounded in the USD)](#3-master-parameter-inventory)
4. [The canonical test battery](#4-the-canonical-test-battery)
5. [THE MAP — behaviour ↔ parameter, group by group](#5-the-map)
   - A. [Straight-line linear scale](#a-straight-line-linear-scale)
   - B. [Turning / skid-steer](#b-turning--skid-steer)
   - C. [Surface friction (per-surface)](#c-surface-friction)
   - D. [Acceleration / motor ramp](#d-acceleration--motor-ramp)
   - E. [Stopping / coast-down](#e-stopping--coast-down)
   - F. [Speed & yaw-rate ceilings](#f-speed--yaw-rate-ceilings)
   - G. [Mass, inertia, centre of mass](#g-mass-inertia-com)
   - H. [Rolling resistance & surface deformation (the carpet/foam gap)](#h-rolling-resistance)
   - I. [Contact fidelity / jitter](#i-contact-fidelity)
   - J. [Timing & latency](#j-timing--latency)
6. [Identifiability & staging order](#6-identifiability--staging-order)
7. [Per-surface starting-range table](#7-per-surface-starting-ranges)
8. [The automated optimiser (ML) design](#8-the-optimiser)
9. [Gaps to build in the repo](#9-gaps-to-build)
10. [Worked example end-to-end](#10-worked-example)

---

## 1. The closed loop

```
            same timed /cmd_vel sequence (cmd_sequence.py)
            ┌───────────────────────────┬───────────────────────────┐
            ▼                           ▼
   REAL rover on surface S        SIM rover (jetson_rover_v2.usd)
   overhead cam → ArUco → grid    Isaac /odom (= true chassis pose)
            │                           │
            ▼                           ▼
     real_run.csv  (t,x,y,yaw)    sim_run.csv (t,x,y,yaw)
            └─────────────┬─────────────┘
                          ▼
                 compare_paths.py  →  discrepancy features
         (endpoint err, heading err, pointwise err, path length,
          + per-segment: steady speed, accel τ, coast dist, yaw rate)
                          ▼
              which feature is off  →  §5 map  →  which θ, which way
                          ▼
            optimiser proposes new θ  →  apply to sim  →  repeat
```

**Read the map in that direction:** a *feature of the discrepancy* points at a *small set of
parameters* and a *direction*. The map's job is to keep that pointing **one-to-one** — to pick
tests that move one feature per parameter, so the optimiser (and you) never have to
disentangle two knobs from one number.

**Fair-comparison note.** Isaac's `ComputeOdometry` publishes the *true simulated chassis
pose/twist* (it reads the physics body state, it does **not** integrate sim wheel encoders).
So comparing real **camera** (truth) to sim **/odom** (truth) is apples-to-apples on real
motion — sim `/odom` is **not** hiding sim wheel slip. Good. (If you ever switch the sim to an
encoder-integrated odom, revisit this — then you'd be comparing truth to belief.)

**Noise floor.** Grid homography + marker detection is good to ~0.5–1.5 cm and ~1–2° once
intrinsics/grid are calibrated. Don't chase sim-vs-real agreement below that; make it a weight
floor in the loss (§8) so the optimiser stops when it's inside measurement noise.

---

## 2. What we can observe

Everything the loop knows about either rover is a pose-vs-time track `(t, x, y, yaw)`. All the
discriminating power comes from **what maneuver produced the track** and **which feature you
extract**. Features, and the raw observable each needs:

| Feature | From | Isolates (preview of §5) |
|---|---|---|
| **Steady-state linear speed** `v_ss` | straight segment, slope of distance vs t after ramp | wheelRadius; top-speed clamp; rolling resistance |
| **Distance per command·time** | straight segment endpoint | wheelRadius (primary), slip |
| **Linear accel time-constant** `τ_v` | straight segment, rise to `v_ss` | drive damping, maxForce, mass |
| **Lateral veer on a straight** | straight segment, y-drift & heading drift | left/right friction or mass asymmetry, COM offset |
| **Steady yaw rate** `ω_ss` | spin-in-place segment, dyaw/dt | wheelDistance, maxAngularSpeed, friction ratio |
| **Degrees turned per command·time** | spin segment, Δyaw | wheelDistance (primary) |
| **Turn translation ("walk")** | spin segment, xy drift of center | skid-steer slip, friction ratio, COM |
| **Yaw accel time-constant** `τ_ω` | spin segment, rise to `ω_ss` | drive damping, inertia(zz), maxForce |
| **Coast distance after stop** | append zero-command, distance after cmd→0 | friction, mass, drive damping, maxForce |
| **Coast/settle time** | time from cmd→0 to v≈0 | same as above |
| **Arc radius** for `(v,ω)` | combined segment, fit circle | wheelRadius ÷ wheelDistance ratio, slip |
| **Repeatability (σ across N runs)** | repeat any segment | sets the loss weight floor; flags real-rover variance |

> **Build note:** `record_traj.py` already logs `(t,x,y,yaw)` for both sources. The per-segment
> feature extraction (`v_ss`, `τ_v`, `ω_ss`, coast distance, …) is the piece to add next to
> `compare_paths.py`; §8 lists it as the optimiser's feature front-end.

---

## 3. Master parameter inventory

Every motion-relevant authored value in `jetson_rover_v2.usd` (confirmed by dumping the crate),
plus the floor material that lives in the environment USD. **Cadence** is how often the
automated loop should touch it:

- **once** — physical constant; measure the real rover once, enter, never optimise.
- **per-surface** — re-identify on each floor (carpet/foam/concrete).
- **per-iter** — the optimiser's live knobs within a surface.
- **rarely** — leave at default unless a specific instability shows up.

| # | Parameter (USD attribute) | Prim path | Current | Cadence | Group |
|---|---|---|---|---|---|
| 1 | `inputs:wheelRadius` | `…/Drive_Graph/SkidSteer_Controller/DiffController` | **0.04 m** | per-iter | A |
| 2 | `inputs:wheelDistance` (effective track) | same DiffController | **0.5 m** | per-surface | B |
| 3 | `inputs:maxLinearSpeed` | same DiffController | **0.5 m/s** | per-surface | F |
| 4 | `inputs:maxAngularSpeed` | same DiffController | **1.9 rad/s** | per-surface | F |
| 5 | corner µ_dynamic | `/World/Physics_Materials/corner_wheel` `physics:dynamicFriction` | **1.0** | per-iter | C |
| 6 | corner µ_static | same | **1.2** | per-iter | C |
| 7 | middle µ_dynamic | `/World/Physics_Materials/middle_wheel` `physics:dynamicFriction` | **0.65** | per-surface | B/C |
| 8 | middle µ_static | same | **0.8** | per-surface | B/C |
| 9 | `physxMaterial:frictionCombineMode` | both wheel materials | **average** | once (design) | C |
| 10 | **floor µ_dynamic / µ_static** | env USD `GroundPlane/CollisionPlane` material | **unauthored → Isaac default** | **per-surface** | C |
| 11 | corner-wheel drive `damping` | `…/joints/{left,right}_{up,down}_wheel_link_joint` `drive:angular:physics:damping` | **10000** | per-iter | D |
| 12 | corner-wheel drive `maxForce` (torque cap) | same 4 joints | **3.4e38 (≈∞)** | per-surface | D/E |
| 13 | corner-wheel drive `stiffness` | same 4 joints | **0** (velocity drive) | rarely | D |
| 14 | `physxJoint:maxJointVelocity` | same 4 joints | **inf** | rarely | F |
| 15 | middle-wheel drive `damping`/`maxForce` | `…/{left,right}_middle_wheel_link_joint` | **0 / 0 (passive)** | rarely | B |
| 16 | base_link `physics:mass` | `/World/ugv_rover/base_link` | **1.40495 kg** | once | G |
| 17 | base_link `physics:centerOfMass` | same | **(-0.00046, 0, -0.0148)** | once | G |
| 18 | base_link `physics:diagonalInertia` | same | **(0.0065, 0.0093, 0.0122)** | once | G |
| 19 | wheel `physics:mass` (×6) | `/World/ugv_rover/*_wheel_link` | **0.08 kg each** | once | G |
| 20 | wheel `physics:diagonalInertia` (×6) | same | **(1.17e-4, 1.71e-4, 1.17e-4)** | once | G |
| 21 | wheel rigid-body `physxRigidBody:angularDamping` | wheel links | **unauthored (default)** | per-surface | H |
| 22 | wheel collider `physxCollision:contactOffset` / `restOffset` | `…/collisions/.../node_STL_BINARY_` | **unauthored (default ~0.02/0)** | rarely | I |
| 23 | wheel collider `physics:approximation` | same | **convexHull** | rarely | I |
| 24 | articulation `solverPositionIterationCount` / `Velocity` | `/World/ugv_rover/base_footprint` | **32 / 4** | rarely | I |
| 25 | scene `physxScene:solverType` | `/physicsScene` | **TGS** | once | I |
| 26 | scene `enableCCD` / `enableStabilization` / `enableGPUDynamics` | `/physicsScene` | **on / off / off** | once | I |
| 27 | scene `frictionCorrelationDistance` / `frictionOffsetThreshold` | env `/PhysicsScene` | **2.5e-4 / 4e-4** | rarely | I |
| 28 | physics dt / render dt | `run_baked_rover.py` (`World(physics_dt=1/240, rendering_dt=1/60)`) | **240 Hz / 60 Hz** | once | J |
| 29 | `/cmd_vel` publish rate | `cmd_sequence.py --rate` | **20 Hz** | once (match real) | J |

Physical geometry the sim already encodes (for reference, all `once`): corner wheels at
`x = ±0.0855 m`, all wheels at `|y| = 0.08726 m` → **physical track 0.1745 m**, **wheelbase
0.171 m**; wheel contact radius ≈ 0.04 m. Note #2 `wheelDistance = 0.5 m` is **~2.9× the
physical track** — it is deliberately *not* geometry; it's the lumped skid-steer knob (§B).

What `apply_sim_calibration.py` can author today: #1–8, #11, #12, #16, #19. **Not yet
scriptable** (gaps, §9): floor material #10, COM #17, inertia #18/#20, angular damping #21,
solver #24, combine mode #9.

---

## 4. The canonical test battery

Five maneuvers, each run **identically on the real rover (per surface) and in sim**, each
designed to light up one feature group. All fit inside a 1 m × 1 m grid. Express every command
as a `cmd_sequence.py` YAML so real and sim get the byte-identical stream.

| Test | Sequence (after 1 s settle) | Primary feature(s) | Isolates (groups) |
|---|---|---|---|
| **T1 Straight** | `lin 0.12, ang 0, 3 s`, then `0,0, 2 s` | `v_ss`, distance, `τ_v`, coast dist, veer | A, D, E, (C) |
| **T2 Spin** | `lin 0, ang 0.5, 3.14 s`, then `0,0, 2 s` | `ω_ss`, Δyaw, turn-walk, `τ_ω`, coast-yaw | B, D, E |
| **T3 Speed sweep** | `lin {0.05,0.10,0.20,0.35,0.5}` × 2.5 s each | `v_ss(cmd)` curve, saturation point | A, F, H |
| **T4 Arc** | `lin 0.12, ang 0.3, 4 s` | arc radius, radius vs command | A÷B ratio, slip |
| **T5 Stop-from-speed** | `lin 0.3, 2.5 s`, then `0,0, 2 s` | coast distance & time at speed | E, G, H |

Run each **N≥5 times per surface** to get σ (repeatability) — that σ is both a data-quality
check on the real rig and the per-feature weight floor in the loss.

The existing `parallel/sequences/square.yaml` is a fine *end-to-end validation* sequence (T6:
"does the whole thing land in the same place"), but it mixes every effect together — use it to
**validate** a calibration, not to **identify** parameters.

---

## 5. THE MAP

Each block: **the real symptom → the sim knob → direction → why → range → isolating test →
couplings.** Symptoms are phrased as "real does X relative to sim."

### A. Straight-line linear scale
**Params:** #1 `wheelRadius` (primary); slip via #5/#6/#10 (secondary).
**Isolating test:** T1 (and T3 for the saturation check).
**Current:** `wheelRadius 0.04 m`. Sim linear speed = `wheelRadius × wheel_rad/s`, and the
DiffController converts `v_cmd → wheel_rad/s = v_cmd / wheelRadius`. With a stiff velocity drive
(#11 damping 10000) and no torque limit, the wheels track that target almost exactly, so
**open-loop sim distance ≈ `wheelRadius`-scaled command** — `wheelRadius` is the clean linear
lever.

| Real vs sim (T1) | Move | Why |
|---|---|---|
| real travels **farther** per command·time | **raise** `wheelRadius` | larger effective radius → more ground distance per wheel-rev at the same commanded wheel speed |
| real travels **less** | **lower** `wheelRadius` | — |
| real `v_ss` right but **ramps slower** | leave radius; see §D | that's the motor ramp, not the scale |
| real can't reach high `v_ss` (T3 saturates low) | see §F/§H | ceiling or rolling resistance, not radius |

**Range → narrow:** start `0.03–0.05 m` (physical ≈0.04). Because the map from `wheelRadius`
to distance is essentially **linear and monotone**, one T1 at a mid command gives you the ratio
directly: `wheelRadius_new = wheelRadius_old × (real_distance / sim_distance)`. Then confirm it
holds across T3's commands (if it doesn't, the gap is slip/resistance, not radius).
**Couples with:** friction (C) — only if wheels slip. Keep corner µ high enough (default 1.0)
that the driven wheels **don't** slip on concrete, so `wheelRadius` stays the sole linear knob;
let *surface* slip show up through the floor material, not through corner-wheel friction.

### B. Turning / skid-steer
**Params:** #2 `wheelDistance` (primary); #7/#8 middle-wheel friction and the corner:middle
friction **ratio** (secondary); #15 middle passive-ness (structural).
**Isolating test:** T2 (and T4 to check it against curved motion).
**Current:** `wheelDistance 0.5 m` — the DiffController splits `(v,ω)` into left/right wheel
speeds using `ω·wheelDistance/2`. A real 4-wheel **skid-steer** scrubs its tyres sideways to
yaw, so it turns **slower** than an ideal differential drive of its true 0.1745 m track would;
inflating `wheelDistance` to 0.5 lumps that lost-yaw-to-scrub into the kinematics.

| Real vs sim (T2) | Move | Why |
|---|---|---|
| real turns **fewer degrees** per command·time | **raise** `wheelDistance` | bigger modelled track → less yaw per wheel-speed split → sim slows to match |
| real turns **more degrees** | **lower** `wheelDistance` | — |
| real **walks** (translates) more while spinning | **raise** middle µ (#7/#8) and/or review COM (#17) | more scrub resistance = more energy into translation; also an off-centre COM makes the spin orbit |
| sim spins **too freely / frictionlessly** | **raise** middle µ ratio vs corner | passive mid-wheels set how hard the chassis fights yaw |

**Range → narrow:** `wheelDistance 0.25–0.9 m`. Like §A it's ~monotone in Δyaw, so scale it:
`wheelDistance_new = wheelDistance_old × (sim_deg / real_deg)` from one T2, then refine with T4.
**Couples with:** **surface friction (C)** — this is the key coupling. More floor grip → more
tyre scrub → the *same* real rover needs a *larger* `wheelDistance` on concrete than on a
slick/low-pile surface. So `wheelDistance` is **per-surface**, and it must be re-fit *after*
the surface friction is set, not before (see staging, §6).

### C. Surface friction (per-surface) — the on-the-fly knob
**Params:** #10 **floor** µ_dynamic/µ_static (the per-surface knob you'll actually sweep),
#5/#6 corner-wheel µ, #7/#8 middle-wheel µ, #9 `frictionCombineMode`.
**Isolating test:** T1 distance *deficit vs the no-slip prediction*, T2 turn-walk, T4 arc slip.
**Current & structural caveat:** there is **no explicit floor material** — the ground uses
Isaac's default. For per-surface work you must **author a floor `PhysicsMaterial` and bind it to
`GroundPlane/CollisionPlane`** (gap §9). Effective contact friction = `combine(µ_wheel,
µ_floor)`, and today `combineMode = average` → `µ_eff = (µ_wheel + µ_floor)/2`. That makes the
floor a *half-strength* knob (dropping floor µ only moves the average halfway).

**Design recommendation:** set `frictionCombineMode = min` on the wheel materials and keep
wheel µ high; then `µ_eff = min(µ_wheel, µ_floor) ≈ µ_floor`, so the **floor material becomes a
clean, single, per-surface knob** — exactly what the on-the-fly loop wants. (Alternatively
`multiply` for a smoother but less intuitive knob.)

| Real vs sim | Move | Why |
|---|---|---|
| real **slips** (travels less than the no-slip `wheelRadius` prediction), sim doesn't | **lower** floor µ_dynamic | less grip → driven wheels spin faster than ground → distance deficit reproduced |
| real grips fine, sim slips | **raise** floor µ_dynamic | — |
| real is **jerky/hard to start from rest**, sim creeps smoothly | **raise** floor µ_static (and the static:dynamic gap) | static > dynamic sets the break-away/stiction feel at launch |
| real **under-rotates / scrubs** more on this surface | **raise** floor µ + re-fit `wheelDistance` (§B) | grip drives scrub; friction and the skid knob move together per surface |

**Range → narrow (rubber-ish wheel on floor, Coulomb):** see the full table in §7. Philosophy:
start each surface at the **material-plausible band**, run T1/T2, then bisect. Because distance
deficit is monotone in µ over the slipping regime (and ~flat once there's enough grip to not
slip), the loop finds the edge fast: sweep coarse (3 points across the band), keep the bracket
that reproduces the real slip, bisect twice.
**Couples with:** `wheelRadius` (A) in the slipping regime (both change distance — that's why
you fix radius on a **high-grip** surface first, §6), and `wheelDistance` (B) always.
**The honest limitation:** Coulomb friction does **not** model carpet-pile rolling resistance or
foam compression — on those surfaces the real rover loses *speed*, not just *grip*. Don't fix a
speed deficit by crushing friction (that adds slip, the wrong signature). Use §H instead.

### D. Acceleration / motor ramp
**Params:** #11 corner drive `damping` (primary), #12 `maxForce` (torque cap), #16 mass.
**Isolating test:** T1 rise to `v_ss` (`τ_v`), T2 rise to `ω_ss` (`τ_ω`).
**Current:** `damping 10000`, `maxForce ≈∞`, `stiffness 0`. For a force-type velocity drive,
applied torque `= damping × (target_vel − current_vel)`, capped at `maxForce`. `damping 10000`
is a very **stiff** tracker → near-instant speed, i.e. the sim currently reaches commanded speed
almost immediately. The real rover has motor inductance, controller ramp, and a gearbox — it
ramps over a measurable `τ`.

| Real vs sim | Move | Why |
|---|---|---|
| real **ramps up slower** to `v_ss` | **lower** `damping` | softer velocity tracking → longer time-constant to reach target |
| real **overshoots then settles** | lower `damping` and/or add a touch of `stiffness` | underdamped tracking produces overshoot |
| real **bogs down / stalls** on a bump or hard turn, sim powers through | **lower** `maxForce` | real motors have finite torque; a cap lets the sim stall/slow under load |
| ramp fine but top speed low on resistive floor | §F/§H, not here | — |

**Range → narrow:** `damping 50–20000` (log-scale search — it spans orders of magnitude);
`maxForce` start `∞`, only bring down (try `0.5–5 N·m`-equivalent) if T5/obstacle loading shows
stall. Fit `damping` to the measured `τ_v` from T1: larger `τ` ⇒ lower `damping`, roughly
`damping ∝ 1/τ_v` near the operating point.
**Couples with:** mass (G) — `τ ∝ inertia/damping`, so set mass from the scale once (§G) and
treat `damping` as the ramp knob. Keep `stiffness = 0` unless you need overshoot.

### E. Stopping / coast-down
**Params:** #12 `maxForce`, #11 `damping`, #16 mass, floor µ (#10) — all via momentum vs braking.
**Isolating test:** T5 (stop from 0.3 m/s), and the trailing zero-command of T1/T2.
**Current:** with `damping 10000` + `maxForce ∞`, the velocity drive actively brakes to zero
almost instantly (big negative tracking torque), so the sim **stops much shorter** than a real
rover that coasts on its momentum.

| Real vs sim | Move | Why |
|---|---|---|
| real **coasts farther** after cmd→0 | **lower** `maxForce` (so the drive can't hard-brake) and/or **raise** mass | less braking torque + more momentum → longer coast; floor µ then bleeds it off |
| real coast governed by friction (slides, then grips) | tune floor µ_dynamic (#10) | coast distance at speed is set by `v²/(2·µ·g)` once drive braking is capped |
| sim coasts too far | raise `maxForce` or raise floor µ | — |

**Range → narrow:** this is the main reason to **finitely cap `maxForce`** — with an infinite
cap the drive is also an infinite brake and coast can't be matched. Pick `maxForce` from the
accel side (§D) and *verify* on the coast side (§E); if accel and coast disagree, the real
asymmetry is motor-drive-vs-freewheel, model it by lowering `maxForce` (affects both) and
letting friction set the coast.
**Couples with:** D (same `maxForce`), G (mass), C (floor µ). Strongly coupled → identify D and
E **together** from T1+T5 on the same surface.

### F. Speed & yaw-rate ceilings
**Params:** #3 `maxLinearSpeed`, #4 `maxAngularSpeed`, #14 `maxJointVelocity`.
**Isolating test:** T3 speed sweep (linear), a yaw sweep variant (angular).
**Current:** `maxLinearSpeed 0.5`, `maxAngularSpeed 1.9`. These clamp `(v,ω)` *before*
kinematics, so they're the hard ceilings Nav2/teleop can command.

| Real vs sim (T3) | Move | Why |
|---|---|---|
| real `v_ss` **plateaus** below the commanded high values | **lower** `maxLinearSpeed` to the plateau | the hardware simply can't go faster; the clamp makes the sim agree and stops Nav2 over-commanding |
| real yaw plateaus below sim | **lower** `maxAngularSpeed` | same, rotational |
| real reaches commanded speed fine | leave clamps at/above real max | clamps aren't the limiter |

**Range → narrow:** read the plateau straight off T3/yaw-sweep: set clamp = measured real top
speed (+5 % headroom). One-shot, per-surface (resistive floors lower the achievable top speed).
**Couples with:** §H — a *low plateau from resistance* (carpet) is **not** a hardware ceiling;
don't encode a resistance loss as a `maxLinearSpeed` clamp or the rover will feel wrongly capped
on firmer floors. Distinguish: a clamp plateaus *flat* regardless of command headroom;
resistance makes `v_ss` sag with a *soft* shoulder.

### G. Mass, inertia, COM
**Params:** #16 base mass, #17 COM, #18 base inertia, #19/#20 wheel mass/inertia.
**Isolating test:** measure once (scale + CAD/estimate); *detect* mis-set via `τ` (D) and
turn-walk (B).
**Current:** base 1.405 kg (likely **pre-battery** — the real rover with battery is heavier;
measure it). COM slightly below/behind centre.

| Real vs sim | Move | Why |
|---|---|---|
| real heavier (weigh it) | **raise** base mass to measured | more momentum (longer coast, slower accel), more normal force (more grip) |
| real spins with an **offset orbit** (center walks a circle) | shift **COM** (#17) toward the real battery bias | yaw about a displaced COM translates the geometric center |
| yaw accel `τ_ω` wrong but linear `τ_v` right | adjust **Izz** (#18 z-inertia) | rotational time-constant scales with yaw inertia, independent of linear |

**Range → narrow:** **measure, don't optimise.** Put the real rover (battery in) on a kitchen
scale → base mass. Estimate inertia from mass + the known 0.1745 m track as a plausibility
check; only let the optimiser nudge Izz within ±30 % if `τ_ω` can't be matched by damping alone.
**Couples with:** D/E (momentum), B (COM→turn-walk). Fixing mass early removes a confounder from
every dynamic fit, so do §G **first** (it's a `once` measurement).

### H. Rolling resistance & surface deformation (the carpet/foam gap)
**Params:** #21 wheel-body `angularDamping` (add it — primary proxy), #12 `maxForce`,
#3 `maxLinearSpeed` (per-surface), optionally a small base `linearDamping`.
**Isolating test:** T3 on each surface (compare `v_ss(cmd)` curves concrete vs carpet vs foam),
T5 coast on each surface.
**Why this exists:** rigid Coulomb contact (C) models **grip/slip** but not the **energy lost**
to deforming carpet pile or compressing foam, or to bearing/gearbox drag. The real rover on
heavy carpet or foam reaches a **lower steady speed** and **coasts shorter** for the same
command — a *speed/energy* loss, not a *slip* loss. Fixing it with friction is wrong: lowering µ
would make the wheels slip (distance deficit *with* wheel-spin), whereas real carpet shows a
distance deficit *without* wheel-spin. Different signature, different knob.

| Real vs sim | Move | Why |
|---|---|---|
| real `v_ss` **sags** on carpet/foam vs concrete, same command, **no wheel-spin** | add/raise wheel `angularDamping` (#21) | a viscous drag torque on each wheel bleeds speed proportional to rotation — the rolling-resistance proxy |
| real coasts **much shorter** on carpet | raise wheel `angularDamping` and/or lower `maxForce` | drag removes kinetic energy during coast |
| real top speed on carpet is a **soft** shoulder (not a flat clamp) | lower per-surface `maxLinearSpeed` *and* add drag | capture both the ceiling and the sag |
| foam: rover **sinks / pitches**, slow and draggy | wheel `angularDamping` + slightly lower `maxForce`; accept rigid-contact won't model sinkage geometry | deformable contact is out of scope for rigid PhysX; drag is the affordable proxy |

**Range → narrow:** `angularDamping` start `0.0` (concrete) and climb per surface; typical
useful band `0–5` (unitless PhysX damping) — fit it to the carpet/foam `v_ss` sag from T3 and
check it reproduces the T5 coast. **This is the parameter that turns "friction per surface" into
a physically honest per-surface model** for soft floors.
**Couples with:** C (don't double-count: set grip with friction on a firm floor first, then add
drag for soft floors), F (soft ceiling).

### I. Contact fidelity / jitter (rarely touched)
**Params:** #24 solver iters (32/4), #22 contact/rest offset, #23 convexHull approximation,
#26 CCD/stabilization, #27 friction correlation distance/offset.
**Isolating test:** visual — wheels buzzing/sinking/penetrating; non-smooth `v_ss`; the
historical `box.geom` crash.
**Why:** these set *numerical* contact quality, not target behaviour. The convex-hull wheel is a
faceted approximation of a round tyre, so at very low speed you can see tiny periodic speed
ripple; raising solver iterations and tuning contact offset stiffen contact.

| Symptom | Move | Why |
|---|---|---|
| wheels visibly **sink** into floor or **buzz** | raise `solverPositionIterationCount`; set a small `restOffset`, `contactOffset ≈ 2–4× restOffset` | more iterations + proper offsets = stiffer, less-penetrating contact |
| high-speed pass-through / tunnelling | keep `enableCCD = true` | continuous collision catches fast contacts |
| periodic speed ripple at crawl | consider finer collider than convexHull for wheels (costly) | hull facets modulate the contact patch |

**Range → narrow:** leave at current values unless a symptom appears; these are **not**
optimiser knobs (they change *fidelity/stability*, and letting an optimiser touch them invites it
to "fix" a behaviour error with a numerical hack). Treat as `rarely`.
**Couples with:** everything, subtly — which is exactly why they're frozen during behaviour fits.

### J. Timing & latency
**Params:** #28 physics/render dt, #29 `/cmd_vel` rate, plus real-side command latency.
**Isolating test:** step-response alignment in T1/T2 — does the real rover start moving a fixed
delay *after* the command vs sim?
**Why:** the real rover has serial + controller latency (command → wheels) of tens of ms; the
sim reacts within a physics step. A pure dead-time offset shows as a **constant time shift**
between real and sim tracks that no gain parameter removes.

| Symptom | Move | Why |
|---|---|---|
| real motion **lags** sim by a constant offset | model a fixed command latency (delay the sim's `/cmd_vel`, or log the real latency and shift in `compare_paths`) | dead time is additive, not a gain; match it separately |
| sim contact unstable only at 60 Hz | keep physics at 240 Hz (already does) | substeps stabilise the six-wheel scrub contact |

**Range → narrow:** measure the real command→motion delay once (high-FPS camera on the first
frame of motion); apply as a fixed shift. Keep physics 240 / render 60. Match `/cmd_vel` publish
rate to whatever drives the real rover (20 Hz today).
**Couples with:** nothing gain-like — handle dead time *before* fitting `τ` (§D) so the ramp fit
isn't polluted by latency.

---

## 6. Identifiability & staging order

The parameters are **coupled** — several of them move the same feature. If you optimise them all
at once against one mixed trajectory, the optimiser finds a *degenerate* combination (e.g. large
`wheelRadius` + heavy slip looks like correct `wheelRadius` + no slip over one run). The fix is
to **identify in stages**, each stage on the test that isolates it, freezing what's already set:

```
STAGE 0  (once, measured)      mass, COM, inertia, geometry, physics dt, cmd latency
   │        → removes momentum/geometry/dead-time confounders from everything below
   ▼
STAGE 1  (once, high-grip)     wheelRadius           via T1 on CONCRETE (no slip regime)
   │        → locks the linear scale where friction can't contaminate it
   ▼
STAGE 2  (once, high-grip)     drive damping, maxForce   via T1-ramp + T5-coast on CONCRETE
   │        → locks motor ramp + torque cap where resistance is minimal
   ▼
STAGE 3  (once)                maxLinear/AngularSpeed    via T3 / yaw-sweep (set to real plateau)
   │
   ▼
FOR EACH SURFACE S (per-surface loop — the "on the fly" part):
   STAGE 4  floor µ_dyn/µ_stat        via T1 distance-deficit + launch feel      (§C)
   STAGE 5  wheel angularDamping      via T3 v_ss-sag + T5 coast (soft floors)    (§H)
   STAGE 6  wheelDistance             via T2 Δyaw  (AFTER µ — scrub depends on grip) (§B)
   STAGE 7  middle-µ ratio / COM trim via T2 turn-walk                            (§B/§G)
   STAGE 8  validate on T6 (square.yaml): endpoint + heading inside noise floor
```

Rationale for the order: Stage 1 and 2 happen on **concrete** precisely because high grip +
firm surface means **no slip and no rolling resistance**, so `wheelRadius`/`damping`/`maxForce`
are observed uncontaminated. Only then do you move to each test surface and let *friction* and
*drag* absorb the surface-specific differences, with `wheelDistance` re-fit **last** because its
value depends on the grip you just set.

**Coupling cheat-sheet (what fights what):**

- `wheelRadius ↔ floor µ`: both change straight-line distance → separate by surface (grip) + test regime.
- `wheelDistance ↔ floor µ`: scrub ∝ grip → always re-fit `wheelDistance` after µ.
- `damping ↔ maxForce ↔ mass`: all shape `τ` and coast → fit D+E together, with mass pinned.
- `maxLinearSpeed ↔ base_linear_damping`: flat clamp vs soft sag → use the shoulder shape to tell them apart.
- solver/contact (I): frozen during behaviour fits, on principle.

---

## 7. Per-surface floor-friction bands (enforced)

Rubber-on-floor Coulomb µ from material tables. With `frictionCombineMode = min` and both tyre
materials fixed at the envelope maximum (static 1.2 / dynamic 1.0), the **floor** value is the
effective tyre-floor coefficient. These bands are **hard limits**, defined once in
`calib_params.SURFACE_FRICTION`. The optimiser starts at the band midpoint and never searches outside it.
`apply_to_stage()` rejects any floor value outside the band (or dynamic > static), including in the
in-memory trial path and the bake.

| Surface (`--surface` / aliases) | µ_static band → start | µ_dynamic band → start | Source quality |
|---|---|---|---|
| **concrete** (`sealed_concrete`) | 0.60–1.00 → 0.80 | 0.50–0.85 → 0.675 | Published: rubber/dry concrete 0.6–0.85 (Engineering Toolbox, Misumi), 1.0/0.8 (Serway), 1.0/0.7 (HowWhy), 0.65/0.5 (ingdemurtas), 0.6 hard – 0.85 soft rubber (UF) |
| **carpet** (`light_carpet`, `heavy_carpet`, …) | 0.60–1.00 → 0.80 | 0.50–0.90 → 0.70 | No published table; robotics practice ~0.6 "decent", >0.9 "excellent" (anecdotal). Narrow with a tilt test. |
| **foam** (`eva`, `foam_padding`) | 0.70–1.20 → 0.95 | 0.50–1.00 → 0.75 | No published rubber-on-EVA value; EVA-backed grip tape 0.85–1.2 static (system value), EVA vs sled 0.5–0.7 kinetic. Narrow with a tilt test. |
| unknown name | 0.60–1.20 | 0.50–1.00 | Union of the above |

Chassis drag and drive damping are **fixed**, not per-surface knobs: the 2026-10-14 Isaac sweep showed
no measurable effect while the drives are velocity locks (see docs/PARAMETER_TEST_VALIDATION.md).
`wheelDistance` is re-fitted per surface **jointly** with floor µ and the torque cap (stage 4, spins + arcs).
Rover geometry (wheel radius, track, wheelbase, wheel placement, colliders) is **locked** and never a parameter.

---

## 8. The optimiser (ML) — automated calibration

The README's "step 6" — the piece to build. It closes the loop in §1 using the camera as cheap
ground truth. Design:

**8.1 Parameter vector θ (per stage, per surface).** Don't optimise all 29 at once — optimise
the small sub-vector for the current stage (§6), everything else frozen:

| Stage | θ (dims) | search space | scale |
|---|---|---|---|
| 1 | `wheelRadius` (1) | 0.03–0.05 | linear |
| 2 | `damping`, `maxForce` (2) | 50–20000 / 0.3–∞ | log |
| 3 | `maxLinear`, `maxAngular` (2) | read-off, not searched | — |
| 4 | floor `µ_dyn`, `µ_stat` (2) | per §7 band | linear, `µ_stat ≥ µ_dyn` constraint |
| 5 | wheel `angularDamping` (1) | 0–5 | linear |
| 6 | `wheelDistance` (1) | 0.25–0.9 | linear |
| 7 | middle-µ ratio, COM-x/z (≤3) | ±30 % | linear |

Low-dimensional per stage (1–2 dims mostly) → **the whole point of staging** is that each
sub-problem is cheap and identifiable.

**8.2 Loss.** Per stage, a weighted sum of the *features that stage controls* (not the raw
pointwise error, which smears effects together):

```
L(θ) = Σ_feature  w_f · ( (feature_sim(θ) − feature_real) / σ_f )²
```

where `σ_f` is the measured real-rover repeatability of that feature (§4) — so the loss is in
"sigmas", comparably scaled across features, and **bottoms out at ~1 when inside measurement
noise** (the natural stop). Stage-specific feature sets:

- Stage 1: `{distance, v_ss}`. Stage 2: `{τ_v, coast_dist}`. Stage 4: `{distance_deficit,
  launch_jerk}`. Stage 5: `{v_ss_sag, coast_dist}`. Stage 6: `{Δyaw, ω_ss}`. Stage 7:
  `{turn_walk_xy}`. Final validation (T6): `{endpoint_err, heading_err}` must fall inside noise.

**8.3 Optimiser choice.** Each evaluation = launch/step Isaac + drive a sequence (seconds to
tens of seconds), so evals are **expensive and few** → use a **sample-efficient black-box**
method, not gradient-free brute force or RL:
- **1–2 dim stages:** coarse grid (3–5 pts) → golden-section / bisection on the bracket. Done in
  <10 evals because the maps are monotone (§A, §B, §C, §F).
- **2–3 dim coupled stages (2, 4, 7):** **Bayesian optimisation** (Optuna TPE or a GP) — handles
  the mild coupling and the noise, and gives you an acquisition-driven narrowing that *is* the
  "sensible range first, then narrow" philosophy made formal. **CMA-ES** is the fallback if a
  stage turns out to be >3 dims or multimodal.
- You do **not** need a neural network or differentiable sim for this; the dimensionality is low
  and the budget argument favours the cheap, interpretable optimiser. (A learned surrogate /
  differentiable-sim approach is a *later* upgrade if you want single-shot calibration, but it's
  the "higher-expense technique" this project is explicitly avoiding.)

**8.4 Where ML genuinely helps (optional, later).** A small regression from **surface
appearance → friction/drag prior** trained on the camera's view of each mat (the overhead feed
already sees the floor): it predicts the §7 starting band from an image so a *new* surface skips
the coarse sweep and goes straight to narrowing. That's the ML lever that scales the rig to
unseen floors cheaply — but it sits *on top of* the optimiser above, which remains the ground
truth.

**8.5 Guardrails.**
- **Freeze non-stage params** (enforced, not trusted) so the optimiser can't fix a behaviour
  error with a numerical/contact hack (§I).
- **Constraints**: `µ_stat ≥ µ_dyn`; θ inside physical bands (a value outside is a modelling
  smell, not a win).
- **Generalisation check**: fit on T1–T5, *validate on T6*; never optimise directly on the
  validation sequence (that's overfitting the calibration to one path).
- **Stop** when validation features are within the noise floor (loss ≈ 1 σ) — chasing lower is
  fitting camera noise.
- **Determinism**: fixed physics dt + fixed sequence + fixed seed so an eval is repeatable; run
  the real rover N times for σ, the sim once per θ.

---

## 9. Build status

What exists vs the on-the-fly loop. Most of this is now **built** (2026-10-07); see
`optimize/README.md` for the runbook and `docs/MODELING.md` for the equations behind the loss.

| Need | Status | Where |
|---|---|---|
| Camera → grid → `(t,x,y,yaw)` ground truth | **built** | `trials/record_traj.py --source camera` |
| Sim `/odom` → `(t,x,y,yaw)` | **built** | `trials/record_traj.py --source odom` |
| Same command stream to both | **built** | `parallel/cmd_sequence.py` |
| Trajectory overlay + endpoint/heading/pointwise error | **built** | `trials/compare_paths.py` |
| **Per-segment feature extractor** (`v_ss`, `τ_v`, `ω_ss`, coast, walk, arc) | **built** | `optimize/features.py` (+ `--aggregate` → targets) |
| **Explicit per-surface floor material** bound to the ground | **built** | `calib_params.author_floor()`, bound at runtime by `run_ground_plane.py --calib` |
| **Runtime (in-memory) θ override** — try θ without re-baking the USD | **built** | `run_baked_rover.run(calibration=…)` / `run_ground_plane.py --calib` |
| Single source of truth for params (schema + prim map) | **built** | `ugv_controller_v2/calibration/calib_params.py` (shared by baker + runtime + optimiser) |
| Author **floor µ / drag / combineMode** (+ bake) | **built** | extended `sim_calibration.yaml` + `apply_sim_calibration.py` |
| **Optimiser** (staged, scipy + analytic warm-start; Optuna optional) | **built** | `optimize/calibrate.py` (`--self-test` validates it) |
| Analytic forward model (warm-start + sign of every sensitivity) | **built** | `optimize/forward_model.py` ≡ `docs/MODELING.md` |
| Isaac-in-the-loop trial backend | **built** | `optimize/sim_eval.py` (needs the GPU/rig) |
| **Per-surface config + results** (θ* per floor) | **built** | `optimize/calibrate.py` → `surfaces/<surface>.yaml` |
| Live-reset fast path (change runtime-mutable θ without relaunching Isaac) | *future* | speedup only; per-θ relaunch is correct today |
| Appearance→friction-prior regressor (skip coarse sweep on a new floor) | *future* | optional ML lever (§8.4) |

**Scope correction (2026-10-07):** geometry is **frozen** — physical wheel radius, track,
wheelbase, mass, inertia are measured from the real rover and never enter the search space
(`calib_params` gives them no bounds; `calibrate.py --measure-geometry` enters them). Note this
means `wheelRadius` is **not** a straight-line tuning knob: set controller `R = physical r`
(linear gain `k_v = r/R = 1`) and let slip/rolling-resistance/latency explain any residual — see
`docs/MODELING.md` §3. `wheelDistance` is **not** geometry (physical track + skid-steer slip
compensation), so it stays calibrated per surface.

---

## 10. Worked example end-to-end

**Observation (real, heavy carpet, T1 `lin 0.12 × 3 s`):** real rover travels **0.27 m**; sim
travels **0.36 m**; in the sim video the wheels are **not** spinning faster than the ground
(no visible slip); real `v_ss` curve from T3 **sags** below concrete.

**Diagnose with the map:**
1. Distance deficit → first suspect `wheelRadius` (§A) **or** slip (§C) **or** resistance (§H).
2. "No wheel-spin" rules out slip (§C) — a friction fix would *add* spin, wrong signature.
3. `wheelRadius` was already locked on concrete in Stage 1 (§6) and reproduced concrete distance
   → it's not the scale.
4. `v_ss` sags with a soft shoulder on carpet → **rolling resistance (§H)**, the
   carpet-pile energy loss that Coulomb contact doesn't model.

**Act:** raise wheel `angularDamping` (#21) on the carpet surface config. Coarse: try
`{0.5, 1.5, 3.0}`; `1.5` brings sim T1 distance to 0.28 m and matches the T3 sag shoulder;
bisect `1.5–3.0` → `2.0` nails both T1 (0.27 m) and the T5 coast. Then **re-fit `wheelDistance`**
(§B, Stage 6) because the added drag slightly changed the scrub, and **validate on T6** — endpoint
error 1.1 cm, heading 1.4°, both inside the ~1.5 cm / 2° camera noise floor → **done for heavy
carpet.** Record `θ*` to `surfaces/heavy_carpet.yaml`.

**Why this is the right fix and "lower the friction" was wrong:** both would have shrunk the sim
distance, but only drag reproduces it *without* introducing wheel-spin and *with* the correct
speed-sag shoulder — i.e. it matches the real mechanism (energy loss), not just the one number
(distance). That distinction — matching the *signature*, not just the *scalar* — is the whole
point of the map.
