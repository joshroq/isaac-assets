# optimize/ — automated sim→real calibrator

Closes the loop from `PARAMETER_MAP.md` / `docs/MODELING.md`: drive the same command sequence on
the real rover (overhead-camera ground truth) and in Isaac, extract per-maneuver features, and
search the **behavioural** sim parameters (friction, drag, drive limits, effective track) so the
sim matches the real rover — **in stages**, using straight, spin, arc and stopping features together.

**Physical geometry is locked.** Wheel radius, physical track, wheelbase, wheel placement
and collider shapes are never authored by calibration. `wheel_distance` is the controller's
turning gain and remains searchable. Base/wheel masses are measured inputs; authoring a
measured mass rescales its inertia.

## Searchable parameters and tests

All **16 non-geometric motion parameters** are searchable with `--backend isaac`:

| Stage | Parameters | Scored tests/features |
|---|---|---|
| 2 — drive and ramps | `drive_damping`, `max_acceleration`, `max_deceleration`, `max_angular_acceleration` | T1 acceleration/speed/stopping, T5 stopping, T2 spin startup/stopping |
| 3 — speed limits | `max_linear_speed`, `max_angular_speed` | T3b highest forward speed and T6 highest yaw rate, or measured `top_speed` read-off |
| 4 — turning/contact | `wheel_distance`, `drive_max_force`, `corner_mu_static`, `corner_mu_dynamic`, `middle_mu_static`, `middle_mu_dynamic`, `floor_mu_static`, `floor_mu_dynamic` | T2 spin rate/angle/drift, T4 arc radius, T6 per-step yaw rates, T1/T3 forward response |
| 5 — chassis damping | `base_linear_damping`, `base_angular_damping` | T1/T3 speed, T1/T5 stopping, T2/T6 spin stopping, T4 arc radius |

Wheel friction, drive damping and chassis damping were restored after being marked fixed.
Corner-wheel dynamic/static bounds are **0.5–1.5 / 0.5–1.8**; middle-wheel bounds are
**0.3–1.2 / 0.3–1.4**. Drive damping searches **1e-6–20000 logarithmically**, including
the low-gain regime that the old 50–20000 range missed. Both chassis damping values search
**0–8 s⁻¹**. Floor coefficients retain their surface-specific bands. Every material requires
static friction >= dynamic friction.

The reduced analytic model does not faithfully represent the seven restored parameters.
They remain at their starting values with `--backend analytic`; the Isaac backend fits them.
Under `--combine-mode min`, wheel coefficients above the floor are locally invisible, so
initial Isaac search vertices explore below the floor coefficient where bounds permit. Wheel/floor
coefficients can still trade off, and stiff drives can mask chassis damping: a good trajectory
fit does not guarantee a unique value for every parameter. The combine mode is an explicit
run setting, not a searched numeric parameter.

## Files
| file | role |
|---|---|
| `../../ugv_controller_v2/calibration/calib_params.py` | single source of truth: param schema + prim map + `apply_to_stage` (shared by baker, runtime override, optimiser) |
| `forward_model.py` | reduced-order analytic model (the MODELING.md equations) — warm-start + offline self-test |
| `features.py` | extract `v_ss, dist, tau_v, coast_*, omega_ss, dyaw, turn_walk, arc_radius` from a `(t,x,y,yaw)` track; `--aggregate` → targets |
| `calibrate.py` | staged optimiser (scipy; analytic warm-start → Isaac refine); measured-mass prompt; `--self-test` |
| `sim_eval.py` | Isaac backend: launch headless Isaac with θ in-memory, drive the baked graph over `/cmd_vel`, record `/ugv_sim/odom`, tear down |
| `../targets/*.yaml` | per-surface real feature targets + sequences (example included) |
| `../surfaces/*.yaml` | optimiser output per surface (measured masses + calibrated θ + a ready-to-bake `sim_calibration_yaml`) |

## 0. One-time: enter measured masses
```bash
python3 calibrate.py --measure-mass            # base/wheel masses -> measured.yaml
```

## 1. Validate the machinery with NO Isaac / NO real data
```bash
python3 calibrate.py --self-test               # recovers known params from synthetic targets
```

## 2. Collect real trials per surface and build targets
On each surface (concrete, light/heavy carpet, foam), drive the battery (PARAMETER_MAP.md §4)
with the real rover and record camera ground truth (repeat N≥5 for σ):
```bash
# real rover driving per trials/trial_real.sh; then aggregate the repeats into targets:
python3 features.py --aggregate run1.csv run2.csv ... --sequence ../parallel/sequences/straight.yaml
```
Assemble a `../targets/<surface>.yaml` (see `../targets/example_heavy_carpet.yaml`): per test a
`sequence` (inline steps or path) and `features: {name: {mean, sigma}}`, plus a read-off
`top_speed: {linear, angular}` from T3b (linear) and T6 (angular).

## 3. Run the calibration
```bash
source ../../ros2env.sh        # Isaac backend needs ROS 2 + a running-capable GPU

# reference surface (concrete): fit all stages, then save as a base
python3 calibrate.py --surface concrete --targets ../targets/concrete.yaml \
    --backend isaac --stages 2,3,4,5 --out ../surfaces/concrete.yaml

# each other surface: fit the per-surface stages, seeded from the concrete base
python3 calibrate.py --surface heavy_carpet --targets ../targets/heavy_carpet.yaml \
    --backend isaac --stages 4,5 --base ../surfaces/concrete.yaml \
    --out ../surfaces/heavy_carpet.yaml
```
`--backend analytic` (default) runs the represented subset without Isaac. Use `--backend isaac`
to fit all 16 parameters. `--max-evals` bounds the Isaac refine per stage; each evaluation may
run several sequences. Omit `--stages` to run every stage.

## 4. Use the result
`../surfaces/<surface>.yaml` contains a `sim_calibration_yaml:` block. Either:
- **runtime (per surface, no bake):** `isaacpy ../../ugv_controller_v2/run_ground_plane.py --headless --calib <that block as a yaml> --combine-mode min`
- **bake it in (one room/surface):** copy the block into `../../ugv_controller_v2/calibration/sim_calibration.yaml` and run `run_apply_calibration.sh` (add `--bake-floor` only for an ENV USD with a ground collider).

## Notes
- The default Isaac backend uses the native v2 ROS 2 graphs. Both processes use
  `ROS_LOCALHOST_ONLY=0`, Fast DDS and the same `ROS_DOMAIN_ID`; `source ros2env.sh`
  sets the matching terminal environment. A terminal inheriting
  `ROS_LOCALHOST_ONLY=1` cannot discover this bridge. The actual odometry topic is
  `/ugv_sim/odom` (override with `CALIB_ODOM_TOPIC` or `run_trial(odom_topic=...)`).
- Sequence durations and CSV timestamps use simulation time from odometry,
  so a slow renderer does not shorten the calibration maneuver. Readiness requires
  advancing odometry samples and a `/cmd_vel` subscriber, rather than topic names.
- Each trial retains `calib.yaml`, `seq.yaml`, and `isaac.log`. Startup, stalled
  odometry and process failures raise errors with the log path; teardown stops only
  that trial's process group. No global ROS daemon or shared-memory cleanup is needed.
- `run_trial(..., mode="direct")` retains the explicit ROS-free physics diagnostic.
  Normal optimizer and demo calls use ROS by default.
- Run a single native trial with
  `trials/trial_sim.sh <sequence.yaml> <out.csv> [calib.yaml]` (paths relative to
  `sim2real_calib/`). `SIM_TRIAL_MODE=direct` explicitly selects the diagnostic.
  Lifecycle regressions run without a GPU:
  `python3 -m unittest discover -s sim2real_calib/optimize -p 'test_*.py' -v`
  from the repository root in a ROS-sourced shell.
- Each Isaac eval relaunches the sim (all physics params are applied pre-play); the analytic
  warm-start keeps the number of expensive refine evals small. A live-reset fast path (change
  only runtime-mutable params without relaunch) is a future speedup.
- `optuna`/`cma` are optional; the default backend is `scipy` (`minimize_scalar` for 1-D stages,
  `differential_evolution` for coupled stages), which is already installed.
- The loss is in units of real-rover σ, so it bottoms out at ≈1 when inside camera noise — don't
  chase lower (that fits measurement noise).
