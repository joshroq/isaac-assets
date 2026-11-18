# sim2real_system: the rover + physics optimiser system, start to finish

This folder contains a snapshot of the rover + optimiser system, grouped in the order
the system runs.

**The simulated rover always drives its wheels.** `/cmd_vel` goes to the baked skid-steer
`Drive_Graph`, which sets the four corner wheels' joint velocity drives; the rover moves
only through PhysX wheel-floor contact. The optimiser tunes the physics of that drive and
contact (drive ramps and torque limit, speed caps, wheel and floor friction, effective
wheel distance, chassis damping). Geometry and masses are locked.

**All files are real, independent copies.** The numbered folders include the source files,
configuration, rover USD, and the full contents of the saved `stations` directory.
You can upload this entire `sim2real_system` folder directly using OneDrive's browser
folder-upload option. No other local folders are needed to view the included files.

Editing a file here does not change the original project. `make_links.py` retains its
historical name but now refreshes **copies**, never symlinks. Run it from the original
repository with `python3 sim2real_system/make_links.py`; it overwrites the snapshot entries
with the current source versions.

This is a browsable source snapshot. The execution commands below refer to the original
repository layout, where Python imports, relative paths, ROS, and Isaac Sim are configured.
The numbered folders are not a standalone runtime installation. Generated run data is not
bundled; it remains in the original `sim2real_calib/loop/runs/`.

```
 Pi camera ──UDP 5000──► 01 receive ─► 02 grid homography + stickers ─► rover pose (x, y, yaw) in metres
                                                                                    │
 Rover (Jetson) ◄─ssh─ 03 rover_agent / rover_link ◄─ 04 homing to start stations ◄─┤
        │  runs a 05 command sequence (cmd_vel)                                     │
        ▼                                                                           ▼
 real track rep*.csv (t, x, y, yaw) ──► 06 features (speeds, turn rates, ramps, coasts) ──► z vs sim
                                                                                    │
 07 physics parameters (calib_params) ◄── 07 staged search (bounded / Nelder-Mead) ◄┘
        │
        ▼
 08 Isaac Sim: wheel drives + PhysX contact ──► sim track ──► 06 the same scoring
        │
 09 closed_loop.py runs all of the above per iteration; 10 verify_params.py checks held-out trials
```

---

## Run it, start to finish

All commands start from `~/projects/isaac-assets` after `source ros2env.sh`, with
`cd sim2real_calib`. The full checklist is `00_environment/RUNBOOK.md`.

| Step | What | Command (file in this folder) |
|---|---|---|
| 0 | ROS environment and Isaac launcher | `source ros2env.sh` (00); Isaac runs through `isaacpy_clean.sh` (00) |
| 1 | Overhead camera streaming | Pi runs `stream_camera.sh` as a service (01). Check with `python3 capture/receive.py` |
| 2 | Grid calibration check | `python3 calib/calibrate_grid.py --verify --source stream --save /tmp/v.png` (02) |
| 3 | Rig self-check | `python3 loop/selfcheck.py` (09) |
| 4 | Sticker tracking check | `python3 calib/verify_stickers.py` (02) |
| 5 | Record the real battery (training + held-out) | `python3 loop/closed_loop.py --surface concrete --real-only --iterations 2 --repeats 3 --tests T1,T2,T3,T3b,T4,T5,T6,R1,R2,R3,R4,R5` (09) |
| 6 | **Staged physics fit**, all 16 keys | `bash loop/staged_physics.sh loop/runs/<session>/iter_01/real concrete` (07) |
| 7 | Held-out verification + report | `python3 loop/verify_params.py --surface concrete --calibrated surfaces/concrete_physics.yaml --combine-mode average --tests ... --real-from loop/runs/<session>/iter_02/real` (10) |
| 8 | Path errors in cm and deg | `python3 optimize/path_errors.py <real dir> --sim NAME=DIR ...` (06) |
| alt | Closed loop with the rover in it (few keys) | `python3 loop/closed_loop.py --surface carpet --keys wheel_distance,drive_max_force --tests T1,T2,T4 --iterations 2` (09) |

Steps 5 and the closed loop also take `--resume <session>`. Without hardware,
`--test-fake-rover` runs a pure-Python fake rover (`11_tests/fake_rover.py`).
`optimize/run_isaac_battery.py --calib <surface yaml>` (08) replays the battery in Isaac with
any parameter set, no rover needed.

---

## What each folder holds (read in this order)

### 00_environment

* `ros2env.sh`: ROS 2 environment for the desktop.
* `isaacpy_clean.sh`: launches Isaac's Python with its bundled ROS and no system ROS.
* `RUNBOOK.md`: the preflight and run checklist.

### 01_overhead_camera

The Logitech C920e on a Raspberry Pi streams to the desktop on UDP 5000.

* `stream_camera.sh`, `sim2real-camera.service`, `install_on_pi.sh`: run on the Pi.
* `stream.py`: `LatestFrame`, the desktop receiver. It always returns the newest frame.

### 02_grid_and_sticker_calibration

**Pixels → metres.**

* `grid_target.py: detect()` (l.342) finds the taped 1 m grid; `solve()` (l.374) fits the
  homography. The border tape sits *inside* the square; the tape is exactly 25.4 mm wide.
* `calibrate_grid.py` fits the homography `H` (image ↔ floor, metres), saved to `grid_calib.json`.
* `sticker_tracker.py: StickerTracker` (l.48) finds three coloured stickers on the rover,
  corrects for their height (`plane_homography()` l.101), and recovers the pose with
  `fit_pose()` (l.26), a Kabsch/Umeyama rigid fit giving x, y in metres and yaw in radians.
* `verify_stickers.py` checks the known sticker spacings against what the camera measures.

### 03_real_rover_link

* `rover_link.py: RoverLink` (the desktop side) starts `rover_agent.py` on the Jetson over
  SSH and sends velocity commands and command sequences. It also handles link stalls,
  base-driver restarts, and `reopen()` when the rover drops off the network.
* `start_bringup.sh` / `stop_all.sh` (on the rover) start and stop the Waveshare base driver,
  the lidar and the odometry.

### 04_pose_tracking_and_homing

Putting the rover at the same start pose every trial.

* `camera_track.py: CameraTrack`: the camera pose stream.
* `fusion.py: PoseFuser`: camera fused with the gyro and odometry between frames.
* `homing.py`: drives to a goal pose.
* `stations.py`: the taught start pose per test, refined by lidar scan matching (`icp2d.py`).
* `camera_guard.py`: refuses to run if the camera moved.
* `loop_config.yaml`: every rig setting (latencies, tolerances, battery floor, scored features).

### 05_test_battery

The 12 command sequences: lists of `(linear m/s, angular rad/s, duration s)`.

| Test | Isolates |
|---|---|
| T1 straight | speed, distance, speed ramp |
| T2 spin | in-place turn rate and total turn, drift during a spin |
| T3 speed sweep | steady speed at four commanded speeds |
| T3b top speed | the linear speed cap |
| T4 arc | turning while driving |
| T5 stop | coast time and distance after a stop |
| T6 spin sweep | turn rate at four commanded turn rates |
| R1-R5 | the Nav2/teleop envelope: straights f/b, spin rates, slow and fast arcs in four directions, a Nav2-like pattern |

### 06_measurement_features_and_error

**How trajectories become numbers and errors.** See the section below.

* `features.py: extract()` (l.114) and `aggregate()` (l.241); `build_targets.py`;
  `compare_paths.py`, `path_errors.py: pair_errors()` (l.55); `response_map.py`
  (command → achieved twist, per step).

### 07_physics_model_and_optimizer

* `calib_params.py`: **the single source of truth** for every physics parameter: schema,
  bounds, per-surface floor-friction bands, which keys are locked/measured, and
  `apply_to_stage()` (l.478), which authors the values onto the USD stage before play.
* `apply_sim_calibration.py`: bakes a parameter set into the USD.
* `MODELING.md` + `figures.tex/pdf`: the equations and free-body diagrams: actuator (PhysX
  velocity drive with a force limit), wheel-ground contact (friction cone, combine modes),
  skid-steer kinematics with an effective wheel distance, chassis Newton-Euler with damping,
  and the signed sensitivity of every feature to every parameter.
* `PARAMETER_MAP.md`: every parameter ↔ the test that isolates it ↔ which way to move it.
* `calibrate.py`: `STAGES` (l.56) groups the keys by the tests that can identify them;
  `Objective` (l.149); `refine_isaac()` (l.250); `write_surface()` (l.410).
* `forward_model.py: simulate()` (l.105): a reduced analytic model of the same physics,
  used only to warm-start (`closed_loop.py --warm`).
* `staged_physics.sh`: the five-stage fit (A ramps, B caps, C traction, D damping, E joint).
* `concrete_physics.yaml`: the full 16-key concrete calibration; `carpet_loop.yaml`: the
  carpet closed-loop calibration.

### 08_isaac_simulation

* `run_baked_rover.py`: loads `jetson_rover_v2.usd`, applies the parameter set
  (`calib_params.apply_to_stage`), and steps physics. `/cmd_vel` → `Drive_Graph` → wheel
  joint velocity drives → PhysX contact.
* `run_ground_plane.py`: the calibration world. `run_small_warehouse.py` /
  `run_obstacle_warehouse.py`: the Nav2 worlds.
* `sim_eval.py: run_trial()` (l.158) launches Isaac, replays a sequence on `/cmd_vel`, and
  records `/ugv_sim/odom`; it clears stale Fast DDS shared memory first.
* `sim_batch.py` runs a whole battery in one launch. `run_isaac_battery.py` is its CLI.

### 09_closed_loop_orchestrator

* `closed_loop.py`: the whole iteration.
  * `real_trial()` (l.380): home, run, record, re-time; `alignment_offset()` (l.462).
  * `build_targets()` (l.590) and `score()` (l.608): the rms z below.
  * `BatchObjective` (l.626): one Isaac launch per candidate, the whole battery at once.
  * `optimise()` (l.716): bounded 1-D search, or Nelder-Mead in a trust box that shrinks
    ×0.6 per iteration.
  * `compare_png()` (l.883), `path_maps()` (l.780): path plots.
* `dashboard.py`: the live window; `param_sheet.py`: the parameter panel.
* `selfcheck.py`: preflight.

### 10_verification_and_report

* `verify_params.py`: scores the baseline and the calibrated parameters against the same
  **held-out** real trials and writes `report.md` (rms z, per-feature z, path errors).
* `report.md`: the concrete held-out report (2026-11-12).

### 11_tests

* `pytest` files, plus script tests `test_stations_sim.py`, `test_agent_local.py`,
  `test_reopen.py` (run them with `python3`), and `fake_rover.py`.

### 12_nav_comparison

The navigation-level test of the optimiser's results: the real rover and the Isaac rover
(calibrated physics / un-calibrated) drive the same course under the same lidar or
lidar+depth Nav2 stack, and the report scores each simulator against the real rover's own
repeat scatter (rms z, path gap ratio). Start with its `README.md`.
Entry point `session.py`; `--fake-real` runs it with no rover (Isaac with deliberately
perturbed physics plays the real rover).

---

## How the error between the rover's trajectory and the sim's is determined

There are **two different error measures**. Keep them apart.

### Step 0: the trajectory itself has units

Each real trial is a CSV `rep*.csv` with columns `t` [s], `x` [m], `y` [m], `yaw` [rad]:

* sticker pixels → floor metres through the grid homography (02);
* time re-aligned so `t = 0` is when the rover starts moving (`alignment_offset()`), removing
  the camera and link lag (concrete 0.58 s camera, carpet 0.71 s, + 0.15 s link).

Each sim trial is the same format, from Isaac's odometry (`sim_eval.py: run_trial()`).

### Error A: direct path comparison, in cm and degrees

1. Put both paths in their own start frame (start pose at the origin, heading along +x).
2. Resample both on one time grid.
3. Pointwise distance `d(t) = sqrt((x_r − x_s)² + (y_r − y_s)²)`.
4. Report endpoint error `d(T)`, mean and max of `d(t)` [cm], and final heading error
   `|wrap(yaw_r(T) − yaw_s(T))|` [deg].

`path_errors.py` does this for every real × sim pair and reports the median, plus real ×
real as the floor. It is reported, not optimised.

### Error B: what the optimiser minimises (rms z, dimensionless)

1. **Features** (`features.py: extract()`), per trial, e.g. steady speed `v_ss`, distance
   `dist`, speed time constant `tau_v`, coast time/distance, spin rate `omega_mean`, total
   turn `dyaw`, turn rates of a sweep `omega_s*`, top speed `v_ss_max`, and for the R tests
   the signed per-step speed `v_k<i>` and turn rate `w_k<i>` over the settled last 40 % of
   each step. `loop_config.yaml: scoring` picks which ones count per test.
2. **Real targets** (`aggregate()`): mean `m` and standard deviation `s` over the real
   repeats.
3. **z per feature** (`closed_loop.py: score()`):

```
σ     = sqrt( s² + (ε_f · |m|)² )        ε_f = Isaac's own run-to-run relative scatter
z     = (sim − m) / σ
rms z = sqrt( mean of z² )               a missing feature counts as z² = 25
```

rms z counts **how many of the real rover's own trial-to-trial scatters** the sim is off by,
so ≈ 1 means the sim sits inside the rover's own repeat spread.

### Result on concrete (held-out, 12 tests × 3 fresh trials)

All 16 physics keys fitted in stages on iteration 1, scored on iteration 2:

| | rms z |
|---|---|
| schema defaults (as-shipped physics) | 4.97 |
| staged physics fit (`concrete_physics.yaml`) | **2.41** |

What changed most: `wheel_distance` 0.5 → 0.319, `drive_max_force` 50 → 8.85,
`max_angular_acceleration` 20 → 5.86, chassis damping 0 → 3.6 / 0.05 → 3.6. Biggest gains:
top speed (z −11.0 → −3.3), spin sweep (z up to 33.6 → 8.5), Nav2 pattern mean path error
16.5 → 7.2 cm. Still open: spins are bistable in Isaac on concrete (T2 flips between too
fast and too slow run to run), and arcs (T4 ≈ 26 cm endpoint) are not improved: the
wheel-contact sim turns too little while driving forward.
