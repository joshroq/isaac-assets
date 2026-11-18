# loop/ — fully automated closed sim2real loop

Drives the **real** rover and the **Isaac** rover through the same test battery,
compares them, **updates the sim parameters**, and repeats with no human in the loop.
The real rover **returns itself to each test's start pose** using overhead-camera
feedback, so repeats and iterations run unattended.

It reuses the existing pieces without modifying them: `parallel/sequences/*.yaml`
(tests), `optimize/features.py` + `build_targets.py` (features and latency alignment),
`optimize/calibrate.py` (loss weighting, bounds, analytic warm start, `write_surface`),
`optimize/sim_eval.py` (Isaac trials), `trials/compare_paths.py`, `calib/` +
`viz/` (camera tracking). `demo/realtime_demo.py` and `optimize/calibrate_live.py` are
untouched and still work as before.

```
           ┌──────────────── iteration k ────────────────┐
 REAL  ──► │ for test × repeat: RETURN to start → drive   │  camera = ground truth
           │ targets = feature mean ± sigma over repeats  │
 SIM   ──► │ current θ on the battery ("before")          │  1 Isaac launch / θ
 UPDATE ─► │ optimiser over θ keys (trust region)         │  ~80–100 s per candidate
 VERIFY ─► │ best θ on the battery ("after") + path maps  │
           └──────────── θ seeds iteration k+1 ───────────┘
```

## Files

| file | runs on | role |
|---|---|---|
| `rover_agent.py` | **rover** | persistent command agent over one SSH pipe: deadman'd velocity, locally-timed sequences, lidar obstacle stop, telemetry (`/odom`, `/odom_rf2o`, bias-corrected gyro, `/scan` on request, `/voltage`); zeroes on link loss / EOF / bad command / signals |
| `rover_link.py` | desktop | `RoverLink`: stages + talks to the agent, heartbeats, starts `start_bringup.sh` (held-open ssh) if the base driver isn't up |
| `camera_track.py` | desktop | `CameraTrack`: background sticker-pose tracker on the UDP stream, recording (+ raw video), 2-sticker occlusion fallback |
| `stations.py` | desktop | **teach & repeat start stations**: camera coarse + lidar fine + acceptance gate; `teach/goto/check/verify/list` |
| `icp2d.py` | desktop | 2-D scan matcher for the station lidar fingerprints (self-test included) |
| `fusion.py`, `localizer.py` | desktop | camera + gyro/rf2o odometry fusion; `FusedTracker` is what homing steers on |
| `camera_guard.py` | desktop | detects/corrects a moved overhead camera against a reference floor image |
| `selfcheck.py` | desktop | preflight of every dependency; `--latency` measures camera lag |
| `tests/` | desktop | `fake_rover.py` (ROS stand-in: carpet slip, biased gyro, raycast LD19, optional mirrored lidar) · `test_stations_sim.py` (teach/verify/biased camera/obstacle/latency) · `test_lidar_convention.py` · `../test_agent_local.py` (agent safety). `closed_loop.py --test-fake-rover` runs the whole loop vs the fake rover + real Isaac |
| `homing.py` | desktop | **return function**: camera-feedback go-to-pose (pre-goal → face → straight approach → trim), latency compensation, deadband adaptation, dead-motor detection |
| `sim_batch.py` | desktop | runs a whole battery in ONE Isaac launch and splits it per test |
| `closed_loop.py` | desktop | the orchestrator |
| `loop_config.yaml` | — | workspace fence, homing gains, per-test start poses, battery floor, keys to update, scored features |

## Reproducible start spots (read this first)

Every trial starts from a **taught station**, not just a camera coordinate. Five
independent layers make "the same spot" hold across trials, iterations and days:

| layer | file | guards against |
|---|---|---|
| **Lidar fingerprint per station** (teach & repeat) | `stations.py`, `icp2d.py` | camera calibration error / camera bumped: the LD19 scan of the room at the taught spot is matched (trimmed point-to-line ICP, ~2 mm / 0.1° on synthetic rooms) and the residual displacement is driven out. The camera is only used for the *relative* correction move, so its absolute error cannot shift the spot. |
| **Camera drift guard** | `camera_guard.py` | the overhead camera moving: each session / iteration registers the live image against a reference image of the floor grid (ORB + RANSAC, rover masked out). Moves of 2 px or more are measured and the grid homography is corrected in memory; moves of 25 px or more are refused (re-calibrate). |
| **Camera + odometry fusion** | `fusion.py`, `localizer.py` | camera lag (~0.2 s) and sticker occlusions: the controller steers on a pose propagated by gyro yaw (immune to spin slip) + rf2o lidar odometry, anchored by latency-shifted camera fixes. |
| **Acceptance gate** | `stations.goto_station` | sloppy starts: rover stationary (odometry), lidar error ≤ 2 cm / 1.5°, camera error reported, camera-vs-lidar disagreement flagged. Rejected starts are retried; every trial stores its gate record. |
| **Obstacle stop + preflight** | `rover_agent.py` (`guard`), `selfcheck.py` | hitting furniture (the agent blocks translation toward anything within 10 cm of the moving face and aborts a running trial); running with a dead sensor, low battery or moved camera; a **mirrored / rotated lidar convention** (`--motion` drives ~9 cm + turns ~25° and compares lidar scan-matching with the camera; a wrong convention would make station corrections push the wrong way). |
| **ROS isolation check** | `rover_link.py` | the rover's ROS being networked (e.g. the live-view comms toggle in `local_ros_env.sh`): the desktop's Isaac publishes `/cmd_vel` during every sim evaluation, which would then also drive the real rover. The link refuses to run unless the rover is `ROS_LOCALHOST_ONLY=1` without a discovery server. |

The summary of each loop session includes a **start-pose repeatability** table (lidar
error mean/max, camera spread, yaw sd per test) so the reproducibility is measured, not assumed.

### One-time setup (and after moving furniture or the camera)

```bash
python3 calib/calibrate_grid.py --verify --source stream   # FAIL -> recalibrate:
python3 calib/calibrate_grid.py --source stream --size 1 --spacing 0.25
#   (re-saves the camera_guard reference itself; a reference older than the grid
#    calibration is refused as stale - re-save it with loop/camera_guard.py --save-reference)
python3 loop/selfcheck.py --motion                  # sensors, camera lag, lidar convention
#   (if it prints a suggested stations: lidar_flip / lidar_offset, put it in loop_config.yaml)
python3 loop/stations.py teach --all                 # drive to each config start pose, record
python3 loop/stations.py verify --reps 5             # measure return repeatability
```

`teach T2_spin --here` records wherever the rover sits (e.g. hand-placed on a taped
mark). Re-teach a station if its lidar view changes (furniture moved): `check` shows the
lidar error and fit quality from the current spot to every station.

## Run it

Prereqs: Pi camera streaming (nothing else holding UDP 5000), grid + stickers
calibrated, rover powered (on a charged pack — see below) and reachable as `ssh rover`.
The base driver is started for you if it isn't running.

```bash
cd ~/projects/isaac-assets && source ros2env.sh && cd sim2real_calib

# full loop: T1 straight, T2 spin, T4 arc; 3 real repeats each; 3 iterations
python3 loop/closed_loop.py --surface carpet

# options
python3 loop/closed_loop.py --surface carpet --iterations 4 --repeats 3 --max-evals 12
python3 loop/closed_loop.py --surface carpet --keys wheel_distance --tests T2,T6
python3 loop/closed_loop.py --surface carpet --init ../surfaces/heavy_carpet.yaml
python3 loop/closed_loop.py --surface carpet --real-only        # drive + record only
python3 loop/closed_loop.py --surface carpet --real-from loop/runs/<session>/iter_01/real  # no rover
# every physics key, staged (A ramps, B caps, C traction, D damping, E joint) on recorded
# real trials - no rover needed; ~5 h, run it detached
bash loop/staged_physics.sh loop/runs/<session>/iter_01/real concrete
```

**One window for the whole session** (`viz/dashboard.py`; `--no-show` to run without it).
It opens once and stays responsive through every phase, including the long Isaac and
optimiser phases. The banner names the current phase (real trials, homing, Isaac
eval *n*, comparing). The window shows:

- the live camera, including while the rover drives itself back to each start
  station;
- the current real trial top-down;
- one gallery tile per test, which updates after every run:
  - when the real repeats finish, it shows them;
  - after each Isaac evaluation, it adds sim BEFORE (red), the latest optimiser
    candidate (grey) and the best so far (blue), with the rms z;
  - at the end of the iteration, it shows real vs BEFORE vs AFTER with the endpoint
    error.

Above the ticker, **SIM PARAMETERS BEING FITTED** lists every key the loop is updating.
For each key it shows:

- the value in use, its start value and the best so far;
- a range bar with the key's bounds, this iteration's trust region (shaded), the current
  value (diamond), the latest candidate (grey) and the best (blue).

**p** opens the full parameter sheet over the camera panel. It shows the fitted keys
with their per-iteration history, then every other sim parameter grouped as: calibrated
but held this session, measured masses, and locked geometry (wheel radius, physical
track, wheelbase). `wheel_distance` is also shown as D/B, its ratio to the physical
track; see `docs/MODELING.md` eq. 4.3 for why that ratio, not D itself, is the
physically meaningful number. The session log scrolls in a ticker. When the loop ends the window stays up until you
press q (`--no-hold` to exit right away). q / Esc mid-run interrupts the loop like
Ctrl-C (rover stopped). `live.jpg` in the session folder mirrors the window every
second.

Pieces on their own:

```bash
python3 loop/selfcheck.py                     # preflight (rover doesn't move)
python3 loop/stations.py list | check | goto T1_straight
python3 loop/camera_guard.py --check [--write]
python3 loop/rover_link.py --check            # link + base driver + battery voltage
python3 loop/homing.py --where                # camera pose of the rover
python3 loop/homing.py --test T1_straight     # return to a test's start pose
python3 loop/homing.py --goal 0.5,0.5,90      # any pose (grid m, deg)
```

## Outputs — `loop/runs/<surface>_<stamp>/`

- `summary.md`, `progress.png` — rms feature error before/after per iteration and the
  parameter trajectory; `session.log`; `live.jpg` (1 Hz snapshot of the session window).
- `iter_NN/real/<test>/repK.{csv,mp4,json}` — aligned camera track, raw video, homing
  error + battery; `repK_raw.csv`, `repK_wheel_odom.csv`, `repK_frames.csv`.
- `iter_NN/<test>/paths_map.{png,mp4}` — **real vs sim-before vs sim-after drawn on the
  real overhead camera view** (sim placed at the real start pose via the grid homography);
  `compare_{before,after}.png` + `metrics_*.json` (start-aligned path errors).
- `iter_NN/opt/evals.csv` + `eval_*/` (each candidate's θ, loss, Isaac log, tracks).
- `iter_NN/theta.yaml`, `iteration.json`, `targets.json`.
- Final calibration → `surfaces/<surface>_loop.yaml` (new file; existing surfaces untouched),
  with the ready-to-bake `sim_calibration_yaml` block.

## Where results go

Every session is kept in its own timestamped folder; nothing is overwritten:

- demos go to `demo/runs/<surface>_<stamp>/`, with `<surface>_latest` linking to the
  newest;
- optimizer sessions go to `loop/runs/<surface>_<stamp>/`.

`RESULTS.md` (in `sim2real_calib/`) catalogs every session of both kinds, newest first,
with links. Each session folder holds:

| File | What |
|---|---|
| `index.md` | start here: settings, summary, videos, then per test the graphics (embedded), metrics and data links |
| `manifest.json` | the same, machine-readable |
| `session_video.mp4` | recording of the whole dashboard window (8 fps, 1280x720), for presenting |
| `final_view.png` | the dashboard at the end (all gallery tiles) |
| `config/` | copies of the grid/sticker calibration, loop config, sim parameters and sequences actually used |
| per test (demo: `<test>/`; optimizer: `iter_NN/<test>/`) | real camera video (`real_camera.mp4` or `real/<test>/rep*.mp4`), paths drawn on it (`paths_map.png/.mp4`), comparison plots, command->response maps, path CSVs, metrics JSON |

The optimizer additionally keeps `progress.png`, `summary.md`, `session.log`,
`iter_NN/theta.yaml` and every Isaac
evaluation under `iter_NN/opt/`.

## How the update works

- **Loss** = Σ ((sim − real_mean) / σ)², σ = real repeat spread ⊕ measured Isaac
  run-to-run noise (same as `calibrate.Objective`). `rms z` ≈ 1 means the sim sits inside
  the real rover's own scatter; the loop stops early at `--converge` (1.2).
- **Before** is scored on the iteration's *fresh* real repeats, so from iteration 2 on it
  is an out-of-sample check of the previous update.
- **Search**: 1 key → bounded Brent; ≥2 keys → Nelder-Mead with reflection, inside a trust
  region (`--trust` of each range, ×0.6 per iteration). Iteration 1 gets an analytic warm
  start for keys the reduced model represents. The best candidate (never worse than the
  current θ on this data) is kept.
- **Keys** default to `wheel_distance, drive_max_force` (`loop_config.yaml optimise.keys`).
  Geometry and masses are locked by `calib_params` and can't be selected.
- **Scored features** per test are in `loop_config.yaml scoring`. `T4_arc` is display-only
  because the sim currently can't follow an arc (open defect, see
  `docs/PARAMETER_TEST_VALIDATION.md`); scoring it would swamp the loss.

## The return function (homing)

Closed loop on the camera pose at 20 Hz:
1. **to_pre**: drive to a pre-goal `run_in_m` (15 cm) behind the start pose. In-place spins
   "walk" the chassis a few cm on carpet, so the big turns happen here, not at the goal.
   If the pre-goal is close behind the rover, it reverses instead of U-turning.
2. **face**: rotate to the start heading.
3. **approach**: drive straight in, steering onto the goal line; reverse if it overshoots.
4. **trim**: small yaw correction, stop, median pose over `settle_s`, verify; a sideways
   offset sends it back to step 1 for a fresh run-in.

Pose is predicted `latency_s` ahead (camera + actuation lag; the base also coasts after
a stop). If commands don't move it, the minimum command is raised step by step (carpet
stiction). If it still doesn't move at full command, it reports dead motors. If the camera
loses the rover for longer than `lost_timeout_s`, it stops and gives up. Measured on carpet
(2026-10-19): short returns took 6–8 s and ended within 1.4 cm / 0.5–3.9°. A ~0.45 m
corner-to-centre return was cut short when the rover's battery browned out, so a
long-return figure for the final controller is still to be measured.

## Safety

- The base has **no cmd_vel timeout** (it holds the last command). The agent always
  publishes (zeros when idle), zeroes a velocity setpoint after 0.5 s without refresh,
  aborts sequences if the desktop goes silent for 1.5 s, and zeroes on EOF/SIGHUP/SIGTERM.
- Trials abort (rover stopped) if the rover leaves the `workspace` box or the camera
  loses it for 1.5 s. Homing refuses goals outside the box.
- **Battery**: this rover browns out on a sagging pack. The motors die first, then the
  Jetson drops off the network. The loop stops below `battery_min_v` (9.9 V) and treats
  "doesn't move at full command" as fatal. Charge between sessions.

## Tests (no hardware needed)

```bash
cd ~/projects/isaac-assets && source ros2env.sh && cd sim2real_calib
python3 loop/test_agent_local.py                  # agent safety (deadman, link loss, EOF...)
python3 loop/icp2d.py && python3 loop/fusion.py   # scan matcher + fusion self-tests
python3 loop/camera_guard.py --self-test          # camera-move detection on the reference
python3 loop/tests/test_stations_sim.py           # teach/return/lidar fine/obstacle/latency
python3 loop/tests/test_lidar_convention.py       # mirrored-lidar detection
python3 loop/closed_loop.py --test-fake-rover --surface carpet --tests T1,T2 \
    --repeats 2 --iterations 1 --max-evals 2 --out /tmp/fake.yaml   # whole loop + Isaac
```
Run the ROS tests one at a time (each uses its own ROS domain, but the fake rover has no
collision with other fakes on the same domain).
