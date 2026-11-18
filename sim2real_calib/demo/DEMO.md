# Sim↔Real path demo (no optimizer)

Groundwork for the demo: run the **real** rover (seen live through the overhead webcam), then
run the **sim** rover from a camera-**matched** overhead viewpoint, draw each path line, and
compare them with error metrics — all with **fixed parameters** (the optimizer is intentionally
NOT engaged yet).

Status: camera tracking uses the green/orange-red/pink sticker triangle.
Measured sticker offsets register the chassis center and heading. The live
preview uses estimated camera geometry; verified sticker-plane projection is
still required for precision trial recording.
See [sticker setup](../calib/README.md). The dead-reckoned reference remains an
offline fallback when the camera isn't available.

```
overhead/   derive_camera_pose.py  camera_model.py  render_overhead.py  camera_setup.yaml
demo/       dead_reckon.py  run_demo.py  run_suite.py  (this file)
parallel/sequences/  T1_straight T2_spin T3_speed_sweep T4_arc T5_stop (+ square)
trials/     trial_real.sh  trial_sim.sh  record_traj.py  compare_paths.py
```

## Live simultaneous demo (realtime_demo.py)

From `sim2real_calib/`, with the camera streaming:

```bash
python3 demo/realtime_demo.py --tests T1,T2,T3,T3b,T4,T5,T6 --surface demo
```

This drives the real and simulated rovers together through every test of the battery,
**unattended**:

- **Return to start:** before each test, the rover drives itself to that test's start
  station, using the same camera homing and lidar fine correction as the closed loop
  (`loop/stations.py`). Taught stations are used when they exist. Otherwise it uses
  the `start_poses` in `loop/loop_config.yaml`. If it cannot reach a start pose, that
  test is skipped and the battery continues.
- **Same start time:** the real sequence is timed on the Jetson through the persistent
  rover link (`loop/rover_link.py`, the closed loop's path), and the sim is clocked
  from the moment the rover reports that it started. Both paths therefore share one
  time base, with the camera's measured latency removed. The old 9 s `--tail` is no
  longer needed.
- **Safety:** the closed loop's geofence applies. Leaving the `workspace` box or
  losing the camera for 1.5 s stops the rover and skips that test. The battery also
  stops below `battery_min_v`.
- **One window for the whole battery** (`viz/dashboard.py`). It stays open and
  responsive throughout: while Isaac starts, while the rover returns to start, and
  while comparisons and videos render. It shows:
  - the live camera, with the real path and the sim path drawn on it;
  - the current test top-down, with the live gap between the two paths;
  - a gallery tile for every finished test (real vs sim + endpoint/mean/heading
    error) that fills in as the battery runs;
  - a log ticker.

  When the battery finishes, the window stays up until you press q (`--no-hold` to
  exit right away). q / Esc during a run stops both rovers. `--no-show` runs without
  a window; `demo/runs/<surface>/live.png` is still written every second.

Options:

- `--return-to-start prompt`: you place the rover and press Enter in the terminal.
  The window stays live while it waits.
- `--return-to-start off`: each test starts wherever the rover is.
- `--no-real`: simulation rehearsal. The sim path is drawn from the test's configured
  start pose.

The rover base driver is started over SSH if it isn't running. The demo sets
matching DDS discovery settings for desktop ROS and Isaac automatically, including
when the launching shell has `ROS_LOCALHOST_ONLY=1`.

During startup, status lines appear in the window ticker and in the terminal. If
startup stalls, check the printed `demo/runs/<surface>/isaac.log` path. Ctrl-C (or q)
aborts the whole battery and releases the rover link, camera, ROS context and Isaac
process group. Only one camera receiver should hold UDP 5000 at a time.

Run the discovery and shutdown regressions from the repository root in a
ROS-sourced shell:

```bash
python3 -m unittest discover -s sim2real_calib/demo -p 'test_realtime_demo.py' -v
```

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

## Full demo suite — every test, per surface (run_suite.py)

The turnkey version of "drive real → track, drive sim → track, compare" for the **whole T1–T5
battery**. Per test it records the real camera path and the sim `/odom` path, builds the
matched-webcam overlay video, and the error metrics; then writes a summary table across tests.
No optimizer, no parameter changes — this quantifies today's sim-vs-real gap on each maneuver.

Prereqs (all on the desktop, one ROS-sourced shell):
- overhead camera streaming, floor grid calibrated (`calib/grid_calib.json`), stickers
  registered for chassis-center recording (`calib/sticker_config.json`);
- rover base driver up: `ssh rover 'bash ~/rover_nav/start_bringup.sh'` (keep running);
- Isaac installed and GPU available. Each sim trial launches its own Isaac process;
  the native ROS interface publishes `/ugv_sim/odom`. Use `source ros2env.sh` for
  matching DDS settings when inspecting the sim from a ROS terminal.

```bash
source ros2env.sh
python3 sim2real_calib/demo/run_suite.py --surface heavy_carpet       # all 5 tests
python3 sim2real_calib/demo/run_suite.py --surface concrete --tests T1,T4
# re-visualize / re-compare from CSVs already recorded (no rig):
python3 sim2real_calib/demo/run_suite.py --surface heavy_carpet --skip-real --skip-sim
```
It pauses before each real drive so you can reposition the rover (`--no-pause` to disable).

Output per test: `demo/runs/<surface>/<Ti>/{real.csv,sim.csv,overhead.mp4,compare.png,metrics.json}`
and across tests: `demo/runs/<surface>/summary.{md,csv}`.

The T1–T5 sequences map to the test battery in `docs/MODELING.md §6` (straight, spin, speed
sweep, arc, coast-stop) and are sized to stay inside the 1×1 m grid. For a single ad-hoc run of
one sequence (and the dead-reckoned-reference fallback), use `run_demo.py` below.

## 0. Match the sim view to the webcam (once)
The sim's overhead camera is driven by `overhead/camera_setup.yaml` (camera height, aim, FOV in
the grid frame). Generate an estimate from the existing grid calibration:
```bash
python3 overhead/derive_camera_pose.py            # writes overhead/camera_setup.yaml
# verified: it reprojects the grid corners onto your clicked corners within ~4 px.
```
This estimate assumes a **nominal C920e FOV** and the homography scale, so the absolute height is
approximate. Make it **exact** by either (preferred) finishing intrinsics calibration
(`calib/calibrate_intrinsics.py` → `camera_intrinsics.yaml`, then re-run derive), or dropping
**measured** numbers into `camera_setup.yaml`:
- `position_m: [x, y, height]` — camera position over the grid (height above the grid plane),
- `look_at_m: [x, y, 0]` — the grid point the lens is aimed at (straight-down ⇒ same x,y as the camera),
- `horizontal_fov_deg` — the camera's true horizontal FOV.

You can also pass a tape-measured height: `derive_camera_pose.py --height 1.85`.

## 1. See the sim from the matched viewpoint (live)
```bash
isaacpy ugv_controller_v2/run_overhead.py                       # live Isaac overhead view + path line
isaacpy ugv_controller_v2/run_overhead.py --calib sim2real_calib/surfaces/concrete.yaml
```
This authors `/World/OverheadCam` at the matched pose/FOV, makes it the active viewport, and
traces the rover's path on the floor as it drives. Drive it with the usual
`keyboard/keyboard_drive.sh` or a sequence.

## 2. Run the demo (both path lines + error)
One command. Reference = real CSV if you have one, else dead-reckoned.
```bash
source ros2env.sh                                               # sim run needs ROS + GPU
python3 sim2real_calib/demo/run_demo.py \
    --sequence sim2real_calib/parallel/sequences/square.yaml
# with a fixed per-surface calibration (still NOT the optimizer):
python3 ... --calib sim2real_calib/surfaces/concrete.yaml
# once camera tracking works, plug the real path in:
python3 ... --real real_run.csv
```
Produces in `demo/runs/<timestamp>/`:
- `demo_overhead.mp4` — both paths drawn in the webcam-matched overhead view (shareable video),
- `compare.png` + printed metrics — endpoint / mean / max / heading error, path lengths,
- `reference.csv`, `sim.csv`.

Offline (no Isaac, reuse a sim `/odom` CSV you already have):
```bash
python3 sim2real_calib/demo/run_demo.py --sequence .../square.yaml --sim-csv sim_run.csv
```

## 3. The real run (what to do now vs later)
- **Now:** drive the real rover through the sequence (see `trials/trial_real.sh`) while watching
  it live via `./run_viz.sh` (the metric overlay). Optionally screen-record that window for the
  demo's "real" footage. The path line is dead-reckoned for comparison until tracking is wired.
- **With measured sticker geometry:** `trials/record_traj.py --source camera --out
  real_run.csv`, then `run_demo.py --real real_run.csv`. No other change — the overlay video and
  the error metrics come out the same way, now against the true measured path.

## Notes
- The optimizer (`optimize/`) is deliberately **not** invoked here. This demo shows the
  *pre-calibration* gap between sim and real; calibrating that gap away is the next phase.
- `render_overhead.py` draws synthetically from the pose track (geometrically the webcam's
  projection); `run_overhead.py` is the photoreal Isaac render from the same camera. Use whichever
  reads better for the audience.
- Grid origin: the sim assumes the grid origin at the rover's start (world 0,0; set
  `grid_origin_m` in `camera_setup.yaml` if you place it elsewhere). Keep the real rover's start
  on the grid's origin corner so the two views line up.
