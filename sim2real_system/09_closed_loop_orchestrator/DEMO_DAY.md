# Demo day: sim2real on a new floor surface

Everything runs from **one command**. `surface_demo.py` runs every stage automatically,
one after another, using the project's own tools:

```bash
cd ~/projects/isaac-assets/sim2real_calib
python3 surface_demo.py --surface light_carpet
```

It runs `setup → preflight → grid → camera → rig → workspace → live → results → save`
(about 40–50 min). It needs you only twice, in click windows:
1. **grid:** click the 4 outer corners (bottom-left, bottom-right, top-right, top-left), then press Enter in the review window.
2. **camera:** with the rover parked at the grid centre facing up the image, click the green, orange-red and pink sticker centres, then press Enter.

There are no yes/no questions; low-battery and resume decisions are automatic. If a
step fails, the run stops and prints the exact command to continue from that step.

```bash
python3 surface_demo.py --surface light_carpet --from live            # continue from a step
python3 surface_demo.py --surface light_carpet --skip grid,camera     # already calibrated
python3 surface_demo.py --surface light_carpet --grid verify          # keep the saved grid, only check it
python3 surface_demo.py --surface light_carpet live results           # only these steps
python3 surface_demo.py --surface light_carpet --list                 # what's done
```

The script sources `ros2env.sh` itself. `collect`, `optimise` and `verify` are extra steps
that only run when named (`python3 surface_demo.py --surface light_carpet collect --tests full`).

## What the audience sees

| step | rover moves? | time | what it demonstrates |
|---|---|---|---|
| `setup` | no | seconds | Creates a rig profile for the new floor: carpet homing gains, a fresh workspace, no old stations. The padded rig is saved first. |
| `preflight` | no | ~30 s | Checks the Pi camera stream and exposure, the overhead frames, the rover link and battery, and that the GPU is free. |
| `grid` | no | ~2 min | You click the 4 corners. The tool finds every tape crossing and fits pixels to metres, then verifies to ≤10 mm. |
| `camera` | no | ~3 min | You click the 3 stickers. The camera sweeps 28 exposure/gain settings and keeps the one where the sticker colours are strongest. The sticker colours are then calibrated, and live tracking is checked. |
| `rig` | **yes** | ~2 min | Measures camera latency and the lidar convention, and the wheel-odometry slip on this floor (`odom_scale`). |
| `workspace` | no | ~1 min | Draws the fence, every start pose, the rover footprint and the homing pre-goals on the camera image. Confirm they're all on the carpet. |
| `live` | **yes** | ~25–35 min | **The main demo.** For each test, the rover drives itself to the start pose and runs it, 3 times. Isaac then runs the same tests, the optimiser fits the sim's physics, and the fitted sim is re-run. A single dashboard window shows all of this live. |
| `collect` | **yes** | full: ~1 h + charges | Only needed for the full 12-test battery (training plus held-out set). Resumes after a battery stop. |
| `optimise` | no | ~15–25 min | Isaac and the optimiser run on recorded trials. Use this if the rover dies mid-`live`. |
| `verify` | no* | ~10–20 min | Held-out check: the fitted sim against real trials it wasn't fitted on. *If no held-out recording exists, the rover drives fresh trials. |
| `results` | no | — | Prints the session pages and fitted parameters, and opens the dashboard and progress plots. |
| `save` | no | — | Saves the light_carpet rig profile. |

The **quick** test set (`--tests quick`, the default) is T1 straight, T2 spin and T6 spin
sweep. Between them these identify both parameters the live loop fits: the turning gain
(`wheel_distance`) and the drive torque cap (`drive_max_force`). `--tests full` runs all
12 tests, which is only practical for `collect` (about 1 h of driving and 2–3 battery charges).

`setup` is already done for light_carpet (see below). If you calibrate before the audience
arrives, start the demo itself with `--skip grid,camera` (or `--from rig`).

The `workspace` image is shown for 5 s and saved to `rig_profiles/light_carpet/workspace.png`.
The run stops automatically if a start pose or homing pre-goal falls outside the fence, but
only you can see whether the fence is on the carpet. Glance at the image during the run,
and if it's wrong, Ctrl-C, edit `loop/loop_config.yaml`, then `--from workspace`.

## Before you go (night before)

- [ ] **Rover battery fully charged.** The pack lasts about 15–20 driving trials; the loop stops at 10.4 V. Bring the charger.
- [ ] **Pi camera:** the Pi, the C920e and its mount, and the **5 V / 3 A supply**. A 2 A supply browns the Pi out and has corrupted its SD card before.
- [ ] **Network at the demo site.** The Pi streams to this desktop over Tailscale (`100.107.253.10`), and the rover is reached as `rover-lan`, then `rover` (Tailscale).
  - **The Pi only knows the home Wi-Fi.** If the demo isn't at home, add the site's Wi-Fi to the Pi beforehand, or bring a hotspot both devices know.
  - Test from the site: `ping pi-cam` (or `100.67.148.19`) and `ssh rover-lan`.
- [ ] **Tape:** 1-inch tape, a tape measure and a square. The grid is **exactly 1 m outer edge to outer edge**, lines every 0.25 m, border tape **inside** the square.
- [ ] **Lighting:** steady and even. The sticker colours are calibrated for one lighting. If it changes mid-demo, all three stickers can vanish and you must re-run `camera`.

## Layout tips (learned on concrete and padded)

- Put the 1 m grid in the middle of the carpet, with **at least 30 cm of carpet beyond every grid edge**. Start poses sit 0.25–0.65 m up the grid, and homing backs the rover up to 0.15 m behind them.
- On the EVA foam, the rover's rear wheels reached the mat edge, the heading kicked 30–40°, and homing hunted forever. If `workspace` shows anything off the carpet, edit `loop/loop_config.yaml`: either `workspace:` (the fence) or `start_poses:` (shift every `y` up).
- Camera: mounted overhead, about 1.6 m up, with the whole grid plus a margin in view.
- Keep the area around the grid clear: the lidar obstacle guard blocks motion toward anything within 10 cm.

## Light carpet is already set up

```text
python3 rig_profile.py list
  light_carpet   <- active: carpet preset (homing turn 0.45-0.9 rad/s, odom_scale 0.89 until measured)
  padded         <- today's EVA-foam rig, saved
```

The grid, camera exposure, sticker colours, camera latency and `odom_scale` from the
padded setup are still in place as placeholders. The `grid`, `camera` and `rig` steps
replace them on the carpet. Do not record data before those three have run.

## If something goes wrong

| symptom | cause | fix |
|---|---|---|
| `camera lost the rover (missing: green, orange_red, pink)` | lighting changed, or the Pi rebooted (exposure reset to auto) | `python3 surface_demo.py --surface light_carpet camera`, or for a reboot just `python3 calib/camera_exposure.py --apply` |
| `camera recording unusable: N s camera gap` during spins | stickers hidden by the mast/antenna at some headings and too faint at others | Re-run `camera`. Even out the lighting (harsh glare washes out orange-red). |
| homing `timed out (8 cm off)` over and over at the same spot | rover stuck on a carpet seam/edge, or the pre-goal is off the floor | Run `workspace`; move `start_poses` away from the edge. |
| `battery X V < 10.4 V` | pack low (it browns out the motors below this) | Charge, then run the printed `--from <step>` command; it resumes automatically (finished trials are kept). |
| `rover does not move` / link stalled | Wi-Fi drop | Wait; `python3 loop/rover_link.py --check`. Runs resume. |
| Isaac `/ugv_sim/odom stopped advancing` | stale DDS shared memory | `sh /opt/ros/humble/bin/fastdds shm clean`, then re-run. |
| GPU busy / Isaac won't start | leftover Isaac | `pgrep -af run_ground_plane`; kill it. |
| `no overhead frames on UDP 5000` | another viewer is open, or the Pi stream is off | Close `live_tracking.py` / `receive.py`; `preflight` starts the Pi stream. |
| `grid verify failed` | camera bumped | Re-run `grid` (recalibrate). |

Ctrl-C (or q / Esc in the dashboard window) always stops the rover.

## Tools behind the steps (to run any part by hand)

| tool | does |
|---|---|
| `rig_profile.py save/activate/new/list` | save / restore / create a surface's whole rig setup (`rig_profiles/<surface>/`) |
| `calib/calibrate_grid.py --source stream --size 1 --spacing 0.25` | grid calibration (`--verify` to check) |
| `calib/camera_exposure.py --tune / --apply / --show / --auto` | overhead camera exposure for the stickers (`calib/camera_settings.json`) |
| `calib/calibrate_stickers.py --points-px ...` | sticker colours |
| `calib/verify_stickers.py`, `live_tracking.py` | live tracking views (view-only) |
| `loop/selfcheck.py --motion` | rig preflight, camera latency, lidar convention |
| `loop/measure_odom_scale.py --write` | wheel-odometry scale for the floor |
| `loop/closed_loop.py --surface S ...` | the loop (`--real-only`, `--real-from`, `--resume`) |
| `loop/verify_params.py` | held-out verification |
| `bash loop/staged_physics.sh <real dir> <surface>` | the full 16-key staged physics fit (~5 h, no rover; for after the demo) |

Outputs: each session gets `loop/runs/<surface>_<stamp>/index.md` (videos, plots, metrics),
the fitted parameters go to `surfaces/<surface>_loop.yaml`, and `RESULTS.md` catalogs every session.

## Going back to the padded floor later

```bash
python3 rig_profile.py activate padded
python3 calib/camera_exposure.py --apply        # exposure 300 / gain 200 (dim light)
python3 calib/calibrate_grid.py --verify --source stream --save /tmp/v.png
python3 surface_demo.py --surface padded collect --tests full    # resumes the 19/72 recording
```

The padded recording `loop/runs/padded_20261118_124645` has 19 of 72 trials (T1–T6 × 2,
R1–R5 × 1). Only resume it with the mats, grid and camera exactly as they were. If
the grid was re-taped, re-run `grid` and `camera` first.
