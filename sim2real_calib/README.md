# sim2real_calib — overhead ground-truth camera rig

Overhead C920e watching the 1 m × 1 m grid provides **ground-truth** rover motion
on each surface (thick/thin carpet, padded, concrete). We compare that to the
Isaac Sim rover driving the *same* `/cmd_vel` commands and tune sim parameters
(+ the command→actual slip gap) until sim matches reality.

The Isaac backend shares the updated `base_link_fixed2.stl` chassis through
`jetson_rover_v2.usd`; no separate calibration mesh needs replacing. Wheel
geometry and per-surface effective `wheelDistance` values are preserved because
the replacement preserves axle locations. The upper deck and its camera/lidar/
pan-tilt mounts move up by 45.7 mm, with the ROS TF graphs following those USD
mounts automatically. See the
[geometry and graph audit](../ugv_controller_v2/BASE_MESH_UPDATE.md).

Topology for the current bring-up:

```
[Windows laptop @ tripod] --C920e--> ffmpeg (H.264/MPEG-TS)
        |  UDP :5000 over Tailscale
        v
[this desktop 100.107.253.10]  receive.py (OpenCV)  -> Isaac Sim + calibration
```

The laptop is a stand-in for the ordered Raspberry Pi; when the Pi arrives it
takes over the sender role unchanged (Linux ffmpeg command instead of the .bat).

## Layout

- `capture/receive.py` — desktop receiver: reads the UDP stream, live view + HUD,
  snapshot (`s`), record (`r`).
- `laptop_sender/list_cameras.bat` — print DirectShow device names on the laptop.
- `laptop_sender/send_windows.bat` — the ffmpeg sender you run on the laptop.

## Quickstart (first light)

**1. Desktop (here) — start the receiver first.** It just listens on UDP 5000:

```bash
python3 sim2real_calib/capture/receive.py
```

**2. Laptop — one-time setup:**
- Install ffmpeg: `winget install Gyan.FFmpeg` (reopen the terminal after).
- Confirm Tailscale is up and the desktop is reachable: `ping 100.107.253.10`.
- Copy the `laptop_sender/` folder over, then run `list_cameras.bat` and make sure
  `CAM=` in `send_windows.bat` matches your webcam's exact name.

**3. Laptop — start streaming:**
```
send_windows.bat
```

Within a second or two the desktop window should show the grid with a live FPS /
latency HUD. Press `s` to grab a still, `r` to record an .mp4.

## Troubleshooting

- **No window / "no frames after 20s":** sender not running, wrong `DESKTOP_IP`,
  or Tailscale down. Verify `ping 100.107.253.10` from the laptop first.
- **Laggy or smeary video:** UDP loss over Tailscale. Lower `RES=640x480` and/or
  `BITRATE=2M` in `send_windows.bat`. Keep `PKTSIZE=1128` (Tailscale MTU limit).
- **Wrong camera:** `list_cameras.bat` shows all devices; make sure `CAM=` is the
  C920e and not the laptop's built-in cam.
- **High latency (`age` climbing):** the receiver already keeps only the newest
  frame; if age still grows, the network can't keep up — drop resolution/bitrate.

## Intrinsic calibration (step 2 — built, `calib/`)

Removes the C920e's lens distortion so measured distances are trustworthy.

```bash
# 1. generate the board, print charuco_board.png at 100% (actual size)
python3 sim2real_calib/calib/make_charuco.py
# 2. tape it flat to a rigid board; measure a square; fix square_len_m in
#    calib/charuco_spec.json if it isn't exactly 35 mm
# 3. STOP receive.py (one UDP-5000 consumer at a time), then capture + solve live:
python3 sim2real_calib/calib/calibrate_intrinsics.py --source stream
#    move the board through tilts/corners/near-far; SPACE grabs a green view;
#    ~15-25 views, then press C.  Writes calib/camera_intrinsics.yaml
```

Aim for RMS reprojection error < 0.5 px (< 1.0 is usable). You can also solve from
a folder of saved PNGs: `--source /path/to/images/`.

## Metric visualization (step 3 — built)

One command on the desktop: start the Pi stream, calibrate the grid if needed, and
open the live metric overlay (stops the stream again on exit to spare the 2A supply):

```bash
./sim2real_calib/run_viz.sh                 # normal launch
./sim2real_calib/run_viz.sh --recalibrate   # re-click the grid corners
./sim2real_calib/run_viz.sh --no-stream      # stream already running
```

Under the hood:
- `calib/calibrate_grid.py --source stream --size 2x2 --spacing 0.5` — any W×H metre
  tape grid: click 4 anchors (outer corners BL,BR,TR,TL), every tape crossing is then
  found automatically and the pixels↔metres homography is fitted to all of them →
  `calib/grid_calib.json`. `--verify` checks a saved calibration against the live grid.
  See [grid calibration](calib/README.md#grid-calibration-any-size).
- `calib/verify_stickers.py` — live feed → every sticker's (x, y), jitter, detection
  rate, triangle edge check, optional known-position pass/fail.
- `viz/metric_overlay.py` — live feed with the metric grid overlaid, hover-to-read
  (x,y) in metres, click-two-points to measure distance, and the rover's colored
  sticker-triangle pose when visible.
- `calib/calibrate_stickers.py` — samples green/front-left, orange-red/front-right,
  pink/rear-center and registers their measured offsets. See [sticker tracking](calib/README.md).
- `capture/receive.py --track` — shows the sticker triangle, reference point and heading.

Both viz tools can also run on a saved frame (`--image … --save …`) for offline
checks / figures.

## Sequential sim-vs-real calibration loop (step 4 — built, `trials/`)

The rovers do NOT run at the same time. One trial at a time: drive+record the REAL
rover, drive+record the SIM with the same sequence, compare the paths, tune sim
params, repeat. This matches how the rover is actually driven (SSH, local execution
on the Jetson — no DDS over the network, which was never reliable here).

```
REAL:  ssh rover: start_bringup (base driver) + run sequence on the rover's /cmd_vel
       desktop:   record camera -> sticker triangle -> grid x,y,yaw -> real_run.csv
SIM:   desktop:   run same sequence -> Isaac /cmd_vel
       desktop:   record /odom -> x,y,yaw                 ->  sim_run.csv
COMPARE: align both to start, overlay + error metrics, tune sim params, repeat
```

Tools:
- `trials/record_traj.py` — record a trajectory to CSV (`--source camera` for the real
  rover via sticker-plane projection + measured body geometry; `--source odom` for the sim).
- `trials/trial_real.sh` — stages + runs the sequence on the rover over SSH (checks the
  base driver is up; zeroes on SSH drop), while recording the camera ground truth.
- `trials/trial_sim.sh` — drives Isaac with the same sequence, records `/odom`.
- `trials/compare_paths.py` — start-aligns both paths, prints endpoint/mean/max/heading
  error + path lengths, and saves an overlay + error-over-time plot.
- `parallel/cmd_sequence.py` + `parallel/sequences/*.yaml` — the timed Twist sequence,
  run locally on each target (`--no-real` for sim; on the rover `--no-sim --real-topic /cmd_vel`).

### Run a round

```bash
# REAL rover (on the rover, keep running):  ssh rover 'bash ~/rover_nav/start_bringup.sh'
# then from the desktop (camera streaming, grid and sticker geometry calibrated):
./sim2real_calib/trials/trial_real.sh   # -> real_run.csv

# SIM (Isaac running + discoverable, source ros2env.sh):
./sim2real_calib/trials/trial_sim.sh    # -> sim_run.csv

# compare:
python3 sim2real_calib/trials/compare_paths.py \
    sim2real_calib/trials/real_run.csv sim2real_calib/trials/sim_run.csv --out compare.png
```

`parallel/parallel_view.py` remains as an optional *live* debug view, but the loop
above is the real workflow.

## (Legacy note) real-time parallel view

Earlier design drove both rovers at once over DDS. Superseded by the sequential loop
because cross-network DDS was never reliable here; kept only as `parallel/parallel_view.py`.

- `parallel/cmd_sequence.py` — one timed Twist sequence to BOTH `/cmd_vel` (sim) and
  `/cmd_vel_remote` (real; the rover watchdog relays it and zeroes on link loss —
  never publish straight to the rover's `/cmd_vel`). Publishes at 20 Hz, zeroes on
  finish/Ctrl-C, prints a start epoch for log alignment.
- `parallel/sequences/square.yaml` — example sequence (edit to taste).
- `parallel/parallel_view.py` — live side-by-side: camera+metric overlay (real) |
  top-down 1x1 m plot of both paths (each relative to its start) + live position error.

### Runbook (multiple terminals)

```bash
# 0. camera streaming (Pi auto-starts on boot, or:)
ssh joshu@pi-cam.local 'sudo -n systemctl start sim2real-camera'

# 1. Isaac sim rover on a ground plane (its own terminal)
isaacpy ugv_controller_v2/run_ground_plane.py
#    confirm its odom topic:  (ROS terminal)  ros2 topic list | grep odom

# 2. real rover only: discovery server up here + rover-bringup on the rover
#    (see rover_comms/). Keep the rover on WALL POWER. Test sim-only FIRST.

# 3. the live comparison window (ROS-sourced terminal)
source ros2env.sh
python3 sim2real_calib/parallel/parallel_view.py            # --sim-odom /odom
#    (--flip-sim-y if the sim path shows mirrored vs the camera)

# 4. send the same sequence to both (another ROS-sourced terminal)
source ros2env.sh
python3 sim2real_calib/parallel/cmd_sequence.py \
    --sequence sim2real_calib/parallel/sequences/square.yaml --no-real   # SIM ONLY
# once happy, drop --no-real to also drive the real rover (deadman required)
```

Safety: always dry-run / sim-only before `--no-real` is removed; confirm the rover's
cmd_vel watchdog is running; rover on wall power (see [[rover-battery-ups-failing]]).

## Next pieces (not built yet)

5. **Sync + logging** — record camera sticker poses + sim `/odom` + the command log on
   one clock for offline trajectory comparison.
6. **Optimizer** — tune sim params + slip (Optuna/CMA-ES) to minimize real-vs-sim error.
