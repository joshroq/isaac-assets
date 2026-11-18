# Live demo guide (sim vs real rover)

This guide is the checked procedure from the 2026-10-29 hardware runs. Each step lists
what it should print. If a step does not print that, use the troubleshooting table at
the end before continuing. **Do not keep data from a run that needed troubleshooting:**
delete its session folder (see "Results"), fix the cause, then re-run.

## 0. Before the audience arrives (~15 min)

**Hardware**

* Plug the rover into wall power. The pack is failing and browns out the motors. In the
  runs the reported voltage fell from 11.9 to 11.0 V over an hour; the loops park and
  stop by themselves below 9.9 V.
* Clear the grid. Turn the Pi overhead camera on; it streams by itself.
* Close every other camera viewer (`live_tracking.py`, `metric_overlay.py`,
  `receive.py`). Only one program can read the camera.

**Shell (every terminal)**

```bash
cd ~/projects/isaac-assets && source ros2env.sh && cd sim2real_calib
export ROVER_HOST=rover-lan        # direct LAN link to the rover (see below)
```

`ROVER_HOST=rover-lan` matters. The default `rover` alias goes over Tailscale, which
relays through the internet, and a 3.5-minute internet blip killed the rover link
mid-test. The LAN path needs only the home Wi-Fi.

**Checks**

| Step | Command | Expect |
|---|---|---|
| Grid still valid | `python3 calib/calibrate_grid.py --verify --source stream` | `PASS: calibration error <= 10 mm` |
| (only if it FAILs) recalibrate | `python3 calib/calibrate_grid.py --source stream --size 1 --spacing 0.25` | click BL, BR, TR, TL; review shows ~24/25 green crossings; ENTER saves |
| Stickers | `python3 calib/verify_stickers.py --duration 15` (rover parked on the grid) | `RESULT: PASS` |
| Rover link | `python3 loop/rover_link.py --check` | `base ready: /cmd_vel subs=1, battery 11.x V ... gyro yes, lidar yes` |
| Start stations | `python3 loop/stations.py verify T1_straight T2_spin T4_arc --reps 1` | three `ACCEPTED` lines |

Notes:

* Always pass `--spacing 0.25` when recalibrating the grid. Without it only the 4
  corners are used and the result was 10.7 mm off.
* If stations report `not taught` (for example after moving the furniture or camera),
  run `python3 loop/stations.py teach T1_straight` (and the same for `T2_spin` and
  `T4_arc`).

## 1. Demo A: real and sim rovers side by side (no optimiser), ~8 min

```bash
python3 demo/realtime_demo.py --tests T1,T2,T4 --surface demo \
    --surface-calib surfaces/carpet_loop.yaml        # calibrated wheel-contact physics
```

What the audience sees, in one window that stays open the whole time:

1. **Startup (~40 s).** Isaac starts while the live overhead camera is already
   showing. The ticker shows "waiting for Isaac … ready".
2. **Return to start.** For each test the rover drives itself back to its taught start
   spot (~25–40 s), then the log shows `ACCEPTED`.
3. **Countdown.** A 3 s countdown, then real and sim drive the same commands at the same
   time. The camera view draws the real path (green) and the sim path (blue). The
   top-down panel shows both clocks: real t+ and sim t+.
4. **Gallery.** When a test finishes, its gallery tile appears with endpoint and
   heading error.
5. **End.** The window stays up until you press **q**. Press **p** at any time for the
   full sim parameter sheet.

**Talking point:** the sim rover drives its wheels through PhysX wheel-floor contact
with the calibrated physics parameters, so every difference on the camera view is a
physics difference. Straights and spins match closely after calibration; arcs remain the
open defect (the wheel-contact sim turns too little while driving forward).

**Rehearsal without the rover:** add `--no-real`.

## 2. Demo B: the optimiser (closed loop), ~30–40 min

```bash
python3 loop/closed_loop.py --surface carpet --tests T1,T2,T4 --repeats 2 --iterations 1 --max-evals 6
```

1. **Real phase.** For 6 trials, the rover returns to each station by itself, drives,
   and the camera records.
2. **Isaac BEFORE.** The current parameters are run on the same tests.
3. **Optimiser.** Up to 6 Isaac evaluations (~1 min each). The **SIM PARAMETERS BEING
   FITTED** panel shows the start, best and latest candidate on each parameter's range.
   Each test tile updates after every evaluation, and **p** shows every parameter.
4. **AFTER.** The best parameters are run, then the before/after tiles and the
   paths-on-camera video are drawn.

**Short version for a time slot** (replays the real data recorded on Oct 20, no rover
driving, ~5 min):

```bash
python3 loop/closed_loop.py --surface carpet --tests T2 --iterations 1 --max-evals 3 \
    --real-from loop/runs/carpet_20261020_194607/iter_01/real
```

## 3. Results: where everything goes

* **`sim2real_calib/RESULTS.md`** catalogs every session, newest first, with links.
* **Demo sessions:** `demo/runs/demo_<stamp>/`; `demo/runs/demo_latest` always points
  to the newest.
* **Optimiser sessions:** `loop/runs/carpet_<stamp>/`.

Open the session's **`index.md`**. It contains the settings and summary, then the
videos:

* `session_video.mp4`: recording of the whole window, ready for slides;
* `final_view.png`: the window at the end;

and then, per test:

* `paths_map.png` / `.mp4`: real vs sim drawn on the real camera video;
* `real_camera.mp4`: the raw camera recording;
* `compare.png`: the comparison plot;
* `response_map.png`: command vs achieved motion;
* the CSVs and `metrics.json`.

`config/` holds copies of the exact calibration and model files used.

**Discarding a bad run:** delete its folder, then refresh the catalog with
`python3 viz/session_index.py`.

## 4. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Window never appears / run ends immediately with `interrupted` | (fixed) GTK reported the new window as "closed" before it was mapped | update; or run with `--no-show` and present `live.png` / `session_video.mp4` |
| `rover agent is not running` mid-run; `tailscale status` shows rover offline | Tailscale relay / internet blip | `export ROVER_HOST=rover-lan`; check `ping 10.168.168.115` |
| `REJECTED` at a station, rover keeps re-shuffling | (fixed) fine moves on carpet made heading worse | now accepts within 4 cm / 6° and ignores ≤3 cm sideways; if still rejected, re-teach that station |
| Sim path much shorter than real | (fixed) Isaac runs ~0.58× real time; the demo now commands the sim on sim time | the log line `Isaac ran at 0.5x real time` is normal |
| `no camera stream` / `UDP 5000 busy` | another viewer is open, or the Pi is off | close viewers; power-cycle the Pi; check with `calibrate_grid.py --verify` |
| `camera check failed: reference is stale` (optimiser) | grid recalibrated without a new guard reference | `python3 loop/camera_guard.py --save-reference` (calibrate_grid does this itself) |
| `battery x V < 9.9 V` | pack low | wall power / charge; the loop parks the rover first |
| Isaac `not ready within 300 s` | slow first launch or stale DDS state | re-run; if repeated, `rm /dev/shm/fastrtps_*; ros2 daemon stop` and retry |
| Rover drives off the grid | — | **q** in the window (or Ctrl-C) stops both rovers; the geofence also aborts a test that leaves the workspace |

## 5. The most robust testing going forward

The goal is to match **every motion Nav2 and teleop command**: straight
forward/back, spins, and arcs of every radius in all four directions. The T1/T2/T4 trio
covers three points of that. The full procedure:

1. **Record the full battery on the real rover** (the T tests plus the response-map
   battery R1-R5), real only, with a fresh iteration as the held-out set:

   ```bash
   python3 loop/stations.py teach R1_straight_fb      # repeat for R2_spin_rates, R3_arcs_slow,
                                                      # R4_arcs_fast, R5_nav_pattern
   python3 loop/closed_loop.py --surface carpet --real-only --iterations 2 --repeats 3 \
       --tests T1,T2,T3,T3b,T4,T5,T6,R1,R2,R3,R4,R5
   ```

2. **Fit every physics key in stages** on iteration 1 (no rover, ~5 h):
   `bash loop/staged_physics.sh loop/runs/<session>/iter_01/real carpet`
   -> `surfaces/carpet_physics.yaml`.
3. **Check it on the held-out iteration 2:**
   `python3 loop/verify_params.py --surface carpet --calibrated surfaces/carpet_physics.yaml --combine-mode average --tests T1,T2,T3,T3b,T4,T5,T6,R1,R2,R3,R4,R5 --real-from loop/runs/<session>/iter_02/real`.

Use 3–5 real repeats per test: the real rover's arcs vary by about 2× between repeats.
