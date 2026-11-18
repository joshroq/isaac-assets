# Efficiency and accuracy notes from the 2026-11-03 hardware sessions

> **2026-11-17:** the response-model drive (chassis twist set directly, wheels free) has been
> removed; the sim rover now always drives its wheels through PhysX wheel-floor contact.
> Notes below that mention `SIM_DRIVE_MODEL`, `motion_model`, `param_direction` or
> `--drive-model` are historical.

These are measured on the real rig: carpet, 1x1 m grid, response-model drive, a battery
of R1-R5 + T1/T2/T4 tests with 2 real repeats each.

## Where an iteration's time goes (~23.5 min)

| Phase | Time | Share | Notes |
|---|---|---|---|
| Real trials (16) | 11.0 min | 47 % | per trial: median 20 s return-to-start + 20 s drive/record |
| Isaac BEFORE (8-test battery) | 5.7 min | 24 % | ~1 min launch + ~3 min of sim time at ~0.6x real time |
| Isaac AFTER | 5.7 min | 24 % | same |
| Refit (steady + dynamics) | 5 s | <1 % | |
| Path maps / plots / videos | 1 min | 4 % | |

One-off costs each session:

| Task | Time |
|---|---|
| Preflight (grid verify, camera guard, rover link, stickers) | ~3 min |
| Teaching 5 new stations | 3.5 min (27-68 s each, about half of it link setup) |
| Sticker colour re-sampling after a lighting change | ~5 min |

## Problems that cost accuracy (found and fixed)

1. **Trial time alignment.** The motion-onset detector fired 1.2-2.3 s late on spins:
   carpet stick-slip keeps the yaw rate under its 0.12 rad/s threshold at first. The
   per-step windows then fell in the next, opposite step: R2 yaw rates had the wrong
   sign, and the spin gain was fitted as 0.075.
   * **Fix:** `closed_loop.alignment_offset`. The onset estimate is used only when it is
     within 0.35 s of the rig latency (camera 0.71 s + link 0.15 s); otherwise the rig
     latency is used. `--realign DIR` re-times already-recorded trials.
   * **Effect:** refit residual rms z went from 3.21 to 1.15 on the same 16 trials.
2. **Lighting.** Warmer, dimmer evening light pushed the yellow sticker's hue to 23-28,
   just under the configured 28 floor: only 34 % detection and 32 % triangle. Run
   `verify_stickers.py` every session; re-sample the colours when lighting changes.
3. **Rover link over the Tailscale relay** dropped with an internet blip. Always use
   `ROVER_HOST=rover-lan`.
4. **Station fine moves on carpet** made heading worse. Fixed with the accept band
   (4 cm / 6°) and by ignoring sideways offsets of 3 cm or less.

5. **Pi camera host dropped offline** (01:06, session 2). It did not return without a
   power cycle; likely a power brownout (it needs 5 V / 3 A). The loop then spent ~2.5 min
   retrying returns to start against a frozen image (tracker "last camera fix" growing
   past 150 s). The power cycle also shifted the camera by 3.2 px (grid 2.2 -> 7.2 mm,
   one sticker edge 5.1 mm); fixed in ~1 min by re-fitting all tape crossings seeded from
   the camera guard's measured shift (no clicks).
   * **To add:** a camera-feed watchdog. With no new frame for >3 s, pause, stop the
     rover and wait for the stream instead of failing returns. Then run the guard check
     plus an automatic grid re-fit when it comes back.
6. **Git checkpoint timeouts** (editor). About 1.5 GB of untracked run videos made the
   checkpoint snapshot exceed 30 s. `loop/runs/` and `demo/runs/` are now in `.gitignore`.

### Evening session, 2026-11-03 (24 trials, held-out validation)

Real phase: 24 trials in 21 min with no homing retries; every station was accepted on the
first attempt via the 4 cm / 6° band. Problems:

7. **The rover's WiFi link dropped mid-session** (22:14). The SSH agent timed out, the rover's
   deadman stopped it mid-spin, and the loop misread the stall as "motors unpowered". Ping to
   the rover showed 25-85 % loss and 2-5 s round trips while the router was clean; power save
   was off and the signal was -56 dBm. It recovered within ~3 min. `--resume` kept the 17
   finished trials.
   * **To add:** before blaming the motors, check `rover.alive` / the agent's last reply and
     report "link lost". Pause and re-ping instead of failing the iteration.
8. **Fusion anchored kilometres away for one tick at startup.** `OdomIntegrator.at()` with a
   camera time just before the first odom sample wrapped `i-1` to the last sample and
   extrapolated by dt/1e-6. **Fixed** (returns the first sample) with a regression check in
   `fusion.py`'s self-test.
9. **The in-loop Isaac verification hung** (odom stopped advancing), twice. Fast DDS had 18
   zombie shared-memory ports from the processes killed earlier, and a publisher blocks on a
   dead reader's port. Run `fastdds shm clean` before every Isaac launch (sim_eval should do
   it).
10. **The hung Isaac processes survived the loop as orphans** holding the GPU.
    `sim_eval._stop_isaac` waited on the `python.sh` wrapper (which exits on SIGTERM) and
    never sent SIGKILL to the group. **Fixed:** the group is always swept with SIGKILL.
11. **The loop now releases the rover right after its last real phase.** It parks, stops
    the base driver and logs `REAL ROVER DONE - safe to power it off`, so the rover is not
    left on during the sim phase.
12. **`battery_min_v` raised to 10.4 V** (the pack sagged to 10.72 V on spins this session).

### Smoke test before the next hardware run, 2026-11-04 (no rover)

Every suite plus a full fake-rover session (`closed_loop.py --test-fake-rover`, model mode,
2 iterations, Isaac verification), traced Isaac runs, and audits of the real data. Found and
fixed:

13. **Isaac's model drive ran on the wrong clock (the main Isaac-vs-model gap).**
    `run_baked_rover` credited `PHYSICS_DT` (1/240 s) per loop iteration, but every 4th
    iteration is a render step that advances a whole `rendering_dt` (4 substeps), so the
    model's clock ran at 4/7 of sim time. Inside Isaac the lags were 1.75× too slow and
    the 2 s trial gap never triggered between tests, so a whole battery shared one random
    drift draw. The R1 straights drifted at about −0.6 rad/m in all three sim trials.
    **Fixed:** the drive gets the real elapsed `world.current_time`. Isaac now starts the
    same trials at the same times with the same draws as the offline model.
14. **The model drive's tracking aimed at the wrong quantity.** It compared the target with the
    velocity read back at the end of each step, but render steps hold the set velocity for 4
    substeps. **Fixed:** the achieved twist is measured from the pose change over each step.
    There is a separate multiplicative gain for render and non-render steps, and floor
    friction is zeroed in model mode too (the wheel zero alone still scrubbed ~30 % of yaw
    per step). Deterministic yaw tracking went from 0.0209 to **0.0059 rad/s** rms per step
    (unbiased), speed 1.6 mm/s. `SIM_DRIVE_LOG=<csv>` traces it per step, and the drive
    logs each new random trial.
15. **Session videos dropped most of the session.** The recorder wrote a frame only when ≥ 1/8 s
    had passed on its loop pass, which aliased (5.7 fps into an 8 fps file) and lost whole
    stretches under load. Real sessions came out at 2-70 % of their length (47 s for 42 min).
    **Fixed:** clock-driven, so slow patches repeat the frame (40 s session → 40.0 s video).
    `--resume` no longer overwrites the first part (`session_video_part2.mp4`, ...).
16. **Fake-rover tests polluted results.** A test session was listed in `RESULTS.md` as a real
    optimizer run and **overwrote `surfaces/carpet_motion_model_loop.yaml` with a model fitted
    to fake data** (restored from backup). Test sessions are now `TESTFAKE_*`, kind `test`,
    left out of the catalog, and write their model only inside the session.
17. **The demo guide pointed at the seed model** (`carpet_motion_model.yaml`, the weakest response
    model). It now uses the frozen validated model
    `surfaces/carpet_motion_model_validated_20261103.yaml` (`*_loop.yaml` is rewritten by
    every loop run).
18. **Hardware robustness:**
    * a link stall during homing is reported as `link to the rover stalled` (from the agent's
      reply age, not "motors unpowered"), and the trial waits up to 90 s for the WiFi;
    * a trial with a camera gap > 0.5 s, or that does not cover its sequence, is rejected
      and retried. None of the 56 real trials so far would have been rejected.
    * `ROVER_HOST` now defaults to `rover-lan`.
19. **Smaller items:**
    * stale tests updated (station accept band; a flaky 3 s timeout);
    * the per-feature "only N trials" warning flood (177 lines per run, and wrong about the
      floor) is now one accurate line;
    * the benign all-NaN lidar warning is silenced;
    * the fake rover no longer prints a traceback at teardown.

**Rig changes to deal with at the next session** (not fixable without the rover):

* the black bag right of the grid is gone and the light is now daylight; the bag appears in
  the taught station lidar scans and bounds the workspace. Run `loop/selfcheck.py` (station
  self-consistency), re-teach stations if it warns, and run `calib/verify_stickers.py`
  under the session's lighting;
* the camera guard now matches only ~200 features (1465 last night): re-save its reference
  (`loop/camera_guard.py --save-reference`) with the rover upright on the grid once
  `calib/calibrate_grid.py --verify --source stream --save /tmp/v.png` passes. It passed
  today at 3.4 mm.
* Not a defect: `g_arc`/`v_blend` are not separately identifiable (v_blend sits at its 0.3
  bound; lifting it leaves the fit identical, rms z 0.521, while g_arc runs to 1.5). Quote
  the effective yaw gain at a speed (0.40 at 0.12 m/s, 0.47 at 0.26 m/s), not `g_arc`.

### 2026-11-04 evening: grid convention + equation-derived parameter directions (no rover)

20. **Grid border convention.** The 1 m border is the OUTER edge of the border tape; the inner
    lines are tape centres. `calibrate_grid.py` now defaults to `--border inside`: border
    targets are inset by half the measured tape width, and the width is now measured
    unbiased (area under the soft tape profile; it read +2.5 mm before). The data confirms
    it: the fit residual is minimal within ~1 mm of half the tape (6.3 → 2.9 mm rms, worst
    11.5 → 4.7 mm). The old frame overstated distances by **2.0 %**. The tape is exactly
    1 inch (25.4 mm); it is now a fixed input (`grid_target.TAPE_WIDTH_M`, `--tape-width-mm`)
    because the image measurement varied with the light (22.6-27.5 mm). The 20:24
    calibration, tonight's 20 trials and the Nov-3 corrected copies were re-solved with
    25.4 mm; the live verify passed at 1.5 mm.
    * `calib/grid_calib.json` was re-solved from the 13:30 detections (backup
      `grid_calib.before_border_fix_20261104.json`).
    * All 56 real trials were re-expressed in `real_gridfix/` copies (`calib/border_fix_data.py`).
    * The current model is `surfaces/carpet_motion_model_gridfix_20261104.yaml`
      (k_v 0.959; it was 0.971 in the old frame).
21. **Parameter directions from the equations** (`optimize/param_direction.py`).
    * Closed-form per-step predictions (end-of-step and onset windows) with exact
      derivatives give every parameter's direction, the tests behind it, whether the
      tests pin it, and a staged damped-Gauss-Newton update.
    * `closed_loop.py --update directed` is now the default; every iteration writes
      `directions.txt/.json`.
    * Validation: 19/19 single-parameter directions synthetic and 19/20 on held-out real
      data (the miss is the weakly pinned v_blend); from ideal kinematics the held-out
      error goes 6.73 → 1.10 per feature (p = 3.6e-22), the same as an independent
      least-squares fit.
    * If the coupled step fails, the update falls back to each parameter's own direction,
      so it never stalls (it used to stop on data the model cannot express, e.g. the fake
      rover).
    * Isaac running the held-out model scores 1.25 vs the physics sim's 2.47 (p = 1.6e-7).
    * Presentation package: `docs/presentation_20261105/`.
22. **Low light halves the camera rate.** The C920 drops to ~15 fps in a dim room (mean
    brightness 1/255 tonight). Run with the room lights on and constant.
23. **Next-session preflight, in order** (lights on, rover upright on the grid):
    1. `calibrate_grid.py --verify --source stream --save /tmp/v.png`
    2. `camera_guard.py --save-reference` (the reference is stale since the 13:30 recalibration;
       the loop refuses to start until this is done)
    3. `verify_stickers.py`
    4. `selfcheck.py` (the bag is gone, and the stations were taught in the old frame:
       re-teach them with `stations.py teach <test>` if it warns)

## Making it faster (recommended, in order of payoff)

1. **DONE 2026-11-03: Isaac no longer runs BEFORE/AFTER in model mode.** Both are scored
   offline as distributions (20 model trials, seconds), and Isaac runs once per iteration
   with 2 stochastic sim trials per test. This saves about 5 min per iteration and makes
   the Isaac run measure the distribution. Original note:
   **Do not run Isaac for BEFORE/AFTER in model mode (about -11 min per iteration, -48 %).**
   The response model is deterministic, and the Isaac model drive tracks it to
   0.4 mm/s and 0.0045 rad/s rms (verified). So BEFORE and AFTER can be scored by
   simulating the model itself in milliseconds (`fit_motion_model.simulate`). Run Isaac
   once at the end of a session as a verification, not twice per iteration.
2. **Skip homing when already at the start (about -3 min per iteration).** Every R test
   is out-and-back and ends within a few cm of its start, so the repeat that follows
   could start immediately after a camera check against the accept band.
   Return-to-start is half of the real-trial time.
3. **Use repeats where the scatter is.** Straights repeat within ~2 % (2 repeats are
   enough); spins and arcs vary up to 2x between repeats (use 3-4). Per-test repeat
   counts would put real time where it reduces σ.
4. **Keep one long-running process.** Each script start costs ~20-30 s of rover-link and
   base-driver setup. One loop session with N iterations is far cheaper than N separate
   runs.
5. **Physics-mode optimiser:** every candidate relaunches Isaac (~60 s overhead per
   ~70 s evaluation). A persistent Isaac that reloads parameters would roughly halve
   optimiser time. Model mode avoids it entirely.

## Making it more accurate

* **R5 (Nav2-like transitions) uses 1.0-1.5 s steps.** Their "steady" tail is mostly
  ramp, so its steady terms dominate the residual (worst z 11-18 before the alignment
  fix). Score R5 on the time series (dynamics) instead, or lengthen its steps to 2.5 s.
* **The spin response is slower than the speed response:** tau_w = 0.36 s vs
  tau_v = 0.08 s once timing was fixed. The model has separate time constants, so this
  is captured. A spin breakaway delay was suspected but turned out to be the alignment
  bug.
* **Pool real data across sessions** (`--prior-real`) so each refit uses every trial
  recorded so far; σ estimates improve with every repeat.
