# Runbook: calibrating the wheel-driven Isaac rover against the real rover

The simulated rover always drives its wheels: `/cmd_vel` → the baked skid-steer
`Drive_Graph` → wheel joint velocity drives → PhysX wheel-floor contact. Calibration
tunes the physics parameters of that drive and contact (`ugv_controller_v2/calibration/calib_params.py`);
geometry and masses are locked.

Reference session: concrete, 2026-11-11/12. Training trials `loop/runs/concrete_20261111_140923/iter_01/real`
(12 tests × 3), held-out `iter_02/real` (12 × 3); staged fit → `surfaces/concrete_physics.yaml`;
held-out report `loop/runs/verify_20261112_082754/report.md` (rms z 4.97 → 2.41).

## 1. Preflight (lights on and steady, rover upright on the grid)

```bash
cd ~/projects/isaac-assets && source ros2env.sh && cd sim2real_calib
python3 calib/calibrate_grid.py --verify --source stream --save /tmp/v.png   # expect PASS
python3 loop/selfcheck.py                                                    # expect READY: 0 fail
python3 calib/verify_stickers.py                                             # expect RESULT: PASS
```

* Use full room light: low light halves the camera frame rate and causes the > 0.5 s
  gaps that get trials rejected.
* Do **not** re-run `calibrate_grid.py` without `--verify` mid-session: it changes the
  frame relative to trials already recorded. If you must, re-save the camera reference
  (`loop/camera_guard.py --save-reference`).
* Teach a start station for every test on a new floor: `python3 loop/stations.py teach <test>`.

## 2. Power

The pack is failing; spins sag it hardest and the loop parks and stops below
`loop_config.yaml battery_min_v`. Prefer wall power. A cut-short run loses nothing:
re-run with `--resume <session>`; trials are recorded round-robin so every test keeps
held-out repeats.

## 3. Record the real battery (training + held-out)

```bash
python3 loop/closed_loop.py --surface concrete --real-only --iterations 2 --repeats 3 \
    --tests T1,T2,T3,T3b,T4,T5,T6,R1,R2,R3,R4,R5 --no-hold
```

Iteration 1 is the training set, iteration 2 the held-out set. When the last trial is
recorded the log and the dashboard say `REAL ROVER DONE`: the rover can be powered off.

## 4. Fit every physics key in stages (no rover, ~5 h)

```bash
setsid nohup bash loop/staged_physics.sh loop/runs/<session>/iter_01/real concrete \
    > /tmp/staged.log 2>&1 &
```

| Stage | Keys | Tests |
|---|---|---|
| A ramps | max_acceleration, max_deceleration, max_angular_acceleration, drive_damping | T1, T5, T2 |
| B caps | max_linear_speed, max_angular_speed | T3b, T6 |
| C traction | wheel_distance, corner/middle/floor mu (static, dynamic), drive_max_force | T1, T3, T2, T6, R2 |
| D damping | base_linear_damping, base_angular_damping | T1, T3, T5, T2, T6 |
| E joint | all 16 keys, small trust region | full battery |

Each stage writes `surfaces/<surface>_physics_<stage>_<stamp>.yaml`; the last becomes
`surfaces/<surface>_physics.yaml`. If stage E does not beat D, copy D over it (concrete:
E 2.34 vs D 1.68, so D was kept). Resume a stage with `START=C INIT=<previous yaml>`.

## 5. Held-out check

```bash
python3 loop/verify_params.py --surface concrete --calibrated surfaces/concrete_physics.yaml \
    --combine-mode average --tests T1,T2,T3,T3b,T4,T5,T6,R1,R2,R3,R4,R5 \
    --real-from loop/runs/<session>/iter_02/real
```

Writes `loop/runs/verify_<stamp>/report.md`: rms z, per-feature z and path errors (cm,
deg) for the baseline and the calibrated parameters on the same held-out trials.

## 6. Quick closed loop (fewer keys, rover in the loop)

```bash
python3 loop/closed_loop.py --surface carpet --keys wheel_distance,drive_max_force \
    --tests T1,T2,T4 --repeats 3 --iterations 2 --max-evals 12
```

Each iteration: real trials → targets → Isaac BEFORE → bounded/Nelder-Mead search in a
shrinking trust region (one Isaac launch per candidate, whole battery) → AFTER. The next
iteration's fresh real trials test the previous update.

## Automatic recovery

| Problem | What the loop does |
|---|---|
| Camera gap > 0.5 s | trial rejected and re-run |
| WiFi stall | waits for the link, reopens it (LAN, then Tailscale) |
| Base driver dead | restarts it on the rover |
| Battery below the floor | parks the rover and stops; `--resume` continues |
| Isaac launch fails | stale `/dev/shm/fastrtps_*` is cleared before every launch |
