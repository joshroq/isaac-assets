# Goal‑4 command replay — the one sim→real tie‑in

The experiment is **sim‑only** (the warehouse has no real twin). The single real‑world
check is here: replay the **exact `/cmd_vel` command stream** the sim produced driving to
**goal 4** on the **real rover**, and verify it ends up in the same place. Goal 4 is the
"more complicated movement" target, so it's the interesting open‑loop sim→real test.

## Where these files come from
`run_nav_trial.py ... --goal 4` automatically drops each trial's command stream here:
```
goal4_replay/<nav>_<condition>_trialNN.csv        # columns: t, vx, wz  (rover replay format)
```
e.g. `rtab_known_map_trial01.csv`. Same format the rover's `~/rover_nav/calib/replay_cmdvel.py`
consumes — no conversion needed.

## Send one to the real rover and replay it
Prereqs: sim physics **calibrated** to the real rover first (see
`../../ugv_controller_v2/calibration/`), so the same commands produce the same motion; and
the rover's sensor/driver bringup running (`~/rover_nav/start_bringup.sh`). **On blocks
first, then floor with a spotter** — this is open‑loop, the rover just executes the commands.
```bash
experiments/goal4_replay/send_to_rover.sh rtab_known_map_trial01.csv      # alias 'rover'
```
It copies the CSV to the rover and runs the replay, publishing straight to `/cmd_vel` (no
runtime scaler — the calibration lives in the sim USD). Then measure where the real rover
stopped vs. where the sim finished (the trial's `final_x/final_y` in `results.csv`) to
quantify the sim→real gap.

> Because it's open‑loop (no localization feedback on the real rover), drift accumulates —
> that gap *is* the measurement. Better sim calibration → smaller gap.
