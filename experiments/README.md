# Navigation experiments — sim nav-system comparison (thesis data)

The experiment is **entirely in the Isaac sim** (`warehouse_basic_obstacles.usd` has no
real-world twin): run a **selected nav system** to a **selected goal**, for **N trials**,
and record each run into one data sheet + a path plot for the paper. It compares the nav
systems against each other in simulation, under two conditions (known-map vs cold-start).

**The one real-world tie-in is goal 4:** the runner also saves the exact `/cmd_vel` command
stream from goal-4 runs to `goal4_replay/`, so those commands can be replayed on the real
rover to check it ends up in the same place (open-loop sim→real). Everything else is sim.

```
experiments/
├── goals.yaml            the 4 goal points (map frame): G1..G4  [edit names/coords here]
├── make_occupancy_map.py build the shared ground-truth 2D map from the sim USD
├── run_nav_trial.py      run trials + record results.csv + path/cmd CSVs + map.npz
├── plot_paths.py         draw the trajectories over the map  ->  figure (PNG + PDF)
├── summarize.py          aggregate results -> per-condition comparison table (summary.csv)
├── goal4_replay/         goal-4 /cmd_vel streams + send_to_rover.sh (the sim→real check)
├── maps/<env>/           map.pgm + map.yaml + map.npz + map_preview.png (the shared map)
└── data/<tag>/           outputs (one dir per condition+nav+goal), git-ignored
```

## Two experiment conditions (why there are two)
- **Experiment A — known-map:** every stack starts with the SAME complete map of the
  environment and only has to *localize + navigate*. Fairest navigation comparison.
- **Experiment B — cold-start:** every stack starts mapless and must map+localize+navigate
  from scratch (this is the original "blind" setup; the visual `--prime` drive lives here).
Keep the Nav2 planner/controller/costmap/footprint constant across stacks in BOTH — only
the sensor/localization source changes. Report A and B as separate results.

## The shared ground-truth map (Experiment A)
Build one authoritative 2D occupancy grid straight from the sim geometry — deterministic,
no driving, anchored at the rover start (rover start = map `(0,0)`), so `goals.yaml` lines
up unchanged:
```bash
~/Applications/isaacsim/python.sh experiments/make_occupancy_map.py \
    --env warehouse_basic_obstacles.usd --out experiments/maps/warehouse
# -> experiments/maps/warehouse/{map.pgm,map.yaml,map.npz,map_preview.png}
```
It boots a headless SimulationApp so the Isaac S3 warehouse props (walls/racks) resolve
exactly as the sim sees them; check `map_preview.png` (it also marks each goal FREE/OCC).
> **Goal check:** the preview flags **G1_south `(1.5,-11.0)` as `OCC` (on an obstacle) —
> unreachable. Nudge it into free space in `goals.yaml`.** G2/G3/G4 are FREE.

### Known-map mode per stack (Experiment A)
Each stack loads a **prior map in its own native format**, all stored under
`experiments/maps/warehouse/`. Build the store ONCE with a mapping run (drive the whole
environment, then stop the stack), then every trial launches in localization-only mode —
no priming drive. Every path below is absolute; `$PWD` assumes you're at the repo root.

**lidar / depth** — shared occupancy grid + AMCL (initial pose defaults to `(0,0,0)` = the
map origin, so no manual 2D-Pose-Estimate):
```bash
ros2 launch isaac_nav_lidar.launch.py use_slam:=false \
    map:=$PWD/experiments/maps/warehouse/map.yaml       # nav_depth: identical flags
```
**rtab** (RTAB-Map db) — one file for both modes:
```bash
# map once (drive, then Ctrl-C to save):        then localize each trial:
ros2 launch isaac_nav_rgb.launch.py localization:=false \
    database_path:=$PWD/experiments/maps/warehouse/rtabmap.db
ros2 launch isaac_nav_rgb.launch.py localization:=true \
    database_path:=$PWD/experiments/maps/warehouse/rtabmap.db
```
**cuvslam** (saved cuVSLAM map folder) — saves on shutdown, relocalizes on startup:
```bash
ros2 launch isaac_nav_cuvslam_nvblox.launch.py \
    map_save_path:=$PWD/experiments/maps/warehouse/cuvslam_map        # map, then Ctrl-C
ros2 launch isaac_nav_cuvslam_nvblox.launch.py localize_on_startup:=true \
    map_load_path:=$PWD/experiments/maps/warehouse/cuvslam_map        # localize
```
**orbslam** (ORB-SLAM3 atlas `.osa`) — **rebuild the wrapper once first**
(`ugv_controller_v2/nav_systems/nav_orbslam3/build.sh`) so the atlas args exist:
```bash
ros2 launch isaac_nav_orbslam3.launch.py \
    save_atlas:=$PWD/experiments/maps/warehouse/orbslam3_atlas        # map, then Ctrl-C
ros2 launch isaac_nav_orbslam3.launch.py localization_mode:=true \
    load_atlas:=$PWD/experiments/maps/warehouse/orbslam3_atlas        # localize
```
All four default (no map/atlas args) to their original **live-mapping** behavior, so
Experiment B (cold-start) uses the same launches with none of these flags. Because the rover
always starts at the same USD pose, each system's map anchors at `(0,0)` and lines up with
`goals.yaml` run-to-run. In Experiment A pair the runner with `--init-maneuver none` (lidar)
or a short `sweep` (visual stacks relocalize almost immediately from the loaded map).

## The loop
1. **Bring up ONE nav system** (this harness does not launch it — you pick the system by
   launching it, which is the interchangeable knob):
   - **Sim:** `isaacpy ugv_controller_v2/run_obstacle_warehouse.py` (terminal 1) +
     a nav stack, e.g. `navlaunch -v2 -rtab` (terminal 2).
   - **Real:** `~/rover_nav/start_bringup.sh` + the matching `~/rover_nav/start_<nav>.sh`.
2. **Run trials** (terminal 3 — system ROS 2, sourced):
   ```bash
   python3 experiments/run_nav_trial.py --nav rtab --condition known_map --goal 3 --trials 5 --return-home
   python3 experiments/run_nav_trial.py --nav cuvslam --condition cold_start --goal 4 --trials 3
   ```
   - `--nav {lidar,depth,rtab,cuvslam,orbslam}` — match the stack you launched
   - `--condition {known_map,cold_start}` — Experiment A or B (launch the stack the matching way)
   - `--goal {1,2,3,4}` (or `--goal-xy X Y`) — **goal 4 also exports its `/cmd_vel` to
     `goal4_replay/`** for the real-rover check; `--trials N`
   - `--return-home` drives back to the start pose between trials so each departs identically
   - **Standardized init (replaces the old `--prime`):** once per run, before any trial's
     timer, the runner does a **fixed maneuver** (`--init-maneuver sweep`, identical for every
     stack — translate out/back + a full left/right sweep, the motion visual SLAM needs) then
     waits on a **readiness gate** (`map->base_footprint` stable for `--init-settle` s, up to
     `--init-timeout`). The nav timer starts only after the gate. Init time/distance/success
     are recorded separately and are **not** counted in navigation time.
     - Experiment A (known-map lidar/AMCL) needs no motion: add `--init-maneuver none` (gate only).
     - Experiment B (cold-start cuvslam/orbslam): keep `--init-maneuver sweep` so tracking starts.
     - If the gate never passes, the run records one `INIT_FAILED` row and skips the trials.
     - (`--prime` is deprecated: accepted but ignored, with a warning.)
3. **Plot the paths** for the paper:
   ```bash
   # compare nav systems on one goal, over the shared ground-truth map:
   python3 experiments/plot_paths.py experiments/data/*_G3_east --group-by nav_system \
       --map experiments/maps/warehouse/map.npz --title "Nav systems, G3"
   # compare the two conditions for one system:
   python3 experiments/plot_paths.py experiments/data/known_map_rtab_* experiments/data/cold_start_rtab_* \
       --group-by condition --title "RTAB: known-map vs cold-start"
   ```
4. **Aggregate** into the comparison table:
   ```bash
   python3 experiments/summarize.py experiments/data/*/results.csv --out summary.csv
   ```

## The data sheet (`results.csv`, one row per trial)
`timestamp, condition, nav_system, goal, goal_x, goal_y, trial,
init_maneuver, init_time_s, init_dist_m, init_success, result,
duration_s, wall_duration_s, path_length_m, straight_line_m, path_efficiency,
start_x, start_y, final_x, final_y, final_pos_error_m, final_yaw_error_deg,
n_recoveries, n_restarts, avg_speed_mps, n_samples, path_file, cmd_file`

- **condition** — `known_map` (Experiment A) or `cold_start` (Experiment B). **cmd_file** —
  the trial's recorded `/cmd_vel` stream (goal-4 trials are also copied to `goal4_replay/`).

- **init_maneuver / init_time_s / init_dist_m / init_success** — the standardized
  initialisation (per run, same on every trial row): which maneuver ran, and how long /
  how far the robot moved before the readiness gate passed (`init_success`). This time is
  **separate** from navigation time — report it on its own (esp. for Experiment B).
- **result** — SUCCEEDED / FAILED / CANCELED (timeout) / **INIT_FAILED** (gate never passed,
  so no trials ran). **n_restarts** — how many times a FAILED goal was re‑issued before
  success/giveup. **n_recoveries** — Nav2 behavior‑tree recoveries (spins/backups).
- **duration_s** is sim time (Isaac `/clock`); **wall_duration_s** is wall-clock (the sim
  runs ~0.35× real-time). Compare stacks on `duration_s`.
- **path_efficiency** = straight‑line / actual path length (1.0 = perfectly direct).
- Each trial's full trajectory is in `paths/<...>.csv` (`t,x,y,yaw`), and `map.npz` is an
  occupancy snapshot for the plot background.

## Repeatability notes (important for the thesis)
- **Start pose:** goals are in the `map` frame, which SLAM anchors at the rover's start.
  Start every run from the **same sim pose** (the USD origin) so goals line up run‑to‑run
  and across nav systems. `--return-home` keeps multi‑trial runs consistent.
- **Fair comparison:** keep the Nav2 planner/controller/costmap/footprint constant across
  stacks — only the sensor/localization source changes. Run identical `--goal`/`--trials`
  for every `--nav` and both `--condition`s; `summarize.py` gives the per‑condition table.
- **Goal 4 → real rover:** goal‑4 command streams land in `goal4_replay/`. Calibrate the sim
  physics first (`../ugv_controller_v2/calibration/`) so replaying them on the real rover
  lands in the same spot — that residual gap is the sim→real measurement.
