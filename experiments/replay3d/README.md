# replay3d — replayable 3D views of navigation trials

Turns each trial's recorded trajectory into an interactive 3D scene you can orbit and
play/scrub through time. The **actual rover** (loaded from its URDF meshes) drives its
recorded path through the warehouse (3D obstacles from the shared occupancy map), and you
can toggle **RViz-style layers** that mirror what each nav stack's RViz config shows.
**No ROS or Isaac needed to view** — pure offline HTML + a bundled copy of Three.js.

## View
Open **`index.html`** in any browser (double-click / `file://` works — no server needed).
Pick a trial, then:
- **Orbit** (drag), **zoom** (scroll); bottom bar = **play/pause**, **scrub**, **speed**.
- **view** dropdown picks a per-nav preset (**lidar / depth / rtab**) matching that stack's
  RViz layer mix; it defaults to the trial's own nav system.
- **layer checkboxes:** `3D obstacles`, `Map` (occupancy overlay), `Path`, `LaserScan`
  (ray-cast against the map), `PointCloud` (forward depth/RGB-D fan), `Grid`, `TF`.

## Replay options
1. **This HTML viewer** — offline, no tools: real URDF robot + 3D objects + track. Best for
   quick sharing and eyeballing trajectories. (Sensor layers were removed as unsatisfactory.)
2. **In-Isaac media-remote viewer** — `experiments/play_replay.sh <replay.usda>` opens a
   physics-neutralised Stage Recorder sampled replay in Isaac Sim with
   play/pause/fast-forward/rewind (speed ladder).

## What the HTML shows
The HTML viewer shows only things that are real or ground-truth (no fake sensors):
- **RobotModel** — the actual URDF (`ugv_rover_isaac_fixed.urdf`) meshes, embedded inline
  (base64 vertex buffers) so it renders over `file://` with no loader.
- **3D objects** — the warehouse obstacles from the shared occupancy map.
- **Path** — the actual recorded trajectory; start (green) / goal (red pole).
- optional **Map** (occupancy overlay) / **Grid** / **TF** toggles.

## Regenerate / extend
```bash
python3 experiments/replay3d/make_replay3d.py --all     # all G4 lidar/depth/rtab trials
python3 experiments/replay3d/make_replay3d.py experiments/data/cold_start_lidar_G2_northwest
```
`--all` = the non-visual G4 cells. Pass any `data/<cell>` dirs to render other goals/stacks
(e.g. once G2/G3 finish). Reads each cell's `results.csv` + `paths/*.csv`, the shared
`maps/warehouse/map.npz`, and the URDF; INIT_FAILED trials are skipped.

## Files
- `vendor/` — Three.js r128 (three.min.js, OrbitControls.js, STLLoader.js), offline.
- `robot/meshes/*.stl`, `robot.js` — rover meshes + per-link transforms from the URDF.
- `warehouse.js` — shared obstacle geometry (occupancy grid downsampled to 0.2 m).
- `viewer.js` — shared scene, layers, ray-cast sensors, playback.
- `trials/<cell>__trialNN.html` — one self-contained replay per trial.
- `index.html` — links to every replay, colour-coded by result.

Scope: generated for **lidar / depth / rtab**. cuVSLAM/ORB-SLAM3 are excluded per the
experiment status (`../RESULTS_STATUS.md`).
