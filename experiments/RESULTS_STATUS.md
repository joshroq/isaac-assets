# Experiment status & limitations (as of 2026-08-07)

Goal G4_center `(5.0, 0.5)`, 5 trials/cell, Nav2 planner/controller/costmap held constant;
only the localization source changes. `duration_s` = sim time. Full per-trial data in
`data/master_results.csv`; per-cell figures in each `data/<cell>/routes.png`.

## Scoreboard

| stack | cold-start | known-map | notes |
|---|---|---|---|
| **lidar** (slam_toolbox / AMCL) | **5/5** | **5/5** | err 0.12–0.22 m, eff ~0.6 |
| **depth** (RTAB ICP / AMCL) | **5/5** | **5/5** | err 0.04–0.19 m |
| **rtab** (RTAB-Map RGB-D) | **5/5** | **5/5** | err 0.17–0.25 m |
| **orbslam** (ORB-SLAM3 RGB-D) | **3/5** | **not achievable** | marginal; see below |
| **cuvslam** (cuVSLAM + nvblox) | **0/5** | **not achievable** | hardware-blocked; see below |

**Six cells (lidar/depth/rtab × both conditions) are solid and report-ready.** The two
visual-VIO stacks fail for *different* reasons — one hardware, one algorithm/scene.

## cuVSLAM — hardware-blocked (needs ≥16 GB VRAM)
- Failure signature: `free(): invalid pointer` → container `exit -6` (heap corruption).
  Isaac's RTX renderer + cuVSLAM's **stereo** pair + nvblox's TSDF/ESDF exceed the **8 GB**
  RTX 5060 Ti. This is a hard VRAM ceiling, not a tunable.
- Mitigations already applied (did **not** solve it): camera render at 640×360,
  `nvblox map_clearing_radius_m` 6→3, container `respawn=True`, jitter tol 34→300 ms,
  EKF pose-rejection 20→5 m. `respawn` keeps the process alive but **each crash resets
  cuVSLAM's tracking**, which is the observed drift (start pose runs away 0→17→28 m → 0/5).
- **Decision: shelve cuVSLAM until a ≥16 GB GPU is in the box** (e.g. the 16 GB RTX 5060 Ti
  variant, RTX 4060 Ti 16 GB, or the RTX 5000 Ada 32 GB). It is VRAM-bound, not compute-
  bound; on adequate VRAM this stack should be competitive. All fixes above stay in place
  so a re-run on better hardware is one command.

## ORB-SLAM3 — algorithm/scene-limited (NOT hardware-fixable)
- **Cold-start = 3/5** with 0.24 m accuracy on the successful trials — the method works,
  and the slowed-rotation fix helped. Trials 4–5 degrade catastrophically (75 m / 124 m
  paths) because the between-trial **return-home rotation breaks ORB-SLAM3 tracking** and
  the pose never recovers, so later trials start already lost. 425 `Fail to track local
  map` events across the run.
- **Known-map = not achievable.** The mapping run lost tracking **475×** and fragmented
  into **11 disjoint sub-maps**; ORB-SLAM3's atlas serializer then **segfaults**
  (`SaveAtlasToFile` → `exit -11`) trying to save that fragmented atlas, so no `.osa`
  prior is ever written and the cell is (correctly) skipped by the fail-fast guard.
- This is inherent to the low-texture warehouse + the non-standard RGB-D-from-sim feed;
  a bigger GPU does not help (ORB-SLAM3 is light on VRAM).
- **Decision: keep as an honest result** — report ORB-SLAM3 as the marginal stack
  (3/5 cold-start; known-map infeasible because the scene prevents a coherent atlas).

## System note — NVIDIA driver mismatch
`nvidia-smi` fails with **driver/library mismatch** (kernel NVRM `595.71.05` vs NVML
`595.84`). A driver package updated without a module reload. **Reboot to resync before any
further GPU-heavy runs** — it also may have aggravated the cuVSLAM heap instability.

## Fixes applied this session (all retained)
- `ugv_nav/localization_launch.py`: guarded optional `cartographer` import (unblocked
  known-map lidar/depth AMCL).
- `run_matrix.sh`: visual stacks render at 640×360; map-prep saves priors via graceful
  SIGINT + verifies + fail-fast skip if the prior is missing.
- `isaac_nav_cuvslam_nvblox.launch.py`: jitter 34→300 ms, nvblox radius 6→3, respawn.
- `ekf_map_fusion.yaml` (cuvslam & orbslam): pose-rejection 20→5 m.
- `rpp.yaml`: rotation 1.8→0.5 rad/s, velocity-smoother angular cap 1.0→0.5, linear 0.5→0.3,
  angular accel 3.2→1.2 (visual stacks only).
- `run_nav_trial.py`: init sweep rotation 0.6→0.4 rad/s.
- `isaac_rgbd.yaml`: ORB features 1200→2000.

## Also confirmed
- **Obstacle collisions OK** — all 14 added shapes (3 cones, 4 cubes, 3 cylinders, 3
  spheres) + floor plane have `CollisionAPI`/`collisionEnabled` (see
  `experiments/inspect_collisions.py`). Not a cause of any failure.
- **known-map lidar's earlier 100s-of-metres runs** were `/map` contamination from a stray
  slam_toolbox process, not a launch bug — a clean slate fixed it (now 5/5). Kill detached
  runs with a **process-group** kill (`kill -9 -<pgid>`, sandbox off); plain pkill misses them.

## Next steps
1. Reboot (fix the driver mismatch).
2. cuVSLAM: re-run both conditions once a ≥16 GB GPU is available (`NAVS=cuvslam` in
   `run_matrix.sh`). Also worth splitting cuVSLAM/nvblox into separate containers there.
3. ORB-SLAM3: report the 3/5 cold-start + known-map-infeasible as the honest finding.
4. The paper's core comparison (lidar/depth/rtab, both conditions) is complete.
