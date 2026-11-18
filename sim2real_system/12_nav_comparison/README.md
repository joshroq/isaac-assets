# navcmp — real vs simulated rover under autonomous navigation

**What it is for:** one question only — *does the simulated rover navigate like the real
rover?* Both rovers drive the **same course** with the **same geometric nav stack**
(slam_toolbox + Nav2; lidar, or lidar + OAK depth cloud — **no visual SLAM, no RGB**), and
the session reports how close each simulator is. It is how you test the **optimizer's
results** at the navigation level: the fitted wheel-contact physics (`sim_physics`, the sim
rover driving its wheels through PhysX) against the un-calibrated sim (`sim_default`), both scored against
the real rover's own repeat-to-repeat scatter. It tunes nothing.

```
course YAML ──► Isaac builds the arena (run_nav_course.py)   you build it on the grid
                         │                                            │
bundle (one launch + ONE Nav2 params file + runner + course) ──► ssh → rover (Jetson-local ROS)
                         │                                            │
              sim: Isaac + stack + runner                real: bringup_lidar + stack + runner
                         └───────── legs.csv / trace.csv / map.json ──┘
                                              │
                                  report/report.md + figures
```

## Files

| file | runs on | role |
|---|---|---|
| `session.py` | desktop | orchestrator: per condition stage bundle → [Isaac] → stack → runner → fetch → teardown; `--resume`; `--fake-real` |
| `mission_runner.py` | **rover and desktop** | waits for Nav2 (re-issues lifecycle STARTUP if bringup aborted), registers the course frame, drives every leg via `navigate_to_pose`, records 10 Hz traces; safety stop |
| `stack/navcmp_stack.launch.py` | **rover and desktop** | slam_toolbox (sync) + stock Nav2 `navigation_launch.py` with the shared params; `platform:=real` adds `ugv_bringup bringup_lidar`, `platform:=sim` remaps `/scan`,`/odom` to Isaac's |
| `nav_params.py` | desktop | builds the ONE Nav2 params file from `ugv_nav/params/teb.yaml` (filters/map server dropped, BT XMLs shipped, depth source for `--stack depth`) |
| `course.py`, `courses/*.yaml` | everywhere | arena walls/boxes, start, goals, fence; SE(2) helpers; occupancy map → boxes |
| `../../ugv_controller_v2/run_nav_course.py` | Isaac | ground plane + the course geometry, placed so the rover spawn = course start |
| `conditions.yaml` | — | what `real`, `sim_physics`, `sim_default`, `fake_real` mean |
| `metrics.py`, `report.py` | desktop | per-leg metrics, rms z vs real scatter, path gap (Frechet), report |
| `tests/` | desktop | offline tests (`synth.py` makes synthetic sessions) |

## Why a bundle instead of the existing launch scripts

The rover's `start_lidar_nav.sh` loads `ugv_nav/param/slam_nav.yaml` from the Jetson's
older ugv_nav, and `start_depth_nav.sh` runs `nav.launch.py use_slam:=true`, which ignores
`use_slam` (AMCL against a stale map). The Isaac launches load `ugv_nav/params/teb.yaml`.
So the stacks were never guaranteed identical. Every session ships one generated params
file, one slam_toolbox config (= `ugv_slam` online_sync) and one launch file to both
sides, and records their sha256 in `session.json`. The only platform differences are
sensor plumbing: the real side starts the base, LD19 and TF; the sim side remaps to
Isaac's topics and uses sim time.

## Run it

```bash
cd ~/projects/isaac-assets && source ros2env.sh && cd sim2real_calib
python3 navcmp/course.py navcmp/courses/grid2x2_box.yaml     # check a course
```

**No rover (pipeline check):** `real` is played by Isaac driving a deliberately perturbed
copy of the calibrated physics parameters. Sessions are named `TESTFAKE_*` and the report says so.

```bash
python3 navcmp/session.py --course navcmp/courses/grid2x2_box.yaml --stack lidar \
    --trials 2 --conditions real,sim_physics,sim_default --fake-real --yes
```

**With the real rover:**

1. Build the arena from the course YAML on the taped grid. Walls and boxes must be at least
   0.40 m tall (the LD19 scan plane). Courses: `grid1x1_square` (shake-down),
   `grid2x2_box` (loop around a box), `grid1x4_slalom` (long arcs: where the drive model
   matters most).
2. Rover on **wall power or a charged pack**, reachable as `ssh rover-lan` (override with
   `ROVER_HOST=rover`). **Nothing else running on it**: the session starts its own
   bringup. A second `ugv_bringup` would shred the serial port, and preflight refuses.
   Halt first with `~/rover_nav/stop_goal.sh`, then `~/rover_nav/stop_all.sh`.
3. Run:
   ```bash
   python3 navcmp/session.py --course navcmp/courses/grid2x2_box.yaml --stack lidar \
       --trials 5 --conditions real,sim_physics,sim_default
   ```
   The `real` condition runs first. After preflight (reachable, ROS isolated, packages
   present, base free) it asks you to put the rover on the start mark and press Enter.
   The SLAM map is anchored there. Trials then run back to back: each trial ends with the
   scored `return` leg back to start. When the log says **`REAL ROVER DONE - safe to
   power it off`**, the rest is Isaac on the desktop.
4. Read `navcmp/runs/<session>/report/report.md`.

Options: `--stack depth` (also starts `~/rover_nav/start_oak.sh` on the rover; Isaac
renders depth at 640x480), `--world-from-real-map` (sim conditions also get walls from the
real run's slam map, so Isaac sees the room beyond the arena), `--nav-overrides x.yaml`
(`{node: {param: value}}`, applied to both rovers), `--on-fail stop-trial`,
`--show-isaac`, `--lidar-profile native`, `--resume <session>` (each invocation writes `parts/partN`; `data/`
is the merge; trial numbering continues).

## Safety (real rover)

The Waveshare base has **no cmd_vel deadman** (the ESP32 holds the last command), so
the runner on the Jetson cancels the goal and publishes zero `/cmd_vel` for 1.5 s on
any of these:

- leaving the course `fence`;
- a leg timeout (sim-time timeout plus a wall-clock bound);
- SIGINT or SIGTERM;
- the battery dropping below 10.4 V;
- its SSH stdin closing (the desktop or the link died).

Only after that does the session stop the stack (base driver), then the OAK driver.
`rover_link`'s isolation rule applies: the rover's ROS must be `ROS_LOCALHOST_ONLY=1`,
otherwise the desktop's Isaac `/cmd_vel` could drive it.

## What the report measures

Per leg (`metrics.py` has exact definitions): success, time, path length, efficiency,
mean speed, share of time turning in place or standing still, minimum lidar clearance,
final position/heading error, and Nav2 recoveries. Diagnostics: `track_v`/`track_w`
(achieved ÷ commanded, the response the optimizer fitted) and odometry-vs-SLAM drift.

- **rms z**: each sim trial's metric minus the real mean, divided by the real sd (floored),
  RMS over metrics × legs × trials. Dimensionless; ≈ 1 means "looks like one more real
  repeat". It uses the same units as the optimizer's rms z.
- **real-vs-real floor**: the same score for each real trial against the other real trials
  (needs ≥ 3 real trials). A perfect simulator scores about this.
- **path gap ratio**: median Frechet distance sim-vs-real ÷ real-vs-real, per leg, for
  successful legs. 1 means the sim paths differ from the real ones no more than real
  repeats differ from each other.
- 95 % CI by resampling whole trials. Use ≥ 5 trials per condition before claiming a
  difference.

Positions come from each rover's own SLAM pose in the course frame. Both rovers are
measured by the same instrument, with no overhead camera and no visual data. The course
frame is registered from the rover's pose at the start mark, so a real rover placed a few
cm off shifts its whole trial by that much. Place it carefully.

## Sim lidar: fixed for this comparison (finding, 2026-11-11)

The rover USD's lidar references NVIDIA's generic `Example_Rotary_2D` profile:
`range_min 1.0 m`, `range_max 200 m`, 3600 beams. **The Isaac rover was blind to
everything within 1 m.** In the first smoke run on the 2.5 m arena, 39 % of beams were
invalid and slam_toolbox lost the rover: Isaac's ground truth went around the arena while
SLAM thought it had moved 0.5 m, and 3 of 4 legs timed out. The real LD19 ranges about
0.02–12 m.

`run_nav_course.py --lidar-profile ld19` (the session default) overrides the OmniLidar
near/far range to 0.10–12 m, in memory only. After the fix, SLAM tracked Isaac's ground
truth within a few cm and every leg succeeded. `--lidar-profile native` reproduces the old
behaviour.

The other Isaac launchers (warehouse experiments, `run_ground_plane.py`) still have the
1 m blind zone. It mattered less in the large warehouse, but it is a sim-to-real
difference worth fixing there too. The beam count (3600 vs LD19 ~450–500) is NOT changed.
Each session also saves one scan at the start mark per condition, and the report's
"sensor check" overlays the real and sim lidar views. A large self-occlusion sector
(≈60–165° in the sim lidar frame, the mast/arm) shows up there too: compare it with the
real one on the first hardware run.

## Known limits

- The real arena beyond the walls is invisible to Isaac unless `--world-from-real-map` is
  used; keep walls tall and the arena closed.
- Isaac's LD19 model, the depth noise, and floor friction outside the drive model are
  sim approximations. They are part of what the comparison measures, not something it
  corrects.
- Isaac renders the depth cloud at only ~1 Hz wall clock (640x480, measured 2026-11-11)
  and runs ~0.6x real time with the RTX lidar, so the depth obstacle layer updates far less
  often than on the real OAK. The report prints the cloud rate per condition. Times in the
  report are ROS-clock seconds (sim time in Isaac), so the slower wall clock does not
  inflate them.
- Nav2's goal checker (`xy_goal_tolerance 0.25`, stateful) ends a leg anywhere within
  25 cm of the goal, so final errors of ~15–25 cm are normal on both rovers.
- Trials share one SLAM session per condition (no reset between trials), the same on both
  rovers.
- The real-rover path (SSH, `bringup_lidar` include, OAK start, battery guard) has not
  been run on hardware yet. Everything else was smoke-tested with `--fake-real`; see the
  first real session's logs (`real/stack.log`, `real/runner.log`).
