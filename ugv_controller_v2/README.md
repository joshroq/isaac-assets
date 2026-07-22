# ugv_controller_v2 — baked-graph rover testbed

The next-generation rover control/nav testbed, reworked around
`jetson_rover_v2.usd`, which now has the **entire ROS 2 interface baked in** as
OmniGraphs + an articulation Drive_Graph. The old `ugv_controller/` is left
untouched; this is a clean parallel implementation.

## What changed vs `ugv_controller/`

| | `ugv_controller/` (old) | `ugv_controller_v2/` (this) |
|---|---|---|
| ROS 2 graphs | built in Python at launch (`ros2_bridge_drive_graph.py`, `depth_camera_graph.py`) | **baked into the USD** |
| Wheel drive | Python `RoverDriveController` + `Ros2CmdVelDriver` | **baked `DifferentialController` graph** |
| Keyboard | Isaac-side `KeyboardOverrideController` (`\` handoff) | **ROS 2 `/cmd_vel` teleop** (`keyboard/`) |
| Isaac runner | `ugv_sim.py` (~460 lines, edits physics schemas, arbitrates writers) | `common/run_baked_rover.py` (open USD, play) |
| Control path | keyboard vs ROS arbitrated in Python | **only `/cmd_vel`** — whoever publishes it drives |

The rover is now driven **exclusively by `geometry_msgs/Twist` on `/cmd_vel`**,
whether that comes from the keyboard teleop or from Nav2. No Python driver, no
write-arbitration.

## Part 1 — SLAM/Nav2 fidelity baseline (current thesis focus)

> **Current thesis Part 1 is not randomized-object navigation.** Part 1 uses a
> carefully matched real/sim room to evaluate SLAM/Nav2 transfer, with controlled,
> repeatable geometry, object placement, route, start pose and goal pose.
> **Randomized-object environment experiments are paused. They belong to the later
> policy-training/generalization phase, not the current SLAM/Nav2 fidelity baseline
> phase** — their code is preserved (not deleted) under `nav_experiment/`. Do not
> create new randomized-object trials unless explicitly asked.

The Part 1 navigation stacks (each differs only in what supplies `map->odom` and
what fills the Nav2 costmap; the planner/controller/`/cmd_vel` output are the shared
upstream `ugv_nav` stack):

| # | stack | dir | localization | obstacle/costmap | RGB | depth | LiDAR | IMU | role |
|---|---|---|---|---|:--:|:--:|:--:|:--:|---|
| 1 | **LiDAR-only Nav2** | `nav_lidar` | LiDAR SLAM (slam_toolbox) | LiDAR `/scan` | – | – | ✅ | – | geometry-only baseline |
| 2 | **LiDAR + raw depth Nav2** | `nav_depth` | LiDAR SLAM (slam_toolbox) | LiDAR + raw depth PointCloud2 | – | ✅ | ✅ | – | geometry/depth baseline |
| 3 | **RTAB-Map RGB-D + LiDAR + Nav2** | `nav_rgb` | RTAB-Map RGB-D + LiDAR `/scan` | LiDAR `/scan` + RGB-D cloud | ✅ | ✅ | ✅ | – | open-source RGB-D/LiDAR SLAM navigation stack |
| 4 | **Isaac ROS Visual SLAM + nvblox + Nav2** | `nav_cuvslam_nvblox` | cuVSLAM (RGB-D, IMU optional) | nvblox depth reconstruction/costmap | ✅ | ✅ | – | ◐ | NVIDIA visual-depth navigation stack |
| 5 | **ORB-SLAM3 + LiDAR + Nav2** *(optional/stretch)* | `nav_orbslam3` | ORB-SLAM3 RGB-D | LiDAR | ✅ | ✅ | ✅ | – | optional classical visual SLAM comparison |

(◐ = optional: cuVSLAM uses the IMU only when `enable_imu_fusion:=true`.)

**RTAB-Map uses RGB-D plus LiDAR in `nav_rgb`.** The restored Part 1 RTAB stack fuses
the LiDAR `/scan` with RGB-D: `subscribe_scan` is `True`, `/scan` is remapped to
`/ugv_sim/scan`, and Nav2 keeps the upstream scan obstacle source while the RGB-D
cloud remains available for visualization.

`nav_dpvo` (DPVO monocular VO) is **not** a Part 1 stack and has been **archived**,
inactive, at `nav_systems/nav_dpvo_archived/`. It is preserved (not deleted) for the
later visual-odometry comparison work.

## Layout

```
ugv_controller_v2/
├── README.md                 this file
├── USD_CONTRACT.md           the baked ROS 2 interface + USD fix checklist
├── run_small_warehouse.py    Isaac launcher: rover + small warehouse
├── run_ground_plane.py       Isaac launcher: rover + ground plane (smoke test)
├── common/
│   ├── run_baked_rover.py    core runner: open jetson_rover_v2.usd, compose env, play
│   └── viewport_setup.py     on-launch camera viewports (windowed, cosmetic)
├── keyboard/
│   ├── keyboard_drive.sh     ROS 2 keyboard teleop → /cmd_vel (upstream ugv_tools node)
│   └── README.md
├── nav_experiment/
│   ├── run_random_small_warehouse.py       small warehouse + Replicator obstacles only
│   ├── run_small_warehouse_random_nav.py   rover + small warehouse + Replicator obstacles
│   ├── random_small_warehouse_obstacles.py NVIDIA Replicator static-obstacle generator
│   └── publish_nav_goal.py                 shell helper for /goal_pose
└── nav_systems/
    ├── nav_lidar/                      [Part 1 #1] LiDAR-only Nav2
    │   └── isaac_nav_lidar.launch.py   ROS 2: upstream ugv_nav use_slam (slam_toolbox)
    ├── nav_depth/                      [Part 1 #2] LiDAR + raw depth Nav2
    │   ├── isaac_nav_depth.launch.py   ROS 2: lidar slam_toolbox map + depth XYZ cloud
    │   │                               (depth_image_proc, NO RGB) fed to costmaps (3D RViz)
    │   └── isaac_depth_nav.rviz
    ├── nav_rgb/                        [Part 1 #3] RTAB-Map RGB-D + LiDAR + Nav2
    │   ├── isaac_nav_rgb.launch.py     ROS 2: RTAB-Map RGB-D (subscribe_scan True) + ugv_nav
    │   │                               navigation; /scan remapped to /ugv_sim/scan (3D RViz)
    │   └── isaac_rgb_nav.rviz
    ├── nav_cuvslam_nvblox/             [Part 1 #4] Isaac ROS Visual SLAM + nvblox + Nav2
    │   ├── isaac_nav_cuvslam_nvblox.launch.py  ROS 2: cuVSLAM (RGB-D, IMU opt) map->odom +
    │   │                               nvblox ESDF slice → Nav2 nvblox_layer costmap (no LiDAR)
    │   ├── isaac_cuvslam_nvblox_nav.rviz
    │   └── README.md                   build (source: ~/projects/slam_source) + run notes
    ├── nav_orbslam3/                   [Part 1 #5, optional/stretch] ORB-SLAM3 + LiDAR + Nav2
    │   ├── isaac_nav_orbslam3.launch.py  ROS 2: ORB-SLAM3 RGB-D visual SLAM -> map->odom
    │   │                                 + ugv_nav navigation (3D RViz + Pangolin)
    │   ├── isaac_orbslam3_nav.rviz
    │   ├── build.sh                      ONE-TIME: libORB_SLAM3.so + the wrapper overlay
    │   ├── config/isaac_rgbd.yaml        ORB-SLAM3 settings OVERRIDE (default: auto from
    │   │                                 camera_info)
    │   └── orbslam3_ros2_wrapper/        C++ ROS 2 node linking projects/slam_source/ORB_SLAM3
    └── nav_dpvo_archived/              ARCHIVED/INACTIVE: DPVO monocular VO (not Part 1)
        ├── isaac_nav_dpvo.launch.py      ROS 2: DPVO monocular VO -> map->odom
        │                                 + ugv_nav navigation (COLCON_IGNORE)
        ├── isaac_dpvo_nav.rviz
        ├── dpvo_bridge_node.py           DPVO -> map->odom + /dpvo/map_points (runs in the
        │                                 `dpvo` conda env: torch/CUDA + system rclpy)
        ├── setup.sh                      environment CHECK (does not build)
        └── README.md
```

`common/visual_map_builder.py` is shared by the two VISUAL systems (`nav_orbslam3`
and the archived `nav_dpvo_archived`): both emit a sparse point cloud rather than an
occupancy grid, so it projects their map points into the `/map` Nav2's global static
layer needs.

There are **no per-system Isaac run scripts** anymore: the Isaac side is
nav-agnostic (it just plays the baked USD), so `run_small_warehouse.py` serves
every nav system. A nav system is otherwise purely a ROS 2 launch file under
`nav_systems/`. The two VISUAL systems are the exceptions, and only because their
localizers are not apt packages like slam_toolbox/RTAB-Map: `nav_orbslam3` is C++ and
compiles a wrapper node into a local colcon overlay, and `nav_dpvo_archived` is
torch+CUDA and runs its bridge in the `dpvo` conda env. The launch files still carry
all the wiring.

## Prerequisites

1. **Apply the USD fixes** in `USD_CONTRACT.md` so the baked topics/frames match
   upstream (`/scan`, plain `/tf`, `base_lidar_link`, …). *You are handling this.*
2. Build + source the nav workspace:
   ```bash
   cd /home/joshroq/projects/isaac-assets/ugv_ws
   colcon build --packages-select ugv_nav ugv_slam --symlink-install
   source install/setup.bash
   ```
3. apt: `ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox`
   (lidar); `ros-humble-rtabmap-slam ros-humble-depth-image-proc` (RGB-D);
   teb also needs `ros-humble-teb-local-planner`.

## Run — keyboard driving

```bash
# Terminal 1 — Isaac (isaacpy scrubs system ROS so the bundled bridge loads)
isaacpy ugv_controller_v2/run_small_warehouse.py
#   ... run_small_warehouse.py --headless           # no window
#   ... run_small_warehouse.py --no-camera-topics   # lidar-only performance/debug
#   ... run_ground_plane.py                          # bare ground-plane smoke test

# Terminal 2 — keyboard teleop (keep focused while driving)
ugv_controller_v2/keyboard/keyboard_drive.sh
```

## Run — lidar navigation (slam_toolbox)

```bash
# Terminal 1 — Isaac
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — Nav2 + slam_toolbox (upstream ugv_nav, use_slam)
source /opt/ros/humble/setup.bash
source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash
source ~/.local/bin/ros2ddsenv
ros2 launch ugv_controller_v2/nav_systems/nav_lidar/isaac_nav_lidar.launch.py
#   ... use_localplan:=rpp   (or dwa, mppi, teb)
#   ... use_robot_state_publisher:=true   (fallback while iterating on the USD)
```

Then in RViz: drive `/cmd_vel` manually first (keyboard) to build the slam_toolbox
map, then send a **2D Nav Goal** → Nav2 plans → `/cmd_vel` → Isaac drives.

## Run — RTAB-Map RGB-D + LiDAR + Nav2 — Part 1 #3

Same Isaac launcher; the ROS 2 side runs RTAB-Map with **RGB-D + LiDAR**
(`subscribe_scan` True, `/scan` remapped to `/ugv_sim/scan`) + `ugv_nav`
navigation, with the **3D RViz** view. Nav2 keeps the upstream scan obstacle source.

```bash
# Terminal 1 — Isaac
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — RTAB-Map RGB-D + LiDAR + Nav2 (3D RViz)
source /opt/ros/humble/setup.bash
source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash
source ~/.local/bin/ros2ddsenv
ros2 launch ugv_controller_v2/nav_systems/nav_rgb/isaac_nav_rgb.launch.py
#   ... localization:=true        # localize vs a saved ~/.ros/rtabmap.db
#   ... use_localplan:=teb        # planner: teb|dwa|rpp|mppi
```

Uses the `/oak/*` camera topics and `/scan`, which Isaac publishes by default. Drive
manually to build the map, then send a **2D Nav Goal**.

## Run — Isaac ROS Visual SLAM (cuVSLAM) + nvblox + Nav2 — Part 1 #4

The main NVIDIA visual-depth stack. cuVSLAM localizes from the RGB-D camera (IMU
optional) and broadcasts `map->odom`; nvblox reconstructs depth into an ESDF slice
that feeds the Nav2 `nvblox_layer` costmap. **No LiDAR.** The cuVSLAM/nvblox nodes and
the nvblox costmap plugin must be installed once from NVIDIA Debian packages or built
once from `~/projects/slam_source` — see `nav_systems/nav_cuvslam_nvblox/README.md`.

```bash
# One-time setup, if needed:
ugv_controller_v2/nav_systems/nav_cuvslam_nvblox/build.sh --debs
# or:
ugv_controller_v2/nav_systems/nav_cuvslam_nvblox/build.sh --source

# Terminal 1 — Isaac
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — cuVSLAM + nvblox + Nav2
navlaunch -cuvslam
#   ... enable_imu_fusion:=true     # cuVSLAM VIO (uses /ugv_sim/imu/data)
#   ... use_localplan:=rpp          # planner: teb|dwa|rpp|mppi
```

Drive manually until cuVSLAM is tracking (it only broadcasts `map->odom` while
tracking), then send a **2D Nav Goal**.

## Run — DEPTH + LIDAR navigation (no RGB)

Appearance-independent sibling of nav_rgb: the **lidar** builds the map
(slam_toolbox, same as nav_lidar), while the **depth** image is turned into an XYZ
point cloud (`depth_image_proc/point_cloud_xyz`, no RGB) that is both plotted in
the 3D RViz view and fed to the Nav2 costmaps + collision monitor as an obstacle
source — so avoidance fuses lidar + depth without ever using the RGB image.

```bash
# Terminal 1 — Isaac
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — depth+lidar Nav2 (3D RViz, depth cloud shaded by height)
source /opt/ros/humble/setup.bash
source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash
source ~/.local/bin/ros2ddsenv
ros2 launch ugv_controller_v2/nav_systems/nav_depth/isaac_nav_depth.launch.py
#   ... use_localplan:=rpp          # planner: teb|dwa|rpp|mppi
#   ... depth_max_range:=3.0        # depth obstacle marking range (m)
#   ... depth_min_height:=0.10      # drop floor points below this height (m)
```

The depth cloud publishes on `/ugv_sim/oak/depth/points` (frame
`Camera_Pseudo_Depth`). Drive manually to build the slam_toolbox map, then send a
**2D Nav Goal**.

## Run — ORB-SLAM3 RGB-D visual SLAM

The feature-based visual-SLAM system. **ORB-SLAM3 itself is the localizer**: the
`orbslam3_ros2_wrapper` node links `libORB_SLAM3.so` straight out of
`projects/slam_source/ORB_SLAM3` and calls `ORB_SLAM3::System::TrackRGBD` (the same
entry point as the upstream `Examples_old/ROS/ORB_SLAM3/src/ros_rgbd.cc`), then
publishes the resulting camera pose as **map->odom** — the exact role slam_toolbox
plays in nav_lidar and RTAB-Map plays in nav_rgb. It is *purely* visual: the lidar
feeds only the Nav2 costmap obstacle layer, never the localization.

ORB-SLAM3 emits a **sparse ORB map-point cloud**, not an occupancy grid, so
`visual_map_builder.py` ray-traces those points into the `/map` Nav2's global static
layer needs. Everything else (Nav2 params, costmaps) is the shared upstream stack.

```bash
# ONE-TIME: build libORB_SLAM3.so + the ROS 2 wrapper overlay
ugv_controller_v2/nav_systems/nav_orbslam3/build.sh

# Terminal 1 — Isaac
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — ORB-SLAM3 + Nav2 (3D RViz, sparse ORB map points)
navlaunch -v2 -orbslam3
#   ... use_localplan:=teb        # planner: teb|dwa|rpp|mppi
#   ... orbslam_viewer:=true      # ORB-SLAM3's own Pangolin viewer (features/keyframes)
#   ... orb_features:=2000        # more features if tracking drops in sparse scenery
#   ... camera_fps:=5             # match `ros2 topic hz /ugv_sim/oak/rgb/image_rect`
```

**Intrinsics are read from the live camera.** The wrapper generates its ORB-SLAM3
settings from `/ugv_sim/oak/rgb/camera_info` at startup, so `fx/fy/cx/cy` always match
the baked RSD455 — the same `camera_info` nav_rgb and nav_depth already depend on. This
matters because the v2 rover references `rsd455.usd` from the Omniverse asset server, so
its intrinsics are not knowable from the repo, and a wrong `fx` is the single most common
way to make ORB-SLAM3 produce garbage. Pass `settings:=config/isaac_rgbd.yaml` to pin
them by hand instead.

ORB-SLAM3 must **initialize before it publishes anything**: drive the rover manually
(keyboard teleop) through textured scenery until the log prints `tracking OK — anchored
map==odom`, then send a **2D Nav Goal**. Until then it logs `not tracking` and holds off
on map->odom, which is intentional — publishing a pose it doesn't trust would corrupt the
map. If tracking never initializes, drive slower, raise `orb_features`, and check the RGB
image is actually lit in RViz.

**Native visualization (on by default).** Alongside RViz, ORB-SLAM3 opens its OWN two
windows from its source: the Pangolin **Map Viewer** (keyframe graph, map points, the
usual ORB-SLAM3 menu) and the OpenCV **Current Frame** view showing the tracked ORB
features live. They run on ORB-SLAM3's own viewer thread. `orbslam_viewer:=false` for
headless runs.

## Run — DPVO monocular visual odometry (ARCHIVED — not Part 1)

> Archived, inactive, at `nav_systems/nav_dpvo_archived/`. Kept for the later
> visual-odometry comparison; not part of the Part 1 SLAM/Nav2 baseline.

The learned/monocular sibling of nav_orbslam3. `dpvo_bridge_node.py` imports the `dpvo`
package straight from `projects/slam_source/DPVO-main`, loads the trained `dpvo.pth`
network, and runs the same `DPVO(cfg, net, ht, wd, viz=…)` / `slam(t, image, intrinsics)`
loop as DPVO's own `demo.py` — then publishes its camera pose as **map->odom**.

Three things make it different from every other system here:

* **Monocular ⇒ scale-ambiguous.** DPVO's translations are only defined up to a global
  scale factor, which Nav2's metric costmaps cannot use. The bridge recovers the scale
  **online** by matching DPVO's travelled distance against Isaac's `/odom`, then applies
  it to both the pose and the patch cloud. (`scale:=<x>` pins it instead;
  `min_scale_travel` is how far you must drive before the estimate is trusted.) This is
  the thing RGB-D gives nav_orbslam3 for free.
* **Odometry, not SLAM.** No loop closure by default — it drifts, and it will not
  re-localize after being lost. Expect worse long-run maps than nav_orbslam3.
* **RGB only.** No depth, no lidar in the localizer.

It runs in the **`dpvo` conda env** (torch 2.7/cu128 + DPVO's compiled CUDA extensions +
DPViewer). That env is python 3.10 — the same minor as ROS Humble — so `rclpy` imports
straight into it and the bridge is one ordinary ROS node; the launch just points the
process at that interpreter. Nothing else on the machine is touched.

```bash
# Check the environment any time (does NOT rebuild anything)
ugv_controller_v2/nav_systems/nav_dpvo_archived/setup.sh

# Terminal 1 — Isaac
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — DPVO + Nav2 (3D RViz + DPVO's own Pangolin viewer)
navlaunch -v2 -dpvo
#   ... viz:=false          # drop DPVO's native window, RViz only
#   ... stride:=2           # process every 2nd frame (demo.py's default)
#   ... scale:=1.0          # pin the metric scale instead of estimating it
#   ... use_localplan:=teb  # planner: teb|dwa|rpp|mppi
```

**Native visualization (on by default).** `viz:=true` makes DPVO open **DPViewer**, its
own Pangolin window — the live image, the tracked patches, and the 3D trajectory + patch
cloud — exactly what `demo.py --viz` shows. It is created *inside* DPVO on the live CUDA
tensors, and runs next to RViz.

Drive manually until the log prints `anchored map==odom` and then `Monocular scale
estimated: …`, and only then send a **2D Nav Goal** — before the scale settles, the map
is the right shape but the wrong size.

### `navlaunch` helper

`~/.local/bin/navlaunch` takes a **`-v2`** modifier combinable with a system tag.
It sources ROS + the ugv_ws overlay + `ros2ddsenv`, checks the Isaac topics, then
launches the selected v2 stack:

```bash
navlaunch -v2 -lidar                    # v2 lidar (slam_toolbox)
navlaunch -v2 -rgb                      # v2 RGB-D + LiDAR RTAB-Map (3D RViz)
navlaunch -v2 -depth                    # v2 depth+lidar (slam_toolbox + depth cloud, no RGB)
navlaunch -v2 -orbslam3                 # v2 ORB-SLAM3 RGB-D visual SLAM (RViz + Pangolin)
navlaunch -v2 -dpvo                     # v2 DPVO monocular VO (RViz + DPViewer)
navlaunch -v2 -lidar use_localplan:=rpp # planner passthrough
navlaunch -v2 -rgb localization:=true   # localize vs saved rtabmap.db
navlaunch -v2 -orbslam3 orbslam_viewer:=false
navlaunch -v2 -dpvo viz:=false
navlaunch -v2                           # -v2 alone defaults to lidar
```

Both visual tags pre-flight the **camera** topic as well as the lidar, since a dead camera
is fatal to a visual localizer rather than cosmetic. `-orbslam3` also sources the wrapper
overlay `build.sh` produces; `-dpvo` checks the `dpvo` conda interpreter exists (its
`setup.sh` verifies the rest). Both fail with a sentence telling you what to run.

(Un-suffixed tags — `-lidar`, `-rgb`, `-depth`, `rpp`, … — still target the **old**
`ugv_controller` launch files. Ported to `-v2`: `-lidar`, `-rgb`, `-depth`, `-orbslam3`,
`-dpvo`.)

`navlaunch -v2` is intentionally sim-scenario agnostic: it only requires a rover
simulation publishing the v2 ROS contract and subscribing to `/cmd_vel`. It does
not read USD geometry directly. Any new obstacle layout must still be observable
through the rover sensors and SLAM/localization before Nav2 can plan around it.

For the **randomized-warehouse** experiment — where you want Isaac, the random
obstacles, the nav stack, and Nav2 bring-up all started together — use
`navrandom` (below), not `navlaunch`. `navrandom -lidar|-depth|-rtab` launches the
whole loop; `navlaunch` remains the ROS-only piece it calls internally.

## Randomized small-warehouse obstacles

**One command for the whole loop.** `navrandom` is the randomized-warehouse front
door: it scatters a warehouse of randomized props (**live, fresh layout every run**
by default — no fixed seed), launches Isaac + the baked rover on it, launches the
reliable nav stack, beats the Nav2 bring-up race, and **auto-drives the rover to
goals across the map on its own** — all in one tmux window. It also tears down any
leftover nav stack first (so a prior DPVO/ORB-SLAM/Nav2 run can't bleed `/map`, TF
and a ghost Nav2 into this one; `--no-clean` to skip). It takes the same `-lidar` /
`-depth` / `-rtab` tags as `navlaunch` (and even tolerates the
`navlaunch -v2 -random -lidar` spelling):

```bash
navrandom -lidar             # auto-drive goals, lidar slam_toolbox (watch Isaac viewport)
navrandom -depth             # lidar map + depth cloud (no RGB)
navrandom -rtab              # RTAB-Map RGB-D + LiDAR
navrandom -lidar --auto 5    # auto-drive 5 goals (default count is 3)
navrandom -lidar --no-auto --rviz   # instead: Nav2 up + RViz, click 2D Goal Pose
navrandom -depth --seed 7 --count 24
```

`navrandom` is **autonomous by default**: it defaults to `--auto` (goals are
picked and sent by `nav_mission.py`, no RViz click needed) and to `--no-rviz`
(RViz's GLX context is the least stable process here and is not part of the control
loop — watch the rover in the Isaac viewport). Use `--no-auto` to just bring Nav2
up and wait, and `--rviz` to launch RViz too.

`navrandom` wraps `nav_experiment/warehouse_lab.sh`; see its `--help`, and
`nav_experiment/README.md`, for the full option list. `navlaunch` itself stays
sim-agnostic (ROS side only, no Isaac), so the whole-loop launcher lives with the
experiment. The manual, per-terminal flow below still works if you want to run the
pieces yourself.

Validate the warehouse/obstacle generation without the rover:

```bash
isaacpy ugv_controller_v2/nav_experiment/run_random_small_warehouse.py
#   ... --headless
#   ... --random-seed 1234 --random-obstacle-count 18
#   ... --obstacle-style props
#   ... --keep-open    # save the USD, then keep Isaac open for inspection
```

Run the normal baked rover ROS setup in `small_warehouse.usd`, with randomized
static obstacles generated by NVIDIA Replicator first, then loaded as a plain
baked USD by the rover process:

```bash
# Terminal 1a — generate the randomized warehouse USD (Replicator runs here only)
isaacpy ugv_controller_v2/nav_experiment/run_random_small_warehouse.py \
  --random-seed 1234 \
  --random-obstacle-count 18

# Terminal 1 — Isaac + baked ROS 2 graphs + randomized static obstacles
isaacpy ugv_controller_v2/nav_experiment/run_small_warehouse_random_nav.py \
  --random-seed 1234 \
  --random-obstacle-count 18
#   ... --headless
#   ... --no-camera-topics
#   ... --no-accessories       # diagnostic: skip Isaac-side camera arm/lights
#   ... --no-viewports         # diagnostic: skip extra visible camera viewports
#   ... --render-every-step    # diagnostic: slower, isolates physics/render timing
#   ... --random-min-spacing 0.45
#   ... --random-x-min -4 --random-x-max 4 --random-y-min -4 --random-y-max 4
#   ... --live-replicator      # debug only: old one-process behavior

# Terminal 2 — any existing v2 nav stack
navlaunch -v2 -lidar
```

From inside `ugv_controller_v2`, the same launcher can be run as:

```bash
isaacpy nav_experiment/run_random_small_warehouse.py --random-seed 1234 --random-obstacle-count 18
isaacpy nav_experiment/run_small_warehouse_random_nav.py --random-seed 1234 --random-obstacle-count 18
```

The default obstacle style is `primitives` because it has no asset-server
dependency. `--obstacle-style props` references NVIDIA warehouse/YCB assets and
requires Isaac's asset root to resolve. Placement uses one global non-overlap
pass across all obstacle types before Replicator authors the static colliders.
The small-warehouse default spacing is `0.45` m; larger spacing may force the
launcher to skip some requested obstacles if they cannot fit in the bounds.
The nav runner loads the generated USD by default so Replicator/SyntheticData
obstacle-generation graphs are not active in the live rover process.

If Isaac logs PhysX velocity-read errors after `[run_baked_rover] Entering
simulation loop`, the random obstacles have already been authored and the launch
may still be usable. Use `--no-accessories` to isolate the windowed accessory
controller path, and `--no-viewports` to remove the extra visible camera
viewports while keeping ROS camera/depth topics enabled. If the errors still
appear with both flags or with `--headless`, try `--render-every-step` to test
whether the 4:1 physics/render stepping is involved. If they still remain, the
likely source is the baked odometry/ROS camera writer/Isaac runtime path rather
than the random-obstacle setup.

Publish a goal from the shell instead of clicking RViz's **2D Nav Goal**:

```bash
source /opt/ros/humble/setup.bash
source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash
source ~/.local/bin/ros2ddsenv
python3 ugv_controller_v2/nav_experiment/publish_nav_goal.py --x 2.0 --y 1.0 --yaw 0.0
```

## Why `isaacpy` (not plain `python.sh`)

The baked ROS 2 bridge dlopens Isaac's **bundled** ROS 2 libs. If a system ROS 2
(Python 3.10) is sourced, it shadows them and the bridge fails. `isaacpy`
(`isaacpy_clean.sh`) scrubs system ROS from the Isaac process while keeping
`ROS_DOMAIN_ID`/`RMW` so DDS still meets the Nav2 terminal. See `isaacpy_clean.sh`.

## Status

- ✅ Core runner + keyboard teleop.
- **Part 1 SLAM/Nav2 stacks** (matched real/sim room; see *Part 1* above):
  - ✅ `nav_lidar` — LiDAR-only Nav2 (geometry baseline)
  - ✅ `nav_depth` — LiDAR + raw depth Nav2 (geometry/depth baseline)
  - ✅ `nav_rgb` — RTAB-Map **RGB-D + LiDAR** + Nav2
  - ✅ `nav_cuvslam_nvblox` — Isaac ROS Visual SLAM (cuVSLAM) + nvblox + Nav2
    (launch wired to `~/projects/slam_source`; requires the Isaac ROS packages to
    be colcon-built + sourced — see `nav_cuvslam_nvblox/README.md`)
  - ◐ `nav_orbslam3` — ORB-SLAM3 + LiDAR + Nav2 (optional/stretch comparison)
- Not Part 1 (archived/inactive, preserved): `nav_dpvo_archived` (DPVO monocular VO —
  kept for the later VO comparison); randomized-object experiments under
  `nav_experiment/` (paused, preserved).
- The systems differ **only in what supplies `map->odom`** and what fills the
  costmap (the controlled variables). Nav2, the shared planner/controller and the
  `/cmd_vel` output are the same upstream `ugv_nav` stack in every one:

  | system | localizer | localization inputs | obstacle/costmap | LiDAR |
  |---|---|---|---|:--:|
  | `nav_lidar` | slam_toolbox | `/scan` | LiDAR `/scan` | ✅ |
  | `nav_depth` | slam_toolbox | `/scan` | LiDAR `/scan` + depth cloud | ✅ |
  | `nav_rgb` | RTAB-Map RGB-D + LiDAR | RGB + depth + `/scan` | LiDAR `/scan` + RGB-D cloud | ✅ |
  | `nav_cuvslam_nvblox` | cuVSLAM (RGB-D, IMU opt.) | RGB + depth (+ IMU) | nvblox ESDF slice | – |
  | `nav_orbslam3` | ORB-SLAM3 RGB-D | RGB + depth | LiDAR `/scan` | ✅ |
  | `nav_dpvo_archived` | DPVO (monocular, learned) | RGB only | LiDAR `/scan` | ✅ |
```
