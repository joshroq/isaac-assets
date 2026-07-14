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
    ├── nav_lidar/
    │   └── isaac_nav_lidar.launch.py   ROS 2: upstream ugv_nav use_slam (slam_toolbox)
    ├── nav_rgb/
    │   └── isaac_nav_rgb.launch.py     ROS 2: ugv_slam rtabmap_rgbd + ugv_nav navigation (3D RViz)
    ├── nav_depth/
    │   ├── isaac_nav_depth.launch.py   ROS 2: lidar slam_toolbox map + depth XYZ cloud
    │   │                               (depth_image_proc, NO RGB) fed to costmaps (3D RViz)
    │   └── isaac_depth_nav.rviz
    └── nav_orbslam3/
        ├── isaac_nav_orbslam3.launch.py  ROS 2: ORB-SLAM3 RGB-D visual SLAM -> map->odom
        │                                 + ugv_nav navigation (3D RViz)
        ├── isaac_orbslam3_nav.rviz
        ├── build.sh                      ONE-TIME: libORB_SLAM3.so + the wrapper overlay
        ├── visual_map_builder.py         sparse ORB map points -> /map (Nav2 static layer)
        ├── config/isaac_rgbd.yaml        ORB-SLAM3 settings OVERRIDE (default: auto from
        │                                 camera_info)
        └── orbslam3_ros2_wrapper/        C++ ROS 2 node linking projects/slam_source/ORB_SLAM3
```

There are **no per-system Isaac run scripts** anymore: the Isaac side is
nav-agnostic (it just plays the baked USD), so `run_small_warehouse.py` serves
every nav system. A nav system is otherwise purely a ROS 2 launch file under
`nav_systems/` — `nav_orbslam3` is the one exception, because ORB-SLAM3 is C++ and
has to be compiled (slam_toolbox and RTAB-Map come from apt), so it also ships the
wrapper node it builds into a local colcon overlay.

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

## Run — RGB-D navigation (RTAB-Map)

Same Isaac launcher; the ROS 2 side runs the real rover's RTAB-Map RGB-D SLAM
(`ugv_slam/rtabmap_rgbd.launch.py` replicated) + `ugv_nav` navigation, with the
**3D RViz** view.

```bash
# Terminal 1 — Isaac
isaacpy ugv_controller_v2/run_small_warehouse.py

# Terminal 2 — RTAB-Map RGB-D + Nav2 (3D RViz)
source /opt/ros/humble/setup.bash
source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash
source ~/.local/bin/ros2ddsenv
ros2 launch ugv_controller_v2/nav_systems/nav_rgb/isaac_nav_rgb.launch.py
#   ... localization:=true        # localize vs a saved ~/.ros/rtabmap.db
#   ... use_localplan:=teb        # planner: teb|dwa|rpp|mppi
```

Uses the `/oak/*` camera topics, which Isaac now publishes by default alongside
lidar/odom/TF. Drive manually to build the map, then send a **2D Nav Goal**.

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

### `navlaunch` helper

`~/.local/bin/navlaunch` takes a **`-v2`** modifier combinable with a system tag.
It sources ROS + the ugv_ws overlay + `ros2ddsenv`, checks the Isaac topics, then
launches the selected v2 stack:

```bash
navlaunch -v2 -lidar                    # v2 lidar (slam_toolbox)
navlaunch -v2 -rgb                      # v2 RGB-D RTAB-Map (3D RViz)
navlaunch -v2 -depth                    # v2 depth+lidar (slam_toolbox + depth cloud, no RGB)
navlaunch -v2 -orbslam3                 # v2 ORB-SLAM3 RGB-D visual SLAM (3D RViz)
navlaunch -v2 -lidar use_localplan:=rpp # planner passthrough
navlaunch -v2 -rgb localization:=true   # localize vs saved rtabmap.db
navlaunch -v2 -orbslam3 orbslam_viewer:=true
navlaunch -v2                           # -v2 alone defaults to lidar
```

For `-orbslam3` it also sources the system's own wrapper overlay (the one `build.sh`
produces) and pre-flights the camera topic, since a dead camera is fatal to a visual
localizer rather than cosmetic.

(Un-suffixed tags — `-lidar`, `-rgb`, `-depth`, `rpp`, … — still target the **old**
`ugv_controller` launch files. Ported to `-v2`: `-lidar`, `-rgb`, `-depth`, `-orbslam3`.)

`navlaunch -v2` is intentionally sim-scenario agnostic: it only requires a rover
simulation publishing the v2 ROS contract and subscribing to `/cmd_vel`. It does
not read USD geometry directly. Any new obstacle layout must still be observable
through the rover sensors and SLAM/localization before Nav2 can plan around it.

## Randomized small-warehouse obstacles

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

## Status / next systems

- ✅ Core runner, keyboard teleop, lidar nav, RGB-D nav, depth+lidar nav,
  ORB-SLAM3 visual SLAM — implemented here.
- ⏭️ `nav_dpvo` (monocular visual odometry) ports the same way. It will also need the
  sparse-cloud → `/map` projector, so lift `nav_orbslam3/visual_map_builder.py` into a
  shared spot at that point (the v1 tree keeps it in `common/` for exactly this reason).
```
