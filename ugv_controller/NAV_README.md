# ugv_controller — Isaac-Sim navigation testbed

A thesis testbed for comparing **multiple navigation systems** on the same UGV
rover in the same Isaac Sim environments. The workspace is organized so the parts
that are identical across every nav system live in one place, and each nav system
is a self-contained folder holding only what is unique to it.

## Layout

```
ugv_controller/
└── common/                              UNIVERSAL — shared by every nav system
    ├── ugv_sim.py                       core runner (nav-system agnostic; takes an
    │                                    extra_sensor_setup hook + asset_path())
    ├── rover_drive_controller.py        low-level differential drive
    ├── keyboard_override_controller.py  keyboard teleop + camera arm ('P' to grab)
    ├── viewport_setup.py                the three on-launch perspectives
    ├── ros2_bridge_drive_graph.py       /cmd_vel, /clock, /odom+TF, /scan, sensor finders
    ├── nav2_bringup_base.launch.py      shared ROS 2 bringup used by nav_dpvo + nav_orbslam3
    ├── depth_camera_graph.py            publishes the /oak topics (shared by nav_depth + nav_rgb + the visual systems)
    ├── visual_map_builder.py            sparse SLAM cloud → /map OccupancyGrid (shared by nav_dpvo + nav_orbslam3)
    └── nav_systems/                      ONE FOLDER PER NAVIGATION SYSTEM
        ├── nav_lidar/                    2D-lidar-only localization (slam_toolbox)
        │   ├── run_small_warehouse_nav.py            Isaac launcher (+ _headless)
        │   ├── run_small_warehouse_nav_headless.py
        │   └── isaac_nav_lidar.launch.py             ROS 2: upstream ugv_nav use_slam path
        ├── nav_depth/                    lidar + depth localization (RTAB-Map ICP, RGB off)
        │   ├── run_small_warehouse_nav.py            Isaac launcher (+ _headless)
        │   ├── run_small_warehouse_nav_headless.py
        │   └── isaac_nav_depth.launch.py             ROS 2: upstream ugv_nav navigation + depth→cloud + rtabmap ICP
        ├── nav_rgb/                      lidar + RGB localization (RTAB-Map RGB-D)
        │   ├── run_small_warehouse_nav.py            Isaac launcher (+ _headless)
        │   ├── run_small_warehouse_nav_headless.py
        │   └── isaac_nav_rgb.launch.py               ROS 2: upstream ugv_nav navigation + rgbd_sync/rtabmap
        ├── nav_dpvo/                     monocular VISUAL ODOMETRY (DPVO, from slam_source)
        │   ├── run_small_warehouse_nav.py            Isaac launcher (+ _headless)
        │   ├── run_small_warehouse_nav_headless.py
        │   ├── dpvo_bridge_node.py                   DPVO → map→odom TF + map-point cloud
        │   └── isaac_nav_dpvo.launch.py              ROS 2: base + DPVO bridge + map builder
        └── nav_orbslam3/                 RGB-D VISUAL SLAM (ORB-SLAM3, from slam_source)
            ├── run_small_warehouse_nav.py            Isaac launcher (+ _headless)
            ├── run_small_warehouse_nav_headless.py
            ├── config/oak_rgbd.yaml                  ORB-SLAM3 settings (OAK/RSD455 intrinsics)
            ├── build.sh                              one-time build of libORB_SLAM3.so + wrapper
            ├── orbslam3_ros2_wrapper/                C++ ROS 2 node (TrackRGBD → map→odom)
            └── isaac_nav_orbslam3.launch.py          ROS 2: base + ORB-SLAM3 bridge + map builder
```

The shared USD assets (`jetson_rover.usd`, `small_warehouse.usd`,
`full_warehouse.usd`) live at the repo root. `common/ugv_sim.py` finds them by
walking up to the directory that contains `jetson_rover.usd`, and exposes
`asset_path("small_warehouse.usd")` so run scripts never hard-code parent counts.

## The navigation systems

All of them drive the rover identically and share the **same Nav2 params** (so the
costmap and planner are constant); they differ only in **what feeds localization**
(the `map → odom` transform). This is the controlled variable for the thesis:

| System | Folder | Localization | Sensors used |
|---|---|---|---|
| **nav_lidar** | `nav_lidar/` | slam_toolbox | 2D lidar `/scan` only |
| **nav_depth** | `nav_depth/` | RTAB-Map ICP (RGB **off**) | depth image → point cloud **+** `/odom` guess; **no RGB** |
| **nav_rgb** | `nav_rgb/` | RTAB-Map RGB-D | front camera RGB + depth (`/oak`) **+** `/scan`, `/odom` guess |
| **nav_dpvo** | `nav_dpvo/` | DPVO monocular visual odometry | front camera **RGB only** (`/oak/rgb`) + `/odom` for scale |
| **nav_orbslam3** | `nav_orbslam3/` | ORB-SLAM3 RGB-D visual SLAM | front camera RGB + depth (`/oak`) |

Exactly one `map → odom` source runs per launch, so they never conflict.

### The two visual-SLAM systems (from `slam_source`)

`nav_dpvo` and `nav_orbslam3` run the DPVO and ORB-SLAM3 source in
`projects/slam_source/` **directly** as the localizer — the same role slam_toolbox
and RTAB-Map play for the other systems. They are camera-only and are excellent at
producing `map → odom`, but (unlike slam_toolbox / RTAB-Map) they do **not** emit a
Nav2 occupancy grid; their native output is a sparse 3-D map-point cloud. So each
one publishes its cloud in the `map` frame and the shared
`common/visual_map_builder.py` projects it onto the ground plane into the `/map`
OccupancyGrid the Nav2 global static layer needs (ray-tracing free space from the
camera, like RTAB-Map's `Grid/RayTracing`). That projector is shared exactly the way
`depth_camera_graph.py` is shared by the two RTAB-Map camera systems.

- **nav_dpvo** wraps DPVO (a learned, patch-based **monocular** VO) in a Python
  bridge (`dpvo_bridge_node.py`) that feeds `/oak/rgb/image_rect` to `dpvo.DPVO`.
  Monocular VO is **up-to-scale**, so the bridge recovers the metric scale online by
  matching DPVO's travelled distance to Isaac's `/odom`. Needs a DPVO `pip install`
  + CUDA + the trained `dpvo.pth` weights.
- **nav_orbslam3** wraps ORB-SLAM3 (sparse ORB features, local BA, DBoW2 loop
  closure) in a **C++** ROS 2 node (`orbslam3_ros2_wrapper`) calling
  `System::TrackRGBD`. RGB-D makes it metric (no scale ambiguity). ORB-SLAM3 is C++,
  so it needs a one-time `./build.sh` (compiles `libORB_SLAM3.so` + the wrapper).

Contrast between the two: DPVO is monocular/learned/scale-recovered; ORB-SLAM3 is
RGB-D/feature-based/metric — a clean VO-vs-SLAM and mono-vs-depth comparison on top
of the existing lidar and RTAB-Map baselines.

**The key contrast:** `nav_depth` and `nav_rgb` use the *same* rover, camera, and
Nav2 params. The only difference is that `nav_rgb` localizes with RGB visual
features + loop closure while `nav_depth` disables RGB and registers on depth
geometry (ICP). Diffing the two isolates the effect of RGB / visual fidelity on
localization. `nav_lidar` is the appearance-invariant, geometry-only baseline.

### Depth camera = RSD455 retuned to the real OAK-D-Lite

The real rover carries a **Luxonis OAK-D-Lite** front depth camera (URDF
`3d_camera_link`, driven by `depthai_ros_driver`, fed into RTAB-Map). Isaac Sim
has **no native OAK-D-Lite asset**, so the rover USD uses the closest built-in
stereo-depth sensor — the **Intel RealSense D455 (RSD455)** — which is the right
choice. `common/depth_camera_graph.py` (shared by `nav_depth` and `nav_rgb`)
retunes its intrinsics (FOV ~69°×54° instead of the D455's ~87°, depth clipped to
0.2–12 m) and publishes on the **same `/oak` topic names** the hardware uses
(`/oak/rgb/image_rect`, `/oak/rgb/camera_info`, `/oak/stereo/image_raw`), so the
RTAB-Map graph is identical to the real rover's. `nav_rgb` consumes RGB + depth;
`nav_depth` consumes only the depth image (+ camera_info) and ignores RGB.

### Three on-launch perspectives

A windowed launch (any system) opens three viewports: the default free-fly
**Perspective** (orbit the whole scene), an **Arm Camera** locked to the pan/tilt
camera atop the rover arm, and a **Front Depth Camera** locked to the RSD455 (what
the `/oak` topics — and RTAB-Map — see).

## Prerequisites (one time)

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-rtabmap-ros ros-humble-depth-image-proc
```

`ros-humble-slam-toolbox` is for `nav_lidar`; `ros-humble-rtabmap-ros` is for the
camera systems (`nav_depth`, `nav_rgb`); `ros-humble-depth-image-proc` is the
depth→point-cloud converter `nav_depth` uses.

`nav_lidar` deliberately follows the real rover's updated `ugv_ws` source layout:
it includes `ugv_nav/launch/nav_bringup/nav2_bringup.launch.py` with
`use_slam:=True` and selects `ugv_nav/params/<localplan>.yaml`. The upstream
default local planner is `teb`; `dwa`, `rpp`, and `mppi` are also available.
Build and source the nav packages before launching it:

```bash
cd /home/joshroq/projects/isaac-assets/ugv_ws
colcon build --packages-select costmap_converter_msgs costmap_converter \
  teb_msgs teb_local_planner ugv_nav ugv_slam --symlink-install
source install/setup.bash
```

If you do not need TEB, `ugv_nav ugv_slam` is enough for the RPP/DWA/MPPI paths.
The Isaac shim removes disabled keepout/speed costmap filters from a temporary
params file at launch time, so the upstream YAMLs stay unchanged while the
terminal avoids continuous "filter mask was not received" warnings unless those
filter servers are explicitly enabled.

The other shims still read the shared Nav2 base inputs from the source tree by
absolute path.

The two visual-SLAM systems have extra, source-specific prerequisites:

```bash
# nav_dpvo (DPVO) — Python + CUDA package + trained weights:
cd /home/joshroq/projects/slam_source/DPVO-main
pip install .                         # needs torch + CUDA (see DPVO README)
./download_models_and_data.sh         # fetches dpvo.pth
# The python that runs the DPVO bridge must import BOTH rclpy and dpvo. If that is
# not `python3`, pass it at launch: ... isaac_nav_dpvo.launch.py python_exec:=/path/to/python

# nav_orbslam3 (ORB-SLAM3) — C++, one-time build:
sudo apt install libpangolin-dev libopencv-dev libeigen3-dev libboost-all-dev \
                 ros-humble-cv-bridge ros-humble-tf2-eigen
cd ugv_controller/common/nav_systems/nav_orbslam3
./build.sh                            # builds libORB_SLAM3.so + the ROS 2 wrapper
```

## Run

Pick a system, then run its Isaac launcher (Terminal 1) and its ROS 2 launch
(Terminal 2). `cd` into the system folder first so the commands are short.

**nav_lidar (lidar only):**
```bash
# Terminal 1 (Isaac)
cd common/nav_systems/nav_lidar && isaacpy run_small_warehouse_nav.py
# Terminal 2 (ROS 2)
source /opt/ros/humble/setup.bash
source /home/joshroq/projects/isaac-assets/ugv_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch common/nav_systems/nav_lidar/isaac_nav_lidar.launch.py
# planner variants:
#   ... isaac_nav_lidar.launch.py use_localplan:=rpp
#   ... isaac_nav_lidar.launch.py use_localplan:=dwa
#   ... isaac_nav_lidar.launch.py use_localplan:=mppi
```

The helper command wraps the same launch and sources `ugv_ws` automatically:

```bash
navlaunch          # lidar + upstream TEB default
navlaunch rpp
navlaunch dwa
navlaunch mppi
navlaunch teb use_rviz:=false
```

The lidar stack follows upstream topic names (`/scan`, `/odom`, `/tf`,
`/tf_static`, `/cmd_vel`) so it behaves like the real rover. To run the real rover
and Isaac rover at the same time, put them on different `ROS_DOMAIN_ID`s unless
you also remap every hardware topic and update the matching Nav2 params.

**nav_depth (lidar + depth, RGB off):**
```bash
# Terminal 1 (Isaac)
cd common/nav_systems/nav_depth && isaacpy run_small_warehouse_nav.py
# Terminal 2 (ROS 2)
source /opt/ros/humble/setup.bash
source <ugv_ws>/install/setup.bash          # ugv_nav Nav2 params + navigation_launch
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch common/nav_systems/nav_depth/isaac_nav_depth.launch.py
# localize against a saved ~/.ros/rtabmap.db instead of mapping live:
#   ... isaac_nav_depth.launch.py localization:=true
```

**nav_rgb (lidar + RGB):**
```bash
# Terminal 1 (Isaac)
cd common/nav_systems/nav_rgb && isaacpy run_small_warehouse_nav.py
# Terminal 2 (ROS 2)
source /opt/ros/humble/setup.bash
source <ugv_ws>/install/setup.bash          # ugv_nav Nav2 params + navigation_launch
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch common/nav_systems/nav_rgb/isaac_nav_rgb.launch.py
# localize against a saved ~/.ros/rtabmap.db instead of mapping live:
#   ... isaac_nav_rgb.launch.py localization:=true
```

**nav_dpvo (DPVO monocular visual odometry):**
```bash
# Terminal 1 (Isaac)
cd common/nav_systems/nav_dpvo && isaacpy run_small_warehouse_nav.py
# Terminal 2 (ROS 2) — python_exec must have BOTH rclpy and dpvo importable
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch common/nav_systems/nav_dpvo/isaac_nav_dpvo.launch.py
#   ... isaac_nav_dpvo.launch.py python_exec:=/path/to/dpvo-python
```

**nav_orbslam3 (ORB-SLAM3 RGB-D):**
```bash
# One-time build first (see Prerequisites): cd .../nav_orbslam3 && ./build.sh
# Terminal 1 (Isaac)
cd common/nav_systems/nav_orbslam3 && isaacpy run_small_warehouse_nav.py
# Terminal 2 (ROS 2)
source /opt/ros/humble/setup.bash
source common/nav_systems/nav_orbslam3/install/setup.bash   # the built wrapper overlay
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch common/nav_systems/nav_orbslam3/isaac_nav_orbslam3.launch.py
```

Both Isaac launchers accept `--headless` and `--lidar-debug-lines` (or use the
`run_small_warehouse_nav_headless.py` script). In RViz, use **2D Nav Goal** → Nav2
plans → `/cmd_vel` → Isaac drives. Press **`P`** in the Isaac window to grab manual
keyboard control, `\` again to return it to Nav2.

### Experimental full warehouse with randomized obstacles

The random-object work is isolated from the production launchers under
`ugv_controller/experimental/random_warehouse_nav/`. It uses a copied runner
(`ugv_sim_random.py`) so changes there cannot affect
`common/nav_systems/*/run_small_warehouse_nav.py`.

```bash
cd ugv_controller/experimental/random_warehouse_nav
isaacpy run_lidar_full_warehouse_random_nav.py

# Reproduce a specific obstacle layout:
isaacpy run_lidar_full_warehouse_random_nav.py --random-seed 1234

# Adjust density and placement area:
isaacpy run_lidar_full_warehouse_random_nav.py \
  --random-obstacle-count 30 \
  --random-x-min -10 --random-x-max 10 \
  --random-y-min -8 --random-y-max 8
```

Use `run_depth_full_warehouse_random_nav.py` or
`run_rgb_full_warehouse_random_nav.py` in the same experimental folder when
testing those systems. The launcher opens `full_warehouse.usd`, then references
NVIDIA Isaac assets such as pallets, cardboxes, KLT bins, traffic cones, and YCB
boxes under
`/World/RandomNavigationObstacles` before the rover and Nav2 graph start. Each
object also gets a simple hidden static collision proxy for stable navigation
testing. Existing warehouse geometry is checked with world-space bounding boxes
so random objects avoid shelves/walls and each other. With no `--random-seed`,
every launch gets a new layout; the console prints `layout_id=<seed>` so a failed
run can be repeated by passing that value back as `--random-seed`.

## Verify (Terminal 2's environment)

```bash
ros2 topic list                       # /clock /cmd_vel /odom /scan /tf /tf_static (+ /oak/* for rtabmap)
ros2 topic hz /scan                   # ~10 Hz, finite ranges
ros2 run tf2_tools view_frames        # map->odom->base_footprint->base_link->...
# camera systems (nav_depth, nav_rgb):
ros2 topic hz /oak/stereo/image_raw   # depth (used by both)
ros2 topic hz /oak/rgb/image_rect     # RGB (used by nav_rgb; published-but-ignored by nav_depth)
ros2 topic hz /oak/depth/points       # nav_depth only: depth->cloud for ICP
# visual-SLAM systems (nav_dpvo, nav_orbslam3):
ros2 topic hz /dpvo/map_points        # nav_dpvo: DPVO patch cloud (or /orbslam3/map_points)
ros2 topic echo /map --once           # OccupancyGrid built by visual_map_builder
ros2 run tf2_ros tf2_echo map odom    # the map->odom the visual SLAM publishes
```
Drive a manual `/cmd_vel` forward, confirm `/odom` advances, drive enough to build
`/map`, then send a 2D Nav Goal.

## Adding a new navigation system (for the thesis)

1. Create a new folder under `common/nav_systems/<your_system>/`.
2. Add a `run_small_warehouse_nav.py` that imports `run, asset_path` from
   `ugv_sim` (use the same `SYSTEM_DIR / COMMON_DIR = parents[1]` bootstrap as the
   existing systems). If the system needs extra Isaac sensors, pass a hook as
   `run(..., extra_sensor_setup=your_hook)` — the hook is called
   `(robot_prim_path, graph_path)` in the pre-play block. Put the hook module in
   the system folder if it is unique to that system, or in `common/` if more than
   one system shares it (as the camera systems share `common/depth_camera_graph.py`).
3. Add a `isaac_<name>.launch.py`. Prefer the upstream-aligned pattern: pull the
   Nav2 stack straight from the built `ugv_nav` package (via
   `get_package_share_directory`) and layer your own `map → odom` source on top, so
   the Isaac run stays aligned with the real rover's `ugv_ws` navigation source.
   `nav_lidar` includes `ugv_nav/launch/nav_bringup/nav2_bringup.launch.py` with
   `use_slam:=True`; `nav_depth`/`nav_rgb` include `navigation_launch.py` and add the
   real rover's RTAB-Map node (RGB-D or depth-ICP) as the `map → odom` source. Only
   the visual-SLAM shims (`nav_dpvo`, `nav_orbslam3`) still include the build-free
   `common/nav2_bringup_base.launch.py` and reuse its `DEFAULT_PARAMS`.
4. Keep `common/` free of system-*specific* logic — only genuinely shared pieces
   (like the camera graph two systems use) belong there; that is the point of the split.

## Known runtime-iteration points (flagged; may need a tweak on first run)

1. **Lidar prim** — `find_lidar_prim()` checks
   `/World/ugv_rover/base_lidar_link/Lidar`, then scans the subtree. The sensor is
   inherited from `jetson_rover.usd`.
2. **Scan orientation** — if scans look rotated, fix the lidar prim's local frame
   in the USD (do NOT add a TF; robot_state_publisher already gives
   base_link→base_lidar_link).
3. **Depth camera prim** — `find_depth_camera_prim()` prefers an RGB camera under
   the front `3d_camera_link`/RSD455 rig; narrow `DEFAULT_DEPTH_CAMERA_HINTS` in
   `common/ros2_bridge_drive_graph.py` if it picks the wrong one.
4. **Camera helper node split** — Isaac Sim 6.0 uses `ROS2CameraHelper` with
   `inputs:type` `rgb`/`depth` for images, but `/oak/rgb/camera_info` must use
   the dedicated `ROS2CameraInfoHelper`; adjust `add_depth_camera()` if these
   node names change in a future Isaac Sim version.
5. **Camera optical frame** — Isaac publishes depth images in
   `oak_rgb_camera_optical_frame`; the camera launches (`nav_depth`, `nav_rgb`) add
   the static TF `3d_camera_link → oak_rgb_camera_optical_frame` (rpy −π/2, 0,
   −π/2). If the RTAB-Map cloud looks rotated, adjust that transform (not the URDF).
6. **localization:=true** (nav_depth/nav_rgb) needs a pre-built `~/.ros/rtabmap.db`; the
   default mapping mode builds it live. Each camera system builds its own db —
   don't reuse a `nav_rgb` db for `nav_depth` (different registration).
7. **nav_depth ICP tuning** — `isaac_nav_depth.launch.py` registers the depth
   point cloud with ICP (`Reg/Strategy=1`, `Reg/Force3DoF=true`). If localization
   drifts or fails to converge, tune `Icp/VoxelSize`, `Icp/MaxCorrespondenceDistance`,
   and the depth cloud density; the OAK-D-Lite's far range (>~6 m) is noisy.
8. **Visual `/map` from a sparse cloud** — `nav_dpvo`/`nav_orbslam3` don't produce
   an occupancy grid; `common/visual_map_builder.py` projects their map-point cloud
   into `/map`. If the grid is too sparse/noisy for the planner, tune its
   `resolution`, `obstacle_z_min/max` (obstacle height band), `max_ray_range`, and
   `occupied_thresh`/`free_thresh` params. As with every SLAM system here, DRIVE
   manually first to build the map before sending a Nav2 goal.
9. **nav_dpvo monocular scale + pose convention** — DPVO is up-to-scale; the bridge
   estimates the metric scale from `/odom` (`min_scale_travel`, or force a fixed
   `scale:=`). Drive a few metres before trusting localization. If the trajectory
   comes out mirrored/rotated, flip `invert_pose:=true` (or adjust the camera
   optical TF), never the URDF. DPVO also needs CUDA + `dpvo.pth`.
10. **nav_orbslam3 build + settings** — needs `./build.sh` (libORB_SLAM3.so + the
    wrapper) and `source install/setup.bash` before launching. Intrinsics live in
    `config/oak_rgbd.yaml` (matched to `depth_camera_graph.py`); `RGBD.DepthMapFactor`
    is 1.0 because Isaac depth is already in metres. If tracking is lost often, raise
    `ORBextractor.nFeatures` or check the RGB/depth sync.

## Rover scale note

The rover USD is authored at **scale 1.0** on every axis (real-world metres),
matching the URDF, the drive constants (`wheel_radius 0.025`, `separation 0.175`),
the Nav2 `robot_radius 0.1`, and the real-size warehouses. Do not reintroduce a
sub-1.0 scale: it would desync Isaac's geometry from the URDF-derived TF and make
the rover move slower than commanded.
