# nav_cuvslam_nvblox — Isaac ROS Visual SLAM (cuVSLAM) + nvblox + Nav2

**The main NVIDIA visual-depth navigation stack for Part 1.**

| aspect | value |
|---|---|
| Localization | cuVSLAM + rf2o lidar odom + wheel odom, fused by a robot_localization EKF (default); `use_localization_fusion:=false` = pure cuVSLAM |
| Obstacle / costmap | nvblox depth reconstruction → ESDF 2D slice → Nav2 `nvblox_layer` |
| RGB used | yes |
| Depth used | yes |
| LiDAR used | **localization only, by default** (rf2o `/scan` odom into the EKF); never a costmap layer |
| IMU used | yes, by default (cuVSLAM VIO; `enable_imu_fusion:=false` to disable) |
| Role | NVIDIA visual-depth navigation stack |

### Localization fusion (why the global costmap now comes up)

By default a **robot_localization EKF owns `map→odom`**, fusing Isaac wheel odom +
**rf2o laser odometry** (`/ugv_sim/scan`) + cuVSLAM's absolute map pose
(`visual_slam/vis/slam_odometry`, covariance-stamped by `cuvslam_pose_relay.py`).
This is ported straight from `nav_orbslam3` (`config/ekf_map_fusion.yaml`). The point:
pure cuVSLAM only publishes `map→odom` once it is *tracking*, and it doesn't track when
the rover is stationary, rotating in place, or while the camera stream is starved — so
the global costmap had **no `map` frame** and you couldn't place a goal. With the EKF,
the wheel + lidar velocities keep `map→odom` alive from t=0 (map frame exists
immediately → global costmap comes up), and cuVSLAM corrects drift when it is tracking.
Prereqs (same as `nav_orbslam3`): `rf2o_laser_odometry` built in `ugv_ws` and
`ros-humble-robot-localization` installed.

```bash
navlaunch -cuvslam                                   # EKF fusion (lidar+wheel+cuVSLAM), default
navlaunch -cuvslam use_localization_fusion:=false    # pure cuVSLAM map->odom (no lidar/EKF)
```

**For repeatable A→B trials:** the EKF gives a *robust* `map→odom`, but repeatability of
a goal coordinate across runs still needs a *consistent* `map` origin. In sim, if the
rover always starts at the same pose, the EKF's `map` frame is defined at that start, so
goals are repeatable relative to it. For hard cross-run/real-world repeatability, pair
this with a saved map + relocalization (cuVSLAM can save/load a map; or use `nav_lidar`'s
`use_localization:=amcl` + saved posegraph as the metric-repeatable reference).

## Pipeline

```
Isaac Sim RGB-D camera (+ IMU)
        │  /ugv_sim/oak/rgb/image_rect + camera_info, /ugv_sim/oak/stereo/image_raw
        ▼
Isaac ROS Visual SLAM / cuVSLAM   (RGBD tracking_mode; VIO if enable_imu_fusion)
        │  broadcasts  map → odom  (TF)   ← the localization output Nav2 consumes
        ▼
nvblox                             (depth + color + camera_info + TF pose)
        │  ESDF 2D slice  /nvblox_node/static_map_slice
        ▼
Nav2  (nvblox_layer costmap plugin + shared ugv_nav planner/controller)
        │
        ▼  /cmd_vel → Isaac
```

cuVSLAM plays the exact role slam_toolbox plays in `nav_lidar` and RTAB-Map plays in
`nav_rgb`: it is the **only** thing that supplies `map->odom`. nvblox replaces the
LiDAR obstacle layer as the **only** costmap obstacle source. The `/ugv_sim/scan`
LiDAR is never subscribed. Everything downstream of the costmap (planner, controller,
BT, behaviors, `/cmd_vel`) is the shared upstream `ugv_nav` Nav2 stack — the local
planner default (`teb`) is the same shared default the other stacks use.

## Setup (one time)

`navlaunch -cuvslam` does **not** build these packages every time. It only sources
ROS overlays and checks that the required packages are visible. The one-time setup is
to install or build:

- `isaac_ros_visual_slam` — cuVSLAM node
- `nvblox_ros` — nvblox node
- `nvblox_nav2` — Nav2 costmap plugin

The easy Humble route is NVIDIA's prebuilt Debian packages. Two one-time steps:

```bash
ugv_controller_v2/nav_systems/nav_cuvslam_nvblox/build.sh --debs           # 1. ROS packages
ugv_controller_v2/nav_systems/nav_cuvslam_nvblox/build.sh --runtime-deps   # 2. CUDA 12 + VPI 3 libs
navlaunch -cuvslam
```

`--debs` configures the NVIDIA Isaac ROS apt repo itself (signing key + source list)
and installs the ROS packages. But those debs **dlopen a CUDA 12 runtime and VPI 3
that they do not declare as dependencies** (they assume NVIDIA's container base image
provides them). On a bare host the nodes otherwise fail to load with
`libcudart.so.12: cannot open shared object file`. `--runtime-deps` installs exactly
the libraries `ldd` reports missing — CUDA 12.6 runtime (`cuda-cudart`, `libcublas`,
`libcusolver`, `libnpp`) from NVIDIA's CUDA apt repo, plus `libnvvpi3` (VPI 3.2) from
the jetson x86_64 repo (`r36.4`). It installs **library packages only** — never a
`cuda-drivers`/`cuda-runtime` metapackage — so your existing NVIDIA driver is left
untouched. Verify with `build.sh --check-runtime`.

> **Blackwell / very new drivers (e.g. RTX 5060 Ti, driver 595):** these release-3.2
> debs were built for CUDA 12.6 (~2024). cuVSLAM/nvblox are precompiled CUDA `.so`s;
> on an `sm_120` (Blackwell) GPU the driver must JIT their PTX to the new arch. If a
> node dies at runtime with `no kernel image is available for execution on the device`,
> the precompiled cubins lack `sm_120` and there is no forward-compatible PTX — in that
> case the debs cannot run on this GPU and the source/container build (against a newer
> CUDA) is the only path. Loading + `map->odom` publishing means it works.

Note: NVIDIA's *current* apt repo (`release-4.5`) is **Jazzy / Ubuntu 24.04 only**.
Humble packages live in the older `release-3` repo (dist `jammy`, component
`release-3.0`). `build.sh` selects that repo line automatically for `ROS_DISTRO=humble`.
This is the recommended path — you do **not** need NVIDIA's Triton dev-container build
for cuVSLAM/nvblox.

`--debs` installs the nvblox **leaf** packages (`nvblox-ros`, `nvblox-nav2`,
`nvblox-rviz-plugin`), **not** the `isaac-ros-nvblox` metapackage. The metapackage
drags in `nvblox-examples-bringup`, which hard-depends on `isaac-ros-triton` →
`gxf-isaac-triton` → `libnvidia-compute-560` + TensorRT 10. That DNN/Triton chain is
irrelevant to cuVSLAM + nvblox mapping and is **unresolvable on newer NVIDIA drivers**:
the pinned `libnvidia-compute-560` conflicts with, e.g., driver 595 on a Blackwell GPU.
cuVSLAM (`isaac-ros-visual-slam`) has no TensorRT dependency at all. The nvblox config
yamls that would come from `nvblox-examples-bringup` are instead read from the
`~/projects/slam_source/isaac_ros_nvblox` source-tree fallback (see below).

If the Debian packages are not available on the machine, use the source path. The
local source lives in `~/projects/slam_source`:

```
~/projects/slam_source/isaac_ros_visual_slam   → isaac_ros_visual_slam (cuVSLAM node)
~/projects/slam_source/isaac_ros_nvblox         → nvblox_ros (nvblox_node),
                                                  nvblox_nav2 (NvbloxCostmapLayer),
                                                  nvblox_examples_bringup (nvblox config yamls)
```

```bash
ugv_controller_v2/nav_systems/nav_cuvslam_nvblox/build.sh --source
```

That creates `~/workspaces/isaac_ros-dev`, checks out the Humble-era Isaac ROS release
branch, and prints the NVIDIA dev-container/colcon commands. The source trees under
`~/projects/slam_source` are currently checked out to `release-3.2` (Humble-era,
v3.2-14), matching this rover stack — so `USE_LOCAL_SOURCE=1 build.sh --source` is
viable if you ever need the source/container path. Prefer `--debs` regardless: source
builds of cuVSLAM/nvblox pull the full NITROS + CUDA/TensorRT dependency stack and are
only convenient inside NVIDIA's dev container.

Before entering NVIDIA's dev container, install Git LFS and pass the workspace root
explicitly:

```bash
sudo apt-get install -y git-lfs docker-buildx
git lfs install
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev
```

After the source build succeeds, keep this in your shell startup or export it before
running:

```bash
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev
navlaunch -cuvslam
```

After setup, you can verify the runtime package visibility with:

```bash
ugv_controller_v2/nav_systems/nav_cuvslam_nvblox/build.sh --check
```

This launch instantiates those nodes/plugin directly (the way the NVIDIA
`nvblox_examples` Isaac Sim bringup does), wired to the ugv rover's ROS contract
instead of Carter's.

This launch reads nvblox's parameter yamls from the installed `nvblox_examples_bringup`
share if present, else from the source tree under `~/projects/slam_source`.

## Run

```bash
# Terminal 1 — Isaac  (headless frees the viewport GPU for the camera renders)
isaacpy ugv_controller_v2/run_small_warehouse.py --headless

# Terminal 2 — cuVSLAM + nvblox + Nav2
navlaunch -cuvslam
#   ... enable_imu_fusion:=true     # cuVSLAM VIO (uses /ugv_sim/imu/data)
#   ... use_localplan:=rpp          # planner: teb|dwa|rpp|mppi
#   ... global_costmap_size:=30.0   # rolling global costmap side length (m)
```

### Camera publish rate — this stack is uniquely GPU-render-bound

This is the **heaviest** stack: it renders FOUR camera render products every frame
(color + pseudo-depth for nvblox, plus the left/right OV9782 stereo pair for cuVSLAM)
while cuVSLAM and nvblox also run on the same GPU. The baked render products leave
`inputs:width/height` unauthored, so Isaac falls back to **1280×720 ×4** — the RTX
depth/color annotators become wall-clock bound and delivery collapses (seen as
`/ugv_sim/oak/stereo/image_raw` at ~0.1 Hz and `image_transport … Synchronized
pairs: 0`). Then nvblox never builds a map (empty costmap), cuVSLAM's `map->odom` is
jittery, and Nav2 has nothing to plan against.

`common/run_baked_rover.py` now authors the render products to **640×480** (the real
OAK-D-Lite resolution — see `ugv_vision/config/params.yaml` and the ORB-SLAM3
`isaac_rgbd.yaml` reference) in-memory before play, ~4× fewer pixels per product. All
SLAM consumers read the live `camera_info`, so intrinsics stay self-consistent.

```bash
isaacpy ugv_controller_v2/run_small_warehouse.py --headless               # 640×480 (default)
isaacpy ugv_controller_v2/run_small_warehouse.py --camera-res 512x384     # even lighter
isaacpy ugv_controller_v2/run_small_warehouse.py --camera-res native      # Isaac's 1280×720
```

Confirm the streams are actually flowing before launching Nav2 (compare stamp deltas,
not `ros2 topic hz` — the sim runs slower than wall-clock):

```bash
ros2 topic hz /ugv_sim/oak/rgb/image_rect /ugv_sim/oak/stereo/image_raw
```

This RViz view is anchored to `odom` (not `map`) so the robot model and nvblox mesh
appear immediately, without waiting for cuVSLAM to publish `map->odom`.

**RViz `box.geom` crash (not a GPU problem):** this machine's
`ros-humble-rviz-rendering` 11.2.27 ships an *incomplete* GLSL150 media set — its
`.../rviz_rendering/ogre_media/materials/scripts/` has only `point_cloud_point` and
`point_cloud_tile` materials; the `box`/`flat_square`/`sphere` geometry-shader
materials are missing. Any point display rendered as **Flat Squares/Squares/Boxes/
Spheres** then throws `Unable to locate geometry program rviz/glsl150/box.geom` and
aborts RViz (the robot model still renders first, which is why it looks load-related
but isn't). This config now uses **Style: Points** for the depth cloud + laser, which
maps to the material that IS present. Proper system fix (restores all styles):
`sudo apt install --reinstall ros-humble-rviz-rendering` (or downgrade to a build
whose GLSL150 media is complete). RViz/cuVSLAM/Nav2 are separate processes — if RViz
ever dies, cuVSLAM + nvblox + Nav2 keep running; you can also launch with
`use_rviz:=false` and start RViz standalone later.

Like the other visual front-ends, cuVSLAM only broadcasts `map->odom` once it is
tracking, and it starts tracking after it sees motion through textured scenery.
Drive the rover manually first (keyboard teleop), then send a **2D Nav Goal**. The
`nav_experiment/nav_mission.py` priming flow (drive-to-init, then goal) also applies.

## Notes / assumptions to verify on first bring-up

- **Depth units:** Isaac's `/ugv_sim/oak/stereo/image_raw` is float32 metres, so
  `depth_scale_factor:=1.0`. If nvblox/cuVSLAM see the scene at 1000× scale, the depth
  is millimetre `16UC1` — pass `depth_scale_factor:=1000.0`.
- **Camera optical frame:** defaults to `Camera_OmniVision_OV9782_Color` (the frame
  the OAK RGB/camera_info carry, per `USD_CONTRACT.md`). Override with
  `camera_optical_frame:=...` if the baked USD frame name changes.
- **No `/map` occupancy grid:** cuVSLAM/nvblox do not publish a `nav_msgs/OccupancyGrid`,
  so the global costmap is a rolling window (`global_costmap_size`) fed only by the
  nvblox layer — there is no map_server/static layer.
