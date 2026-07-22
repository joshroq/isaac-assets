# `jetson_rover_v2.usd` — baked ROS 2 contract + fix checklist

`ugv_controller_v2` drives the rover entirely through the OmniGraphs baked into
`jetson_rover_v2.usd`. This file documents **exactly what those graphs publish
today** (introspected from the crate) and the **fixes needed** so the interface
matches the upstream `ugv_ws` Nav2 contract the launchers expect.

The Python/launch side (`run_baked_rover.py`, `isaac_nav_lidar.launch.py`) is
already written to the **target** contract. Make the USD edits below in Isaac Sim
and the two sides meet.

---

## What is baked in today (as introspected)

### Drive — `/World/ugv_rover/Drive_Graph/SkidSteer_Controller`
**Rebuilt 2026-07-08** (the original GUI-built graph was corrupted and stalled Kit;
see `tools/repair_drive_graph.py`). Now: `OnPlaybackTick` →
`ROS2SubscribeTwist(/cmd_vel)` → `DifferentialController` → two
`IsaacArticulationController` nodes (`robotPath=/World/ugv_rover/base_footprint`)
driving the four corner wheels
(`left_up`, `left_down`, `right_up`, `right_down` `_wheel_link_joint`).
Params baked: `wheel_radius=0.04`, `wheel_distance=0.175`,
`max_linear_speed=0.5`, `max_angular_speed=1.9`. `/cmd_vel` is plain/global (the
shared command bus). The skid-steer math lives entirely in this graph.

### The OmniGraphs under `/ROS2_Graphs` — normalized state (2026-07-08)

**Design:** the sim publishes all its sensor DATA under the `ugv_sim` namespace so
it can run at the same time as the real rover (which uses the plain names); the leaf
names match the ugv_ws source convention, and the v2 nav launch files remap
`/ugv_sim/<name>` → `/<name>` (`SetRemap`) to feed the upstream stack. Shared/global
topics stay plain: `/clock`, `/cmd_vel`, `/tf`, `/tf_static`.

**Empirical Isaac namespacing rule (verified live — corrects the earlier note):** the
`nodeNamespace` is prepended EVEN when `topicName` starts with `/`. i.e.
`effective = ("/"+ns if ns else "") + "/" + topicName.lstrip("/")`. So a leading `/`
is NOT absolute. All data publishers below use `nodeNamespace=ugv_sim` + a RELATIVE,
source-convention `topicName` (`tools/normalize_ros2_topics.py` enforces this).

| Graph / node | ns | topicName | Effective topic | frame | Status |
|---|---|---|---|---|---|
| ROS_Clock/PublishClock | — | `clock` | `/clock` | — | ✅ global |
| ROS_Cmd_Vel/SubscribeTwist | — | *(default)* | `/cmd_vel` | — | ✅ global |
| ROS_TF/PublisherTF | — | `tf` | `/tf` | — | ✅ global |
| ROS_TF_Static/×4 | — | `tf_static` | `/tf_static` | — | ✅ global |
| ROS_Odometry/PublisherOdometry | ugv_sim | `odom` | `/ugv_sim/odom` | — | ✅ |
| ROS_Odometry/TF nodes | — | — | `/tf` (world→odom→base_link) | — | ✅ global |
| ROS_Lidar/LaserScanPublish | ugv_sim | `scan` | `/ugv_sim/scan` | `base_lidar_link` | ✅ |
| ROS_Lidar/PointCloudPublish | ugv_sim | `lidar/points` | `/ugv_sim/lidar/points` | `base_lidar_link` | ✅ (disabled) |
| ROS_JointStates/PublisherJointState | ugv_sim | `joint_states` | `/ugv_sim/joint_states` | — | ✅ |
| ROS_Camera/RGBPublish | ugv_sim | `oak/rgb/image_rect` | `/ugv_sim/oak/rgb/image_rect` | `Camera_OmniVision_OV9782_Color` | ✅ |
| ROS_Camera/CameraInfoPublish | ugv_sim | `oak/rgb/camera_info` | `/ugv_sim/oak/rgb/camera_info` | `Camera_OmniVision_OV9782_Color` | ✅ |
| ROS_Camera/DepthPublish | ugv_sim | `oak/stereo/image_raw` | `/ugv_sim/oak/stereo/image_raw` | `Camera_Pseudo_Depth` | ✅ |
| ROS_Camera/DepthPclPublish | ugv_sim | `oak/stereo/depth_pcl` | `/ugv_sim/oak/stereo/depth_pcl` | `Camera_Pseudo_Depth` | ✅ (disabled) |
| ROS_Camera/LeftRGBPublish | ugv_sim | `oak/left/image_rect` | `/ugv_sim/oak/left/image_rect` | `Camera_OmniVision_OV9782_Left` | ✅ (stereo, for cuVSLAM) |
| ROS_Camera/LeftCameraInfoPublish | ugv_sim | `oak/left/camera_info` | `/ugv_sim/oak/left/camera_info` | `Camera_OmniVision_OV9782_Left` | ✅ (stereo, for cuVSLAM) |
| ROS_Camera/RightRGBPublish | ugv_sim | `oak/right/image_rect` | `/ugv_sim/oak/right/image_rect` | `Camera_OmniVision_OV9782_Right` | ✅ (stereo, for cuVSLAM) |
| ROS_Camera/RightCameraInfoPublish | ugv_sim | `oak/right/camera_info` | `/ugv_sim/oak/right/camera_info` | `Camera_OmniVision_OV9782_Right` | ✅ (stereo, for cuVSLAM) |
| ROS_IMU/PublisherImu | ugv_sim | `imu/data` | `/ugv_sim/imu/data` | `Imu_Sensor` | ✅ |

**Camera publish rate (`inputs:frameSkipCount`, measured 2026-07-14).** The camera helpers
publish on rendered frames: `rate = render_rate / (frameSkipCount + 1)`, and
`run_baked_rover.py` renders at 60 Hz sim time (`RENDERING_DT = 1/60`). The hardware
reference is **30 fps** (`ugv_ws/.../ugv_vision/config/params.yaml` → `framerate: 30.0`;
the OAK-D config sets none) — but that cap is **not the binding constraint here**, the GPU
is. Measured sim-time rates on this box (RTX 5060 Ti, headless RayTracedLighting):

| `frameSkipCount` | RGB | depth | camera_info |
|---|---|---|---|
| **0** (every render) — **current** | 17.1 fps | 11.4 fps | 34.3 fps |
| 1 ("cap at 30") | 5.7 fps | 5.7 fps | 17.1 fps |

Image delivery is bounded by RTX render/annotator throughput, so the cameras never approach
60 fps and a skip count only throttles an already-starved stream (it discards renders that
DID carry a fresh image). Rendering at 30 Hz instead (`RENDERING_DT = 1/30` + skip 0) is
worse still (5.3 fps). So `frameSkipCount` is left at **0** — the closest achievable to
30 fps without exceeding it. `tools/set_camera_fps.py --fps 30` re-applies the cap if the
render pipeline ever gets fast enough to exceed 30 fps. Measure with sim-time stamp deltas,
NOT `ros2 topic hz` (wall time; the sim runs at ~0.35x real time).

Camera optical frames are the TF leaf names Isaac publishes under
`front_camera_link/Realsense/RSD455/…` (`Camera_OmniVision_OV9782_Color` for RGB,
`Camera_Pseudo_Depth` for depth, `Camera_OmniVision_OV9782_{Left,Right}` for the
cuVSLAM stereo pair) — NOT `oak`, which is not a frame in the tree. The Left/Right
stereo publishers were added to the baked `ROS_Camera` graph so Isaac ROS cuVSLAM
(stereo-only in 3.x) has a real stereo pair; cuVSLAM needs `base_link →
Camera_OmniVision_OV9782_{Left,Right}` on TF for the rig extrinsics/baseline.
Camera topics are ON by default; launch Isaac with `--no-camera-topics` only for
lidar-only performance/debug sessions.
TF frames covered (keep): `base_footprint→base_link`, `base_link→{base_lidar_link,
front_camera_link, base_imu_link, wheels, pt_*}`, camera chain under
`front_camera_link/Realsense` (incl. `Imu_Sensor`).

---

## Target contract (what `ugv_ws` Nav2 expects) and how the sim meets it

From `ugv_nav/params/*.yaml` + `slam_toolbox_localization.yaml` the upstream stack
subscribes to the PLAIN source names:

```
subscribes : /cmd_vel
consumes   : /scan   (scan_topic /scan, frame base_lidar_link)
             /odom   (odom_frame_id odom, base base_footprint/base_link)
             /clock  /tf  /tf_static
frames     : map (slam) → odom → base_footprint → base_link → base_lidar_link / wheels
```

The sim publishes the sensor data namespaced (`/ugv_sim/scan`, `/ugv_sim/odom`,
`/ugv_sim/oak/*`) and the **v2 launch files bridge the gap with `SetRemap`**
(`/scan→/ugv_sim/scan`, `/odom→/ugv_sim/odom`; rtabmap RGB/depth point directly at
`/ugv_sim/oak/*`). So the upstream stack runs unchanged and the real rover (plain
names) and sim rover (`ugv_sim` names) can be up at the same time. `/cmd_vel`,
`/clock`, `/tf`, `/tf_static` are shared/global. Costmaps use
`robot_base_frame: base_link`; slam_toolbox/controllers use `base_frame: base_footprint`
— both frames exist with a static transform, so either works.

---

## Fix checklist — all DONE (2026-07-08)

The USD side is applied by `tools/normalize_ros2_topics.py` (attribute-only, runs in
standalone pxr); the launch side lives in `nav_systems/*/*.launch.py`.

1. ✅ `/tf`, `/tf_static` plain (global). `/clock`, `/cmd_vel` plain.
2. ✅ Lidar `/ugv_sim/scan`, frame `base_lidar_link`.
   - Runtime check: if `/scan` looks **rotated** in RViz, fix the lidar prim
     orientation at `/World/ugv_rover/base_lidar_link/Example_Rotary_2D` (never a
     compensating TF).
3. ✅ Odom `/ugv_sim/odom`. Lidar point cloud `/ugv_sim/lidar/points`, frame
   `base_lidar_link` (disabled; unused by nav).
4. ✅ `joint_states` → `/ugv_sim/joint_states` (namespaced). `jointNames` already
   cleared (auto-publishes all DOFs), so no length error.
5. ✅ Camera (for nav_rgb), all under `ugv_sim` with source `oak/*` leaf names:
   - RGB `/ugv_sim/oak/rgb/image_rect`, frame `Camera_OmniVision_OV9782_Color`.
   - camera_info `/ugv_sim/oak/rgb/camera_info`, same frame; `topicNameRight` cleared.
   - depth `/ugv_sim/oak/stereo/image_raw`, frame `Camera_Pseudo_Depth`.
   - depth_pcl `/ugv_sim/oak/stereo/depth_pcl` (disabled).
   - Camera topics are ON by default; use `--no-camera-topics` for lidar-only
     performance/debug sessions.
6. ✅ Launch bridge: `SetRemap` in `isaac_nav_lidar.launch.py` /
   `isaac_nav_rgb.launch.py`; rtabmap remaps → `/ugv_sim/oak/*`.
7. ✅ **RSD455 camera rig free-fall fix (2026-07-09).** The RealSense `rsd455.usd`
   reference defines `.../Realsense/RSD455` as its OWN rigid body
   (`PhysicsRigidBodyAPI`), nested inside `front_camera_link` (also a rigid body,
   fixed-jointed to `base_link`). PhysX lets the un-jointed inner body free-fall,
   so on play the whole camera rig (RGB + pseudo-depth + IMU prims) sank through
   the floor (measured z: −0.87 → −3.7 → −23.6 m over ~300 steps) — the "depth
   camera appears then disappears" symptom. Result: RGB rendered the empty
   background (flat/black) and depth was 100% inf → RGB-D cloud had 0 points, and
   the IMU read ~0 (an accelerometer in free-fall). **Fix:** author
   `physics:rigidBodyEnabled = False` on `.../Realsense/RSD455` so the rig is a
   static child of the properly-jointed `front_camera_link`. After the fix the
   camera holds z≈0.10 m, depth is ~100% finite (0.18–9.3 m), RGB is a real lit
   image, and the IMU reads |a|≈9.82. This is the v2 equivalent of the v1 depth
   despawn fix. NOTE: no scene light needs adding — the small_warehouse's own
   lights illuminate the (now correctly-placed) camera; a dome light was tried
   and rejected (it lowered contrast and added a fake gray sky).
8. ✅ **IMU publisher (2026-07-09).** New `/ROS2_Graphs/ROS_IMU` graph
   (OnPlaybackTick → IsaacReadIMU(`.../RSD455/Imu_Sensor`) → ROS2PublishImu),
   namespaced `ugv_sim`, topic `imu/data` → `/ugv_sim/imu/data`, frame
   `Imu_Sensor`. `ReadIMU.inputs:imuPrim` MUST be a real relationship target
   (a raw-string `.set()` authors an EMPTY target → "Invalid Imu sensor prim").
   Needs the `isaacsim.sensors.physics` + `isaacsim.sensors.physics.nodes`
   extensions (added to the runner). `Imu_Sensor.visualize` set False (kills the
   moving axis gizmo that showed in the viewport).
9. ✅ **base_imu_link TF (2026-07-09).** Added `.../base_link/base_imu_link` to
   `ROS_TF_Static/ComputeBasePublisherTF.inputs:targetPrims`, so
   `base_link→base_imu_link` is on `/tf_static`. The nav launch files
   (`nav_lidar`, `nav_rgb`) NO LONGER strip `base_imu_link`/`imu_joint` from the
   RViz URDF (the frame now exists), matching the real rover's URDF.

### Optional / notes
- **Odometry base frame:** `PublisherOdometry.chassisFrameId` / `TFOdom2Robot.childFrameId`
  are `base_link`; upstream `base_frame` is `base_footprint`. Resolves via the static
  `base_footprint→base_link`, so either works. Left as `base_link`.
- **Simultaneous TF:** `/tf` is shared/global, so running the real rover and sim TF
  trees at literally the same time would collide frames. Only the sensor DATA topics
  are namespaced today; full TF/`cmd_vel` namespacing (run the whole sim nav stack
  under `ugv_sim`) is a larger change, not done.

### ⚙️ Sanity checks (already correct — don't change)

- `/cmd_vel` subscribe topic — correct, matches the launchers and keyboard teleop.
- `/clock` — correct, global, never namespaced.
- Drive params (`wheel_radius 0.04`, `wheel_distance 0.175`) — match the URDF and
  the real rover. Leave as authored.

---

## Verify after the fixes (ROS terminal)

```bash
source /opt/ros/humble/setup.bash && source ~/.local/bin/ros2ddsenv
ros2 topic list          # expect: /clock /cmd_vel /tf /tf_static
                         #         /ugv_sim/{odom,scan,joint_states} + /ugv_sim/oak/*
ros2 topic hz /ugv_sim/scan       # ~6 Hz, finite ranges
ros2 topic hz /clock
ros2 run tf2_tools view_frames    # world→odom→base_link→base_lidar_link / wheels / camera leaves
ros2 topic echo /ugv_sim/odom --once   # advances after you drive /cmd_vel forward
```
