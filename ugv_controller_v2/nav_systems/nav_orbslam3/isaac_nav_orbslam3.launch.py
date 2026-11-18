"""Nav system: ORB-SLAM3 (RGB-D) VISUAL SLAM for the BAKED-GRAPH rover (ugv_controller_v2).

PART 1 STACK HEADER  (Part 1 #5 — ORB-SLAM3 + LiDAR + Nav2, OPTIONAL/STRETCH)
    Localization       : ORB-SLAM3 RGB-D + LiDAR (rf2o) fused by a robot_localization EKF
    Obstacle / costmap : LiDAR /scan
    RGB used           : yes
    Depth used         : yes
    LiDAR used         : yes (localization fusion + initial /map build + costmap obstacle)
    IMU used           : no
    Role               : optional classical visual SLAM comparison

The feature-based visual-SLAM sibling of nav_rgb. ORB-SLAM3 runs DIRECTLY from the source
checkout in projects/slam_source/ORB_SLAM3 (this system's orbslam3_ros2_wrapper links its
libORB_SLAM3.so and calls ORB_SLAM3::System::TrackRGBD, the same way the upstream ROS
example Examples_old/ROS/ORB_SLAM3/src/ros_rgbd.cc does).

  * LOCALIZER : ORB-SLAM3 RGB-D, fused with 2D-lidar odometry. The wrapper feeds Isaac's
    camera (/ugv_sim/oak/rgb/image_rect + /ugv_sim/oak/stereo/image_raw) to TrackRGBD,
    reads the camera pose back, and — in the default FUSED mode (use_localization_fusion) —
    publishes it as an ABSOLUTE map-frame pose on /orbslam3/pose. A robot_localization EKF
    fuses that pose with rf2o laser odometry (from /scan) and Isaac's wheel odom to OWN a
    continuous map->odom. This is the fix for pure-visual fragility: ORB-SLAM3 loses
    tracking under in-place rotation (little parallax) and would otherwise FREEZE map->odom,
    making the lidar map appear to spin with the rover and the robot spin in place; the
    lidar/wheel odometry keeps the pose alive and the EKF absorbs the visual correction
    when tracking recovers. Set use_localization_fusion:=false to revert to the original
    behavior where ORB-SLAM3 alone broadcasts map->odom (100% visual, no lidar in loc).
    ORB feature tracking + local bundle adjustment + DBoW2 loop closure; RGB-D makes it
    metric (no monocular scale drift).
  * MAP : ORB-SLAM3 emits a SPARSE map-point cloud, not a Nav2 occupancy grid, so
    visual_map_builder.py projects /orbslam3/map_points into the /map that Nav2's global
    static layer needs (ray-tracing free space from the camera). This is the one piece
    nav_lidar/nav_rgb get for free from their SLAM backends. To keep INITIAL mapping from
    being blocked on visual features, the SAME builder also folds the 2D lidar /scan into
    that grid (use_lidar_map, ON by default), giving a dense metric /map from the first
    scan. use_lidar_map:=false restores the visual-only grid.
  * NAV2 : the planner/controller/bt/behavior servers come from upstream
    `ugv_nav/launch/nav_bringup/navigation_launch.py` — the same source nav_rgb and the
    real rover use. The EKF (or ORB-SLAM3, unfused) supplies map->odom and the map builder
    supplies /map, so no slam_toolbox/amcl runs here (contrast nav_lidar).

The 2D lidar feeds the Nav2 costmap obstacle layer exactly as the upstream params define
it (same as every other v2 system), seeds the initial /map (above), AND — in fused mode —
drives rf2o laser odometry into the localization EKF. So map->odom is now a lidar+visual
fusion; use_localization_fusion:=false is the knob back to a pure-visual map->odom source.

Contrast the siblings:
    nav_lidar    : slam_toolbox on /scan (2D lidar SLAM), 2D RViz.
    nav_rgb      : RTAB-Map RGB-D (RGB + depth + scan), dense colored cloud, 3D RViz.
    nav_depth    : lidar SLAM + depth obstacle cloud, no RGB, 3D RViz.
    nav_orbslam3 : ORB-SLAM3 RGB-D visual SLAM, sparse ORB map points, 3D RViz. THIS.

CONTRACT THIS LAUNCH EXPECTS ISAAC TO PUBLISH (see ugv_controller_v2/USD_CONTRACT.md)
    subscribes : /cmd_vel
    publishes  : /clock, /odom, /scan, /tf + /tf_static, and the OAK camera on
                 /ugv_sim/oak/rgb/image_rect, /ugv_sim/oak/rgb/camera_info,
                 /ugv_sim/oak/stereo/image_raw (RGB frame Camera_OmniVision_OV9782_Color,
                 present in /tf — Isaac owns the TF tree, so unlike the v1 system this
                 launch publishes NO static camera transform).
    The EKF then provides map->odom (fusing ORB-SLAM3 + rf2o laser odom + /odom wheel
    odom), visual_map_builder provides /map, Nav2 outputs /cmd_vel. (Unfused: ORB-SLAM3
    broadcasts map->odom directly and rf2o/EKF do not run.)

INTRINSICS: the wrapper generates its ORB-SLAM3 settings from the live
/ugv_sim/oak/rgb/camera_info (settings:="" , the default), so fx/fy/cx/cy always match
the baked camera — the same camera_info nav_rgb and nav_depth already rely on. Pass
settings:=config/isaac_rgbd.yaml to pin them by hand instead.

PREREQUISITES
  * ONE-TIME BUILD: ./build.sh in this folder (builds libORB_SLAM3.so + the wrapper
    overlay). `navlaunch -v2 -orbslam3` sources the overlay for you.
  * Build + source ugv_ws so `ugv_nav` AND `rf2o_laser_odometry` are on the ament path
    (rf2o is vendored in ugv_ws/src/ugv_else; it drives the fused localization):
        cd <ugv_ws> && colcon build --packages-select ugv_nav rf2o_laser_odometry --symlink-install
        source install/setup.bash
  * apt: ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-cv-bridge
         ros-humble-tf2-eigen ros-humble-robot-localization libpangolin-dev libopencv-dev
         libeigen3-dev libboost-all-dev
    (robot_localization runs the EKF that fuses ORB-SLAM3 + lidar into map->odom; both it
     and rf2o are only needed when use_localization_fusion is on — the default.)

RUN (Isaac running the baked rover elsewhere):
  navlaunch -v2 -orbslam3                       # or, by hand:
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  source <this folder>/install/setup.bash       # the built wrapper overlay
  source ~/.local/bin/ros2ddsenv
  ros2 launch <this file>                              # fused lidar+visual loc, 3D RViz
  ros2 launch <this file> use_localplan:=teb           # planner: teb|dwa|rpp|mppi
  ros2 launch <this file> use_localization_fusion:=false  # pure-visual map->odom (no EKF)
  ros2 launch <this file> orbslam_viewer:=false        # drop ORB-SLAM3's own windows
  ros2 launch <this file> settings:=<orbslam3.yaml>    # pin intrinsics by hand

KNOWN-MAP MODE (thesis Experiment A — build an ORB-SLAM3 atlas once, then localize):
  # 0) ONE-TIME: rebuild the wrapper so load_atlas/save_atlas/localization_mode exist:
  #      ./build.sh
  # 1) mapping run: drive the environment; atlas saved to <base>.osa on exit (Ctrl-C)
  ros2 launch <this file> \
      save_atlas:=<repo>/experiments/maps/warehouse/orbslam3_atlas
  # 2) per-trial localization run: load that atlas + localization-only, no priming drive
  ros2 launch <this file> localization_mode:=true \
      load_atlas:=<repo>/experiments/maps/warehouse/orbslam3_atlas
Default (no atlas args) is unchanged: live ORB-SLAM3 mapping.

VISUALIZATION: two views run at once. RViz shows the Nav2 world (costmaps, plans, the
sparse map points as a cloud), while ORB-SLAM3's OWN viewer (orbslam_viewer, ON by
default) opens its native Pangolin "Map Viewer" (keyframe graph + map points, with the
usual ORB-SLAM3 menu) and the OpenCV "Current Frame" window showing the tracked ORB
features live. They run on ORB-SLAM3's own viewer thread, independent of RViz.
"""

import os
import sys
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

# URDF for the RViz RobotModel. Use the Isaac-matched URDF whose link names equal the
# frames the baked USD publishes on /tf, so RViz renders the model at live poses.
UGV_WS_SRC = "/home/joshroq/projects/isaac-assets/ugv_ws/src/ugv_main"
UGV_DESCRIPTION_SRC = os.path.join(UGV_WS_SRC, "ugv_description")
URDF_PATH = os.path.join(UGV_DESCRIPTION_SRC, "urdf", "ugv_rover_isaac_fixed.urdf")

_LOCALPLAN_CHOICES = ("teb", "dwa", "rpp", "mppi")

THIS_DIR = Path(__file__).parent
# Shared with nav_dpvo: both visual systems emit a sparse cloud, not an occupancy grid.
MAP_BUILDER = str(THIS_DIR.parent.parent / "common" / "visual_map_builder.py")
DEFAULT_RVIZ_CONFIG = str(THIS_DIR / "isaac_orbslam3_nav.rviz")

# The sim publishes sensor DATA under the `ugv_sim` namespace (so the sim and the real
# rover can run at once); ORB-SLAM3 is pointed straight at those names.
RGB_TOPIC = "/ugv_sim/oak/rgb/image_rect"
DEPTH_TOPIC = "/ugv_sim/oak/stereo/image_raw"
CAMERA_INFO_TOPIC = "/ugv_sim/oak/rgb/camera_info"
# ORB-SLAM3's sparse map points (frame=map) -> visual_map_builder -> /map.
MAP_POINTS_TOPIC = "/orbslam3/map_points"
# 2D lidar the map builder ALSO folds into /map for a dense initial map. Sim data is
# namespaced. In fused mode this same scan drives rf2o laser odometry (below).
SCAN_TOPIC = "/ugv_sim/scan"
DEFAULT_LASER_FRAME = "base_lidar_link"

# ── Co-equal lidar+visual localization fusion (use_localization_fusion) ──────────
# ORB-SLAM3's absolute map-frame pose, rf2o's laser odometry, and Isaac's wheel odom are
# fused by a robot_localization EKF that OWNS map->odom, so localization keeps updating
# when visual tracking drops (e.g. in-place rotation). Isaac's wheel odom is /ugv_sim/odom.
ORBSLAM_POSE_TOPIC = "/orbslam3/pose"
LIDAR_ODOM_TOPIC = "/orbslam3/lidar_odom"
WHEEL_ODOM_TOPIC = "/ugv_sim/odom"
DEFAULT_EKF_CONFIG = str(THIS_DIR / "config" / "ekf_map_fusion.yaml")
DEFAULT_BASE_FRAME = "base_footprint"

DEFAULT_VOCAB = "/home/joshroq/projects/slam_source/ORB_SLAM3/Vocabulary/ORBvoc.txt"
# The baked USD's RGB optical frame (USD_CONTRACT.md). It is on /tf, so no static
# transform publisher is needed here — Isaac owns the tree.
DEFAULT_CAMERA_FRAME = "Camera_OmniVision_OV9782_Color"


def _load_robot_description() -> str:
    if not os.path.isfile(URDF_PATH):
        raise RuntimeError(f"Isaac rover URDF not found: {URDF_PATH}")
    with open(URDF_PATH, "r") as f:
        robot_description = f.read()
    desc_root = "file://" + UGV_DESCRIPTION_SRC.rstrip("/") + "/"
    # Resolve mesh URIs to file://; map the front camera's mesh to its shipped name
    # (3d_camera_link.stl) so front_camera_link renders (see nav_lidar for detail).
    robot_description = (
        robot_description
        .replace("../meshes/", desc_root + "meshes/")
        .replace("package://ugv_description/", desc_root)
        .replace("front_camera_link.stl", "3d_camera_link.stl")
    )
    # base_imu_link is kept: the baked USD publishes base_link->base_imu_link on
    # /tf_static, so the RViz RobotModel resolves the frame (see nav_lidar/nav_rgb).
    return robot_description


def _require_file(path: str, label: str) -> str:
    if not os.path.isfile(path):
        raise RuntimeError(f"{label} not found: {path}")
    return path


def _launch_value(context, name: str, default: str) -> str:
    value = context.launch_configurations.get(name, default)
    return default if value == "" else value


def _launch_bool(context, name: str, default: bool = False) -> bool:
    value = str(context.launch_configurations.get(name, str(default))).strip().lower()
    return value in ("1", "true", "yes", "on")


def _drop_filter(params: dict, filter_name: str) -> None:
    filters = params.get("filters")
    if isinstance(filters, list) and filter_name in filters:
        params["filters"] = [item for item in filters if item != filter_name]
        if not params["filters"]:
            params.pop("filters", None)
    params.pop(filter_name, None)


def _write_params_without_filters(source_file: str) -> str:
    """Strip keepout/speed costmap filters (their info servers don't run under
    navigation_launch), avoiding continuous "filter mask was not received"
    warnings without touching the selected planner/controller/behavior params."""
    with open(source_file, "r") as f:
        data = yaml.safe_load(f)
    for outer, inner in (("local_costmap", "local_costmap"),
                         ("global_costmap", "global_costmap")):
        params = data.get(outer, {}).get(inner, {}).get("ros__parameters", {})
        _drop_filter(params, "keepout_filter")
        _drop_filter(params, "speed_filter")
    for server in ("keepout_filter_mask_server", "keepout_costmap_filter_info_server",
                   "speed_filter_mask_server", "speed_costmap_filter_info_server"):
        data.pop(server, None)
    fd, path = tempfile.mkstemp(prefix="isaac_v2_nav_orbslam3_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _launch_setup(context, *args, **kwargs):
    ugv_nav_dir = get_package_share_directory("ugv_nav")
    navigation_launch = _require_file(
        os.path.join(ugv_nav_dir, "launch", "nav_bringup", "navigation_launch.py"),
        "ugv_nav navigation launch",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")
    vocabulary = LaunchConfiguration("vocabulary")
    settings = LaunchConfiguration("settings")
    orbslam_viewer = LaunchConfiguration("orbslam_viewer")
    camera_optical_frame = LaunchConfiguration("camera_optical_frame")

    localplan = context.launch_configurations.get("use_localplan", "rpp")
    if localplan not in _LOCALPLAN_CHOICES:
        raise RuntimeError(
            f"use_localplan must be one of {_LOCALPLAN_CHOICES}, got '{localplan}'"
        )
    params_file = _require_file(
        os.path.join(ugv_nav_dir, "params", f"{localplan}.yaml"),
        f"ugv_nav {localplan} params",
    )
    # v2-specific 3D RViz view: built-in RViz displays only, pointed at the sim's
    # namespaced topics, plus the ORB-SLAM3 sparse map-point cloud.
    rviz_config = _launch_value(context, "rviz_config", DEFAULT_RVIZ_CONFIG)
    _require_file(rviz_config, "nav_orbslam3 RViz config")
    _require_file(MAP_BUILDER, "visual_map_builder")

    vocab_path = _launch_value(context, "vocabulary", DEFAULT_VOCAB)
    _require_file(vocab_path, "ORB-SLAM3 vocabulary (run ./build.sh to uncompress it)")
    # settings:="" is the normal path: the wrapper generates them from camera_info.
    settings_path = context.launch_configurations.get("settings", "")
    if settings_path:
        _require_file(settings_path, "ORB-SLAM3 settings yaml")

    # Resolve the numeric args to real Python scalars rather than handing launch a
    # substitution: launch would YAML-evaluate the string, so `depth_max:=10` would
    # arrive as an int where the node declared a double and rclcpp would throw. Casting
    # here makes every spelling of a number work (same reason nav_depth casts queue_size).
    camera_fps = int(float(_launch_value(context, "camera_fps", "10")))
    sync_queue = int(float(_launch_value(context, "sync_queue", "30")))
    orb_features = int(float(_launch_value(context, "orb_features", "1200")))
    depth_min = float(_launch_value(context, "depth_min", "0.1"))
    depth_max = float(_launch_value(context, "depth_max", "10.0"))
    map_resolution = float(_launch_value(context, "map_resolution", "0.05"))
    map_z_min = float(_launch_value(context, "map_obstacle_z_min", "0.08"))
    map_z_max = float(_launch_value(context, "map_obstacle_z_max", "1.5"))
    map_max_ray = float(_launch_value(context, "map_max_ray_range", "8.0"))
    sensor_frame = _launch_value(context, "camera_optical_frame", DEFAULT_CAMERA_FRAME)
    # Lidar fusion into the initial /map. use_lidar_map:=false falls back to the
    # visual-only sparse grid (scan_topic left empty -> the builder never subscribes).
    use_lidar_map = _launch_bool(context, "use_lidar_map", True)
    scan_topic = SCAN_TOPIC if use_lidar_map else ""
    laser_frame = _launch_value(context, "laser_frame", DEFAULT_LASER_FRAME)
    map_scan_max_range = float(_launch_value(context, "map_scan_max_range", "12.0"))

    # Co-equal lidar+visual localization fusion (default ON). When ON, a robot_localization
    # EKF owns map->odom (fusing ORB-SLAM3's absolute pose + rf2o laser odom + wheel odom),
    # so the ORB-SLAM3 bridge runs with publish_tf=false/publish_pose=true. When OFF, the
    # bridge broadcasts map->odom itself (pure-visual, the original behavior).
    use_fusion = _launch_bool(context, "use_localization_fusion", True)
    base_frame = _launch_value(context, "base_frame", DEFAULT_BASE_FRAME)
    ekf_config = _launch_value(context, "ekf_config", DEFAULT_EKF_CONFIG)
    if use_fusion:
        _require_file(ekf_config, "robot_localization EKF config")

    params_file = _write_params_without_filters(params_file)

    actions = []

    # robot_state_publisher runs ONLY to publish /robot_description for the RViz
    # RobotModel display; its /tf + /tf_static are remapped to dead topics so it never
    # competes with the baked USD's TF (Isaac owns the tree).
    actions.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time,
                     "robot_description": _load_robot_description()}],
        remappings=[("/tf", "/rsp_tf_unused"),
                    ("/tf_static", "/rsp_tf_static_unused")],
        output="screen",
    ))

    # ── ORB-SLAM3 RGB-D bridge — the VISUAL localizer of this system ─────────────
    # Camera RGB+depth -> TrackRGBD -> /orbslam3/map_points, plus either the map->odom TF
    # (unfused) or the absolute /orbslam3/pose the EKF fuses (fused, the default).
    # Built once by ./build.sh; needs `source install/setup.bash` (navlaunch does it).
    # It reads only the camera + /tf, so it needs no /scan or /odom remap and stays
    # outside the SetRemap group below.
    actions.append(Node(
        package="orbslam3_ros2_wrapper", executable="rgbd_odom_node",
        name="orbslam3_rgbd_bridge",
        parameters=[{
            "use_sim_time": use_sim_time,
            "vocabulary": vocabulary,
            "settings": settings,          # "" -> generate from camera_info
            "use_viewer": orbslam_viewer,
            "rgb_topic": RGB_TOPIC,
            "depth_topic": DEPTH_TOPIC,
            "camera_info_topic": CAMERA_INFO_TOPIC,
            "cloud_topic": MAP_POINTS_TOPIC,
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": base_frame,
            "camera_optical_frame": camera_optical_frame,
            "camera_fps": camera_fps,
            "sync_queue": sync_queue,
            "depth_min": depth_min,
            "depth_max": depth_max,
            "orb_features": orb_features,
            # Fused mode: the EKF owns map->odom, so the bridge stops broadcasting it and
            # instead publishes its absolute pose for the EKF to fuse. Non-fused: the
            # bridge broadcasts map->odom itself (the original pure-visual behavior).
            "publish_tf": not use_fusion,
            "publish_pose": use_fusion,
            "pose_topic": ORBSLAM_POSE_TOPIC,
            # Atlas persistence (Experiment A). Empty/false = live mapping (unchanged).
            "load_atlas": LaunchConfiguration("load_atlas"),
            "save_atlas": LaunchConfiguration("save_atlas"),
            "localization_mode": LaunchConfiguration("localization_mode"),
        }],
        respawn=use_respawn, respawn_delay=2.0,
        output="screen",
    ))

    # ── Sparse map points + 2D lidar -> /map (the Nav2 global static layer) ───────
    # ORB-SLAM3 has no occupancy grid of its own; this projects its cloud into one and
    # ALSO folds in the 2D lidar /scan (use_lidar_map, default on) so the initial map is
    # dense and metric from the first scan instead of waiting on visual features. Passing
    # scan_topic:="" (use_lidar_map:=false) reverts to the visual-only sparse grid. This
    # is the MAP-geometry use of the lidar; its LOCALIZATION use is the rf2o+EKF path below.
    map_builder_cmd = [
        sys.executable, MAP_BUILDER, "--ros-args",
        "-p", ["use_sim_time:=", use_sim_time],
        "-p", f"sensor_frame:={sensor_frame}",
        "-p", f"resolution:={map_resolution}",
        "-p", f"obstacle_z_min:={map_z_min}",
        "-p", f"obstacle_z_max:={map_z_max}",
        "-p", f"max_ray_range:={map_max_ray}",
        "-p", f"scan_topic:={scan_topic}",
        "-p", f"laser_frame:={laser_frame}",
        "-p", f"scan_max_range:={map_scan_max_range}",
        "-r", f"cloud:={MAP_POINTS_TOPIC}",
    ]
    actions.append(ExecuteProcess(cmd=map_builder_cmd, output="screen"))

    # ── Co-equal lidar+visual localization fusion ────────────────────────────────
    # rf2o turns /scan into laser odometry (rotation-robust, where ORB-SLAM3 loses
    # tracking); the EKF fuses it with ORB-SLAM3's absolute pose and Isaac's wheel odom to
    # own a continuous map->odom. rf2o is vendored in ugv_ws (ugv_else/rf2o_laser_odometry)
    # and robot_localization comes from apt (see PREREQUISITES). Both stay off when
    # use_localization_fusion:=false (then the ORB-SLAM3 bridge broadcasts map->odom).
    if use_fusion:
        actions.append(Node(
            package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry", output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "laser_scan_topic": SCAN_TOPIC,
                "odom_topic": LIDAR_ODOM_TOPIC,
                # rf2o ALWAYS creates a publisher on imu_topic, so "" throws
                # InvalidTopicNameError and kills the node — use a valid unused name.
                "imu_topic": "/orbslam3/rf2o_imu_unused",
                "publish_tf": False,        # Isaac owns odom->base; the EKF owns map->odom
                "base_frame_id": base_frame,
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
            }],
        ))
        actions.append(Node(
            package="robot_localization", executable="ekf_node",
            name="ekf_filter_node", output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        ))

    # ── Nav2 ────────────────────────────────────────────────────────────────────
    # The real rover's Nav2 navigation servers (planner/controller/bt/behaviors),
    # upstream source — ORB-SLAM3 supplies map->odom, the map builder supplies /map.
    # The sim's data topics are namespaced, so redirect /scan (costmap obstacle layer +
    # collision monitor) and /odom (controller) to the sim names; this propagates into
    # the include. /tf, /tf_static, /clock and /cmd_vel stay plain (shared/global).
    actions.append(GroupAction([
        SetRemap("/scan", "/ugv_sim/scan"),
        SetRemap("/odom", "/ugv_sim/odom"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "namespace": "",
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": autostart,
                "use_composition": use_composition,
                "use_respawn": use_respawn,
                "log_level": log_level,
            }.items(),
        ),
    ]))

    actions.append(Node(
        condition=IfCondition(use_rviz),
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use the /clock published by Isaac Sim"),
        DeclareLaunchArgument("use_localplan", default_value="rpp",
                              description="Nav2 local planner: teb, dwa, rpp, mppi "
                                          "(teb needs ros-humble-teb-local-planner)"),
        DeclareLaunchArgument("vocabulary", default_value=DEFAULT_VOCAB,
                              description="Path to ORB-SLAM3 ORBvoc.txt"),
        DeclareLaunchArgument("settings", default_value="",
                              description="ORB-SLAM3 settings yaml. EMPTY (default) = "
                                          "generate it from the live camera_info, so the "
                                          "intrinsics always match the baked camera"),
        DeclareLaunchArgument("orbslam_viewer", default_value="true",
                              description="Show ORB-SLAM3's OWN native visualization "
                                          "alongside RViz: the Pangolin 'Map Viewer' "
                                          "(keyframe graph + map points) and the OpenCV "
                                          "'Current Frame' (tracked ORB features). Needs a "
                                          "display; set false for headless runs"),
        DeclareLaunchArgument("camera_optical_frame", default_value=DEFAULT_CAMERA_FRAME,
                              description="RGB optical frame on /tf. If the map comes out "
                                          "mirrored/rotated, this is the knob — never the URDF"),
        DeclareLaunchArgument("camera_fps", default_value="10",
                              description="Camera rate written into the generated settings; "
                                          "drives ORB-SLAM3's keyframe cadence. Match "
                                          "`ros2 topic hz /ugv_sim/oak/rgb/image_rect`"),
        DeclareLaunchArgument("sync_queue", default_value="30",
                              description="message_filters queue for the RGB+depth "
                                          "approximate-time sync"),
        DeclareLaunchArgument("depth_min", default_value="0.1",
                              description="Min valid depth (m); below this -> no depth"),
        DeclareLaunchArgument("depth_max", default_value="10.0",
                              description="Max valid depth (m); above this (and Isaac's "
                                          "inf no-returns) -> no depth"),
        DeclareLaunchArgument("orb_features", default_value="1200",
                              description="ORB features per frame (raise if tracking is "
                                          "lost in sparse scenery)"),
        DeclareLaunchArgument("map_resolution", default_value="0.05",
                              description="Cell size (m) of the /map built from the "
                                          "ORB-SLAM3 map points"),
        DeclareLaunchArgument("map_obstacle_z_min", default_value="0.08",
                              description="Min height (m) of a map point counted as an "
                                          "obstacle (drops the floor)"),
        DeclareLaunchArgument("map_obstacle_z_max", default_value="1.5",
                              description="Max height (m) of a map point counted as an "
                                          "obstacle (drops the ceiling)"),
        DeclareLaunchArgument("map_max_ray_range", default_value="8.0",
                              description="Max range (m) from the camera a map point is "
                                          "trusted for the occupancy grid"),
        DeclareLaunchArgument("use_lidar_map", default_value="true",
                              description="Fold the 2D lidar /scan into the initial /map so "
                                          "it is dense and metric from the first scan "
                                          "(ORB-SLAM3 still owns map->odom). false = the "
                                          "old visual-only sparse grid from map points"),
        DeclareLaunchArgument("laser_frame", default_value=DEFAULT_LASER_FRAME,
                              description="2D lidar TF frame folded into /map when "
                                          "use_lidar_map is on (baked USD: base_lidar_link)"),
        DeclareLaunchArgument("map_scan_max_range", default_value="12.0",
                              description="Max range (m) a lidar return is trusted for the "
                                          "/map occupancy grid"),
        DeclareLaunchArgument("use_localization_fusion", default_value="true",
                              description="Fuse ORB-SLAM3 + rf2o laser odometry + wheel "
                                          "odom with a robot_localization EKF that owns "
                                          "map->odom, so localization keeps updating when "
                                          "visual tracking drops (e.g. in-place rotation). "
                                          "false = ORB-SLAM3 alone broadcasts map->odom "
                                          "(original pure-visual behavior). Needs "
                                          "robot_localization + rf2o (see PREREQUISITES)"),
        DeclareLaunchArgument("base_frame", default_value=DEFAULT_BASE_FRAME,
                              description="Robot base frame the EKF/rf2o/bridge estimate "
                                          "(baked USD: base_footprint)"),
        DeclareLaunchArgument("ekf_config", default_value=DEFAULT_EKF_CONFIG,
                              description="robot_localization EKF params for the fused "
                                          "map->odom (used when use_localization_fusion)"),
        DeclareLaunchArgument("save_atlas", default_value="",
                              description="Base path (no extension) ORB-SLAM3 saves its "
                                          "atlas to on exit -> <path>.osa (mapping run, "
                                          "Experiment A; e.g. .../experiments/maps/"
                                          "warehouse/orbslam3_atlas). Empty = live mapping. "
                                          "Needs the wrapper rebuilt (./build.sh)"),
        DeclareLaunchArgument("load_atlas", default_value="",
                              description="Base path (no extension) ORB-SLAM3 loads its "
                                          "atlas from at startup -> <path>.osa "
                                          "(localization run). Empty = live mapping"),
        DeclareLaunchArgument("localization_mode", default_value="false",
                              description="Call ActivateLocalizationMode(): relocalize + "
                                          "track against the loaded atlas without adding "
                                          "keyframes. Pair with load_atlas (Experiment A)"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz config; defaults to this v2 ORB-SLAM3 view"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Automatically start the nav2 stack"),
        DeclareLaunchArgument("use_composition", default_value="False",
                              description="Use composed Nav2 bringup"),
        DeclareLaunchArgument("use_respawn", default_value="False",
                              description="Respawn crashed nodes (composition off only)"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Nav2 log level"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with the 3D ORB-SLAM3 view"),
        OpaqueFunction(function=_launch_setup),
    ])
