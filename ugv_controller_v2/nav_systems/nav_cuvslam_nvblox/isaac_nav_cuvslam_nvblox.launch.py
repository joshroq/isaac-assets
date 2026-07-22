"""Nav system: Isaac ROS Visual SLAM (cuVSLAM) + nvblox + Nav2 — Part 1 stack.

PART 1 STACK HEADER
    Localization       : Isaac ROS Visual SLAM / cuVSLAM (STEREO, IMU on by default)
    Obstacle / costmap : nvblox depth reconstruction (ESDF 2D slice → Nav2 costmap)
    RGB used           : yes (stereo pair for cuVSLAM; color for nvblox)
    Depth used         : yes (nvblox reconstruction; cuVSLAM does not use depth)
    LiDAR used         : NO  (the 2D /scan is never used for localization or costmap)
    IMU used           : yes, by default (cuVSLAM VIO; enable_imu_fusion:=false to disable)
    Role               : the NVIDIA visual-depth navigation stack for Part 1

This is the main NVIDIA visual-depth navigation stack for the Part 1 SLAM/Nav2
fidelity study. It does NOT depend on `/ugv_sim/scan` for localization or obstacle
detection — cuVSLAM localizes from the OAK stereo pair (+ IMU) and nvblox reconstructs
the scene from the pseudo-depth image into the Nav2 costmap.

PIPELINE
    Isaac Sim OAK stereo pair (+ IMU)         Isaac Sim color + pseudo-depth
      /ugv_sim/oak/{left,right}/image_rect      /ugv_sim/oak/rgb/image_rect
      + camera_info                             /ugv_sim/oak/stereo/image_raw
            │                                          │
            ▼                                          ▼
    Isaac ROS Visual SLAM / cuVSLAM (stereo)   nvblox (depth + color + TF pose)
            │  broadcasts  map → odom  (TF) ─────────► uses cuVSLAM's map->odom
            │                                          │  ESDF 2D slice
            │                                          ▼  /nvblox_node/static_map_slice
            └───────────────► Nav2 (nvblox_layer costmap plugin, shared ugv_nav) ──► /cmd_vel

SOURCE (built + sourced separately; see this folder's README.md)
    Isaac ROS Visual SLAM : ~/projects/slam_source/isaac_ros_visual_slam
    nvblox                : ~/projects/slam_source/isaac_ros_nvblox
    The cuVSLAM node (isaac_ros_visual_slam), the nvblox node (nvblox_ros) and the
    Nav2 costmap plugin (nvblox_nav2::NvbloxCostmapLayer) must be colcon-built and
    on the ament/plugin path — this launch instantiates them like the NVIDIA
    nvblox_examples Isaac Sim bringup does, but wired to the ugv rover's ROS
    contract and the shared ugv_nav Nav2 stack.

CONTRACT THIS LAUNCH EXPECTS ISAAC TO PUBLISH (see ugv_controller_v2/USD_CONTRACT.md)
    subscribes : /cmd_vel
    publishes  : /clock, /tf + /tf_static, /ugv_sim/odom, /ugv_sim/imu/data, and the OAK:
                 - stereo pair for cuVSLAM: /ugv_sim/oak/left/image_rect (+ camera_info,
                   frame Camera_OmniVision_OV9782_Left) and /ugv_sim/oak/right/image_rect
                   (+ camera_info, frame Camera_OmniVision_OV9782_Right)
                 - color for nvblox: /ugv_sim/oak/rgb/image_rect + camera_info
                   (frame Camera_OmniVision_OV9782_Color)
                 - pseudo-depth for nvblox: /ugv_sim/oak/stereo/image_raw
                   (frame Camera_Pseudo_Depth)
                 /scan is NOT used. cuVSLAM then provides map->odom; nvblox provides the
                 costmap slice; Nav2 outputs /cmd_vel.

RUN (Isaac running this system's rover elsewhere):
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  source <isaac_ros_ws>/install/setup.bash     # cuVSLAM + nvblox + nvblox_nav2
  source ~/.local/bin/ros2ddsenv
  ros2 launch <this file>
  ros2 launch <this file> enable_imu_fusion:=true     # cuVSLAM VIO (uses the IMU)
  ros2 launch <this file> use_localplan:=rpp          # planner: teb|dwa|rpp|mppi
"""

import os
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode

# --- Isaac Sim ROS contract (ugv_sim namespace) -----------------------------------
# cuVSLAM (Isaac ROS 3.x) is a STEREO tracker: it needs a left+right rectified image
# pair, NOT mono+depth (the old RGBD tracking_mode was removed). The rover's OAK/RSD455
# module has a real OV9782 stereo pair; the baked ROS_Camera graph now publishes both
# eyes (see USD_CONTRACT.md). cuVSLAM localizes from that pair; nvblox still uses the
# color + pseudo-depth images below for reconstruction.
LEFT_IMAGE_TOPIC = "/ugv_sim/oak/left/image_rect"
LEFT_INFO_TOPIC = "/ugv_sim/oak/left/camera_info"
RIGHT_IMAGE_TOPIC = "/ugv_sim/oak/right/image_rect"
RIGHT_INFO_TOPIC = "/ugv_sim/oak/right/camera_info"
LEFT_OPTICAL_FRAME = "Camera_OmniVision_OV9782_Left"
RIGHT_OPTICAL_FRAME = "Camera_OmniVision_OV9782_Right"

# Color + pseudo-depth (used by nvblox reconstruction + the collision-monitor cloud).
RGB_IMAGE_TOPIC = "/ugv_sim/oak/rgb/image_rect"
RGB_INFO_TOPIC = "/ugv_sim/oak/rgb/camera_info"
DEPTH_IMAGE_TOPIC = "/ugv_sim/oak/stereo/image_raw"
IMU_TOPIC = "/ugv_sim/imu/data"
ODOM_TOPIC = "/ugv_sim/odom"

# Depth XYZ cloud we synthesize for the collision monitor + RViz (nvblox owns the
# planning costmap; this cloud is just a live safety/visualization obstacle source).
DEPTH_CLOUD_TOPIC = "/ugv_sim/oak/depth/points"
# nvblox's ESDF 2D slice, consumed by the Nav2 nvblox costmap layer (static mode).
NVBLOX_SLICE_TOPIC = "/nvblox_node/static_map_slice"

# nvblox parameter files, from the source tree in ~/projects/slam_source. Preferred
# lookup is the installed nvblox_examples_bringup share; fall back to source.
NVBLOX_SRC_CONFIG = os.path.expanduser(
    "~/projects/slam_source/isaac_ros_nvblox/nvblox_examples/"
    "nvblox_examples_bringup/config/nvblox"
)

# The ugv rover's Isaac-matched URDF, for the RViz RobotModel (same as nav_lidar).
UGV_WS_SRC = "/home/joshroq/projects/isaac-assets/ugv_ws/src/ugv_main"
UGV_DESCRIPTION_SRC = os.path.join(UGV_WS_SRC, "ugv_description")
URDF_PATH = os.path.join(UGV_DESCRIPTION_SRC, "urdf", "ugv_rover_isaac_fixed.urdf")

_LOCALPLAN_CHOICES = ("teb", "dwa", "rpp", "mppi")
DEFAULT_RVIZ_CONFIG = str(Path(__file__).with_name("isaac_cuvslam_nvblox_nav.rviz"))
NVBLOX_CONTAINER = "nvblox_container"


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


def _load_robot_description() -> str:
    if not os.path.isfile(URDF_PATH):
        raise RuntimeError(f"Isaac rover URDF not found: {URDF_PATH}")
    with open(URDF_PATH, "r") as f:
        robot_description = f.read()
    desc_root = "file://" + UGV_DESCRIPTION_SRC.rstrip("/") + "/"
    return (
        robot_description
        .replace("../meshes/", desc_root + "meshes/")
        .replace("package://ugv_description/", desc_root)
        .replace("front_camera_link.stl", "3d_camera_link.stl")
    )


def _nvblox_config(name: str) -> str:
    """Resolve an nvblox config yaml, preferring the installed bringup share and
    falling back to the source tree in ~/projects/slam_source."""
    try:
        share = get_package_share_directory("nvblox_examples_bringup")
        candidate = os.path.join(share, "config", "nvblox", name)
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    return _require_file(os.path.join(NVBLOX_SRC_CONFIG, name),
                         f"nvblox config {name}")


def _drop_filter(params: dict, filter_name: str) -> None:
    filters = params.get("filters")
    if isinstance(filters, list) and filter_name in filters:
        params["filters"] = [item for item in filters if item != filter_name]
        if not params["filters"]:
            params.pop("filters", None)
    params.pop(filter_name, None)


def _set_nvblox_costmap_layer(params: dict, global_frame: str) -> None:
    """Replace a costmap's obstacle/static/denoise/voxel layers with the nvblox
    costmap layer (fed by nvblox's ESDF slice), keeping only inflation on top."""
    inflation = params.get("inflation_layer")
    params["plugins"] = ["nvblox_layer", "inflation_layer"]
    params["nvblox_layer"] = {
        "plugin": "nvblox::nav2::NvbloxCostmapLayer",
        "enabled": True,
        "nav2_costmap_global_frame": global_frame,
        "nvblox_map_slice_topic": NVBLOX_SLICE_TOPIC,
        "convert_to_binary_costmap": True,
    }
    if isinstance(inflation, dict):
        params["inflation_layer"] = inflation
    else:
        params["inflation_layer"] = {
            "plugin": "nav2_costmap_2d::InflationLayer",
            "enabled": True,
            "cost_scaling_factor": 10.0,
            "inflation_radius": 0.55,
        }
    # Drop the LiDAR/appearance layers this stack no longer uses.
    for layer in ("obstacle_layer", "static_layer", "denoise_layer", "voxel_layer"):
        params.pop(layer, None)


def _prepare_nav_params(context, source_file: str) -> str:
    """Rewrite the upstream ugv_nav params for a cuVSLAM + nvblox (no LiDAR) run.

    None of this touches the selected planner/controller/smoother/behavior params —
    the local planner stays exactly the shared ugv_nav default:
      1. Local + global costmaps use the nvblox costmap layer (+ inflation) instead
         of the LiDAR obstacle/static layers. nvblox is the only obstacle source.
      2. The global costmap is made a rolling window with a fixed size, so it needs
         no /map occupancy grid (cuVSLAM/nvblox do not publish one — nvblox feeds
         the costmap through its layer plugin).
      3. keepout/speed filters are stripped (their info servers don't run here).
      4. The collision_monitor uses the depth XYZ cloud (no LiDAR scan).
    """
    global_size = float(_launch_value(context, "global_costmap_size", "20.0"))
    min_h = float(_launch_value(context, "depth_min_height", "0.08"))
    max_h = float(_launch_value(context, "depth_max_height", "1.5"))

    with open(source_file, "r") as f:
        data = yaml.safe_load(f)

    local = data.get("local_costmap", {}).get("local_costmap", {}).get("ros__parameters", {})
    if local:
        _drop_filter(local, "keepout_filter")
        _drop_filter(local, "speed_filter")
        _set_nvblox_costmap_layer(local, local.get("global_frame", "odom"))

    glob = data.get("global_costmap", {}).get("global_costmap", {}).get("ros__parameters", {})
    if glob:
        _drop_filter(glob, "keepout_filter")
        _drop_filter(glob, "speed_filter")
        _set_nvblox_costmap_layer(glob, glob.get("global_frame", "map"))
        # No /map occupancy grid exists in this stack: make the global costmap a
        # rolling window of a fixed size so it is self-sufficient. Nav2 declares
        # costmap width/height as *integer* metres, so these must stay ints — passing
        # a float (e.g. 20.0) raises "parameter 'height' ... is of type {integer},
        # setting it to {double} is not allowed" and kills planner_server on configure.
        glob["rolling_window"] = True
        glob["width"] = int(round(global_size))
        glob["height"] = int(round(global_size))
        glob["track_unknown_space"] = False

    for server in ("keepout_filter_mask_server", "keepout_costmap_filter_info_server",
                   "speed_filter_mask_server", "speed_costmap_filter_info_server"):
        data.pop(server, None)

    # Collision monitor: depth cloud instead of the LiDAR scan.
    cm = data.get("collision_monitor", {}).get("ros__parameters", {})
    pc = cm.get("pointcloud")
    if isinstance(pc, dict):
        pc["topic"] = DEPTH_CLOUD_TOPIC
        pc["enabled"] = True
        pc["min_height"] = min_h
        pc["max_height"] = max_h
    if cm:
        cm["observation_sources"] = ["pointcloud"]

    fd, path = tempfile.mkstemp(prefix="isaac_v2_nav_cuvslam_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _visual_slam_node(context) -> ComposableNode:
    """Isaac ROS Visual SLAM / cuVSLAM, STEREO tracking, wired to the OAK stereo pair.

    cuVSLAM (Isaac ROS 3.x, cuVSLAM 12.6) is stereo-only: image_0/1 are the left/right
    rectified images, num_cameras=2. (The old single-camera RGBD tracking_mode was
    removed from the node.) enable_imu_fusion adds the IMU (cuVSLAM VIO). cuVSLAM
    broadcasts map->odom, the localization output Nav2 consumes (the role slam_toolbox
    plays in nav_lidar, RTAB-Map in nav_rgb)."""
    imu_fusion = _launch_bool(context, "enable_imu_fusion", default=True)
    ground_constraint = _launch_bool(context, "enable_ground_constraint", default=True)
    left_frame = _launch_value(context, "left_optical_frame", LEFT_OPTICAL_FRAME)
    right_frame = _launch_value(context, "right_optical_frame", RIGHT_OPTICAL_FRAME)

    params = {
        "use_sim_time": True,
        "num_cameras": 2,                 # stereo pair (left=0, right=1)
        "rectified_images": True,         # /oak/{left,right}/image_rect are rectified
        "enable_image_denoising": False,
        "enable_localization_n_mapping": True,   # SLAM (loop closure), not bare VO
        "enable_imu_fusion": imu_fusion,
        "enable_ground_constraint_in_odometry": ground_constraint,
        "enable_ground_constraint_in_slam": ground_constraint,
        "map_frame": "map",
        "odom_frame": "odom",
        "base_frame": "base_link",
        "publish_map_to_odom_tf": True,   # this is how Nav2 gets map->odom
        "publish_odom_to_base_tf": False,  # Isaac already owns odom->base_link
        "camera_optical_frames": [left_frame, right_frame],
        "image_jitter_threshold_ms": 34.0,
        "enable_slam_visualization": True,
        "enable_landmarks_view": True,
        "enable_observations_view": True,
    }
    if imu_fusion:
        params["imu_frame"] = _launch_value(context, "imu_frame", "base_imu_link")

    remappings = [
        ("visual_slam/image_0", LEFT_IMAGE_TOPIC),
        ("visual_slam/camera_info_0", LEFT_INFO_TOPIC),
        ("visual_slam/image_1", RIGHT_IMAGE_TOPIC),
        ("visual_slam/camera_info_1", RIGHT_INFO_TOPIC),
    ]
    if imu_fusion:
        remappings.append(("visual_slam/imu", IMU_TOPIC))

    return ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[params],
        remappings=remappings,
    )


def _nvblox_node() -> ComposableNode:
    """nvblox reconstruction: RGB-D → TSDF/ESDF, publishing the 2D ESDF slice the
    Nav2 nvblox costmap layer consumes. Single camera, no LiDAR. Poses come from TF
    (map->odom from cuVSLAM, odom->base_link + base->camera from Isaac)."""
    base_config = _nvblox_config("nvblox_base.yaml")
    sim_config = _nvblox_config(os.path.join("specializations", "nvblox_sim.yaml"))
    remappings = [
        ("camera_0/depth/image", DEPTH_IMAGE_TOPIC),
        ("camera_0/depth/camera_info", RGB_INFO_TOPIC),
        ("camera_0/color/image", RGB_IMAGE_TOPIC),
        ("camera_0/color/camera_info", RGB_INFO_TOPIC),
    ]
    return ComposableNode(
        name="nvblox_node",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        parameters=[base_config, sim_config,
                    {"use_sim_time": True, "num_cameras": 1, "use_lidar": False}],
        remappings=remappings,
    )


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
    queue_size = int(context.launch_configurations.get("queue_size", "30"))

    localplan = context.launch_configurations.get("use_localplan", "teb")
    if localplan not in _LOCALPLAN_CHOICES:
        raise RuntimeError(
            f"use_localplan must be one of {_LOCALPLAN_CHOICES}, got '{localplan}'"
        )
    params_file = _require_file(
        os.path.join(ugv_nav_dir, "params", f"{localplan}.yaml"),
        f"ugv_nav {localplan} params",
    )
    rviz_config = _launch_value(context, "rviz_config", DEFAULT_RVIZ_CONFIG)
    _require_file(rviz_config, "nav_cuvslam_nvblox RViz config")
    params_file = _prepare_nav_params(context, params_file)

    actions = []

    # cuVSLAM + nvblox run together in one isolated component container, exactly the
    # way the NVIDIA nvblox_examples Isaac Sim bringup composes them.
    actions.append(ComposableNodeContainer(
        name=NVBLOX_CONTAINER,
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[
            _visual_slam_node(context),
            _nvblox_node(),
        ],
        output="screen",
    ))

    # robot_state_publisher publishes /robot_description ONLY for the RViz
    # RobotModel; its /tf are dead-ended so it never competes with Isaac's TF.
    actions.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time,
                     "robot_description": _load_robot_description()}],
        remappings=[("/tf", "/rsp_tf_unused"),
                    ("/tf_static", "/rsp_tf_static_unused")],
        output="screen",
    ))

    # Depth image -> XYZ cloud (no RGB) for the collision monitor + RViz. nvblox owns
    # the planning costmap; this is just a live safety/visualization obstacle source.
    actions.append(Node(
        package="depth_image_proc", executable="point_cloud_xyz_node",
        name="isaac_depth_cloud",
        parameters=[{"use_sim_time": use_sim_time, "queue_size": queue_size}],
        remappings=[
            ("image_rect", DEPTH_IMAGE_TOPIC),
            ("camera_info", RGB_INFO_TOPIC),
            ("points", DEPTH_CLOUD_TOPIC),
        ],
        output="screen",
    ))

    # Nav2: the shared ugv_nav planner/controller/bt/behaviors, with the costmaps
    # rewritten to the nvblox layer. cuVSLAM supplies map->odom, so NO slam_toolbox,
    # amcl or RTAB-Map runs. Only /odom is redirected to the sim namespace — there is
    # NO /scan remap (this stack never uses the LiDAR). /tf, /tf_static, /clock and
    # /cmd_vel stay plain (shared/global).
    actions.append(GroupAction([
        SetRemap("/odom", ODOM_TOPIC),
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
        DeclareLaunchArgument("use_localplan", default_value="teb",
                              description="Nav2 local planner: teb, dwa, rpp, mppi "
                                          "(shared ugv_nav default, same as the other "
                                          "stacks; teb needs ros-humble-teb-local-planner)"),
        DeclareLaunchArgument("enable_imu_fusion", default_value="true",
                              description="Fuse /ugv_sim/imu/data in cuVSLAM (VIO); on by "
                                          "default. Disable with enable_imu_fusion:=false"),
        DeclareLaunchArgument("imu_frame", default_value="base_imu_link",
                              description="TF frame of the IMU (used iff enable_imu_fusion)"),
        DeclareLaunchArgument("enable_ground_constraint", default_value="true",
                              description="Constrain cuVSLAM to the ground plane "
                                          "(recommended for this planar rover)"),
        DeclareLaunchArgument("left_optical_frame", default_value=LEFT_OPTICAL_FRAME,
                              description="Left stereo camera optical frame (must match "
                                          "the left camera_info header frame Isaac "
                                          "publishes)"),
        DeclareLaunchArgument("right_optical_frame", default_value=RIGHT_OPTICAL_FRAME,
                              description="Right stereo camera optical frame (must match "
                                          "the right camera_info header frame Isaac "
                                          "publishes)"),
        DeclareLaunchArgument("global_costmap_size", default_value="20.0",
                              description="Rolling global costmap side length (m); this "
                                          "stack has no /map occupancy grid"),
        DeclareLaunchArgument("depth_min_height", default_value="0.08",
                              description="Min height (m) of collision-monitor depth points"),
        DeclareLaunchArgument("depth_max_height", default_value="1.5",
                              description="Max height (m) of collision-monitor depth points"),
        DeclareLaunchArgument("queue_size", default_value="30",
                              description="depth+camera_info sync queue for the depth cloud"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz config; defaults to this stack's view"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Automatically start the nav2 stack"),
        DeclareLaunchArgument("use_composition", default_value="False",
                              description="Use composed Nav2 bringup"),
        DeclareLaunchArgument("use_respawn", default_value="False",
                              description="Respawn crashed nodes (composition off only)"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Nav2 log level"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with the nvblox navigation view"),
        OpaqueFunction(function=_launch_setup),
    ])
