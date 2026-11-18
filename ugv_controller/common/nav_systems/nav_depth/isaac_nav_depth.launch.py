"""Nav system: DEPTH-only RTAB-Map, mirroring the real rover's 2D-scan SLAM path.

Isaac-Sim shim that runs RTAB-Map depth-only: RGB is disabled and the front depth
camera is flattened into a synthetic 2D laser scan that drives rtabmap the same way
the real rover's lidar does. This is the appearance-INDEPENDENT sibling of nav_rgb —
same rover, camera, frames, and Nav2 params; only the map->odom source differs.
Diffing the two isolates the effect of RGB / visual fidelity on localization.

Why a 2D scan instead of feeding depth straight in: RTAB-Map cannot build an
occupancy grid from a bare depth image (its RGB-D frame needs RGB), and its
scan_cloud path is fragile. Projecting depth to a 2D scan uses rtabmap's most
reliable grid + registration path — exactly what the source uses for the real lidar
(subscribe_scan:True, Grid/Sensor=0) — just sourced from depth. This works for the
warehouse and future/randomized maps because rtabmap maps LIVE each run.

Pipeline (only the hardware bringup is replaced; Isaac publishes /odom,
odom->base_footprint, /cmd_vel, and the depth image on /oak/stereo/image_raw with
intrinsics on /oak/rgb/camera_info):
    /oak/stereo/image_raw --(depth_image_proc)--> /oak/depth/points
        --(pointcloud_to_laserscan)--> /oak/depth/scan
        --> rtabmap (subscribe_scan) + Nav2 costmaps (obstacle_layer/collision_monitor)
Isaac's lidar /scan is deliberately NOT used — the system is fully depth-driven.
The Nav2 navigation servers come from upstream navigation_launch.py; RTAB-Map
supplies /map and the map->odom TF, so no slam_toolbox or amcl runs here.

PREREQUISITES:
  * Build + source ugv_ws so `ugv_nav` is on the ament path:
        cd <ugv_ws> && colcon build --packages-select ugv_nav --symlink-install
        source install/setup.bash
  * apt: ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rtabmap-slam
         ros-humble-depth-image-proc ros-humble-pointcloud-to-laserscan

RUN (with Isaac running this system's run_small_warehouse_nav.py elsewhere):
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch <this folder>/isaac_nav_depth.launch.py
  # localize against a saved ~/.ros/rtabmap.db instead of mapping live:
  ros2 launch <this folder>/isaac_nav_depth.launch.py localization:=true
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The URDF lives in the (unbuilt) ugv_ws source tree; read it by absolute path and
# rewrite package:// mesh URIs to file:// so robot_state_publisher needs no install.
UGV_WS_SRC = "/home/joshroq/projects/isaac-assets/ugv_ws/src/ugv_main"
UGV_DESCRIPTION_SRC = os.path.join(UGV_WS_SRC, "ugv_description")
URDF_PATH = os.path.join(UGV_DESCRIPTION_SRC, "urdf", "ugv_rover_isaac_fixed.urdf")

# Nav2 param file per local planner — exactly the mapping upstream nav.launch.py uses.
_LOCALPLAN_CHOICES = ("teb", "dwa", "rpp", "mppi")

# Depth is flattened to a synthetic 2D laser scan (pointcloud_to_laserscan) published
# here, then fed to rtabmap AND the Nav2 costmaps as the ONLY range source, so the
# system stays fully depth-driven (Isaac's lidar /scan is never used). Building the grid
# from a 2D scan is rtabmap's most reliable path — the same one the source uses for the
# real lidar — which is why depth-only works this way instead of via scan_cloud.
DEPTH_SCAN_TOPIC = "/oak/depth/scan"

# rtabmap's default scan input -> the depth-derived 2D scan. RGB is never subscribed.
RTABMAP_REMAPPINGS = [("scan", DEPTH_SCAN_TOPIC)]


def _load_robot_description() -> str:
    if not os.path.isfile(URDF_PATH):
        raise RuntimeError(f"Isaac rover URDF not found: {URDF_PATH}")

    with open(URDF_PATH, "r") as f:
        robot_description = f.read()
    mesh_root = "file://" + UGV_DESCRIPTION_SRC.rstrip("/") + "/"
    # Share Isaac geometry; retain the legacy camera TF name used by this stack.
    return (robot_description.replace("../meshes/", mesh_root + "meshes/")
            .replace("package://ugv_description/", mesh_root)
            .replace("front_camera_link", "3d_camera_link"))


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


def _repoint_scan_topic(params: dict, scan_topic: str) -> None:
    """Point an obstacle_layer / collision_monitor 'scan' source at scan_topic."""
    scan = params.get("scan")
    if isinstance(scan, dict) and "topic" in scan:
        scan["topic"] = scan_topic


def _prepare_nav_params(source_file: str, scan_topic: str) -> str:
    """Rewrite the upstream Nav2 params for a depth-only, filter-free run.

    Two edits, neither of which touches the selected local planner, controller,
    smoother, behavior, or velocity params:

    1. Strip the keepout/speed costmap filters — their filter-info servers only run
       under the full nav2_bringup, so the plugins would spam "filter mask not
       received" here (navigation_launch only).
    2. Repoint every LaserScan observation source (local + global obstacle_layer and
       the collision_monitor) from Isaac's lidar /scan to the depth-derived scan, so
       obstacle avoidance is depth-only, consistent with rtabmap's input.
    """
    with open(source_file, "r") as f:
        data = yaml.safe_load(f)

    for outer, inner in (
        ("local_costmap", "local_costmap"),
        ("global_costmap", "global_costmap"),
    ):
        params = data.get(outer, {}).get(inner, {}).get("ros__parameters", {})
        _drop_filter(params, "keepout_filter")
        _drop_filter(params, "speed_filter")
        obstacle_layer = params.get("obstacle_layer")
        if isinstance(obstacle_layer, dict):
            _repoint_scan_topic(obstacle_layer, scan_topic)

    collision_monitor = data.get("collision_monitor", {}).get("ros__parameters", {})
    _repoint_scan_topic(collision_monitor, scan_topic)

    for server in (
        "keepout_filter_mask_server", "keepout_costmap_filter_info_server",
        "speed_filter_mask_server", "speed_costmap_filter_info_server",
    ):
        data.pop(server, None)

    fd, path = tempfile.mkstemp(prefix="isaac_nav_depth_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _rtabmap_parameters(use_sim_time, qos):
    """RTAB-Map DEPTH-only params, driven by a synthetic 2D laser scan.

    The depth cloud is flattened to a 2D scan (pointcloud_to_laserscan) and fed as
    rtabmap's laser scan — the same well-tested 2D-scan SLAM path the real rover uses
    for its lidar, just sourced from depth and with RGB off. Registration is 2D scan
    ICP (Reg/Strategy=1), the grid is built from that scan (Grid/Sensor=0), and the
    odometry guess comes from Isaac's odom->base_footprint TF. use_sim_time/qos are the
    only sim additions.
    """
    return {
        "frame_id": "base_footprint",
        "queue_size": 20,
        "approx_sync": True,
        "subscribe_rgb": False,
        "subscribe_depth": False,
        "subscribe_scan": True,         # 2D scan synthesized from the depth cloud
        "subscribe_odom_info": False,
        "Reg/Strategy": "1",            # 1 = ICP (2D scan matching)
        "Reg/Force3DoF": "true",        # planar rover
        "Icp/VoxelSize": "0.05",
        "Icp/MaxCorrespondenceDistance": "0.1",
        "Icp/PointToPlane": "false",
        "RGBD/NeighborLinkRefining": "true",
        "Rtabmap/DetectionRate": "3.5",
        # Occupancy grid (/map) built from the 2D scan (Grid/Sensor=0). Height-band
        # ground segmentation (NormalsSegmentation=false + Min/MaxGroundHeight) mirrors
        # the real rover's ugv_slam/rtabmap.launch.py grid params.
        "RGBD/CreateOccupancyGrid": "true",
        "Grid/Sensor": "0",                  # 0 = build the grid from the laser scan
        "Grid/RangeMin": "0.2",
        "Grid/RangeMax": "6.0",              # OAK depth is noisy past ~6 m
        "Grid/MinGroundHeight": "-0.2",
        "Grid/MaxGroundHeight": "0.0",
        "Grid/MaxObstacleHeight": "1.0",
        "Grid/NormalsSegmentation": "false",
        "Grid/RayTracing": "true",           # carve free/traversable cells
        # Isaac's depth render lags the /tf stream; a longer wait covers the startup
        # race. rtabmap declares this as a double, so it must be a float, not a string.
        "wait_for_transform": 0.5,
        "use_sim_time": use_sim_time,
        "qos": qos,
    }


def _launch_setup(context, *args, **kwargs):
    ugv_nav_dir = get_package_share_directory("ugv_nav")
    navigation_launch = _require_file(
        os.path.join(ugv_nav_dir, "launch", "nav_bringup", "navigation_launch.py"),
        "ugv_nav navigation launch",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = int(context.launch_configurations.get("qos", "2"))
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")

    # Resolve the local planner -> params/<localplan>.yaml (same as upstream/nav_lidar).
    localplan = context.launch_configurations.get("use_localplan", "rpp")
    if localplan not in _LOCALPLAN_CHOICES:
        raise RuntimeError(
            f"use_localplan must be one of {_LOCALPLAN_CHOICES}, got '{localplan}'"
        )
    params_file = _require_file(
        os.path.join(ugv_nav_dir, "params", f"{localplan}.yaml"),
        f"ugv_nav {localplan} params",
    )
    rviz_config = _launch_value(
        context, "rviz_config", os.path.join(ugv_nav_dir, "rviz", "view_nav_2d.rviz")
    )
    _require_file(rviz_config, "ugv_nav RViz config")
    params_file = _prepare_nav_params(params_file, DEPTH_SCAN_TOPIC)

    robot_description = _load_robot_description()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time,
                     "robot_description": robot_description}],
        output="screen",
    )

    # Isaac publishes the depth image in an optical frame; supply the static TF
    # 3d_camera_link -> oak_rgb_camera_optical_frame (REP-103 body->optical:
    # rpy = -pi/2, 0, -pi/2) so the depth point cloud lands on the robot. Identical
    # to nav_rgb (same camera, same frames).
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="oak_rgb_optical_tf",
        arguments=["--frame-id", "3d_camera_link",
                   "--child-frame-id", "oak_rgb_camera_optical_frame",
                   "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708"],
        parameters=[{"use_sim_time": use_sim_time}],
        respawn=use_respawn, respawn_delay=2.0,
        output="screen",
    )

    # Depth image -> PointCloud2 (/oak/depth/points). The depth is aligned to the RGB
    # pinhole, so it uses /oak/rgb/camera_info. RGB image is NOT consumed.
    depth_to_cloud = Node(
        package="depth_image_proc", executable="point_cloud_xyz_node",
        name="oak_depth_to_pointcloud", output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        respawn=use_respawn, respawn_delay=2.0,
        remappings=[
            ("image_rect", "/oak/stereo/image_raw"),
            ("camera_info", "/oak/rgb/camera_info"),
            ("points", "/oak/depth/points"),
        ])

    # PointCloud2 -> synthetic 2D LaserScan (DEPTH_SCAN_TOPIC). Flattens the depth cloud
    # into the planar base_footprint frame between MinGroundHeight..MaxObstacleHeight,
    # giving rtabmap + the Nav2 costmaps a reliable 2D scan built purely from depth.
    depth_to_scan = Node(
        package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="oak_depth_to_scan", output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "base_footprint",
            "transform_tolerance": 0.3,   # cover Isaac's depth/TF lag
            "min_height": 0.05,           # ignore the floor
            "max_height": 1.0,            # up to the rover's obstacle height
            "angle_min": -1.2,            # forward cone (~±69°, OAK-D-Lite HFOV)
            "angle_max": 1.2,
            "angle_increment": 0.0087,    # ~0.5°
            "scan_time": 0.2,
            "range_min": 0.2,
            "range_max": 6.0,             # OAK depth is noisy past ~6 m
            "use_inf": True,
        }],
        respawn=use_respawn, respawn_delay=2.0,
        remappings=[("cloud_in", "/oak/depth/points"),
                    ("scan", DEPTH_SCAN_TOPIC)])

    # RTAB-Map DEPTH-only localization/mapping (2D scan from depth), RGB off.
    rtabmap_parameters = _rtabmap_parameters(use_sim_time, qos)

    # Mapping (SLAM) mode — builds the map live and localizes within it.
    rtabmap_slam = Node(
        condition=LaunchConfigurationEquals("localization", "false"),
        package="rtabmap_slam", executable="rtabmap", output="screen",
        parameters=[rtabmap_parameters],
        remappings=RTABMAP_REMAPPINGS,
        respawn=use_respawn, respawn_delay=2.0,
        arguments=["-d"])  # delete previous ~/.ros/rtabmap.db

    # Pure-localization mode — localize against the saved database.
    rtabmap_localization = Node(
        condition=LaunchConfigurationEquals("localization", "true"),
        package="rtabmap_slam", executable="rtabmap", output="screen",
        parameters=[rtabmap_parameters,
                    {"Mem/IncrementalMemory": "False",
                     "Mem/InitWMWithAllNodes": "True"}],
        remappings=RTABMAP_REMAPPINGS,
        respawn=use_respawn, respawn_delay=2.0)

    # The real rover's Nav2 navigation servers (planner/controller/bt/behaviors),
    # shared upstream source — RTAB-Map above provides the map + map->odom.
    nav2 = IncludeLaunchDescription(
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
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return [
        robot_state_publisher,
        camera_optical_tf,
        depth_to_cloud,
        depth_to_scan,
        rtabmap_slam,
        rtabmap_localization,
        nav2,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use the /clock published by Isaac Sim"),
        DeclareLaunchArgument("use_localplan", default_value="rpp",
                              description="Nav2 local planner: teb, dwa, rpp, mppi "
                                          "(teb needs ros-humble-teb-local-planner)"),
        DeclareLaunchArgument("localization", default_value="false",
                              description="Localize against a saved ~/.ros/rtabmap.db "
                                          "(true) instead of mapping live (false)"),
        DeclareLaunchArgument("qos", default_value="2",
                              description="QoS used for rtabmap input sensor topics"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="Optional RViz config path"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Automatically start the nav2 stack"),
        DeclareLaunchArgument("use_composition", default_value="False",
                              description="Use composed Nav2 bringup"),
        DeclareLaunchArgument("use_respawn", default_value="False",
                              description="Respawn crashed nodes (composition off only)"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Nav2 log level"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with the 2D navigation view"),
        OpaqueFunction(function=_launch_setup),
    ])
