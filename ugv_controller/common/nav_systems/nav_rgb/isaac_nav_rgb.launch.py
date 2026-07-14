"""Nav system: LiDAR + RGB-D RTAB-Map, mirroring the real rover.

Isaac-Sim shim around the SAME RTAB-Map RGB-D localization the real WaveShare
rover runs via `ugv_slam/launch/rtabmap_rgbd.launch.py`. The `rtabmap` node here
uses that launch's parameters and `/oak` topic remappings verbatim, so the sim
localizes exactly as the hardware. Only the hardware bringup is replaced: Isaac
publishes `/scan`, `/odom`, `odom -> base_footprint`, `/cmd_vel`, and the front
camera on `/oak/rgb/image_rect`, `/oak/rgb/camera_info`, `/oak/stereo/image_raw`
(the same topic names the real depthai/oak driver uses). The Nav2 navigation
servers come from upstream `ugv_nav/launch/nav_bringup/navigation_launch.py`, the
same source the real rover and nav_lidar use — RTAB-Map supplies `/map` and the
`map -> odom` TF, so no slam_toolbox or amcl runs here.

Sibling: nav_depth (isaac_nav_depth.launch.py) is the same rover + camera but
disables RGB in RTAB-Map and registers on depth geometry (ICP) only. Diffing the
two isolates the effect of RGB / visual fidelity on localization.

Equivalence to the real rover:
    real:  ros2 launch ugv_slam rtabmap_rgbd.launch.py   (+ ugv_nav navigation)
    isaac: ros2 launch isaac_nav_rgb.launch.py
Both resolve to the same rtabmap node config and the same ugv_nav Nav2 params, so
this can run against the same ROS 2 graph the real rover uses.

PREREQUISITES:
  * Build + source ugv_ws so `ugv_nav` is on the ament path:
        cd <ugv_ws> && colcon build --packages-select ugv_nav --symlink-install
        source install/setup.bash
  * apt: ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rtabmap-slam
         ros-humble-rtabmap-sync ros-humble-depth-image-proc

RUN (with Isaac running this system's run_small_warehouse_nav.py elsewhere):
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch <this folder>/isaac_nav_rgb.launch.py
  # localize against a saved ~/.ros/rtabmap.db instead of mapping live:
  ros2 launch <this folder>/isaac_nav_rgb.launch.py localization:=true
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
URDF_PATH = os.path.join(UGV_DESCRIPTION_SRC, "urdf", "ugv_rover.urdf")

# Nav2 param file per local planner — exactly the mapping upstream nav.launch.py uses.
_LOCALPLAN_CHOICES = ("teb", "dwa", "rpp", "mppi")

# rtabmap input remappings -> the /oak topics Isaac publishes (identical to the
# real rover's ugv_slam/launch/rtabmap_rgbd.launch.py). /scan is rtabmap's default
# topic name and maps to Isaac's /scan.
RTABMAP_REMAPPINGS = [
    ("rgb/image", "/oak/rgb/image_rect"),
    ("rgb/camera_info", "/oak/rgb/camera_info"),
    ("depth/image", "/oak/stereo/image_raw"),
]


def _load_robot_description() -> str:
    if not os.path.isfile(URDF_PATH):
        raise RuntimeError(f"Isaac rover URDF not found: {URDF_PATH}")

    with open(URDF_PATH, "r") as f:
        robot_description = f.read()
    mesh_root = "file://" + UGV_DESCRIPTION_SRC.rstrip("/") + "/"
    return robot_description.replace("package://ugv_description/", mesh_root)


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
    """Strip keepout/speed costmap filters from the Nav2 params.

    The upstream planner params list keepout/speed filters in the costmaps, but the
    matching filter-info servers only run under the full nav2_bringup. nav_rgb runs
    RTAB-Map + navigation_launch only, so dropping the disabled filter plugins avoids
    continuous "filter mask was not received" warnings without touching the selected
    local planner, controller, smoother, behavior, or velocity params.
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

    for server in (
        "keepout_filter_mask_server", "keepout_costmap_filter_info_server",
        "speed_filter_mask_server", "speed_costmap_filter_info_server",
    ):
        data.pop(server, None)

    fd, path = tempfile.mkstemp(prefix="isaac_nav_rgb_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _rtabmap_parameters(use_sim_time, qos):
    """RTAB-Map RGB-D params, verbatim from ugv_slam/rtabmap_rgbd.launch.py.

    Only use_sim_time (Isaac /clock) and qos (Isaac best-effort sensors) are
    sim-necessary additions, so the sim localizes exactly as the hardware.
    """
    return {
        "frame_id": "base_footprint",
        "queue_size": 20,
        "subscribe_rgb": True,
        "subscribe_depth": True,
        "subscribe_scan": True,
        "subscribe_odom_info": False,
        "approx_sync": True,
        "Rtabmap/DetectionRate": "3.5",
        # Occupancy grid (/map) built from the depth image (Grid/Sensor default = camera).
        # Height-band ground segmentation mirrors the real rover's ugv_slam/rtabmap.launch.py
        # so the /map is populated for Nav2's global costmap.
        "Grid/RangeMin": "0.2",
        "Grid/MinGroundHeight": "-0.2",
        "Grid/MaxGroundHeight": "0.0",
        "Grid/MaxObstacleHeight": "1.0",
        "Grid/NormalsSegmentation": "false",
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
    params_file = _write_params_without_filters(params_file)

    robot_description = _load_robot_description()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time,
                     "robot_description": robot_description}],
        output="screen",
    )

    # Isaac publishes the depth/RGB images in an optical frame; supply the static TF
    # 3d_camera_link -> oak_rgb_camera_optical_frame (REP-103 body->optical:
    # rpy = -pi/2, 0, -pi/2) so rtabmap can place the camera on the robot.
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

    # Visualization only: publishes the same /oak/depth/points topic nav_depth does
    # so the shared RViz "Depth Cloud" display shows what the depth sensor sees in
    # both camera systems (depth is aligned to the RGB pinhole -> /oak/rgb/camera_info).
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

    # RTAB-Map RGB-D localization/mapping — the real rover's node, verbatim. rgbd_sync
    # fuses RGB+depth; rtabmap consumes the synced image + /scan, using Isaac's
    # odom->base_footprint TF as the odometry guess and publishing /map + map->odom.
    rtabmap_parameters = _rtabmap_parameters(use_sim_time, qos)

    rgbd_sync = Node(
        package="rtabmap_sync", executable="rgbd_sync", output="screen",
        parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.01,
                     "use_sim_time": use_sim_time, "qos": qos}],
        respawn=use_respawn, respawn_delay=2.0,
        remappings=RTABMAP_REMAPPINGS)

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
        rgbd_sync,
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
