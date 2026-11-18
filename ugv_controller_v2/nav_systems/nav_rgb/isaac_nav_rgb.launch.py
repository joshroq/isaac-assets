"""Nav system: RTAB-Map RGB-D + LiDAR + Nav2 for the BAKED-GRAPH rover.

The RGB-D counterpart of ugv_controller_v2/nav_systems/nav_lidar. It is built the
same way nav_lidar is: it launches the **real rover's own ugv_ws source** and only
replaces the hardware bringup with Isaac's baked ROS 2 graphs.

  * RTAB-Map — the rtabmap node + parameters + `/oak` remappings are replicated
    VERBATIM from `ugv_slam/launch/rtabmap_rgbd.launch.py` (the real rover's RGB-D
    SLAM launch). We drop only that launch's hardware includes
    (`ugv_vision/oak_d_lite.launch.py` + `ugv_bringup/bringup_lidar.launch.py` +
    robot_pose_publisher), because Isaac already publishes the camera, lidar,
    odometry and TF. Everything RTAB-Map-side is identical to hardware.
  * Nav2 — the planner/controller/bt/behavior servers come from upstream
    `ugv_nav/launch/nav_bringup/navigation_launch.py`, the same source nav_lidar
    and the real rover use. RTAB-Map supplies `/map` + `map->odom`, so no
    slam_toolbox/amcl runs here (contrast nav_lidar, which uses slam_toolbox).

Equivalence to the real rover:
    real:  ros2 launch ugv_slam rtabmap_rgbd.launch.py   (+ ugv_nav navigation)
    isaac: ros2 launch isaac_nav_rgb.launch.py

CONTRACT THIS LAUNCH EXPECTS ISAAC TO PUBLISH (see ugv_controller_v2/USD_CONTRACT.md)
    subscribes : /cmd_vel
    publishes  : /clock, /odom, /scan, /tf + /tf_static, and the OAK camera on
                 /oak/rgb/image_rect, /oak/rgb/camera_info, /oak/stereo/image_raw
                 (frame = the camera optical frame, present in /tf).
    RTAB-Map then provides /map + map->odom; Nav2 outputs /cmd_vel.

PREREQUISITES
  * Build + source ugv_ws so `ugv_nav` is on the ament path:
        cd <ugv_ws> && colcon build --packages-select ugv_nav --symlink-install
        source install/setup.bash
  * apt: ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rtabmap-slam
         ros-humble-rtabmap-sync ros-humble-depth-image-proc

RUN (Isaac running this system's rover elsewhere):
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  source ~/.local/bin/ros2ddsenv
  ros2 launch <this file>                              # live mapping, 3D RViz
  ros2 launch <this file> localization:=true           # localize vs saved db
  ros2 launch <this file> use_localplan:=teb            # planner: teb|dwa|rpp|mppi
  ros2 launch <this file> use_robot_state_publisher:=true  # USD-iteration fallback

KNOWN-MAP MODE (thesis Experiment A — build one RTAB-Map db, then localize in it):
  # 1) mapping run: drive the full environment, db saved on exit to the named file
  ros2 launch <this file> localization:=false \
      database_path:=<repo>/experiments/maps/warehouse/rtabmap.db
  # 2) per-trial localization run: reuse that db (no relearning), no priming drive
  ros2 launch <this file> localization:=true \
      database_path:=<repo>/experiments/maps/warehouse/rtabmap.db
Default (no database_path) is unchanged: live mapping into ~/.ros/rtabmap.db.
"""

import os
import tempfile
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

# URDF for the RViz RobotModel. Use the Isaac-matched URDF whose link names equal
# the frames the baked USD publishes on /tf, so RViz renders the model at live poses.
UGV_WS_SRC = "/home/joshroq/projects/isaac-assets/ugv_ws/src/ugv_main"
UGV_DESCRIPTION_SRC = os.path.join(UGV_WS_SRC, "ugv_description")
URDF_PATH = os.path.join(UGV_DESCRIPTION_SRC, "urdf", "ugv_rover_isaac_fixed.urdf")

_LOCALPLAN_CHOICES = ("teb", "dwa", "rpp", "mppi")

# rtabmap input remappings -> the camera topics Isaac publishes. The real rover's
# ugv_slam/rtabmap_rgbd.launch.py uses the plain /oak/* names; the sim publishes the
# SAME leaf names under the `ugv_sim` namespace (so real + sim can run at once), so
# here we point rtabmap at /ugv_sim/oak/*. (The /scan and /odom inputs are handled by
# the SetRemap group below.)
RTABMAP_REMAPPINGS = [
    ("rgb/image", "/ugv_sim/oak/rgb/image_rect"),
    ("rgb/camera_info", "/ugv_sim/oak/rgb/camera_info"),
    ("depth/image", "/ugv_sim/oak/stereo/image_raw"),
]

RGBD_CLOUD_TOPIC = "/ugv_sim/oak/rgbd/points"
DEFAULT_RVIZ_CONFIG = str(Path(__file__).with_name("isaac_rgb_nav.rviz"))


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
    # base_imu_link is kept: the baked USD now publishes base_link->base_imu_link
    # on /tf_static (ROS_TF_Static graph) and the IMU on /ugv_sim/imu/data, so the
    # RViz RobotModel resolves the frame instead of fataling on a missing transform.
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
    fd, path = tempfile.mkstemp(prefix="isaac_v2_nav_rgb_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _rtabmap_parameters(use_sim_time, queue_size, qos):
    """RTAB-Map RGB-D params, VERBATIM from ugv_slam/rtabmap_rgbd.launch.py.

    Only use_sim_time (Isaac /clock) and qos (Isaac best-effort sensors) are
    sim-necessary additions, so the sim localizes exactly as the hardware.
    """
    return {
        "frame_id": "base_footprint",
        "queue_size": queue_size,
        "subscribe_rgb": True,
        "subscribe_depth": True,
        "subscribe_scan": True,
        "subscribe_odom_info": False,
        "approx_sync": True,
        "Rtabmap/DetectionRate": "3.5",
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
    queue_size = int(context.launch_configurations.get("queue_size", "20"))
    qos = int(context.launch_configurations.get("qos", "2"))
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")

    localplan = context.launch_configurations.get("use_localplan", "rpp")
    if localplan not in _LOCALPLAN_CHOICES:
        raise RuntimeError(
            f"use_localplan must be one of {_LOCALPLAN_CHOICES}, got '{localplan}'"
        )
    params_file = _require_file(
        os.path.join(ugv_nav_dir, "params", f"{localplan}.yaml"),
        f"ugv_nav {localplan} params",
    )
    # v2-specific 3D RViz view: uses built-in RViz displays only, points at the
    # sim namespaced topics, and avoids rtabmap_rviz_plugins because that plugin
    # is not installed in the current Humble environment.
    rviz_config = _launch_value(
        context, "rviz_config", DEFAULT_RVIZ_CONFIG
    )
    _require_file(rviz_config, "ugv_nav RViz config")
    params_file = _write_params_without_filters(params_file)

    actions = []

    # robot_state_publisher runs ONLY to publish /robot_description for the RViz
    # RobotModel display; its /tf + /tf_static are remapped to dead topics so it
    # never competes with the baked USD's TF (Isaac owns the tree).
    actions.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time,
                     "robot_description": _load_robot_description()}],
        remappings=[("/tf", "/rsp_tf_unused"),
                    ("/tf_static", "/rsp_tf_static_unused")],
        output="screen",
    ))

    # RTAB-Map RGB-D — the real rover's node, verbatim (rtabmap_rgbd.launch.py).
    # rtabmap subscribes to RGB + depth (via RTABMAP_REMAPPINGS -> /ugv_sim/oak/*)
    # + /scan, uses Isaac's odom/TF as the guess, and publishes /map + map->odom.
    rtabmap_params = _rtabmap_parameters(use_sim_time, queue_size, qos)
    # Named database so a mapping run and a later localization run share ONE known
    # file (Experiment A). Empty (default) = RTAB-Map's own ~/.ros/rtabmap.db, i.e.
    # unchanged behavior. Point both modes at e.g.
    # experiments/maps/warehouse/rtabmap.db: map once (localization:=false, which
    # passes -d to start fresh and saves on exit), then localize:=true reuses it.
    database_path = _launch_value(context, "database_path", "")
    if database_path:
        rtabmap_params = {**rtabmap_params, "database_path": database_path}

    # The sim publishes sensor DATA under the `ugv_sim` namespace (so it can run
    # next to the real rover). rtabmap's RGB/depth are pointed at /ugv_sim/oak/*
    # explicitly above; its /scan and Nav2's /odom are redirected here with SetRemap
    # (propagates into the navigation include). /tf, /tf_static, /clock and /cmd_vel
    # stay plain (shared/global).
    actions.append(GroupAction([
        SetRemap("/scan", "/ugv_sim/scan"),
        SetRemap("/odom", "/ugv_sim/odom"),
        Node(  # SLAM (mapping) mode
            condition=LaunchConfigurationEquals("localization", "false"),
            package="rtabmap_slam", executable="rtabmap", output="screen",
            parameters=[rtabmap_params],
            remappings=RTABMAP_REMAPPINGS,
            respawn=use_respawn, respawn_delay=2.0,
            arguments=["-d"],  # delete previous ~/.ros/rtabmap.db
        ),
        Node(  # pure-localization mode (localize vs saved db)
            condition=LaunchConfigurationEquals("localization", "true"),
            package="rtabmap_slam", executable="rtabmap", output="screen",
            parameters=[rtabmap_params,
                        {"Mem/IncrementalMemory": "False",
                         "Mem/InitWMWithAllNodes": "True"}],
            remappings=RTABMAP_REMAPPINGS,
            respawn=use_respawn, respawn_delay=2.0,
        ),
        # The real rover's Nav2 navigation servers (planner/controller/bt/behaviors),
        # upstream source — RTAB-Map above supplies the map + map->odom.
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

    # A live colored RGB-D point cloud for RViz, generated from Isaac's RGB,
    # depth, and camera_info topics. This gives a 3D camera visualization without
    # depending on RTAB-Map's optional RViz plugin package.
    actions.append(Node(
        package="rtabmap_util",
        executable="point_cloud_xyzrgb",
        name="isaac_rgbd_cloud",
        parameters=[{"approx_sync": True, "qos": qos, "use_sim_time": use_sim_time}],
        remappings=[
            ("rgb/image", "/ugv_sim/oak/rgb/image_rect"),
            ("rgb/camera_info", "/ugv_sim/oak/rgb/camera_info"),
            ("depth/image", "/ugv_sim/oak/stereo/image_raw"),
            ("cloud", RGBD_CLOUD_TOPIC),
        ],
        output="screen",
    ))

    actions.append(Node(
        condition=IfCondition(use_rviz),
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    ))

    # Raw RGB-camera image in its own rqt_image_view window (on by default).
    # Disable with use_image_view:=false.
    actions.append(Node(
        condition=IfCondition(LaunchConfiguration("use_image_view")),
        package="rqt_image_view", executable="rqt_image_view",
        name="rgb_raw_image_view",
        arguments=["/ugv_sim/oak/rgb/image_rect"],
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
        DeclareLaunchArgument("localization", default_value="false",
                              description="Localize against a saved RTAB-Map db (true, "
                                          "Mem/IncrementalMemory:=False) instead of "
                                          "mapping live (false). Pair with database_path "
                                          "to reuse a specific map (Experiment A)"),
        DeclareLaunchArgument("database_path", default_value="",
                              description="RTAB-Map database file for BOTH mapping and "
                                          "localization (e.g. .../experiments/maps/"
                                          "warehouse/rtabmap.db). Empty = ~/.ros/rtabmap.db"),
        DeclareLaunchArgument("queue_size", default_value="20",
                              description="rtabmap input queue size (upstream default 20)"),
        DeclareLaunchArgument("qos", default_value="2",
                              description="QoS for rtabmap input sensor topics"),
        DeclareLaunchArgument("use_robot_state_publisher", default_value="false",
                              description="Run robot_state_publisher from the URDF. "
                                          "OFF by default: the baked USD owns all TF."),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz config; defaults to this v2 RGB-D view"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Automatically start the nav2 stack"),
        DeclareLaunchArgument("use_composition", default_value="False",
                              description="Use composed Nav2 bringup"),
        DeclareLaunchArgument("use_respawn", default_value="False",
                              description="Respawn crashed nodes (composition off only)"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Nav2 log level"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with the 3D navigation view"),
        DeclareLaunchArgument("use_image_view", default_value="true",
                              description="Open the raw RGB-camera image in a separate "
                                          "rqt_image_view window (needs ros-humble-rqt-image-view)"),
        OpaqueFunction(function=_launch_setup),
    ])
