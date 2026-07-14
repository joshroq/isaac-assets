"""Nav system: LIDAR-ONLY SLAM (slam_toolbox), mirroring the real rover.

This is an Isaac-Sim shim around the same `ugv_nav` Nav2 stack used by the real
WaveShare rover. It includes upstream
`ugv_nav/launch/nav_bringup/nav2_bringup.launch.py` with `use_slam:=True`, so
`navigation_launch.py`, `slam_launch.py`, and the selected
`ugv_nav/params/<localplan>.yaml` are shared with the real rover path. Only the
hardware bringup is replaced: Isaac publishes `/scan`, `/odom`, `odom ->
base_footprint`, and consumes `/cmd_vel`. The shim also runs
`robot_state_publisher` from the rover URDF so the static base and sensor frames
match the source workspace.

Equivalence to the real rover:
    real:  ros2 launch ugv_nav nav.launch.py use_slam:=true use_localplan:=teb
    isaac: ros2 launch isaac_nav_lidar.launch.py use_localplan:=teb
Both resolve to the same Nav2 params and upstream slam_toolbox launch source.

PREREQUISITES:
  * Build + source ugv_ws so `ugv_nav` and `ugv_slam` are on the ament path:
        cd <ugv_ws> && colcon build --packages-select ugv_nav ugv_slam --symlink-install
        source install/setup.bash
  * apt: ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
         (rpp/dwa/mppi controllers ship with navigation2; `teb` needs
          ros-humble-teb-local-planner, which is optional here.)

RUN (with Isaac running this system's run_small_warehouse_nav.py elsewhere):
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch <this folder>/isaac_nav_lidar.launch.py            # teb default
  ros2 launch <this folder>/isaac_nav_lidar.launch.py use_localplan:=rpp
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
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


def _write_params_for_enabled_filters(
    source_file: str,
    keepout_enabled: bool,
    speed_enabled: bool,
) -> str:
    """Write a temporary params file matching the optional filter launches.

    Upstream planner params list keepout/speed filters in the costmaps, while
    upstream nav2_bringup starts the matching filter-info servers only when the
    launch args are enabled. Removing disabled filter plugins avoids continuous
    "filter mask was not received" warnings without changing the selected local
    planner, SLAM, controller, smoother, behavior, or velocity params.
    """
    with open(source_file, "r") as f:
        data = yaml.safe_load(f)

    for outer, inner in (
        ("local_costmap", "local_costmap"),
        ("global_costmap", "global_costmap"),
    ):
        params = data.get(outer, {}).get(inner, {}).get("ros__parameters", {})
        if not keepout_enabled:
            _drop_filter(params, "keepout_filter")
        if not speed_enabled:
            _drop_filter(params, "speed_filter")

    if not keepout_enabled:
        data.pop("keepout_filter_mask_server", None)
        data.pop("keepout_costmap_filter_info_server", None)
    if not speed_enabled:
        data.pop("speed_filter_mask_server", None)
        data.pop("speed_costmap_filter_info_server", None)

    fd, path = tempfile.mkstemp(prefix="isaac_nav_lidar_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _launch_setup(context, *args, **kwargs):
    ugv_nav_dir = get_package_share_directory("ugv_nav")
    nav_bringup = os.path.join(
        ugv_nav_dir, "launch", "nav_bringup", "nav2_bringup.launch.py"
    )
    _require_file(nav_bringup, "ugv_nav nav2 bringup launch")

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_localization = LaunchConfiguration("use_localization")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    use_keepout_zones = LaunchConfiguration("use_keepout_zones")
    use_speed_zones = LaunchConfiguration("use_speed_zones")

    # Resolve the local planner -> params/<localplan>.yaml (same as upstream).
    localplan = context.launch_configurations.get("use_localplan", "teb")
    if localplan not in _LOCALPLAN_CHOICES:
        raise RuntimeError(
            f"use_localplan must be one of {_LOCALPLAN_CHOICES}, got '{localplan}'"
        )
    params_file = _require_file(
        os.path.join(ugv_nav_dir, "params", f"{localplan}.yaml"),
        f"ugv_nav {localplan} params",
    )
    keepout_enabled = _launch_bool(context, "use_keepout_zones")
    speed_enabled = _launch_bool(context, "use_speed_zones")

    map_yaml = _launch_value(
        context,
        "map", os.path.join(ugv_nav_dir, "maps", "map.yaml")
    )
    pbstream = _launch_value(
        context,
        "pbstream", os.path.join(ugv_nav_dir, "maps", "map.pbstream")
    )
    posegraph = _launch_value(
        context,
        "posegraph", os.path.join(ugv_nav_dir, "maps", "map")
    )
    keepout_mask = _launch_value(
        context,
        "keepout_mask", os.path.join(ugv_nav_dir, "maps", "mask.yaml")
    )
    speed_mask = _launch_value(context, "speed_mask", "")
    emcl2_params_file = _launch_value(
        context,
        "emcl2_params_file",
        os.path.join(ugv_nav_dir, "params", "emcl2_quick_start.param.yaml"),
    )
    slam_toolbox_params_file = _launch_value(
        context,
        "slam_toolbox_params_file",
        os.path.join(ugv_nav_dir, "params", "slam_toolbox_localization.yaml"),
    )
    rviz_config = _launch_value(
        context,
        "rviz_config", os.path.join(ugv_nav_dir, "rviz", "view_nav_2d.rviz")
    )

    # These are launch-time dependencies in upstream nav2_bringup even though
    # map_server is not started for use_slam:=True.
    _require_file(map_yaml, "ugv_nav map yaml")
    _require_file(params_file, "ugv_nav params")
    _require_file(rviz_config, "ugv_nav RViz config")
    if speed_enabled and not speed_mask:
        raise RuntimeError(
            "use_speed_zones:=True requires speed_mask:=/path/to/speed_mask.yaml"
        )
    params_file = _write_params_for_enabled_filters(
        params_file,
        keepout_enabled=keepout_enabled,
        speed_enabled=speed_enabled,
    )

    robot_description = _load_robot_description()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time,
                     "robot_description": robot_description}],
        output="screen",
    )

    # The real rover's Nav2 stack, verbatim: lidar SLAM branch (use_slam:=True).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_bringup),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "use_slam": "True",
            "use_sim_time": use_sim_time,
            "use_localization": use_localization,
            "params_file": params_file,
            "slam_toolbox_params_file": slam_toolbox_params_file,
            "map": map_yaml,
            "pbstream": pbstream,
            "posegraph": posegraph,
            "keepout_mask": keepout_mask,
            "speed_mask": speed_mask,
            "emcl2_params_file": emcl2_params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
            "use_keepout_zones": use_keepout_zones,
            "use_speed_zones": use_speed_zones,
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

    return [robot_state_publisher, nav2, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="",
                              description="Top-level namespace for the Nav2 stack"),
        DeclareLaunchArgument("use_namespace", default_value="false",
                              description="Apply namespace to the Nav2 stack"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use the /clock published by Isaac Sim"),
        DeclareLaunchArgument("use_localplan", default_value="teb",
                              description="Nav2 local planner: teb, dwa, rpp, mppi "
                                          "(teb needs ros-humble-teb-local-planner)"),
        DeclareLaunchArgument("use_localization", default_value="amcl",
                              description="Upstream argument retained for parity; "
                                          "ignored while use_slam is forced true"),
        DeclareLaunchArgument("map", default_value="",
                              description="Optional map yaml path; defaults to ugv_nav/maps/map.yaml"),
        DeclareLaunchArgument("pbstream", default_value="",
                              description="Optional Cartographer pbstream path"),
        DeclareLaunchArgument("posegraph", default_value="",
                              description="Optional slam_toolbox posegraph path prefix"),
        DeclareLaunchArgument("keepout_mask", default_value="",
                              description="Optional keepout mask yaml path"),
        DeclareLaunchArgument("speed_mask", default_value="",
                              description="Optional speed mask yaml path"),
        DeclareLaunchArgument("emcl2_params_file", default_value="",
                              description="Optional EMCL2 params path"),
        DeclareLaunchArgument("slam_toolbox_params_file", default_value="",
                              description="Optional upstream slam_toolbox params path"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="Optional RViz config path"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Automatically start the nav2 stack"),
        DeclareLaunchArgument("use_composition", default_value="True",
                              description="Use composed Nav2 bringup"),
        DeclareLaunchArgument("use_respawn", default_value="False",
                              description="Respawn crashed nodes (composition off only)"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Nav2 log level"),
        DeclareLaunchArgument("use_keepout_zones", default_value="False",
                              description="Start upstream keepout filter info servers"),
        DeclareLaunchArgument("use_speed_zones", default_value="False",
                              description="Start upstream speed filter info servers"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with the 2D navigation view"),
        OpaqueFunction(function=_launch_setup),
    ])
