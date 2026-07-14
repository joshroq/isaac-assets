"""Nav system: DEPTH + LIDAR fusion (NO RGB) for the BAKED-GRAPH rover (v2).

The appearance-independent sibling of nav_rgb. It navigates with the depth camera
AND the lidar, but never the RGB image:

  * MAP + map->odom : the real rover's LIDAR SLAM (slam_toolbox), exactly the
    nav_lidar path — includes upstream `ugv_nav/.../nav2_bringup.launch.py` with
    `use_slam:=True`, so navigation_launch/slam_launch and the selected
    `ugv_nav/params/<localplan>.yaml` are shared verbatim with the real rover.
    The 2D lidar is the reliable global map source.
  * DEPTH        : Isaac's depth image is turned into an XYZ point cloud by
    `depth_image_proc/point_cloud_xyz` (depth + camera_info -> points, NO RGB).
    That cloud is (a) plotted in the 3D RViz view (the depth counterpart of
    nav_rgb's RGB-D cloud) and (b) added to the Nav2 local+global costmaps and the
    collision monitor as a PointCloud2 obstacle source, so obstacle avoidance
    fuses lidar + depth (depth catches obstacles the 2D scan plane misses).

Contrast the siblings:
    nav_lidar : lidar only (slam_toolbox), 2D RViz.
    nav_rgb   : RTAB-Map RGB-D (RGB + depth + scan), colored cloud, 3D RViz.
    nav_depth : lidar SLAM + depth obstacle cloud, uncolored (height-shaded) cloud,
                3D RViz. RGB is never subscribed.

CONTRACT THIS LAUNCH EXPECTS ISAAC TO PUBLISH (see ugv_controller_v2/USD_CONTRACT.md)
    subscribes : /cmd_vel
    publishes  : /clock, /odom, /scan, /tf + /tf_static, and the depth image on
                 /ugv_sim/oak/stereo/image_raw with intrinsics on
                 /ugv_sim/oak/rgb/camera_info (depth frame = Camera_Pseudo_Depth,
                 present in /tf). RGB is NOT used.

PREREQUISITES
  * Build + source ugv_ws so `ugv_nav`/`ugv_slam` are on the ament path:
        cd <ugv_ws> && colcon build --packages-select ugv_nav ugv_slam --symlink-install
        source install/setup.bash
  * apt: ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
         ros-humble-depth-image-proc  (teb also needs ros-humble-teb-local-planner)

RUN (Isaac running this system's rover elsewhere):
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  source ~/.local/bin/ros2ddsenv
  ros2 launch <this file>                                  # teb default, 3D RViz
  ros2 launch <this file> use_localplan:=rpp
  ros2 launch <this file> use_robot_state_publisher:=true  # USD-iteration fallback
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
from launch_ros.actions import Node, SetRemap

# The URDF lives in the (unbuilt) ugv_ws source tree; read it by absolute path and
# rewrite mesh URIs to file:// so robot_state_publisher needs no install. We use the
# Isaac-matched URDF whose link names equal the frames the baked USD publishes on /tf.
UGV_WS_SRC = "/home/joshroq/projects/isaac-assets/ugv_ws/src/ugv_main"
UGV_DESCRIPTION_SRC = os.path.join(UGV_WS_SRC, "ugv_description")
URDF_PATH = os.path.join(UGV_DESCRIPTION_SRC, "urdf", "ugv_rover_isaac_fixed.urdf")

_LOCALPLAN_CHOICES = ("teb", "dwa", "rpp", "mppi")

# Isaac's depth image + shared intrinsics (namespaced `ugv_sim`; see USD_CONTRACT).
DEPTH_IMAGE_TOPIC = "/ugv_sim/oak/stereo/image_raw"
DEPTH_INFO_TOPIC = "/ugv_sim/oak/rgb/camera_info"
# The XYZ cloud we synthesize from depth (no RGB). Plotted in RViz + fed to costmaps.
DEPTH_CLOUD_TOPIC = "/ugv_sim/oak/depth/points"
DEFAULT_RVIZ_CONFIG = str(Path(__file__).with_name("isaac_depth_nav.rviz"))


def _load_robot_description() -> str:
    if not os.path.isfile(URDF_PATH):
        raise RuntimeError(f"Isaac rover URDF not found: {URDF_PATH}")
    with open(URDF_PATH, "r") as f:
        robot_description = f.read()
    desc_root = "file://" + UGV_DESCRIPTION_SRC.rstrip("/") + "/"
    robot_description = (
        robot_description
        .replace("../meshes/", desc_root + "meshes/")
        .replace("package://ugv_description/", desc_root)
        .replace("front_camera_link.stl", "3d_camera_link.stl")
    )
    # base_imu_link kept: the baked USD publishes base_link->base_imu_link on
    # /tf_static, so the RViz RobotModel resolves it (see nav_lidar/nav_rgb).
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


def _add_depth_obstacle_source(params: dict, cloud_topic: str,
                               min_h: float, max_h: float,
                               max_range: float) -> None:
    """Add the depth XYZ cloud as a PointCloud2 marking/clearing source to a
    costmap obstacle_layer, keeping the existing scan source. Floor points are
    excluded with min_obstacle_height so the ground is not marked lethal."""
    obstacle_layer = params.get("obstacle_layer")
    if not isinstance(obstacle_layer, dict):
        return
    sources = obstacle_layer.get("observation_sources", "scan")
    names = sources.split() if isinstance(sources, str) else list(sources)
    if "depth_cloud" not in names:
        names.append("depth_cloud")
    obstacle_layer["observation_sources"] = " ".join(names)
    obstacle_layer["depth_cloud"] = {
        "topic": cloud_topic,
        "data_type": "PointCloud2",
        "marking": True,
        "clearing": True,
        "min_obstacle_height": min_h,
        "max_obstacle_height": max_h,
        "obstacle_max_range": max_range,
        "obstacle_min_range": 0.0,
        "raytrace_max_range": max_range + 0.5,
        "raytrace_min_range": 0.0,
    }


def _prepare_nav_params(context, source_file: str) -> str:
    """Rewrite the upstream Nav2 params for a depth+lidar, filter-free run.

    Edits (none touch the selected planner/controller/smoother/behavior params):
      1. Strip keepout/speed filters (their info servers don't run under this
         bringup) unless explicitly enabled.
      2. Add the depth XYZ cloud as a PointCloud2 obstacle source to the local +
         global costmap obstacle_layer (alongside the lidar scan).
      3. Enable the collision_monitor's pointcloud source on the depth cloud.
    """
    keepout_enabled = _launch_bool(context, "use_keepout_zones")
    speed_enabled = _launch_bool(context, "use_speed_zones")
    min_h = float(_launch_value(context, "depth_min_height", "0.08"))
    max_h = float(_launch_value(context, "depth_max_height", "1.5"))
    max_range = float(_launch_value(context, "depth_max_range", "4.0"))

    with open(source_file, "r") as f:
        data = yaml.safe_load(f)

    for outer, inner in (("local_costmap", "local_costmap"),
                         ("global_costmap", "global_costmap")):
        params = data.get(outer, {}).get(inner, {}).get("ros__parameters", {})
        if not keepout_enabled:
            _drop_filter(params, "keepout_filter")
        if not speed_enabled:
            _drop_filter(params, "speed_filter")
        _add_depth_obstacle_source(params, DEPTH_CLOUD_TOPIC, min_h, max_h, max_range)

    if not keepout_enabled:
        data.pop("keepout_filter_mask_server", None)
        data.pop("keepout_costmap_filter_info_server", None)
    if not speed_enabled:
        data.pop("speed_filter_mask_server", None)
        data.pop("speed_costmap_filter_info_server", None)

    # Collision monitor: point its (already-declared) pointcloud source at the depth
    # cloud and add it to the active sources, so emergency stop also uses depth.
    cm = data.get("collision_monitor", {}).get("ros__parameters", {})
    pc = cm.get("pointcloud")
    if isinstance(pc, dict):
        pc["topic"] = DEPTH_CLOUD_TOPIC
        pc["enabled"] = True
        pc["min_height"] = min_h
        pc["max_height"] = max_h
        srcs = cm.get("observation_sources", ["scan"])
        if isinstance(srcs, str):
            srcs = srcs.split()
        if "pointcloud" not in srcs:
            srcs.append("pointcloud")
        cm["observation_sources"] = srcs

    fd, path = tempfile.mkstemp(prefix="isaac_v2_nav_depth_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def _launch_setup(context, *args, **kwargs):
    ugv_nav_dir = get_package_share_directory("ugv_nav")
    nav_bringup = _require_file(
        os.path.join(ugv_nav_dir, "launch", "nav_bringup", "nav2_bringup.launch.py"),
        "ugv_nav nav2 bringup launch",
    )

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

    map_yaml = _launch_value(context, "map", os.path.join(ugv_nav_dir, "maps", "map.yaml"))
    pbstream = _launch_value(context, "pbstream", os.path.join(ugv_nav_dir, "maps", "map.pbstream"))
    posegraph = _launch_value(context, "posegraph", os.path.join(ugv_nav_dir, "maps", "map"))
    keepout_mask = _launch_value(context, "keepout_mask", os.path.join(ugv_nav_dir, "maps", "mask.yaml"))
    speed_mask = _launch_value(context, "speed_mask", "")
    emcl2_params_file = _launch_value(
        context, "emcl2_params_file",
        os.path.join(ugv_nav_dir, "params", "emcl2_quick_start.param.yaml"),
    )
    slam_toolbox_params_file = _launch_value(
        context, "slam_toolbox_params_file",
        os.path.join(ugv_nav_dir, "params", "slam_toolbox_localization.yaml"),
    )
    rviz_config = _launch_value(context, "rviz_config", DEFAULT_RVIZ_CONFIG)

    _require_file(map_yaml, "ugv_nav map yaml")
    _require_file(rviz_config, "nav_depth RViz config")
    if _launch_bool(context, "use_speed_zones") and not speed_mask:
        raise RuntimeError(
            "use_speed_zones:=True requires speed_mask:=/path/to/speed_mask.yaml"
        )
    params_file = _prepare_nav_params(context, params_file)

    actions = []

    # robot_state_publisher runs ONLY to publish /robot_description for the RViz
    # RobotModel; its /tf + /tf_static are remapped to dead topics so it never
    # competes with the baked USD's TF (Isaac owns the tree).
    actions.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time,
                     "robot_description": _load_robot_description()}],
        remappings=[("/tf", "/rsp_tf_unused"),
                    ("/tf_static", "/rsp_tf_static_unused")],
        output="screen",
    ))

    # Depth image -> XYZ cloud (NO RGB). depth_image_proc/point_cloud_xyz takes the
    # depth image + camera_info and outputs points in the depth optical frame
    # (Camera_Pseudo_Depth, present in /tf). This is both the RViz visualization and
    # the costmap obstacle source.
    # queue_size is large so the ~21 Hz camera_info buffer reliably finds the
    # matching stamp for each ~2 Hz depth frame (they share Isaac's sim clock).
    actions.append(Node(
        package="depth_image_proc", executable="point_cloud_xyz_node",
        name="isaac_depth_cloud",
        parameters=[{"use_sim_time": use_sim_time, "queue_size": queue_size}],
        remappings=[
            ("image_rect", DEPTH_IMAGE_TOPIC),
            ("camera_info", DEPTH_INFO_TOPIC),
            ("points", DEPTH_CLOUD_TOPIC),
        ],
        output="screen",
    ))

    # The real rover's Nav2 stack, LIDAR SLAM branch (use_slam:=True) — identical to
    # nav_lidar. Sim data topics are namespaced, so redirect just /scan and /odom to
    # the sim's names (propagates into the include). /tf, /tf_static, /clock, /cmd_vel
    # stay plain. The depth cloud source added to the params is already on its final
    # topic, so it needs no remap.
    actions.append(GroupAction([
        SetRemap("/scan", "/ugv_sim/scan"),
        SetRemap("/odom", "/ugv_sim/odom"),
        IncludeLaunchDescription(
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
                              description="Upstream arg retained for parity; "
                                          "ignored while use_slam is forced true"),
        DeclareLaunchArgument("use_robot_state_publisher", default_value="false",
                              description="Run robot_state_publisher from the URDF. "
                                          "OFF by default: the baked USD owns all TF."),
        DeclareLaunchArgument("queue_size", default_value="30",
                              description="message_filters queue for depth+camera_info "
                                          "sync in the depth->cloud node"),
        DeclareLaunchArgument("depth_min_height", default_value="0.08",
                              description="Min height (m) of depth points marked as "
                                          "obstacles (drops the floor)"),
        DeclareLaunchArgument("depth_max_height", default_value="1.5",
                              description="Max height (m) of depth points marked as obstacles"),
        DeclareLaunchArgument("depth_max_range", default_value="4.0",
                              description="Max range (m) for depth obstacle marking "
                                          "(OAK depth is noisy far out)"),
        DeclareLaunchArgument("map", default_value="",
                              description="Optional map yaml; defaults to ugv_nav/maps/map.yaml"),
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
                              description="RViz config; defaults to this v2 depth view"),
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
                              description="Launch RViz with the 3D depth navigation view"),
        OpaqueFunction(function=_launch_setup),
    ])
