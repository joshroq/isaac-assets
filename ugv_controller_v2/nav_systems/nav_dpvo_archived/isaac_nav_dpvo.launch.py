"""ARCHIVED / INACTIVE — DPVO (Deep Patch Visual Odometry), ugv_controller_v2.

    ┌─────────────────────────────────────────────────────────────────────────┐
    │ NOT A PART 1 STACK. DPVO (learned monocular visual odometry) is paused    │
    │ for the Part 1 SLAM/Nav2 fidelity baseline. It is preserved here for the  │
    │ later policy-training/generalization / visual-odometry comparison work —  │
    │ nothing is deleted (DPVO source, the conda env, and this bridge all stay).│
    │ See ugv_controller_v2/README.md (Part 1 stacks) for the active list.      │
    └─────────────────────────────────────────────────────────────────────────┘

Nav system: DPVO (Deep Patch Visual Odometry) for the BAKED-GRAPH rover (ugv_controller_v2).

The learned/monocular visual-SLAM sibling of nav_orbslam3. map->odom comes from DPVO run
DIRECTLY from the source checkout in projects/slam_source/DPVO-main (dpvo_bridge_node.py
imports the `dpvo` package, loads the trained dpvo.pth network, and runs the same
`DPVO(cfg, net, ht, wd, viz=...)` / `slam(t, image, intrinsics)` loop as DPVO's demo.py).

  * LOCALIZER : DPVO, MONOCULAR. It tracks dense image *patches* with a trained recurrent
    network (no hand-crafted features, no loop closure by default), consuming only
    /ugv_sim/oak/rgb/image_rect + camera_info. The bridge reads DPVO's camera pose and
    broadcasts map->odom — the role slam_toolbox plays in nav_lidar. Neither the depth
    image nor the lidar is used to localize.
  * SCALE : monocular VO is only defined up to a global scale, so the bridge recovers the
    metric scale online by matching DPVO's travelled distance against Isaac's /odom. This
    is the one thing nav_orbslam3 gets for free (RGB-D is metric).
  * MAP : DPVO emits a SPARSE patch-point cloud, not an occupancy grid, so the shared
    common/visual_map_builder.py projects /dpvo/map_points into the /map Nav2's global
    static layer needs — exactly as it does for nav_orbslam3.
  * NAV2 : upstream `ugv_nav/launch/nav_bringup/navigation_launch.py`, same as nav_rgb and
    nav_orbslam3. DPVO supplies map->odom and the map builder supplies /map, so no
    slam_toolbox/amcl runs here.

As in the other visual systems, the 2D lidar still feeds the Nav2 costmap obstacle layer
exactly as the upstream params define it — it just never contributes to localization. So
the map->odom SOURCE remains the only controlled variable across the systems.

VISUALIZATION — two views at once:
  * DPVO's OWN native viewer (DPViewer, from DPVO-main/DPViewer): a Pangolin window with
    the live image, the tracked patches and the 3D trajectory + patch cloud — what
    `demo.py --viz` shows. It is created inside DPVO itself on the live CUDA tensors
    (viz:=true, the default).
  * RViz: the Nav2 world — costmaps, plans, the projected /map, and /dpvo/map_points.
Set viz:=false for headless runs (or if DPViewer was not built), use_rviz:=false to drop
RViz.

Contrast the siblings:
    nav_lidar    : slam_toolbox on /scan (2D lidar SLAM), 2D RViz.
    nav_rgb      : RTAB-Map RGB-D (RGB + depth + scan), dense colored cloud, 3D RViz.
    nav_depth    : lidar SLAM + depth obstacle cloud, no RGB, 3D RViz.
    nav_orbslam3 : ORB-SLAM3 RGB-D visual SLAM — features, metric, loop closure.
    nav_dpvo     : DPVO monocular visual ODOMETRY — learned patches, up-to-scale, drifts. THIS.

CONTRACT THIS LAUNCH EXPECTS ISAAC TO PUBLISH (see ugv_controller_v2/USD_CONTRACT.md)
    subscribes : /cmd_vel
    publishes  : /clock, /odom, /scan, /tf + /tf_static, and the camera RGB on
                 /ugv_sim/oak/rgb/image_rect with intrinsics on
                 /ugv_sim/oak/rgb/camera_info (RGB frame Camera_OmniVision_OV9782_Color,
                 present in /tf — Isaac owns the TF tree, so this launch publishes NO
                 static camera transform). Depth is NOT used.
    DPVO then provides map->odom, visual_map_builder provides /map, Nav2 outputs /cmd_vel.

PREREQUISITES
  * The `dpvo` CONDA ENV (torch 2.7/cu128 + DPVO's compiled CUDA extensions + dpviewer).
    It is python 3.10 — the same minor as ROS Humble — so rclpy/tf2_ros import straight
    into it and the bridge runs as one ordinary ROS node there. Verify it any time with
    ./setup.sh in this folder (it checks the env, the weights and the viewer; it does not
    rebuild anything).
  * Build + source ugv_ws so `ugv_nav` is on the ament path.
  * apt: ros-humble-navigation2 ros-humble-nav2-bringup.

RUN (Isaac running the baked rover elsewhere):
  navlaunch -v2 -dpvo                           # or, by hand:
  source /opt/ros/humble/setup.bash
  source <ugv_ws>/install/setup.bash
  source ~/.local/bin/ros2ddsenv
  ros2 launch <this file>                       # DPViewer + RViz
  ros2 launch <this file> viz:=false            # RViz only
  ros2 launch <this file> use_localplan:=teb    # planner: teb|dwa|rpp|mppi
  ros2 launch <this file> stride:=2             # process every 2nd frame (demo default)
  ros2 launch <this file> scale:=1.0            # pin the metric scale instead of estimating
"""

import os
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
BRIDGE_NODE = str(THIS_DIR / "dpvo_bridge_node.py")
# DPVO needs torch + CUDA + its compiled extensions, which system python does not have.
# They live in the `dpvo` conda env (python 3.10 + torch 2.7/cu128 + dpviewer). That env
# is python 3.10 — the SAME minor as ROS Humble — so /opt/ros's rclpy, tf2_ros and the
# message packages import straight into it once ROS is sourced (verified). That means the
# bridge runs as ONE ordinary ROS node in that interpreter: venv torch + system rclpy,
# no IPC bridge, no second process. setup.sh checks all of this.
DPVO_PYTHON = "/home/joshroq/miniforge3/envs/dpvo/bin/python"
# Shared with nav_orbslam3: both visual systems emit a sparse cloud, not an occupancy grid.
MAP_BUILDER = str(THIS_DIR.parent.parent / "common" / "visual_map_builder.py")
DEFAULT_RVIZ_CONFIG = str(THIS_DIR / "isaac_dpvo_nav.rviz")

DPVO_PATH = "/home/joshroq/projects/slam_source/DPVO-main"

# The sim publishes sensor DATA under the `ugv_sim` namespace; DPVO reads it directly.
RGB_TOPIC = "/ugv_sim/oak/rgb/image_rect"
CAMERA_INFO_TOPIC = "/ugv_sim/oak/rgb/camera_info"
# DPVO's sparse patch points (frame=map) -> visual_map_builder -> /map.
MAP_POINTS_TOPIC = "/dpvo/map_points"

# The baked USD's RGB optical frame (USD_CONTRACT.md). It is on /tf, so no static
# transform publisher is needed here — Isaac owns the tree.
DEFAULT_CAMERA_FRAME = "Camera_OmniVision_OV9782_Color"


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
    navigation_launch), avoiding continuous "filter mask was not received" warnings
    without touching the selected planner/controller/behavior params."""
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
    fd, path = tempfile.mkstemp(prefix="isaac_v2_nav_dpvo_", suffix=".yaml")
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

    localplan = context.launch_configurations.get("use_localplan", "rpp")
    if localplan not in _LOCALPLAN_CHOICES:
        raise RuntimeError(
            f"use_localplan must be one of {_LOCALPLAN_CHOICES}, got '{localplan}'"
        )
    params_file = _require_file(
        os.path.join(ugv_nav_dir, "params", f"{localplan}.yaml"),
        f"ugv_nav {localplan} params",
    )
    rviz_config = _launch_value(context, "rviz_config", DEFAULT_RVIZ_CONFIG)
    _require_file(rviz_config, "nav_dpvo RViz config")
    _require_file(MAP_BUILDER, "visual_map_builder")
    _require_file(BRIDGE_NODE, "dpvo_bridge_node")

    # The bridge must run in the `dpvo` conda env (torch + the compiled DPVO CUDA
    # extensions + dpviewer); system python has no torch. Fail with the fix, not an
    # ImportError 200 lines deep.
    python_exe = _launch_value(context, "python_executable", DPVO_PYTHON)
    if not os.path.isfile(python_exe):
        raise RuntimeError(
            f"nav_dpvo python interpreter not found: {python_exe}\n"
            f"DPVO needs torch + CUDA + its compiled extensions (the `dpvo` conda env).\n"
            f"Check the environment with:\n"
            f"    {THIS_DIR / 'setup.sh'}\n"
            f"(or pass python_executable:=/path/to/python if you keep DPVO elsewhere)."
        )

    dpvo_path = _launch_value(context, "dpvo_path", DPVO_PATH)
    if not os.path.isdir(dpvo_path):
        raise RuntimeError(f"DPVO source not found: {dpvo_path}")

    # Resolve the numeric args to real Python scalars: launch would YAML-evaluate the
    # substituted string, so `scale:=1` would arrive as an int where the node declared a
    # double and rclpy would throw. Casting here makes every spelling of a number work.
    stride = int(float(_launch_value(context, "stride", "1")))
    warmup = int(float(_launch_value(context, "warmup_frames", "8")))
    scale = float(_launch_value(context, "scale", "0.0"))
    min_scale_travel = float(_launch_value(context, "min_scale_travel", "0.5"))
    map_resolution = float(_launch_value(context, "map_resolution", "0.05"))
    map_z_min = float(_launch_value(context, "map_obstacle_z_min", "0.08"))
    map_z_max = float(_launch_value(context, "map_obstacle_z_max", "1.5"))
    map_max_ray = float(_launch_value(context, "map_max_ray_range", "8.0"))
    sensor_frame = _launch_value(context, "camera_optical_frame", DEFAULT_CAMERA_FRAME)
    viz = "true" if _launch_bool(context, "viz", True) else "false"
    invert_pose = "true" if _launch_bool(context, "invert_pose", False) else "false"

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

    # ── DPVO bridge — THE localizer of this system ──────────────────────────────
    # Run as a bare process (not a launch_ros Node) so we can point it at the venv
    # interpreter that has torch. It reads only the camera + /tf, so it needs no /scan or
    # /odom remap and stays outside the SetRemap group below. viz:=true opens DPVO's own
    # Pangolin viewer (DPViewer) next to RViz.
    actions.append(ExecuteProcess(
        cmd=[python_exe, BRIDGE_NODE, "--ros-args",
             "-p", ["use_sim_time:=", use_sim_time],
             "-p", f"dpvo_path:={dpvo_path}",
             "-p", f"network:={_launch_value(context, 'network', 'dpvo.pth')}",
             "-p", f"config:={_launch_value(context, 'config', 'config/default.yaml')}",
             "-p", f"rgb_topic:={RGB_TOPIC}",
             "-p", f"info_topic:={CAMERA_INFO_TOPIC}",
             "-p", f"cloud_topic:={MAP_POINTS_TOPIC}",
             "-p", f"camera_optical_frame:={sensor_frame}",
             "-p", "map_frame:=map",
             "-p", "odom_frame:=odom",
             "-p", "base_frame:=base_footprint",
             "-p", f"stride:={stride}",
             "-p", f"warmup_frames:={warmup}",
             "-p", f"scale:={scale}",
             "-p", f"min_scale_travel:={min_scale_travel}",
             "-p", f"invert_pose:={invert_pose}",
             "-p", f"viz:={viz}"],
        output="screen",
    ))

    # ── Sparse patch points -> /map (the Nav2 global static layer) ───────────────
    # DPVO has no occupancy grid of its own; this projects its cloud into one. Shared with
    # nav_orbslam3 — plain system python is fine here (numpy + rclpy only, no torch).
    actions.append(ExecuteProcess(
        cmd=["python3", MAP_BUILDER, "--ros-args",
             "-p", ["use_sim_time:=", use_sim_time],
             "-p", f"sensor_frame:={sensor_frame}",
             "-p", f"resolution:={map_resolution}",
             "-p", f"obstacle_z_min:={map_z_min}",
             "-p", f"obstacle_z_max:={map_z_max}",
             "-p", f"max_ray_range:={map_max_ray}",
             "-r", f"cloud:={MAP_POINTS_TOPIC}"],
        output="screen",
    ))

    # ── Nav2 ────────────────────────────────────────────────────────────────────
    # DPVO supplies map->odom, the map builder supplies /map. The sim's data topics are
    # namespaced, so redirect /scan (costmap obstacle layer + collision monitor) and /odom
    # (controller) to the sim names; this propagates into the include. /tf, /tf_static,
    # /clock and /cmd_vel stay plain (shared/global).
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
        DeclareLaunchArgument("viz", default_value="true",
                              description="Show DPVO's OWN native visualization (DPViewer: "
                                          "Pangolin window with the live image, tracked "
                                          "patches and 3D trajectory), alongside RViz. "
                                          "Needs a display + DPViewer built by setup.sh"),
        DeclareLaunchArgument("python_executable", default_value="",
                              description="Interpreter for the DPVO bridge; defaults to the "
                                          "venv setup.sh builds here (torch + CUDA)"),
        DeclareLaunchArgument("dpvo_path", default_value=DPVO_PATH,
                              description="DPVO source checkout"),
        DeclareLaunchArgument("network", default_value="dpvo.pth",
                              description="Trained DPVO weights (relative to dpvo_path)"),
        DeclareLaunchArgument("config", default_value="config/default.yaml",
                              description="DPVO config (default.yaml or fast.yaml)"),
        DeclareLaunchArgument("stride", default_value="1",
                              description="Process every Nth RGB frame. Isaac's camera is "
                                          "already slow, so 1 by default (demo.py uses 2)"),
        DeclareLaunchArgument("warmup_frames", default_value="8",
                              description="Frames DPVO needs before its pose is trusted"),
        DeclareLaunchArgument("scale", default_value="0.0",
                              description="Metric scale for the monocular trajectory. "
                                          "0 = estimate it online against Isaac's /odom"),
        DeclareLaunchArgument("min_scale_travel", default_value="0.5",
                              description="Metres of odom travel before the online scale "
                                          "estimate is trusted"),
        DeclareLaunchArgument("invert_pose", default_value="false",
                              description="Flip the pose if the trajectory comes out "
                                          "mirrored — never fix that in the URDF"),
        DeclareLaunchArgument("camera_optical_frame", default_value=DEFAULT_CAMERA_FRAME,
                              description="RGB optical frame on /tf"),
        DeclareLaunchArgument("map_resolution", default_value="0.05",
                              description="Cell size (m) of the /map built from DPVO's points"),
        DeclareLaunchArgument("map_obstacle_z_min", default_value="0.08",
                              description="Min height (m) of a point counted as an obstacle"),
        DeclareLaunchArgument("map_obstacle_z_max", default_value="1.5",
                              description="Max height (m) of a point counted as an obstacle"),
        DeclareLaunchArgument("map_max_ray_range", default_value="8.0",
                              description="Max range (m) a point is trusted for the grid"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz config; defaults to this v2 DPVO view"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Automatically start the nav2 stack"),
        DeclareLaunchArgument("use_composition", default_value="False",
                              description="Use composed Nav2 bringup"),
        DeclareLaunchArgument("use_respawn", default_value="False",
                              description="Respawn crashed nodes (composition off only)"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Nav2 log level"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with the 3D DPVO view"),
        OpaqueFunction(function=_launch_setup),
    ])
