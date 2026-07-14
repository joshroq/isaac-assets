"""
Nav system: DPVO monocular VISUAL ODOMETRY localization.

map->odom comes from DPVO (Deep Patch Visual Odometry) run DIRECTLY from
projects/slam_source/DPVO-main. The dpvo_bridge_node feeds the front camera's RGB
image (/oak/rgb/image_rect, the RSD455 emulating the real OAK-D-Lite) to DPVO,
reads its camera trajectory, recovers the monocular scale online against Isaac's
/odom, and publishes the map->odom transform — the exact role slam_toolbox plays in
nav_lidar and RTAB-Map plays in nav_rgb. DPVO is a learned, monocular, feature-free
(patch-based) visual odometry, so this is the appearance-and-motion-dependent
system with NO depth and NO lidar in the loop.

Because DPVO emits only a sparse patch cloud (not a Nav2 occupancy grid), the
shared common/visual_map_builder.py projects /dpvo/map_points into the /map the Nav2
global static layer needs. Everything else (Nav2 params, costmaps, RViz) is the
universal base — only the map->odom SOURCE differs, which is the controlled variable
for the thesis.

This launch = the universal base bringup (common/nav2_bringup_base.launch.py) +
the camera optical-frame TF + the DPVO bridge + the shared visual map builder.

The matching Isaac run script publishes the /oak topics via
depth_camera_graph.setup_depth_camera (shared module in common/). DPVO uses only the
RGB image + camera_info; the depth stream is published but ignored.

PREREQUISITES:
  apt : ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher
  DPVO: `pip install .` in projects/slam_source/DPVO-main (needs torch + CUDA) and
        the trained weights DPVO-main/dpvo.pth (download_models_and_data.sh).
  The python that runs the DPVO bridge must have BOTH rclpy and the dpvo package
  importable — pass it via python_exec:=/path/to/that/python if it is not `python3`.

RUN (with Isaac running this system's run_small_warehouse_nav.py elsewhere):
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch <this folder>/isaac_nav_dpvo.launch.py
"""

import importlib.util
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# common/ is two levels up from this system folder (.../common/nav_systems/nav_dpvo/).
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
COMMON_DIR = os.path.dirname(os.path.dirname(THIS_DIR))
BASE_LAUNCH = os.path.join(COMMON_DIR, 'nav2_bringup_base.launch.py')
DPVO_BRIDGE = os.path.join(THIS_DIR, 'dpvo_bridge_node.py')
MAP_BUILDER = os.path.join(COMMON_DIR, 'visual_map_builder.py')

# Read the canonical Nav2 params path from the base launch (single source of truth).
_spec = importlib.util.spec_from_file_location('nav2_bringup_base', BASE_LAUNCH)
_base_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_base_mod)
DEFAULT_PARAMS = _base_mod.DEFAULT_PARAMS


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    python_exec = LaunchConfiguration('python_exec')
    dpvo_path = LaunchConfiguration('dpvo_path')

    declare_cmds = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the /clock published by Isaac Sim'),
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS,
                              description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically start the nav2 stack'),
        DeclareLaunchArgument('use_respawn', default_value='False',
                              description='Respawn crashed nodes'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz with the 2D navigation view'),
        DeclareLaunchArgument('python_exec', default_value='python3',
                              description='Python interpreter that has BOTH rclpy and '
                                          'the dpvo package importable (torch+CUDA)'),
        DeclareLaunchArgument('dpvo_path',
                              default_value='/home/joshroq/projects/slam_source/DPVO-main',
                              description='Path to the DPVO source checkout'),
    ]

    base_args = {'use_sim_time': use_sim_time,
                 'params_file': params_file,
                 'use_rviz': use_rviz,
                 'autostart': autostart,
                 'use_respawn': use_respawn}

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(BASE_LAUNCH),
        launch_arguments=base_args.items())

    # ── Camera optical-frame TF ─────────────────────────────────────────────────
    # Isaac publishes the RGB image in an optical frame; supply the static TF
    # 3d_camera_link -> oak_rgb_camera_optical_frame (REP-103 body->optical:
    # rpy = -pi/2, 0, -pi/2) so DPVO's camera pose lands on the robot. Identical to
    # the RTAB-Map camera launches (same camera, same frames).
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_rgb_optical_tf',
        arguments=['--frame-id', '3d_camera_link',
                   '--child-frame-id', 'oak_rgb_camera_optical_frame',
                   '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708'],
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=use_respawn, respawn_delay=2.0,
        output='screen')

    # ── DPVO bridge: /oak RGB -> map->odom TF + /dpvo/map_points ─────────────────
    # Launched via ExecuteProcess (build-free, like the rest of the ROS side) so it
    # can run under a python_exec that has DPVO + torch installed. Passing the ROS
    # args explicitly keeps use_sim_time consistent with Isaac's /clock.
    dpvo_bridge = ExecuteProcess(
        cmd=[python_exec, DPVO_BRIDGE, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['dpvo_path:=', dpvo_path]],
        output='screen')

    # ── Shared sparse-cloud -> /map projector (for the Nav2 static layer) ────────
    map_builder = ExecuteProcess(
        cmd=[sys.executable, MAP_BUILDER, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-r', 'cloud:=/dpvo/map_points'],
        output='screen')

    ld = LaunchDescription()
    for cmd in declare_cmds:
        ld.add_action(cmd)
    ld.add_action(base)
    ld.add_action(camera_optical_tf)
    ld.add_action(dpvo_bridge)
    ld.add_action(map_builder)
    return ld
