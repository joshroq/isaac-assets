"""
Nav system: ORB-SLAM3 (RGB-D) VISUAL SLAM localization.

map->odom comes from ORB-SLAM3 run DIRECTLY from projects/slam_source/ORB_SLAM3.
The orbslam3_ros2_wrapper node feeds the front camera's RGB image + aligned depth
(/oak/rgb/image_rect + /oak/stereo/image_raw, the RSD455 emulating the real
OAK-D-Lite) to ORB_SLAM3::System::TrackRGBD, reads its camera pose, and publishes
the map->odom transform — the exact role slam_toolbox plays in nav_lidar and
RTAB-Map plays in nav_rgb. ORB-SLAM3 is a sparse ORB-feature SLAM with local bundle
adjustment and DBoW2 loop closure; RGB-D makes it metric (no monocular scale
ambiguity, unlike nav_dpvo). This is the feature-based visual-SLAM system.

Because ORB-SLAM3 emits only a sparse map-point cloud (not a Nav2 occupancy grid),
the shared common/visual_map_builder.py projects /orbslam3/map_points into the /map
the Nav2 global static layer needs. Everything else (Nav2 params, costmaps, RViz) is
the universal base — only the map->odom SOURCE differs (the controlled variable).

This launch = the universal base bringup (common/nav2_bringup_base.launch.py) +
the camera optical-frame TF + the ORB-SLAM3 RGB-D wrapper + the shared visual map
builder.

The matching Isaac run script publishes the /oak topics via
depth_camera_graph.setup_depth_camera (shared module in common/).

PREREQUISITES:
  apt : ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher
        ros-humble-cv-bridge ros-humble-tf2-eigen libpangolin-dev libopencv-dev
        libeigen3-dev libboost-all-dev
  ONE-TIME BUILD: run ./build.sh in this folder (builds libORB_SLAM3.so + the
        wrapper), then `source install/setup.bash` before the ros2 launch below.

RUN (with Isaac running this system's run_small_warehouse_nav.py elsewhere):
  source /opt/ros/humble/setup.bash
  source <this folder>/install/setup.bash          # the built wrapper overlay
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch <this folder>/isaac_nav_orbslam3.launch.py
"""

import importlib.util
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# common/ is two levels up from this system folder (.../common/nav_systems/nav_orbslam3/).
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
COMMON_DIR = os.path.dirname(os.path.dirname(THIS_DIR))
BASE_LAUNCH = os.path.join(COMMON_DIR, 'nav2_bringup_base.launch.py')
MAP_BUILDER = os.path.join(COMMON_DIR, 'visual_map_builder.py')
DEFAULT_SETTINGS = os.path.join(THIS_DIR, 'config', 'oak_rgbd.yaml')
DEFAULT_VOCAB = '/home/joshroq/projects/slam_source/ORB_SLAM3/Vocabulary/ORBvoc.txt'

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
    vocabulary = LaunchConfiguration('vocabulary')
    settings = LaunchConfiguration('settings')
    orbslam_viewer = LaunchConfiguration('orbslam_viewer')

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
        DeclareLaunchArgument('vocabulary', default_value=DEFAULT_VOCAB,
                              description='Path to ORB-SLAM3 ORBvoc.txt'),
        DeclareLaunchArgument('settings', default_value=DEFAULT_SETTINGS,
                              description='Path to the ORB-SLAM3 RGB-D settings yaml'),
        DeclareLaunchArgument('orbslam_viewer', default_value='false',
                              description='Show the ORB-SLAM3 Pangolin viewer'),
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
    # Isaac publishes the RGB + depth images in an optical frame; supply the static
    # TF 3d_camera_link -> oak_rgb_camera_optical_frame (REP-103 body->optical:
    # rpy = -pi/2, 0, -pi/2) so ORB-SLAM3's camera pose lands on the robot.
    # Identical to the RTAB-Map camera launches (same camera, same frames).
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

    # ── ORB-SLAM3 RGB-D bridge: /oak RGB+depth -> map->odom + /orbslam3/map_points
    # Built once by ./build.sh; requires `source install/setup.bash` first so this
    # package resolves.
    orbslam3 = Node(
        package='orbslam3_ros2_wrapper',
        executable='rgbd_odom_node',
        name='orbslam3_rgbd_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vocabulary': vocabulary,
            'settings': settings,
            'use_viewer': orbslam_viewer,
            'rgb_topic': '/oak/rgb/image_rect',
            'depth_topic': '/oak/stereo/image_raw',
            'cloud_topic': '/orbslam3/map_points',
        }],
        respawn=use_respawn, respawn_delay=2.0)

    # ── Shared sparse-cloud -> /map projector (for the Nav2 static layer) ────────
    map_builder = ExecuteProcess(
        cmd=[sys.executable, MAP_BUILDER, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-r', 'cloud:=/orbslam3/map_points'],
        output='screen')

    ld = LaunchDescription()
    for cmd in declare_cmds:
        ld.add_action(cmd)
    ld.add_action(base)
    ld.add_action(camera_optical_tf)
    ld.add_action(orbslam3)
    ld.add_action(map_builder)
    return ld
