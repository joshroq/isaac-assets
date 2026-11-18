"""
Universal ROS 2 Nav2 bringup for the Isaac-Sim rover — SHARED by every nav system.

This is the part of the launch that is identical no matter which navigation
system is being tested. It is BUILD-FREE: it reads the URDF, Nav2 params, and
RViz config straight from the ugv_ws source tree by absolute path (no colcon
build required), and drops the hardware-only pieces Isaac Sim provides:

  * NO bringup_lidar  -> Isaac publishes /scan (RTX lidar).
  * NO base_node/EKF  -> Isaac publishes /odom and the odom->base_footprint TF.

What it starts (all with use_sim_time:=true, since Isaac publishes /clock):
  * robot_state_publisher + joint_state_publisher from the URDF -> static TF
        base_footprint -> base_link -> {base_lidar_link, 3d_camera_link, wheels}
  * a composed Nav2 container + Nav2 navigation (navigation_launch.py) -> /cmd_vel
  * RViz with the existing 2D nav view (optional)

It deliberately does NOT start a localization (map->odom) source — each nav
system supplies its own and includes this base:
  * nav_systems/nav_lidar/isaac_nav_lidar.launch.py -> slam_toolbox (lidar)
  * nav_systems/nav_depth/isaac_nav_depth.launch.py -> RTAB-Map ICP (depth, RGB off)
  * nav_systems/nav_rgb/isaac_nav_rgb.launch.py     -> RTAB-Map RGB-D (RGB + depth)

So exactly one map->odom publisher ever runs.

PREREQUISITES (apt; see ../NAV_README.md) — NOT installed by default:
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                   ros-humble-joint-state-publisher
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# Absolute paths into the existing (unbuilt) workspace source tree. The ugv_ws
# location is independent of this folder, so these are unaffected by the reorg.
UGV_WS_SRC = "/home/joshroq/projects/isaac-assets/ugv_ws/src/ugv_main"
URDF_PATH = os.path.join(UGV_WS_SRC, "ugv_description", "urdf", "ugv_rover_isaac_fixed.urdf")
UGV_DESCRIPTION_SRC = os.path.join(UGV_WS_SRC, "ugv_description")
# Upstream ugv_ws (ros2-humble-develop-251125) replaced the monolithic
# param/slam_nav.yaml with a per-local-planner params/ layout. rpp.yaml is the full
# Nav2 stack (RotationShim+RPP); the depth/rgb/visual systems layer their own
# localization on top of it, exactly as nav_lidar now does via ugv_nav upstream.
DEFAULT_PARAMS = os.path.join(UGV_WS_SRC, "ugv_nav", "params", "rpp.yaml")
DEFAULT_RVIZ = os.path.join(UGV_WS_SRC, "ugv_nav", "rviz", "view_nav_2d.rviz")


def _load_robot_description():
    """Read the URDF and make package:// mesh paths work without colcon install."""
    with open(URDF_PATH, 'r') as f:
        robot_description = f.read()

    mesh_root = "file://" + UGV_DESCRIPTION_SRC.rstrip("/") + "/"
    # Share Isaac geometry; retain the legacy camera TF name used by this stack.
    return (robot_description.replace("../meshes/", mesh_root + "meshes/")
            .replace("package://ugv_description/", mesh_root)
            .replace("front_camera_link", "3d_camera_link"))


def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    declare_cmds = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the /clock published by Isaac Sim'),
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS,
                              description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically start the nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='True',
                              description='Use composed Nav2 bringup'),
        DeclareLaunchArgument('use_respawn', default_value='False',
                              description='Respawn crashed nodes (composition disabled only)'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz with the 2D navigation view'),
        DeclareLaunchArgument('log_level', default_value='info', description='log level'),
    ]

    # ── Robot description / static TF from the URDF (no hardware, no build) ──────
    robot_description = _load_robot_description()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        output='screen')

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ── Composed Nav2 container + Nav2 navigation (no localization here) ─────────
    bringup_cmd_group = GroupAction([
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': '',
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', DEFAULT_RVIZ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    ld = LaunchDescription()
    for cmd in declare_cmds:
        ld.add_action(cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(bringup_cmd_group)
    ld.add_action(rviz_node)
    return ld
