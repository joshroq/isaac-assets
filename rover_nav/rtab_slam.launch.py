# Headless RTAB-Map RGB-D + LiDAR SLAM (no rtabmap_viz). base+lidar+OAK+rtabmap -> /map + map->odom.
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = {"frame_id":"base_footprint","queue_size":20,"subscribe_rgb":True,
              "subscribe_depth":True,"subscribe_scan":True,"subscribe_odom_info":False,
              "approx_sync":True,"Rtabmap/DetectionRate":"3.5","use_sim_time":False}
    remap = [("rgb/image","oak/rgb/image_rect"),("rgb/camera_info","oak/rgb/camera_info"),
             ("depth/image","oak/stereo/image_raw")]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_bringup'),'launch','bringup_lidar.launch.py')),
            launch_arguments={'use_rviz':'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_vision'),'launch','oak_d_lite.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_pose_publisher'),'launch','robot_pose_publisher_launch.py'))),
        Node(package='rtabmap_slam',executable='rtabmap',output='screen',
             parameters=[params],remappings=remap,arguments=['-d']),
    ])
