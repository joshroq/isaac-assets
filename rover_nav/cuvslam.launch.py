# Isaac ROS Visual SLAM (cuVSLAM), stereo + IMU -> map->odom. Needs the OAK stereo pair
# (start_cuvslam.sh brings it up). Nav/costmap use the lidar separately.
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    vslam = ComposableNode(
        name='visual_slam_node', package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'use_sim_time': False, 'num_cameras': 2, 'image_jitter_threshold_ms': 55.0, 'rectified_images': True,
            'enable_imu_fusion': True, 'imu_frame': 'oak_imu_frame',
            'enable_localization_n_mapping': True,
            'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
            'publish_map_to_odom_tf': True, 'publish_odom_to_base_tf': False,
            'camera_optical_frames': ['oak_left_camera_optical_frame','oak_right_camera_optical_frame'],
        }],
        remappings=[
            ('visual_slam/image_0','/oak/left/image_rect'),
            ('visual_slam/camera_info_0','/oak/left/camera_info'),
            ('visual_slam/image_1','/oak/right/image_rect'),
            ('visual_slam/camera_info_1','/oak/right/camera_info'),
            ('visual_slam/imu','/oak/imu/data'),
        ])
    return LaunchDescription([ComposableNodeContainer(
        name='cuvslam_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', composable_node_descriptions=[vslam], output='screen')])
