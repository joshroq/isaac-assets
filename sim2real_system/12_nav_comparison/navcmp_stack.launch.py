"""navcmp nav stack — the SAME launch on the real rover and next to Isaac.

    ros2 launch <bundle>/stack/navcmp_stack.launch.py platform:=real stack:=lidar bundle_dir:=<bundle>
    ros2 launch <bundle>/stack/navcmp_stack.launch.py platform:=sim  stack:=depth bundle_dir:=<bundle>

Starts (both platforms, identical parameters):
  slam_toolbox (sync, mapping)        stack/slam_toolbox.yaml  (= ugv_slam online_sync config)
  Nav2 navigation_launch.py (stock)   stack/nav2_<stack>.yaml  (= navcmp/nav_params.py output)
  stack 'depth': depth_image_proc point_cloud_xyz -> /navcmp/depth_points (depth only, no RGB)

Platform differences are ONLY the sensor plumbing:
  real  starts the base + LD19 + TF (ugv_bringup bringup_lidar.launch.py, the same include
        rover_nav/start_lidar_nav.sh uses); wall clock. The OAK driver for 'depth' is
        rover_nav/start_oak.sh (started separately by the session, it owns the USB watcher).
  sim   Isaac publishes TF//clock and the namespaced sensors, so /scan -> /ugv_sim/scan and
        /odom -> /ugv_sim/odom are remapped (exactly as isaac_nav_lidar.launch.py does);
        use_sim_time true.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

DEPTH_INPUTS = {
    "real": ("/oak/stereo/image_raw", "/oak/stereo/camera_info"),
    # Isaac's depth shares the RGB intrinsics (see isaac_nav_depth.launch.py / USD_CONTRACT)
    "sim": ("/ugv_sim/oak/stereo/image_raw", "/ugv_sim/oak/rgb/camera_info"),
}
DEPTH_TOPIC = "/navcmp/depth_points"


def _setup(context, *args, **kwargs):
    platform = context.launch_configurations["platform"]
    stack = context.launch_configurations["stack"]
    bundle = os.path.abspath(os.path.expanduser(context.launch_configurations["bundle_dir"]))
    if platform not in ("real", "sim"):
        raise RuntimeError(f"platform must be real|sim, got {platform!r}")
    if stack not in ("lidar", "depth"):
        raise RuntimeError(f"stack must be lidar|depth, got {stack!r}")
    sim = platform == "sim"
    use_sim_time = "true" if sim else "false"

    template = os.path.join(bundle, "stack", f"nav2_{stack}.yaml")
    with open(template) as f:
        text = f.read().replace("@NAVCMP_DIR@", bundle)
    fd, params_file = tempfile.mkstemp(prefix=f"navcmp_nav2_{stack}_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        f.write(text)
    slam_params = os.path.join(bundle, "stack", "slam_toolbox.yaml")

    actions = []
    if not sim:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ugv_bringup"), "launch", "bringup_lidar.launch.py")),
            launch_arguments={"use_rviz": "false"}.items()))

    if stack == "depth":
        image, info = DEPTH_INPUTS[platform]
        actions.append(Node(
            package="depth_image_proc", executable="point_cloud_xyz_node",
            name="navcmp_depth_cloud", output="log",
            parameters=[{"use_sim_time": sim, "queue_size": 10}],
            remappings=[("image_rect", image), ("camera_info", info), ("points", DEPTH_TOPIC)]))

    nav = [
        Node(package="slam_toolbox", executable="sync_slam_toolbox_node", name="slam_toolbox",
             output="screen", parameters=[slam_params, {"use_sim_time": sim}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")),
            launch_arguments={"namespace": "", "use_sim_time": use_sim_time,
                              "autostart": "true", "params_file": params_file,
                              "use_composition": "False", "use_respawn": "False"}.items()),
    ]
    if sim:
        nav = [SetRemap("/scan", "/ugv_sim/scan"), SetRemap("/odom", "/ugv_sim/odom")] + nav
    actions.append(GroupAction(nav))
    print(f"[navcmp_stack] platform={platform} stack={stack} params={params_file}", flush=True)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("platform", description="real | sim"),
        DeclareLaunchArgument("stack", default_value="lidar", description="lidar | depth"),
        DeclareLaunchArgument("bundle_dir", description="directory holding stack/ (this file's parent's parent)"),
        OpaqueFunction(function=_setup),
    ])
