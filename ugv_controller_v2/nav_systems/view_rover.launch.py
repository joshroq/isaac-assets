"""Desktop-side LIVE VIEWER for the PHYSICAL rover (ssh + Fast DDS Discovery Server).

This is the real-rover twin of the `navlaunch -v2 -<stack>` sim pipeline. The
difference in WHERE the work runs:

    sim   : one machine runs Isaac + the FULL Nav2 stack + RViz. isaac_nav_*.launch.py
            remap /scan -> /ugv_sim/scan because Isaac namespaces its sensor data.
    rover : the Nav2 stack + SLAM run ON THE ROVER (rover_nav/start_*.sh). The desktop
            only VISUALIZES — it subscribes to the rover's PLAIN topics (/scan, /map,
            /oak/..., /tf, /plan, costmaps, /robot_description) over the discovery-server
            link and publishes /goal_pose (RViz "2D Nav Goal") + /initialpose back.

So this launch is VIZ-ONLY: no Nav2, no SLAM, no /scan remap. It runs
  * robot_state_publisher  — ONLY to supply /robot_description with file:// mesh URIs
    that resolve on THIS desktop, so the RViz RobotModel has geometry to render. Its
    own /tf + /tf_static are remapped to dead topics so it never competes with the TF
    the rover publishes over the link (the rover owns the whole tree).
  * rviz2  — with the SAME per-stack .rviz used by the sim, but with the `/ugv_sim/`
    namespace stripped so the displays bind to the rover's plain topic names. The sim
    .rviz files stay the single source of truth; this launch de-namespaces a COPY at
    runtime, so the two never drift.

USAGE (front door: rover_comms/desktop/view_rover.sh, which sources the discovery-server
env first — do that, or the desktop sees nothing):
    ros2 launch ugv_controller_v2/nav_systems/view_rover.launch.py stack:=lidar
    ros2 launch .../view_rover.launch.py stack:=depth
    ros2 launch .../view_rover.launch.py stack:=rgb        # (== rtab)
    ros2 launch .../view_rover.launch.py stack:=orbslam3
    ros2 launch .../view_rover.launch.py stack:=cuvslam

The `stack` must match the nav stack you started on the rover (rover_nav/start_*.sh
with ROVER_NAV_COMMS set so it broadcasts to this desktop). Setting a goal: click
"2D Nav Goal" in RViz (publishes /goal_pose, which the rover's Nav2 subscribes to over
the link) — or rover_nav/send_goal.py over ssh as the RViz-less fallback.
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The URDF lives in the (unbuilt) ugv_ws source tree; read it by absolute path and
# rewrite mesh URIs to file:// so robot_state_publisher needs no install.
#
# Use the REAL rover's URDF (ugv_rover.urdf = UGV_MODEL on the Jetson), NOT the
# sim's ugv_rover_isaac_fixed.urdf. The two differ: the Isaac URDF adds
# front_camera_link + left/right_middle_wheel_link (which the baked USD publishes),
# but the physical rover's /tf has 3d_camera_link and only 4 wheels. Loading the
# Isaac URDF here left those 3 links with no matching TF -> RViz RobotModel error.
# The real URDF's 13 links exactly match the frames the rover broadcasts on /tf.
REPO_ROOT = "/home/joshroq/projects/isaac-assets"
UGV_DESCRIPTION_SRC = os.path.join(REPO_ROOT, "ugv_ws/src/ugv_main/ugv_description")
URDF_PATH = os.path.join(UGV_DESCRIPTION_SRC, "urdf", "ugv_rover.urdf")

# Per-stack canonical .rviz — the SAME files the sim `navlaunch -v2 -<stack>` uses.
NAV_SYSTEMS = os.path.join(REPO_ROOT, "ugv_controller_v2", "nav_systems")
UGV_NAV_RVIZ = os.path.join(
    REPO_ROOT, "ugv_ws/src/ugv_main/ugv_nav/rviz/view_nav_2d.rviz"
)
STACK_RVIZ = {
    # lidar uses the rover's own upstream 2D nav view (already plain topics).
    "lidar": UGV_NAV_RVIZ,
    "depth": os.path.join(NAV_SYSTEMS, "nav_depth", "isaac_depth_nav.rviz"),
    "rgb": os.path.join(NAV_SYSTEMS, "nav_rgb", "isaac_rgb_nav.rviz"),
    "rtab": os.path.join(NAV_SYSTEMS, "nav_rgb", "isaac_rgb_nav.rviz"),
    "orbslam3": os.path.join(NAV_SYSTEMS, "nav_orbslam3", "isaac_orbslam3_nav.rviz"),
    "cuvslam": os.path.join(
        NAV_SYSTEMS, "nav_cuvslam_nvblox", "isaac_cuvslam_nvblox_nav.rviz"
    ),
}

# Sim topic name  ->  real-rover topic name. ORDER MATTERS: the specific camera
# remaps run before the generic "/ugv_sim/" -> "/" strip. The rover's OAK driver
# (depthai_ros_driver, name "oak") does NOT rectify RGB by default, so the rectified
# image display binds to /oak/rgb/image_raw; the depth/rgbd clouds both come from the
# single depth_image_proc cloud the rover publishes at /oak/points.
TOPIC_REWRITES = [
    ("/ugv_sim/oak/rgb/image_rect", "/oak/rgb/image_raw"),
    ("/ugv_sim/oak/depth/points", "/oak/points"),
    ("/ugv_sim/oak/rgbd/points", "/oak/points"),
    ("/ugv_sim/", "/"),  # generic: /ugv_sim/scan -> /scan, /ugv_sim/odom -> /odom, ...
]

# The rover's OAK camera driver ALSO publishes a global /robot_description (the camera's
# own model, with package://depthai_descriptions meshes that don't exist on the desktop
# -> RViz "Error retrieving file ... OAK-D-PRO.stl"). To avoid that collision, our local
# robot_state_publisher publishes the rover model on a PRIVATE topic and the RViz
# RobotModel is pointed at it (see _rover_rviz_config), so the OAK's /robot_description is
# simply ignored.
DESC_TOPIC = "/view_rover/robot_description"


def _load_robot_description() -> str:
    if not os.path.isfile(URDF_PATH):
        raise RuntimeError(f"rover URDF not found: {URDF_PATH}")
    with open(URDF_PATH, "r") as f:
        robot_description = f.read()
    # ugv_rover.urdf references meshes as package://ugv_description/... — resolve to
    # file:// so RViz's RobotModel loads them without a built/sourced ugv_ws.
    desc_root = "file://" + UGV_DESCRIPTION_SRC.rstrip("/") + "/"
    return robot_description.replace("package://ugv_description/", desc_root)


def _rover_rviz_config(stack: str) -> str:
    """Return a temp .rviz derived from the sim config with sim topic names
    rewritten to the rover's plain topic names (single source of truth = the sim
    .rviz, de-namespaced at runtime so they never drift)."""
    src = STACK_RVIZ[stack]
    if not os.path.isfile(src):
        raise RuntimeError(f"RViz config for stack '{stack}' not found: {src}")
    with open(src, "r") as f:
        text = f.read()
    for old, new in TOPIC_REWRITES:
        text = text.replace(old, new)
    # Point the RobotModel display at our private description topic (see DESC_TOPIC).
    text = text.replace("/robot_description", DESC_TOPIC)
    fd, path = tempfile.mkstemp(prefix=f"view_rover_{stack}_", suffix=".rviz")
    with os.fdopen(fd, "w") as f:
        f.write(text)
    return path


def _launch_setup(context, *args, **kwargs):
    stack = context.launch_configurations.get("stack", "lidar").lower()
    if stack not in STACK_RVIZ:
        raise RuntimeError(
            f"stack must be one of {sorted(set(STACK_RVIZ))}, got '{stack}'"
        )
    rviz_config = context.launch_configurations.get("rviz_config", "") \
        or _rover_rviz_config(stack)

    actions = [
        # /robot_description for the RViz RobotModel (meshes resolve locally). The
        # rover owns TF, so this node's own /tf + /tf_static go to dead topics.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"use_sim_time": False,
                         "robot_description": _load_robot_description()}],
            remappings=[("/tf", "/rsp_tf_unused"),
                        ("/tf_static", "/rsp_tf_static_unused"),
                        ("/robot_description", DESC_TOPIC)],
            output="screen",
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": False}],  # real rover: wall clock
            output="screen",
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "stack", default_value="lidar",
            description="Which rover nav stack to visualize: "
                        "lidar | depth | rgb (== rtab) | orbslam3 | cuvslam. "
                        "Must match the rover_nav/start_*.sh stack running on the rover."),
        DeclareLaunchArgument(
            "rviz_config", default_value="",
            description="Override the .rviz (default: the sim stack config, "
                        "de-namespaced to the rover's plain topics)."),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch RViz (false = just publish /robot_description)."),
        OpaqueFunction(function=_launch_setup),
    ])
