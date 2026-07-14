"""
Isaac Sim ROS 2 Bridge Action Graph setup for rover drive control.

This module builds the ROS 2 bridge graph inside Isaac Sim:

    ROS2SubscribeTwist -> DifferentialController -> ArticulationController

The generated /cmd_vel wheel-output nodes are retained for graph compatibility
but are disabled by ugv_sim.py. Actual wheel actuation is handled by
ros2_cmd_vel_driver.py so ROS 2 and keyboard control both use
RoverDriveController, matching ugv_ws/ugv_driver.py more closely.
"""

from __future__ import annotations


DEFAULT_GRAPH_PATH = "/World/ActionGraph_ROS2_Drive"
DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"
DEFAULT_CLOCK_TOPIC = "/clock"
DEFAULT_ROS_DOMAIN_ID = 0

# Must match rover_drive_controller.py. Real-hardware spec: 80 mm tyre -> 0.04 m
# rolling radius (the physical collider in jetson_rover.usd is scaled to match).
WHEEL_RADIUS = 0.04
WHEEL_DISTANCE = 0.175
MAX_LINEAR_SPEED = 1.3   # product max speed
MAX_ANGULAR_SPEED = 1.0

# PARITY NOTE: the real driver (ugv_ws ugv_driver.py) bumps a commanded angular
# velocity to ±0.2 rad/s when linear == 0 and 0 < |angular| < 0.2. Both keyboard
# control and the Python ROS 2 /cmd_vel path use rover_drive_controller.drive(),
# so they share that behavior. The stock DifferentialController nodes created
# here are kept disabled by ugv_sim.py.

# The rover is a SIX-wheel skid-steer (all six on the ground, like the real
# robot). We drive only the four CORNER wheels — up (front) + down (rear) each
# side — and leave the centre pair as free-rolling idlers near the turn pivot.
# This matches the Gazebo skid_steer_drive model (ugv_gazebo/models/ugv_rover/
# model.sdf, num_wheel_pairs=2, corner joints only): one speed per side is all
# the differential-drive kinematics needs. The centre wheels keep their colliders.
LEFT_WHEEL_JOINTS = [
    "left_up_wheel_link_joint",
    "left_down_wheel_link_joint",
]

RIGHT_WHEEL_JOINTS = [
    "right_up_wheel_link_joint",
    "right_down_wheel_link_joint",
]

REQUIRED_EXTENSIONS = [
    "isaacsim.ros2.bridge",
    "isaacsim.robot.wheeled_robots",
    "isaacsim.core.nodes",
    "isaacsim.sensors.experimental.rtx",  # OmniLidar schema/runtime support
    "isaacsim.sensors.rtx.nodes",  # RTX lidar debug draw writer
    "isaacsim.sensors.physx",  # IsaacReadLidarBeams (PhysX range-sensor lidar)
    "omni.graph.nodes",
]


def enable_ros2_bridge_extensions() -> None:
    """Enable Isaac Sim extensions required by the ROS 2 drive graph."""
    from isaacsim.core.utils.extensions import enable_extension

    for extension_name in REQUIRED_EXTENSIONS:
        enable_extension(extension_name)


def ensure_ros2_drive_graph(
    robot_prim_path: str,
    graph_path: str = DEFAULT_GRAPH_PATH,
    ros_domain_id: int = DEFAULT_ROS_DOMAIN_ID,
    cmd_vel_topic: str = DEFAULT_CMD_VEL_TOPIC,
    clock_topic: str = DEFAULT_CLOCK_TOPIC,
) -> None:
    """
    Recreate the generated ROS 2 drive Action Graph.

    The graph subscribes to cmd_vel, publishes clock, and contains generated
    wheel-control nodes. ugv_sim.py keeps those wheel-output nodes disabled and
    routes /cmd_vel through ros2_cmd_vel_driver.py instead, so the active control
    path matches RoverDriveController and the real ugv_ws driver semantics.
    """
    import omni.graph.core as og
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("[ROS2BridgeDriveGraph] No USD stage is loaded.")

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        raise RuntimeError(
            "[ROS2BridgeDriveGraph] Robot prim does not exist: "
            f"{robot_prim_path}"
        )

    graph_prim = stage.GetPrimAtPath(graph_path)
    if graph_prim.IsValid():
        stage.RemovePrim(graph_path)

    # ConstructArray exposes input0 implicitly; input1..inputN-1 must be created
    # and wired explicitly. Derive the wiring from the wheel-joint counts so the
    # graph follows LEFT/RIGHT_WHEEL_JOINTS (now 4-wheel drive) with no hard-coded
    # per-wheel lines to fall out of sync.
    velocity_input_attrs = [
        (f"{side}VelocityArray.inputs:input{i}", "double")
        for side, joints in (("Left", LEFT_WHEEL_JOINTS), ("Right", RIGHT_WHEEL_JOINTS))
        for i in range(1, len(joints))
    ]
    velocity_input_connects = [
        (f"{side}SpeedIndex.outputs:value", f"{side}VelocityArray.inputs:input{i}")
        for side, joints in (("Left", LEFT_WHEEL_JOINTS), ("Right", RIGHT_WHEEL_JOINTS))
        for i in range(len(joints))
    ]

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinearVelocity", "omni.graph.nodes.BreakVector3"),
                ("BreakAngularVelocity", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("LeftSpeedIndex", "omni.graph.nodes.ArrayIndex"),
                ("RightSpeedIndex", "omni.graph.nodes.ArrayIndex"),
                ("LeftVelocityArray", "omni.graph.nodes.ConstructArray"),
                ("RightVelocityArray", "omni.graph.nodes.ConstructArray"),
                ("LeftArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("RightArtController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CREATE_ATTRIBUTES: velocity_input_attrs,
            keys.CONNECT: [
                ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "LeftArtController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RightArtController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinearVelocity.inputs:tuple"),
                ("BreakLinearVelocity.outputs:x", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngularVelocity.inputs:tuple"),
                ("BreakAngularVelocity.outputs:z", "DiffController.inputs:angularVelocity"),
                ("DiffController.outputs:velocityCommand", "LeftSpeedIndex.inputs:array"),
                ("DiffController.outputs:velocityCommand", "RightSpeedIndex.inputs:array"),
                *velocity_input_connects,
                ("LeftVelocityArray.outputs:array", "LeftArtController.inputs:velocityCommand"),
                ("RightVelocityArray.outputs:array", "RightArtController.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", int(ros_domain_id)),
                ("ROS2Context.inputs:useDomainIDEnvVar", True),
                ("SubscribeTwist.inputs:topicName", cmd_vel_topic),
                ("PublishClock.inputs:topicName", clock_topic),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", WHEEL_DISTANCE),
                ("DiffController.inputs:maxLinearSpeed", MAX_LINEAR_SPEED),
                ("DiffController.inputs:maxAngularSpeed", MAX_ANGULAR_SPEED),
                ("LeftSpeedIndex.inputs:index", 0),
                ("RightSpeedIndex.inputs:index", 1),
                ("LeftVelocityArray.inputs:arraySize", len(LEFT_WHEEL_JOINTS)),
                ("LeftVelocityArray.inputs:arrayType", "double[]"),
                ("RightVelocityArray.inputs:arraySize", len(RIGHT_WHEEL_JOINTS)),
                ("RightVelocityArray.inputs:arrayType", "double[]"),
                ("LeftArtController.inputs:robotPath", robot_prim_path),
                ("LeftArtController.inputs:jointNames", LEFT_WHEEL_JOINTS),
                ("RightArtController.inputs:robotPath", robot_prim_path),
                ("RightArtController.inputs:jointNames", RIGHT_WHEEL_JOINTS),
            ],
        },
    )

    print(
        "[ROS2BridgeDriveGraph] Created drive graph:\n"
        f"  graph:        {graph_path}\n"
        f"  robot:        {robot_prim_path}\n"
        f"  domain id:    {ros_domain_id}\n"
        f"  cmd_vel:      {cmd_vel_topic}\n"
        f"  clock:        {clock_topic}"
    )


# Names of the OmniGraph nodes that actually write wheel velocities onto the
# rover articulation. Toggling these is how the keyboard takes over from ROS 2.
WHEEL_OUTPUT_NODES = ["LeftArtController", "RightArtController"]


def set_wheel_output_enabled(
    enabled: bool,
    graph_path: str = DEFAULT_GRAPH_PATH,
) -> None:
    """
    Enable or disable the ROS 2 Bridge's wheel-output nodes.

    When the keyboard override is active, the keyboard owns the wheels, so we
    disable these nodes to stop /cmd_vel from competing for the same joints.
    When the keyboard hands control back, we re-enable them and ROS 2 drives.

    Only the articulation-controller nodes are toggled, so /clock keeps
    publishing in both states.
    """
    import omni.graph.core as og

    for node_name in WHEEL_OUTPUT_NODES:
        node_path = f"{graph_path}/{node_name}"
        node = og.get_node_by_path(node_path)
        if node is None or not node.is_valid():
            print(
                "[ROS2BridgeDriveGraph] Warning: wheel-output node not found: "
                f"{node_path}"
            )
            continue
        node.set_disabled(not enabled)

    print(
        "[ROS2BridgeDriveGraph] ROS 2 wheel output "
        + ("ENABLED (generated ROS 2 graph drives wheels)" if enabled
           else "DISABLED (generated graph does not drive wheels)")
    )


# ─────────────────────────────────────────────────────────────────────────────
# Nav2 publishers (odometry + TF + LaserScan)
#
# These are ADDED to the same Action Graph as the drive nodes, but are NOT in
# WHEEL_OUTPUT_NODES, so they keep publishing regardless of whether the keyboard
# or ROS 2 owns the wheels. They provide the data the real rover's hardware
# normally supplies (encoder odometry + the LD lidar), which Nav2 needs:
#   * /odom            (nav_msgs/Odometry)        frames: odom -> base_footprint
#   * /tf  odom->base_footprint (raw transform)
#   * /scan            (sensor_msgs/LaserScan)    frame:  base_lidar_link
#
# robot_state_publisher (ROS side, from the URDF) supplies base_footprint ->
# base_link -> {base_lidar_link, wheels, ...}; slam_toolbox supplies map -> odom.
# ─────────────────────────────────────────────────────────────────────────────

ODOM_TOPIC = "/odom"
ODOM_FRAME_ID = "odom"
BASE_FRAME_ID = "base_footprint"

SCAN_TOPIC = "/scan"
LIDAR_FRAME_ID = "base_lidar_link"
# The rover USD has a real lidar baked in under base_lidar_link. It may be the
# older PhysX range-sensor lidar or the newer inherited RTX OmniLidar
# (for example, Example_2D_Rotary/Exampled_2D_rotary). We use the baked sensor
# as-is; DEFAULT_LIDAR_SUBPATH is relative to the rover root.
DEFAULT_LIDAR_SUBPATH = "base_lidar_link/Lidar"
LIDAR_TYPE_TOKENS = ("Lidar", "OmniLidar", "OmniLidarSensor")


def _is_rtx_lidar_prim(prim) -> bool:
    return (
        prim is not None
        and prim.IsValid()
        and str(prim.GetTypeName()) == "OmniLidar"
        and prim.HasAPI("OmniSensorGenericLidarCoreAPI")
    )


def _is_lidar_prim(prim) -> bool:
    if prim is None or not prim.IsValid():
        return False
    return str(prim.GetTypeName()) in LIDAR_TYPE_TOKENS or _is_rtx_lidar_prim(prim)


def add_odometry_and_tf(
    robot_prim_path: str,
    graph_path: str = DEFAULT_GRAPH_PATH,
    odom_topic: str = ODOM_TOPIC,
    odom_frame_id: str = ODOM_FRAME_ID,
    base_frame_id: str = BASE_FRAME_ID,
) -> None:
    """
    Add odometry + the odom->base_footprint raw TF to the existing drive graph.

    IsaacComputeOdometry reads the rover articulation pose/velocity and feeds:
      * ROS2PublishOdometry        -> /odom
      * ROS2PublishRawTransformTree -> /tf  (odom -> base_footprint)
    """
    import omni.graph.core as og
    import usdrt

    keys = og.Controller.Keys
    og.Controller.edit(
        graph_path,
        {
            keys.CREATE_NODES: [
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                # Existing nodes (created by ensure_ros2_drive_graph) must be
                # referenced by absolute path when editing an existing graph.
                (f"{graph_path}/OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishRawTF.inputs:execIn"),
                (f"{graph_path}/ROS2Context.outputs:context", "PublishOdom.inputs:context"),
                (f"{graph_path}/ROS2Context.outputs:context", "PublishRawTF.inputs:context"),
                (f"{graph_path}/ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                (f"{graph_path}/ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:position", "PublishRawTF.inputs:translation"),
                ("ComputeOdom.outputs:orientation", "PublishRawTF.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_prim_path)]),
                ("PublishOdom.inputs:topicName", odom_topic),
                ("PublishOdom.inputs:odomFrameId", odom_frame_id),
                ("PublishOdom.inputs:chassisFrameId", base_frame_id),
                ("PublishRawTF.inputs:topicName", "/tf"),
                ("PublishRawTF.inputs:parentFrameId", odom_frame_id),
                ("PublishRawTF.inputs:childFrameId", base_frame_id),
                ("PublishRawTF.inputs:staticPublisher", False),
            ],
        },
    )

    print(
        "[ROS2BridgeDriveGraph] Added odometry + TF:\n"
        f"  odom topic:   {odom_topic}\n"
        f"  TF:           {odom_frame_id} -> {base_frame_id}"
    )


def find_lidar_prim(
    robot_prim_path: str,
    preferred_subpath: str = DEFAULT_LIDAR_SUBPATH,
):
    """
    Locate the lidar sensor already baked into the rover USD.

    We do NOT author a lidar — the rover USD ships one (the user positions it).
    Tries the known subpath first, then scans the rover subtree for a lidar-typed
    prim. Returns the prim path, or None (with a warning) if none is found, in
    which case /scan is simply not published.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    candidate = f"{robot_prim_path}/{preferred_subpath}"
    prim = stage.GetPrimAtPath(candidate)
    if _is_lidar_prim(prim):
        print(
            f"[ROS2BridgeDriveGraph] Using baked-in lidar: {candidate} "
            f"(type={prim.GetTypeName()})"
        )
        return candidate
    if prim.IsValid():
        print(
            "[ROS2BridgeDriveGraph] Preferred lidar path exists but is not a "
            f"lidar sensor: {candidate} (type={prim.GetTypeName()}); "
            "scanning descendants."
        )

    root = stage.GetPrimAtPath(robot_prim_path)
    if root.IsValid():
        prefix = robot_prim_path.rstrip("/") + "/"
        for p in stage.Traverse():
            path = str(p.GetPath())
            if path != robot_prim_path and not path.startswith(prefix):
                continue
            if _is_lidar_prim(p):
                print(
                    f"[ROS2BridgeDriveGraph] Found lidar by type: {path} "
                    f"(type={p.GetTypeName()})"
                )
                return path

    print(
        "[ROS2BridgeDriveGraph] Warning: no lidar prim found under "
        f"{robot_prim_path}. /scan will not be published."
    )
    return None


def add_lidar_scan(
    lidar_prim_path: str,
    graph_path: str = DEFAULT_GRAPH_PATH,
    scan_topic: str = SCAN_TOPIC,
    frame_id: str = LIDAR_FRAME_ID,
    show_debug_view: bool = False,
) -> None:
    """
    Publish a sensor_msgs/LaserScan from the baked-in lidar, choosing the right
    pipeline for the sensor type so either lidar works:

      * RTX lidar (prim type "OmniLidar")   -> IsaacCreateRenderProduct +
        ROS2RtxLidarHelper. Render-based; supports real OEM sensor profiles
        (e.g. Slamtec RPLIDAR_S2E). RECOMMENDED for fidelity. Needs the renderer
        (works headless/offscreen).
      * PhysX range-sensor lidar ("Lidar")  -> IsaacReadLidarBeams +
        ROS2PublishLaserScan. Physics-raycast; this sensor type is deprecated.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(lidar_prim_path) if stage is not None else None
    type_name = str(prim.GetTypeName()) if (prim is not None and prim.IsValid()) else ""

    if type_name == "Lidar":
        if show_debug_view:
            print(
                "[ROS2BridgeDriveGraph] Warning: lidar debug view is only "
                "supported for RTX OmniLidar sensors; ignoring for PhysX lidar."
            )
        _add_physx_lidar_scan(lidar_prim_path, graph_path, scan_topic, frame_id)
    else:
        # OmniLidar (RTX) — and anything else — goes through the RTX pipeline.
        _add_rtx_lidar_scan(
            lidar_prim_path,
            graph_path,
            scan_topic,
            frame_id,
            show_debug_view=show_debug_view,
        )


def _add_rtx_lidar_scan(
    lidar_prim_path: str,
    graph_path: str = DEFAULT_GRAPH_PATH,
    scan_topic: str = SCAN_TOPIC,
    frame_id: str = LIDAR_FRAME_ID,
    show_debug_view: bool = False,
) -> None:
    """
    RTX lidar -> LaserScan via a render product. Requires an OmniLidar prim.

    Wires IsaacCreateRenderProduct(lidar) -> ROS2RtxLidarHelper(type=laser_scan).
    """
    import omni.graph.core as og
    import usdrt

    keys = og.Controller.Keys
    og.Controller.edit(
        graph_path,
        {
            keys.CREATE_NODES: [
                ("LidarRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("LidarScanHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.CONNECT: [
                # Existing nodes referenced by absolute path (see add_odometry_and_tf).
                (f"{graph_path}/OnPlaybackTick.outputs:tick", "LidarRenderProduct.inputs:execIn"),
                ("LidarRenderProduct.outputs:execOut", "LidarScanHelper.inputs:execIn"),
                ("LidarRenderProduct.outputs:renderProductPath",
                 "LidarScanHelper.inputs:renderProductPath"),
                (f"{graph_path}/ROS2Context.outputs:context", "LidarScanHelper.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("LidarRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(lidar_prim_path)]),
                ("LidarScanHelper.inputs:topicName", scan_topic),
                ("LidarScanHelper.inputs:frameId", frame_id),
                ("LidarScanHelper.inputs:type", "laser_scan"),
                ("LidarScanHelper.inputs:showDebugView", bool(show_debug_view)),
                ("LidarScanHelper.inputs:resetSimulationTimeOnStop", True),
            ],
        },
    )

    print(
        "[ROS2BridgeDriveGraph] Added lidar scan (RTX ROS2RtxLidarHelper):\n"
        f"  scan topic:   {scan_topic}\n"
        f"  frame id:     {frame_id}\n"
        f"  debug view:   {'on' if show_debug_view else 'off'}\n"
        f"  lidar prim:   {lidar_prim_path}"
    )


def _add_physx_lidar_scan(
    lidar_prim_path: str,
    graph_path: str = DEFAULT_GRAPH_PATH,
    scan_topic: str = SCAN_TOPIC,
    frame_id: str = LIDAR_FRAME_ID,
) -> None:
    """
    Publish a sensor_msgs/LaserScan from a PhysX range-sensor lidar.

    Wires IsaacReadLidarBeams(lidarPrim) -> ROS2PublishLaserScan into the existing
    drive graph. This is physics-raycast based, so it needs no renderer/render
    product and works headless. All scan geometry (FOV, resolution, ranges, row
    count) is taken from the lidar prim's live outputs, so it adapts to the
    sensor config.

    NOTE: ROS2PublishLaserScan requires a single-row (2D) scan — its buffer-size
    check fails if the lidar produces multiple elevation rows. Configure the
    baked lidar as 2D (one elevation row), matching the real 2D LDS lidar.
    """
    import omni.graph.core as og
    import usdrt

    keys = og.Controller.Keys
    og.Controller.edit(
        graph_path,
        {
            keys.CREATE_NODES: [
                ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarBeams"),
                ("PublishLaserScan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
            ],
            keys.CONNECT: [
                # Existing nodes referenced by absolute path (see add_odometry_and_tf).
                (f"{graph_path}/OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PublishLaserScan.inputs:execIn"),
                (f"{graph_path}/ROS2Context.outputs:context", "PublishLaserScan.inputs:context"),
                (f"{graph_path}/ReadSimTime.outputs:simulationTime", "PublishLaserScan.inputs:timeStamp"),
                ("ReadLidar.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
                ("ReadLidar.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
                ("ReadLidar.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
                ("ReadLidar.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
                ("ReadLidar.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
                ("ReadLidar.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
                ("ReadLidar.outputs:numCols", "PublishLaserScan.inputs:numCols"),
                ("ReadLidar.outputs:numRows", "PublishLaserScan.inputs:numRows"),
                ("ReadLidar.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),
            ],
            keys.SET_VALUES: [
                ("ReadLidar.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_prim_path)]),
                ("PublishLaserScan.inputs:topicName", scan_topic),
                ("PublishLaserScan.inputs:frameId", frame_id),
            ],
        },
    )

    print(
        "[ROS2BridgeDriveGraph] Added lidar scan (PhysX IsaacReadLidarBeams):\n"
        f"  scan topic:   {scan_topic}\n"
        f"  frame id:     {frame_id}\n"
        f"  lidar prim:   {lidar_prim_path}"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Front 3D depth camera — LOCATOR ONLY (universal)
#
# Every nav system needs to *find* the front depth camera prim baked into the
# rover USD: the camera nav systems (nav_depth, nav_rgb) publish its stream
# (see common/depth_camera_graph.py), and the on-launch viewports show its view
# regardless of which nav system is active. So the prim *finder* lives here in
# common; the ROS publishing graph is the shared common/depth_camera_graph.py.
# ─────────────────────────────────────────────────────────────────────────────

# Where the front depth camera lives in the rover USD. We search the subtree for
# a Camera-typed prim, preferring one under 3d_camera_link / the RSD455 rig.
DEFAULT_DEPTH_CAMERA_HINTS = ("3d_camera", "rsd455", "realsense", "d455")


def _is_camera_prim(prim) -> bool:
    return (
        prim is not None
        and prim.IsValid()
        and str(prim.GetTypeName()) == "Camera"
    )


def find_depth_camera_prim(robot_prim_path: str):
    """
    Locate the front 3D depth camera (RSD455) baked into the rover USD.

    Like find_lidar_prim(), we never author a camera — the rover USD ships one.
    Scans the rover subtree for Camera-typed prims and prefers, in order:
      1. a colour/RGB camera under the front 3d_camera_link rig,
      2. any camera whose path hints at the front depth rig (RSD455/realsense),
      3. any camera under the rover at all.
    Returns the prim path, or None (with a warning) if no camera is found.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    root = stage.GetPrimAtPath(robot_prim_path)
    if not root.IsValid():
        print(
            "[ROS2BridgeDriveGraph] Warning: rover prim not found for depth "
            f"camera search: {robot_prim_path}"
        )
        return None

    prefix = robot_prim_path.rstrip("/") + "/"
    cameras = []
    for p in stage.Traverse():
        path = str(p.GetPath())
        if path != robot_prim_path and not path.startswith(prefix):
            continue
        if _is_camera_prim(p):
            cameras.append(path)

    if not cameras:
        print(
            "[ROS2BridgeDriveGraph] Warning: no Camera prim found under "
            f"{robot_prim_path}. Depth-camera topics will not be published."
        )
        return None

    def _is_front_rig(path: str) -> bool:
        lower = path.lower()
        return any(hint in lower for hint in DEFAULT_DEPTH_CAMERA_HINTS)

    def _is_color(path: str) -> bool:
        lower = path.lower()
        return ("color" in lower) or ("rgb" in lower)

    front = [c for c in cameras if _is_front_rig(c)]
    pool = front or cameras
    color = [c for c in pool if _is_color(c)]
    chosen = (color or pool)[0]

    print(
        f"[ROS2BridgeDriveGraph] Using depth camera prim: {chosen}\n"
        f"  (candidates: {cameras})"
    )
    return chosen
