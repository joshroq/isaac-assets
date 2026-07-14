"""
Front 3D depth-camera publishing — SHARED by the camera-based nav systems.

Lives in common/ because BOTH camera systems use it: nav_rgb (RTAB-Map RGB-D) and
nav_depth (RTAB-Map depth-only / ICP, RGB ignored). It publishes the rover's front
depth sensor (the Intel RealSense D455 baked into the USD, retuned here to emulate
the real rover's Luxonis OAK-D-Lite) as an RGB-D stream on the SAME topic names the
real depthai/oak driver uses, so the downstream RTAB-Map graph is identical to the
hardware one:

    /oak/rgb/image_rect    (sensor_msgs/Image, rgb)     <- colour
    /oak/rgb/camera_info   (sensor_msgs/CameraInfo)     <- shared intrinsics
    /oak/stereo/image_raw  (sensor_msgs/Image, 32FC1 m) <- aligned depth

RGB + depth come from one render product, so they are intrinsically registered
(same pinhole, same resolution) — exactly what RTAB-Map's rgbd_sync expects. The
nav_rgb system consumes RGB + depth; nav_depth consumes depth + camera_info only
(it turns the depth image into a point cloud and never subscribes to RGB). Images
publish in an optical frame (oak_rgb_camera_optical_frame); each camera system's
launch supplies the static TF 3d_camera_link -> oak_rgb_camera_optical_frame.

The runner (common/ugv_sim.py) stays nav-system agnostic: the camera run scripts
pass setup_depth_camera() as the run(..., extra_sensor_setup=...) hook, so only
those systems pull the camera into the ROS 2 graph. The lidar-only system passes
no hook and never publishes these topics.
"""

from __future__ import annotations


# Topic names mirror the real rover's oak_d_lite/depthai driver so the RTAB-Map
# remappings in the camera systems' launches match the hardware launch verbatim.
DEPTH_RGB_TOPIC = "/oak/rgb/image_rect"
DEPTH_INFO_TOPIC = "/oak/rgb/camera_info"
DEPTH_DEPTH_TOPIC = "/oak/stereo/image_raw"
DEPTH_CAMERA_FRAME_ID = "oak_rgb_camera_optical_frame"

# Published image resolution. OAK-D-Lite RGB is 4:3; 640x480 matches the real
# rover's usb/oak stream resolution and keeps RTAB-Map light.
DEPTH_CAMERA_WIDTH = 640
DEPTH_CAMERA_HEIGHT = 480

# OAK-D-Lite intrinsics emulation (so the RSD455 sim sensor behaves like the
# real camera). Derived for a 4:3 frame:
#   HFOV = 2*atan(horizApert / (2*focal)) ~= 69 deg
#   VFOV = 2*atan(vertApert  / (2*focal)) ~= 54 deg
# vs. the stock D455's ~87 deg HFOV. Aperture units are mm to match focalLength.
OAK_FOCAL_LENGTH = 18.15
OAK_HORIZONTAL_APERTURE = 24.95
OAK_VERTICAL_APERTURE = 18.71
# OAK-D-Lite usable stereo depth range (m). Beyond ~12 m disparity is unreliable;
# below ~0.2 m is inside the min-Z. RTX depth is clipped to this band.
OAK_CLIPPING_RANGE = (0.2, 12.0)

# Drive graph the depth publishers attach to (matches common's DEFAULT_GRAPH_PATH).
DEFAULT_GRAPH_PATH = "/World/ActionGraph_ROS2_Drive"


def setup_depth_camera(robot_prim_path: str, graph_path: str = DEFAULT_GRAPH_PATH) -> None:
    """
    run()'s extra_sensor_setup hook for the depth-camera nav system.

    Locates the front depth camera (via common's find_depth_camera_prim) and, if
    present, wires its RGB-D publishers into the existing drive graph. Called from
    the pre-play sensor block in common/ugv_sim.py.
    """
    from ros2_bridge_drive_graph import find_depth_camera_prim

    camera_prim_path = find_depth_camera_prim(robot_prim_path=robot_prim_path)
    if camera_prim_path is None:
        print(
            "[depth_camera_graph] No depth camera prim found; RTAB-Map "
            "localization will lack /oak/rgb + /oak/stereo."
        )
        return
    add_depth_camera(camera_prim_path=camera_prim_path, graph_path=graph_path)


def _tune_depth_camera_intrinsics(camera_prim_path: str) -> None:
    """
    Retune the RSD455 camera prim to OAK-D-Lite optics (FOV + depth clip range),
    so the sim depth sensor follows the camera the real rover actually carries.

    USD-schema edit only — must run BEFORE the sim plays (like the wheel drive
    and lidar setup), never after the articulation views are bound.
    """
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(camera_prim_path) if stage is not None else None
    if prim is None or not prim.IsValid():
        print(
            "[depth_camera_graph] Warning: cannot tune intrinsics, depth "
            f"camera prim invalid: {camera_prim_path}"
        )
        return

    cam = UsdGeom.Camera(prim)
    cam.CreateFocalLengthAttr().Set(float(OAK_FOCAL_LENGTH))
    cam.CreateHorizontalApertureAttr().Set(float(OAK_HORIZONTAL_APERTURE))
    cam.CreateVerticalApertureAttr().Set(float(OAK_VERTICAL_APERTURE))
    cam.CreateClippingRangeAttr().Set(
        (float(OAK_CLIPPING_RANGE[0]), float(OAK_CLIPPING_RANGE[1]))
    )

    print(
        "[depth_camera_graph] Tuned depth camera to OAK-D-Lite optics:\n"
        f"  focalLength:    {OAK_FOCAL_LENGTH} mm\n"
        f"  aperture (h,v): {OAK_HORIZONTAL_APERTURE} x {OAK_VERTICAL_APERTURE} mm "
        "(~69 x 54 deg FOV)\n"
        f"  clip range:     {OAK_CLIPPING_RANGE[0]}-{OAK_CLIPPING_RANGE[1]} m"
    )


def add_depth_camera(
    camera_prim_path: str,
    graph_path: str = DEFAULT_GRAPH_PATH,
    rgb_topic: str = DEPTH_RGB_TOPIC,
    info_topic: str = DEPTH_INFO_TOPIC,
    depth_topic: str = DEPTH_DEPTH_TOPIC,
    frame_id: str = DEPTH_CAMERA_FRAME_ID,
    width: int = DEPTH_CAMERA_WIDTH,
    height: int = DEPTH_CAMERA_HEIGHT,
) -> None:
    """
    Publish the front depth camera's RGB-D stream into the existing drive graph.

    Retunes the camera to OAK-D-Lite optics, then wires one render product to
    RGB/depth ROS2CameraHelper nodes plus the dedicated ROS2CameraInfoHelper.
    These are NOT in WHEEL_OUTPUT_NODES, so they keep publishing regardless of
    who owns the wheels — exactly like the odom/lidar publishers.
    """
    _tune_depth_camera_intrinsics(camera_prim_path)

    import omni.graph.core as og
    import usdrt

    keys = og.Controller.Keys
    og.Controller.edit(
        graph_path,
        {
            keys.CREATE_NODES: [
                ("DepthRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthRgbHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthDepthHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                # Existing nodes referenced by absolute path (see add_odometry_and_tf).
                (f"{graph_path}/OnPlaybackTick.outputs:tick", "DepthRenderProduct.inputs:execIn"),
                ("DepthRenderProduct.outputs:execOut", "DepthRgbHelper.inputs:execIn"),
                ("DepthRenderProduct.outputs:execOut", "DepthDepthHelper.inputs:execIn"),
                ("DepthRenderProduct.outputs:execOut", "DepthInfoHelper.inputs:execIn"),
                ("DepthRenderProduct.outputs:renderProductPath", "DepthRgbHelper.inputs:renderProductPath"),
                ("DepthRenderProduct.outputs:renderProductPath", "DepthDepthHelper.inputs:renderProductPath"),
                ("DepthRenderProduct.outputs:renderProductPath", "DepthInfoHelper.inputs:renderProductPath"),
                (f"{graph_path}/ROS2Context.outputs:context", "DepthRgbHelper.inputs:context"),
                (f"{graph_path}/ROS2Context.outputs:context", "DepthDepthHelper.inputs:context"),
                (f"{graph_path}/ROS2Context.outputs:context", "DepthInfoHelper.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("DepthRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                ("DepthRenderProduct.inputs:width", int(width)),
                ("DepthRenderProduct.inputs:height", int(height)),
                # Colour image -> /oak/rgb/image_rect (sim pinhole is distortion-free,
                # so the raw render already equals the rectified image).
                ("DepthRgbHelper.inputs:type", "rgb"),
                ("DepthRgbHelper.inputs:topicName", rgb_topic),
                ("DepthRgbHelper.inputs:frameId", frame_id),
                # Aligned depth (metres, 32FC1) -> /oak/stereo/image_raw.
                ("DepthDepthHelper.inputs:type", "depth"),
                ("DepthDepthHelper.inputs:topicName", depth_topic),
                ("DepthDepthHelper.inputs:frameId", frame_id),
                # Shared intrinsics -> /oak/rgb/camera_info.
                ("DepthInfoHelper.inputs:topicName", info_topic),
                ("DepthInfoHelper.inputs:frameId", frame_id),
            ],
        },
    )

    print(
        "[depth_camera_graph] Added front depth camera (RSD455 -> OAK-D-Lite):\n"
        f"  rgb topic:    {rgb_topic}\n"
        f"  depth topic:  {depth_topic}\n"
        f"  info topic:   {info_topic}\n"
        f"  frame id:     {frame_id}\n"
        f"  resolution:   {width}x{height}\n"
        f"  camera prim:  {camera_prim_path}"
    )
