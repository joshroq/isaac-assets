"""
On-launch camera viewports for the UGV rover in Isaac Sim.

Opens THREE perspectives when the (windowed) sim starts:

  1. Main viewport  — the default free-fly "Perspective" camera. Left untouched
                      so you can still orbit/zoom around the whole scene in third
                      person (W/A/S/D/Q/E fly controls).
  2. Arm camera     — a viewport locked to the pan/tilt camera at the TOP of the
                      rover arm (pt_camera_link). Follows the arm as it pans/tilts.
  3. Front depth    — a viewport locked to the front 3D depth sensor
                      (RSD455 emulating the real OAK-D-Lite) at 3d_camera_link.
                      This is the same camera feeding the /oak depth topics, so
                      what you see is what RTAB-Map localizes from.

This module only configures the viewport UI. It does NOT publish anything and is
only called in windowed (non-headless) mode — headless has no viewports.
"""

from __future__ import annotations


# Prim-name hints used to locate the two on-board cameras in the rover USD.
ARM_CAMERA_HINTS = ("pt_camera_link", "pt_link2")
FRONT_CAMERA_LINK_HINTS = ("3d_camera_link", "rsd455", "realsense", "d455")
ARM_CAMERA_NAME = "ArmViewCam"
FRONT_VIEW_CAMERA_NAME = "FrontRgbViewCam"

# Forward-looking orientation (quaternion w,x,y,z) for a USD camera mounted on an
# x-forward rover link: maps the camera's default -Z view direction onto the
# link's +X (forward) with world +Z up. Used only when we must author a fallback
# camera because the arm link ships none.
_FORWARD_CAM_QUAT = (0.5, 0.5, -0.5, -0.5)
FALLBACK_WINDOW_SIZE = (1600, 900)
MIN_CAMERA_VIEW_WIDTH = 480
MIN_CAMERA_VIEW_HEIGHT = 300
CAMERA_VIEW_MARGIN_TOP = 32


def _find_prim_by_name_hints(stage, robot_prim_path: str, hints, type_name=None):
    """First prim under the rover whose path contains any hint (optionally typed)."""
    prefix = robot_prim_path.rstrip("/") + "/"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path != robot_prim_path and not path.startswith(prefix):
            continue
        if type_name is not None and str(prim.GetTypeName()) != type_name:
            continue
        lower = path.lower()
        if any(hint in lower for hint in hints):
            return prim
    return None


def _ensure_arm_camera(robot_prim_path: str):
    """
    Return the path of a Camera at the top of the rover arm.

    Prefers a Camera prim already under pt_camera_link / pt_link2; if the arm
    link ships no camera, authors a forward-looking one as its child so the
    viewport has something to lock to. Returns None if the arm link is missing.
    """
    import omni.usd
    from pxr import UsdGeom, Gf

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    existing = _find_prim_by_name_hints(
        stage, robot_prim_path, ARM_CAMERA_HINTS, type_name="Camera"
    )
    if existing is not None:
        print(f"[viewport_setup] Using existing arm camera: {existing.GetPath()}")
        return str(existing.GetPath())

    arm_link = _find_prim_by_name_hints(stage, robot_prim_path, ARM_CAMERA_HINTS)
    if arm_link is None:
        print(
            "[viewport_setup] Warning: arm link (pt_camera_link/pt_link2) not "
            "found; skipping arm-camera viewport."
        )
        return None

    cam_path = arm_link.GetPath().AppendChild(ARM_CAMERA_NAME)
    if not stage.GetPrimAtPath(cam_path).IsValid():
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.CreateFocalLengthAttr().Set(18.0)
        cam.CreateClippingRangeAttr().Set((0.05, 1000.0))
        # Author a single orient op so the camera looks forward off the arm.
        cam.AddOrientOp().Set(Gf.Quatf(*_FORWARD_CAM_QUAT))
        print(f"[viewport_setup] Authored fallback arm camera: {cam_path}")
    return str(cam_path)


def _ensure_front_view_camera(robot_prim_path: str, depth_camera_prim_path: str | None):
    """
    Return a reliable RGB viewport camera for the front depth rig.

    Some sensor camera prims render black in an Isaac viewport even while they
    work as render-product inputs. For the human viewport, author a sibling view
    camera under the same front camera link and aim it forward. The ROS RGB-D
    publisher still uses the original depth_camera_prim_path.
    """
    import omni.usd
    from pxr import UsdGeom, Gf

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return depth_camera_prim_path

    front_link = _find_prim_by_name_hints(
        stage, robot_prim_path, FRONT_CAMERA_LINK_HINTS
    )
    if front_link is None and depth_camera_prim_path is not None:
        depth_prim = stage.GetPrimAtPath(depth_camera_prim_path)
        parent = depth_prim.GetParent() if depth_prim.IsValid() else None
        front_link = parent if parent is not None and parent.IsValid() else None

    if front_link is None:
        return depth_camera_prim_path

    cam_path = front_link.GetPath().AppendChild(FRONT_VIEW_CAMERA_NAME)
    if not stage.GetPrimAtPath(cam_path).IsValid():
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.CreateFocalLengthAttr().Set(18.0)
        cam.CreateHorizontalApertureAttr().Set(24.95)
        cam.CreateVerticalApertureAttr().Set(18.71)
        cam.CreateClippingRangeAttr().Set((0.05, 1000.0))
        cam.AddOrientOp().Set(Gf.Quatf(*_FORWARD_CAM_QUAT))
        print(f"[viewport_setup] Authored front RGB view camera: {cam_path}")
    return str(cam_path)


def _get_isaac_window_size() -> tuple[int, int]:
    """Best-effort Isaac app window size for deterministic viewport placement."""
    try:
        import omni.appwindow

        app_window = omni.appwindow.get_default_app_window()
        for method_name in ("get_size", "get_window_size", "get_framebuffer_size"):
            method = getattr(app_window, method_name, None)
            if method is None:
                continue
            size = method()
            if size is not None and len(size) >= 2:
                width, height = int(size[0]), int(size[1])
                if width > 0 and height > 0:
                    return width, height

        width = getattr(app_window, "width", None)
        height = getattr(app_window, "height", None)
        if width is not None and height is not None:
            width, height = int(width), int(height)
            if width > 0 and height > 0:
                return width, height
    except Exception as exc:
        print(f"[viewport_setup] Warning: could not read Isaac window size: {exc}")

    return FALLBACK_WINDOW_SIZE


# Fraction of the Isaac window width the rover-camera column occupies (left third).
LEFT_COLUMN_WIDTH_RATIO = 1.0 / 3.0


def _left_third_layout() -> dict:
    """Geometry for two rover camera views stacked inside the left third.

    `width_ratio` is what the docking path uses (a fraction of the main viewport);
    the pixel `width`/`height`/positions are only the floating fallback when
    docking is unavailable.
    """
    window_width, window_height = _get_isaac_window_size()
    left_width = max(MIN_CAMERA_VIEW_WIDTH, int(window_width * LEFT_COLUMN_WIDTH_RATIO))
    usable_height = max(MIN_CAMERA_VIEW_HEIGHT * 2, window_height - CAMERA_VIEW_MARGIN_TOP)
    half_height = max(MIN_CAMERA_VIEW_HEIGHT, usable_height // 2)
    return {
        "width_ratio": LEFT_COLUMN_WIDTH_RATIO,
        "width": left_width,
        "height": half_height,
        "top_position": (0, CAMERA_VIEW_MARGIN_TOP),
        "bottom_position": (0, CAMERA_VIEW_MARGIN_TOP + half_height),
    }


def _set_window_geometry(window, position, width: int, height: int) -> None:
    """Best-effort explicit geometry for floating or newly-created viewport windows."""
    for attr, value in (
        ("position_x", int(position[0])),
        ("position_y", int(position[1])),
        ("width", int(width)),
        ("height", int(height)),
    ):
        try:
            setattr(window, attr, value)
        except Exception:
            pass


def _make_viewport(name: str, camera_path: str, position, width: int, height: int):
    """Create a docked viewport window locked to camera_path. Returns the window."""
    from omni.kit.viewport.utility import create_viewport_window

    window = create_viewport_window(
        name,
        width=int(width),
        height=int(height),
        position_x=int(position[0]),
        position_y=int(position[1]),
    )
    _set_window_geometry(window, position, width, height)
    try:
        window.viewport_api.set_active_camera(camera_path)
    except Exception as exc:  # API name varies slightly across Kit versions
        print(f"[viewport_setup] Warning: could not lock '{name}' to {camera_path}: {exc}")
    return window


# Keeps the deferred-docking update subscription alive until it has run. A bare
# local handle would be garbage-collected the moment setup_nav_viewports returns,
# unsubscribing before the dock ever happens.
_DOCK_SUB = None
# Frames to wait before docking. A freshly created viewport window is not in the
# workspace yet, so docking on frame 0 silently no-ops and the windows stay
# floating (the "I have to drag them" symptom). A few app updates let Kit
# register the windows first.
_DOCK_DELAY_FRAMES = 4


def _apply_dock(windows, layout):
    """Dock the rover camera windows into the LEFT third of the main viewport.

    Layout: a single left column = `width_ratio` of the main Viewport, split
    top/bottom — top = front depth, bottom = arm. Splitting the main Viewport
    (rather than floating) is what makes them panes of the Isaac window.
    """
    import omni.ui as ui

    main = ui.Workspace.get_window("Viewport")
    if main is None:
        print("[viewport_setup] Warning: main 'Viewport' window not found; "
              "leaving rover cameras floating.")
        return False

    top = windows[0]
    bottom = windows[1] if len(windows) > 1 else None

    # Carve the left column off the main viewport, then split it in two.
    top.dock_in(main, ui.DockPosition.LEFT, layout["width_ratio"])
    if bottom is not None:
        bottom.dock_in(top, ui.DockPosition.BOTTOM, 0.5)

    # Show them as plain panes (no tab bar) and keep them put.
    for win in windows:
        for attr, value in (("dock_tab_bar_visible", False),
                            ("dock_tab_bar_enabled", False)):
            try:
                setattr(win, attr, value)
            except Exception:
                pass
    print("[viewport_setup] Docked rover cameras: left third, "
          "top (depth) / bottom (arm) split.")
    return True


def _dock_rover_viewports(windows, layout):
    """
    Dock the rover camera windows as a vertical stack in the left third of the
    Isaac window, deferred by a few frames so Kit has registered the windows.

    Docking is best-effort across Kit versions: if it fails, the windows remain
    open as floating windows at their fallback positions.
    """
    global _DOCK_SUB
    if not windows:
        return

    # Floating fallback geometry, applied immediately so that even if docking is
    # unavailable the windows land roughly in the left column rather than centred.
    _set_window_geometry(windows[0], layout["top_position"],
                         layout["width"], layout["height"])
    if len(windows) > 1:
        _set_window_geometry(windows[1], layout["bottom_position"],
                             layout["width"], layout["height"])

    try:
        import omni.kit.app
    except Exception as exc:
        print(f"[viewport_setup] Warning: omni.kit.app unavailable for docking: {exc}")
        return

    state = {"frames": 0}

    def _on_update(_event):
        global _DOCK_SUB
        state["frames"] += 1
        if state["frames"] < _DOCK_DELAY_FRAMES:
            return
        try:
            _apply_dock(windows, layout)
        except Exception as exc:
            print(f"[viewport_setup] Warning: docking rover cameras failed: {exc}")
        finally:
            _DOCK_SUB = None  # drop the handle -> unsubscribe (one-shot)

    _DOCK_SUB = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(_on_update, name="dock_rover_cams")
    )


def setup_nav_viewports(robot_prim_path: str, depth_camera_prim_path: str | None):
    """
    Open the arm-camera and front-depth-camera viewports alongside the default
    Perspective. Safe to call only in windowed mode; failures are non-fatal.

    Parameters
    ----------
    robot_prim_path : str
        Rover articulation root (e.g. "/World/ugv_rover").
    depth_camera_prim_path : str or None
        The front depth camera prim found by find_depth_camera_prim(); when None,
        only the arm viewport is opened (plus the default Perspective).
    """
    windows = []
    layout = _left_third_layout()

    # 3. Front 3D depth camera perspective (RSD455 -> OAK-D-Lite). The ROS
    #    publisher still uses depth_camera_prim_path; the viewport gets a
    #    guaranteed forward-looking RGB camera on the same physical link.
    if depth_camera_prim_path is not None:
        try:
            front_view_cam = _ensure_front_view_camera(
                robot_prim_path, depth_camera_prim_path
            )
            windows.append(
                _make_viewport(
                    "Front Depth Camera",
                    front_view_cam,
                    layout["top_position"],
                    layout["width"],
                    layout["height"],
                )
            )
            print(
                f"[viewport_setup] Front depth viewport -> {front_view_cam} "
                f"(sensor prim: {depth_camera_prim_path})"
            )
        except Exception as exc:
            print(f"[viewport_setup] Warning: front depth viewport failed: {exc}")
    else:
        print("[viewport_setup] No depth camera prim; skipping front depth viewport.")

    # 2. Pan/tilt camera at the top of the arm.
    try:
        arm_cam = _ensure_arm_camera(robot_prim_path)
        if arm_cam is not None:
            windows.append(
                _make_viewport(
                    "Arm Camera",
                    arm_cam,
                    layout["bottom_position"],
                    layout["width"],
                    layout["height"],
                )
            )
            print(f"[viewport_setup] Arm viewport -> {arm_cam}")
    except Exception as exc:
        print(f"[viewport_setup] Warning: arm viewport failed: {exc}")

    _dock_rover_viewports(windows, layout)

    # 1. The main viewport keeps its default free-fly Perspective camera (the
    #    movable third-person view) — we deliberately leave it untouched.
    print(
        "[viewport_setup] Perspectives ready: main free-fly Perspective + "
        f"{len(windows)} on-board camera viewport(s)."
    )
    return windows
