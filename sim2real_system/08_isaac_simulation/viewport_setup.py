"""
On-launch camera viewports for the baked-graph rover in Isaac Sim (windowed only).

Two on-board camera views dock as a vertical stack in the LEFT THIRD of the Isaac
window, alongside the default free-fly Perspective (left untouched for
third-person orbit/zoom):

  * TOP    — Arm camera   : locked to the pan/tilt camera at the top of the arm.
  * BOTTOM — Front camera : locked to the baked RGB front camera.

Both viewports lock directly to explicit baked camera prim paths. The lock is
re-asserted after the windows dock — a freshly created viewport window is not
registered yet, so an immediate set_active_camera can silently no-op (the
"viewport isn't locked / shows a free view" symptom).

This module only configures the viewport UI; it publishes nothing and every
failure is non-fatal.
"""

from __future__ import annotations


FALLBACK_WINDOW_SIZE = (1600, 900)
MIN_CAMERA_VIEW_WIDTH = 480
MIN_CAMERA_VIEW_HEIGHT = 300
CAMERA_VIEW_MARGIN_TOP = 32
LEFT_COLUMN_WIDTH_RATIO = 1.0 / 3.0


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
            if size is not None and len(size) >= 2 and int(size[0]) > 0 and int(size[1]) > 0:
                return int(size[0]), int(size[1])
        width = getattr(app_window, "width", None)
        height = getattr(app_window, "height", None)
        if width and height and int(width) > 0 and int(height) > 0:
            return int(width), int(height)
    except Exception as exc:
        print(f"[viewport_setup] Warning: could not read Isaac window size: {exc}")
    return FALLBACK_WINDOW_SIZE


def _left_third_layout() -> dict:
    """Geometry for two rover camera views stacked inside the left third."""
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
    for attr, value in (("position_x", int(position[0])), ("position_y", int(position[1])),
                        ("width", int(width)), ("height", int(height))):
        try:
            setattr(window, attr, value)
        except Exception:
            pass


def _lock_camera(window, camera_path: str) -> None:
    """Point a viewport window's active camera at camera_path (best effort)."""
    try:
        window.viewport_api.set_active_camera(camera_path)
        # Some Kit builds expose a lock flag, some do not. Set it when present so
        # clicking inside the pane does not immediately turn it back into a free
        # perspective camera.
        for attr in ("camera_locked", "lock_camera", "is_camera_locked"):
            if hasattr(window.viewport_api, attr):
                try:
                    setattr(window.viewport_api, attr, True)
                except Exception:
                    pass
    except Exception as exc:
        print(f"[viewport_setup] Warning: could not lock viewport to {camera_path}: {exc}")


def _make_viewport(name: str, camera_path: str, position, width: int, height: int):
    """Create a viewport window locked to camera_path. Returns the window."""
    from omni.kit.viewport.utility import create_viewport_window

    window = create_viewport_window(
        name, width=int(width), height=int(height),
        position_x=int(position[0]), position_y=int(position[1]),
    )
    _set_window_geometry(window, position, width, height)
    window._rover_camera_path = camera_path  # re-asserted after docking
    _lock_camera(window, camera_path)
    return window


# Keeps the deferred-docking update subscription alive until it has run.
_DOCK_SUB = None
_DOCK_DELAY_FRAMES = 4


def _apply_dock(windows, layout):
    """Dock the camera windows into the LEFT third, split top (arm) / bottom (front),
    and RE-ASSERT each camera lock now that the windows are registered."""
    import omni.ui as ui

    main = ui.Workspace.get_window("Viewport")
    if main is not None:
        top = windows[0]
        bottom = windows[1] if len(windows) > 1 else None
        top.dock_in(main, ui.DockPosition.LEFT, layout["width_ratio"])
        if bottom is not None:
            bottom.dock_in(top, ui.DockPosition.BOTTOM, 0.5)
        for win in windows:
            for attr, value in (("dock_tab_bar_visible", False), ("dock_tab_bar_enabled", False)):
                try:
                    setattr(win, attr, value)
                except Exception:
                    pass
        print("[viewport_setup] Docked rover cameras: left third, top (arm) / bottom (front).")
    else:
        print("[viewport_setup] Warning: main 'Viewport' window not found; cameras left floating.")

    # Re-assert the camera lock — the windows are registered now, so this takes.
    for win in windows:
        cam = getattr(win, "_rover_camera_path", None)
        if cam:
            _lock_camera(win, cam)
    return True


def _dock_rover_viewports(windows, layout):
    """Dock the camera windows as a left-third vertical stack, deferred a few frames
    so Kit has registered them. Floating fallback if docking is unavailable."""
    global _DOCK_SUB
    if not windows:
        return

    _set_window_geometry(windows[0], layout["top_position"], layout["width"], layout["height"])
    if len(windows) > 1:
        _set_window_geometry(windows[1], layout["bottom_position"], layout["width"], layout["height"])

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
            _DOCK_SUB = None  # drop handle -> unsubscribe (one-shot)

    _DOCK_SUB = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(_on_update, name="dock_rover_cams")
    )


def setup_nav_viewports(
    front_camera_prim: str | None,
    arm_camera_prim: str | None,
    robot_prim_path: str = "/World/ugv_rover",
):
    """Open the arm-camera (top) and front-camera (bottom) viewports docked in the
    left third, each locked to the given scene-camera prim. Windowed mode only.

    Parameters
    ----------
    front_camera_prim, arm_camera_prim : str or None
        Exact Camera prim paths. A None or missing prim skips that viewport
        (with a warning).
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    windows = []
    layout = _left_third_layout()

    for name, prim_path, position in (
        ("Arm Camera", arm_camera_prim, layout["top_position"]),
        ("Front Camera", front_camera_prim, layout["bottom_position"]),
    ):
        if not prim_path:
            print(f"[viewport_setup] No prim given for '{name}'; skipping.")
            continue
        if stage is not None and not stage.GetPrimAtPath(prim_path).IsValid():
            print(f"[viewport_setup] Warning: camera prim not found for '{name}': {prim_path}")
            continue
        try:
            windows.append(_make_viewport(name, prim_path, position,
                                          layout["width"], layout["height"]))
            print(f"[viewport_setup] {name} viewport -> {prim_path}")
        except Exception as exc:
            print(f"[viewport_setup] Warning: '{name}' viewport failed: {exc}")

    _dock_rover_viewports(windows, layout)
    print(f"[viewport_setup] Perspectives ready: free-fly Perspective + "
          f"{len(windows)} locked on-board viewport(s).")
    return windows
