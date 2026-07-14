"""Minimal launcher: full warehouse map + randomized obstacles (NVIDIA Replicator).

SCOPE
-----
This is deliberately stripped down to ONE job: open the warehouse map in Isaac Sim
with randomized static obstacles scattered by ``omni.replicator``. There is NO
rover, NO ROS 2, and NO Nav2 here — every navigation-system piece was removed so
the map + obstacle generation can be proven out in isolation first. Once this is
reliable, the rover / sensors / Nav2 can be layered back on top.

Obstacle placement is done with NVIDIA Replicator (see
``random_warehouse_obstacles.py``): obstacles are created and scattered with
``rep.create.*`` + ``rep.randomizer.scatter_2d`` and given static colliders with
``rep.physics.collider``.

USAGE (launch through isaacpy_clean.sh so no system ROS shadows Isaac's Python)
    ./isaacpy_clean.sh ugv_controller/experimental/random_warehouse_nav/run_random_warehouse.py
    ./isaacpy_clean.sh .../run_random_warehouse.py --headless
    ./isaacpy_clean.sh .../run_random_warehouse.py --random-seed 1234 --random-obstacle-count 18
    ./isaacpy_clean.sh .../run_random_warehouse.py --obstacle-style props
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
WAREHOUSE_USD_NAME = "full_warehouse.usd"


def _find_asset_root(start: Path) -> Path:
    """Nearest ancestor of `start` containing the warehouse USD (the repo root)."""
    for candidate in (start, *start.parents):
        if (candidate / WAREHOUSE_USD_NAME).exists():
            return candidate
    return start


ASSET_ROOT = _find_asset_root(SCRIPT_DIR)
WAREHOUSE_USD = ASSET_ROOT / WAREHOUSE_USD_NAME


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open the full warehouse with randomized obstacles (Replicator only; "
        "no rover / ROS 2 / Nav2)."
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without the Isaac viewport (e.g. remote-desktop display trouble).",
    )
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    from random_warehouse_obstacles import add_random_obstacle_args

    add_random_obstacle_args(parser)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    if not WAREHOUSE_USD.exists():
        print(f"[run_random_warehouse] Warehouse USD not found: {WAREHOUSE_USD}")
        return 1
    if WAREHOUSE_USD.stat().st_size == 0:
        print(f"[run_random_warehouse] Warehouse USD is empty (0 bytes): {WAREHOUSE_USD}")
        return 1

    # SimulationApp must be created before importing any other Isaac Sim module.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    try:
        import omni.usd
        from isaacsim.core.api import World

        if str(SCRIPT_DIR) not in sys.path:
            sys.path.insert(0, str(SCRIPT_DIR))
        from random_warehouse_obstacles import (
            obstacle_config_from_args,
            scatter_random_obstacles,
        )

        # ── Open the warehouse map ──────────────────────────────────────────────
        if not omni.usd.get_context().open_stage(str(WAREHOUSE_USD)):
            raise RuntimeError(f"Could not open warehouse USD: {WAREHOUSE_USD}")
        for _ in range(10):
            simulation_app.update()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Isaac Sim did not open the warehouse stage.")
        if not stage.GetPrimAtPath("/World").IsValid():
            from pxr import UsdGeom

            UsdGeom.Xform.Define(stage, "/World")

        # ── Scatter randomized obstacles via Replicator (before physics plays) ──
        try:
            authored = scatter_random_obstacles(
                simulation_app, **obstacle_config_from_args(args)
            )
        except Exception:
            import traceback

            print("[run_random_warehouse] ERROR scattering random obstacles:")
            traceback.print_exc()
            return 1

        # ── Start the sim. Obstacles are static colliders, so nothing moves. ────
        world = World()
        world.reset()
        world.play()

        print("[run_random_warehouse] Setup complete.")
        print(f"[run_random_warehouse] Warehouse USD: {WAREHOUSE_USD}")
        print(f"[run_random_warehouse] Obstacles:     {authored} (style={args.obstacle_style})")
        print(f"[run_random_warehouse] Window:        {'headless' if args.headless else 'visible'}")
        print("[run_random_warehouse] No rover / ROS 2 / Nav2 — map + obstacles only.")

        while simulation_app.is_running():
            world.step(render=True)

    finally:
        simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
