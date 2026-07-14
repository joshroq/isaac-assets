"""Isaac launcher: small warehouse + Replicator obstacles only.

    isaacpy ugv_controller_v2/nav_experiment/run_random_small_warehouse.py
    isaacpy ugv_controller_v2/nav_experiment/run_random_small_warehouse.py --headless
    isaacpy ugv_controller_v2/nav_experiment/run_random_small_warehouse.py --random-seed 1234
    isaacpy ugv_controller_v2/nav_experiment/run_random_small_warehouse.py --obstacle-style props

This is a map-generation/debug launcher. It opens ``small_warehouse.usd``,
scatters randomized static obstacles with NVIDIA Replicator, and saves a baked
USD that the nav runner can load without Replicator active in the live rover
process. It intentionally does not load the rover and does not start ROS 2/Nav2.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
V2_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from random_small_warehouse_obstacles import (  # noqa: E402
    add_random_obstacle_args,
    obstacle_config_from_args,
    scatter_random_obstacles,
)


def _find_asset_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / "small_warehouse.usd").exists():
            return candidate
    return start


ASSET_ROOT = _find_asset_root(SCRIPT_DIR)
WAREHOUSE_USD = ASSET_ROOT / "small_warehouse.usd"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "generated"


def _slug_number(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def default_generated_usd(args) -> Path:
    seed = "auto" if args.random_seed is None else str(args.random_seed)
    return DEFAULT_OUTPUT_DIR / (
        "small_warehouse_random_"
        f"seed-{seed}_"
        f"count-{max(0, args.random_obstacle_count)}_"
        f"style-{getattr(args, 'obstacle_style', 'primitives')}_"
        f"spacing-{_slug_number(max(0.0, args.random_min_spacing))}_"
        f"x-{_slug_number(args.random_x_min)}-{_slug_number(args.random_x_max)}_"
        f"y-{_slug_number(args.random_y_min)}-{_slug_number(args.random_y_max)}.usd"
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open small_warehouse.usd with randomized Replicator obstacles."
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without the Isaac viewport.",
    )
    parser.add_argument(
        "--output-usd",
        type=Path,
        default=None,
        help="Path for the baked randomized warehouse USD. Defaults under nav_experiment/generated/.",
    )
    parser.add_argument(
        "--keep-open",
        action="store_true",
        help="After saving, keep Isaac open for visual inspection instead of exiting.",
    )
    add_random_obstacle_args(parser)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if not WAREHOUSE_USD.exists():
        print(f"[run_random_small_warehouse] Missing warehouse USD: {WAREHOUSE_USD}")
        return 1
    if WAREHOUSE_USD.stat().st_size == 0:
        print(f"[run_random_small_warehouse] Empty warehouse USD: {WAREHOUSE_USD}")
        return 1

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    try:
        import omni.usd
        from isaacsim.core.api import World

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

        authored = scatter_random_obstacles(
            simulation_app,
            **obstacle_config_from_args(args),
        )

        output_usd = (args.output_usd or default_generated_usd(args)).expanduser().resolve()
        output_usd.parent.mkdir(parents=True, exist_ok=True)
        if not stage.GetRootLayer().Export(str(output_usd)):
            raise RuntimeError(f"Could not export randomized warehouse USD: {output_usd}")
        print(f"[run_random_small_warehouse] Saved USD: {output_usd}", flush=True)

        if not args.keep_open:
            print("[run_random_small_warehouse] Save complete; exiting.", flush=True)
            return 0

        world = World()
        world.reset()
        world.play()

        print("[run_random_small_warehouse] Setup complete.", flush=True)
        print(f"[run_random_small_warehouse] Warehouse USD: {WAREHOUSE_USD}", flush=True)
        print(f"[run_random_small_warehouse] Obstacles:     {authored}", flush=True)
        print(f"[run_random_small_warehouse] Window:        {'headless' if args.headless else 'visible'}", flush=True)
        print("[run_random_small_warehouse] No rover / ROS 2 / Nav2.", flush=True)

        while simulation_app.is_running():
            world.step(render=True)
    finally:
        simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
