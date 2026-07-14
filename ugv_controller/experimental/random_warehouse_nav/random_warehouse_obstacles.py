"""Launch-time random static obstacles, scattered with NVIDIA Replicator.

Scope note
----------
This module is intentionally NARROW: it only scatters randomized static
obstacles into an already-open warehouse stage. There is no rover, no ROS 2, and
no navigation wiring here — all of that was removed so the map + obstacle
generation can be verified on its own first (see ``run_random_warehouse.py``).

How it works
------------
Placement and randomization are done through ``omni.replicator.core`` (Replicator),
NVIDIA's domain-randomization framework — not hand-authored USD transforms:

* Obstacles are created with ``rep.create.cube`` / ``rep.create.cylinder``
  (``primitives`` style) or ``rep.create.from_usd`` (``props`` style).
* They are placed with ``rep.randomizer.scatter_2d`` onto an invisible ground
  surface, bounded to the requested X/Y region, with per-group collision checking
  so obstacles in the same group do not interpenetrate.
* Their yaw is randomized with ``rep.modify.pose`` +
  ``rep.distribution.uniform``.
* ``rep.physics.collider`` gives each obstacle a STATIC collider (collision but no
  rigid body), so they read as fixed geometry to any future lidar / camera / Nav2
  layer and never fall or drift when the sim plays.

The whole graph is materialized once with ``rep.orchestrator.preview()`` before
physics starts, so the map opens with obstacles already in place.

``seed=None`` chooses a fresh layout each run; pass ``--random-seed`` (which calls
``rep.set_global_seed``) to reproduce a layout.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass


ROOT_PATH = "/World/RandomObstacles"
SURFACE_PATH = f"{ROOT_PATH}/_scatter_surface"

# Bounded so waiting on referenced payloads (props style) can never hang forever.
_MAX_LOAD_FRAMES = 900


@dataclass(frozen=True)
class ObstacleTemplate:
    name: str
    footprint: tuple[float, float]  # (x_width, y_depth) of the collidable footprint
    height: float
    weight: float = 1.0
    # primitives style
    shape: str = "box"  # "box" | "cylinder"
    color: tuple[float, float, float] = (0.75, 0.35, 0.15)
    # props style
    url: str = ""
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


# ── Primitive obstacles: no external references, cheap analytic collision ────────
# Sizes loosely mimic the warehouse props they are named after so a layout tuned on
# primitives transfers to props style.
PRIMITIVE_TEMPLATES: tuple[ObstacleTemplate, ...] = (
    ObstacleTemplate(
        name="pallet_box", footprint=(1.20, 1.00), height=0.30, weight=2.0,
        shape="box", color=(0.55, 0.35, 0.18),
    ),
    ObstacleTemplate(
        name="stack", footprint=(0.60, 0.50), height=0.70, weight=4.0,
        shape="box", color=(0.72, 0.55, 0.32),
    ),
    ObstacleTemplate(
        name="bin", footprint=(0.60, 0.40), height=0.32, weight=2.0,
        shape="box", color=(0.20, 0.45, 0.70),
    ),
    ObstacleTemplate(
        name="cone", footprint=(0.40, 0.40), height=0.72, weight=1.5,
        shape="cylinder", color=(0.90, 0.45, 0.05),
    ),
    ObstacleTemplate(
        name="drum", footprint=(0.58, 0.58), height=0.90, weight=1.5,
        shape="cylinder", color=(0.30, 0.55, 0.30),
    ),
    ObstacleTemplate(
        name="post", footprint=(0.30, 0.30), height=1.20, weight=1.0,
        shape="cylinder", color=(0.65, 0.65, 0.70),
    ),
)


# ── Prop obstacles: reference real NVIDIA assets (visuals), forced static ────────
PROP_TEMPLATES: tuple[ObstacleTemplate, ...] = (
    ObstacleTemplate(
        name="pallet", footprint=(1.30, 1.05), height=0.18, weight=2.0,
        url="/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd",
    ),
    ObstacleTemplate(
        name="cardbox", footprint=(0.62, 0.48), height=0.50, weight=4.0,
        url="/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd",
    ),
    ObstacleTemplate(
        name="klt_bin", footprint=(0.60, 0.40), height=0.25, weight=2.0,
        url="/Isaac/Props/KLT_Bin/small_KLT_visual.usd",
    ),
    ObstacleTemplate(
        name="traffic_cone", footprint=(0.45, 0.45), height=0.72, weight=1.0,
        url="/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd",
    ),
    ObstacleTemplate(
        name="cracker_box", footprint=(0.26, 0.08), height=0.34, weight=1.0,
        scale=(2.0, 2.0, 2.0),
        url="/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
    ),
)


def _templates_for(style: str) -> tuple[ObstacleTemplate, ...]:
    return PROP_TEMPLATES if style == "props" else PRIMITIVE_TEMPLATES


def _allocate_counts(
    templates: tuple[ObstacleTemplate, ...], count: int
) -> "list[tuple[ObstacleTemplate, int]]":
    """Split ``count`` obstacles across the templates, weighted, deterministically.

    Weighting is purely proportional (largest-remainder rounding); it does not use
    an RNG, so which templates appear is stable while Replicator still randomizes
    where each instance lands and how it is rotated.
    """
    if count <= 0 or not templates:
        return []
    total_weight = sum(max(0.0, t.weight) for t in templates) or float(len(templates))
    raw = [(t, count * max(0.0, t.weight) / total_weight) for t in templates]
    floored = [(t, int(v)) for t, v in raw]
    remainder = count - sum(n for _, n in floored)
    # Hand out the leftover to the largest fractional parts.
    order = sorted(range(len(raw)), key=lambda i: raw[i][1] - floored[i][1], reverse=True)
    counts = [n for _, n in floored]
    for i in order[:remainder]:
        counts[i] += 1
    return [(templates[i], counts[i]) for i in range(len(templates)) if counts[i] > 0]


# ── Replicator scatter ───────────────────────────────────────────────────────────
def _ensure_root(stage) -> None:
    from pxr import UsdGeom

    if stage.GetPrimAtPath(ROOT_PATH).IsValid():
        stage.RemovePrim(ROOT_PATH)
    UsdGeom.Scope.Define(stage, ROOT_PATH)


def _create_scatter_surface(rep, x_range, y_range):
    """A large invisible ground plane for scatter_2d to sample on.

    Placement is bounded independently via scatter_2d's ``min_samp``/``max_samp``,
    so the plane only needs to be big enough to cover the region.
    """
    cx = 0.5 * (x_range[0] + x_range[1])
    cy = 0.5 * (y_range[0] + y_range[1])
    span = max(x_range[1] - x_range[0], y_range[1] - y_range[0]) + 4.0
    return rep.create.plane(
        position=(cx, cy, 0.0),
        scale=(span, span, 1.0),
        visible=False,
        name="obstacle_scatter_surface",
        parent=ROOT_PATH,
    )


def _scatter_group(rep, group, surface, template, x_range, y_range, seed):
    """Scatter one obstacle group onto the surface and make it a static collider."""
    approximation = "boundingCube" if template.shape != "cylinder" else "convexHull"
    with group:
        rep.randomizer.scatter_2d(
            surface_prims=surface,
            min_samp=(x_range[0], y_range[0], None),
            max_samp=(x_range[1], y_range[1], None),
            offset=template.height * 0.5,  # lift so the base sits on the floor
            check_for_collisions=1,        # no interpenetration within this group
            seed=seed,
        )
        rep.modify.pose(
            rotation=rep.distribution.uniform((0.0, 0.0, -180.0), (0.0, 0.0, 180.0)),
        )
        rep.physics.collider(approximation_shape=approximation)


def _build_primitive_group(rep, template, n):
    w, d = template.footprint
    if template.shape == "cylinder":
        # Unit cylinder is radius 0.5 (diameter 1), height 1 → scale by footprint/height.
        return rep.create.cylinder(
            scale=(max(w, d), max(w, d), template.height),
            count=n, parent=ROOT_PATH, name=f"obstacle_{template.name}",
        )
    return rep.create.cube(
        scale=(w, d, template.height),
        count=n, parent=ROOT_PATH, name=f"obstacle_{template.name}",
    )


def _build_prop_group(rep, assets_root_path, template, n):
    return rep.create.from_usd(
        assets_root_path + template.url,
        scale=template.scale,
        count=n, parent=ROOT_PATH, name=f"obstacle_{template.name}",
    )


def _wait_for_stage_ready(simulation_app, max_frames: int = _MAX_LOAD_FRAMES) -> int:
    """Pump the app until no payloads are still loading (bounded).

    Cooking PhysX / starting RTX on half-streamed geometry or materials is a
    classic hard-crash source; block here so downstream play() sees resolved prims.
    """
    import omni.usd

    ctx = omni.usd.get_context()
    frames = 0
    for _ in range(3):  # warm-up so freshly added references register as loading
        simulation_app.update()
        frames += 1
    while frames < max_frames:
        try:
            _, _, loading = ctx.get_stage_loading_status()
        except Exception:
            loading = 0
        if not loading:
            break
        simulation_app.update()
        frames += 1
    return frames


def scatter_random_obstacles(
    simulation_app,
    *,
    count: int = 12,
    obstacle_style: str = "primitives",
    x_range: tuple[float, float] = (-9.0, 9.0),
    y_range: tuple[float, float] = (-9.0, 9.0),
    seed: "int | None" = None,
    min_spacing: float = 1.35,  # accepted for CLI symmetry; scatter uses collision checks
) -> int:
    """Scatter randomized static obstacles into the current stage with Replicator.

    Call this AFTER the environment stage is open and BEFORE physics plays. Returns
    the number of obstacles authored.
    """
    import omni.replicator.core as rep
    import omni.usd

    style = "props" if str(obstacle_style).lower() == "props" else "primitives"
    templates = _templates_for(style)
    allocation = _allocate_counts(templates, max(0, count))

    stage = omni.usd.get_context().get_stage()
    _ensure_root(stage)

    if seed is not None:
        rep.set_global_seed(int(seed))

    assets_root_path = ""
    if style == "props":
        from isaacsim.storage.native import get_assets_root_path

        assets_root_path = get_assets_root_path() or ""
        if not assets_root_path:
            raise RuntimeError(
                "obstacle_style='props' needs the NVIDIA Isaac asset root, but it "
                "could not be resolved (asset server unreachable). Re-run with "
                "--obstacle-style primitives, or fix the asset server / local cache."
            )

    surface = _create_scatter_surface(rep, x_range, y_range)

    authored = 0
    for template, n in allocation:
        if style == "props":
            group = _build_prop_group(rep, assets_root_path, template, n)
        else:
            group = _build_primitive_group(rep, template, n)
        # A distinct per-group seed keeps groups from stacking in the same spots.
        group_seed = None if seed is None else int(seed) + authored + 1
        _scatter_group(rep, group, surface, template, x_range, y_range, group_seed)
        authored += n

    # Execute the randomization graph once so obstacles are placed before play().
    rep.orchestrator.preview()
    frames = _wait_for_stage_ready(simulation_app)

    print(
        "[random_obstacles] "
        f"style={style} seed={'auto' if seed is None else seed} "
        f"authored={authored}/{count} groups={len(allocation)} "
        f"load_frames={frames} x_range={x_range} y_range={y_range}"
    )
    for template, n in allocation:
        print(f"[random_obstacles]   {n:2d}x {template.name} (h={template.height:.2f})")
    return authored


# ── CLI plumbing (shared by the launcher) ────────────────────────────────────────
def add_random_obstacle_args(parser, *, default_count: int = 12) -> None:
    parser.add_argument(
        "--obstacle-style",
        choices=("primitives", "props"),
        default="primitives",
        help="primitives (default, dependency-free collidable boxes/cylinders) or "
        "props (reference real NVIDIA warehouse/YCB assets; needs an asset server).",
    )
    parser.add_argument(
        "--random-obstacle-count", type=int, default=default_count,
        help="Number of launch-time static obstacles to scatter in the warehouse.",
    )
    parser.add_argument(
        "--random-seed", type=int, default=None,
        help="Reproduce a random obstacle layout. Omit for a fresh layout each launch.",
    )
    parser.add_argument(
        "--random-x-min", type=float, default=-9.0,
        help="Minimum X coordinate for random obstacle placement.",
    )
    parser.add_argument(
        "--random-x-max", type=float, default=9.0,
        help="Maximum X coordinate for random obstacle placement.",
    )
    parser.add_argument(
        "--random-y-min", type=float, default=-9.0,
        help="Minimum Y coordinate for random obstacle placement.",
    )
    parser.add_argument(
        "--random-y-max", type=float, default=9.0,
        help="Maximum Y coordinate for random obstacle placement.",
    )
    parser.add_argument(
        "--random-min-spacing", type=float, default=1.35,
        help="Advisory minimum spacing between obstacles (scatter uses collision checks).",
    )


def obstacle_config_from_args(args) -> dict:
    x_range = tuple(sorted((args.random_x_min, args.random_x_max)))
    y_range = tuple(sorted((args.random_y_min, args.random_y_max)))
    return dict(
        count=max(0, args.random_obstacle_count),
        obstacle_style=getattr(args, "obstacle_style", "primitives"),
        x_range=x_range,
        y_range=y_range,
        seed=args.random_seed,
        min_spacing=max(0.0, args.random_min_spacing),
    )
