"""Random static obstacles for the v2 small-warehouse scenarios.

This is the v2 version of the experimental Replicator obstacle generator. It
only authors fixed, collidable obstacles into the already-open Isaac stage; the
rover, ROS 2 graphs, and Nav2 are still owned by the baked rover USD and the
v2 nav launch files.

Obstacles are created with NVIDIA Replicator and materialized once before
physics starts:

* ``props`` style (the DEFAULT) references real NVIDIA Simple_Warehouse / YCB
  assets — pallets, card boxes, KLT bins, traffic cones — so the scene looks
  like a warehouse instead of coloured blocks. Each prop URL is resolved against
  the Isaac asset root before use; if a single asset cannot be resolved (offline
  Nucleus, renamed asset), that obstacle falls back to a same-footprint
  primitive so one missing file never aborts the whole layout.
* ``primitives`` style creates dependency-free boxes and cylinders (no asset
  server needed) — useful for fully offline CI.
* A deterministic rejection sampler places obstacles inside the requested X/Y
  bounds with one global non-overlap check across every template type.
* ``rep.physics.collider`` makes each obstacle a static collision object.
"""

from __future__ import annotations

from dataclasses import dataclass
import random


ROOT_PATH = "/World/RandomObstacles"
_MAX_LOAD_FRAMES = 900
_MAX_PLACEMENT_ATTEMPTS_PER_OBSTACLE = 500
_REPLICATOR_ONE_SHOT_PATHS = ("/Orchestrator", "/WriterOrchestrator")


@dataclass(frozen=True)
class ObstacleTemplate:
    name: str
    footprint: tuple[float, float]
    height: float
    weight: float = 1.0
    shape: str = "box"
    color: tuple[float, float, float] = (0.75, 0.35, 0.15)
    url: str = ""
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass(frozen=True)
class ObstaclePlacement:
    template: ObstacleTemplate
    x: float
    y: float
    yaw_deg: float
    half_width: float
    half_depth: float


PRIMITIVE_TEMPLATES: tuple[ObstacleTemplate, ...] = (
    ObstacleTemplate("pallet_box", (1.20, 1.00), 0.30, weight=2.0, color=(0.55, 0.35, 0.18)),
    ObstacleTemplate("stack", (0.60, 0.50), 0.70, weight=4.0, color=(0.72, 0.55, 0.32)),
    ObstacleTemplate("bin", (0.60, 0.40), 0.32, weight=2.0, color=(0.20, 0.45, 0.70)),
    ObstacleTemplate("cone", (0.40, 0.40), 0.72, weight=1.5, shape="cylinder", color=(0.90, 0.45, 0.05)),
    ObstacleTemplate("drum", (0.58, 0.58), 0.90, weight=1.5, shape="cylinder", color=(0.30, 0.55, 0.30)),
    ObstacleTemplate("post", (0.30, 0.30), 1.20, weight=1.0, shape="cylinder", color=(0.65, 0.65, 0.70)),
)


# Real NVIDIA warehouse/YCB assets. Each carries a footprint/height/shape/colour
# so it can fall back to a matching primitive if the asset URL cannot be resolved
# against the live Isaac asset root (offline Nucleus or a renamed asset). `shape`
# and `color` are ONLY used by that fallback; the resolved prop uses its own mesh.
PROP_TEMPLATES: tuple[ObstacleTemplate, ...] = (
    ObstacleTemplate("pallet", (1.30, 1.05), 0.18, weight=2.0, color=(0.55, 0.35, 0.18),
                     url="/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd"),
    ObstacleTemplate("cardbox_d", (0.62, 0.48), 0.50, weight=3.0, color=(0.72, 0.55, 0.32),
                     url="/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd"),
    ObstacleTemplate("cardbox_a", (0.52, 0.42), 0.40, weight=3.0, color=(0.70, 0.52, 0.30),
                     url="/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01.usd"),
    ObstacleTemplate("klt_bin", (0.60, 0.40), 0.25, weight=2.0, color=(0.20, 0.45, 0.70),
                     url="/Isaac/Props/KLT_Bin/small_KLT_visual.usd"),
    ObstacleTemplate("traffic_cone", (0.45, 0.45), 0.72, weight=1.5, shape="cylinder",
                     color=(0.90, 0.45, 0.05),
                     url="/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd"),
    ObstacleTemplate("cracker_box", (0.16, 0.06), 0.21, weight=1.0, scale=(2.0, 2.0, 2.0),
                     color=(0.80, 0.20, 0.15),
                     url="/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"),
)


def _templates_for(style: str) -> tuple[ObstacleTemplate, ...]:
    return PROP_TEMPLATES if style == "props" else PRIMITIVE_TEMPLATES


def _allocate_counts(
    templates: tuple[ObstacleTemplate, ...], count: int
) -> list[tuple[ObstacleTemplate, int]]:
    if count <= 0 or not templates:
        return []
    total_weight = sum(max(0.0, t.weight) for t in templates) or float(len(templates))
    raw = [(t, count * max(0.0, t.weight) / total_weight) for t in templates]
    floored = [(t, int(v)) for t, v in raw]
    remainder = count - sum(n for _, n in floored)
    order = sorted(range(len(raw)), key=lambda i: raw[i][1] - floored[i][1], reverse=True)
    counts = [n for _, n in floored]
    for i in order[:remainder]:
        counts[i] += 1
    return [(templates[i], counts[i]) for i in range(len(templates)) if counts[i] > 0]


def _ensure_root(stage) -> None:
    from pxr import UsdGeom

    if stage.GetPrimAtPath(ROOT_PATH).IsValid():
        stage.RemovePrim(ROOT_PATH)
    UsdGeom.Scope.Define(stage, ROOT_PATH)


def _footprint_half_extents(
    template: ObstacleTemplate, min_spacing: float
) -> tuple[float, float]:
    w, d = template.footprint
    return 0.5 * (w + min_spacing), 0.5 * (d + min_spacing)


def _footprint_sort_area(template: ObstacleTemplate, min_spacing: float) -> float:
    half_w, half_d = _footprint_half_extents(template, min_spacing)
    return half_w * half_d


def _place_obstacles(
    allocation: list[tuple[ObstacleTemplate, int]],
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    min_spacing: float,
    seed: int | None,
) -> list[ObstaclePlacement]:
    """Pick globally non-overlapping X/Y positions before Replicator authors USD."""
    rng = random.Random(seed)
    pending: list[ObstacleTemplate] = []
    for template, n in allocation:
        pending.extend([template] * n)
    pending.sort(key=lambda t: _footprint_sort_area(t, min_spacing), reverse=True)

    placements: list[ObstaclePlacement] = []
    for template in pending:
        half_w, half_d = _footprint_half_extents(template, min_spacing)
        min_x = x_range[0] + half_w
        max_x = x_range[1] - half_w
        min_y = y_range[0] + half_d
        max_y = y_range[1] - half_d
        if min_x > max_x or min_y > max_y:
            print(
                "[random_small_warehouse_obstacles] "
                f"skip {template.name}: footprint does not fit inside bounds",
                flush=True,
            )
            continue

        placed = False
        for _ in range(_MAX_PLACEMENT_ATTEMPTS_PER_OBSTACLE):
            x = rng.uniform(min_x, max_x)
            y = rng.uniform(min_y, max_y)
            if all(
                abs(x - p.x) >= half_w + p.half_width
                or abs(y - p.y) >= half_d + p.half_depth
                for p in placements
            ):
                placements.append(
                    ObstaclePlacement(
                        template=template,
                        x=x,
                        y=y,
                        yaw_deg=rng.uniform(-180.0, 180.0),
                        half_width=half_w,
                        half_depth=half_d,
                    )
                )
                placed = True
                break

        if not placed:
            print(
                "[random_small_warehouse_obstacles] "
                f"skip {template.name}: no collision-free placement after "
                f"{_MAX_PLACEMENT_ATTEMPTS_PER_OBSTACLE} attempts",
                flush=True,
            )

    return placements


def _apply_static_pose_and_collider(rep, group, placement: ObstaclePlacement, *, is_prop: bool):
    template = placement.template
    if is_prop:
        # Real prop meshes are irregular; a convex hull tracks their shape far
        # better than an axis-aligned box, and their pivot sits on the floor.
        approximation = "convexHull"
        z = 0.0
    else:
        # Replicator primitives are centred on their origin, so lift by half height.
        approximation = "convexHull" if template.shape == "cylinder" else "boundingCube"
        z = template.height * 0.5
    with group:
        rep.modify.pose(
            position=(placement.x, placement.y, z),
            rotation=(0.0, 0.0, placement.yaw_deg),
        )
        rep.physics.collider(approximation_shape=approximation)


def _build_primitive_group(rep, template, index):
    w, d = template.footprint
    if template.shape == "cylinder":
        return rep.create.cylinder(
            scale=(max(w, d), max(w, d), template.height),
            parent=ROOT_PATH,
            name=f"obstacle_{template.name}_{index:03d}",
        )
    return rep.create.cube(
        scale=(w, d, template.height),
        parent=ROOT_PATH,
        name=f"obstacle_{template.name}_{index:03d}",
    )


def _asset_url_exists(full_url: str) -> bool:
    """True if a Nucleus/local asset URL resolves. Never raises — a lookup
    failure just means 'fall back to a primitive for this obstacle'."""
    try:
        import omni.client

        result, _ = omni.client.stat(full_url)
        return result == omni.client.Result.OK
    except Exception:
        return False


def _build_prop_group(rep, assets_root_path, template, index):
    """Return (group, is_prop). Resolves the real warehouse asset; if it cannot
    be found, builds a same-footprint primitive so one bad URL never aborts the
    layout."""
    full_url = assets_root_path + template.url
    if _asset_url_exists(full_url):
        group = rep.create.from_usd(
            full_url,
            scale=template.scale,
            parent=ROOT_PATH,
            name=f"obstacle_{template.name}_{index:03d}",
        )
        return group, True
    print(
        "[random_small_warehouse_obstacles] "
        f"WARN {template.name}: asset not found ({full_url}); using primitive fallback",
        flush=True,
    )
    return _build_primitive_group(rep, template, index), False


def _wait_for_stage_ready(simulation_app, max_frames: int = _MAX_LOAD_FRAMES) -> int:
    import omni.usd

    ctx = omni.usd.get_context()
    frames = 0
    for _ in range(3):
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


def _cleanup_replicator_one_shot_graphs(stage) -> None:
    """Remove Replicator's one-shot orchestration graphs after obstacle preview.

    The baked camera/depth publishers also use SyntheticData/Replicator writer
    infrastructure. Leaving the obstacle-generation orchestrators on the stage
    after preview can make those post-process graphs share more state than this
    experiment needs. The static obstacle prims remain under ROOT_PATH.
    """
    for path in _REPLICATOR_ONE_SHOT_PATHS:
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)


def scatter_random_obstacles(
    simulation_app,
    *,
    count: int = 14,
    obstacle_style: str = "props",
    x_range: tuple[float, float] = (-5.0, 5.0),
    y_range: tuple[float, float] = (-5.0, 5.0),
    seed: int | None = None,
    min_spacing: float = 0.45,
) -> int:
    """Scatter randomized static obstacles into the current small-warehouse stage."""
    import omni.replicator.core as rep
    import omni.usd

    style = "primitives" if str(obstacle_style).lower() == "primitives" else "props"
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
            print(
                "[random_small_warehouse_obstacles] "
                "WARN Isaac asset root unavailable; every prop will fall back to a "
                "primitive. Configure the Nucleus/asset server for real warehouse "
                "assets, or pass --obstacle-style primitives to silence this.",
                flush=True,
            )

    placements = _place_obstacles(allocation, x_range, y_range, min_spacing, seed)

    authored = 0
    prop_hits = 0
    placement_counts: dict[str, int] = {}
    for index, placement in enumerate(placements, start=1):
        template = placement.template
        if style == "props":
            group, is_prop = _build_prop_group(rep, assets_root_path, template, index)
        else:
            group, is_prop = _build_primitive_group(rep, template, index), False
        _apply_static_pose_and_collider(rep, group, placement, is_prop=is_prop)
        placement_counts[template.name] = placement_counts.get(template.name, 0) + 1
        prop_hits += int(is_prop)
        authored += 1

    rep.orchestrator.preview()
    frames = _wait_for_stage_ready(simulation_app)
    _cleanup_replicator_one_shot_graphs(stage)
    simulation_app.update()

    real_props = f" real_props={prop_hits}/{authored}" if style == "props" else ""
    print(
        "[random_small_warehouse_obstacles] "
        f"style={style} seed={'auto' if seed is None else seed} "
        f"authored={authored}/{count}{real_props} templates={len(allocation)} "
        f"load_frames={frames} min_spacing={min_spacing} "
        f"x_range={x_range} y_range={y_range}",
        flush=True,
    )
    for template, _ in allocation:
        n = placement_counts.get(template.name, 0)
        if n:
            print(f"[random_small_warehouse_obstacles]   {n:2d}x {template.name}", flush=True)
    return authored


def add_random_obstacle_args(parser, *, default_count: int = 14) -> None:
    parser.add_argument(
        "--obstacle-style",
        choices=("primitives", "props"),
        default="props",
        help="Obstacle visuals: props (real NVIDIA warehouse assets, the default) "
             "or primitives (dependency-free boxes/cylinders for offline use).",
    )
    parser.add_argument(
        "--random-obstacle-count",
        type=int,
        default=default_count,
        help="Number of launch-time static obstacles to scatter.",
    )
    parser.add_argument(
        "--random-seed",
        type=int,
        default=None,
        help="Reproduce a random obstacle layout. Omit for a fresh layout.",
    )
    parser.add_argument("--random-x-min", type=float, default=-5.0)
    parser.add_argument("--random-x-max", type=float, default=5.0)
    parser.add_argument("--random-y-min", type=float, default=-5.0)
    parser.add_argument("--random-y-max", type=float, default=5.0)
    parser.add_argument(
        "--random-min-spacing",
        type=float,
        default=0.45,
        help="Minimum extra spacing between obstacle footprints.",
    )


def obstacle_config_from_args(args) -> dict:
    x_range = tuple(sorted((args.random_x_min, args.random_x_max)))
    y_range = tuple(sorted((args.random_y_min, args.random_y_max)))
    return {
        "count": max(0, args.random_obstacle_count),
        "obstacle_style": getattr(args, "obstacle_style", "props"),
        "x_range": x_range,
        "y_range": y_range,
        "seed": args.random_seed,
        "min_spacing": max(0.0, args.random_min_spacing),
    }
