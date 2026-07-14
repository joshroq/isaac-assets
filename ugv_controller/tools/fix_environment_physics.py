#!/usr/bin/env python3
"""
Audit or author static collision + high-friction physics material on warehouse USDs.

Why this exists:
  ugv_sim.py opens small_warehouse.usd / full_warehouse.usd as the stage. It only
  creates Isaac's default ground plane when no environment USD is supplied. That
  means warehouse floor collision and friction need to exist in the warehouse USD
  itself, or be authored as overrides before simulation starts.

Default behavior is conservative: apply only to floor-like meshes. Use
--all-static-meshes if you want every non-rover mesh in the environment to get a
static collider and the same friction material.

Run with Isaac Sim's Python:
  /home/joshroq/Applications/isaacsim/python.sh \
      ugv_controller/tools/fix_environment_physics.py --audit small_warehouse.usd

  /home/joshroq/Applications/isaacsim/python.sh \
      ugv_controller/tools/fix_environment_physics.py --apply small_warehouse.usd full_warehouse.usd

  /home/joshroq/Applications/isaacsim/python.sh \
      ugv_controller/tools/fix_environment_physics.py --apply --all-static-meshes full_warehouse.usd
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path


def _bootstrap_isaac_usd_modules() -> None:
    """Expose Isaac Sim's bundled pxr modules when run from python.sh."""
    roots = []
    env_root = os.environ.get("ISAACSIM_PATH")
    if env_root:
        roots.append(Path(env_root))
    roots.extend([
        Path("/home/joshroq/Applications/isaacsim"),
        Path.home() / "Applications" / "isaacsim",
    ])

    module_paths: list[str] = []
    lib_paths: list[str] = []
    for root in roots:
        extcache = root / "extscache"
        if not extcache.exists():
            continue
        for ext in sorted(extcache.glob("omni.usd.libs-*")) + sorted(
            extcache.glob("omni.usd.schema.physx-*")
        ):
            if (ext / "pxr").exists():
                module_paths.append(str(ext))
            if (ext / "bin").exists():
                lib_paths.append(str(ext / "bin"))

    if lib_paths and os.environ.get("ENV_PHYSICS_BOOTSTRAPPED") != "1":
        env = os.environ.copy()
        old_ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = ":".join(lib_paths + ([old_ld] if old_ld else []))
        env["ENV_PHYSICS_BOOTSTRAPPED"] = "1"
        os.execve(sys.executable, [sys.executable, *sys.argv], env)

    for path in reversed(module_paths):
        if path not in sys.path:
            sys.path.insert(0, path)


_bootstrap_isaac_usd_modules()

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

try:
    from pxr import PhysxSchema
except ImportError:  # pragma: no cover
    PhysxSchema = None


ROVER_PATH_PREFIXES = (
    "/World/ugv_rover",
    "/World/jetson_rover",
)
FLOOR_NAME_HINTS = (
    "floor",
    "ground",
    "concrete",
    "plane",
    "foundation",
    "slab",
)
MATERIAL_PATH = "/World/Physics_Materials/HighFrictionEnvironment"


def is_rover_path(path: str) -> bool:
    return any(path == prefix or path.startswith(prefix + "/") for prefix in ROVER_PATH_PREFIXES)


def mesh_prims(stage: Usd.Stage):
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Mesh" and not is_rover_path(str(prim.GetPath())):
            yield prim


def world_range(cache: UsdGeom.BBoxCache, prim: Usd.Prim):
    try:
        bbox = cache.ComputeWorldBound(prim)
        rng = bbox.ComputeAlignedRange()
        if rng.IsEmpty():
            return None
        return rng
    except Exception:
        return None


def range_size(rng) -> Gf.Vec3d:
    return rng.GetMax() - rng.GetMin()


def looks_floor_like(prim: Usd.Prim, rng, stage_min_z: float) -> bool:
    path_lower = str(prim.GetPath()).lower()
    if any(hint in path_lower for hint in FLOOR_NAME_HINTS):
        return True

    size = range_size(rng)
    if size[0] <= 0.0 or size[1] <= 0.0 or size[2] < 0.0:
        return False

    area_xy = float(size[0] * size[1])
    thin_enough = float(size[2]) <= 0.25
    close_to_stage_bottom = float(rng.GetMin()[2]) <= stage_min_z + 0.30
    broad_enough = area_xy >= 0.5
    return thin_enough and close_to_stage_bottom and broad_enough


def collect_targets(stage: Usd.Stage, all_static_meshes: bool):
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    meshes = []
    stage_min_z = None
    for prim in mesh_prims(stage):
        rng = world_range(cache, prim)
        if rng is None:
            continue
        meshes.append((prim, rng))
        z = float(rng.GetMin()[2])
        stage_min_z = z if stage_min_z is None else min(stage_min_z, z)

    if stage_min_z is None:
        return []
    if all_static_meshes:
        return meshes
    return [(prim, rng) for prim, rng in meshes if looks_floor_like(prim, rng, stage_min_z)]


def ensure_material(stage: Usd.Stage, static_friction: float, dynamic_friction: float):
    UsdGeom.Scope.Define(stage, "/World/Physics_Materials")
    material = UsdShade.Material.Define(stage, MATERIAL_PATH)
    prim = material.GetPrim()

    physics_mat = UsdPhysics.MaterialAPI.Apply(prim)
    physics_mat.CreateStaticFrictionAttr(static_friction)
    physics_mat.CreateDynamicFrictionAttr(dynamic_friction)
    physics_mat.CreateRestitutionAttr(0.0)

    if PhysxSchema is not None:
        physx_mat = PhysxSchema.PhysxMaterialAPI.Apply(prim)
        physx_mat.CreateFrictionCombineModeAttr("average")
        physx_mat.CreateRestitutionCombineModeAttr("average")

    return material


def has_collision(prim: Usd.Prim) -> bool:
    return prim.HasAPI(UsdPhysics.CollisionAPI)


def apply_static_collision(
    prim: Usd.Prim,
    material,
    contact_offset: float,
    rest_offset: float,
) -> None:
    UsdPhysics.CollisionAPI.Apply(prim)
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mesh_collision.CreateApproximationAttr("none")

    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        material,
        UsdShade.Tokens.strongerThanDescendants,
        "physics",
    )

    if PhysxSchema is not None:
        physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx_collision.CreateContactOffsetAttr(contact_offset)
        physx_collision.CreateRestOffsetAttr(rest_offset)


def audit_targets(targets) -> None:
    print(f"Candidate meshes: {len(targets)}")
    for prim, rng in targets[:200]:
        size = range_size(rng)
        print(
            f"  {prim.GetPath()} "
            f"collision={has_collision(prim)} "
            f"minZ={float(rng.GetMin()[2]):.3f} "
            f"size=({float(size[0]):.3f}, {float(size[1]):.3f}, {float(size[2]):.3f})"
        )
    if len(targets) > 200:
        print(f"  ... {len(targets) - 200} more")


def process_usd(path: Path, args) -> None:
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"Could not open USD: {path}")

    targets = collect_targets(stage, all_static_meshes=args.all_static_meshes)
    print(f"\n=== {path} ===")
    print("Mode:", "all non-rover meshes" if args.all_static_meshes else "floor-like meshes")
    audit_targets(targets)

    if not args.apply:
        print("Audit only. Add --apply to author changes.")
        return

    if args.backup:
        backup = path.with_suffix(path.suffix + ".bak")
        if not backup.exists():
            shutil.copy2(path, backup)
            print(f"Backup written: {backup}")
        else:
            print(f"Backup exists, not overwritten: {backup}")

    material = ensure_material(stage, args.static_friction, args.dynamic_friction)
    for prim, _rng in targets:
        apply_static_collision(
            prim,
            material,
            contact_offset=args.contact_offset,
            rest_offset=args.rest_offset,
        )

    stage.GetRootLayer().Save()
    print(
        f"Applied static collision + material to {len(targets)} meshes "
        f"(static={args.static_friction}, dynamic={args.dynamic_friction})."
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("usd", nargs="+", type=Path, help="Environment USD file(s)")
    parser.add_argument("--audit", action="store_true", help="Inspect only")
    parser.add_argument("--apply", action="store_true", help="Write changes")
    parser.add_argument(
        "--all-static-meshes",
        action="store_true",
        help="Apply to every non-rover Mesh, not just floor-like meshes",
    )
    parser.add_argument("--static-friction", type=float, default=1.2)
    parser.add_argument("--dynamic-friction", type=float, default=1.0)
    parser.add_argument("--contact-offset", type=float, default=0.02)
    parser.add_argument("--rest-offset", type=float, default=0.0)
    parser.add_argument("--no-backup", dest="backup", action="store_false")
    parser.set_defaults(backup=True)
    args = parser.parse_args()

    if args.audit and args.apply:
        parser.error("Use either --audit or --apply, not both.")
    if not args.audit and not args.apply:
        args.audit = True

    for usd in args.usd:
        process_usd(usd.expanduser().resolve(), args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
