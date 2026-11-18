#!/usr/bin/env python3
"""Repair the WHEEL physics of jetson_rover_v2.usd (attribute/binding-only, standalone pxr).

WHY (both found by sim2real_calib/docs/PARAMETER_TEST_VALIDATION.md, 2026-10-14)
    1. WRONG FRICTION MATERIALS. Each wheel collider prim
       `<wheel>_link/collisions/<mesh>/node_STL_BINARY_` carries its own
       `material:binding:physics`, which beats the `collisions` Xform's all-purpose binding.
       Five of the six point at /World/Physics_Materials/middle_wheel — including three DRIVEN
       corner wheels (left_up, left_down, right_down). Only right_up resolved to corner_wheel.
       So `corner_mu_*` tuned ONE tyre and `middle_mu_*` tuned five (3 driven + 2 passive), and
       the left/right friction asymmetry made in-place spins walk.

    2. WHEEL INERTIA ~2.6x TOO LARGE. The URDF (CAD export) gives each wheel 0.2071 kg with the
       inertia of a uniform hollow cylinder of THAT mass (r_o=0.04, r_hub=0.007, w=0.0425):
           I_spin = m (r_o^2 + r_i^2) / 2 = 1.707e-4 kg m^2
       The USD later overrode the mass to 0.08 kg but kept the inertia, so I_spin > m r_o^2
       (1.28e-4) — physically impossible for a 4 cm wheel. Reflected into the chassis it added
       6 I/r^2 = 0.64 kg of phantom translational inertia (now 0.25 kg).

    NOT changed here: the wheel collider is a 38-sided prism (76 rim vertices) cooked as a
    convexHull with PhysX's default 64-vertex limit, so the cooked hull may drop a few rim
    vertices (worst case ~0.5 mm rolling ripple). The cooked hull can't be queried for these
    Xform-hosted colliders and GPU-compatible cooking caps hulls at 64 anyway, so it is left
    as-is (see the validation doc's suggestions).

HOW
    Per wheel: author `material:binding:physics` (weakerThanDescendants, like the original) on
    every collider prim -> corner_wheel for the 4 driven corner wheels, middle_wheel for the 2
    passive middle wheels; set diagonalInertia from the collision-mesh geometry and the authored
    mass (uniform-density hollow cylinder, same assumption as the CAD export). Idempotent: a re-run reports no changes.

    The calibration schema (calib_params.py) authors wheel MASS only. If you ever change
    `wheel_mass`, re-run this tool so the inertia follows it.

USAGE (needs pxr; no Isaac app required)
    ugv_controller_v2/tools/fix_wheel_physics.sh             # repair + back up + save
    ugv_controller_v2/tools/fix_wheel_physics.sh --dry-run   # report only, author nothing
"""

import argparse
import datetime
import math
import shutil
import sys

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
except ImportError:
    sys.exit(
        "pxr not importable. Run with Isaac's USD libs, e.g.:\n"
        "  ugv_controller_v2/tools/fix_wheel_physics.sh"
    )

DEFAULT_USD = "/home/joshroq/projects/isaac-assets/jetson_rover_v2.usd"
ROVER = "/World/ugv_rover"
MAT_CORNER = "/World/Physics_Materials/corner_wheel"
MAT_MIDDLE = "/World/Physics_Materials/middle_wheel"
# Driven corner wheels (velocity drives) vs passive middle wheels (maxForce 0).
WHEEL_MATERIAL = {
    "left_up": MAT_CORNER, "left_down": MAT_CORNER,
    "right_up": MAT_CORNER, "right_down": MAT_CORNER,
    "left_middle": MAT_MIDDLE, "right_middle": MAT_MIDDLE,
}


def wheel_colliders(stage, link):
    return [p for p in Usd.PrimRange(link) if p.HasAPI(UsdPhysics.CollisionAPI)]


def mesh_cylinder_dims(stage, link, collider):
    """(r_outer, r_inner, width) of the wheel mesh in the LINK frame, spin axis = link Z
    (the revolute joint axis; see the joint's localRot1)."""
    to_link = UsdGeom.Xformable(link).ComputeLocalToWorldTransform(0).GetInverse()
    pts = []
    for p in Usd.PrimRange(collider):
        if p.IsA(UsdGeom.Mesh):
            xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0) * to_link
            pts += [xf.Transform(q) for q in UsdGeom.Mesh(p).GetPointsAttr().Get()]
    radii = [math.hypot(q[0], q[1]) for q in pts]
    zs = [q[2] for q in pts]
    return max(radii), min(radii), max(zs) - min(zs)


def set_if_changed(attr, value, changes, label, tol=1e-9):
    before = attr.Get()
    same = before is not None and (
        all(abs(a - b) <= tol for a, b in zip(before, value)) if hasattr(value, "__len__")
        else abs(before - value) <= tol if isinstance(value, float) else before == value)
    if not same:
        attr.Set(value)
        changes.append((label, before, value))


def fix(stage):
    changes = []
    for wheel, mat_path in WHEEL_MATERIAL.items():
        link = stage.GetPrimAtPath(f"{ROVER}/{wheel}_wheel_link")
        if not link or not link.IsValid():
            sys.exit(f"missing wheel link {wheel}")
        material = UsdShade.Material(stage.GetPrimAtPath(mat_path))
        colliders = wheel_colliders(stage, link)
        if not colliders:
            sys.exit(f"no collider under {link.GetPath()}")

        # 1. physics material
        for col in colliders:
            bound, _ = UsdShade.MaterialBindingAPI(col).ComputeBoundMaterial("physics")
            if not bound or bound.GetPath() != material.GetPath():
                UsdShade.MaterialBindingAPI.Apply(col)
                UsdShade.MaterialBindingAPI(col).Bind(
                    material, UsdShade.Tokens.weakerThanDescendants, "physics")
                changes.append((f"{col.GetPath()} physics material",
                                str(bound.GetPath()) if bound else None, mat_path))

        # 2. inertia consistent with the authored mass
        mass_api = UsdPhysics.MassAPI(link)
        m = mass_api.GetMassAttr().Get()
        r_o, r_i, w = mesh_cylinder_dims(stage, link, colliders[0])
        i_spin = m * (r_o ** 2 + r_i ** 2) / 2.0
        i_trans = m * (3.0 * (r_o ** 2 + r_i ** 2) + w ** 2) / 12.0
        # principalAxes maps principal Y -> link Z (the spin axis); keep it, so spin goes in Y.
        set_if_changed(mass_api.GetDiagonalInertiaAttr(),
                       Gf.Vec3f(i_trans, i_spin, i_trans), changes,
                       f"{link.GetPath()} diagonalInertia (m={m:.4g}, r={r_o:.4f}/{r_i:.4f}, w={w:.4f})",
                       tol=1e-10)

    return changes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", default=DEFAULT_USD)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        sys.exit(f"could not open {args.usd}")
    changes = fix(stage)
    if not changes:
        print("Nothing to change (wheel physics already repaired).")
        return 0
    for label, before, after in changes:
        print(f"  {'DRY ' if args.dry_run else 'SET '} {label}: {before!r} -> {after!r}")
    if args.dry_run:
        return 0
    backup = f"{args.usd}.bak-{datetime.datetime.now():%Y%m%d-%H%M%S}-wheels"
    shutil.copy2(args.usd, backup)
    print(f"backup: {backup}")
    stage.GetRootLayer().Save()
    print(f"saved {args.usd}  ({len(changes)} changes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
