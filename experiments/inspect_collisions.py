#!/usr/bin/env python3
"""Audit collision state of every geometry prim in an environment USD.

Bare-pxr (no SimulationApp): resolves the locally-authored inline obstacle
geometry. The S3-referenced warehouse shell prims are reported too (as
UNRESOLVED refs) but the point of this tool is the ADDED obstacle shapes.

    ~/Applications/isaacsim/python.sh experiments/inspect_collisions.py \
        --env warehouse_basic_obstacles.usd
"""
from __future__ import annotations
import argparse, glob, os, sys
from pathlib import Path

Usd = UsdGeom = UsdPhysics = Gf = None


def _bootstrap_pxr_standalone() -> None:
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_INSPECT_REEXEC") == "1":
        return
    candidates = []
    for base in (os.environ.get("ISAACSIM_PATH"),
                 os.path.expanduser("~/Applications/isaacsim"),
                 os.path.expanduser("~/.local/share/ov/pkg/isaac-sim")):
        if base:
            candidates += glob.glob(os.path.join(base, "extscache", "omni.usd.libs-*"))
    for lib in sorted(candidates, reverse=True):
        if not os.path.isdir(os.path.join(lib, "pxr")):
            continue
        env = dict(os.environ)
        env["_INSPECT_REEXEC"] = "1"
        env["PYTHONPATH"] = lib + os.pathsep + env.get("PYTHONPATH", "")
        env["LD_LIBRARY_PATH"] = (os.path.join(lib, "bin") + os.pathsep
                                  + env.get("LD_LIBRARY_PATH", ""))
        os.execve(sys.executable, [sys.executable] + sys.argv, env)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", default="warehouse_basic_obstacles.usd")
    args = ap.parse_args()

    _bootstrap_pxr_standalone()
    global Usd, UsdGeom, UsdPhysics, Gf
    from pxr import Usd as _U, UsdGeom as _UG, UsdPhysics as _UP, Gf as _G
    Usd, UsdGeom, UsdPhysics, Gf = _U, _UG, _UP, _G

    env_path = Path(args.env).resolve()
    stage = Usd.Stage.Open(str(env_path))
    if not stage:
        sys.exit(f"Could not open {env_path}")

    print(f"# stage: {env_path}")
    print(f"# defaultPrim: {stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else None}")
    print(f"# upAxis: {UsdGeom.GetStageUpAxis(stage)}  metersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")
    print()

    GEOM_TYPES = ("Mesh", "Cube", "Sphere", "Cylinder", "Cone", "Capsule", "Plane")
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())

    rows = []
    for prim in stage.Traverse():
        tname = prim.GetTypeName()
        has_refs = prim.HasAuthoredReferences()
        is_geom = tname in GEOM_TYPES
        if not (is_geom or has_refs):
            continue

        # collision state
        has_coll_api = prim.HasAPI(UsdPhysics.CollisionAPI)
        coll_enabled = None
        approx = None
        has_rigid = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        if has_coll_api:
            cattr = prim.GetAttribute("physics:collisionEnabled")
            if cattr and cattr.HasAuthoredValue():
                coll_enabled = bool(cattr.Get())
            else:
                coll_enabled = True  # default of CollisionAPI is enabled
            mattr = prim.GetAttribute("physics:approximation")
            if mattr and mattr.HasAuthoredValue():
                approx = mattr.Get()

        # world-space translation for context
        try:
            t = xcache.GetLocalToWorldTransform(prim).ExtractTranslation()
            pos = f"({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})"
        except Exception:
            pos = "?"

        rows.append(dict(path=str(prim.GetPath()), type=str(tname) or "-",
                         refs=has_refs, geom=is_geom, coll_api=has_coll_api,
                         coll_en=coll_enabled, approx=str(approx) if approx else "",
                         rigid=has_rigid, pos=pos))

    # inline geometry = authored geom prims without external references = the ADDED obstacles
    inline = [r for r in rows if r["geom"] and not r["refs"]]
    referenced = [r for r in rows if r["refs"]]

    print(f"=== INLINE GEOMETRY (the added obstacle shapes): {len(inline)} ===")
    hdr = f"{'collAPI':7} {'enabled':7} {'rigid':5} {'approx':16} {'type':9} {'pos':22} path"
    print(hdr); print("-" * len(hdr))
    n_missing = 0
    for r in sorted(inline, key=lambda r: r["path"]):
        ok = r["coll_api"] and r["coll_en"]
        if not ok:
            n_missing += 1
        print(f"{str(r['coll_api']):7} {str(r['coll_en']):7} {str(r['rigid']):5} "
              f"{r['approx']:16} {r['type']:9} {r['pos']:22} {r['path']}"
              + ("   <-- NO COLLISION" if not ok else ""))

    print()
    print(f"=== REFERENCED PRIMS (warehouse shell / composed assets): {len(referenced)} ===")
    for r in sorted(referenced, key=lambda r: r["path"])[:60]:
        print(f"  {r['type']:12} collAPI={r['coll_api']} {r['path']}")

    print()
    print(f"SUMMARY: {len(inline)} inline geom prims, "
          f"{len(inline) - n_missing} WITH collision, {n_missing} WITHOUT collision.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
