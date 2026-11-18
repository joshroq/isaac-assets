#!/usr/bin/env python3
"""Bake the canonical chassis STL into the shared Isaac asset without reimporting.

Run with Isaac's standalone pxr Python via sync_base_mesh.sh. --check is read-only.
This preserves prim paths, materials and calibrated drivetrain physics. The fixed2
deck translation is applied once to its mounts, including stronger scene overrides.
Future STL edits require a new geometry audit, particularly for mass and mounts.
"""
from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

from pxr import Gf, Sdf, Usd, UsdGeom, Vt

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / "ugv_ws/src/ugv_main/ugv_description"
STL = Path("meshes/ugv_rover/base_link_fixed2.stl")
ALIAS = Path("meshes/ugv_rover/base_link_fixed.stl")
URDF = Path("urdf/ugv_rover_isaac_fixed.urdf")
ASSET = Path("urdf/ugv_rover_isaac_fixed")
BASE = ASSET / "configuration/ugv_rover_isaac_fixed_base.usd"
PHYSICS = ASSET / "configuration/ugv_rover_isaac_fixed_physics.usd"
MESH = "/meshes/base_link_fixed/node_STL_BINARY_/mesh"
DECK_LIFT = 0.0457
DECK_LINKS = ("base_lidar_link", "front_camera_link", "pt_base_link",
              "pt_link1", "pt_link2", "pt_camera_link")
DECK_JOINTS = ("base_lidar_link_joint", "front_camera_link_joint", "pt_base_link_joint")


def read_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Truncated binary STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if not count or len(data) != 84 + 50 * count:
        raise ValueError(f"Expected a nonempty binary STL: {path}")
    points, normals = [], []
    for face in struct.iter_unpack("<12fH", data[84:]):
        triangle = [Gf.Vec3f(*face[i:i + 3]) for i in (3, 6, 9)]
        if not all(math.isfinite(x) for v in triangle for x in v):
            raise ValueError("STL contains non-finite coordinates")
        # Compute normals from winding: some normals in the CAD export disagree.
        normal = Gf.Cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        if normal.GetLength() == 0:
            raise ValueError("STL contains a degenerate triangle")
        normal.Normalize()
        points.extend(triangle)
        normals.extend([normal] * 3)
    extent = [Gf.Vec3f(*(fn(p[i] for p in points) for i in range(3)))
              for fn in (min, max)]
    return {
        "points": Vt.Vec3fArray(points),
        "normals": Vt.Vec3fArray(normals),
        "faceVertexCounts": Vt.IntArray([3] * count),
        "faceVertexIndices": Vt.IntArray(range(len(points))),
        "extent": Vt.Vec3fArray(extent),
    }, hashlib.sha256(data).hexdigest()


def installed_descriptions():
    for prefix in (ROOT / "install", ROOT / "ugv_ws/install"):
        for path in (prefix / "ugv_description/share/ugv_description",
                     prefix / "share/ugv_description"):
            if path.is_dir() and path.resolve() != DESCRIPTION.resolve():
                yield path


def lift_deck_once(layer, attributes):
    data = dict(layer.customLayerData)
    previous = data.get("baseMeshDeckLift")
    if previous is not None:
        if previous != DECK_LIFT:
            raise ValueError(f"Unexpected previous deck lift in {layer.identifier}: {previous}")
        return
    for path in attributes:
        attr = layer.GetAttributeAtPath(path)
        if not attr or attr.default is None:
            raise ValueError(f"Missing deck mount: {layer.identifier}: {path}")
        position = attr.default
        position[2] += DECK_LIFT
        attr.default = position
    data["baseMeshDeckLift"] = DECK_LIFT
    layer.customLayerData = data
    layer.Save()


def sync(values, digest):
    layer = Sdf.Layer.FindOrOpen(str(DESCRIPTION / BASE))
    mesh = layer.GetPrimAtPath(MESH)
    if not mesh or mesh.typeName != "Mesh":
        raise RuntimeError(f"Missing shared chassis mesh: {MESH}")
    for name, value in values.items():
        mesh.attributes[name].default = value
    mesh.attributes["normals"].SetInfo("interpolation", "vertex")
    # The importer authored one white color per old vertex. Resize it too.
    color = mesh.attributes["primvars:displayColor"]
    color.default = Vt.Vec3fArray([Gf.Vec3f(1)] * len(values["points"]))
    color.SetInfo("interpolation", "vertex")
    data = dict(layer.customLayerData)
    data["baseMeshSource"] = STL.as_posix()
    data["baseMeshSHA256"] = digest
    layer.customLayerData = data
    layer.Save()
    # USD link prims are siblings, so moving the pan base alone does not move its arm.
    lift_deck_once(layer, [f"/ugv_rover/{link}.xformOp:translate" for link in DECK_LINKS])
    physics = Sdf.Layer.FindOrOpen(str(DESCRIPTION / PHYSICS))
    lift_deck_once(physics, [f"/ugv_rover/joints/{joint}.physics:localPos0"
                            for joint in DECK_JOINTS])
    for name in ("jetson_rover.usd", "jetson_rover_v2.usd"):
        scene = Sdf.Layer.FindOrOpen(str(ROOT / name))
        lift_deck_once(scene, ["/World/ugv_rover/front_camera_link.xformOp:translate"])
    shutil.copyfile(DESCRIPTION / STL, DESCRIPTION / ALIAS)
    # Existing non-symlink colcon installs would otherwise retain the old geometry.
    # Copy only files this migration owns, not unrelated local package settings.
    for installed in installed_descriptions():
        for relative in (STL, ALIAS, URDF, BASE, PHYSICS):
            source, target = DESCRIPTION / relative, installed / relative
            if source.resolve() == target.resolve():
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            # pxr may have this installed layer cached in an interactive session.
            if relative in (BASE, PHYSICS):
                cached = Sdf.Layer.Find(str(target))
                if cached:
                    cached.Reload()
        print(f"Synced installed package: {installed}")


def check_description(description, values, digest):
    assert (description / ALIAS).read_bytes() == (description / STL).read_bytes(), \
        f"Stale compatibility STL: {description}"
    assert hashlib.sha256((description / STL).read_bytes()).hexdigest() == digest
    robot = ET.parse(description / URDF).getroot()
    base = robot.find("link[@name='base_link']")
    for kind in ("visual", "collision"):
        assert base.find(f"{kind}/geometry/mesh").get("filename") == "../" + STL.as_posix()
    for mesh in robot.findall(".//mesh"):
        path = description / URDF.parent / mesh.get("filename")
        assert path.is_file(), f"Unresolved URDF mesh: {path}"
    for joint, z in zip(DECK_JOINTS, (0.0857, 0.067653, 0.0857)):
        origin = robot.find(f"joint[@name='{joint}']/origin")
        assert abs(float(origin.get("xyz").split()[2]) - z) < 1e-8
    layer = Sdf.Layer.FindOrOpen(str(description / BASE))
    assert layer.customLayerData.get("baseMeshSHA256") == digest
    for name, expected in values.items():
        assert layer.GetAttributeAtPath(MESH + "." + name).default == expected, \
            f"Stale {name}: {description / BASE}"


def check_stage(path, root, values):
    mask = Usd.StagePopulationMask()
    # Avoid loading unrelated network camera assets while checking the chassis.
    for child in ("base_link", "joints", "Drive_Graph", *DECK_LINKS):
        mask.Add(root + "/" + child)
    if root.startswith("/World/"):
        mask.Add("/ROS2_Graphs")
    # Inactivate only the remote sensor assets; keep their local parent mounts.
    session = Sdf.Layer.CreateAnonymous()
    for child in ("front_camera_link/Realsense", "base_lidar_link/Example_Rotary_2D"):
        sensor = Sdf.CreatePrimInLayer(session, root + "/" + child)
        sensor.active = False
        sensor.referenceList.ClearEditsAndMakeExplicit()
    stage = Usd.Stage.OpenMasked(Sdf.Layer.FindOrOpen(str(path)), session, mask)
    assert stage, f"Cannot open {path}"
    meshes = {}
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if prim.GetTypeName() == "Mesh" and str(prim.GetPath()).startswith(root + "/base_link/"):
            meshes[str(prim.GetPath())] = prim
    for branch in ("visuals", "collisions"):
        name = root + f"/base_link/{branch}/base_link_fixed/node_STL_BINARY_/mesh"
        assert name in meshes, f"Missing composed mesh: {path}: {name}"
        mesh = UsdGeom.Mesh(meshes[name])
        for attr, expected in values.items():
            assert mesh.GetPrim().GetAttribute(attr).Get() == expected, \
                f"Geometry override hiding updated {attr}: {path}: {name}"
        assert len(mesh.GetDisplayColorAttr().Get()) == len(values["points"])
    # This replacement leaves the axle mounts intact. Catch accidental rescaling.
    for side, y in (("left", 0.08726), ("right", -0.08726)):
        for position, x in (("up", 0.0855), ("middle", 0), ("down", -0.0855)):
            joint = stage.GetPrimAtPath(root + f"/joints/{side}_{position}_wheel_link_joint")
            actual = joint.GetAttribute("physics:localPos0").Get()
            assert all(abs(a - b) < 1e-7 for a, b in zip(actual, (x, y, -0.041212))), \
                f"Axle mount changed: {joint.GetPath()}: {actual}"
    for joint, z in zip(DECK_JOINTS, (0.0857, 0.067653, 0.0857)):
        prim = stage.GetPrimAtPath(root + "/joints/" + joint)
        assert abs(prim.GetAttribute("physics:localPos0").Get()[2] - z) < 1e-7
    for link, old_z in zip(DECK_LINKS, (0.13, 0.111953, 0.13, 0.178, 0.221003, 0.2204339206)):
        if link == "front_camera_link" and root.startswith("/World/"):
            old_z = 0.10970034263670468  # retain the scene's existing camera adjustment
        prim = stage.GetPrimAtPath(root + "/" + link)
        assert abs(prim.GetAttribute("xformOp:translate").Get()[2] - old_z - DECK_LIFT) < 1e-7, \
            f"Deck mount not lifted exactly once: {path}: {link}"
    diff = stage.GetPrimAtPath(root + "/Drive_Graph/SkidSteer_Controller/DiffController")
    if diff:
        print(f"  Drive: R={diff.GetAttribute('inputs:wheelRadius').Get()}, "
              f"effective D={diff.GetAttribute('inputs:wheelDistance').Get()}")
    print(f"Verified visuals, collisions and mounts: {path.relative_to(ROOT)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify without writing")
    args = parser.parse_args()
    values, digest = read_stl(DESCRIPTION / STL)
    if not args.check:
        sync(values, digest)
    for description in (DESCRIPTION, *installed_descriptions()):
        check_description(description, values, digest)
        check_stage(description / ASSET / "ugv_rover_isaac_fixed.usd", "/ugv_rover", values)
    for name in ("jetson_rover.usd", "jetson_rover_v2.usd"):
        check_stage(ROOT / name, "/World/ugv_rover", values)
    print(f"Chassis: {len(values['faceVertexCounts'])} triangles, "
          f"extent={values['extent']}, sha256={digest}")


if __name__ == "__main__":
    main()
