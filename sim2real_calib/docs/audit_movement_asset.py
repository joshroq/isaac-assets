"""Read-only USD motion audit. Run with Isaac's standalone Python/USD libraries."""
import json
from pathlib import Path
import sys

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ugv_controller_v2" / "calibration"))
import calib_params as C

stage = Usd.Stage.Open(str(ROOT / "jetson_rover_v2.usd"))
parameters = []
for spec in C.SCHEMA:
    values = []
    for path, name in spec["targets"]:
        prim = stage.GetPrimAtPath(path)
        attr = prim.GetAttribute(name) if prim else None
        value = attr.Get() if attr else None
        values.append({"prim": path, "attribute": name, "resolved_value": value,
                       "authored": bool(attr and attr.HasAuthoredValueOpinion())})
    parameters.append({"key": spec["key"], "yaml_path": ".".join(spec["yaml"]),
                       "schema_default": spec["default"], "bounds": [spec["lo"], spec["hi"]],
                       "stage": spec["stage"], "usd": values})

bindings, masses = [], []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if prim.HasAPI(UsdPhysics.CollisionAPI) and "wheel" in path:
        material, relationship = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
        bindings.append({"collider": path, "physics_material": str(material.GetPath()) if material else None,
                         "binding_relationship": str(relationship.GetPath()) if relationship else None})
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        attr = prim.GetAttribute("physics:mass")
        value = attr.Get() if attr else None
        if value is not None and value > 0:
            masses.append({"prim": path, "mass": value})

data = {"asset": "jetson_rover_v2.usd", "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "schema_count": len(C.SCHEMA), "searchable_count": len(C.OPTIMISABLE),
        "frozen_keys": C.FROZEN, "parameters": parameters, "wheel_material_bindings": bindings,
        "explicit_rigid_body_masses": masses,
        "explicit_rigid_body_mass_sum": sum(m["mass"] for m in masses),
        "scope": "Static locally resolved USD composition; remote sensor assets may require Kit resolution."}
output = Path(__file__).with_name("movement_asset_audit.json")
output.write_text(json.dumps(data, indent=2) + "\n")
print(f"Wrote {output}; {len(parameters)} parameters; {len(bindings)} wheel colliders")
