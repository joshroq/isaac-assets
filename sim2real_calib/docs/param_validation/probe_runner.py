"""run_ground_plane.py + in-memory stage edits from $PROBE_EDITS (json [[path|'SCENE', attr, type, value], ...])."""
import json, os, sys
from pathlib import Path
V2 = Path("/home/joshroq/projects/isaac-assets/ugv_controller_v2")
sys.path.insert(0, str(V2 / "common")); sys.path.insert(0, str(V2))
import run_baked_rover as rb
_orig = rb._apply_calibration
def patched(stage, calibration, combine_mode):
    _orig(stage, calibration, combine_mode)
    from pxr import Sdf
    T = {"float": Sdf.ValueTypeNames.Float, "int": Sdf.ValueTypeNames.Int, "token": Sdf.ValueTypeNames.Token, "bool": Sdf.ValueTypeNames.Bool}
    for path, attr, typ, val in json.loads(os.environ.get("PROBE_EDITS", "[]")):
        prims = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"] if path == "SCENE" else [stage.GetPrimAtPath(path)]
        for p in prims:
            a = p.GetAttribute(attr)
            if not a or not a.IsValid():
                a = p.CreateAttribute(attr, T[typ])
            a.Set(val); print(f"[PROBE] {p.GetPath()}.{attr} = {val}", flush=True)
rb._apply_calibration = patched
sys.argv[0] = str(V2 / "run_ground_plane.py")
import runpy
runpy.run_path(str(V2 / "run_ground_plane.py"), run_name="__main__")
