import sys, json, os, yaml
# Copied from the 2026-10-14 validation session; trial outputs go to /tmp/wheelaudit/runs/ (scratch).
from pathlib import Path
REPO = Path("/home/joshroq/projects/isaac-assets")
sys.path.insert(0, str(REPO / "sim2real_calib/optimize")); sys.path.insert(0, str(REPO / "ugv_controller_v2/calibration"))
import sim_eval, features as F, calib_params as C
import numpy as np
sim_eval.RUN_GROUND = Path(__file__).with_name("probe_runner.py")
label = sys.argv[1]; names = sys.argv[2].split(",")
T4 = yaml.safe_load((REPO / "sim2real_calib/parallel/sequences/T4_arc.yaml").read_text())["steps"]
T2 = yaml.safe_load((REPO / "sim2real_calib/parallel/sequences/T2_spin.yaml").read_text())["steps"]
D = C.defaults()
WH = [f"/World/ugv_rover/{w}_wheel_link" for w in ("left_up","left_middle","left_down","right_up","right_middle","right_down")]
ART = "/World/ugv_rover/base_footprint"
CASES = {
 "no_middle_collision": ({}, [["/World/ugv_rover/left_middle_wheel_link/collisions/left_up_wheel_link/node_STL_BINARY_", "physics:collisionEnabled", "bool", False], ["/World/ugv_rover/right_middle_wheel_link/collisions/right_up_wheel_link/node_STL_BINARY_", "physics:collisionEnabled", "bool", False]], T4),
 "no_middle_collision_spin": ({}, [["/World/ugv_rover/left_middle_wheel_link/collisions/left_up_wheel_link/node_STL_BINARY_", "physics:collisionEnabled", "bool", False], ["/World/ugv_rover/right_middle_wheel_link/collisions/right_up_wheel_link/node_STL_BINARY_", "physics:collisionEnabled", "bool", False]], T2),
 "compliant": ({}, [[m, a, "float", v] for m in ("/World/Physics_Materials/corner_wheel", "/World/Physics_Materials/middle_wheel") for a, v in (("physxMaterial:compliantContactStiffness", 3000.0), ("physxMaterial:compliantContactDamping", 30.0))], T4),
 "ftype_two": ({}, [["SCENE", "physxScene:frictionType", "token", "twoDirectional"]], T4),
 "ftype_one": ({}, [["SCENE", "physxScene:frictionType", "token", "oneDirectional"]], T4),
 "sphere_wheels": ({}, [[f"{l}/collisions/{c}/node_STL_BINARY_", "physics:approximation", "token", "boundingSphere"] for l, c in (("/World/ugv_rover/left_up_wheel_link","left_up_wheel_link"),("/World/ugv_rover/left_down_wheel_link","left_down_wheel_link"),("/World/ugv_rover/right_up_wheel_link","right_up_wheel_link"),("/World/ugv_rover/right_down_wheel_link","right_down_wheel_link"),("/World/ugv_rover/left_middle_wheel_link","left_up_wheel_link"),("/World/ugv_rover/right_middle_wheel_link","right_up_wheel_link"))], T4),
 "static_eq_dynamic": ({"floor_mu_static": 0.75}, [], T4),
 "corrdist_0.1": ({}, [["SCENE", "physxScene:frictionCorrelationDistance", "float", 0.1]], T4),
 "offset_0.001": ({}, [["SCENE", "physxScene:frictionOffsetThreshold", "float", 0.001]], T4),
 "vel_iters_16": ({}, [[ART, "physxArticulation:solverVelocityIterationCount", "int", 16]], T4),
 "friction_every_iter": ({}, [["SCENE", "physxScene:enableEnhancedDeterminism", "bool", False], ["SCENE", "physxScene:frictionType", "token", "twoDirectional"]], T4),
}
out_path = Path(f"/tmp/wheelaudit/runs/{label}.json")
out = json.loads(out_path.read_text()) if out_path.exists() else {}
for n in names:
    over, edits, steps = CASES[n]
    os.environ["PROBE_EDITS"] = json.dumps(edits)
    wd = Path(f"/tmp/wheelaudit/runs/{label}/{n}")
    try:
        csv = sim_eval.run_trial({**D, **over}, steps, surface=n, combine_mode="min", workdir=str(wd), startup_timeout=400)
        t, x, y, yaw = F.load_csv(csv); v, w, yu = F._speed_yawrate(t, x, y, yaw)
        a = steps[0]["duration"]; b = a + steps[1]["duration"]; m = (t >= a + 1) & (t < b)
        res = {"v_med": float(np.median(v[m])), "w_med": float(np.median(w[m])), "yaw_total": float(yu[m][-1]-yu[m][0]), "cmd": steps[1]}
        out[n] = res; print(n, res, flush=True)
        print("  ", [l.strip() for l in open(wd / "isaac.log") if "[PROBE]" in l][:3], flush=True)
    except Exception as e:
        print(n, "FAILED", e, flush=True)
    out_path.write_text(json.dumps(out, indent=1))
