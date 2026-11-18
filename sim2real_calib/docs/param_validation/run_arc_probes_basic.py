import sys, json, time, yaml
# Copied from the 2026-10-14 validation session; trial outputs go to /tmp/wheelaudit/runs/ (scratch).
from pathlib import Path
REPO = Path("/home/joshroq/projects/isaac-assets")
sys.path.insert(0, str(REPO / "sim2real_calib/optimize")); sys.path.insert(0, str(REPO / "ugv_controller_v2/calibration"))
import sim_eval, features as F, calib_params as C
import numpy as np
label = sys.argv[1]; names = sys.argv[2].split(",")
T4 = yaml.safe_load((REPO / "sim2real_calib/parallel/sequences/T4_arc.yaml").read_text())["steps"]
D = C.defaults()
CASES = {
 "default": ({}, "ros", T4),
 "direct": ({}, "direct", T4),
 "D_eq_B": ({"wheel_distance": 0.1745}, "ros", T4),
 "maxF_0.15": ({"drive_max_force": 0.15}, "ros", T4),
 "middle_mu_0.05": ({"middle_mu_dynamic": 0.05, "middle_mu_static": 0.05}, "ros", T4),
 "floor_mu_0.4": ({"floor_mu_dynamic": 0.4, "floor_mu_static": 0.45}, "ros", T4),
 "wide_arc": ({}, "ros", [{"linear":0,"angular":0,"duration":1.0},{"linear":0.15,"angular":0.15,"duration":6.0},{"linear":0,"angular":0,"duration":2.0}]),
}
out_path = Path(f"/tmp/wheelaudit/runs/{label}.json")
out = json.loads(out_path.read_text()) if out_path.exists() else {}
for n in names:
    over, mode, steps = CASES[n]
    wd = Path(f"/tmp/wheelaudit/runs/{label}/{n}")
    try:
        csv = sim_eval.run_trial({**D, **over}, steps, surface=n, combine_mode="min", workdir=str(wd), startup_timeout=400, mode=mode)
        t, x, y, yaw = F.load_csv(csv); v, w, yu = F._speed_yawrate(t, x, y, yaw)
        a = steps[0]["duration"]; b = a + steps[1]["duration"]; m = (t >= a + 1) & (t < b)
        res = {"v_med": float(np.median(v[m])), "w_med": float(np.median(w[m])), "yaw_total": float(yu[m][-1]-yu[m][0]),
               "path": float(np.sum(np.hypot(np.diff(x[m]), np.diff(y[m])))), "cmd": steps[1], "csv": csv}
        out[n] = res; print(n, {k: (round(v_, 4) if isinstance(v_, float) else v_) for k, v_ in res.items() if k != "csv"}, flush=True)
    except Exception as e:
        print(n, "FAILED", e, flush=True)
    out_path.write_text(json.dumps(out, indent=1))
