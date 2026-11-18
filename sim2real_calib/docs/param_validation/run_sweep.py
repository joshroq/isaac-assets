"""Isaac sensitivity sweep: each optimisable key at lo/hi on T1/T2/T4 (+ repeats, + probes). Resumable."""
# Copied from the 2026-10-14 validation session; trial outputs go to /tmp/wheelaudit/runs/ (scratch).
import sys, json, time, yaml
from pathlib import Path
REPO = Path("/home/joshroq/projects/isaac-assets")
sys.path.insert(0, str(REPO / "sim2real_calib/optimize")); sys.path.insert(0, str(REPO / "ugv_controller_v2/calibration"))
import sim_eval, features as F, calib_params as C
label = sys.argv[1]
SEQ = REPO / "sim2real_calib/parallel/sequences"
seq = {n: yaml.safe_load((SEQ / f"{n}.yaml").read_text())["steps"] for n in ("T1_straight", "T2_spin", "T4_arc", "T3_speed_sweep", "T5_stop")}
D = C.defaults(); tests = ("T1_straight", "T2_spin", "T4_arc")
cases = [(f"default_r{r}", {}, t) for r in range(3) for t in tests]
for k in C.OPTIMISABLE:
    s = C.SCHEMA_BY_KEY[k]
    for tag, v in (("lo", s["lo"]), ("hi", s["hi"])):
        cases += [(f"{k}={tag}", {k: v}, t) for t in tests]
cases += [("drive_max_force=0.05", {"drive_max_force": 0.05}, t) for t in ("T1_straight", "T2_spin")]
cases += [("drive_max_force=0.02", {"drive_max_force": 0.02}, t) for t in ("T1_straight", "T2_spin")]
cases += [("drive_damping=0.5", {"drive_damping": 0.5}, t) for t in ("T1_straight", "T2_spin")]
cases += [("default", {}, "T5_stop"), ("default", {}, "T3_speed_sweep")]
out_path = Path(f"/tmp/wheelaudit/runs/{label}.json")
out = json.loads(out_path.read_text()) if out_path.exists() else {}
for name, over, t in cases:
    key = f"{name}|{t}"
    if key in out: continue
    wd = Path(f"/tmp/wheelaudit/runs/{label}/{name}_{t}")
    t0 = time.time()
    try:
        csv = sim_eval.run_trial({**D, **over}, seq[t], surface="sweep", combine_mode="min", workdir=str(wd), startup_timeout=400)
        out[key] = {"case": name, "test": t, "theta": over, "csv": csv, "features": F.extract(*F.load_csv(csv), seq[t])}
        print(f"[{len(out)}/{len(cases)}] {key} {time.time()-t0:.0f}s", {a: round(b, 4) for a, b in out[key]['features'].items()}, flush=True)
    except Exception as e:
        print(f"{key} FAILED {e}", flush=True)
    out_path.write_text(json.dumps(out, indent=1))
