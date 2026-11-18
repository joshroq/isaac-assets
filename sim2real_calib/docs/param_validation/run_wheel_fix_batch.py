"""Run a labelled batch of Isaac trials via sim_eval (one Isaac at a time) and print features."""
# Copied from the 2026-10-14 validation session; trial outputs go to /tmp/wheelaudit/runs/ (scratch).
import sys, json, time, yaml, traceback
from pathlib import Path
REPO = Path("/home/joshroq/projects/isaac-assets")
sys.path.insert(0, str(REPO / "sim2real_calib/optimize"))
sys.path.insert(0, str(REPO / "ugv_controller_v2/calibration"))
import sim_eval, features as F, calib_params as C
import numpy as np

label = sys.argv[1]
cases = sys.argv[2].split(",")
SEQ = REPO / "sim2real_calib/parallel/sequences"
def seq(name): return yaml.safe_load((SEQ / f"{name}.yaml").read_text())["steps"]
D = C.defaults()
CASES = {
    "T1":        (D, "T1_straight"),
    "T2":        (D, "T2_spin"),
    "T4":        (D, "T4_arc"),
    "T3":        (D, "T3_speed_sweep"),
    "T2_cornerlow": ({**D, "corner_mu_dynamic": 0.4, "corner_mu_static": 0.45}, "T2_spin"),
    "T2_middlelow": ({**D, "middle_mu_dynamic": 0.3, "middle_mu_static": 0.35}, "T2_spin"),
    "T1_nocalib": ({}, "T1_straight"),
}
out = {}
for c in cases:
    theta, sname = CASES[c]
    wd = Path(f"/tmp/wheelaudit/runs/{label}/{c}")
    t0 = time.time()
    try:
        csv = sim_eval.run_trial(theta, seq(sname), surface=c, combine_mode="min", workdir=str(wd), startup_timeout=400)
        f = F.extract(*F.load_csv(csv), seq(sname))
        out[c] = f
        print(f"[{label}/{c}] {time.time()-t0:.0f}s", {k: round(v, 4) for k, v in f.items()}, flush=True)
    except Exception as e:
        print(f"[{label}/{c}] FAILED {e}", flush=True)
json.dump(out, open(f"/tmp/wheelaudit/runs/{label}.json", "w"), indent=1)
