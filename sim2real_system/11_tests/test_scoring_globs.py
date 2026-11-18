"""closed_loop.build_targets with glob scoring entries ("v_k*") - no hardware needed.

    python3 -m pytest -q sim2real_calib/loop/tests/test_scoring_globs.py
"""
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

LOOP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOOP))
import closed_loop as CL  # noqa: E402


def test_glob_patterns_select_signed_step_features():
    steps = CL.F.load_sequence(str(CL.SEQ_DIR / "R2_spin_rates.yaml"))
    with tempfile.TemporaryDirectory() as d:
        reps = []
        for r, gain in enumerate((0.3, 0.32)):
            t, a, rows = 0.0, 0.0, [(0.0, 0.5, 0.5, 0.0)]
            for s in steps:
                for _ in range(int(round(s["duration"] / 0.02))):
                    a += gain * s["angular"] * 0.02
                    t += 0.02
                    rows.append((t, 0.5, 0.5, math.atan2(math.sin(a), math.cos(a))))
            p = Path(d) / f"rep{r + 1}.csv"
            np.savetxt(p, rows, delimiter=",", header="t,x,y,yaw", comments="")
            reps.append({"csv": str(p)})
        t = CL.build_targets({"R2_spin_rates": reps}, {"R2_spin_rates": steps},
                             {"R2_spin_rates": ["w_k*"]}, lambda *a: None)
    feats = t["R2_spin_rates"]["features"]
    assert len(feats) == 8 and all(k.startswith("w_k") for k in feats)
    assert abs(feats["w_k1"]["mean"] - 0.31 * 0.4) < 0.01
    assert CL.is_scored("v_k3", ["v_k*"]) and not CL.is_scored("v_ss", ["v_k*"])
