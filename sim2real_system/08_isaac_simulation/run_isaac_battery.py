#!/usr/bin/env python3
"""Run the test battery in Isaac and keep the per-test tracks (no rover needed).

The sim rover drives its wheels through PhysX wheel-floor contact with the given physics
parameters, exactly as in the closed loop.

    # a calibrated physics surface (e.g. the staged concrete fit)
    python3 optimize/run_isaac_battery.py <out dir> --calib surfaces/concrete_physics.yaml \\
        --surface concrete --combine-mode average
    # the as-shipped rover (URDF/schema defaults + the surface floor band)
    python3 optimize/run_isaac_battery.py <out dir> --physics-defaults --surface carpet

Writes <out dir>/<test>.csv (+ <test>__repK.csv with --repeats > 1).
"""
import argparse
import sys
from pathlib import Path

import yaml

SC = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SC / "loop"), str(SC / "optimize"), str(SC.parent / "ugv_controller_v2" / "calibration")]
import calib_params as C  # noqa: E402
import calibrate as CAL  # noqa: E402
import features as F  # noqa: E402
import sim_batch  # noqa: E402

TESTS = ["T1_straight", "T2_spin", "T3_speed_sweep", "T3b_top_speed", "T4_arc", "T5_stop",
         "T6_spin_sweep", "R1_straight_fb", "R2_spin_rates", "R3_arcs_slow", "R4_arcs_fast",
         "R5_nav_pattern"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--calib", help="surfaces/*.yaml (its 'calibrated') or a nested sim_calibration yaml")
    g.add_argument("--physics-defaults", action="store_true", help="schema defaults + surface floor band")
    ap.add_argument("--surface", default="carpet", help="floor band for unset floor keys")
    ap.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"])
    ap.add_argument("--repeats", type=int, default=1,
                    help="sim trials per test (PhysX contact is not bit-exact run to run)")
    ap.add_argument("--tests", default=",".join(TESTS))
    a = ap.parse_args()
    seed = None
    if a.calib:
        doc = yaml.safe_load(Path(a.calib).read_text()) or {}
        seed = doc.get("calibrated") or C.flatten(doc.get("sim_calibration_yaml", doc))
    plan = {}
    for k in range(1, a.repeats + 1):
        for t in a.tests.split(","):
            plan[t if k == 1 else f"{t}__rep{k}"] = F.load_sequence(str(SC / f"parallel/sequences/{t}.yaml"))
    theta = CAL.start_theta(CAL.load_measured(str(SC / "optimize/measured.yaml")), a.surface, seed)
    csvs = sim_batch.run_battery(theta, plan, Path(a.out), combine_mode=a.combine_mode)
    print(f"wrote {len(csvs)} tracks to {a.out}", flush=True)


if __name__ == "__main__":
    main()
