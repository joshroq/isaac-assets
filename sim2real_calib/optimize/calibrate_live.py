#!/usr/bin/env python3
"""Closed-loop sim->real AUTO-CALIBRATION (safe: the optimiser iterates the SIM only).

This is the full-loop companion to ../demo/realtime_demo.py. realtime_demo.py drives both
rovers and compares; THIS script closes the loop: it takes ONE real reference maneuver,
then repeatedly runs the Isaac sim with different parameter values until the sim's motion
feature matches the real one, and WRITES the updated sim parameter to a surface file.

SAFETY: the real rover drives at most once, and only the in-place T2_spin by default - it
never translates toward furniture. By default it REUSES an already-recorded real reference
(demo/runs/live/<seq>/real.csv) so the rover does not move at all. Every optimiser iteration
is sim-side (headless Isaac), so nothing the loop does can drive the rover into anything.

Pipeline (per the staged map, PARAMETER_MAP.md §6):
  real.csv --features--> target (e.g. spin dyaw, omega_ss)
  loop:  theta[param] = x  -->  Isaac sim  -->  sim features  -->  loss(sim vs real)
  scipy minimises loss -> calibrated param -> written to surfaces/<surface>.yaml

Default param: wheel_distance (stage 4), identified by the T2_spin features. Raising it
increases sim yaw, so it is the knob that makes the sim spin match the real spin.

  source ros2env.sh
  python3 optimize/calibrate_live.py                       # reuse last real spin, tune wheel_distance
  python3 optimize/calibrate_live.py --max-evals 8         # tighter Isaac budget
  python3 optimize/calibrate_live.py --real-csv path.csv   # a specific real reference
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SC = HERE.parent
REPO = SC.parent
sys.path.insert(0, str(REPO / "ugv_controller_v2" / "calibration"))
sys.path.insert(0, str(HERE))

import calib_params as C      # noqa: E402
import features as F          # noqa: E402
import forward_model as fm    # noqa: E402
import sim_eval               # noqa: E402

SEQ_DIR = SC / "parallel" / "sequences"
# param -> (stage, the real test that identifies it, the features scored)
from calibrate import STAGES  # noqa: E402  reuse the staged schedule
STAGE_BY_KEY = {k: st for st in STAGES for k in st["keys"]}


def load_sequence(path):
    import yaml
    data = yaml.safe_load(Path(path).read_text())
    steps = data["steps"]
    for s in steps:
        s.setdefault("linear", 0.0); s.setdefault("angular", 0.0); s.setdefault("duration", 0.0)
    return steps


def default_real_csv(seq_name):
    cand = SC / "demo" / "runs" / "live" / seq_name / "real.csv"
    return cand if cand.exists() else None


def fmt_feats(got, target):
    bits = []
    for k in target:
        g = got.get(k, float("nan"))
        bits.append(f"{k}={g:.4f}(real {target[k]['mean']:.4f})")
    return "  ".join(bits)


def loss_of(got, target):
    loss = 0.0
    for k in target:
        if k in got and np.isfinite(got[k]):
            loss += ((got[k] - target[k]["mean"]) / max(target[k]["sigma"], 1e-6)) ** 2
        else:
            loss += 25.0
    return loss


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--param", default="wheel_distance", help="parameter key to calibrate")
    ap.add_argument("--feats", default=None,
                    help="comma list of features to score (default: the param's stage feats "
                         "for this sequence). For a spin, omega_mean (dyaw/T) is robust to the "
                         "sim's stick-slip judder that makes the median omega_ss overstate rotation.")
    ap.add_argument("--sequence", default=str(SEQ_DIR / "T2_spin.yaml"))
    ap.add_argument("--real-csv", default=None, help="real reference CSV (default: reuse last live run)")
    ap.add_argument("--surface", default="live", help="run label (output file name)")
    ap.add_argument("--floor", default=None,
                    help="floor material (concrete|carpet|foam or alias): sets the literature "
                         "band floor friction may take; default = all-materials envelope")
    ap.add_argument("--max-evals", type=int, default=10, help="Isaac refine budget")
    ap.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"])
    ap.add_argument("--out", default=None, help="surfaces/<surface>.yaml (default)")
    ap.add_argument("--outdir", default=None, help="artifact dir (plots, sim CSVs)")
    args = ap.parse_args()

    key = args.param
    if key not in C.SCHEMA_BY_KEY or key not in C.OPTIMISABLE:
        sys.exit(f"'{key}' is not an optimisable parameter. Optimisable: {C.OPTIMISABLE}")
    schema = C.SCHEMA_BY_KEY[key]
    st = STAGE_BY_KEY.get(key)
    if args.feats:
        feats_needed = [f.strip() for f in args.feats.split(",") if f.strip()]
    else:
        seq_stem = Path(args.sequence).stem
        feats_needed = (st["tests"].get(seq_stem) or sorted({f for fs in st["tests"].values() for f in fs})
                        if st else ["omega_mean", "dyaw"])
    steps = load_sequence(args.sequence)
    seq_name = Path(args.sequence).stem

    real_csv = Path(args.real_csv) if args.real_csv else default_real_csv(seq_name)
    if not real_csv or not real_csv.exists():
        sys.exit(f"no real reference CSV. Record one with demo/realtime_demo.py "
                 f"--sequence {args.sequence}, or pass --real-csv.")
    outdir = Path(args.outdir or (SC / "optimize" / "runs" / f"{args.surface}_{seq_name}"))
    outdir.mkdir(parents=True, exist_ok=True)

    # --- 1. real reference features ---
    real_feats_all = F.extract(*F.load_csv(str(real_csv)), steps)
    target = F.aggregate([real_feats_all])            # single-trial -> sigma floored
    target = {k: v for k, v in target.items() if k in feats_needed}
    if not target:
        sys.exit(f"real reference has none of the stage features {feats_needed}; "
                 f"got {sorted(real_feats_all)}")
    print("=" * 70)
    print(f"CLOSED-LOOP CALIBRATION   param={key}  (stage {st['id'] if st else '?'} "
          f"{st['name'] if st else ''})")
    print(f"  real reference : {real_csv}")
    print(f"  maneuver       : {seq_name}  scoring features: {list(target)}")
    for k in target:
        print(f"    real {k:10s} = {target[k]['mean']:.4f}  (sigma {target[k]['sigma']:.4f})")
    base = C.defaults()
    base.update(C.surface_start(args.floor))     # floor friction starts mid-band
    # A one-parameter search must respect the other coefficient, which stays at baseline.
    lo, hi = C.bounds(key, args.floor)
    for static, dynamic in C.FRICTION_PAIRS:
        if key == static:
            lo = max(lo, base[dynamic])
        elif key == dynamic:
            hi = min(hi, base[static])
    log_scale = schema["scale"] == "log"
    search_bounds = (math.log10(lo), math.log10(hi)) if log_scale else (lo, hi)

    def from_search(x):
        return float(10 ** x if log_scale else x)

    print(f"  bounds         : {key} in [{lo}, {hi}]  default {schema['default']}")
    print("=" * 70)

    evals = []   # (x, loss, got, csv)

    def sim_eval_at(x, tag=""):
        theta = dict(base); theta[key] = float(x)
        t0 = time.time()
        csv = sim_eval.run_trial(theta, steps, surface=f"{args.surface}_{seq_name}",
                                 combine_mode=args.combine_mode)
        got = F.extract(*F.load_csv(csv), steps)
        L = loss_of(got, target)
        print(f"  [eval {len(evals)+1:2d}] {key}={x:.4f} -> {fmt_feats(got, target)}  "
              f"loss={L:.3f}  ({time.time()-t0:.0f}s) {tag}")
        evals.append((float(x), L, got, csv))
        return L, got, csv

    # --- 2. analytic warm-start (instant, no Isaac): where should x roughly be? ---
    def analytic_loss(x):
        theta = dict(base); theta[key] = float(x)
        tr = fm.simulate(theta, steps, combine_mode=args.combine_mode)
        got = F.extract(np.array(tr["t"]), np.array(tr["x"]), np.array(tr["y"]),
                        np.array(tr["yaw"]), steps)
        return loss_of(got, target)

    from scipy.optimize import minimize_scalar
    try:
        if schema.get("isaac_only"):
            raise ValueError("this parameter requires Isaac contact/drive physics")
        warm = minimize_scalar(lambda x: analytic_loss(from_search(x)),
                               bounds=search_bounds, method="bounded",
                               options={"xatol": 1e-3, "maxiter": 60}).x
        warm = from_search(warm)
        print(f"[warm-start] analytic model hint: {key} ~ {warm:.4f} (informational only; "
              f"the Isaac search below is the authority)")
    except Exception as e:
        print(f"[warm-start] analytic hint unavailable ({e})")

    # --- 3. Isaac baseline (default) for the before/after comparison ---
    print(f"\n[baseline] sim at default {key}={schema['default']} (BEFORE calibration):")
    L0, got0, csv0 = sim_eval_at(schema["default"], tag="<- BEFORE")

    # --- 4. Isaac refine: bounded search over the FULL schema bounds. The loss is
    #        smooth & monotone-dominated in the spin-rate feature, so bounded Brent
    #        converges; we do not trust the analytic model for the bracket. ---
    print(f"\n[refine] Isaac bounded search {key} in [{lo:.3f}, {hi:.3f}], "
          f"budget {args.max_evals} evals:")
    res = minimize_scalar(lambda x: sim_eval_at(from_search(x))[0], bounds=search_bounds,
                          method="bounded", options={"xatol": 2e-3, "maxiter": args.max_evals})
    best_x = from_search(res.x)

    # --- 5. final verification eval at the calibrated value ---
    print(f"\n[verify] sim at calibrated {key}={best_x:.4f} (AFTER calibration):")
    Lb, gotb, csvb = sim_eval_at(best_x, tag="<- AFTER")

    # --- 6. write the updated sim parameter ---
    theta_final = dict(base); theta_final[key] = best_x
    out = Path(args.out) if args.out else (SC / "surfaces" / f"{args.surface}.yaml")
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        import yaml
        doc = {"surface": args.surface, "combine_mode": args.combine_mode,
               "calibrated_param": key, "maneuver": seq_name,
               "before": {"value": schema["default"], **{k: float(got0.get(k, float('nan'))) for k in target}},
               "after": {"value": best_x, **{k: float(gotb.get(k, float('nan'))) for k in target}},
               "real_target": {k: float(target[k]["mean"]) for k in target},
               "sim_calibration_yaml": C.nest({key: best_x})}
        out.write_text(yaml.safe_dump(doc, sort_keys=False))
    except Exception as e:
        print(f"[warn] could not write surface yaml: {e}")

    # --- 7. before/after/real plot ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        def yaw_t(csv):
            t, x, y, yaw = F.load_csv(csv)
            return t, np.degrees(np.unwrap(yaw))
        tr_r, yr = yaw_t(str(real_csv)); yr = yr - yr[0]
        tr_b, yb = yaw_t(csv0); yb = yb - yb[0]
        tr_a, ya = yaw_t(csvb); ya = ya - ya[0]
        fig, (axA, axB) = plt.subplots(1, 2, figsize=(13, 5))
        axA.plot(tr_r, yr, "k-", lw=2, label="real (camera)")
        axA.plot(tr_b, yb, "r--", lw=1.8, label=f"sim BEFORE ({key}={schema['default']})")
        axA.plot(tr_a, ya, "g-", lw=1.8, label=f"sim AFTER ({key}={best_x:.3f})")
        axA.set_xlabel("t [s]"); axA.set_ylabel("heading [deg]")
        axA.set_title(f"heading vs time  ({seq_name})"); axA.legend(); axA.grid(alpha=.3)
        # feature bar: real vs before vs after for the primary feature
        prim = "dyaw" if "dyaw" in target else list(target)[0]
        rv = math.degrees(target[prim]["mean"]) if "yaw" in prim else target[prim]["mean"]
        bv = math.degrees(got0.get(prim, 0)) if "yaw" in prim else got0.get(prim, 0)
        av = math.degrees(gotb.get(prim, 0)) if "yaw" in prim else gotb.get(prim, 0)
        axB.bar(["real", "sim before", "sim after"], [rv, bv, av],
                color=["k", "r", "g"])
        axB.set_title(f"{prim} ({'deg' if 'yaw' in prim else 'SI'})"); axB.grid(alpha=.3, axis="y")
        fig.tight_layout(); fig.savefig(outdir / "calibration.png", dpi=110)
        print(f"[plot] -> {outdir/'calibration.png'}")
    except Exception as e:
        print(f"[warn] plot skipped: {e}")

    # --- 8. summary ---
    print("\n" + "=" * 70)
    print(f"RESULT  {key}:  {schema['default']}  ->  {best_x:.4f}")
    for k in target:
        print(f"  {k:10s}  real={target[k]['mean']:.4f}  "
              f"sim_before={got0.get(k, float('nan')):.4f}  sim_after={gotb.get(k, float('nan')):.4f}")
    print(f"  loss      before={L0:.3f}  after={Lb:.3f}")
    print(f"  wrote     {out}")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
