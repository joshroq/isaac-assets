#!/usr/bin/env python3
"""Sim↔real DEMO driver — draw both path lines in the webcam-matched overhead view + error.

Runs the sim rover through a sequence, draws its path, and overlays a REFERENCE path for
comparison — WITHOUT the optimizer (fixed parameters; this is a before-calibration demo). The
reference is either the real rover's camera-tracked path (`--real file.csv`, once that is wired)
or, until then, the dead-reckoned commanded path (clearly labelled as a stand-in).

Pipeline:
  1. reference path  : --real CSV  OR  dead_reckon(sequence)        -> reference.csv
  2. sim path        : run Isaac once (optimize/sim_eval) OR --sim-csv  -> sim.csv
  3. overlay video   : render_overhead([reference, sim]) in the matched view -> demo_overhead.mp4
  4. error           : trials/compare_paths.py reference sim           -> compare.png + metrics

Fully offline parts (3,4 and dead-reckon) work now; step 2 needs the Isaac rig + ROS.

    # full run on the rig (fixed calibration, NOT the optimizer):
    source ros2env.sh
    python3 run_demo.py --sequence ../parallel/sequences/square.yaml
    # offline (reuse an existing sim CSV, dead-reckoned reference):
    python3 run_demo.py --sequence ../parallel/sequences/square.yaml --sim-csv /tmp/sim_run.csv
"""
from __future__ import annotations

import argparse
import datetime
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "sim2real_calib" / "overhead"))
sys.path.insert(0, str(REPO / "sim2real_calib" / "optimize"))
sys.path.insert(0, str(REPO / "ugv_controller_v2" / "calibration"))

import dead_reckon
import render_overhead as ro


def _load_theta(calib):
    import calib_params as C
    theta = C.defaults()
    if calib and Path(calib).exists():
        import yaml
        doc = yaml.safe_load(Path(calib).read_text()) or {}
        # accept a surfaces/*.yaml (has sim_calibration_yaml / calibrated) or a raw calib yaml
        src = doc.get("sim_calibration_yaml", doc)
        theta.update(C.flatten(src))
        if "calibrated" in doc:
            theta.update({k: v for k, v in doc["calibrated"].items() if k in theta})
    return theta


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--setup", default=str(REPO / "sim2real_calib" / "overhead" / "camera_setup.yaml"))
    ap.add_argument("--calib", default=None, help="FIXED sim calibration (surfaces/*.yaml or raw); NOT the optimizer")
    ap.add_argument("--combine-mode", default="min")
    ap.add_argument("--real", default=None, help="real rover path CSV (camera); omit = dead-reckoned stand-in")
    ap.add_argument("--sim-csv", default=None, help="reuse an existing sim /odom CSV (skip running Isaac)")
    ap.add_argument("--no-sim", action="store_true", help="don't run Isaac (requires --sim-csv)")
    ap.add_argument("--fps", type=float, default=30.0)
    args = ap.parse_args()

    outdir = Path(args.outdir or (HERE / "runs" / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")))
    outdir.mkdir(parents=True, exist_ok=True)
    print(f"[demo] outdir: {outdir}")
    print("[demo] NOTE: optimizer is intentionally NOT run — fixed parameters.")

    # 1. reference path
    ref_csv = outdir / "reference.csv"
    if args.real:
        import shutil
        shutil.copy2(args.real, ref_csv)
        ref_label = "real (camera)"
        print(f"[demo] reference = real camera path: {args.real}")
    else:
        dead_reckon_main(args.sequence, str(ref_csv))
        ref_label = "commanded (dead-reckoned) [camera tracking TODO]"
        print("[demo] reference = dead-reckoned commanded path (stand-in; wire camera tracking later)")

    # 2. sim path
    sim_csv = outdir / "sim.csv"
    if args.sim_csv:
        import shutil
        shutil.copy2(args.sim_csv, sim_csv)
        print(f"[demo] sim path = existing CSV: {args.sim_csv}")
    elif args.no_sim:
        sys.exit("--no-sim requires --sim-csv")
    else:
        try:
            import sim_eval
            seq_steps = dead_reckon.load_sequence(args.sequence)
            theta = _load_theta(args.calib)
            print("[demo] running Isaac once (fixed params) — this needs the GPU + ROS ...")
            produced = sim_eval.run_trial(theta, seq_steps, surface="demo",
                                          combine_mode=args.combine_mode)
            import shutil
            shutil.copy2(produced, sim_csv)
        except Exception as e:
            sys.exit(f"[demo] sim run failed ({e}).\n"
                     "On the rig: `source ros2env.sh` first. Offline: pass --sim-csv <file>.")

    # 3. overlay video in the matched overhead view
    setup = ro.load_setup(args.setup)
    tracks = [ro.load_track(str(ref_csv)), ro.load_track(str(sim_csv))]
    mp4 = outdir / "demo_overhead.mp4"
    ro.render(tracks, setup, out=str(mp4), fps=args.fps, labels=[ref_label, "sim (/odom)"])

    # 4. error metrics + overlay plot (reuse compare_paths.py)
    cmp_png = outdir / "compare.png"
    cp = REPO / "sim2real_calib" / "trials" / "compare_paths.py"
    print("[demo] error metrics (reference vs sim):")
    subprocess.run([sys.executable, str(cp), str(ref_csv), str(sim_csv), "--out", str(cmp_png)],
                   check=False)
    print(f"\n[demo] done. Artifacts in {outdir}:")
    print(f"  overhead video : {mp4}")
    print(f"  comparison plot: {cmp_png}")
    print(f"  paths          : {ref_csv.name}, {sim_csv.name}")
    return 0


def dead_reckon_main(seq, out):
    rows = dead_reckon.integrate(dead_reckon.load_sequence(seq))
    import csv
    with open(out, "w") as f:
        w = csv.writer(f); w.writerow(["t", "x", "y", "yaw"])
        for r in rows:
            w.writerow([f"{r[0]:.3f}", f"{r[1]:.5f}", f"{r[2]:.5f}", f"{r[3]:.5f}"])


if __name__ == "__main__":
    raise SystemExit(main())
