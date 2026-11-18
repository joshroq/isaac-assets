#!/usr/bin/env python3
"""Sim-vs-real DEMO SUITE: run the full battery (T1-T5) end to end, per surface.

For EACH test it: drives the real rover + records the camera path, drives the sim +
records /odom, then builds the webcam-matched overhead overlay and the error metrics.
No optimizer, no parameter changes - this is the before-calibration demo that shows
how far sim and real currently diverge on every maneuver.

Per test Ti -> runs/<surface>/<Ti>/ :  real.csv  sim.csv  overhead.mp4  compare.png  metrics.json
Across tests ->  runs/<surface>/summary.{md,csv}

Run from a ROS-sourced shell (source ros2env.sh) with:
  - overhead camera streaming, grid calibrated, sticker geometry measured
  - the rover base driver up:  ssh rover 'bash ~/rover_nav/start_bringup.sh'
  - Isaac installed and GPU available; each sim trial launches and stops its own instance

    python3 run_suite.py --surface heavy_carpet
    python3 run_suite.py --surface concrete --tests T1,T2
    # offline re-viz/compare from CSVs already recorded:
    python3 run_suite.py --surface heavy_carpet --skip-real --skip-sim
"""
from __future__ import annotations

import argparse
import csv
import datetime
import glob
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SC = HERE.parent                      # sim2real_calib/
SEQ_DIR = SC / "parallel" / "sequences"
TRIALS = SC / "trials"
sys.path.insert(0, str(SC / "overhead"))

ALL_TESTS = ["T1", "T2", "T3", "T4", "T5"]
ROVER = os.environ.get("ROVER_HOST", "rover")


def preflight(args):
    """Check the rig ONCE up front so failures surface before the rover is driven."""
    ok = True
    print("[suite] preflight ...")
    if not args.skip_real:
        import cv2
        cap = cv2.VideoCapture("udp://@:5000?overrun_nonfatal=1&fifo_size=50000000",
                               cv2.CAP_FFMPEG)
        got = False
        t0 = time.time()
        while time.time() - t0 < 8:
            r, _ = cap.read()
            if r:
                got = True
                break
        cap.release()
        print(f"  camera stream (UDP 5000) : {'OK' if got else 'DOWN - start the Pi camera'}")
        ok = ok and got
        r = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", ROVER,
             'source ~/rover_nav/local_ros_env.sh >/dev/null 2>&1; '
             'timeout 6 ros2 topic info /cmd_vel 2>/dev/null | '
             'sed -n "s/.*Subscription count: *//p"'],
            capture_output=True, text=True)
        subs = (r.stdout.strip() or "0")
        rok = subs.isdigit() and int(subs) >= 1
        msg = (f"OK ({subs})" if rok else
               f"NOT LISTENING - start: ssh {ROVER} bash ~/rover_nav/start_bringup.sh")
        print(f"  rover /cmd_vel subscriber: {msg}")
        ok = ok and rok
    if not args.skip_sim:
        # trial_sim.sh starts Isaac per trial and verifies live native ROS samples.
        isaacpy = os.environ.get("ISAACPY", str(TRIALS.parents[1] / "isaacpy_clean.sh"))
        sok = os.path.exists(isaacpy)
        print(f"  isaacpy (sim launcher)   : {'OK' if sok else 'NOT FOUND at ' + isaacpy}")
        ok = ok and sok
    return ok


def seq_for(test):
    hits = sorted(glob.glob(str(SEQ_DIR / f"{test}_*.yaml")))
    if not hits:
        sys.exit(f"no sequence file for {test} in {SEQ_DIR}")
    return hits[0]


def run(cmd, **kw):
    print("  $", " ".join(str(c) for c in cmd))
    return subprocess.run([str(c) for c in cmd], **kw)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--surface", required=True, help="surface label (e.g. heavy_carpet)")
    ap.add_argument("--tests", default=",".join(ALL_TESTS), help="comma list, e.g. T1,T2")
    ap.add_argument("--outroot", default=str(HERE / "runs"))
    ap.add_argument("--skip-real", action="store_true", help="reuse existing real.csv")
    ap.add_argument("--skip-sim", action="store_true", help="reuse existing sim.csv")
    ap.add_argument("--no-pause", action="store_true", help="don't pause to reposition the rover")
    ap.add_argument("--skip-preflight", action="store_true", help="skip the upfront rig check")
    ap.add_argument("--setup", default=str(SC / "overhead" / "camera_setup.yaml"))
    ap.add_argument("--fps", type=float, default=30.0)
    args = ap.parse_args()

    tests = [t.strip() for t in args.tests.split(",") if t.strip()]
    base = Path(args.outroot) / args.surface
    base.mkdir(parents=True, exist_ok=True)
    print(f"[suite] surface={args.surface}  tests={tests}  -> {base}")

    if not args.skip_preflight and not preflight(args):
        sys.exit("[suite] preflight failed - fix the above (or --skip-preflight to force).")

    import render_overhead as ro
    try:
        setup = ro.load_setup(args.setup)
    except Exception as e:
        setup = None
        print(f"[suite] WARN: overhead setup unavailable ({e}); skipping mp4s.")

    rows = []
    for ti in tests:
        seq = seq_for(ti)
        d = base / ti
        d.mkdir(parents=True, exist_ok=True)
        real_csv, sim_csv = d / "real.csv", d / "sim.csv"
        print(f"\n===== {ti}  ({Path(seq).name}) =====")

        # --- real trial ---
        if not args.skip_real:
            if not args.no_pause:
                input(f"[suite] position the rover for {ti}, then press Enter to drive it...")
            run([TRIALS / "trial_real.sh", seq, real_csv])
        if not real_csv.exists():
            print(f"[suite] {ti}: no real.csv - skipping this test."); continue

        # --- sim trial ---
        if not args.skip_sim:
            run([TRIALS / "trial_sim.sh", seq, sim_csv])
        if not sim_csv.exists():
            print(f"[suite] {ti}: no sim.csv - skipping this test."); continue

        # --- overhead overlay video (matched webcam view) ---
        if setup is not None:
            try:
                tracks = [ro.load_track(str(real_csv)), ro.load_track(str(sim_csv))]
                ro.render(tracks, setup, out=str(d / "overhead.mp4"), fps=args.fps,
                          labels=["real (camera)", "sim (/odom)"])
            except Exception as e:
                print(f"[suite] {ti}: overhead render skipped ({e})")

        # --- error metrics + plot ---
        mj = d / "metrics.json"
        run([sys.executable, TRIALS / "compare_paths.py", real_csv, sim_csv,
             "--out", d / "compare.png", "--json", mj, "--label", ti], check=False)
        if mj.exists():
            m = json.load(open(mj)); m["test"] = ti; rows.append(m)

    # --- summary ---
    if rows:
        cols = ["test", "endpoint_err_m", "mean_err_m", "max_err_m", "heading_err_deg",
                "path_len_real_m", "path_len_sim_m"]
        with open(base / "summary.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
        with open(base / "summary.md", "w") as f:
            f.write(f"# sim-vs-real demo — {args.surface}\n\n")
            f.write(f"_generated {datetime.datetime.now():%Y-%m-%d %H:%M}_\n\n")
            f.write("| test | endpoint err (cm) | mean err (cm) | max err (cm) | "
                    "heading err (deg) | real len (m) | sim len (m) |\n")
            f.write("|---|---|---|---|---|---|---|\n")
            for r in rows:
                f.write(f"| {r['test']} | {r['endpoint_err_m']*100:.1f} | "
                        f"{r['mean_err_m']*100:.1f} | {r['max_err_m']*100:.1f} | "
                        f"{r['heading_err_deg']:.1f} | {r['path_len_real_m']:.3f} | "
                        f"{r['path_len_sim_m']:.3f} |\n")
        print(f"\n[suite] summary -> {base/'summary.md'}  (+ summary.csv)")
        print(open(base / "summary.md").read())
    else:
        print("\n[suite] no completed tests - nothing to summarize.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
