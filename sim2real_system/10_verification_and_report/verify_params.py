#!/usr/bin/env python3
"""Held-out check: did the calibration make the sim match the real rover better?

Drives a FRESH real battery (station starts, same as the loop) that the optimiser never
saw, then simulates the same battery under each parameter set (default: the schema
baseline vs a calibrated surfaces/*.yaml) and scores every set against the same real
data: per-feature z, rms z, and start-aligned path error per test. Writes
loop/runs/verify_<stamp>/report.md (+ path maps / compare plots per test and set).

    python3 loop/verify_params.py --calibrated ../surfaces/carpet_loop.yaml
    python3 loop/verify_params.py --calibrated X.yaml --real-from loop/runs/<s>/iter_02/real
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import closed_loop as CLP          # noqa: E402  (shares all the machinery)
from closed_loop import C, F, CAL, ST   # noqa: E402


def load_theta(path, surface, measured):
    import yaml
    if path in (None, "baseline"):
        return CAL.start_theta(measured, surface, None)
    doc = yaml.safe_load(Path(path).read_text()) or {}
    seed = doc.get("calibrated") or C.flatten(doc.get("sim_calibration_yaml", doc))
    return CAL.start_theta(measured, surface, seed)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--surface", default="carpet")
    ap.add_argument("--calibrated", required=True, help="calibrated surfaces/*.yaml")
    ap.add_argument("--baseline", default="baseline",
                    help="'baseline' (schema start + floor band) or another yaml")
    ap.add_argument("--tests", default="T1,T2,T4")
    ap.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"],
                    help="friction combine mode for the CALIBRATED set (use the one it was "
                         "fitted with, e.g. average for concrete_physics.yaml)")
    ap.add_argument("--baseline-combine", default="min", choices=["min", "average", "multiply"],
                    help="combine mode for the baseline set (min = how the sim ran before)")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--real-from", default=None)
    ap.add_argument("--keep-bringup", action="store_true")
    ap.add_argument("--allow-approximate", action="store_true")
    args = ap.parse_args()

    cfg = CLP.load_config(str(CLP.CONFIG))
    tests = [CLP.resolve_test(t.strip()) for t in args.tests.split(",") if t.strip()]
    plan = {stem: F.load_sequence(str(path)) for stem, path in tests}
    scoring = {t: cfg["scoring"].get(t, []) for t in plan}
    out = HERE / "runs" / f"verify_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out.mkdir(parents=True)
    log = CLP.Log(out / "verify.log")
    measured = CAL.load_measured(str(CLP.SC / "optimize" / "measured.yaml"))
    sets = {"baseline": load_theta(args.baseline, args.surface, measured),
            "calibrated": load_theta(args.calibrated, args.surface, measured)}
    keys = [k for k in C.OPTIMISABLE if abs(sets["baseline"][k] - sets["calibrated"][k]) > 1e-9]
    log(f"[verify] parameter sets differ in: "
        + ", ".join(f"{k} {sets['baseline'][k]:.4g} -> {sets['calibrated'][k]:.4g}" for k in keys))

    # ---- fresh real data ----
    rover = cam = tracker = None
    try:
        if args.real_from:
            real = CLP.load_real_from(args.real_from, plan)
            sys.path.insert(0, str(CLP.SC / "viz"))
            from metric_overlay import load_calib
            H_m2img = load_calib(str(CLP.SC / "calib" / "grid_calib.json"))[1]
        else:
            from camera_track import CameraTrack
            from rover_link import RoverLink
            from localizer import FusedTracker
            fz = cfg.get("fusion", {})
            cam = CameraTrack(allow_approximate=args.allow_approximate)
            cam.latency_s = fz.get("cam_latency_s", 0.0)
            CLP.camera_check(cam, log)
            H_m2img = cam.H_m2img
            rover = RoverLink(log=log, logfile=str(out / "rover_agent.log"), fusion_cfg=fz)
            rover.open()
            ST.set_guard(rover, cfg)
            tracker = FusedTracker(cam, rover, cam_latency_s=fz.get("cam_latency_s", 0.2),
                                   gain=fz.get("gain", 0.3))
            view = CLP.LiveView(cam, out, False)
            real = CLP.real_phase(rover, cam, tracker, cfg, plan, args.repeats, out, log, view)
            p = cfg.get("park")
            if p:
                try:
                    CLP.go_to_pose(rover, tracker, (p["x"], p["y"], math.radians(p["yaw"])),
                                   cfg, log=log)
                except Exception as exc:
                    log(f"[real] park failed: {exc}")
    finally:
        if tracker is not None and tracker is not cam:
            tracker.close()
        if rover is not None:
            rover.close(stop_bringup=not args.keep_bringup)
        if cam is not None:
            cam.close()
    targets = CLP.build_targets(real, plan, scoring, log)

    # ---- sim under each parameter set, scored on the SAME real data ----
    results = {}
    run_plan = {t: plan[t] for t in plan if real.get(t)}
    for name, theta in sets.items():
        log(f"[verify] sim with {name} parameters")
        combine = args.combine_mode if name == "calibrated" else args.baseline_combine
        log(f"[verify]   friction combine mode {combine}")
        csvs = CLP.run_battery(theta, run_plan, out / f"sim_{name}", combine)
        feats = {t: F.extract(*F.load_csv(csvs[t]), run_plan[t]) for t in run_plan}
        loss, n, parts = CLP.score(feats, targets)
        paths = {}
        for t in run_plan:
            tdir = out / t
            tdir.mkdir(exist_ok=True)
            per_rep = []
            for i, r in enumerate(real[t]):
                m = CLP.compare_png(r["csv"], csvs[t], tdir / f"compare_{name}_rep{i+1}.png",
                                    tdir / f"metrics_{name}_rep{i+1}.json", f"{t} {name}")
                if m:
                    per_rep.append(m)
            paths[t] = {k: float(np.mean([m[k] for m in per_rep]))
                        for k in ("endpoint_err_m", "mean_err_m", "heading_err_deg")} if per_rep else {}
        results[name] = {"loss": loss, "rms_z": math.sqrt(loss / max(n, 1)), "z": parts,
                         "paths": paths, "csvs": csvs,
                         "feats": {t: {k: v for k, v in f.items() if CLP.is_scored(k, scoring.get(t, []))}
                                   for t, f in feats.items()}}
        log(f"[verify] {name}: rms_z={results[name]['rms_z']:.2f}  z={parts}")
    for t in run_plan:                   # one map per test with both sim paths
        try:
            CLP.path_maps(t, real[t], results["baseline"]["csvs"][t],
                          results["calibrated"]["csvs"][t], out / t, H_m2img, log,
                          "red = baseline sim, cyan = calibrated sim")
        except Exception as exc:
            log(f"[viz] {t}: {exc}")

    # ---- report ----
    b, c = results["baseline"], results["calibrated"]
    L = [f"# Held-out verification — {args.surface}", "",
         f"{datetime.datetime.now():%Y-%m-%d %H:%M} · fresh real trials: "
         + ", ".join(f"{t} ×{len(real[t])}" for t in run_plan), "",
         "Changed parameters: " + ", ".join(f"`{k}` {sets['baseline'][k]:.4g} → "
                                            f"{sets['calibrated'][k]:.4g}" for k in keys), "",
         "| metric | baseline sim | calibrated sim |", "|---|---|---|",
         f"| rms feature error (σ units) | {b['rms_z']:.2f} | {c['rms_z']:.2f} |"]
    for t in run_plan:
        for k, lab, sc_ in (("endpoint_err_m", "endpoint error", 100), ("mean_err_m", "mean path error", 100),
                            ("heading_err_deg", "final heading error", 1)):
            u = "cm" if sc_ == 100 else "°"
            if b["paths"].get(t) and c["paths"].get(t):
                L.append(f"| {t} {lab} | {b['paths'][t][k]*sc_:.1f} {u} | {c['paths'][t][k]*sc_:.1f} {u} |")
    L += ["", "## Per-feature (real mean vs sim)", "",
          "| test.feature | real | baseline sim | calibrated sim | z baseline | z calibrated |",
          "|---|---|---|---|---|---|"]
    for t, tg in targets.items():
        for f_, v in tg["features"].items():
            L.append(f"| {t}.{f_} | {v['mean']:.4f} ± {v['sigma']:.4f} | "
                     f"{b['feats'].get(t, {}).get(f_, float('nan')):.4f} | "
                     f"{c['feats'].get(t, {}).get(f_, float('nan')):.4f} | "
                     f"{b['z'].get(f'{t}.{f_}')} | {c['z'].get(f'{t}.{f_}')} |")
    L += ["", "Path errors are start-aligned and averaged over the real repeats. "
          "`<test>/paths_map.png|mp4`: real (green) vs baseline sim (red) vs calibrated sim (cyan) "
          "on the overhead camera view."]
    (out / "report.md").write_text("\n".join(L) + "\n")
    (out / "results.json").write_text(json.dumps(
        {k: {kk: vv for kk, vv in v.items() if kk != "csvs"} for k, v in results.items()},
        indent=2, default=float))
    log(f"[verify] rms_z baseline {b['rms_z']:.2f} -> calibrated {c['rms_z']:.2f}; "
        f"report {out/'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
