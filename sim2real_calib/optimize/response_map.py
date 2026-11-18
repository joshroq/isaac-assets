#!/usr/bin/env python3
"""Command -> response map: what twist the rover actually achieves for each command.

For every non-idle step of a sequence, the signed steady forward speed v_k<i> and yaw rate
w_k<i> (features.py) are paired with the commanded (v, w). Real repeats are aggregated
(mean +/- sd); sim runs likewise. This is the comparison that matters for navigation:
Nav2 (TEB + rotation shim) and the keyboard teleop only ever command (v, w) pairs inside
|v| <= 0.26 m/s, |w| <= 1.0 rad/s, so matching this map over that box is matching every
motion the rover will be asked to make.

    python3 optimize/response_map.py --sequence parallel/sequences/R3_arcs_slow.yaml \\
        --real loop/runs/<s>/iter_01/real/R3_arcs_slow/rep*.csv \\
        --sim  loop/runs/<s>/iter_01/opt/eval_001_before/R3_arcs_slow.csv \\
        --out /tmp/r3_map.png --json /tmp/r3_map.json

Plot: (left) the (v, w) command plane, an arrow from each command to what real (green)
and sim (blue) achieved; (middle) yaw-rate gain w_ach/w_cmd vs |v_cmd| - arcs vs spins;
(right) speed gain v_ach/v_cmd vs |w_cmd| - how much turning bleeds forward speed.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import features as F  # noqa: E402

REAL_C, SIM_C = "#2ca02c", "#1f77b4"


def step_responses(steps, csv_paths):
    """-> [{i, v_cmd, w_cmd, v: [per run], w: [per run]}] for every non-idle step."""
    runs = [F.extract(*F.load_csv(str(p)), steps) for p in csv_paths]
    out = []
    for i, s in enumerate(steps):
        if abs(s["linear"]) < 1e-9 and abs(s["angular"]) < 1e-9:
            continue
        out.append(dict(i=i, v_cmd=float(s["linear"]), w_cmd=float(s["angular"]),
                        v=[r[f"v_k{i}"] for r in runs if f"v_k{i}" in r],
                        w=[r[f"w_k{i}"] for r in runs if f"w_k{i}" in r]))
    return out


def summarise(rows):
    for r in rows:
        for k in ("v", "w"):
            a = np.asarray(r[k], float)
            r[f"{k}_mean"] = float(a.mean()) if len(a) else None
            r[f"{k}_sd"] = float(a.std(ddof=1)) if len(a) > 1 else None
            r[f"{k}_min"] = float(a.min()) if len(a) else None
            r[f"{k}_max"] = float(a.max()) if len(a) else None
    return rows


def merge(real_rows, sim_rows):
    """Pair real and sim by step index (same sequence)."""
    sim = {r["i"]: r for r in sim_rows}
    out = []
    for r in real_rows or sim_rows:
        s = sim.get(r["i"], {}) if real_rows else r
        out.append(dict(i=r["i"], v_cmd=r["v_cmd"], w_cmd=r["w_cmd"],
                        real=None if not real_rows else {k: r[k] for k in (
                            "v_mean", "v_sd", "w_mean", "w_sd", "v_min", "v_max", "w_min", "w_max")},
                        sim=None if not s else {k: s.get(k) for k in ("v_mean", "v_sd", "w_mean", "w_sd")}))
    return out


def gap(rows):
    """Per-step sim-real gap in units of the real repeat spread (sd floored like features)."""
    zs, outside = [], 0
    for r in rows:
        if not r["real"] or not r["sim"] or r["sim"]["v_mean"] is None or r["real"]["v_mean"] is None:
            continue
        for k, floor in (("v", F.ABS_SIGMA_FLOOR["v_k"]), ("w", F.ABS_SIGMA_FLOOR["w_k"])):
            sd = max(r["real"][f"{k}_sd"] or 0.0, floor, 0.02 * abs(r["real"][f"{k}_mean"]))
            zs.append((r["sim"][f"{k}_mean"] - r["real"][f"{k}_mean"]) / sd)
            # outside every real repeat (by more than the floor): robust with few repeats
            lo, hi = r["real"][f"{k}_min"] - floor, r["real"][f"{k}_max"] + floor
            outside += not (lo <= r["sim"][f"{k}_mean"] <= hi)
    z = np.asarray(zs)
    return dict(n=int(len(z)), rms_z=float(np.sqrt(np.mean(z ** 2))) if len(z) else None,
                max_abs_z=float(np.max(np.abs(z))) if len(z) else None, outside_real_range=outside)


def table(rows, label=""):
    lines = [f"{label}  step |  cmd v   cmd w |  real v   real w |   sim v    sim w",
             "-" * 72]
    f = lambda d, k: "    -  " if not d or d.get(k) is None else f"{d[k]:+7.3f}"
    for r in rows:
        lines.append(f"{'':{len(label)}}  {r['i']:4d} | {r['v_cmd']:+6.2f} {r['w_cmd']:+6.2f} | "
                     f"{f(r['real'], 'v_mean')} {f(r['real'], 'w_mean')} | "
                     f"{f(r['sim'], 'v_mean')} {f(r['sim'], 'w_mean')}")
    return "\n".join(lines)


def plot(rows, out, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))
    for r in rows:
        vc, wc = r["v_cmd"], r["w_cmd"]
        ax[0].plot(vc, wc, "k+", ms=8)
        for d, c in ((r["real"], REAL_C), (r["sim"], SIM_C)):
            if not d or d["v_mean"] is None:
                continue
            ax[0].annotate("", (d["v_mean"], d["w_mean"]), (vc, wc),
                           arrowprops=dict(arrowstyle="->", color=c, lw=1.4))
            if abs(wc) > 1e-6:
                ax[1].plot(abs(vc), d["w_mean"] / wc, "o", color=c, mfc=c if abs(vc) > 1e-6 else "none")
            if abs(vc) > 1e-6:
                ax[2].plot(abs(wc), d["v_mean"] / vc, "o", color=c)
    ax[0].set(xlabel="v [m/s]", ylabel="w [rad/s]", title="command (+) -> achieved")
    ax[1].set(xlabel="|v_cmd| [m/s]  (0 = spin in place)", ylabel="w_ach / w_cmd",
              title="yaw-rate gain")
    ax[2].set(xlabel="|w_cmd| [rad/s]  (0 = straight)", ylabel="v_ach / v_cmd",
              title="forward-speed gain")
    for a in ax:
        a.grid(alpha=.3)
    for a in ax[1:]:
        a.axhline(1.0, color="k", ls=":", lw=1)
    ax[0].plot([], [], color=REAL_C, label="real")
    ax[0].plot([], [], color=SIM_C, label="sim")
    ax[0].legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--real", nargs="*", default=[], help="real track CSV(s) (repeats)")
    ap.add_argument("--sim", nargs="*", default=[], help="sim track CSV(s)")
    ap.add_argument("--out", help="plot PNG")
    ap.add_argument("--json", help="write rows + gap as JSON")
    args = ap.parse_args(argv)
    if not args.real and not args.sim:
        ap.error("give --real and/or --sim tracks")
    steps = F.load_sequence(args.sequence)
    name = Path(args.sequence).stem
    real = summarise(step_responses(steps, args.real)) if args.real else []
    sim = summarise(step_responses(steps, args.sim)) if args.sim else []
    rows = merge(real, sim)
    print(table(rows, name))
    g = gap(rows)
    if g["n"]:
        print(f"\nsim-real gap over {g['n']} step terms: rms z {g['rms_z']:.2f}, "
              f"worst |z| {g['max_abs_z']:.1f}  (z = gap / real repeat sd); "
              f"{g['outside_real_range']}/{g['n']} sim values outside the range of every real repeat")
    if args.out:
        plot(rows, args.out, f"{name}: command -> response")
        print(f"wrote {args.out}")
    if args.json:
        Path(args.json).write_text(json.dumps({"sequence": name, "rows": rows, "gap": g}, indent=1))
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
