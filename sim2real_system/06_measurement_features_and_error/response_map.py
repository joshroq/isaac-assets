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


# Per non-idle step of the sequence: the commanded (v, w) and the achieved signed
# v_k<i> / w_k<i> of every run (features.extract).
def step_responses(steps, csv_paths):
    """-> [{i, v_cmd, w_cmd, v: [per run], w: [per run]}] for every non-idle step."""
    run_features = [F.extract(*F.load_csv(str(csv_path)), steps) for csv_path in csv_paths]
    responses = []
    for step_index, step in enumerate(steps):
        if abs(step["linear"]) < 1e-9 and abs(step["angular"]) < 1e-9:
            continue
        responses.append(dict(i=step_index, v_cmd=float(step["linear"]), w_cmd=float(step["angular"]),
                        v=[run[f"v_k{step_index}"] for run in run_features if f"v_k{step_index}" in run],
                        w=[run[f"w_k{step_index}"] for run in run_features if f"w_k{step_index}" in run]))
    return responses


# Add mean / sd / min / max of the achieved v and w to every step row (in place).
def summarise(rows):
    for row in rows:
        for axis in ("v", "w"):
            values = np.asarray(row[axis], float)
            row[f"{axis}_mean"] = float(values.mean()) if len(values) else None
            row[f"{axis}_sd"] = float(values.std(ddof=1)) if len(values) > 1 else None
            row[f"{axis}_min"] = float(values.min()) if len(values) else None
            row[f"{axis}_max"] = float(values.max()) if len(values) else None
    return rows


# Pair real and sim step rows by step index.
def merge(real_rows, sim_rows):
    """Pair real and sim by step index (same sequence)."""
    sim_by_step = {row["i"]: row for row in sim_rows}
    merged = []
    for row in real_rows or sim_rows:
        sim_row = sim_by_step.get(row["i"], {}) if real_rows else row
        merged.append(dict(i=row["i"], v_cmd=row["v_cmd"], w_cmd=row["w_cmd"],
                        real=None if not real_rows else {key: row[key] for key in (
                            "v_mean", "v_sd", "w_mean", "w_sd", "v_min", "v_max", "w_min", "w_max")},
                        sim=None if not sim_row else {key: sim_row.get(key) for key in ("v_mean", "v_sd", "w_mean", "w_sd")}))
    return merged


# Sim-real gap over all steps: z = (sim mean - real mean) / real sd (sd floored like
# features.aggregate); returns count, rms z, worst |z| and how many sim values fall
# outside the range of every real repeat.
def gap(rows):
    """Per-step sim-real gap in units of the real repeat spread (sd floored like features)."""
    z_scores, outside_count = [], 0
    for row in rows:
        if not row["real"] or not row["sim"] or row["sim"]["v_mean"] is None or row["real"]["v_mean"] is None:
            continue
        for axis, sd_floor in (("v", F.ABS_SIGMA_FLOOR["v_k"]), ("w", F.ABS_SIGMA_FLOOR["w_k"])):
            real_sd = max(row["real"][f"{axis}_sd"] or 0.0, sd_floor, 0.02 * abs(row["real"][f"{axis}_mean"]))
            z_scores.append((row["sim"][f"{axis}_mean"] - row["real"][f"{axis}_mean"]) / real_sd)
            # outside every real repeat (by more than the floor): robust with few repeats
            range_low, range_high = row["real"][f"{axis}_min"] - sd_floor, row["real"][f"{axis}_max"] + sd_floor
            outside_count += not (range_low <= row["sim"][f"{axis}_mean"] <= range_high)
    z_array = np.asarray(z_scores)
    return dict(n=int(len(z_array)), rms_z=float(np.sqrt(np.mean(z_array ** 2))) if len(z_array) else None,
                max_abs_z=float(np.max(np.abs(z_array))) if len(z_array) else None, outside_real_range=outside_count)


# Text table: command vs real vs sim per step.
def table(rows, label=""):
    lines = [f"{label}  step |  cmd v   cmd w |  real v   real w |   sim v    sim w",
             "-" * 72]
    format_cell = lambda d, k: "    -  " if not d or d.get(k) is None else f"{d[k]:+7.3f}"
    for row in rows:
        lines.append(f"{'':{len(label)}}  {row['i']:4d} | {row['v_cmd']:+6.2f} {row['w_cmd']:+6.2f} | "
                     f"{format_cell(row['real'], 'v_mean')} {format_cell(row['real'], 'w_mean')} | "
                     f"{format_cell(row['sim'], 'v_mean')} {format_cell(row['sim'], 'w_mean')}")
    return "\n".join(lines)


# Three panels: command -> achieved arrows in the (v, w) plane, yaw-rate gain vs |v|,
# forward-speed gain vs |w| (real green, sim blue).
def plot(rows, out, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for row in rows:
        v_cmd, w_cmd = row["v_cmd"], row["w_cmd"]
        axes[0].plot(v_cmd, w_cmd, "k+", ms=8)
        for achieved, colour in ((row["real"], REAL_C), (row["sim"], SIM_C)):
            if not achieved or achieved["v_mean"] is None:
                continue
            axes[0].annotate("", (achieved["v_mean"], achieved["w_mean"]), (v_cmd, w_cmd),
                           arrowprops=dict(arrowstyle="->", color=colour, lw=1.4))
            if abs(w_cmd) > 1e-6:
                axes[1].plot(abs(v_cmd), achieved["w_mean"] / w_cmd, "o", color=colour, mfc=colour if abs(v_cmd) > 1e-6 else "none")
            if abs(v_cmd) > 1e-6:
                axes[2].plot(abs(w_cmd), achieved["v_mean"] / v_cmd, "o", color=colour)
    axes[0].set(xlabel="v [m/s]", ylabel="w [rad/s]", title="command (+) -> achieved")
    axes[1].set(xlabel="|v_cmd| [m/s]  (0 = spin in place)", ylabel="w_ach / w_cmd",
              title="yaw-rate gain")
    axes[2].set(xlabel="|w_cmd| [rad/s]  (0 = straight)", ylabel="v_ach / v_cmd",
              title="forward-speed gain")
    for axis in axes:
        axis.grid(alpha=.3)
    for axis in axes[1:]:
        axis.axhline(1.0, color="k", ls=":", lw=1)
    axes[0].plot([], [], color=REAL_C, label="real")
    axes[0].plot([], [], color=SIM_C, label="sim")
    axes[0].legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


# Command line: map of real and/or sim tracks for one sequence (+ PNG / JSON).
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--real", nargs="*", default=[], help="real track CSV(s) (repeats)")
    parser.add_argument("--sim", nargs="*", default=[], help="sim track CSV(s)")
    parser.add_argument("--out", help="plot PNG")
    parser.add_argument("--json", help="write rows + gap as JSON")
    args = parser.parse_args(argv)
    if not args.real and not args.sim:
        parser.error("give --real and/or --sim tracks")
    steps = F.load_sequence(args.sequence)
    name = Path(args.sequence).stem
    real = summarise(step_responses(steps, args.real)) if args.real else []
    sim = summarise(step_responses(steps, args.sim)) if args.sim else []
    rows = merge(real, sim)
    print(table(rows, name))
    gap_stats = gap(rows)
    if gap_stats["n"]:
        print(f"\nsim-real gap over {gap_stats['n']} step terms: rms z {gap_stats['rms_z']:.2f}, "
              f"worst |z| {gap_stats['max_abs_z']:.1f}  (z = gap / real repeat sd); "
              f"{gap_stats['outside_real_range']}/{gap_stats['n']} sim values outside the range of every real repeat")
    if args.out:
        plot(rows, args.out, f"{name}: command -> response")
        print(f"wrote {args.out}")
    if args.json:
        Path(args.json).write_text(json.dumps({"sequence": name, "rows": rows, "gap": gap_stats}, indent=1))
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
