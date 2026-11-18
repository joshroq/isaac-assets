#!/usr/bin/env python3
"""Build the comparison report for a navcmp session.

    python3 navcmp/report.py navcmp/runs/<session>          # -> <session>/report/report.md

Writes report/report.md (read this first), summary.json, leg_metrics.csv, path_gaps.csv
and the figures paths.png, agreement.png, metric_z.png, speed.png. Metric definitions and
units are in metrics.py's docstring (repeated in the report).
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import course as C  # noqa: E402
import metrics as M  # noqa: E402

# categorical slots (validated: dataviz validate_palette, light surface) - fixed per
# condition so a condition keeps its colour in every figure
SLOTS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
FIXED = {"real": SLOTS[0], "sim_physics": SLOTS[1], "sim_default": SLOTS[2], "fake_real": SLOTS[3]}
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df"
OBSTACLE = "#bdbcb6"


def colors(names):
    out, free = {}, [c for c in SLOTS if c not in FIXED.values()]
    for n in names:
        out[n] = FIXED.get(n) or free.pop(0)
    return out


def _style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def _draw_course(ax, course, real_map=None, reg=None):
    from matplotlib.patches import Polygon
    if real_map is not None and reg is not None:
        T = tuple(reg["T_course_map"])
        res, w, h = real_map["resolution"], real_map["width"], real_map["height"]
        o = tuple(real_map["origin"])
        xs, ys = [], []
        data = real_map["data"]
        for r in range(0, h):
            for c in range(0, w):
                if data[r * w + c] >= 65:
                    x, y = C.apply(o, (c + 0.5) * res, (r + 0.5) * res)
                    x, y = C.apply(T, x, y)
                    xs.append(x)
                    ys.append(y)
        ax.scatter(xs, ys, s=1.5, color=MUTED, linewidths=0, zorder=1, label="real lidar map")
    for b in course.boxes:
        ax.add_patch(Polygon(b.corners(), closed=True, facecolor=OBSTACLE, edgecolor=MUTED,
                             linewidth=0.6, zorder=2))
    xmin, xmax, ymin, ymax = course.fence
    ax.plot([xmin, xmax, xmax, xmin, xmin], [ymin, ymin, ymax, ymax, ymin], color=MUTED,
            linewidth=0.8, linestyle=(0, (3, 3)), zorder=2)
    sx, sy, sth = course.start
    ax.annotate("", xy=(sx + 0.18 * math.cos(sth), sy + 0.18 * math.sin(sth)), xytext=(sx, sy),
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.2), zorder=6)
    ax.text(sx, sy - 0.07, "start", fontsize=7, color=INK2, ha="center", va="top", zorder=6)
    for g in course.goals:
        ax.plot(g.x, g.y, marker="x", color=INK, markersize=7, mew=1.5, zorder=6)
        ax.text(g.x + 0.05, g.y + 0.05, g.name, fontsize=7, color=INK2, zorder=6)
    x0, x1, y0, y1 = course.bounds()
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")


def _trial_xy(cond, trial):
    rows = cond["traces"].get(trial, [])
    return ([M._f(r["x"]) for r in rows], [M._f(r["y"]) for r in rows])


def fig_paths(path, course, conds, order, col, real="real"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    others = [n for n in order if n != real]
    panels = [real] + others
    ncol = min(len(panels), 3)
    nrow = math.ceil(len(panels) / ncol)
    x0, x1, y0, y1 = course.bounds()
    pw = 4.6 if (x1 - x0) <= 1.5 * (y1 - y0) else 6.0          # wide courses: wider panels
    ph = min(max(pw * (y1 - y0) / (x1 - x0), 2.0), 5.0) + 0.9
    if pw == 6.0:
        ncol = min(len(panels), 2)
        nrow = math.ceil(len(panels) / ncol)
    fig, axes = plt.subplots(nrow, ncol, figsize=(pw * ncol, ph * nrow), squeeze=False)
    rmap = conds.get(real, {}).get("map")
    reg = conds.get(real, {}).get("meta", {}).get("registration")
    for ax, name in zip(axes.flat, panels):
        _style(ax)
        _draw_course(ax, course, rmap, reg)
        shown = [real] if name == real else [real, name]
        for n in shown:
            c = conds.get(n)
            if not c:
                continue
            for t in sorted(c["traces"]):
                xs, ys = _trial_xy(c, t)
                ax.plot(xs, ys, color=col[n], linewidth=1.6 if n == name else 1.2,
                        alpha=0.9 if n == name else 0.55, zorder=4 if n == name else 3,
                        label=n if t == min(c["traces"]) else None)
        ax.set_title(name if name == real else f"{real} vs {name}", fontsize=10, color=INK, loc="left")
        ax.legend(fontsize=7, frameon=False, loc="upper right")
        ax.set_xlabel("x [m] (course / grid frame)", fontsize=8, color=INK2)
        ax.set_ylabel("y [m]", fontsize=8, color=INK2)
    for ax in list(axes.flat)[len(panels):]:
        ax.axis("off")
    fig.suptitle("Driven paths (SLAM pose, every trial)", fontsize=11, color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_agreement(path, res, order, col, real="real"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sims = [n for n in order if n != real and n in res["conditions"]]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.5, 0.6 + 0.55 * max(len(sims), 2)))
    for ax in (a1, a2):
        _style(ax)
        ax.grid(True, axis="x", color=GRID, linewidth=0.6)
        ax.grid(False, axis="y")
    ys = list(range(len(sims)))[::-1]
    for y, n in zip(ys, sims):
        e = res["conditions"][n]
        z, (lo, hi) = e.get("rms_z", math.nan), e.get("rms_z_ci", (math.nan, math.nan))
        a1.barh(y, z, height=0.5, color=col[n], zorder=3)
        if math.isfinite(lo):
            a1.plot([lo, hi], [y, y], color=INK, linewidth=1.2, zorder=4)
        a1.text(z, y + 0.32, f" {z:.2f}", fontsize=8, color=INK, va="bottom")
        g = e.get("path_gap_ratio", math.nan)
        a2.barh(y, g if math.isfinite(g) else 0, height=0.5, color=col[n], zorder=3)
        a2.text(g if math.isfinite(g) else 0, y + 0.32,
                f" {g:.2f}  ({e.get('frechet_cm', math.nan):.0f} cm)" if math.isfinite(g) else " n/a",
                fontsize=8, color=INK, va="bottom")
    floor = res.get("real_floor_rms_z", math.nan)
    if math.isfinite(floor):
        a1.axvline(floor, color=INK2, linestyle=(0, (4, 3)), linewidth=1)
        a1.text(floor, -0.55, f" real-vs-real {floor:.2f}", fontsize=7, color=INK2, ha="left", va="bottom")
    a2.axvline(1.0, color=INK2, linestyle=(0, (4, 3)), linewidth=1)
    a2.text(1.0, -0.55, " real-vs-real = 1", fontsize=7, color=INK2, ha="left", va="bottom")
    for ax in (a1, a2):
        ax.set_yticks(ys)
        ax.set_yticklabels(sims, fontsize=9, color=INK)
        ax.set_ylim(-0.6, len(sims) - 0.2)
        ax.set_xlim(left=0)
    a1.set_title("Behaviour gap: rms z (lower = closer to real)", fontsize=9, color=INK, loc="left")
    a2.set_title("Path gap ratio: Frechet vs real / real vs real", fontsize=9, color=INK, loc="left")
    a1.set_xlabel("rms z  (1 ~ one real repeat; bar = 95 % CI)", fontsize=8, color=INK2)
    a2.set_xlabel("ratio  (1 = as close as real repeats)", fontsize=8, color=INK2)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_metric_z(path, res, order, col, real="real"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sims = [n for n in order if n != real and n in res["conditions"]]
    mets = M.SCORE_METRICS
    fig, ax = plt.subplots(figsize=(8, 0.5 + 0.48 * len(mets)))
    _style(ax)
    ax.grid(False, axis="y")
    ax.axvspan(-1, 1, color="#f0efeb", zorder=0)
    ax.axvline(0, color=MUTED, linewidth=0.8, zorder=1)
    step = 0.6 / max(len(sims), 1)
    for k, n in enumerate(sims):
        mz = res["conditions"][n]["per_metric_mean_z"]
        for i, m in enumerate(mets):
            y = len(mets) - 1 - i + 0.3 - k * step - step / 2
            z = mz.get(m, math.nan)
            if math.isfinite(z):
                zc = max(-6, min(6, z))
                ax.plot([0, zc], [y, y], color=col[n], linewidth=2, zorder=2)
                ax.plot(zc, y, "o", color=col[n], markersize=7, mec="white", mew=1.5, zorder=3,
                        label=n if i == 0 else None)
                if abs(z) > 6:
                    ax.text(zc, y, f" {z:.0f}", fontsize=7, color=INK2, va="center")
    ax.set_yticks(range(len(mets))[::-1])
    ax.set_yticklabels([M.LABELS[m] for m in mets], fontsize=8, color=INK)
    ax.set_xlim(-6.5, 6.5)
    ax.set_xlabel("mean z = (sim - real mean) / real sd   (shaded: within the real rover's own scatter)",
                  fontsize=8, color=INK2)
    ax.set_title("Which behaviours differ, and in which direction", fontsize=10, color=INK, loc="left")
    ax.legend(fontsize=8, frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_speed(path, conds, order, col, real="real"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    others = [n for n in order if n != real and n in conds]
    if not others:
        return
    fig, axes = plt.subplots(len(others), 1, figsize=(9, 2.3 * len(others)), squeeze=False, sharex=True)
    for ax, n in zip(axes.flat, others):
        _style(ax)
        for name in (real, n):
            c = conds.get(name)
            if not c:
                continue
            for t in sorted(c["traces"]):
                rows = c["traces"][t]
                ax.plot([M._f(r["t"]) for r in rows], [M._f(r["odom_v"]) for r in rows],
                        color=col[name], linewidth=1.0, alpha=0.75 if name == n else 0.5,
                        label=name if t == min(c["traces"]) else None)
        ax.set_ylabel("speed [m/s]", fontsize=8, color=INK2)
        ax.set_title(f"{real} vs {n}: forward speed through each trial", fontsize=9, color=INK, loc="left")
        ax.legend(fontsize=7, frameon=False, loc="upper right")
    axes.flat[-1].set_xlabel("time since trial start [s]", fontsize=8, color=INK2)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_scans(path, conds, order, col, real="real"):
    """Lidar view from the start mark: each sim condition's scan over the real one."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    others = [n for n in order if n != real and conds.get(n, {}).get("start_scan")]
    rs = conds.get(real, {}).get("start_scan")
    if not rs or not others:
        return False
    fig, axes = plt.subplots(1, len(others), figsize=(4.2 * len(others), 4.2), squeeze=False)
    for ax, n in zip(axes.flat, others):
        _style(ax)
        for name, z in ((real, 2), (n, 3)):
            pts = M.scan_points(conds[name]["start_scan"])
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=4, color=col[name],
                       linewidths=0, alpha=0.8, zorder=z, label=name)
        ax.plot(0, 0, marker="+", color=INK, markersize=10, zorder=5)
        ax.set_aspect("equal")
        ax.set_title(f"{real} vs {n}", fontsize=9, color=INK, loc="left")
        ax.set_xlabel("x [m] (lidar frame)", fontsize=8, color=INK2)
        ax.legend(fontsize=7, frameon=False, loc="upper right", markerscale=3)
    fig.suptitle("Lidar scan at the start mark (valid returns only)", fontsize=10, color=INK,
                 x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def _fmt(v, nd=2):
    return "–" if v is None or not math.isfinite(v) else f"{v:.{nd}f}"


def build(session_dir):
    sd = Path(session_dir)
    cfg = json.loads((sd / "session.json").read_text())
    course = C.load(sd / "bundle" / "course.yaml")
    order = [n for n in cfg["order"] if (sd / n / "data").exists()]
    conds = {n: M.load_condition(sd / n) for n in order}
    res = M.compare(conds)
    col = colors(cfg["order"])
    out = sd / "report"
    out.mkdir(exist_ok=True)

    with (out / "leg_metrics.csv").open("w", newline="") as f:
        keys = ["condition", "trial", "leg", "leg_index", "result", "success", "straight_m"] + \
               M.SCORE_METRICS + M.DIAG_METRICS
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for n, rows in res["rows"].items():
            for r in rows:
                w.writerow(dict(r, condition=n))
    with (out / "path_gaps.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["condition", "leg", "real_trial", "other_trial", "frechet_cm", "mean_dev_cm"])
        for n, e in res["conditions"].items():
            for g in e.get("path_gaps", []):
                w.writerow([n, g["leg"], g["real_trial"], g["other_trial"],
                            round(g["frechet_cm"], 2), round(g["mean_dev_cm"], 2)])

    figs = {}
    if conds:
        fig_paths(out / "paths.png", course, conds, order, col)
        figs["paths"] = "paths.png"
    if "error" not in res and len(order) > 1:
        fig_agreement(out / "agreement.png", res, order, col)
        fig_metric_z(out / "metric_z.png", res, order, col)
        fig_speed(out / "speed.png", conds, order, col)
        figs.update(agreement="agreement.png", metric_z="metric_z.png", speed="speed.png")
    if conds and fig_scans(out / "scans.png", conds, order, col):
        figs["scans"] = "scans.png"

    L = []
    fake = cfg.get("fake_real")
    L.append("# Navigation comparison — real vs simulated rover\n")
    if fake:
        L.append("> **TEST SESSION — NO REAL ROVER.** The `real` condition was played by Isaac "
                 "driving a deliberately perturbed copy of the calibrated physics parameters "
                 "(`conditions.yaml: fake_real`). These numbers only show that the pipeline "
                 "works; they say nothing about the real rover.\n")
    L.append(f"- session `{sd.name}` · created {cfg['created']} · git `{cfg.get('git', '')}`")
    L.append(f"- course **{course.name}** ({len(course.legs())} legs/trial: "
             f"{', '.join(l.name for l in course.legs())}) — {course.description}")
    L.append(f"- nav stack **{cfg['stack']}** (slam_toolbox + Nav2 TEB/rotation-shim"
             f"{' + OAK depth cloud' if cfg['stack'] == 'depth' else ''}; no visual SLAM, no RGB). "
             "Identical params on every condition — bundle sha256 in `session.json`.")
    L.append(f"- trials requested per condition: {cfg['trials']} · Isaac lidar profile "
             f"`{cfg.get('lidar_profile', 'native')}` (ld19 = 0.10–12 m; native = the USD's 1–200 m)\n")
    L.append("| condition | what it is | trials | leg success | time / trial [s] | recoveries / trial |")
    L.append("|---|---|---|---|---|---|")
    for n in order:
        e = res["conditions"].get(n, {})
        spec = cfg["conditions"][n]
        L.append(f"| {n} | {spec.get('note', spec['platform'])} | {e.get('trials', 0)} | "
                 f"{_fmt(100 * e.get('success_rate', math.nan), 0)} % | {_fmt(e.get('trial_time_s'), 1)} | "
                 f"{_fmt(e.get('recoveries_per_trial'), 1)} |")
    if cfg["stack"] == "depth":
        hz = {n: conds[n]["meta"].get("depth_hz_wall") for n in order}
        L.append("\nDepth cloud rate (wall clock): " + ", ".join(
            f"{n} {h:.1f} Hz" if isinstance(h, (int, float)) else f"{n} not recorded"
            for n, h in hz.items())
            + ". Very different rates mean the depth obstacle layer updates differently.")
    missing = [n for n in cfg["order"] if n not in order]
    if missing:
        L.append(f"\nNo data for: {', '.join(missing)}.")
    L.append("")
    if "error" in res:
        L.append(f"**{res['error']}**")
    else:
        floor = res["real_floor_rms_z"]
        rr = res["real_vs_real_frechet_cm"]
        L.append("## How close is each simulator to the real rover?\n")
        L.append("| condition | rms z [95 % CI] | path gap: Frechet vs real [cm] | path gap ratio | verdict |")
        L.append("|---|---|---|---|---|")
        L.append(f"| real vs real (floor) | {_fmt(floor)} | {_fmt(rr, 1)} | 1.00 | what a perfect sim scores |")
        sims = [n for n in order if n != res["real"]]
        n_real = res["conditions"][res["real"]]["trials"]
        enough = n_real >= 3
        best = min(sims, key=lambda n: res["conditions"][n].get("rms_z", math.inf), default=None)
        for n in sims:
            e = res["conditions"][n]
            lo, hi = e["rms_z_ci"]
            z = e["rms_z"]
            if not enough:
                verdict = f"withheld: {n_real} real trial(s), need ≥ 3"
            elif math.isfinite(floor) and math.isfinite(lo) and lo <= floor * 1.15:
                verdict = "indistinguishable from real repeats (CI reaches the floor)"
            elif math.isfinite(z) and z <= 2.0:
                verdict = "close (within ~2 real sd)"
            else:
                verdict = "clearly different"
            if enough and n == best and len(sims) > 1:
                verdict = "**closest** — " + verdict
            L.append(f"| {n} | {_fmt(z)} [{_fmt(lo)}–{_fmt(hi)}] | {_fmt(e['frechet_cm'], 1)} | "
                     f"{_fmt(e['path_gap_ratio'])} | {verdict} |")
        L.append("")
        if figs.get("agreement"):
            L.append("![agreement](agreement.png)\n")
        L.append("**Reading it.** *rms z* puts every behaviour metric of every sim trial in units "
                 "of the real rover's own trial-to-trial scatter for that leg "
                 "(z = (sim − real mean) / real sd), then takes the root-mean-square: 1 means a sim "
                 "trial looks like one more real repeat. The *floor* is the same number for each "
                 "real trial scored against the other real trials (needs ≥ 3 real trials). The "
                 "*path gap ratio* compares where the rovers drove: median Frechet distance "
                 "(the largest gap between two paths walked in step) sim-vs-real over real-vs-real. "
                 "The CI resamples whole trials. Few trials → wide CIs: run ≥ 5 per condition "
                 "before claiming a difference.\n")
        if figs.get("metric_z"):
            L.append("![metric z](metric_z.png)\n")
        L.append("## Paths\n")
    if figs.get("paths"):
        L.append("![paths](paths.png)\n")
    if figs.get("speed"):
        L.append("![speed](speed.png)\n")

    scans = {n: M.scan_summary(conds[n].get("start_scan")) for n in order}
    if any(scans.values()):
        L.append("## Sensor check — lidar at the start mark\n")
        L.append("Same pose, both rovers: if these differ a lot, the sim lidar is part of any "
                 "navigation gap (range limits, beam count, self-occlusion by the mast).\n")
        L.append("| condition | beams | resolution [deg] | valid share | range limits [m] | nearest return [m] |")
        L.append("|---|---|---|---|---|---|")
        for n, sc in scans.items():
            if sc:
                L.append(f"| {n} | {sc['beams']} | {_fmt(sc['resolution_deg'], 2)} | "
                         f"{_fmt(100 * sc['valid_share'], 0)} % | {_fmt(sc['range_min'])}–{_fmt(sc['range_max'], 1)} | "
                         f"{_fmt(sc['nearest_m'])} |")
        L.append("")
        if figs.get("scans"):
            L.append("![scans](scans.png)\n")
    if "table" in res:
        L.append("## Per-leg metrics (mean ± sd over trials)\n")
        names = order
        for leg, mets in res["table"].items():
            L.append(f"### {leg}\n")
            L.append("| metric | " + " | ".join(names) + " |")
            L.append("|---|" + "---|" * len(names))
            for m in ["success"] + M.SCORE_METRICS + M.DIAG_METRICS:
                cells = []
                for n in names:
                    mu, sdv, k = mets.get(m, {}).get(n, (math.nan, math.nan, 0))
                    nd = 0 if m in ("final_err_cm", "odom_drift_cm") else 2
                    cells.append("–" if not k or not math.isfinite(mu)
                                 else f"{_fmt(mu, nd)} ± {_fmt(sdv, nd)}" if math.isfinite(sdv)
                                 else _fmt(mu, nd))
                tag = "" if m in M.SCORE_METRICS or m == "success" else " *(diag)*"
                L.append(f"| {M.LABELS[m]}{tag} | " + " | ".join(cells) + " |")
            L.append("")
        L.append("Scored metrics feed rms z; *(diag)* metrics are diagnostics: `track_v`/`track_w` "
                 "are the drive response the optimizer fitted (achieved ÷ commanded), "
                 "`odom_drift` is wheel-odometry slip on the real rover but SLAM error in Isaac "
                 "(whose odometry is ground truth). Failed legs are kept in success/time but "
                 "their paths are left out of the path gap.\n")

    L.append("## Definitions\n")
    L.append("```\n" + M.__doc__.strip() + "\n```\n")
    L.append("## Files\n")
    L.append("- `leg_metrics.csv` — every leg of every trial, every condition")
    L.append("- `path_gaps.csv` — every real/sim path pair per leg")
    L.append("- `summary.json` — the numbers above")
    L.append("- `../<condition>/data/` — raw runner output (trace.csv 10 Hz, cmd.csv, legs.csv, "
             "map.json, meta.json); `../<condition>/*.log` — Isaac, stack and runner logs")
    (out / "report.md").write_text("\n".join(L) + "\n")

    summary = {k: v for k, v in res.items() if k not in ("rows", "paths", "table")}
    for e in summary.get("conditions", {}).values():
        e.pop("path_gaps", None)
    (out / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
    return out / "report.md"


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    print(build(sys.argv[1]))
