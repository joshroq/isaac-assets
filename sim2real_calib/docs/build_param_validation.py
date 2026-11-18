"""Figures + tables for PARAMETER_TEST_VALIDATION.md (no Isaac launch; reads saved results).

Inputs (param_validation/):
  isaac_sweep.json     every Isaac trial of the 2026-10-14 sweep: {"case|test": {theta, features}}
  wheel_fix.json       before/after-fix Isaac trials (same harness, defaults + friction probes)
  traces/<test>.csv    the default-parameter, post-fix Isaac trace of each test (t,x,y,yaw)
Outputs: param_validation/figures/*.png and sensitivity_isaac.csv / sensitivity_analytic.csv
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "sim2real_calib" / "optimize"))
sys.path.insert(0, str(REPO / "ugv_controller_v2" / "calibration"))
import calib_params as C            # noqa: E402
import features as F                # noqa: E402
import forward_model as fm          # noqa: E402

DATA = HERE / "param_validation"
OUT = DATA / "figures"
OUT.mkdir(parents=True, exist_ok=True)
SEQ_DIR = REPO / "sim2real_calib" / "parallel" / "sequences"

# reference palette (dataviz skill): categorical slots 1-2, text inks, diverging blue<->red
BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df"
DIVERGING = LinearSegmentedColormap.from_list("div", ["#184f95", "#6da7ec", "#f0efec", "#ec8a89", "#a8282a"])
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10.5, "text.color": INK,
                     "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})

NULL_KEYS = ("max_linear_speed", "max_angular_speed")
TESTS = {"T1_straight": ["v_ss", "dist", "tau_v", "coast_dist", "coast_time"],
         "T2_spin": ["omega_ss", "dyaw", "tau_omega", "turn_walk"],
         "T4_arc": ["arc_radius"]}
# The parameter -> test assignment and bounds that the 2026-10-14 sweep VALIDATED (calibrate.py's
# STAGES before the 2026-10-15 rework). Frozen here so the figures stay reproducible; the current
# schedule is calibrate.STAGES. Stage 3/4 ran on T3_sweep, whose features came from its FIRST
# 1.2 s straight (0.05 m/s); the same feature names are shown on T1's columns.
OLD_STAGES = [
    (2, "T1_straight", ["tau_v", "coast_dist", "coast_time"], {"drive_damping": (50.0, 20000.0), "drive_max_force": (0.3, 50.0)}),
    (3, "T3_sweep", ["v_ss", "omega_ss"], {"max_linear_speed": (0.2, 0.8), "max_angular_speed": (0.8, 3.0)}),
    (4, "T3_sweep", ["dist", "v_ss", "tau_v", "coast_dist"], {"floor_mu_dynamic": (0.3, 1.1), "floor_mu_static": (0.3, 1.3),
                                                            "corner_mu_dynamic": (0.5, 1.5), "corner_mu_static": (0.5, 1.8)}),
    (5, "T1_straight", ["v_ss", "coast_dist", "coast_time"], {"base_linear_damping": (0.0, 8.0), "base_angular_damping": (0.0, 8.0)}),
    (6, "T2_spin", ["dyaw", "omega_ss"], {"wheel_distance": (0.25, 0.9)}),
    (7, "T2_spin", ["turn_walk"], {"middle_mu_dynamic": (0.3, 1.2), "middle_mu_static": (0.3, 1.4)}),
]
SWEPT = ["wheel_distance", "max_linear_speed", "max_angular_speed", "corner_mu_dynamic", "corner_mu_static",
         "middle_mu_dynamic", "middle_mu_static", "floor_mu_dynamic", "floor_mu_static", "drive_damping",
         "drive_max_force", "base_linear_damping", "base_angular_damping"]
ASSIGNED, OLD_BOUNDS = {}, {}
for sid, test, feats, keys in OLD_STAGES:
    for k, b in keys.items():
        ASSIGNED[k] = (sid, test, feats)
        OLD_BOUNDS[k] = b


def seq(name):
    import yaml
    return yaml.safe_load((SEQ_DIR / f"{name}.yaml").read_text())["steps"]


def save(fig, name):
    fig.savefig(OUT / f"{name}.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


# ── sensitivity matrices ──────────────────────────────────────────────────────
def isaac_matrix(sweep):
    cols = [(t, f) for t, fs in TESTS.items() for f in fs]
    reps = {c: [] for c in cols}
    for v in sweep.values():
        # null runs: the 3 default repeats + both speed caps at both bounds. Every T1/T2/T4
        # command is below the caps' LOWER bounds (0.2 m/s, 0.8 rad/s), so those runs differ
        # from the defaults only by run-to-run noise (ROS timing, stick-slip).
        if v["case"].startswith("default_r") or v["case"].split("=")[0] in NULL_KEYS:
            for f in TESTS.get(v["test"], []):
                if f in v["features"]:
                    reps[(v["test"], f)].append(v["features"][f])
    mean = {c: float(np.mean(r)) for c, r in reps.items() if r}
    sigma = {c: max(0.02 * abs(mean[c]), 1e-3, float(np.std(reps[c], ddof=1)) if len(reps[c]) > 1 else 0)
             for c in mean}
    n_null = {c: len(r) for c, r in reps.items()}
    M = np.full((len(SWEPT), len(cols)), np.nan)
    for i, k in enumerate(SWEPT):
        for j, (t, f) in enumerate(cols):
            lo, hi = sweep.get(f"{k}=lo|{t}"), sweep.get(f"{k}=hi|{t}")
            if lo and hi and f in lo["features"] and f in hi["features"] and (t, f) in sigma:
                M[i, j] = (hi["features"][f] - lo["features"][f]) / sigma[(t, f)]
    return cols, M, mean, sigma, n_null


def analytic_matrix(cols):
    base = C.defaults()
    M = np.full((len(SWEPT), len(cols)), np.nan)
    cache = {}

    def feats(theta, t):
        tr = fm.simulate(theta, seq(t), combine_mode="min")
        return F.extract(np.array(tr["t"]), np.array(tr["x"]), np.array(tr["y"]), np.array(tr["yaw"]), seq(t))
    for t in TESTS:
        cache[t] = feats(base, t)
    for i, k in enumerate(SWEPT):
        for t in TESTS:
            flo, fhi = feats({**base, k: OLD_BOUNDS[k][0]}, t), feats({**base, k: OLD_BOUNDS[k][1]}, t)
            for j, (tt, f) in enumerate(cols):
                if tt == t and f in flo and f in fhi and f in cache[t] and np.isfinite(fhi[f] - flo[f]):
                    M[i, j] = (fhi[f] - flo[f]) / max(0.02 * abs(cache[t][f]), 1e-3)
    return M


def heatmap(cols, M, title, name, note):
    fig, ax = plt.subplots(figsize=(13.5, 7.4))
    ax.grid(False)
    shown = np.clip(M, -10, 10)
    im = ax.imshow(np.ma.masked_invalid(shown), cmap=DIVERGING, vmin=-10, vmax=10, aspect="auto")
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            v = M[i, j]
            txt = "n/a" if not np.isfinite(v) else ("0" if abs(v) < 0.5 else f"{v:+.0f}" if abs(v) < 100 else f"{v:+.0e}")
            ax.text(j, i, txt, ha="center", va="center", fontsize=8.5,
                    color="white" if np.isfinite(v) and abs(v) > 6.5 else INK2)
    for i, k in enumerate(SWEPT):
        sid, test, feats = ASSIGNED[k]
        for j, (t, f) in enumerate(cols):
            on_test = t == test or (test == "T3_sweep" and t == "T1_straight")
            if on_test and f in feats:
                ax.add_patch(Rectangle((j - .48, i - .46), .96, .92, fill=False, lw=2.2, ec=INK,
                                       ls="-" if t == test else (0, (3, 2))))
    ax.set_xticks(range(len(cols)), [f"{f}\n{t.split('_')[0]}" for t, f in cols], fontsize=9)
    ax.set_yticks(range(len(SWEPT)),
                  [f"{k}  (stage {ASSIGNED[k][0]} → {ASSIGNED[k][1]})" for k in SWEPT], fontsize=9)
    for x in (4.5, 8.5):
        ax.axvline(x, color=INK2, lw=1.2)
    ax.set_title(title, loc="left", weight="bold")
    cb = fig.colorbar(im, ax=ax, fraction=0.025, pad=0.01)
    cb.set_label("feature change lo→hi bound, in σ (clipped ±10)")
    import textwrap
    fig.text(0.01, -0.06, textwrap.fill(note, 150), fontsize=9, color=INK2, va="top")
    save(fig, name)


def write_matrix_csv(cols, M, path):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["parameter", "stage", "assigned_test", "assigned_features"] + [f"{t}:{f}" for t, f in cols])
        for i, k in enumerate(SWEPT):
            sid, test, feats = ASSIGNED[k]
            w.writerow([k, sid, test, " ".join(feats)] + [("" if not np.isfinite(v) else f"{v:.2f}") for v in M[i]])


# ── per-test kinematic diagrams (real post-fix Isaac traces) ──────────────────
def load_trace(test):
    t, x, y, yaw = F.load_csv(DATA / "traces" / f"{test}.csv")
    return t, x - x[0], y - y[0], np.unwrap(yaw) - yaw[0]


def ideal_path(steps, dt=0.01):
    x = y = psi = 0.0
    X, Y = [0.0], [0.0]
    for s in steps:
        for _ in range(int(round(s["duration"] / dt))):
            psi += s["angular"] * dt
            x += s["linear"] * math.cos(psi) * dt
            y += s["linear"] * math.sin(psi) * dt
            X.append(x); Y.append(y)
    return np.array(X), np.array(Y)


def cmd_profile(steps, key):
    T, V, t = [0.0], [steps[0][key]], 0.0
    for s in steps:
        T += [t, t + s["duration"]]; V += [s[key], s[key]]; t += s["duration"]
    return np.array(T[1:]), np.array(V[1:])


def to_body(t, x, y, psi):
    v, w, _ = F._speed_yawrate(t, x, y, psi)
    vx = np.gradient(x) / np.maximum(np.gradient(t), 1e-6)
    vy = np.gradient(y) / np.maximum(np.gradient(t), 1e-6)
    v_fwd = vx * np.cos(psi) + vy * np.sin(psi)          # signed forward speed
    return v_fwd, w


TEST_TEXT = {
    "T1_straight": ("T1 straight: 0.15 m/s for 4 s, then a commanded stop",
                    "scored for: stage 2 drive_damping, drive_max_force (tau_v, coast_*);\n"
                    "stage 5 base_linear_damping, base_angular_damping (v_ss, coast_*)"),
    "T2_spin": ("T2 spin: 0.6 rad/s in place for 8 s, then a commanded stop",
                "scored for: stage 6 wheel_distance (omega_ss, dyaw);\nstage 7 middle_mu_dynamic, middle_mu_static (turn_walk)"),
    "T3_speed_sweep": ("T3 speed sweep: 0.05 → 0.10 → 0.15 → 0.20 m/s, 1.2 s each",
                       "scored for: stage 3 max_linear/angular_speed (v_ss, omega_ss);\n"
                       "stage 4 corner/floor mu (dist, v_ss, tau_v, coast_dist) — features read the 0.05 step"),
    "T4_arc": ("T4 arc: 0.12 m/s + 0.4 rad/s for 6 s (commanded radius 0.30 m)",
               "scored for: nothing (no stage uses T4)"),
    "T5_stop": ("T5 stop: 0.20 m/s for 3 s, then 4 s commanded stop",
                "scored for: nothing (no stage uses T5)"),
}


def test_figure(test):
    steps = seq(test)
    t, x, y, psi = load_trace(test)
    feats = F.extract(*F.load_csv(DATA / "traces" / f"{test}.csv"), steps)
    v, w = to_body(t, x, y, psi)
    segs = F._segments(steps)
    fig = plt.figure(figsize=(13.5, 5.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.55], hspace=0.12, wspace=0.18)
    ax_p = fig.add_subplot(gs[:, 0]); ax_v = fig.add_subplot(gs[0, 1]); ax_w = fig.add_subplot(gs[1, 1], sharex=ax_v)
    ix, iy = ideal_path(steps)
    ax_p.plot(ix, iy, color=MUTED, lw=2, ls=(0, (4, 3)), label="ideal (commanded) path")
    ax_p.plot(x, y, color=BLUE, lw=2, label="Isaac base pose")
    span = max(np.ptp(x), np.ptp(y), np.ptp(ix), np.ptp(iy), 0.02)
    for k in range(0, len(t), max(1, len(t) // 14)):
        ax_p.annotate("", (x[k] + 0.07 * span * math.cos(psi[k]), y[k] + 0.07 * span * math.sin(psi[k])), (x[k], y[k]),
                      arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1))
    ax_p.set_aspect("equal", adjustable="datalim"); ax_p.set_xlabel("x [m] (start heading)"); ax_p.set_ylabel("y [m]")
    ax_p.legend(loc="best", fontsize=8.5, frameon=False)
    ax_p.set_title("top view (arrows = heading)", fontsize=10, color=INK2)
    for ax, key, sig, lab in ((ax_v, "linear", v, "forward speed v [m/s]"), (ax_w, "angular", w, "yaw rate Ω [rad/s]")):
        T, V = cmd_profile(steps, key)
        ax.plot(T, V, color=MUTED, lw=2, ls=(0, (4, 3)), label="command")
        ax.plot(t, sig, color=BLUE if key == "linear" else ORANGE, lw=2, label="Isaac")
        ax.set_ylabel(lab)
        for typ, a, b, _ in segs:
            if typ != "idle":
                ax.axvspan(a, b, color=GRID, alpha=0.35, lw=0)
    ax_w.set_xlabel("time since sequence start [s]")
    plt.setp(ax_v.get_xticklabels(), visible=False)
    ax_v.legend(loc="upper right", fontsize=8.5, frameon=False)
    # feature annotations
    straights = [(a, b) for typ, a, b, _ in segs if typ == "straight"]
    spins = [(a, b) for typ, a, b, _ in segs if typ == "spin"]
    if straights and "v_ss" in feats:
        a, b = max(straights, key=lambda ab: ab[1] - ab[0])
        ax_v.axvspan(b - F.SS_TAIL * (b - a), b, color=BLUE, alpha=0.12, lw=0)
        ax_v.annotate(f"v_ss = {feats['v_ss']:.3f}\n(median, last 40 %)", (b - 0.2 * (b - a), feats["v_ss"]),
                      (b - 0.2 * (b - a), feats["v_ss"] * 0.45), ha="center", fontsize=8.5,
                      arrowprops=dict(arrowstyle="-", color=INK2))
        ax_v.axvline(a + feats["tau_v"], color=INK2, lw=1, ls=":")
        ax_v.text(a + feats["tau_v"] + 0.05, feats["v_ss"] * 0.2, f"tau_v = {feats['tau_v']:.2f} s", fontsize=8.5, color=INK2)
    if "coast_time" in feats:
        i = next(k for k, s in enumerate(segs) if s[0] == "idle" and k > 0 and segs[k - 1][0] in ("straight", "arc"))
        a = segs[i][1]
        ax_v.axvspan(a, a + max(feats["coast_time"], 0.05), color=ORANGE, alpha=0.25, lw=0)
        ax_v.text(a + 0.15, ax_v.get_ylim()[1] * 0.55,
                  f"coast: {feats['coast_dist']*1000:.0f} mm in {feats['coast_time']:.2f} s", fontsize=8.5, color=INK2)
    if spins and "omega_ss" in feats:
        a, b = max(spins, key=lambda ab: ab[1] - ab[0])
        ax_w.axvspan(b - F.SS_TAIL * (b - a), b, color=ORANGE, alpha=0.12, lw=0)
        ax_w.text(0.01, 0.97,
                  f"omega_ss = {feats['omega_ss']:.3f} rad/s (median, shaded tail)\n"
                  f"mean rate dyaw/T = {feats['dyaw'] / (b - a):.3f} rad/s, dyaw = {feats['dyaw']:.2f} rad\n"
                  f"turn_walk = {feats['turn_walk']*1000:.0f} mm",
                  transform=ax_w.transAxes, va="top", fontsize=8.5, color=INK2,
                  bbox=dict(boxstyle="round,pad=0.3", fc="#fcfcfb", ec=GRID))
    if "arc_radius" in feats:
        ax_p.text(0.02, 0.02, f"arc_radius = median v / median |Ω| = {feats['arc_radius']:.2f} m (commanded 0.30 m)",
                  transform=ax_p.transAxes, fontsize=8.5, color=INK2)
    title, scored = TEST_TEXT[test]
    fig.suptitle(title, x=0.01, ha="left", weight="bold", y=1.0)
    fig.text(0.01, 0.935, scored, fontsize=9, color=INK2, va="top")
    fig.subplots_adjust(top=0.84)
    save(fig, f"test_{test}")
    return feats


# ── wheel-fix before/after (small multiples, one metric each) ────────────────
def wheel_fix_figure(fix):
    cases = [("T2", "defaults"), ("T2_cornerlow", "corner_mu → 0.40/0.45"), ("T2_middlelow", "middle_mu → 0.30/0.35")]
    metrics = [("omega_ss", "steady yaw rate [rad/s]\n(command 0.6)"), ("turn_walk", "centre drift during spin [m]\n(ideal 0)")]
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.4))
    xs = np.arange(len(cases))
    for ax, (m, lab) in zip(axes, metrics):
        for off, label, col in ((-0.19, "before fix", ORANGE), (0.19, "after fix", BLUE)):
            vals = [fix[label.split()[0]].get(c, {}).get(m, np.nan) for c, _ in cases]
            bars = ax.bar(xs + off, vals, width=0.36, color=col, label=label, edgecolor="#fcfcfb", lw=2)
            for b, v in zip(bars, vals):
                if np.isfinite(v):
                    ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.3f}", ha="center", va="bottom", fontsize=8.5, color=INK2)
        ax.set_xticks(xs, [c[1] for c in cases]); ax.set_ylabel(lab); ax.grid(axis="x", visible=False)
    axes[0].legend(frameon=False, loc="upper right")
    fig.suptitle("T2 spin in Isaac, same harness: what the wheel-material fix changes", x=0.01, ha="left", weight="bold")
    fig.tight_layout()
    save(fig, "wheel_fix_before_after")


def main():
    sweep = json.loads((DATA / "isaac_sweep.json").read_text())
    fix = json.loads((DATA / "wheel_fix.json").read_text())
    cols, Mi, mean, sigma, n_null = isaac_matrix(sweep)
    Ma = analytic_matrix(cols)
    write_matrix_csv(cols, Mi, DATA / "sensitivity_isaac.csv")
    write_matrix_csv(cols, Ma, DATA / "sensitivity_analytic.csv")
    heatmap(cols, Mi, "Isaac (post-fix): how far each scored feature moves when one parameter goes lo → hi",
            "sensitivity_isaac",
            "σ = max(2 % of the default value, 1e-3, std of 7 null runs: 3 default repeats + both speed caps at both bounds, which no T1/T2/T4 command reaches). Black boxes = the features "
            "calibrate.py scores for that parameter (dashed = scored on T3_sweep; shown on T1's same-named feature). "
            "A boxed '0' means the stage cannot identify the parameter.")
    heatmap(cols, Ma, "Analytic warm-start model (forward_model.py): same sweep",
            "sensitivity_analytic",
            "σ = max(2 % of the default value, 1e-3). This is the model calibrate.py uses for every stage's warm start "
            "(and the whole fit with --backend analytic).")
    for t in ("T1_straight", "T2_spin", "T3_speed_sweep", "T4_arc", "T5_stop"):
        test_figure(t)
    wheel_fix_figure(fix)
    print(json.dumps({"isaac_default_mean": {f"{t}:{f}": round(v, 4) for (t, f), v in mean.items()},
                      "isaac_sigma": {f"{t}:{f}": round(v, 4) for (t, f), v in sigma.items()},
                      "n_null": {f"{t}:{f}": n for (t, f), n in n_null.items()}}, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
