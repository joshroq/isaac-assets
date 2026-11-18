"""Distribution-level sim-vs-real scoring: many sim trials against many real trials.

The real rover does not repeat itself (arcs vary up to 2x between repeats), so matching
ONE sim run to the real MEAN says nothing about whether the sim behaves like the rover
across trials. Per feature, with the real repeats (mean m_r, sd s_r, n_r) and the sim
trials (m_s, s_s, n_s):

  mean term    z_mean   = (m_s - m_r) / s_pool,  s_pool = sqrt((s_r^2 + s_s^2) / 2)
               - how far apart the two distributions' centres are, in units of the
                 trial-to-trial scatter (a standardised mean difference)
  spread term  z_spread = ln(s_s / s_r) / sqrt(1/(2(n_r-1)) + 1/(2(n_s-1)))
               - is the sim as variable as the rover (log sd ratio over its sampling
                 error); only when both sides have >= 2 trials

Both sides' sds get the same floor as features.aggregate, so a deterministic sim (s_s = 0)
is scored as "far too consistent" rather than dividing by zero.
loss = sum z_mean^2 + spread_weight * sum z_spread^2.
"""
from __future__ import annotations

import math

import numpy as np

import features as F


def _floor(feature, mean):
    key = feature.split("_k")[0] + "_k" if "_k" in feature else feature
    return max(0.02 * abs(mean), F.ABS_SIGMA_FLOOR.get(key, 1e-3))


def summarize(trials):
    """[feature dict per trial] -> {feature: (mean, sd, n)} (sd ddof=1, 0 if n == 1)."""
    keys = set().union(*[set(d) for d in trials]) if trials else set()
    out = {}
    for k in keys:
        a = np.array([d[k] for d in trials if k in d and np.isfinite(d[k])], float)
        if len(a):
            out[k] = (float(a.mean()), float(a.std(ddof=1)) if len(a) > 1 else 0.0, int(len(a)))
    return out


def score(sim, targets, spread_weight=1.0):
    """sim: {test: [feature dict per sim trial]} (or a single dict for one trial);
    targets: {test: {"features": {feat: {"mean", "sigma", "n"}}}} (closed_loop format).
    -> dict(loss, n, rms_mean, rms_spread, parts_mean, parts_spread, missing)."""
    zm, zs, parts_m, parts_s, missing = [], [], {}, {}, []
    for test, tgt in targets.items():
        trials = sim.get(test) or []
        if isinstance(trials, dict):
            trials = [trials]
        S = summarize(trials)
        for f, t in tgt["features"].items():
            if f not in S:
                missing.append(f"{test}.{f}")
                continue
            m_s, s_s, n_s = S[f]
            m_r, s_r, n_r = t["mean"], t["sigma"], t.get("n", 2)
            fl = _floor(f, m_r)
            s_r, s_s_f = max(s_r, fl), max(s_s, fl)
            pool = math.sqrt((s_r ** 2 + s_s_f ** 2) / 2.0) if n_s > 1 else s_r
            z = (m_s - m_r) / pool
            zm.append(z)
            parts_m[f"{test}.{f}"] = round(float(z), 2)
            if n_s >= 2 and n_r >= 2:
                se = math.sqrt(1.0 / (2 * (n_r - 1)) + 1.0 / (2 * (n_s - 1)))
                zz = math.log(s_s_f / s_r) / se
                zs.append(zz)
                parts_s[f"{test}.{f}"] = round(float(zz), 2)
    zm, zs = np.asarray(zm), np.asarray(zs)
    loss = float(np.sum(zm ** 2) + spread_weight * np.sum(zs ** 2) + 1e3 * len(missing))
    n = len(zm) + len(zs)
    return dict(loss=loss, n=n, rms=math.sqrt(loss / max(n, 1)) if not missing else float("inf"),
                rms_mean=float(np.sqrt(np.mean(zm ** 2))) if len(zm) else float("nan"),
                rms_spread=float(np.sqrt(np.mean(zs ** 2))) if len(zs) else float("nan"),
                parts_mean=parts_m, parts_spread=parts_s, missing=missing)
