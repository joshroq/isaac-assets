"""Signed per-step features (v_k<i>, w_k<i>) and the command->response map.

    python3 -m unittest discover -s sim2real_calib/optimize -p test_response_map.py -v
"""
import glob
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import features as F  # noqa: E402
import response_map as RM  # noqa: E402

SEQ_DIR = HERE.parent / "parallel" / "sequences"


def track(steps, gain_v=1.0, gain_w=1.0, dt=0.02, yaw0=1.2):
    """Ideal unicycle response to the commands, scaled by fixed gains."""
    t, x, y, a = 0.0, 0.3, 0.4, yaw0
    rows = [(t, x, y, a)]
    for s in steps:
        for _ in range(int(round(s["duration"] / dt))):
            v, w = gain_v * s["linear"], gain_w * s["angular"]
            x += v * math.cos(a) * dt
            y += v * math.sin(a) * dt
            a += w * dt
            t += dt
            rows.append((t, x, y, math.atan2(math.sin(a), math.cos(a))))
    return np.array(rows)


def write(rows, path):
    np.savetxt(path, rows, delimiter=",", header="t,x,y,yaw", comments="")
    return path


class SignedStepFeatures(unittest.TestCase):
    def test_backward_and_right_turns_keep_their_sign(self):
        steps = [dict(linear=0.0, angular=0.0, duration=1.0),
                 dict(linear=0.12, angular=0.3, duration=2.5),
                 dict(linear=0.0, angular=0.0, duration=0.8),
                 dict(linear=-0.12, angular=-0.3, duration=2.5),
                 dict(linear=0.0, angular=-0.8, duration=2.0)]
        r = track(steps, gain_v=0.8, gain_w=0.5)
        f = F.extract(r[:, 0], r[:, 1], r[:, 2], r[:, 3], steps)
        self.assertAlmostEqual(f["v_k1"], 0.096, places=3)
        self.assertAlmostEqual(f["w_k1"], 0.15, places=3)
        self.assertAlmostEqual(f["v_k3"], -0.096, places=3)
        self.assertAlmostEqual(f["w_k3"], -0.15, places=3)
        self.assertAlmostEqual(f["w_k4"], -0.40, places=3)
        self.assertAlmostEqual(f["v_k4"], 0.0, places=3)
        self.assertNotIn("v_k0", f)                    # idle steps produce no terms

    def test_signed_features_get_absolute_sigma_floors(self):
        agg = F.aggregate([{"w_k2": 0.0001}, {"w_k2": 0.0002}, {"v_k2": 0.0}])
        self.assertGreaterEqual(agg["w_k2"]["sigma"], F.ABS_SIGMA_FLOOR["w_k"])


class ResponseMap(unittest.TestCase):
    def test_map_pairs_real_and_sim_and_flags_the_arc_gap(self):
        steps = F.load_sequence(str(SEQ_DIR / "R3_arcs_slow.yaml"))
        with tempfile.TemporaryDirectory() as d:
            real = [write(track(steps, 0.8, g), Path(d) / f"real{i}.csv")
                    for i, g in enumerate((0.45, 0.5, 0.55))]
            sim = [write(track(steps, 0.8, 0.05), Path(d) / "sim.csv")]   # sim barely turns
            rows = RM.merge(RM.summarise(RM.step_responses(steps, real)),
                            RM.summarise(RM.step_responses(steps, sim)))
            self.assertEqual(len(rows), 12)              # 6 arcs out + 6 retraced back
            first = rows[0]
            self.assertAlmostEqual(first["real"]["w_mean"], 0.5 * first["w_cmd"], places=3)
            g = RM.gap(rows)
            self.assertGreater(g["rms_z"], 3.0)
            self.assertGreaterEqual(g["outside_real_range"], 12)   # every yaw term
            same = RM.gap(RM.merge(RM.summarise(RM.step_responses(steps, real)),
                                   RM.summarise(RM.step_responses(steps, real))))
            self.assertLess(same["rms_z"], 0.5)
            self.assertEqual(same["outside_real_range"], 0)
            RM.plot(rows, str(Path(d) / "map.png"), "test")
            self.assertTrue((Path(d) / "map.png").stat().st_size > 1000)

    def test_battery_sequences_cover_the_nav2_envelope(self):
        cmds = [(s["linear"], s["angular"])
                for f in sorted(glob.glob(str(SEQ_DIR / "R*.yaml")))
                for s in F.load_sequence(f) if s["linear"] or s["angular"]]
        v = np.array([c[0] for c in cmds])
        w = np.array([c[1] for c in cmds])
        self.assertTrue((np.abs(v) <= 0.26 + 1e-9).all() and (np.abs(w) <= 1.0 + 1e-9).all())
        self.assertTrue(((v > 0) & (w == 0)).any() and ((v < 0) & (w == 0)).any())   # straight f/b
        self.assertTrue(((v == 0) & (w > 0)).any() and ((v == 0) & (w < 0)).any())   # spins l/r
        for sv in (1, -1):                                                         # arcs, all 4
            for sw in (1, -1):
                self.assertTrue(((np.sign(v) == sv) & (np.sign(w) == sw)).any())
        radii = np.abs(v[(v != 0) & (w != 0)] / w[(v != 0) & (w != 0)])
        self.assertLess(radii.min(), 0.3)
        self.assertGreater(radii.max(), 2.0)


if __name__ == "__main__":
    unittest.main()
