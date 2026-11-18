"""Optimizer regressions with cheap objectives; no Isaac or physical rover required."""
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

import calibrate as cal

C = cal.C
RESTORED = {"corner_mu_dynamic", "corner_mu_static", "middle_mu_dynamic", "middle_mu_static",
            "drive_damping", "base_linear_damping", "base_angular_damping"}


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.theta = cal.start_theta(cal.load_measured(None), "concrete")

    def test_all_motion_parameters_have_one_search_stage(self):
        self.assertTrue(RESTORED <= set(C.OPTIMISABLE))
        self.assertEqual(set(C.OPTIMISABLE),
                         set(C.SCHEMA_BY_KEY) - set(C.LOCKED) - set(C.MEASURED))
        self.assertEqual(C.LOCKED, ["wheel_radius"])
        self.assertIn("wheel_distance", C.OPTIMISABLE)
        scheduled = [k for stage in cal.STAGES for k in stage["keys"]]
        self.assertCountEqual(scheduled, C.OPTIMISABLE)
        for stage in cal.STAGES:
            self.assertCountEqual(stage["keys"], C.stage_keys(stage["id"]))
        angular_stage = next(s for s in cal.STAGES if "base_angular_damping" in s["keys"])
        self.assertIn("spin_coast_angle", angular_stage["tests"]["T2_spin"])

    def test_reject_invalid_wheel_friction_and_floor_friction(self):
        for static, dynamic in C.FRICTION_PAIRS:
            for value in (-0.1, math.inf, math.nan, self.theta[static] + 0.1):
                with self.subTest(dynamic=dynamic, value=value):
                    invalid = dict(self.theta, **{dynamic: value})
                    with self.assertRaises(ValueError):
                        C.validate(invalid, "concrete")
            invalid = dict(self.theta, **{static: math.nan})
            with self.assertRaises(ValueError):
                C.validate(invalid, "concrete")

    def test_objective_rejects_invalid_pairs_before_running_simulation(self):
        terms = [("probe", [], {"probe": {"mean": 0., "sigma": 1.}}, 1.)]
        for static, dynamic in C.FRICTION_PAIRS:
            obj = cal.Objective(self.theta, [dynamic], terms, "isaac", "min", "concrete")
            obj.predict = Mock(return_value={"probe": 0.})
            self.assertEqual(obj([self.theta[static] + .1]), cal.INFEASIBLE_PENALTY)
            obj.predict.assert_not_called()

    def test_search_vertices_are_feasible_and_span_every_stage(self):
        for stage in cal.STAGES:
            keys = stage["keys"]
            simplex = cal.initial_simplex(keys, cal._x0(keys, self.theta, "concrete"),
                                          self.theta, "concrete")
            with self.subTest(stage=stage["id"]):
                self.assertEqual(np.linalg.matrix_rank(simplex[1:] - simplex[0]), len(keys))
                for vertex in simplex:
                    theta = dict(self.theta)
                    theta.update(cal._to_theta(keys, vertex))
                    C.validate(theta, "concrete")
                    for key in keys:
                        lo, hi = C.bounds(key, "concrete")
                        self.assertGreaterEqual(theta[key], lo * (1 - 1e-12))
                        self.assertLessEqual(theta[key], hi * (1 + 1e-12))

    def test_friction_search_crosses_minimum_combine_plateau(self):
        # A default 5% Nelder-Mead simplex sees the same contact friction everywhere:
        # all wheel coefficients exceed the floor. The restored search must escape it.
        for static, dynamic in C.FRICTION_PAIRS[:2]:
            keys = [dynamic, static]
            terms = [("contact", [], {"sliding": {"mean": .55, "sigma": .01},
                                       "static": {"mean": .60, "sigma": .01}}, 1.)]
            obj = cal.Objective(self.theta, keys, terms, "isaac", "min", "concrete")
            obj.predict = lambda theta, steps: {
                "sliding": min(theta[dynamic], theta["floor_mu_dynamic"]),
                "static": min(theta[static], theta["floor_mu_static"])}
            best, loss = cal.refine_isaac(obj, keys, cal._x0(keys, self.theta, "concrete"),
                                          max_evals=160, surface="concrete")
            with self.subTest(material=static):
                self.assertAlmostEqual(best[dynamic], .55, delta=.005)
                self.assertAlmostEqual(best[static], .60, delta=.005)
                self.assertLess(loss, .1)
                self.assertLessEqual(obj.n_eval, 160)

    def test_drive_gain_search_reaches_low_gain_regime(self):
        keys = ["drive_damping"]
        terms = [("drive", [], {"log_gain": {"mean": -4., "sigma": 1.}}, 1.)]
        obj = cal.Objective(self.theta, keys, terms, "isaac", "min", "concrete")
        obj.predict = lambda theta, steps: {"log_gain": math.log10(theta["drive_damping"])}
        best, loss = cal.refine_isaac(obj, keys, cal._x0(keys, self.theta, "concrete"),
                                      max_evals=80, surface="concrete")
        self.assertAlmostEqual(math.log10(best["drive_damping"]), -4., delta=.01)
        self.assertLess(loss, .001)

    def test_chassis_damping_search_moves_both_coefficients(self):
        keys = ["base_linear_damping", "base_angular_damping"]
        terms = [("damping", [], {"linear_decay": {"mean": 2., "sigma": 1.},
                                   "yaw_decay": {"mean": 3., "sigma": 1.}}, 1.)]
        obj = cal.Objective(self.theta, keys, terms, "isaac", "min", "concrete")
        obj.predict = lambda theta, steps: {"linear_decay": theta[keys[0]],
                                           "yaw_decay": theta[keys[1]]}
        best, loss = cal.refine_isaac(obj, keys, cal._x0(keys, self.theta, "concrete"),
                                      max_evals=120, surface="concrete")
        self.assertAlmostEqual(best[keys[0]], 2., delta=.01)
        self.assertAlmostEqual(best[keys[1]], 3., delta=.01)
        self.assertLess(loss, .001)

    def test_restored_parameters_require_isaac_instead_of_surrogate_artifacts(self):
        with patch.object(cal.fm, "simulate", side_effect=AssertionError("surrogate used")):
            self.assertEqual(cal.analytic_sensitive_keys(self.theta, sorted(RESTORED), [],
                                                         "min", "concrete"), [])

    def test_calibration_output_keeps_restored_values_and_excludes_locked_geometry(self):
        theta = dict(self.theta)
        theta.update(corner_mu_dynamic=.55, corner_mu_static=.60,
                     middle_mu_dynamic=.40, middle_mu_static=.50,
                     drive_damping=.0001, base_linear_damping=2., base_angular_damping=3.)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "surface.yaml"
            cal.write_surface(path, "concrete", theta, {}, "min")
            out = yaml.safe_load(path.read_text())
        flat = C.flatten(out["sim_calibration_yaml"])
        for key in RESTORED:
            self.assertEqual(out["calibrated"][key], theta[key])
            self.assertEqual(flat[key], theta[key])
            self.assertNotIn(key, out["fixed"])
        self.assertNotIn("wheel_radius", flat)
        self.assertEqual(set(out["measured"]), set(C.MEASURED))


if __name__ == "__main__":
    unittest.main()
