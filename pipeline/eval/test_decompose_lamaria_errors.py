"""Synthetic contracts for the read-only error decomposition (Stages 0-2).

Each test plants a known answer: a pure yaw, a pure tilt, a route rotated by a
fixed heading about a pivot, a uniformly scaled route, a 1 ms association.
"""
import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import decompose_lamaria_errors as dec  # noqa: E402


class SwingTwistTest(unittest.TestCase):
    def test_pure_yaw_is_all_twist(self):
        twist, swing = dec.twist_about_z(Rotation.from_euler('z', 10, degrees=True).as_matrix())
        self.assertAlmostEqual(twist[0], 10.0, places=9)
        self.assertAlmostEqual(swing[0], 0.0, places=9)

    def test_pure_tilt_is_all_swing(self):
        twist, swing = dec.twist_about_z(Rotation.from_euler('x', 10, degrees=True).as_matrix())
        self.assertAlmostEqual(twist[0], 0.0, places=9)
        self.assertAlmostEqual(swing[0], 10.0, places=9)

    def test_yaw_then_tilt_separates_exactly(self):
        # R = Rz(a) Rx(b): quaternion z/w projection returns a exactly, swing returns b.
        R = Rotation.from_euler('z', 25, degrees=True).as_matrix() @ Rotation.from_euler('x', 4, degrees=True).as_matrix()
        twist, swing = dec.twist_about_z(R)
        self.assertAlmostEqual(twist[0], 25.0, places=8)
        self.assertAlmostEqual(swing[0], 4.0, places=8)

    def test_negative_yaw_sign_and_wrap(self):
        twist, _ = dec.twist_about_z(Rotation.from_euler('z', -170, degrees=True).as_matrix())
        self.assertAlmostEqual(twist[0], -170.0, places=8)


class ChordalMeanTest(unittest.TestCase):
    def test_symmetric_yaws_average_to_identity(self):
        Rs = Rotation.from_euler('z', [[1], [-1]], degrees=True).as_matrix()
        np.testing.assert_allclose(dec.chordal_mean(Rs), np.eye(3), atol=1e-12)

    def test_identical_rotations_return_themselves(self):
        R = Rotation.from_euler('zyx', [100, 10, 3], degrees=True).as_matrix()
        np.testing.assert_allclose(dec.chordal_mean(np.stack([R, R, R])), R, atol=1e-12)


class HeadingIntegralTest(unittest.TestCase):
    def test_constant_heading_error_about_pivot_is_reproduced_exactly(self):
        n = 301
        t = np.arange(n, dtype=float)
        p_gt = np.c_[t, 0.3 * t]                     # straight GT path, 1.044 m/s
        pivot = 150
        psi = np.radians(1.0)
        Rz = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
        e_obs = (p_gt - p_gt[pivot]) @ (Rz - np.eye(2)).T   # aligned estimate = GT rotated by 1 deg about the pivot
        e_pred = dec.heading_integral_prediction(t, p_gt, np.full(n, psi), e_obs, pivot)
        np.testing.assert_allclose(e_pred, e_obs, atol=1e-9)
        # 1 deg over 150 m of path gives about 2.6 m at the ends, so the test is not vacuous.
        self.assertGreater(np.linalg.norm(e_obs[0]), 2.5)

    def test_time_varying_heading_integrates_interval_by_interval(self):
        n = 200
        t = np.arange(n, dtype=float)
        p_gt = np.c_[np.cumsum(np.cos(0.01 * t)), np.cumsum(np.sin(0.01 * t))]   # gently curving path
        psi = np.radians(np.linspace(-1, 2, n))
        pivot = 80
        d = np.diff(p_gt, axis=0)
        psi_mid = 0.5 * (psi[1:] + psi[:-1])
        inc = dec.rotate2d(psi_mid, d) - d
        e_obs = np.zeros((n, 2))
        for i in range(pivot + 1, n):
            e_obs[i] = e_obs[i - 1] + inc[i - 1]
        for i in range(pivot - 1, -1, -1):
            e_obs[i] = e_obs[i + 1] - inc[i]
        e_pred = dec.heading_integral_prediction(t, p_gt, psi, e_obs, pivot)
        np.testing.assert_allclose(e_pred, e_obs, atol=1e-9)

    def test_rotate2d_quarter_turn(self):
        np.testing.assert_allclose(dec.rotate2d(np.array([np.pi / 2]), np.array([[1., 0.]])), [[0., 1.]], atol=1e-12)


class WindowRatioTest(unittest.TestCase):
    def test_uniform_scale_and_heading_are_recovered(self):
        t = np.arange(0, 200, 0.2)
        p_gt = np.c_[1.2 * t, 0.5 * t, 0.1 * t]               # 1.3 m/s, 3-D
        Rz = Rotation.from_euler('z', 2.0, degrees=True).as_matrix()
        p_est = 0.9 * (p_gt @ Rz.T)                           # 10 % short and rotated +2 deg
        tw, ratio, heading = dec.windowed_displacement_ratio(t, p_est, p_gt, path_m=15.0, max_dt_s=30.0)
        self.assertGreater(len(ratio), 500)
        np.testing.assert_allclose(ratio, 0.9, atol=1e-9)
        np.testing.assert_allclose(heading, 2.0, atol=1e-9)
        self.assertTrue(np.all(np.diff(tw) > 0))

    def test_windows_respect_time_limit(self):
        t = np.r_[np.arange(0, 10, 1.0), np.arange(100, 110, 1.0)]   # a 90 s gap
        p = np.c_[0.1 * t, np.zeros_like(t), np.zeros_like(t)]       # 0.1 m/s: 15 m needs 150 s
        tw, ratio, _ = dec.windowed_displacement_ratio(t, p, p, path_m=15.0, max_dt_s=30.0)
        self.assertEqual(len(ratio), 0)


class Sim3AndMatchingTest(unittest.TestCase):
    def test_apply_sim3_matches_row_wise_definition(self):
        R = Rotation.from_euler('zyx', [105, 13, -1], degrees=True).as_matrix()
        s, t = 1.008, np.array([-270., -366., -29.])
        p = np.random.default_rng(0).normal(size=(5, 3))
        expected = np.array([s * R @ row + t for row in p])
        np.testing.assert_allclose(dec.apply_sim3(s, R, t, p), expected, atol=1e-12)

    def test_official_matcher_order_and_tolerance(self):
        sys.path.insert(0, str(dec.TOOLKIT))
        est = np.arange(0, 100, 1) * 50_000_000 + 1_000_000_000          # 20 Hz, longer than GT
        gt = np.array([est[3] + 1, est[10] - 999_999, est[20] + 2_000_000, est[-1]])
        est_ids, gt_ids, unmatched = dec.official_match(est, gt)
        self.assertEqual(est_ids.tolist(), [3, 10, 99])
        self.assertEqual(gt_ids.tolist(), [0, 1, 3])
        self.assertEqual(unmatched, [int(gt[2])])

    def test_rolling_median_window(self):
        t = np.arange(0, 10, 1.0)
        x = np.array([0, 0, 0, 100, 0, 0, 0, 0, 0, 0], dtype=float)
        med = dec.rolling_median(t, x, 5.0)
        self.assertEqual(med[3], 0.0)          # single spike removed by a 5-sample median
        self.assertEqual(dec.rolling_median(t, x, 0.5)[3], 100.0)   # 1-sample window keeps it


class BatchedSwingTwistTest(unittest.TestCase):
    def test_batch_of_rotations_is_handled_row_wise(self):
        Rs = np.stack([Rotation.from_euler('z', 10, degrees=True).as_matrix(),
                       Rotation.from_euler('x', 5, degrees=True).as_matrix(),
                       (Rotation.from_euler('z', -30, degrees=True) * Rotation.from_euler('y', 2, degrees=True)).as_matrix()])
        twist, swing = dec.twist_about_z(Rs)
        np.testing.assert_allclose(twist, [10.0, 0.0, -30.0], atol=1e-8)
        np.testing.assert_allclose(swing, [0.0, 5.0, 2.0], atol=1e-8)


if __name__ == '__main__':
    unittest.main()
