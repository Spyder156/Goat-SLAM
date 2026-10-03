"""Scoring safeguards: missing timestamps and independently initialized maps."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

from score_lamaria import associate_dense, input_coverage, load_raw_rows, seconds_to_ns


class ScoreInputTest(unittest.TestCase):
    def test_integer_ns_conversion_does_not_round_through_float(self):
        self.assertEqual(seconds_to_ns('1700000000.000000123'), 1700000000000000123)

    def test_boundary_frames_remain_in_source_coverage_denominator(self):
        source = [1000000000 + i * 50000000 for i in range(6)]
        processed = source[1:-1]
        scored = [stamp - 1 for stamp in processed[1:]]
        result = input_coverage(processed, source, scored)
        self.assertEqual(result['native_input_frames'], 6)
        self.assertEqual(result['processed_input_frames'], 4)
        self.assertEqual(result['unsupported_boundary_frames'], 2)
        self.assertEqual(result['unsupported_boundary_prefix_frames'], 1)
        self.assertEqual(result['unsupported_boundary_suffix_frames'], 1)
        self.assertEqual(result['missing_native_input_poses'], 3)
        self.assertEqual(result['missing_processed_input_poses'], 1)
        self.assertEqual(result['pose_coverage_fraction'], .5)
        self.assertEqual(result['processed_pose_coverage_fraction'], .75)
        self.assertEqual(result['pose_to_input_max_difference_ns'], 1)
        with self.assertRaisesRegex(ValueError, 'interior'):
            input_coverage([source[1], source[3]], source, [source[1]])
        with self.assertRaisesRegex(ValueError, 'subset'):
            input_coverage([source[-1] + 50000000], source, [])
        with self.assertRaisesRegex(ValueError, 'does not correspond'):
            input_coverage(processed, source, [source[0]])

    def test_independent_maps_are_never_concatenated(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'atlas.csv'
            fields = ['t_s', 'map_id', 'map_init_kf_id', 'coordinate_frame', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
            with path.open('w') as handle:
                writer = csv.writer(handle)
                writer.writerow(fields)
                writer.writerow(['1.0', 0, 0, 'map_0_init_0_body0', 0, 0, 0, 0, 0, 0, 1])
                writer.writerow(['2.0', 0, 10, 'map_0_init_10_body0', 0, 0, 0, 0, 0, 0, 1])
                writer.writerow(['2.1', 0, 10, 'map_0_init_10_body0', 1, 0, 0, 0, 0, 0, 1])
            with self.assertRaisesRegex(ValueError, 'independent map'):
                load_raw_rows(path)
            rows, stamps, scope = load_raw_rows(path, 'largest')
            self.assertEqual(stamps, [2000000000, 2100000000])
            self.assertEqual(scope['atlas_map_epochs'], 2)
            self.assertEqual(scope['omitted_other_map_poses'], 1)
            self.assertEqual(len(rows), 2)
            self.assertFalse(scope['uses_ground_truth_for_selection'])

    def test_shorter_estimate_preserves_real_errors_and_full_gt_denominator(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'third_party/lamaria_toolkit'))
        import pycolmap
        from lamaria.structs.trajectory import Trajectory
        from lamaria.utils.timestamps import matching_time_indices
        from lamaria.eval.pgt_evaluation import evaluate_wrt_pgt
        from lamaria.utils.metrics import calculate_pose_recall

        def trajectory(stamps, x):
            return Trajectory(invert_poses=False, corresponding_sensor='cam0', _timestamps=list(stamps),
                              _poses=[pycolmap.Rigid3d(pycolmap.Rotation3d(), np.array([x, 0., 0.])) for _ in stamps])

        est = trajectory([2000000000, 3000000000], 10.)
        gt = trajectory([1000000000, 2000000000, 3000000000], 0.)
        denominator, missing = associate_dense(est, gt, matching_time_indices)
        errors = evaluate_wrt_pgt(est, gt, pycolmap.Sim3d())
        np.testing.assert_allclose(errors, [10., 10.])
        self.assertEqual(denominator, 3)
        self.assertEqual(missing, [1000000000])
        self.assertEqual(calculate_pose_recall(errors, denominator, 5.), 0.)
        self.assertAlmostEqual(calculate_pose_recall(errors, denominator, 10.), 200/3)


if __name__ == '__main__':
    unittest.main()
