import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

VIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VIZ))
spec = importlib.util.spec_from_file_location('metric_report', VIZ/'make_lamaria_metric_init_report.py')
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


class MetricInitReportTests(unittest.TestCase):
    def test_rejected_proposal_keeps_uncomputed_conditioning_distinct(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.log'
            path.write_text('[METRIC-INIT] t=67.992200762 map=0 stage=0 accepted=0 scale=0.8767 sigma=inf imu_sigma=inf reason=stereo consistency rejected metric correction\n')
            result = report.parse_metric_records(path)
        self.assertEqual(result['parse_report']['rejected_proposals'], 1)
        row = result['records'][0]
        self.assertIsNone(row['sigma'])
        self.assertIn('not computed', row['conditioning_status'])
        self.assertEqual(result['parse_report']['completed_stages'], 0)

    def test_acceptance_does_not_imply_commit_and_malformed_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.log'
            path.write_text('[METRIC-INIT] t=70 map=0 stage=0 accepted=1 scale=1.01\n'
                            '[METRIC-INIT] t=70.1 commit=0 reason=source_map_changed\n'
                            '[METRIC-INIT] broken telemetry\n'
                            '[METRIC-INIT] t=71 map=0 stage=0 accepted=0 scale=nan reason=invalid scale\n')
            result = report.parse_metric_records(path)
        self.assertEqual(result['parse_report']['accepted_proposals'], 1)
        self.assertEqual(result['parse_report']['completed_stages'], 0)
        self.assertEqual(result['parse_report']['commit_records'], 1)
        self.assertEqual(len(result['parse_report']['malformed']), 1)
        self.assertEqual(result['parse_report']['rejected_proposals'], 1)

    def test_joint_diagnostics_are_distinct_from_scalar_proposal_and_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.log'
            path.write_text('[METRIC-INIT] t=70 map=0 stage=0 accepted=1 scale=.88 sigma=.02 imu_sigma=.03 reason=joint accepted\n'
                            '[JOINT-INIT] t=70 stage=0 refined=1 accepted=1 points=100 boundary_kfs=2 '
                            'before0=.4 after0=.5 before1=.6 after1=.4 baseline_sigma=.01 reason=joint accepted\n')
            result = report.parse_metric_records(path)
        self.assertEqual(result['parse_report']['accepted_proposals'], 1)
        self.assertEqual(result['parse_report']['joint_records'], 1)
        self.assertEqual(result['parse_report']['completed_stages'], 0)
        self.assertEqual(result['records'][1]['kind'], 'joint')
        self.assertEqual(result['records'][1]['baseline_sigma'], .01)

    def test_exact_timestamp_filter_and_both_gauge_partition(self):
        stamps = [10**11+i*10**9 for i in range(6)]
        def make_data():
            return {'online': {t: {'good': True, 'gauge': (0, 0, 0)} for t in stamps},
                    'final': {t: np.zeros(3) for t in stamps}}
        run, baseline = make_data(), make_data()
        baseline['online'][stamps[2]]['gauge'] = (0, 0, 1)
        baseline['online'][stamps[3]]['gauge'] = (0, 0, 1)
        run['online'][stamps[3]]['gauge'] = (0, 0, 2)
        run['online'][stamps[4]]['good'] = False
        baseline['final'][stamps[5]+1] = baseline['final'].pop(stamps[5])
        gt = {t: np.zeros(3) for t in stamps}
        selected, counts = report.select_common(run, baseline, gt)
        self.assertEqual(selected.tolist(), stamps[:4])
        self.assertEqual(counts['excluded_state_or_coasting'], 1)
        groups = report.comparison_groups(selected, run, baseline)
        self.assertEqual([group['indices'] for group in groups], [[0, 1], [2], [3]])

    def test_rigid_metric_check_does_not_hide_stretch(self):
        gt = np.array([[0., 0., 0.], [1., 0., 0.], [1., 2., 0.], [0., 2., 1.]])
        estimate = 2*gt + np.array([5., 3., -2.])
        summary = report.summary_fit(estimate, gt, np.arange(4)*10**9)
        self.assertEqual(summary['se3_scale_fixed_1']['scale_estimate_to_gt'], 1.)
        self.assertGreater(summary['se3_scale_fixed_1']['rmse_m'], .5)
        self.assertAlmostEqual(summary['diagnostic_sim3']['scale_estimate_to_gt'], .5)
        self.assertLess(summary['diagnostic_sim3']['rmse_m'], 1e-12)

    def test_equal_input_control_requires_complete_same_inputs_not_same_fit_times(self):
        reference = {'input_timestamps_sha256': 'same-sequence', 'input_frames': 2401,
                     'online_covers_exact_input': True, 'returncode': 0, 'raw_export_returncode': 0,
                     'feature_directories': ['cam0', 'cam1'], 'image_dataset': 'sequence',
                     'recorded_calibrated_imu_sha256': 'same-imu', 'library_sha256': 'baseline'}
        candidate = {**reference, 'library_sha256': 'candidate'}
        result = report.compare_input_controls(candidate, reference)
        self.assertTrue(result['equal_input_control'])
        self.assertFalse(result['same_library'])
        longer = {**reference, 'input_frames': 18000, 'input_timestamps_sha256': 'full-run'}
        result = report.compare_input_controls(candidate, longer)
        self.assertFalse(result['equal_input_control'])
        self.assertIn('future BA', result['interpretation'])
        for change in ({'recorded_calibrated_imu_sha256': None}, {'feature_directories': []},
                       {'online_covers_exact_input': False}, {'returncode': 1}):
            self.assertFalse(report.compare_input_controls({**candidate, **change}, reference)['equal_input_control'])


if __name__ == '__main__':
    unittest.main()
