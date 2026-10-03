import csv
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from baby_feature_overlays import BabyFeatureOverlay, _mutual_matches, _ordered_cache_indices


class BabyOverlayTests(unittest.TestCase):
    def test_ordered_filter_and_unique_reordering(self):
        cached = np.array([[10.125, 20], [30, 40], [50, 60]], dtype=np.float32)
        indices, method = _ordered_cache_indices(cached, [[10.12, 20], [50, 60]])
        self.assertEqual(indices.tolist(), [0, 2])
        self.assertEqual(method, 'filtered_subsequence')
        indices, method = _ordered_cache_indices(cached, [[50, 60], [30, 40]])
        self.assertEqual(indices.tolist(), [2, 1])
        self.assertEqual(method, 'unique_reordered')

    def test_rounded_duplicate_cannot_invent_descriptor_identity(self):
        cached = np.array([[1.001, 2], [1.002, 2], [3, 4]])
        with self.assertRaisesRegex(ValueError, 'ambiguous_rounded_duplicate'):
            _ordered_cache_indices(cached, [[1, 2], [3, 4]])
        indices, _ = _ordered_cache_indices(cached, [[1, 2], [1, 2], [3, 4]])
        self.assertEqual(indices.tolist(), [0, 1, 2])

    def test_native_matcher_requires_mutual_distinct_appearance(self):
        descriptors = np.array([[0]*32, [255]*32, [85]*32], dtype=np.uint8)
        self.assertEqual(_mutual_matches(descriptors, descriptors), [(0, 0), (1, 1), (2, 2)])
        duplicate = np.array([[0]*32, [0]*32, [255]*32], dtype=np.uint8)
        # Identical closest and second-closest rows fail the native ratio gate.
        self.assertEqual(_mutual_matches(descriptors, duplicate), [(1, 2)])

    def test_saved_sos_state_and_cache_reset_are_authoritative(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/'config').mkdir()
            (run/'config/settings.yaml').write_text('%YAML:1.0\nCamera.width: 640\n')
            caches = [run/'cam0', run/'cam1']
            for cache in caches:
                cache.mkdir()
            (run/'command.json').write_text(json.dumps({'command': [
                f'KP_DIR={caches[0]}', f'KP_DIR1={caches[1]}']}))
            stamps = [1_000_000_000+i*50_000_000 for i in range(5)]
            descriptors = np.array([[0]*32, [255]*32, [85]*32], dtype=np.uint8)
            pixels = np.array([[10, 10], [20, 20], [30, 30]], dtype='<f4')
            payload = (struct.pack('<i', 3)+pixels.tobytes()+
                       np.ones(3, dtype='<f4').tobytes()+descriptors.tobytes())
            for cache in caches:
                for stamp in stamps:
                    (cache/f'{stamp}.kp').write_bytes(payload)
            with (run/'online_fixture.csv').open('w') as handle:
                writer = csv.writer(handle)
                writer.writerow(['input_t_s', 'state', 'pose_available'])
                for stamp, state, pose in zip(stamps, [1, 2, 3, 3, 2], [0, 1, 1, 1, 1]):
                    writer.writerow([stamp/1e9, state, pose])
            events = [
                {'time': stamps[1]/1e9, 'event': 'warm', 'tracks': 0},
                {'time': stamps[2]/1e9, 'event': 'enter', 'tracks': 6},
                {'time': stamps[2]/1e9, 'event': 'accepted', 'tracks': 6, 'camera_inliers': [15, 20]},
                {'time': stamps[3]/1e9, 'event': 'reset', 'reason': 'map_update', 'tracks': 6},
                {'time': stamps[3]/1e9, 'event': 'rejected', 'reason': 'invalid_window', 'tracks': 0},
                {'time': stamps[4]/1e9, 'event': 'recovered', 'reason': 'retained_window_refined', 'tracks': 6},
            ]
            (run/'run.log').write_text(''.join('[BABY_DIAG] '+json.dumps(event)+'\n' for event in events))
            overlay = BabyFeatureOverlay(run)
            rows = [[(-1, float(x), float(y), 0) for x, y in pixels] for _ in range(2)]
            outputs = [overlay.advance(stamp, rows) for stamp in stamps]
            self.assertEqual([output['mode'] for output in outputs],
                             ['normal', 'normal', 'baby', 'imu_only', 'normal'])
            self.assertEqual(outputs[2]['accepted_counts'], [15, 20])
            self.assertEqual(outputs[2]['cameras'][0]['ages'].tolist(), [2, 2, 2])
            self.assertEqual(outputs[3]['cameras'][0]['ages'].tolist(), [1, 1, 1])
            self.assertTrue(outputs[3]['active'])
            self.assertFalse(outputs[4]['active'])
            summary = overlay.summary()
            self.assertEqual(summary['counts'].get('logged_pair_count_mismatches', 0), 0)
            self.assertFalse(summary['native_accepted_point_identities_inferred'])
            json.dumps(summary)


if __name__ == '__main__':
    unittest.main()
