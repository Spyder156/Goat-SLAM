"""Reference-history transport must preserve geometry without filling poses."""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from export_lamaria_native_refinement import POSE_KEYS, transport_history


def pose(position, rotvec):
    return np.r_[position, Rotation.from_rotvec(rotvec).as_quat()]


def row(reference, value):
    return {'reference_kf_id': str(reference), **dict(zip(POSE_KEYS, value))}


class HistoryTransport(unittest.TestCase):
    def setUp(self):
        self.old = {7: pose([1., 2., 3.], [.1, -.2, .3]),
                    19: pose([-2., 5., 1.], [-.2, .4, .1])}
        self.frame = pose([2., -3., 5.], [.2, .1, -.1])

    def test_identity_preserves_body_history(self):
        actual = transport_history([row(7, self.frame)], self.old, self.old)[0]
        np.testing.assert_allclose(actual, self.frame, atol=1e-14)

    def test_nonidentity_reference_preserves_relative_body_transform(self):
        new = {**self.old, 7: pose([4., -1., 6.], [-.4, .5, .2])}
        actual = transport_history([row(7, self.frame)], self.old, new)[0]
        old_rotation = Rotation.from_quat(self.old[7][3:])
        new_rotation = Rotation.from_quat(new[7][3:])
        relative_position = old_rotation.inv().apply(self.frame[:3] - self.old[7][:3])
        np.testing.assert_allclose(new_rotation.inv().apply(actual[:3] - new[7][:3]), relative_position, atol=1e-14)
        relative_rotation = old_rotation.inv() * Rotation.from_quat(self.frame[3:])
        actual_relative = new_rotation.inv() * Rotation.from_quat(actual[3:])
        self.assertLess((actual_relative.inv() * relative_rotation).magnitude(), 1e-14)

    def test_reference_id_selects_correction(self):
        new = {key: value.copy() for key, value in self.old.items()}
        new[19][:3] += [1., -2., 3.]
        rows = [row(19, self.frame), row(7, self.frame)]
        actual = transport_history(rows, self.old, new)
        np.testing.assert_allclose(actual[0, :3], self.frame[:3] + [1., -2., 3.])
        np.testing.assert_allclose(actual[1], self.frame, atol=1e-14)
        self.assertEqual(len(actual), len(rows))

    def test_missing_reference_fails(self):
        with self.assertRaisesRegex(ValueError, 'Missing surviving reference'):
            transport_history([row(8, self.frame)], self.old, self.old)


if __name__ == '__main__':
    unittest.main()
