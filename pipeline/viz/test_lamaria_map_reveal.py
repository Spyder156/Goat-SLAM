"""A culled reference must not postpone a surviving landmark's display."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from make_lamaria_output import map_reveal_times


class MapRevealTest(unittest.TestCase):
    def test_uses_point_identity_and_preserves_geometry_with_missing_observations(self):
        # Reference times have advanced after culling; rows are not ID-sorted.
        cloud = np.array([[90., 1., 2., 3., 0., 42.],
                          [80., 4., 5., 6., 1., 7.],
                          [70., 7., 8., 9., 2., 99.]])
        original = cloud.copy()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'mp_fixture.csv'
            path.with_name('ml_fixture_kfpts.csv').write_text(
                'kfid,t,cam,pointid,u,v\n'
                '1,20,0,7,10,20\n'
                '2,35,1,42,11,21\n'
                '3,12,0,42,12,22\n'
                '4,30,1,7,13,23\n'
                '5,1,0,12345,14,24\n')
            corrected, report = map_reveal_times(cloud, path)
            np.testing.assert_array_equal(corrected[:, 0], [12., 20., 70.])
            np.testing.assert_array_equal(corrected[:, 1:], original[:, 1:])
            np.testing.assert_array_equal(cloud, original)
            self.assertEqual(report['points_with_observation'], 2)
            self.assertEqual(report['points_without_observation'], 1)
            self.assertEqual(report['maximum_reveal_advance_s'], 78.)


if __name__ == '__main__':
    unittest.main()
