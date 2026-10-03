import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('output', Path(__file__).parents[1] / 'make_lamaria_output.py')
output = importlib.util.module_from_spec(spec)
spec.loader.exec_module(output)


class EvaluationOutputContract(unittest.TestCase):
    def cameras(self):
        return [{'parameters': np.array([230., 230., 320., 240., .01, -.001, 0, 0, 0, 0, .0001, -.0001, 0, 0, 0, 0])} for _ in range(2)]

    def test_sim3_preserves_both_camera_reprojections(self):
        cameras = self.cameras()
        rotation = Rotation.from_rotvec([.2, -.1, .3]).as_matrix()
        position = np.array([3., -2., .5])
        rig = np.eye(4)
        rig[:3, :3] = Rotation.from_rotvec([0., .8, 0.]).as_matrix()
        rig[:3, 3] = [.137, .005, -.003]
        local = np.array([[1., .4, 3.], [-1., .1, 4.], [.2, -1., 3.]])
        points = local @ rotation.T + position
        scale = .8861150149275466
        align = Rotation.from_rotvec([-.6, .7, .2]).as_matrix()
        offset = np.array([-75., -500., 3.])
        transformed = scale * (points @ align.T) + offset
        display_rig = rig.copy(); display_rig[:3, 3] *= scale
        transformed_pose = (align @ rotation, scale * (align @ position) + offset)
        for cam in (0, 1):
            rc = rotation if cam == 0 else rotation @ rig[:3, :3]
            pc = position if cam == 0 else position + rotation @ rig[:3, 3]
            pixels = output.project_fisheye624((points-pc) @ rc, cameras[cam]['parameters'])
            observations = [(index, *uv, 1) for index, uv in enumerate(pixels)]
            error, count = output.reprojection_stats(observations, cam, transformed_pose, transformed,
                                                     dict(enumerate(range(3))), display_rig, cameras)
            self.assertEqual(count, 3)
            self.assertLess(error, 1e-10)
        # Forgetting to scale the baseline is detectably wrong for cam1.
        error, _ = output.reprojection_stats(observations, 1, transformed_pose, transformed,
                                            dict(enumerate(range(3))), rig, cameras)
        self.assertGreater(error, .1)

    def test_saved_score_transform_is_used_without_refitting(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            times = np.array([1., 2., 3.])
            positions = np.array([[0., 0., 0.], [2., 1., 0.], [5., 3., 1.]])
            rotations = np.repeat(np.eye(3)[None], 3, axis=0)
            quaternion = Rotation.from_rotvec([.2, -.5, 1.]).as_quat()
            saved = {'run': str(run), 'estimated_poses': 3, 'Score2D': 67.19,
                     'CP_sim3': {'scale': .886, 'rotation_xyzw': quaternion.tolist(), 'translation': [5., 6., 7.]}}
            (run/'scores.json').write_text(json.dumps(saved))
            np.savetxt(run/'estimated_cam0_ns.txt', np.column_stack((times*1e9, positions, Rotation.from_matrix(rotations).as_quat())))
            # Deliberately incompatible GT: the exporter may compute residuals,
            # but it must never fit a replacement transform to make it agree.
            gt = np.column_stack((times, positions*5, Rotation.from_matrix(rotations).as_quat()))
            scale, rotation, offset, report, camera = output.evaluation_alignment(run/'scores.json', run, times, positions, rotations, gt)
            self.assertEqual(scale, .886)
            np.testing.assert_array_equal(offset, [5., 6., 7.])
            np.testing.assert_allclose(rotation, Rotation.from_quat(quaternion).as_matrix(), atol=0, rtol=0)
            self.assertEqual(report['challenge_score'], 67.19)
            np.testing.assert_array_equal(camera[:, 1:4], positions)
            with self.assertRaisesRegex(ValueError, 'geometry disagree'):
                output.evaluation_alignment(run/'scores.json', run, times, positions+.01, rotations, gt)

    def test_repeated_full_commits_keep_history_without_touching_input(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            cameras = self.cameras(); originals = [c['parameters'].copy() for c in cameras]
            lines = []
            for stamp, change in ((12., .2), (20., .4)):
                for cam in (0, 1):
                    parameters = originals[cam].copy(); parameters[:2] += change + cam*.1
                    parameters[4] += change*.001
                    lines.append(f'[CALIBRATION-COMMIT] t={stamp} map=0 cam={cam} params=' + ','.join(map(str, parameters)))
            (run/'run.log').write_text('\n'.join(lines))
            report = output.apply_committed_radial_calibration(run, cameras)
            self.assertEqual(report['commit_count'], 2)
            self.assertAlmostEqual(cameras[1]['parameters'][0], 230.5)
            self.assertFalse((run/'config').exists())
            self.assertEqual(report['cameras'][0]['original_parameters'], originals[0].tolist())

    def test_incomplete_commit_and_wrong_radial_update_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            camera = self.cameras()[0]['parameters']
            line = '[CALIBRATION-COMMIT] t=12 map=0 cam=0 params=' + ','.join(map(str, camera))
            (run/'run.log').write_text(line)
            with self.assertRaisesRegex(ValueError, 'Incomplete'):
                output.apply_committed_radial_calibration(run, self.cameras())
            camera[0] += 1.
            (run/'run.log').write_text('[RADIAL-COMMIT] t=12 map=0 cam=0 params=' + ','.join(map(str, camera)))
            with self.assertRaisesRegex(ValueError, 'another intrinsic'):
                output.apply_committed_radial_calibration(run, self.cameras())

    def test_multi_map_evaluation_never_uses_unselected_legacy_gauge(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            score = {'estimated_poses': 2, 'trajectory_selection': {'atlas_map_epochs': 2,
                     'selected_coordinate_frame': 'map_7_init_9_body0', 'selected_map_id': '7', 'selected_map_init_kf_id': '9'}}
            path = run/'scores.json'; path.write_text(json.dumps(score))
            (run/'atlas_fixture_trajectory.csv').write_text(
                't_s,map_id,map_init_kf_id,coordinate_frame,tx,ty,tz,qx,qy,qz,qw\n'
                '1,0,2,map_0_init_2_body0,1000000,0,0,0,0,0,1\n'
                '2,7,9,map_7_init_9_body0,1,2,3,0,0,0,1\n'
                '3,7,9,map_7_init_9_body0,2,3,4,0,0,0,1\n')
            (run/'atlas_fixture_points.csv').write_text(
                't_s,map_id,map_init_kf_id,coordinate_frame,point_id,x,y,z,cam\n'
                '1,0,2,map_0_init_2_body0,100,1000001,0,5,0\n'
                '2,7,9,map_7_init_9_body0,101,2,3,5,2\n')
            legacy = run/'mp_fixture.csv'; legacy.write_text('t,x,y,z,cam,id\n1,1000001,0,5,0,100\n')
            body, cloud, excluded, report = output.selected_evaluation_geometry(run, path, {'mp_': legacy})
            np.testing.assert_array_equal(body[:, 1:4], [[1, 2, 3], [2, 3, 4]])
            np.testing.assert_array_equal(cloud[:, 1:4], [[2, 3, 5]])
            self.assertEqual(excluded, {1000000000})
            self.assertEqual(report['selected_coordinate_frame'], 'map_7_init_9_body0')
            self.assertFalse(report['cross_map_alignment_performed'])
            self.assertEqual(report['omitted_other_map_poses'], 1)
            self.assertEqual(int(cloud[0, 5]), 101)


if __name__ == '__main__':
    unittest.main()
