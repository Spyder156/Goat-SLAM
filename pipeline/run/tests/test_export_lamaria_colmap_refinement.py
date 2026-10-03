import argparse
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('adapter', Path(__file__).parents[1]/'export_lamaria_colmap_refinement.py')
adapter = importlib.util.module_from_spec(spec); spec.loader.exec_module(adapter)
spec = importlib.util.spec_from_file_location('viz', Path(__file__).parents[2]/'viz/make_lamaria_output.py')
viz = importlib.util.module_from_spec(spec); spec.loader.exec_module(viz)


class ColmapExportContract(unittest.TestCase):
    def fixture(self, root):
        source, solver, out = root/'source', root/'solver', root/'derived'
        (source/'config').mkdir(parents=True); solver.mkdir()
        stamps = [1000000000, 1050000000]
        tbc = np.eye(4); tbc[:3, :3] = Rotation.from_rotvec([.1, -.2, .05]).as_matrix(); tbc[:3, 3] = [.01, -.04, .02]
        rig = np.eye(4); rig[:3, :3] = Rotation.from_rotvec([0., .65, 0.]).as_matrix(); rig[:3, 3] = [.137, 0., .01]
        params = np.array([230., 230., 320., 240., .01, -.001, 0., 0., 0., 0., .001, -.001, 0., 0., 0., 0.])
        names = ['fx', 'fy', 'cx', 'cy']+[f'k{i}' for i in range(1, 7)]+['p1', 'p2', 's1', 's2', 's3', 's4']
        text = '%YAML:1.0\nCamera.width: 640\nCamera.height: 480\n'
        for cam in (1, 2):
            text += ''.join(f'Camera{cam}.{name}: {value:.17g}\n' for name, value in zip(names, params))
        for name, transform in [('IMU.T_b_c1', tbc), ('Rig.T_c0_c1', rig)]:
            text += f'{name}: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n   data: ['+', '.join(map(str, transform.ravel()))+']\n'
        (source/'config/settings.yaml').write_text(text)
        (source/'config/timestamps_ns.txt').write_text('950000000\n1000000000\n1050000000\n')
        native = root/'native.txt'; native.write_text('950000000\n1000000000\n1050000000\n1100000000\n')
        body = np.column_stack(([[0., 0., 0.], [.1, .2, .3]], Rotation.from_rotvec([[0., 0., .1], [.05, -.1, .2]]).as_quat()))
        with (source/'atlas_source_trajectory.csv').open('w') as handle:
            writer = csv.writer(handle); writer.writerow(['t_s', 'map_id', 'map_init_kf_id', 'coordinate_frame', 'history_index', 'reference_kf_id'])
            writer.writerows([[adapter.seconds(t), 7, 9, 'map_7_init_9_body0', index+12, index+51] for index, t in enumerate(stamps)])
        with (solver/'body_trajectory_ns.txt').open('w') as handle:
            for stamp, pose in zip(stamps, body):
                handle.write(str(stamp)+' '+' '.join(format(x, '.17g') for x in pose)+'\n')
        calibration, camlines = [], []
        for cam in (0, 1):
            initial = params.copy(); initial[2:4] += .5
            final = initial.copy(); final[:2] += .2+cam*.1; final[2] += .1
            calibration.append({'camera_id': cam+1, 'cam_index': cam, 'initial': initial.tolist(), 'final': final.tolist()})
            camlines.append(f'{cam+1} RAD_TAN_THIN_PRISM_FISHEYE 640 480 '+' '.join(format(x, '.17g') for x in final))
        (solver/'calibration.json').write_text(json.dumps({'cameras': calibration}))
        (solver/'cameras.txt').write_text('\n'.join(camlines)+'\n')
        xyz = {100: np.array([1., .2, 3.]), 101: np.array([2., .1, 4.])}
        lines = []; tracks = {100: [], 101: []}
        for index, (stamp, pose) in enumerate(zip(stamps, body)):
            rwb = Rotation.from_quat(pose[3:]).as_matrix()
            for cam, ext in enumerate((tbc, tbc@rig)):
                iid = index*2+cam+1; rwc = rwb@ext[:3, :3]; pwc = pose[:3]+rwb@ext[:3, 3]
                rcw = rwc.T; tcw = -rcw@pwc; q = Rotation.from_matrix(rcw).as_quat()[[3, 0, 1, 2]]
                lines.append(f'{iid} '+' '.join(format(x, '.17g') for x in np.r_[q, tcw])+f' {cam+1} cam{cam}/{stamp}.png')
                tokens = []
                for idx, pid in enumerate([100, 101] if cam == 0 else [100]):
                    uv = viz.project_fisheye624(((xyz[pid]-pwc)@rwc)[None], np.array(calibration[cam]['final']))[0]
                    tokens += [format(float(uv[0]), '.17g'), format(float(uv[1]), '.17g'), str(pid)]
                    tracks[pid] += [str(iid), str(idx)]
                tokens += ['50.5', '60.5', '-1']; lines.append(' '.join(tokens))
        (solver/'images.txt').write_text('\n'.join(lines)+'\n')
        (solver/'points3D.txt').write_text(''.join(str(pid)+' '+' '.join(map(str, point))+' 255 255 255 0 '+' '.join(tracks[pid])+'\n' for pid, point in xyz.items()))
        (solver/'result.json').write_text(json.dumps({'usable': True, 'mode': 'vi_calib', 'frames': 2, 'images': 4, 'points': 2}))
        return argparse.Namespace(source=source, solver_output=solver, model=None, native_timestamps=native, output=out, sift_work=None)

    def test_all_geometry_and_native_pixels_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.fixture(Path(directory)); original = (args.source/'config/settings.yaml').read_bytes()
            result = adapter.export(args)
            self.assertFalse(result['GT_used']); self.assertFalse(result['interpolation_performed'])
            self.assertEqual(result['poses'], 2); self.assertEqual(result['SIFT_features'], 10)
            self.assertEqual(result['mapped_SIFT_observations'], 6)
            self.assertEqual((args.output/'config/settings.yaml').read_bytes(), original)
            self.assertEqual((args.source/'config/settings.yaml').read_bytes(), original)
            coverage = json.loads((args.output/'coverage.json').read_text())
            self.assertEqual(coverage['missing_native_input_poses'], 2)
            self.assertEqual(coverage['missing_processed_input_poses'], 1)
            with (args.output/'mp_derived.csv').open() as handle:
                point_rows = list(csv.DictReader(handle))
            with (args.output/'atlas_derived_trajectory.csv').open() as handle:
                atlas_rows = list(csv.DictReader(handle))
            self.assertEqual(atlas_rows[0]['coordinate_frame'], 'map_7_init_9_body0')
            self.assertEqual(atlas_rows[1]['history_index'], '13')
            self.assertEqual(atlas_rows[1]['reference_kf_id'], '52')
            self.assertEqual([int(row['cam']) for row in point_rows], [2, 0])
            self.assertTrue(all(row['t'] == '1.000000000' for row in point_rows))
            tbc, rig, cameras = viz.load_calibration(args.output/'config/settings.yaml')
            committed = viz.apply_committed_radial_calibration(args.output, cameras)
            self.assertTrue(committed['committed'])
            times, positions, rotations, _ = viz.body_to_cam0(viz.load_table(args.output/'f_derived.txt', 8), tbc)
            points = np.array([[float(row[key]) for key in ('x', 'y', 'z')] for row in point_rows])
            point_ids = {int(row['pointid']): i for i, row in enumerate(point_rows)}
            for stamp, group in viz.keypoint_groups(args.output/'kp_derived.csv'):
                index = int(np.flatnonzero(np.isclose(times, stamp*1e-9, atol=1e-10))[0])
                for cam in (0, 1):
                    error, count = viz.reprojection_stats(group[cam], cam, (rotations[index], positions[index]), points, point_ids, rig, cameras)
                    self.assertEqual(count, 2 if cam == 0 else 1)
                    self.assertLess(error, 1e-8)
                    self.assertTrue(any(row[1:3] == (50., 60.) and not row[3] for row in group[cam]))
            with self.assertRaises(FileExistsError):
                adapter.export(args)

    def test_no_missing_timestamp_replacement_or_failed_result(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.fixture(Path(directory))
            p = args.solver_output/'body_trajectory_ns.txt'
            p.write_text(p.read_text().splitlines()[0]+'\n')
            with self.assertRaisesRegex(ValueError, 'timestamp coverage'):
                adapter.export(args)
            self.assertFalse(args.output.exists())

    def test_distinct_sift_features_in_same_image_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.fixture(Path(directory))
            images = args.solver_output/'images.txt'
            lines = images.read_text().splitlines()
            original = lines[1].split()[:3]
            lines[1] += ' '+' '.join(original)
            images.write_text('\n'.join(lines)+'\n')
            points = args.solver_output/'points3D.txt'
            lines = points.read_text().splitlines(); lines[0] += ' 1 3'
            points.write_text('\n'.join(lines)+'\n')
            result = adapter.export(args)
            self.assertEqual(result['SIFT_features'], 11)
            self.assertEqual(result['mapped_SIFT_observations'], 7)
            self.assertEqual(result['landmarks_with_repeated_image_entries'], 1)
            self.assertEqual(result['additional_distinct_features_in_same_image'], 1)
        with tempfile.TemporaryDirectory() as directory:
            args = self.fixture(Path(directory))
            points = args.solver_output/'points3D.txt'
            lines = points.read_text().splitlines(); lines[0] += ' 1 0'
            points.write_text('\n'.join(lines)+'\n')
            with self.assertRaisesRegex(ValueError, 'duplicate exact observation'):
                adapter.export(args)
            self.assertFalse(args.output.exists())
        with tempfile.TemporaryDirectory() as directory:
            args = self.fixture(Path(directory))
            (args.solver_output/'result.json').write_text(json.dumps({'usable': False}))
            with self.assertRaisesRegex(ValueError, 'usable solution'):
                adapter.export(args)
            self.assertFalse(args.output.exists())


if __name__ == '__main__':
    unittest.main()
