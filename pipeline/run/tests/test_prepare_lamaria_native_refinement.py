import argparse
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import pycolmap
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('native_adapter', Path(__file__).parents[1] / 'prepare_lamaria_native_refinement.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class NativeGraphContract(unittest.TestCase):
    def fixture(self, root):
        source = root / 'source'
        (source / 'config').mkdir(parents=True)
        tbc = adapter.matrix(Rotation.from_rotvec([.08, -.05, .03]).as_matrix(), [.02, -.04, .06])
        rig = adapter.matrix(Rotation.from_rotvec([.03, .25, -.04]).as_matrix(), [.13, -.03, .02])
        params = np.array([240., 240., 320., 240., .01, -.002, .0001, 0, 0, 0, .0001, -.0002, .0001, 0, -.0001, 0])
        text = '%YAML:1.0\nCamera.width: 640\nCamera.height: 480\n'
        for cam in (1, 2):
            text += ''.join(f'Camera{cam}.{name}: {value:.17g}\n' for name, value in zip(adapter.PARAMETERS, params))
        for name, value in [('IMU.T_b_c1', tbc), ('Rig.T_c0_c1', rig)]:
            text += f'{name}: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n   data: [' + ', '.join(map(str, value.ravel())) + ']\n'
        (source / 'config/settings.yaml').write_text(text)
        final_params = params.copy()
        final_params[:2] += .6
        log = ''
        for cam in (0, 1):
            log += f'[CALIBRATION-COMMIT] t=1.500000000 map=7 cam={cam} params=' + ','.join(map(str, final_params)) + '\n'
        log += '[CALIBRATION-TRIAL] t=1.500000000 accepted=1 initial_cost=20 fixed_cost=18 final_cost=16\n'
        log += '[CALIBRATION-CANDIDATE] t=2.000000000 cam=0 params=' + ','.join(map(str, final_params * 1.01)) + '\n'
        (source / 'run.log').write_text(log)
        body = {31: adapter.matrix(Rotation.from_rotvec([.05, -.03, .02]).as_matrix(), [1., 2., .3]),
                37: adapter.matrix(Rotation.from_rotvec([.08, -.02, .05]).as_matrix(), [1.15, 2.05, .35]),
                41: adapter.matrix(Rotation.from_rotvec([.1, .02, .08]).as_matrix(), [1.30, 2.08, .4])}
        points = {100 + i: np.array([1 + .02 * i, 2 + .004 * i * i, 4 + .03 * i]) for i in range(24)}
        kfrows, pointrows, camrows, obsrows = [], [], [], []
        for index, (kid, pose) in enumerate(body.items()):
            stamp = f'{1 + index}.000000000'
            kfrows.append([stamp, 7, 31, 'map_7_init_31_body0', kid, *pose[:3, 3], *Rotation.from_matrix(pose[:3, :3]).as_quat()])
            for cam in (0, 1):
                tcw = adapter.rigid_inverse(pose @ tbc @ (np.eye(4) if cam == 0 else rig))
                camrows.append([kid, stamp, cam, *tcw[:3, :3].ravel(), *tcw[:3, 3]])
                camera = pycolmap.Camera(model=adapter.MODEL, width=640, height=480, params=final_params)
                for pid, point in points.items():
                    uv = camera.img_from_cam(tcw[:3, :3] @ point + tcw[:3, 3])
                    obsrows.append([kid, stamp, cam, pid, *uv])
        for pid, point in points.items():
            pointrows.append(['1.000000000', 7, 31, 'map_7_init_31_body0', pid, *point, 2])
        files = {
            'atlas_fixture_keyframes.csv': (['t_s', 'map_id', 'map_init_kf_id', 'coordinate_frame', 'keyframe_id', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'], kfrows),
            'atlas_fixture_points.csv': (['t_s', 'map_id', 'map_init_kf_id', 'coordinate_frame', 'point_id', 'x', 'y', 'z', 'cam'], pointrows),
            'ml_fixture_kfpose_b0.csv': (['kfid', 't', 'cam'] + [f'r{i}{j}' for i in range(3) for j in range(3)] + ['tx', 'ty', 'tz'], camrows),
            'ml_fixture_kfpts.csv': (['kfid', 't', 'cam', 'pointid', 'u', 'v'], obsrows),
        }
        for name, (header, rows) in files.items():
            with (source / name).open('w') as stream:
                writer = csv.writer(stream); writer.writerow(header); writer.writerows(rows)
        return argparse.Namespace(source=source, output=root / 'prepared'), body, points, final_params

    def test_native_rig_geometry_pixels_ids_and_committed_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            args, body, points, params = self.fixture(Path(directory))
            source_settings = (args.source / 'config/settings.yaml').read_bytes()
            result = adapter.prepare(args)
            self.assertFalse(result['ground_truth_used'])
            self.assertEqual(result['frames'], 3)
            self.assertEqual(result['retained_landmarks'], 24)
            self.assertEqual(result['observations'], 144)
            self.assertEqual(result['cross_camera_landmarks'], 24)
            self.assertEqual(result['calibration']['timestamp_s'], '1.500000000')
            self.assertEqual((args.source / 'config/settings.yaml').read_bytes(), source_settings)
            reconstruction = pycolmap.Reconstruction(str(args.output / 'model'))
            for camera in reconstruction.cameras.values():
                expected = params.copy(); expected[2:4] += .5
                np.testing.assert_allclose(camera.params, expected, atol=1e-12)
            max_error = 0.
            for image in reconstruction.images.values():
                camera = reconstruction.cameras[image.camera_id]
                tcw = image.cam_from_world()
                for feature in image.points2D:
                    predicted = camera.img_from_cam(tcw * reconstruction.points3D[feature.point3D_id].xyz)
                    max_error = max(max_error, float(np.linalg.norm(predicted - feature.xy)))
            self.assertLess(max_error, 1e-9)
            self.assertEqual(set(reconstruction.points3D), set(points))
            with self.assertRaises(FileExistsError):
                adapter.prepare(args)

    def test_exact_duplicate_dedup_and_conflicting_observation_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            args, *_ = self.fixture(Path(directory))
            path = args.source / 'ml_fixture_kfpts.csv'
            first = path.read_text().splitlines()[1]
            with path.open('a') as stream:
                stream.write(first + '\n')
            result = adapter.prepare(args)
            self.assertEqual(result['exact_duplicate_rows_removed'], 1)
            self.assertEqual(result['observations'], 144)
            fields = first.split(','); fields[-1] = str(float(fields[-1]) + 1)
            with path.open('a') as stream:
                stream.write(','.join(fields) + '\n')
            args.output = Path(directory) / 'conflicting'
            with self.assertRaisesRegex(ValueError, 'conflicting'):
                adapter.prepare(args)

    def test_dangling_reference_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            args, *_ = self.fixture(Path(directory))
            path = args.source / 'ml_fixture_kfpts.csv'
            with path.open('a') as stream:
                stream.write('31,1.000000000,0,99999,300,240\n')
            with self.assertRaisesRegex(ValueError, 'unknown'):
                adapter.prepare(args)

    def test_bad_rig_or_mixed_map_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            args, *_ = self.fixture(Path(directory))
            path = args.source / 'ml_fixture_kfpose_b0.csv'
            lines = path.read_text().splitlines(); fields = lines[1].split(','); fields[-1] = str(float(fields[-1]) + .02)
            lines[1] = ','.join(fields); path.write_text('\n'.join(lines) + '\n')
            with self.assertRaisesRegex(ValueError, 'mismatch'):
                adapter.prepare(args)
        with tempfile.TemporaryDirectory() as directory:
            args, *_ = self.fixture(Path(directory))
            path = args.source / 'atlas_fixture_keyframes.csv'
            lines = path.read_text().splitlines(); fields = lines[-1].split(','); fields[1] = '8'
            lines[-1] = ','.join(fields); path.write_text('\n'.join(lines) + '\n')
            with self.assertRaisesRegex(ValueError, 'one nonempty native map'):
                adapter.prepare(args)


if __name__ == '__main__':
    unittest.main()
