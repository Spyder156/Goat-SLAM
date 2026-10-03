#!/usr/bin/env python3
"""Export a completed native COLMAP rig solve for LaMAria scoring and Rerun.

No alignment, GT access, pose interpolation, or pixel rematching occurs here.
The solver must retain every original SLAM pose timestamp. COLMAP's +0.5 pixel
centre offset is removed from observations and final intrinsic principal points.
The original physical calibration remains the official scoring input.
"""
import argparse
import csv
from decimal import Decimal
from collections import Counter
import hashlib
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path
import shutil

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def data_lines(path):
    with path.open() as handle:
        for line in handle:
            if line.strip() and not line.lstrip().startswith('#'):
                yield line.split()


def stamps_file(path):
    values = [int(row[0]) for row in data_lines(path)]
    if not values or any(b <= a for a, b in zip(values, values[1:])):
        raise ValueError(f'Nonempty strictly chronological integer timestamps required: {path}')
    return values


def seconds(stamp):
    return f'{stamp//1000000000}.{stamp%1000000000:09d}'


def source_timestamps(source):
    paths = list(source.glob('atlas_*_trajectory.csv'))
    if len(paths) != 1:
        raise ValueError('Source must have exactly one atlas trajectory')
    with paths[0].open() as handle:
        rows = list(csv.DictReader(handle))
    if not rows or len({row['coordinate_frame'] for row in rows}) != 1:
        raise ValueError('Source trajectory must use one existing map; no independent-map stitching')
    required = ('map_id', 'map_init_kf_id', 'coordinate_frame', 'history_index', 'reference_kf_id')
    if any(any(key not in row for key in required) for row in rows):
        raise ValueError('Source atlas is missing map/history identity fields')
    return paths[0], [int(Decimal(row['t_s'])*1000000000) for row in rows], rows


def read_poses(path):
    stamps, values = [], []
    for row in data_lines(path):
        if len(row) != 8:
            raise ValueError('Expected timestamp_ns px py pz qx qy qz qw')
        stamps.append(int(row[0])); values.append([float(x) for x in row[1:]])
    if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('Solver poses must be nonempty and strictly chronological')
    values = np.asarray(values)
    if not np.isfinite(values).all() or np.max(abs(np.linalg.norm(values[:, 3:], axis=1)-1)) > 1e-6:
        raise ValueError('Solver poses contain nonfinite values or nonunit xyzw quaternions')
    return stamps, values


def calibration(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError('Cannot read source settings')
    tbc = fs.getNode('IMU.T_b_c1').mat()
    rig = fs.getNode('Rig.T_c0_c1').mat()
    width = int(fs.getNode('Camera.width').real()); height = int(fs.getNode('Camera.height').real())
    names = ['fx', 'fy', 'cx', 'cy'] + [f'k{i}' for i in range(1, 7)] + ['p1', 'p2', 's1', 's2', 's3', 's4']
    cameras = [np.array([fs.getNode(f'Camera{cam+1}.{name}').real() for name in names]) for cam in (0, 1)]
    fs.release()
    for transform in (tbc, rig):
        if transform is None or transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError('Source rig/body calibration requires finite 4x4 matrices')
        u, _, vt = np.linalg.svd(transform[:3, :3])
        transform[:3, :3] = u @ np.diag([1., 1., np.linalg.det(u@vt)]) @ vt
    return (tbc, tbc@rig), cameras, (width, height)


def read_cameras(model, solver_calibration, source_cameras, dimensions):
    cameras = {}
    for row in data_lines(model/'cameras.txt'):
        cid = int(row[0]); params = np.asarray([float(x) for x in row[4:]])
        if cid in cameras or row[1] != 'RAD_TAN_THIN_PRISM_FISHEYE' or params.shape != (16,):
            raise ValueError('Require two distinct native 16-parameter Fisheye624 COLMAP cameras')
        if tuple(map(int, row[2:4])) != dimensions or not np.isfinite(params).all() or np.any(params[:2] <= 0):
            raise ValueError('Invalid camera dimensions or parameters')
        cameras[cid] = {'colmap': params}
    entries = json.loads(solver_calibration.read_text())['cameras']
    if len(entries) != 2 or len(cameras) != 2 or {entry['cam_index'] for entry in entries} != {0, 1}:
        raise ValueError('Require a calibration record for each physical camera')
    for entry in entries:
        cid, cam = int(entry['camera_id']), int(entry['cam_index'])
        if cid not in cameras or 'cam' in cameras[cid]:
            raise ValueError('Duplicate or unknown physical camera identity')
        initial, final = np.asarray(entry['initial']), np.asarray(entry['final'])
        expected = source_cameras[cam].copy(); expected[2:4] += .5
        if initial.shape != (16,) or final.shape != (16,) or not np.allclose(initial, expected, atol=2e-6, rtol=1e-7):
            raise ValueError('Solver initial calibration is not the supplied native model plus the pixel-centre offset')
        if not np.allclose(final, cameras[cid]['colmap'], atol=1e-10, rtol=1e-10):
            raise ValueError('Final model and solver calibration disagree')
        native = final.astype(float).copy(); native[2:4] -= .5
        cameras[cid].update({'cam': cam, 'native': native})
    return cameras


def image_index(model, cameras, stamps, body, extrinsics):
    """Index image offsets without retaining millions of SIFT observations."""
    pose_index = {stamp: i for i, stamp in enumerate(stamps)}
    rotations = Rotation.from_quat(body[:, 3:]).as_matrix()
    images, paired = {}, set()
    max_position_error = max_rotation_error = 0.
    with (model/'images.txt').open() as handle:
        while True:
            line = handle.readline()
            if not line:
                break
            if not line.strip() or line.startswith('#'):
                continue
            row = line.split()
            if len(row) != 10:
                raise ValueError('Malformed COLMAP image pose row')
            iid, cid = int(row[0]), int(row[8])
            if iid in images or cid not in cameras:
                raise ValueError('Duplicate image or unknown camera')
            name = Path(row[9]); stamp = int(name.stem); cam = cameras[cid]['cam']
            if name.parts[0] != f'cam{cam}' or stamp not in pose_index or (stamp, cam) in paired:
                raise ValueError('Image identity is inconsistent with physical camera or solver timestamps')
            quaternion = np.array([float(x) for x in row[1:5]])
            if abs(np.linalg.norm(quaternion)-1) > 1e-6:
                raise ValueError('Nonunit COLMAP wxyz image quaternion')
            rcw = Rotation.from_quat(quaternion[[1, 2, 3, 0]]).as_matrix()
            tcw = np.array([float(x) for x in row[5:8]])
            index = pose_index[stamp]; tbc = extrinsics[cam]
            expected_rotation = rotations[index] @ tbc[:3, :3]
            expected_position = body[index, :3] + rotations[index] @ tbc[:3, 3]
            max_position_error = max(max_position_error, float(np.max(np.abs(-rcw.T@tcw-expected_position))))
            max_rotation_error = max(max_rotation_error, float(np.max(np.abs(rcw.T-expected_rotation))))
            offset = handle.tell(); observation_line = handle.readline()
            if not observation_line or observation_line.startswith('#'):
                raise ValueError('Missing COLMAP image observation line')
            tokens = len(observation_line.split())
            if tokens % 3:
                raise ValueError('Malformed COLMAP image observations')
            images[iid] = {'stamp': stamp, 'cam': cam, 'offset': offset, 'count': tokens//3, 'name': row[9]}
            paired.add((stamp, cam))
    if paired != {(stamp, cam) for stamp in stamps for cam in (0, 1)}:
        raise ValueError('Solver model does not preserve both images at every source pose timestamp')
    if max_position_error > 1e-4 or max_rotation_error > 2e-5:
        raise ValueError(f'COLMAP camera/body/rig geometry disagrees: {max_position_error} m, {max_rotation_error} rotation')
    return images, {'camera_body_position_max_difference_m': max_position_error,
                    'camera_body_rotation_max_difference': max_rotation_error}


def read_points(model, images):
    points = {}
    repeated_image_landmarks = repeated_image_observations = 0
    for row in data_lines(model/'points3D.txt'):
        if len(row) < 10 or (len(row)-8) % 2:
            raise ValueError('Malformed or unobserved COLMAP point')
        pid = int(row[0]); xyz = np.array([float(x) for x in row[1:4]])
        if pid < 0 or pid in points or not np.isfinite(xyz).all():
            raise ValueError('Invalid or duplicate point')
        earliest, cam_bits, image_ids, observations, signature = None, 0, set(), set(), 0
        for i in range(8, len(row), 2):
            iid, point_index = int(row[i]), int(row[i+1])
            if iid not in images or not 0 <= point_index < images[iid]['count'] or (iid, point_index) in observations:
                raise ValueError(f'Point {pid} track references unknown image, invalid feature, or duplicate exact observation: {(iid, point_index)}')
            image = images[iid]; image_ids.add(iid); observations.add((iid, point_index))
            earliest = image['stamp'] if earliest is None else min(earliest, image['stamp'])
            cam_bits |= 1 << image['cam']
            signature += (iid << 32) + point_index
        # Native SIFT tracks may contain distinct orientation/feature indices
        # from the same image. Preserve those observations and their weights.
        repeated = len(observations)-len(image_ids)
        repeated_image_landmarks += int(repeated > 0)
        repeated_image_observations += repeated
        points[pid] = {'xyz': xyz, 'stamp': earliest, 'cam': {1: 0, 2: 1, 3: 2}[cam_bits],
                       'expected_count': len(observations), 'expected_signature': signature,
                       'observed_count': 0, 'observed_signature': 0}
    if not points:
        raise ValueError('No surviving triangulated SIFT landmarks')
    return points, {'landmarks_with_repeated_image_entries': repeated_image_landmarks,
                    'additional_distinct_features_in_same_image': repeated_image_observations,
                    'track_observation_uniqueness': '(COLMAP image ID, image feature index); all native entries preserved'}


def export(args):
    source, out = args.source.resolve(), args.output.resolve()
    solver = (args.solver_output or args.model).resolve()
    model = (args.model or solver).resolve()
    if out.exists():
        raise FileExistsError(f'Preserving existing output: {out}')
    result = json.loads((solver/'result.json').read_text())
    if result.get('usable') is not True:
        raise ValueError('Solver did not report a usable solution; no successful run will be fabricated')
    original_atlas, original_stamps, source_rows = source_timestamps(source)
    stamps, body = read_poses(solver/'body_trajectory_ns.txt')
    if stamps != original_stamps:
        raise ValueError('Solver changed pose timestamp coverage; interpolation or selected-keyframe scoring is not allowed')
    sift_manifest = None
    if args.sift_work is not None:
        sift_manifest = args.sift_work/'manifest.json'
        supplied = json.loads(sift_manifest.read_text())
        if relocated_path(supplied['source_run']).resolve() != source or supplied['pose_timestamps'] != len(stamps) or supplied['ground_truth_used']:
            raise ValueError('SIFT input provenance disagrees with the supplied source run')
    native_path = args.native_timestamps
    if native_path is None and (source/'config/native_timestamps_ns.txt').is_file():
        native_path = source/'config/native_timestamps_ns.txt'
    if native_path is None and args.sift_work is not None:
        image_path = args.sift_work/f'images/cam0/{stamps[0]}.png'
        if image_path.exists():
            native_path = image_path.resolve().parents[3]/'timestamps.txt'
    if native_path is None or not native_path.is_file():
        raise ValueError('Full native timestamp list is required; provide --native-timestamps')
    processed = stamps_file(source/'config/timestamps_ns.txt')
    native = stamps_file(native_path)
    native_indices = {stamp: i for i, stamp in enumerate(native)}
    if any(stamp not in native_indices for stamp in processed):
        raise ValueError('Processed images are not native timestamps')
    processed_ids = [native_indices[stamp] for stamp in processed]
    if processed_ids != list(range(processed_ids[0], processed_ids[0]+len(processed_ids))):
        raise ValueError('Processed source timestamps contain interior omissions')
    if not set(stamps).issubset(processed):
        raise ValueError('An exported pose lacks a processed source image')
    extrinsics, source_cameras, dimensions = calibration(source/'config/settings.yaml')
    cameras = read_cameras(model, solver/'calibration.json', source_cameras, dimensions)
    images, geometry = image_index(model, cameras, stamps, body, extrinsics)
    points, track_geometry = read_points(model, images)
    geometry.update(track_geometry)
    if result['frames'] != len(stamps) or result['images'] != len(images) or result['points'] != len(points):
        raise ValueError('Solver result counts disagree with actual exported geometry')
    out.mkdir(parents=True); (out/'config').mkdir(); (out/'input').mkdir()
    shutil.copy2(source/'config/settings.yaml', out/'config/settings.yaml')
    shutil.copy2(source/'config/timestamps_ns.txt', out/'config/timestamps_ns.txt')
    shutil.copy2(native_path, out/'config/native_timestamps_ns.txt')
    execution_manifest = solver/'execution_manifest.json'
    if execution_manifest.is_file():
        shutil.copy2(execution_manifest, out/'config/solver_execution_manifest.json')
    observation_metadata = solver/'observation_metadata.json'
    if observation_metadata.is_file():
        shutil.copy2(observation_metadata, out/'config/solver_observation_metadata.json')
    planned_manifest = args.sift_work.parent/'offline_experiment.json' if args.sift_work is not None else None
    if planned_manifest is not None and planned_manifest.is_file():
        shutil.copy2(planned_manifest, out/'config/solver_suite_plan.json')
    if (source/'input/euroc').is_dir():
        (out/'input/euroc').symlink_to((source/'input/euroc').resolve(), target_is_directory=True)
    (out/'solver_output').symlink_to(solver, target_is_directory=True)
    stem = out.name
    pose_path = out/f'f_{stem}.txt'
    shutil.copy2(solver/'body_trajectory_ns.txt', pose_path)
    with (out/f'atlas_{stem}_trajectory.csv').open('w') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t_s', 'map_id', 'map_init_kf_id', 'coordinate_frame', 'history_index', 'reference_kf_id', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
        for stamp, pose, row in zip(stamps, body, source_rows):
            writer.writerow([seconds(stamp), *[row[key] for key in ('map_id', 'map_init_kf_id', 'coordinate_frame', 'history_index', 'reference_kf_id')],
                             *[format(float(x), '.17g') for x in pose]])
    with (out/f'mp_{stem}.csv').open('w') as handle:
        writer = csv.writer(handle); writer.writerow(['t', 'x', 'y', 'z', 'cam', 'pointid'])
        for pid, point in sorted(points.items(), key=lambda item: (item[1]['stamp'], item[0])):
            writer.writerow([seconds(point['stamp']), *[format(float(x), '.17g') for x in point['xyz']], point['cam'], pid])
    feature_count = mapped_count = 0; frames_without_visual = 0
    with (model/'images.txt').open() as src, (out/f'kp_{stem}.csv').open('w') as kp, (out/f'ml_{stem}_kfpts.csv').open('w') as ml, (out/'visual_support.csv').open('w') as support:
        kw, mw, sw = csv.writer(kp), csv.writer(ml), csv.writer(support)
        kw.writerow(['t', 'cam', 'pointid', 'u', 'v', 'mapped'])
        mw.writerow(['t', 'cam', 'kfid', 'pointid', 'u', 'v'])
        sw.writerow(['timestamp_ns', 'cam', 'features', 'mapped', 'has_final_landmark'])
        for count, (iid, image) in enumerate(sorted(images.items(), key=lambda item: (item[1]['stamp'], item[1]['cam']))):
            src.seek(image['offset']); row = src.readline().split(); mapped = 0
            for index in range(image['count']):
                u, v = float(row[3*index])-.5, float(row[3*index+1])-.5
                pid = int(row[3*index+2])
                if not math.isfinite(u) or not math.isfinite(v) or pid < -1:
                    raise ValueError('Invalid native image observation')
                if pid >= 0:
                    if pid not in points:
                        raise ValueError('Orphaned image-to-landmark association')
                    point = points[pid]; mapped += 1
                    point['observed_count'] += 1; point['observed_signature'] += (iid << 32)+index
                    mw.writerow([seconds(image['stamp']), image['cam'], iid, pid, format(u, '.12g'), format(v, '.12g')])
                kw.writerow([seconds(image['stamp']), image['cam'], pid, format(u, '.12g'), format(v, '.12g'), int(pid >= 0)])
            feature_count += image['count']; mapped_count += mapped; frames_without_visual += int(mapped == 0)
            sw.writerow([image['stamp'], image['cam'], image['count'], mapped, int(mapped > 0)])
            if count % 2000 == 0:
                print(f'Exported SIFT observations for {count}/{len(images)} images', flush=True)
    if any(p['observed_count'] != p['expected_count'] or p['observed_signature'] != p['expected_signature'] for p in points.values()):
        raise ValueError('COLMAP image associations and landmark track lists disagree')
    with (out/'run.log').open('w') as handle:
        handle.write(f'Offline global refinement of {source}; mode={result["mode"]}; no online replay.\n')
        for camera in sorted(cameras.values(), key=lambda camera: camera['cam']):
            params = ','.join(format(float(x), '.17g') for x in camera['native'])
            handle.write(f'[CALIBRATION-COMMIT] t={seconds(stamps[-1])} map={source_rows[0]["map_id"]} cam={camera["cam"]} params={params}\n')
    solver_support = None
    if (solver/'frame_support.csv').is_file():
        with (solver/'frame_support.csv').open() as handle:
            support_rows = list(csv.DictReader(handle))
        if [int(row['timestamp_ns']) for row in support_rows] != stamps:
            raise ValueError('Solver support log does not cover the exact exported timestamps')
        solver_support = dict(Counter(row['pose_support'] for row in support_rows))
        shutil.copy2(solver/'frame_support.csv', out/'solver_frame_support.csv')
    coverage = {'run': str(out), 'input_frames': len(processed), 'native_input_frames': len(native),
                'input_start_s': processed[0]*1e-9, 'input_end_s': processed[-1]*1e-9,
                'unique_input_frames_with_exported_pose': len(stamps), 'exported_pose_fraction': len(stamps)/len(processed),
                'missing_processed_input_poses': len(processed)-len(stamps), 'missing_native_input_poses': len(native)-len(stamps),
                'independent_segments_exported': 1, 'estimator_returncode': 0, 'export_returncode': None,
                'pose_export_scope': 'All original source SLAM pose timestamps retained; solver_frame_support.csv distinguishes optimized and unchanged input states',
                'online_quality_source': 'Not applicable: offline refinement; see visual_support.csv and solver frame_support.csv',
                'solver_pose_support_counts': solver_support,
                'camera_images_with_zero_visual_observations': frames_without_visual,
                'interpolation_performed': False, 'new_timestamps_generated': False,
                'interpretation': ['Pose coverage does not imply visual support: unsupported frames retain an explicit solver status.',
                                   'A mapped SIFT observation is a final landmark association, not a claim of an accepted optimization residual.',
                                   'Initial missing poses and unsupported source boundaries remain missing.']}
    save(out/'coverage.json', coverage)
    save(out/'result.json', {'returncode': 0, 'export_returncode': None, 'kind': 'offline_colmap_global_refinement',
                           'solver_result': result, 'geometry_validation': geometry})
    inputs = [original_atlas, source/'config/settings.yaml', source/'config/timestamps_ns.txt', native_path,
              solver/'body_trajectory_ns.txt', solver/'calibration.json', solver/'result.json',
              model/'images.txt', model/'cameras.txt', model/'points3D.txt']
    if sift_manifest is not None:
        inputs.append(sift_manifest)
    if execution_manifest.is_file():
        inputs.append(execution_manifest)
    if observation_metadata.is_file():
        inputs.append(observation_metadata)
    if planned_manifest is not None and planned_manifest.is_file():
        inputs.append(planned_manifest)
    manifest = {'source_run': str(source), 'solver_output': str(solver), 'output_run': str(out),
                'GT_used': False, 'alignment_applied': False, 'interpolation_performed': False,
                'pixel_conversion': 'native = COLMAP - 0.5, for observations and cx/cy',
                'official_calibration': 'Exact copy of source settings; estimated intrinsics recorded separately',
                'coordinate_frame': source_rows[0]['coordinate_frame'],
                'atlas_history_identity': 'Source SLAM map/history/reference IDs preserved; they are not COLMAP image IDs',
                'observation_identity': 'ml kfid values are COLMAP image IDs; final SIFT point tracks replace source ORB associations',
                'poses': len(stamps), 'images': len(images), 'landmarks': len(points), 'SIFT_features': feature_count,
                'mapped_SIFT_observations': mapped_count, **geometry,
                'input_sha256': {str(path.resolve()): sha(path) for path in inputs},
                'adapter_sha256': sha(Path(__file__).resolve())}
    if execution_manifest.is_file():
        manifest['effective_solver_execution'] = {'source': str(execution_manifest),
                                                   'sha256': sha(execution_manifest),
                                                   'copied_to': 'config/solver_execution_manifest.json',
                                                   'precedence': 'Actual per-arm binary and settings; takes precedence over the initial suite plan'}
    if planned_manifest is not None and planned_manifest.is_file():
        manifest['original_solver_suite_plan'] = {'source': str(planned_manifest.resolve()),
                                                  'sha256': sha(planned_manifest), 'copied_to': 'config/solver_suite_plan.json'}
    if observation_metadata.is_file():
        manifest['solver_observation_metadata_audit'] = {'source': str(observation_metadata),
                                                         'sha256': sha(observation_metadata),
                                                         'copied_to': 'config/solver_observation_metadata.json'}
    save(out/'export_manifest.json', manifest)
    save(out/'command.json', {'command': sys.argv, 'source_run': str(source), 'solver_output': str(solver), 'GT_used': False})
    (out/'README.txt').write_text('Offline COLMAP refinement export. Raw world_from_imu-right poses, native timestamps, xyzw.\n'
                                 'No GT, alignment, interpolation, or replacement of initial missing poses.\n'
                                 'Green = triangulated SIFT landmark; red = extracted SIFT without a final landmark.\n'
                                 'Map points are final optimized positions revealed at their first retained observation.\n'
                                 'Atlas map/history/reference IDs retain original SLAM identity; ml kfid identifies COLMAP images.\n'
                                 'config/settings.yaml remains the original physical/official calibration; run.log records the final estimated model.\n')
    print(json.dumps(manifest, indent=2), flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', '--source', dest='source', type=relocated_path, required=True)
    parser.add_argument('--solver-output', type=relocated_path, help='Defaults to --model')
    parser.add_argument('--model', type=relocated_path, help='COLMAP TEXT model; defaults to --solver-output')
    parser.add_argument('--sift-work', type=relocated_path, help='Check SIFT provenance and locate the full native timestamps')
    parser.add_argument('--native-timestamps', type=relocated_path, help='Full native source image list, including unsupported boundaries')
    parser.add_argument('--run-dir', '--output', dest='output', type=relocated_path, required=True)
    args = parser.parse_args()
    if args.model is None and args.solver_output is None:
        parser.error('Provide --model or --solver-output')
    export(args)


if __name__ == '__main__':
    main()
