#!/usr/bin/env python3
"""Export native-keyframe VI refinement through the saved SLAM reference history.

Every original exported timestamp is retained. Each frame keeps its measured
relative pose to its surviving reference keyframe; only that reference's world
pose changes. This is not interpolation, a new dense solve, or GT alignment.
"""
import argparse
import csv
from decimal import Decimal
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from export_lamaria_colmap_refinement import (
    calibration, image_index, read_cameras, read_points, read_poses,
    save, seconds, sha, source_timestamps, stamps_file,
)


POSE_KEYS = ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')


def unique_file(folder, pattern):
    paths = list(folder.glob(pattern))
    if len(paths) != 1:
        raise ValueError(f'Expected exactly one {pattern} in {folder}: {paths}')
    return paths[0]


def read_keyframes(source):
    path = unique_file(source, 'atlas_*_keyframes.csv')
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    if not rows or len({row['coordinate_frame'] for row in rows}) != 1:
        raise ValueError('Require one existing source-map coordinate system')
    poses, stamps = {}, {}
    for row in rows:
        identity = int(row['keyframe_id'])
        if identity in poses:
            raise ValueError('Duplicate native keyframe ID')
        poses[identity] = np.array([float(row[key]) for key in POSE_KEYS])
        stamps[identity] = int(Decimal(row['t_s']) * 1000000000)
    return path, rows, poses, stamps


def transport_history(rows, old_keyframes, new_keyframes, kf_stamps=None, missing_reference='error'):
    """Apply new_T_world_ref * old_T_ref_world to existing body poses.

    missing_reference='nearest': a history pose whose reference keyframe was culled before export is
    transported with the correction of the surviving keyframe nearest in time (expects kf_stamps in ns;
    row 't_s' in seconds). The number of such rows is left in transport_history.last_substitutions.
    """
    transport_history.last_substitutions = 0
    surviving = sorted((stamp, identity) for identity, stamp in (kf_stamps or {}).items() if identity in old_keyframes)
    if set(old_keyframes) != set(new_keyframes):
        raise ValueError('Refinement must preserve every retained keyframe')
    corrections = {}
    for identity, old in old_keyframes.items():
        new = new_keyframes[identity]
        if not np.isfinite(np.r_[old, new]).all():
            raise ValueError('Nonfinite keyframe pose')
        rotation = Rotation.from_quat(new[3:]) * Rotation.from_quat(old[3:]).inv()
        corrections[identity] = (rotation, new[:3] - rotation.apply(old[:3]))
    output = np.empty((len(rows), 7))
    for i, row in enumerate(rows):
        identity = int(row['reference_kf_id'])
        if identity not in corrections:
            if missing_reference != 'nearest' or not surviving:
                raise ValueError(f'Missing surviving reference keyframe {identity}')
            stamp = int(Decimal(row['t_s']) * 1000000000)
            identity = min(surviving, key=lambda item: abs(item[0] - stamp))[1]
            transport_history.last_substitutions += 1
        old = np.array([float(row[key]) for key in POSE_KEYS])
        if not np.isfinite(old).all():
            raise ValueError('Nonfinite source history pose')
        rotation, translation = corrections[identity]
        output[i, :3] = rotation.apply(old[:3]) + translation
        output[i, 3:] = (rotation * Rotation.from_quat(old[3:])).as_quat()
    return output


def export(source, prepared, model, out, missing_reference='error'):
    source, prepared, model, out = [Path(path).resolve() for path in (source, prepared, model, out)]
    if out.exists():
        raise FileExistsError(f'Choose a fresh output: {out}')
    result = json.loads((model / 'result.json').read_text())
    if result.get('usable') is not True or result['mode'] != 'vi_fixed':
        raise ValueError('Require a usable fixed-calibration VI result')
    original_atlas, stamps, rows = source_timestamps(source)
    kf_path, kf_rows, old_keyframes, kf_stamps = read_keyframes(source)
    identity_keys = ('coordinate_frame', 'map_id', 'map_init_kf_id')
    history_gauges = {tuple(row[key] for key in identity_keys) for row in rows}
    keyframe_gauges = {tuple(row[key] for key in identity_keys) for row in kf_rows}
    if len(history_gauges) != 1 or history_gauges != keyframe_gauges:
        raise ValueError('Trajectory and keyframes must share the same map coordinate system')
    prepared_manifest = json.loads((prepared / 'manifest.json').read_text())
    if Path(prepared_manifest['source_run']).resolve() != source:
        raise ValueError('Prepared graph belongs to another source run')
    solver_stamps, solver_body = read_poses(model / 'body_trajectory_ns.txt')
    if set(solver_stamps) != set(kf_stamps.values()) or len(solver_stamps) != len(kf_stamps):
        raise ValueError('Solver changed the retained keyframe timestamps')
    solver_by_stamp = dict(zip(solver_stamps, solver_body))
    new_keyframes = {identity: solver_by_stamp[stamp] for identity, stamp in kf_stamps.items()}
    body = transport_history(rows, old_keyframes, new_keyframes, kf_stamps, missing_reference)
    nearest_reference_rows = transport_history.last_substitutions
    identity_body = transport_history(rows, old_keyframes, old_keyframes, kf_stamps, missing_reference)
    original_body = np.array([[float(row[key]) for key in POSE_KEYS] for row in rows])
    identity_position_error = float(np.max(np.abs(identity_body[:, :3] - original_body[:, :3])))
    identity_rotation_error = float(np.max((Rotation.from_quat(identity_body[:, 3:]).inv() *
                                           Rotation.from_quat(original_body[:, 3:])).magnitude()))
    if identity_position_error > 1e-9 or identity_rotation_error > 1e-9:
        raise ValueError('Unchanged keyframes do not preserve the original history')
    settings = prepared / 'accepted_settings.yaml'
    extrinsics, camera_params, dimensions = calibration(settings)
    cameras = read_cameras(model, model / 'calibration.json', camera_params, dimensions)
    for camera in cameras.values():
        if not np.allclose(camera['native'], camera_params[camera['cam']], rtol=0., atol=1e-9):
            raise ValueError('Fixed-calibration experiment changed its lenses')
    images, geometry = image_index(model, cameras, solver_stamps, solver_body, extrinsics)
    points, tracks = read_points(model, images)
    geometry.update(tracks)
    if (result['frames'] != len(new_keyframes) or result['images'] != len(images) or
            result['points'] != len(points) or result['imu_intervals'] != len(new_keyframes)-1):
        raise ValueError('Actual optimized graph disagrees with solver report')
    # Verify reciprocal associations before exporting any successful candidate.
    with (model / 'images.txt').open() as handle:
        for iid, image in images.items():
            handle.seek(image['offset'])
            tokens = handle.readline().split()
            for index in range(image['count']):
                pid = int(tokens[3*index+2])
                if pid < 0:
                    continue
                if pid not in points:
                    raise ValueError('Dangling optimized image observation')
                points[pid]['observed_count'] += 1
                points[pid]['observed_signature'] += (iid << 32) + index
    if any(point['observed_count'] != point['expected_count'] or
           point['observed_signature'] != point['expected_signature'] for point in points.values()):
        raise ValueError('Optimized image observations and point tracks disagree')
    processed = stamps_file(source / 'config/timestamps_ns.txt')
    if not set(stamps).issubset(processed):
        raise ValueError('Source history contains timestamps absent from input')
    out.mkdir(parents=True)
    (out / 'config').mkdir()
    shutil.copy2(settings, out / 'config/settings.yaml')
    shutil.copy2(source / 'config/settings.yaml', out / 'config/source_settings.yaml')
    shutil.copy2(source / 'config/timestamps_ns.txt', out / 'config/timestamps_ns.txt')
    shutil.copy2(source / 'config/timestamps_ns.txt', out / 'config/native_timestamps_ns.txt')
    for filename in ('execution_manifest.json', 'calibration.json', 'frame_support.csv', 'imu_states.csv'):
        if (model / filename).is_file():
            shutil.copy2(model / filename, out / f'solver_{filename}')
    stem = out.name
    with (out / f'f_{stem}.txt').open('w') as handle:
        for stamp, pose in zip(stamps, body):
            handle.write(str(stamp) + ' ' + ' '.join(format(float(value), '.17g') for value in pose) + '\n')
    with (out / f'atlas_{stem}_trajectory.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        for row, pose in zip(rows, body):
            writer.writerow({**row, **{key: format(float(value), '.17g') for key, value in zip(POSE_KEYS, pose)}})
    with (out / f'atlas_{stem}_keyframes.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(kf_rows[0]))
        writer.writeheader()
        for row in kf_rows:
            pose = new_keyframes[int(row['keyframe_id'])]
            writer.writerow({**row, **{key: format(float(value), '.17g') for key, value in zip(POSE_KEYS, pose)}})
    with (out / f'mp_{stem}.csv').open('w') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t', 'x', 'y', 'z', 'cam', 'pointid'])
        for pid, point in sorted(points.items(), key=lambda item: (item[1]['stamp'], item[0])):
            writer.writerow([seconds(point['stamp']), *point['xyz'], point['cam'], pid])
    with (out / f'ml_{stem}_kfpts.csv').open('w') as handle, (model / 'images.txt').open() as src:
        writer = csv.writer(handle)
        writer.writerow(['kfid', 't', 'cam', 'pointid', 'u', 'v'])
        stamp_to_id = {stamp: identity for identity, stamp in kf_stamps.items()}
        for iid, image in sorted(images.items(), key=lambda item: (item[1]['stamp'], item[1]['cam'])):
            src.seek(image['offset'])
            tokens = src.readline().split()
            for index in range(image['count']):
                u, v, pid = tokens[3*index:3*index+3]
                if int(pid) >= 0:
                    writer.writerow([stamp_to_id[image['stamp']], seconds(image['stamp']), image['cam'], pid,
                                     format(float(u)-.5, '.12g'), format(float(v)-.5, '.12g')])
    # Preserve causal image associations for comparison; do not relabel them as
    # accepted offline factors. The map export contains only surviving points.
    kp_source = unique_file(source, 'kp_*.csv')
    (out / f'kp_{stem}.csv').symlink_to(kp_source)
    (out / 'source_run').symlink_to(source, target_is_directory=True)
    (out / 'solver_output').symlink_to(model, target_is_directory=True)
    if (source / 'input/euroc').is_dir():
        (out / 'input').mkdir()
        (out / 'input/euroc').symlink_to((source / 'input/euroc').resolve(), target_is_directory=True)
    with (out / 'run.log').open('w') as handle:
        handle.write('Offline native keyframe VI; full frame history transported through its saved reference keyframe.\n')
        for camera in sorted(cameras.values(), key=lambda value: value['cam']):
            handle.write(f'[CALIBRATION-COMMIT] t={seconds(stamps[-1])} map={rows[0]["map_id"]} cam={camera["cam"]} params=' +
                         ','.join(format(float(value), '.17g') for value in camera['native']) + '\n')
    shift = np.linalg.norm(body[:, :3] - original_body[:, :3], axis=1)
    steps = np.linalg.norm(np.diff(body[:, :3], axis=0), axis=1)
    transport = {'method': 'new_world_from_reference_body * old_reference_body_from_world * old_world_from_body',
                 'source_history_identity_max_position_error_m': identity_position_error,
                 'history_rows_transported_via_nearest_surviving_keyframe': nearest_reference_rows,
                 'source_history_identity_max_rotation_error_rad': identity_rotation_error,
                 'source_pose_count': len(stamps), 'optimized_keyframes': len(new_keyframes),
                 'keyframe_only_optimization': True, 'new_timestamps_generated': False, 'interpolation_performed': False,
                 'frame_position_change_median_m': float(np.median(shift)), 'frame_position_change_maximum_m': float(shift.max()),
                 'final_largest_adjacent_step_m': float(steps.max()),
                 'non_keyframe_states': 'Relative source history preserved; no separate dense-frame visual/IMU reoptimization.',
                 'image_overlay_semantics': 'Original online associations, not offline residual-inlier classifications.'}
    coverage = {'run': str(out), 'input_frames': len(processed), 'native_input_frames': len(processed),
                'input_start_s': processed[0]*1e-9, 'input_end_s': processed[-1]*1e-9,
                'unique_input_frames_with_exported_pose': len(stamps), 'exported_pose_fraction': len(stamps)/len(processed),
                'missing_native_input_poses': len(processed)-len(stamps), 'independent_segments_exported': 1,
                'estimator_returncode': 0, 'pose_export_scope': transport['method'], **transport}
    save(out / 'coverage.json', coverage)
    save(out / 'result.json', {'returncode': 0, 'kind': 'offline_native_keyframe_vi',
                             'solver_result': result, 'geometry_validation': geometry, 'history_transport': transport})
    files = [original_atlas, kf_path, settings, model/'result.json', model/'body_trajectory_ns.txt',
             model/'calibration.json', model/'images.txt', model/'points3D.txt', Path(__file__)]
    save(out / 'command.json', {'command': sys.argv, 'source_run': str(source), 'prepared_input': str(prepared),
                               'solver_output': str(model), 'GT_used': False,
                               'source_sha256': {str(path): sha(path) for path in files}})
    (out / 'README.txt').write_text('Native keyframe VI refinement. Raw world_from_body xyzw; no evaluation alignment.\n'
        'All source frame timestamps preserved through exact reference-keyframe correction; no dense-frame reoptimization.\n'
        'Image keypoints retain source online associations. See result.json and coverage.json for provenance.\n')
    print(json.dumps({'output': str(out), 'poses': len(stamps), 'keyframes': len(new_keyframes),
                      'points': len(points), 'history_transport': transport}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--prepared', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--missing-reference', choices=('error', 'nearest'), default='error',
                        help='History poses whose reference keyframe was culled: fail (default) or use the nearest surviving keyframe')
    args = parser.parse_args()
    export(args.source_run, args.prepared, args.model, args.out, args.missing_reference)
