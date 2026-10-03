#!/usr/bin/env python3
"""Prepare fixed-calibration VI-BA from one saved native SLAM map.

No images, ground truth, rematching, triangulation, or trajectory alignment are
used. Original native keyframe observations become a COLMAP text rig graph.
Velocity, bias, covariance, and feature octave were not serialized by the source
CSV exporter; the manifest makes that limitation explicit.
"""
import argparse
import csv
from decimal import Decimal
import hashlib
import json
import re
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

PARAMETERS = ['fx', 'fy', 'cx', 'cy'] + [f'k{i}' for i in range(1, 7)] + [
    'p1', 'p2', 's1', 's2', 's3', 's4']
MODEL = 'RAD_TAN_THIN_PRISM_FISHEYE'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def one(source, pattern):
    paths = list(source.glob(pattern))
    if len(paths) != 1:
        raise ValueError(f'Expected exactly one {pattern}; found {len(paths)}')
    return paths[0]


def stamp_ns(value):
    exact = Decimal(value) * 1000000000
    if exact != exact.to_integral_value():
        raise ValueError(f'Timestamp has subnanosecond precision: {value}')
    return int(exact)


def matrix(rotation, translation):
    value = np.eye(4)
    value[:3, :3] = rotation
    value[:3, 3] = translation
    return value


def rigid_inverse(value):
    return matrix(value[:3, :3].T, -value[:3, :3].T @ value[:3, 3])


def numbers(values):
    return ' '.join(format(float(value), '.17g') for value in values)


def pose_tokens(value):
    xyzw = Rotation.from_matrix(value[:3, :3]).as_quat()
    return numbers(np.r_[xyzw[[3, 0, 1, 2]], value[:3, 3]])


def read_settings(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError('Cannot open source settings')
    dimensions = tuple(int(fs.getNode('Camera.' + name).real()) for name in ('width', 'height'))
    cameras = []
    for cam in (1, 2):
        if any(fs.getNode(f'Camera{cam}.{name}').empty() for name in PARAMETERS):
            raise ValueError('Missing native camera parameter')
        cameras.append(np.array([fs.getNode(f'Camera{cam}.{name}').real() for name in PARAMETERS]))
    transforms = []
    corrections = {}
    for name in ('IMU.T_b_c1', 'Rig.T_c0_c1'):
        transform = fs.getNode(name).mat()
        if transform is None or transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError(f'Invalid transform: {name}')
        transform = transform.astype(float)
        if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
            raise ValueError(f'Invalid homogeneous row: {name}')
        u, singular, vt = np.linalg.svd(transform[:3, :3])
        if np.max(abs(singular - 1)) > 1e-5 or np.linalg.det(transform[:3, :3]) <= 0:
            raise ValueError(f'Not a proper physical rotation: {name}')
        rotation = u @ vt
        corrections[name] = float(np.max(abs(rotation - transform[:3, :3])))
        transform[:3, :3] = rotation
        transforms.append(transform)
    fs.release()
    if min(dimensions) <= 0:
        raise ValueError('Invalid image dimensions')
    return cameras, dimensions, transforms[0], transforms[1], corrections


def accepted_calibration(log_path, original, map_id):
    """Use paired committed values, never a rejected CALIBRATION-CANDIDATE."""
    groups = {}
    accepted = set()
    commit = re.compile(r'\[CALIBRATION-COMMIT\] t=(\S+) map=(\d+) cam=([01]) params=(\S+)')
    trial = re.compile(r'\[CALIBRATION-TRIAL\] t=(\S+) accepted=1(?:\s|$)')
    with log_path.open() as stream:
        for line in stream:
            match = commit.search(line)
            if match and int(match[2]) == map_id:
                time, cam = match[1], int(match[3])
                values = np.array([float(x) for x in match[4].split(',')])
                if values.shape != (16,) or not np.isfinite(values).all():
                    raise ValueError('Malformed committed camera parameters')
                if cam in groups.setdefault(time, {}):
                    raise ValueError('Repeated camera commit at one timestamp')
                groups[time][cam] = values
            match = trial.search(line)
            if match:
                accepted.add(match[1])
    if any(set(group) != {0, 1} or time not in accepted for time, group in groups.items()):
        raise ValueError('Incomplete or unconfirmed calibration commit')
    latest = max(groups, key=Decimal) if groups else None
    values = [groups[latest][cam].copy() for cam in (0, 1)] if latest else [x.copy() for x in original]
    for value in values:
        if min(value[:2]) <= 0 or abs(value[0] - value[1]) > 1e-7:
            raise ValueError('Native Fisheye624 requires a single positive focal length')
    return values, {'source': 'last paired accepted commit' if latest else 'source settings; no commits',
                    'timestamp_s': latest, 'accepted_paired_commits': len(groups),
                    'native_parameters': [x.tolist() for x in values]}


def read_keyframes(path):
    keyframes, map_keys, times = {}, set(), set()
    with path.open() as stream:
        for row in csv.DictReader(stream):
            kid, stamp = int(row['keyframe_id']), stamp_ns(row['t_s'])
            key = (int(row['map_id']), int(row['map_init_kf_id']), row['coordinate_frame'])
            position = np.array([float(row[name]) for name in ('tx', 'ty', 'tz')])
            quaternion = np.array([float(row[name]) for name in ('qx', 'qy', 'qz', 'qw')])
            if kid in keyframes or stamp in times or kid < 0:
                raise ValueError('Duplicate keyframe or timestamp')
            if not np.isfinite(np.r_[position, quaternion]).all() or abs(np.linalg.norm(quaternion) - 1) > 1e-5:
                raise ValueError('Invalid body pose or nonunit xyzw quaternion')
            keyframes[kid] = {'stamp_ns': stamp, 'body': matrix(Rotation.from_quat(quaternion).as_matrix(), position)}
            times.add(stamp); map_keys.add(key)
    if not keyframes or len(map_keys) != 1:
        raise ValueError('Exactly one nonempty native map required; no map joining or selection')
    return keyframes, next(iter(map_keys))


def read_points(path, map_key):
    points = {}
    with path.open() as stream:
        for row in csv.DictReader(stream):
            key = (int(row['map_id']), int(row['map_init_kf_id']), row['coordinate_frame'])
            pid = int(row['point_id'])
            xyz = np.array([float(row[name]) for name in ('x', 'y', 'z')])
            if key != map_key or pid in points or pid < 0 or not np.isfinite(xyz).all():
                raise ValueError('Point identity, map, or finite XYZ contract failed')
            points[pid] = xyz
    if not points:
        raise ValueError('No native landmarks')
    return points


def validate_camera_snapshots(path, keyframes, tbc, rig):
    seen, rotation_error, translation_error = set(), 0., 0.
    with path.open() as stream:
        for row in csv.DictReader(stream):
            kid, cam = int(row['kfid']), int(row['cam'])
            if kid not in keyframes or cam not in (0, 1) or (kid, cam) in seen:
                raise ValueError('Invalid camera snapshot identity')
            if stamp_ns(row['t']) != keyframes[kid]['stamp_ns']:
                raise ValueError('Camera snapshot timestamp does not match keyframe')
            observed = matrix(np.array([float(row[f'r{i}{j}']) for i in range(3) for j in range(3)]).reshape(3, 3),
                              [float(row[x]) for x in ('tx', 'ty', 'tz')])
            expected = rigid_inverse(keyframes[kid]['body'] @ tbc @ (np.eye(4) if cam == 0 else rig))
            rotation_error = max(rotation_error, float(np.max(abs(observed[:3, :3] - expected[:3, :3]))))
            translation_error = max(translation_error, float(np.max(abs(observed[:3, 3] - expected[:3, 3]))))
            if not np.isfinite(observed).all():
                raise ValueError('Nonfinite camera snapshot')
            seen.add((kid, cam))
    if seen != {(kid, cam) for kid in keyframes for cam in (0, 1)}:
        raise ValueError('Missing camera snapshot')
    # Source uses float32 geometry, 6-decimal camera matrices, 9-decimal body
    # quaternions. Camera translation recomposition amplifies rotation rounding
    # over a kilometre-scale coordinate frame. Do not fit a corrective transform.
    if rotation_error > 1e-5 or translation_error > 0.002:
        raise ValueError(f'Native camera/body/rig mismatch: R={rotation_error}, t={translation_error} m')
    return {'rotation_max_element_error': rotation_error,
            'translation_max_element_error_m': translation_error,
            'source_precision': 'float32 native geometry; camera matrix CSV 6 decimals; body CSV 9 decimals',
            'alignment_fitted': False}


def read_observations(path, keyframes, points, dimensions):
    observations = {(kid, cam): {} for kid in keyframes for cam in (0, 1)}
    pixels = {key: {} for key in observations}
    duplicates = rows = 0
    with path.open() as stream:
        for row in csv.DictReader(stream):
            rows += 1
            kid, cam, pid = int(row['kfid']), int(row['cam']), int(row['pointid'])
            key = (kid, cam)
            if key not in observations or pid not in points:
                raise ValueError('Observation references an unknown keyframe, camera, or landmark')
            if stamp_ns(row['t']) != keyframes[kid]['stamp_ns']:
                raise ValueError('Observation timestamp does not match keyframe')
            uv = (float(row['u']), float(row['v']))
            if not np.isfinite(uv).all() or not (0 <= uv[0] < dimensions[0] and 0 <= uv[1] < dimensions[1]):
                raise ValueError('Native observation outside finite image bounds')
            if pid in observations[key]:
                if observations[key][pid] != uv:
                    raise ValueError('One landmark has conflicting native observations in the same image')
                duplicates += 1
                continue
            if uv in pixels[key] and pixels[key][uv] != pid:
                raise ValueError('Two landmarks claim the same serialized native image coordinate')
            observations[key][pid] = uv
            pixels[key][uv] = pid
    support, cameras = {}, {}
    for (kid, cam), image in observations.items():
        for pid in image:
            support[pid] = support.get(pid, 0) + 1
            cameras[pid] = cameras.get(pid, 0) | (1 << cam)
    retained = {pid: points[pid] for pid, count in support.items() if count >= 2}
    removed_observations = sum(count for pid, count in support.items() if pid not in retained)
    for image in observations.values():
        for pid in list(image):
            if pid not in retained:
                del image[pid]
    counts = {'source_rows': rows, 'exact_duplicate_rows_removed': duplicates,
              'source_landmarks': len(points), 'retained_landmarks': len(retained),
              'unobserved_landmarks_excluded': len(points.keys() - support.keys()),
              'single_view_landmarks_excluded': sum(count < 2 for count in support.values()),
              'single_view_observations_excluded': removed_observations,
              'observations': sum(map(len, observations.values())),
              'cross_camera_landmarks': sum(cameras[pid] == 3 for pid in retained),
              'zero_observation_images': sum(not image for image in observations.values()),
              'minimum_track_length': min((support[pid] for pid in retained), default=0)}
    return observations, retained, counts


def write_model(output, keyframes, points, observations, cameras, dimensions, tbc, rig):
    output.mkdir()
    order = sorted(keyframes, key=lambda kid: keyframes[kid]['stamp_ns'])
    images = {(kid, cam): 2 * index + cam + 1 for index, kid in enumerate(order) for cam in (0, 1)}
    transforms = {}
    for kid in order:
        transforms[kid] = rigid_inverse(keyframes[kid]['body'] @ tbc)
    with (output / 'cameras.txt').open('w') as stream:
        stream.write('# Native Fisheye624, principal point shifted +0.5 once\n')
        for cam, params in enumerate(cameras):
            shifted = params.copy(); shifted[2:4] += .5
            stream.write(f'{cam+1} {MODEL} {dimensions[0]} {dimensions[1]} {numbers(shifted)}\n')
    rig_inverse = rigid_inverse(rig)
    (output / 'rigs.txt').write_text('# RIG_ID NUM_SENSORS REF_SENSOR_TYPE REF_SENSOR_ID SENSORS\n'
                                    f'1 2 CAMERA 1 CAMERA 2 1 {pose_tokens(rig_inverse)}\n')
    tracks = {pid: [] for pid in points}
    with (output / 'frames.txt').open('w') as frames, (output / 'images.txt').open('w') as image_file:
        frames.write('# FRAME_ID RIG_ID RIG_FROM_WORLD NUM_DATA_IDS DATA_IDS\n')
        image_file.write('# Native keyframes only; no fabricated dense observations\n')
        for kid in order:
            frames.write(f'{kid} 1 {pose_tokens(transforms[kid])} 2 CAMERA 1 {images[kid, 0]} CAMERA 2 {images[kid, 1]}\n')
            for cam in (0, 1):
                iid = images[kid, cam]
                tcw = transforms[kid] if cam == 0 else rig_inverse @ transforms[kid]
                image_file.write(f'{iid} {pose_tokens(tcw)} {cam+1} cam{cam}/{keyframes[kid]["stamp_ns"]}.png\n')
                tokens = []
                for index, (pid, uv) in enumerate(observations[kid, cam].items()):
                    tokens.extend([format(uv[0] + .5, '.17g'), format(uv[1] + .5, '.17g'), str(pid)])
                    tracks[pid].append((iid, index))
                image_file.write(' '.join(tokens) + '\n')
    with (output / 'points3D.txt').open('w') as stream:
        stream.write('# Original native landmark IDs/XYZ; error is unset metadata, not a measured zero\n')
        for pid, xyz in points.items():
            track = ' '.join(f'{iid} {index}' for iid, index in tracks[pid])
            stream.write(f'{pid} {numbers(xyz)} 255 255 255 0 {track}\n')
    return images, transforms


def verify_model(path, keyframes, points, observations, images, cameras, dimensions, transforms):
    import pycolmap
    reconstruction = pycolmap.Reconstruction(str(path))
    if (reconstruction.num_reg_images() != 2 * len(keyframes)
            or reconstruction.num_reg_frames() != len(keyframes)
            or reconstruction.num_points3D() != len(points)):
        raise ValueError('COLMAP roundtrip changed graph counts')
    count, pose_error = 0, 0.
    for kid, transform in transforms.items():
        actual = reconstruction.frames[kid].rig_from_world.matrix()
        pose_error = max(pose_error, float(np.max(abs(actual - transform[:3]))))
    if pose_error > 1e-9:
        raise ValueError('COLMAP changed the saved native keyframe pose')
    for (kid, cam), image_id in images.items():
        image = reconstruction.images[image_id]
        expected = list(observations[kid, cam].items())
        if image.camera_id != cam + 1 or image.frame_id != kid or len(image.points2D) != len(expected):
            raise ValueError('COLMAP image identity or point count changed')
        if image.name != f'cam{cam}/{keyframes[kid]["stamp_ns"]}.png':
            raise ValueError('COLMAP timestamp changed')
        for feature, (pid, uv) in zip(image.points2D, expected):
            if feature.point3D_id != pid or not np.allclose(feature.xy, np.array(uv) + .5, atol=1e-12, rtol=0):
                raise ValueError('COLMAP observation identity or pixel convention changed')
            count += 1
    for pid, point in reconstruction.points3D.items():
        if not np.allclose(point.xyz, points[pid], atol=1e-12, rtol=0):
            raise ValueError('COLMAP changed original metric XYZ')
        for element in point.track.elements:
            if reconstruction.images[element.image_id].points2D[element.point2D_idx].point3D_id != pid:
                raise ValueError('COLMAP reciprocal ownership failed')
    for cam, native in enumerate(cameras):
        expected = native.copy(); expected[2:4] += .5
        camera = reconstruction.cameras[cam + 1]
        if camera.model.name != MODEL or (camera.width, camera.height) != dimensions or not np.allclose(camera.params, expected, atol=1e-12, rtol=0):
            raise ValueError('COLMAP changed camera model or parameters')
    return {'pycolmap_version': pycolmap.__version__, 'reciprocal_observations_checked': count,
            'camera0_pose_max_element_roundtrip_difference': pose_error,
            'original_metric_xyz_preserved': True, 'native_pixels_preserved_with_offset': .5}


def prepare(args):
    source, output = args.source.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError(f'Preserving existing output: {output}')
    paths = {name: one(source, pattern) for name, pattern in {
        'keyframes': 'atlas_*_keyframes.csv', 'points': 'atlas_*_points.csv',
        'observations': 'ml_*_kfpts.csv', 'camera_snapshots': 'ml_*_kfpose_b0.csv'}.items()}
    paths.update(settings=source / 'config/settings.yaml', log=source / 'run.log')
    original, dimensions, tbc, rig, orthogonalization = read_settings(paths['settings'])
    keyframes, map_key = read_keyframes(paths['keyframes'])
    points = read_points(paths['points'], map_key)
    camera_audit = validate_camera_snapshots(paths['camera_snapshots'], keyframes, tbc, rig)
    cameras, calibration = accepted_calibration(paths['log'], original, map_key[0])
    observations, points, counts = read_observations(paths['observations'], keyframes, points, dimensions)
    if len(keyframes) < 2 or len(points) < 21 or counts['cross_camera_landmarks'] < 20:
        raise ValueError('Native graph does not meet existing VI solver support requirements')
    output.mkdir(parents=True)
    images, transforms = write_model(output / 'model', keyframes, points, observations, cameras, dimensions, tbc, rig)
    verification = verify_model(output / 'model', keyframes, points, observations, images, cameras, dimensions, transforms)
    settings = paths['settings'].read_text()
    for cam, params in enumerate(cameras):
        for name, value in zip(PARAMETERS, params):
            settings, replacements = re.subn(rf'(?m)^Camera{cam+1}\.{name}:\s*[^\n]+$',
                                             f'Camera{cam+1}.{name}: {value:.17g}', settings)
            if replacements != 1:
                raise ValueError('Source settings parameter missing or repeated')
    (output / 'accepted_settings.yaml').write_text(settings)
    with (output / 'keyframe_mapping.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['keyframe_id', 'timestamp_ns', 'frame_id', 'cam0_image_id', 'cam1_image_id'])
        for kid in sorted(keyframes, key=lambda kid: keyframes[kid]['stamp_ns']):
            writer.writerow([kid, keyframes[kid]['stamp_ns'], kid, images[kid, 0], images[kid, 1]])
    with (output / 'original_keyframe_body_poses_ns.txt').open('w') as stream:
        stream.write('# timestamp_ns px py pz qx qy qz qw; unchanged saved body gauge\n')
        for kid in sorted(keyframes, key=lambda kid: keyframes[kid]['stamp_ns']):
            body = keyframes[kid]['body']
            stream.write(f'{keyframes[kid]["stamp_ns"]} {numbers(np.r_[body[:3, 3], Rotation.from_matrix(body[:3, :3]).as_quat()])}\n')
    manifest = {
        'source_run': str(source), 'output': str(output), 'ground_truth_used': False,
        'map_id': map_key[0], 'map_init_kf_id': map_key[1], 'coordinate_frame': map_key[2],
        'frames': len(keyframes), 'images': 2 * len(keyframes), **counts,
        'keyframe_mapping_csv': str(output / 'keyframe_mapping.csv'),
        'calibration': calibration, 'camera_snapshot_validation': camera_audit,
        'rotation_orthogonalization_max_elements': orthogonalization,
        'roundtrip_validation': verification, 'metric_rig_baseline_m': float(np.linalg.norm(rig[:3, 3])),
        'timestamp_start_ns': min(x['stamp_ns'] for x in keyframes.values()),
        'timestamp_end_ns': max(x['stamp_ns'] for x in keyframes.values()),
        'input_hashes': {name: {'path': str(path), 'sha256': sha(path)} for name, path in paths.items()},
        'adapter_sha256': sha(Path(__file__)),
        'output_hashes': {str(path.relative_to(output)): sha(path) for path in sorted(output.rglob('*')) if path.is_file()},
        'limitations': [
            'Only saved keyframe observations are optimized, not all video-frame observations.',
            'Native velocities, biases, preintegrations and their uncertainty were not serialized; VI backend must initialize and integrate supplied calibrated IMU.',
            'Feature octave/covariance and original pooled feature indices were not serialized; adapter creates unique local image indices and retains 0.001-pixel CSV measurements.',
            'Pixel-coordinate uniqueness is checked at serialized precision; original feature-slot ownership cannot be independently reconstructed.',
            'Point ERROR=0 in COLMAP text is unset metadata; it is not a measured reprojection statistic.',
            'Graph timestamps are exact decimal encodings from saved keyframe CSV; full-history coverage must be preserved separately by the parent exporter.',
        ],
    }
    save(output / 'manifest.json', manifest)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', '--source-run', dest='source', type=Path, required=True)
    parser.add_argument('--output', '--out', dest='output', type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args)
    print(json.dumps({key: result[key] for key in ('frames', 'images', 'retained_landmarks', 'observations', 'cross_camera_landmarks', 'output')}, indent=2))


if __name__ == '__main__':
    main()
