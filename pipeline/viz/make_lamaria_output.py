#!/usr/bin/env python3
"""Package a LaMAria run in metric or explicitly evaluation-aligned coordinates.

Raw f_*.txt is T_world_body, xyzw, nanoseconds. Raw mp_*.csv already uses
the SAME exported world frame. Convert body poses to cam0 with the run's
T_body_cam0, then apply ONE optional SE3 alignment to trajectory AND points.
By default scale is unchanged. --evaluation-score applies the already saved
official control-point Sim3 to every displayed 3D object; it never fits a new
transform, edits the estimator output, or changes a submitted trajectory.
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import re
import hashlib
from collections import defaultdict, deque
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

COLORS = np.array([[60, 140, 255], [245, 75, 65], [55, 215, 90]], dtype=np.uint8)
MAPPED_COLOR = np.array([40, 230, 80], dtype=np.uint8)
UNMATCHED_COLOR = np.array([245, 60, 60], dtype=np.uint8)


def load_table(path, columns, delimiter=None, skiprows=0):
    if path is None or not path.exists() or not path.stat().st_size:
        return np.empty((0, columns))
    try:
        data = np.loadtxt(path, delimiter=delimiter, skiprows=skiprows, ndmin=2)
    except ValueError as error:
        raise ValueError(f'Cannot read {path}: {error}') from error
    return data.reshape(-1, columns) if data.size else np.empty((0, columns))


def raw_files(run):
    folder = run / 'raw' if (run / 'raw').is_dir() else run
    found = {}
    for prefix, suffix in [('f_', '.txt'), ('mp_', '.csv'), ('kp_', '.csv')]:
        paths = sorted(folder.glob(prefix + '*' + suffix))
        if len(paths) > 1:
            raise ValueError(f'Ambiguous {prefix} exports in {folder}: {paths}')
        found[prefix] = paths[0] if paths else None
    stems = {p.name[len(k):-len(p.suffix)] for k, p in found.items() if p is not None}
    if len(stems) > 1:
        raise ValueError(f'Trajectory, map, and keypoint exports have different run names: {stems}')
    return found


def map_reveal_times(cloud, point_file):
    """Use surviving observations, not a reference KF changed by culling.

    The final export does not retain culled observations or historical BA
    positions. These times therefore describe earliest *surviving* evidence,
    rather than claiming to reconstruct the exact runtime landmark birth.
    """
    result = cloud.copy()
    observation_file = point_file.with_name('ml_' + point_file.stem[3:] + '_kfpts.csv')
    report = {'source': str(observation_file), 'points_with_observation': 0,
              'points_without_observation': len(cloud), 'points_revealed_earlier': 0,
              'maximum_reveal_advance_s': 0.}
    if not observation_file.is_file() or not len(cloud):
        report['method'] = 'Current reference-keyframe timestamp fallback; observation export unavailable'
        return result, report
    indices = {int(row[5]): index for index, row in enumerate(cloud)}
    earliest = np.full(len(cloud), np.inf)
    with observation_file.open() as handle:
        for row in csv.DictReader(handle):
            index = indices.get(int(row['pointid']))
            if index is not None:
                stamp = float(row['t'])
                if np.isfinite(stamp) and stamp < earliest[index]:
                    earliest[index] = stamp
    known = np.isfinite(earliest)
    advance = cloud[known, 0] - earliest[known]
    result[known, 0] = earliest[known]
    report.update({'method': 'Earliest surviving keyframe observation; current-reference fallback for missing observations',
                   'points_with_observation': int(known.sum()),
                   'points_without_observation': int((~known).sum()),
                   'points_revealed_earlier': int((advance > 1e-6).sum()),
                   'maximum_reveal_advance_s': float(max(0., advance.max())) if len(advance) else 0.})
    return result, report


def load_calibration(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f'Cannot open calibration {path}')
    def matrix(key):
        value = fs.getNode(key).mat()
        if value is None or value.shape != (4, 4):
            raise ValueError(f'Missing 4x4 calibration {key}')
        return value.astype(np.float64)
    Tbc = matrix('IMU.T_b_c1')
    T01 = matrix('Rig.T_c0_c1')
    names = ['fx', 'fy', 'cx', 'cy'] + [f'k{i}' for i in range(1, 7)] + ['p1', 'p2', 's1', 's2', 's3', 's4']
    cameras = []
    for cam in (0, 1):
        cameras.append({'parameters': np.array([fs.getNode(f'Camera{cam+1}.{name}').real() for name in names]),
                        'width': int(fs.getNode('Camera.width').real()),
                        'height': int(fs.getNode('Camera.height').real()),
                        'mask': fs.getNode(f'Camera{cam+1}.mask').string()})
    fs.release()
    for T in (Tbc, T01):
        U, _, Vt = np.linalg.svd(T[:3, :3])
        T[:3, :3] = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    return Tbc, T01, cameras


def apply_committed_radial_calibration(run, cameras, output_dir=None):
    """Use the recorded final SLAM model for final-map reprojection diagnostics.

    Input calibration and official scoring remain unchanged. Repeated complete
    two-camera commits are retained in provenance; final optimized landmarks
    are projected with the final committed model, not historical intrinsics.
    """
    log = run / 'run.log'
    commits = []
    pending = {}
    originals = [camera['parameters'].copy() for camera in cameras]
    current = [value.copy() for value in originals]
    pattern = re.compile(r'\[(RADIAL|CALIBRATION)-COMMIT\] t=([0-9.eE+\-]+) map=(\d+) cam=([01]) params=([0-9.eE+,\-]+)')
    if log.exists():
        with log.open(errors='replace') as handle:
            for line in handle:
                for match in pattern.finditer(line):
                    kind, stamp, map_id, camera, values = match.groups()
                    camera = int(camera)
                    key = (kind, float(stamp), int(map_id))
                    if pending and next(iter(pending.values()))['key'] != key:
                        raise ValueError('Incomplete or inconsistent two-camera calibration commit')
                    if camera in pending:
                        raise ValueError('Repeated camera within one calibration commit')
                    parameters = np.array([float(x) for x in values.split(',')])
                    if parameters.shape != (16,) or not np.isfinite(parameters).all() or np.any(parameters[:2] <= 0):
                        raise ValueError('Invalid committed Fisheye624 parameters')
                    fixed = np.ones(16, dtype=bool); fixed[4:6] = False
                    if kind == 'RADIAL' and not np.allclose(parameters[fixed], current[camera][fixed], rtol=1e-6, atol=2e-6):
                        raise ValueError('Recorded radial commit changed another intrinsic parameter')
                    pending[camera] = {'key': key, 'parameters': parameters.tolist()}
                    if set(pending) == {0, 1}:
                        if commits and key[1] < commits[-1]['timestamp_s']:
                            raise ValueError('Calibration commits run backwards in time')
                        commits.append({'kind': kind, 'timestamp_s': key[1], 'map_id': key[2],
                                        'cameras': {cam: {'parameters': pending[cam]['parameters']} for cam in (0, 1)}})
                        current = [np.array(pending[cam]['parameters']) for cam in (0, 1)]
                        pending = {}
    if pending:
        raise ValueError('Incomplete two-camera calibration commit at end of log')
    if not commits:
        return {'source': 'fixed input calibration', 'committed': False}
    records = {cam: {'timestamp_s': commits[-1]['timestamp_s'], 'map_id': commits[-1]['map_id'],
                     'parameters': current[cam].tolist(), 'original_parameters': originals[cam].tolist()} for cam in (0, 1)}
    for cam in (0, 1):
        cameras[cam]['parameters'] = current[cam]
    result = {'source': 'accepted camera calibration commits in run.log', 'committed': True,
              'scope': 'Final optimized map reprojections; original image feature coordinates unchanged',
              'official_scoring_calibration_changed': False, 'cameras': records,
              'commit_count': len(commits), 'history': commits}
    if output_dir is not None:
        (output_dir / 'config/estimated_intrinsics.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def evaluation_alignment(path, run, times, positions, rotations, gt):
    """Read the exact saved evaluator transform, checking its source trajectory."""
    saved = json.loads(path.read_text())
    if relocated_path(saved['run']).resolve() != run.resolve():
        raise ValueError('Evaluation score belongs to a different source run')
    if saved['estimated_poses'] != len(times):
        raise ValueError('Evaluation and visualization trajectory counts disagree')
    scored_camera = load_table(path.parent / 'estimated_cam0_ns.txt', 8)
    if len(scored_camera) != len(times) or not np.allclose(scored_camera[:, 0]*1e-9, times, atol=1e-7, rtol=0):
        raise ValueError('Evaluation and visualization timestamps disagree')
    # Legacy f_ exports and atlas scoring use different float SE3 compositions;
    # permit sub-millimetre serialization differences, then display the exact
    # scored camera poses. This does not permit a different map or convention.
    camera_difference = float(np.max(np.abs(scored_camera[:, 1:4]-positions)))
    rotation_difference = float(np.max(np.abs(Rotation.from_quat(scored_camera[:, 4:8]).as_matrix()-rotations)))
    if camera_difference > 1e-3 or rotation_difference > 2e-5:
        raise ValueError('Evaluation and visualization camera geometry disagree')
    sim3 = saved['CP_sim3']
    scale = float(sim3['scale'])
    quaternion = np.asarray(sim3['rotation_xyzw'], dtype=float)
    translation = np.asarray(sim3['translation'], dtype=float)
    if (not np.isfinite(scale) or scale <= 0 or quaternion.shape != (4,) or translation.shape != (3,)
            or not np.isfinite(quaternion).all() or not np.isfinite(translation).all()
            or abs(np.linalg.norm(quaternion)-1) > 1e-6):
        raise ValueError('Invalid saved evaluation Sim3')
    rotation = Rotation.from_quat(quaternion).as_matrix()
    aligned = scale * (scored_camera[:, 1:4] @ rotation.T) + translation
    index, error = nearest_indices(gt[:, 0], times) if len(gt) else (np.empty(0, dtype=int), np.empty(0))
    keep = error <= .001
    residual = aligned[keep]-gt[index[keep], 1:4] if len(gt) else np.empty((0, 3))
    return scale, rotation, translation, {
        'alignment': 'Saved official control-point Sim3; no additional fit',
        'alignment_mode': 'evaluation',
        'evaluation_score_source': str(path.resolve()),
        'evaluation_score_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'evaluation_source_camera_max_difference_m': camera_difference,
        'evaluation_poses_source': str((path.parent / 'estimated_cam0_ns.txt').resolve()),
        'evaluation_source_rotation_max_difference': rotation_difference,
        'ate_rmse_m': float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))) if len(residual) else None,
        'gt_associations': int(keep.sum()), 'gt_unique_associations': int(len(np.unique(index[keep]))),
        'gt_in_common_frame': True, 'challenge_score': saved['Score2D'],
        'challenge_score_note': 'Saved local official Score2D; raw submission is unchanged. The evaluator, not SLAM, applies this GT-derived Sim3.',
        'evaluation_scores': saved,
    }, scored_camera


def selected_evaluation_geometry(run, score_path, raw):
    """Use explicit atlas frame labels when multiple map gauges survive.

    Legacy f_/mp_ select most keyframes; official local scoring selects most
    poses. Those policies can select different maps. Never infer a shared
    transform between them or apply one map's CP Sim3 to another map's points.
    """
    saved = json.loads(score_path.read_text())
    selection = saved.get('trajectory_selection', {})
    if int(selection.get('atlas_map_epochs', 1)) <= 1:
        return None
    frame = selection['selected_coordinate_frame']
    trajectories = sorted(run.glob('atlas_*_trajectory.csv'))
    points = sorted(run.glob('atlas_*_points.csv'))
    if len(trajectories) != 1 or len(points) != 1:
        raise ValueError('Multi-map evaluation requires explicitly framed atlas trajectory and point exports')
    body, excluded_stamps = [], set()
    selected_key = (str(selection['selected_map_id']), str(selection['selected_map_init_kf_id']))
    with trajectories[0].open() as handle:
        for row in csv.DictReader(handle):
            stamp = round(float(row['t_s'])*1e9)
            if row['coordinate_frame'] != frame:
                excluded_stamps.add(stamp)
                continue
            if (row['map_id'], row['map_init_kf_id']) != selected_key:
                raise ValueError('Selected atlas map identity disagrees with its coordinate frame')
            body.append([stamp, *[float(row[key]) for key in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')]])
    body = np.asarray(body, dtype=float).reshape(-1, 8)
    if len(body) != saved['estimated_poses']:
        raise ValueError('Selected atlas pose count differs from the scored map')
    body = body[np.argsort(body[:, 0], kind='stable')]
    cloud = []
    with points[0].open() as handle:
        for row in csv.DictReader(handle):
            if row['coordinate_frame'] != frame:
                continue
            if (row['map_id'], row['map_init_kf_id']) != selected_key:
                raise ValueError('Selected atlas landmark identity disagrees with its coordinate frame')
            cloud.append([float(row['t_s']), *[float(row[key]) for key in ('x', 'y', 'z')], int(row['cam']), int(row['point_id'])])
    cloud = np.asarray(cloud, dtype=float).reshape(-1, 6)
    if len(cloud) and len(np.unique(cloud[:, 5])) != len(cloud):
        raise ValueError('Duplicate landmark identity within selected atlas map')
    reveal = {'method': 'Selected atlas point reference-keyframe timestamps; no historical creation times supplied',
              'points_with_observation': 0, 'points_without_observation': len(cloud)}
    # Reuse legacy observation timings only when its landmark set demonstrably
    # belongs to the selected frame. Coordinates always come from labeled atlas.
    legacy = load_table(raw['mp_'], 6, delimiter=',', skiprows=1)
    if raw['mp_'] is not None and set(legacy[:, 5].astype(int)) == set(cloud[:, 5].astype(int)):
        cloud, reveal = map_reveal_times(cloud, raw['mp_'])
    report = {'selected_coordinate_frame': frame, 'selected_map_id': selected_key[0],
              'selected_map_init_kf_id': selected_key[1], 'atlas_map_epochs': selection['atlas_map_epochs'],
              'selected_pose_count': len(body), 'omitted_other_map_poses': len(excluded_stamps),
              'landmark_source': str(points[0].resolve()), 'selected_landmarks': len(cloud),
              'pose_source': str(trajectories[0].resolve()), 'map_reveal_timing': reveal,
              'cross_map_alignment_performed': False,
              'missing_interval_policy': 'Only the scored map has 3D geometry. Other maps are omitted; both native camera streams continue.'}
    return body, cloud, excluded_stamps, report


def nearest_indices(times, query):
    if not len(times):
        return np.zeros(len(query), dtype=int), np.full(len(query), np.inf)
    following = np.searchsorted(times, query).clip(0, len(times)-1)
    previous = (following-1).clip(0, len(times)-1)
    index = np.where(abs(times[previous]-query) < abs(times[following]-query), previous, following)
    return index, abs(times[index]-query)


def body_to_cam0(raw, Tbc):
    if not len(raw):
        return np.empty(0), np.empty((0, 3)), np.empty((0, 3, 3)), np.empty((0, 3, 3))
    times = raw[:, 0] * 1e-9
    if np.any(np.diff(times) <= 0):
        raise ValueError('Exported trajectory timestamps are not strictly increasing')
    Rwb = Rotation.from_quat(raw[:, 4:8]).as_matrix()
    p0 = raw[:, 1:4] + np.einsum('nij,j->ni', Rwb, Tbc[:3, 3])
    R0 = Rwb @ Tbc[:3, :3]
    return times, p0, R0, Rwb


def load_imu(dataset):
    path = dataset / 'mav0/imu0/data.csv'
    if not path.exists():
        path = dataset / 'imu0/data.csv'
    data = load_table(path if path.exists() else None, 7, delimiter=',', skiprows=1)
    if len(data):
        data[:, 0] *= 1e-9
    return data


def acceleration_near(imu, t):
    if not len(imu):
        return None
    lo, hi = np.searchsorted(imu[:, 0], [t-.25, t+.25])
    if hi-lo < 5:
        index = int(np.argmin(abs(imu[:, 0]-t)))
        lo, hi = max(0, index-100), min(len(imu), index+101)
    force = np.mean(imu[lo:hi, 4:7], axis=0)
    return force / np.linalg.norm(force) if np.linalg.norm(force) > 1e-8 else None


def rotate_vector_to_z(up):
    up = up / np.linalg.norm(up)
    z = np.array([0., 0., 1.])
    cross = np.cross(up, z)
    norm = np.linalg.norm(cross)
    if norm < 1e-10:
        return np.eye(3) if up[2] > 0 else Rotation.from_rotvec([np.pi, 0, 0]).as_matrix()
    return Rotation.from_rotvec(cross/norm * np.arctan2(norm, np.dot(up, z))).as_matrix()


def common_alignment(times, positions, rotations, Rwb, gt, imu):
    if len(gt) and len(times):
        index, error = nearest_indices(gt[:, 0], times)
        keep = error <= .02
        if keep.sum() >= 3:
            estimated, truth = positions[keep], gt[index[keep], 1:4]
            me, mg = estimated.mean(0), truth.mean(0)
            U, _, Vt = np.linalg.svd((truth-mg).T @ (estimated-me))
            rotation = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
            translation = mg-rotation @ me
            residual = estimated @ rotation.T + translation-truth
            return rotation, translation, {'alignment': 'SE3 position fit to camera0 GT; scale fixed at 1',
                    'ate_rmse_m': float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
                    'gt_associations': int(keep.sum()), 'gt_unique_associations': int(len(np.unique(index[keep]))), 'gt_in_common_frame': True}
        if keep.any():
            first = int(np.flatnonzero(keep)[0])
            truth = gt[index[first]]
            rotation = Rotation.from_quat(truth[4:8]).as_matrix() @ rotations[first].T
            translation = truth[1:4]-rotation @ positions[first]
            return rotation, translation, {'alignment': 'One common SE3 from the first associated camera pose; fewer than three positions for ATE',
                    'ate_rmse_m': None, 'gt_associations': int(keep.sum()),
                    'gt_unique_associations': int(len(np.unique(index[keep]))), 'gt_in_common_frame': True}
    up = acceleration_near(imu, times[0]) if len(times) else None
    rotation = rotate_vector_to_z(Rwb[0] @ up) if up is not None else np.eye(3)
    translation = -rotation @ positions[0] if len(positions) else np.zeros(3)
    return rotation, translation, {'alignment': 'One common gravity-based SE3; no usable GT fit' if up is not None else 'Common origin translation; gravity orientation unavailable',
                                  'ate_rmse_m': None, 'gt_associations': 0, 'gt_unique_associations': 0, 'gt_in_common_frame': False}


def display_turns(imu, t, Tbc, T01):
    up = acceleration_near(imu, t)
    turns, notes = [], []
    for cam, T in enumerate((Tbc, Tbc @ T01)):
        if up is None:
            turns.append(0); notes.append(f'cam{cam}: raw orientation, no IMU estimate'); continue
        uv = (T[:3, :3].T @ up)[:2]
        if np.linalg.norm(uv) < .2:
            turn = 0
        else:
            variants = [uv, np.array([-uv[1], uv[0]]), -uv, np.array([uv[1], -uv[0]])]
            turn = int(np.argmax([-v[1] for v in variants]))
        turns.append(turn)
        notes.append(f'cam{cam}: display rotated {90*turn} degrees clockwise; projected accelerometer-up={uv.tolist()}')
    return turns, notes


def rotate_pixels(points, width, height, turns):
    p = np.asarray(points, dtype=float).reshape(-1, 2).copy()
    for _ in range(turns):
        p = np.column_stack((height-1-p[:, 1], p[:, 0]))
        width, height = height, width
    return p


def project_fisheye624(points, parameters):
    x, y, z = points.T
    radius = np.hypot(x, y)
    theta = np.arctan2(radius, z)
    polynomial = np.ones(len(points))
    for i, coefficient in enumerate(parameters[4:10]):
        polynomial += coefficient * theta**(2*i+2)
    factor = np.divide(theta*polynomial, radius, out=np.zeros(len(points)), where=radius > 1e-12)
    a, b = x*factor, y*factor
    rho = a*a+b*b
    p0, p1, s0, s1, s2, s3 = parameters[10:]
    u = a+p0*(rho+2*a*a)+2*p1*a*b+s0*rho+s1*rho*rho
    v = b+p1*(rho+2*b*b)+2*p0*a*b+s2*rho+s3*rho*rho
    return np.column_stack((parameters[0]*u+parameters[2], parameters[1]*v+parameters[3]))


def keypoint_groups(path):
    if path is None:
        return
    with path.open() as f:
        reader = csv.reader(f)
        next(reader, None)
        timestamp, group = None, [[], []]
        for row in reader:
            if len(row) < 6:
                continue
            current = round(float(row[0])*1e9)
            if timestamp is not None and current < timestamp:
                raise ValueError('Keypoint dump runs backwards in time; regenerate chronological input first')
            if current != timestamp:
                if timestamp is not None:
                    yield timestamp, group
                timestamp, group = current, [[], []]
            cam = int(row[1])
            if cam in (0, 1):
                group[cam].append((int(row[2]), float(row[3]), float(row[4]), int(row[5])))
        if timestamp is not None:
            yield timestamp, group


def image_path(dataset, cam, stamp):
    folders = [dataset / f'mav0/cam{cam}/data', dataset / f'cam{cam}/data']
    for folder in folders:
        for suffix in ('.png', '.jpg', '.jpeg'):
            path = folder / f'{stamp}{suffix}'
            if path.exists():
                return path
    return None


def load_display_image(path, camera, turns, config):
    gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return None
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    if camera['mask']:
        mask_path = Path(camera['mask'])
        if not mask_path.is_absolute():
            mask_path = config.parent / mask_path
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None or mask.shape != gray.shape:
            raise ValueError(f'Configured mask unavailable or wrong resolution: {mask_path}')
        masked = mask > 0
        image[masked] = np.clip(image[masked]*.38 + [64, 16, 16], 0, 255).astype(np.uint8)
    return np.ascontiguousarray(np.rot90(image, -turns))


def reprojection_stats(observations, cam, pose, points, point_index, T01, cameras):
    if pose is None:
        return float('nan'), 0
    R0, p0 = pose
    Rc, pc = (R0, p0) if cam == 0 else (R0 @ T01[:3, :3], p0+R0 @ T01[:3, 3])
    selected = [(point_index[r[0]], r[1:3]) for r in observations if r[3] and r[0] in point_index]
    if not selected:
        return float('nan'), 0
    ids = np.array([r[0] for r in selected])
    pixels = np.array([r[1] for r in selected])
    Xc = (points[ids]-pc) @ Rc
    prediction = project_fisheye624(Xc, cameras[cam]['parameters'])
    valid = (Xc[:, 2] > 0) & np.isfinite(prediction).all(1)
    if not valid.any():
        return float('nan'), 0
    residual = prediction[valid]-pixels[valid]
    return float(np.sqrt(np.mean(np.sum(residual*residual, axis=1)))), int(valid.sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=relocated_path, required=True)
    parser.add_argument('--output-dir', type=relocated_path, help='New visualization folder; defaults to the source run folder')
    parser.add_argument('--mapped-segment', action='store_true', help='Include all camera frames within the saved map trajectory interval')
    parser.add_argument('--dataset', type=relocated_path, required=True)
    parser.add_argument('--timestamps', type=relocated_path, required=True)
    parser.add_argument('--config', type=relocated_path, required=True)
    parser.add_argument('--gt', type=relocated_path, help='Camera0 T_world_cam0 TUM: seconds or nanoseconds, xyzw')
    parser.add_argument('--evaluation-score', type=relocated_path, help='Saved local scores.json: display its exact CP Sim3 instead of the default metric SE3; requires a new --output-dir')
    parser.add_argument('--no-images', action='store_true', help='Debug only: explicitly omit video/stills')
    args = parser.parse_args()
    if args.evaluation_score and not args.output_dir:
        parser.error('--evaluation-score requires a new --output-dir to preserve the metric recording')
    run = args.run_dir.resolve()
    out = args.output_dir.resolve() if args.output_dir else run
    if args.output_dir and out.exists():
        raise FileExistsError(f'Choose a new visualization output folder: {out}')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'config').mkdir(exist_ok=True); (out / 'viz').mkdir(exist_ok=True)
    config_copy = out / 'config' / args.config.name
    if args.config.resolve() != config_copy.resolve():
        shutil.copy2(args.config, config_copy)
    raw = raw_files(run)
    timestamps = np.array([int(line.strip().split()[0]) for line in args.timestamps.read_text().splitlines() if line.strip() and not line.startswith('#')], dtype=np.int64)
    if not len(timestamps) or np.any(np.diff(timestamps) <= 0):
        raise ValueError('Input timestamp list must be nonempty and strictly chronological')
    source_frames = len(timestamps)
    source_duration = float((timestamps[-1]-timestamps[0])*1e-9)
    # Keep the physical display convention stable when selecting a later map.
    # A different head pose at a crop boundary is not a different sensor mount.
    # Estimate orientation at the source sequence start, before filtering time.
    display_reference_time = float(timestamps[0]*1e-9)
    Tbc, T01, cameras = load_calibration(args.config)
    estimated_intrinsics = apply_committed_radial_calibration(run, cameras, out)
    imu_dataset = run / 'input/euroc' if (run / 'input/euroc/mav0/imu0/data.csv').exists() else args.dataset
    imu = load_imu(imu_dataset)
    selected_geometry = selected_evaluation_geometry(run, args.evaluation_score, raw) if args.evaluation_score else None
    source_body = selected_geometry[0] if selected_geometry is not None else load_table(raw['f_'], 8)
    times, positions, rotations, Rwb = body_to_cam0(source_body, Tbc)
    if args.mapped_segment:
        if not len(times):
            raise ValueError('No exported trajectory is available for --mapped-segment')
        timestamps = timestamps[(timestamps*1e-9 >= times[0]-1e-6) & (timestamps*1e-9 <= times[-1]+1e-6)]
    if not len(timestamps):
        raise ValueError('The selected recording interval has no camera frames')
    frame_times = timestamps * 1e-9
    dt = float(np.median(np.diff(frame_times))) if len(frame_times) > 1 else .05
    if args.output_dir:
        (out / 'config/timestamps_ns.txt').write_text(''.join(f'{int(t)}\n' for t in timestamps))
    gt = load_table(args.gt, 8)
    if len(gt):
        if np.median(gt[:, 0]) > 1e10:
            gt[:, 0] *= 1e-9
        gt = gt[np.argsort(gt[:, 0], kind='stable')]
        gt = gt[(gt[:, 0] >= frame_times[0]-.02) & (gt[:, 0] <= frame_times[-1]+.02)]
    if args.evaluation_score:
        alignment_scale, alignment_R, alignment_t, score, scored_camera = evaluation_alignment(args.evaluation_score, run, times, positions, rotations, gt)
        positions = scored_camera[:, 1:4]
        rotations = Rotation.from_quat(scored_camera[:, 4:8]).as_matrix()
    else:
        alignment_R, alignment_t, score = common_alignment(times, positions, rotations, Rwb, gt, imu)
        alignment_scale = 1.
        score.update({'alignment_mode': 'metric', 'challenge_score': None,
                      'challenge_score_note': 'Metric visualization; no official score transform applied.'})
    positions = alignment_scale * (positions @ alignment_R.T) + alignment_t
    rotations = alignment_R @ rotations
    # Both camera centers and all map points must receive the same Sim3.
    # Scale only the displayed relative translation, never the source rig YAML.
    display_T01 = T01.copy()
    display_T01[:3, 3] *= alignment_scale
    quaternions = Rotation.from_matrix(rotations).as_quat() if len(rotations) else np.empty((0, 4))
    np.savetxt(out / 'traj.csv', np.column_stack((times, positions, quaternions)), delimiter=',', header='t,px,py,pz,qx,qy,qz,qw', comments='', fmt='%.9f')
    cloud = selected_geometry[1] if selected_geometry is not None else load_table(raw['mp_'], 6, delimiter=',', skiprows=1)
    if selected_geometry is not None:
        reveal_report = selected_geometry[3]['map_reveal_timing']
        score['geometry_selection'] = selected_geometry[3]
    elif raw['mp_'] is not None:
        cloud, reveal_report = map_reveal_times(cloud, raw['mp_'])
    else:
        reveal_report = {'method': 'No map point export', 'points_with_observation': 0}
    cloud = cloud[np.argsort(cloud[:, 0], kind='stable')] if len(cloud) else cloud
    points = alignment_scale * (cloud[:, 1:4] @ alignment_R.T) + alignment_t
    np.savetxt(out / 'points.csv', points, delimiter=',', header='x,y,z', comments='', fmt='%.9f')
    point_index = {int(row[5]): index for index, row in enumerate(cloud)}
    scale = float(np.linalg.norm(np.ptp(positions, axis=0))) if len(positions) else 0.
    scale_fallback = scale < 1e-8
    if scale_fallback:
        scale = 1.
    center = positions.mean(0) if len(positions) else np.zeros(3)
    finite_points = np.isfinite(points).all(1)
    keep = finite_points.copy()
    if len(points) and len(positions):
        finite = np.flatnonzero(keep)
        keep[finite] = cKDTree(positions).query(points[finite])[0] <= 1.5*scale
    dropped = int((~keep).sum())
    display_filter = {'point_count_cap': None, 'downsampling': False,
                      'policy': 'Keep finite points within 1.5 trajectory diagonals of a trajectory position; reveal each once at its earliest retained observation.',
                      'maximum_distance_from_trajectory_m': 1.5*scale,
                      'source_points': len(points),
                      'nonfinite_points_omitted': int((~finite_points).sum()),
                      'distant_points_omitted': int((finite_points & ~keep).sum()),
                      'eligible_points': int(keep.sum()),
                      'full_points_csv_preserved': True}
    turns, orientation_notes = display_turns(imu, display_reference_time, Tbc, T01)
    for note in orientation_notes:
        print(note, flush=True)
    print(f'Cloud display: {int(keep.sum())}/{len(points)} points; dropped {dropped} beyond 1.5*S. Full points.csv retained.', flush=True)
    mask_note = 'No camera masks configured.' if not any(c['mask'] for c in cameras) else 'Configured masks shaded in the image panes.'
    frame_pose, frame_error = nearest_indices(times, frame_times)
    pose_available = frame_error <= max(1e-6, .25*dt)
    score.update({'input_frames': len(timestamps), 'exported_cam0_poses': len(times),
                  'source_run': str(run), 'source_input_frames': source_frames, 'source_duration_s': source_duration,
                  'recording_interval_s': [float(frame_times[0]), float(frame_times[-1])],
                  'selection': 'Saved map interval, full frame rate' if args.mapped_segment else 'Full requested input',
                  'keypoint_colors': 'Green: mapped; red: detected but unmapped. Green fading trails follow persistent map-point IDs.',
                  'map_growth': 'Final optimized landmarks revealed at their earliest surviving keyframe observation; historical BA positions and culled observations are not saved.',
                  'map_reveal_timing': reveal_report,
                  'frame_coverage': float(pose_available.mean()), 'gt_coverage': score['gt_unique_associations']/len(gt) if len(gt) else None,
                  'map_points_full': len(points), 'map_points_displayed': int(keep.sum()), 'map_points_display_dropped': dropped,
                  'map_display_filter': display_filter,
                  'map_landmark_ids_logged': True, 'trajectory_source_timestamps_logged': True,
                  'trajectory_diagonal_m': scale, 'stationary_visual_scale_fallback_1m': scale_fallback,
                  'alignment_rotation': alignment_R.tolist(), 'alignment_translation': alignment_t.tolist(), 'alignment_scale': alignment_scale,
                  'source_rig_baseline_m': float(np.linalg.norm(T01[:3, 3])),
                  'display_rig_baseline_m': float(np.linalg.norm(display_T01[:3, 3])),
                  'display_orientation': orientation_notes, 'display_orientation_reference_s': display_reference_time,
                  'display_clockwise_degrees': [90*turn for turn in turns],
                  'mask_note': mask_note, 'images_enabled': not args.no_images,
                  'export_scope': 'Only the scored coordinate frame is rendered; other maps have no shared 3D gauge. Runtime image keypoints may belong to other maps.' if selected_geometry is not None else 'ORB exports the largest map. The keypoint dump can also contain observations from other maps.',
                  'reprojection_note': 'Computed from available final-map point IDs and exported frame poses using Fisheye624. Missing coverage is NaN, never zero.',
                  'estimated_intrinsics': estimated_intrinsics,
                  'frusta_note': 'Equivalent-pinhole frusta from fx,fy,cx,cy; not the true fisheye field of view.'})
    # Connect the file sink BEFORE logging any recording data.
    rr.init('lamaria_evaluation' if args.evaluation_score else 'lamaria_slam', spawn=False)
    rr.save(str(out / 'run.rrd'))
    image_views = [rrb.Horizontal(rrb.Spatial2DView(origin='/cam0', name='Cam0 — green mapped / red unmapped'),
                                  rrb.Spatial2DView(origin='/cam1', name='Cam1 — green mapped / red unmapped')),
                   rrb.TimeSeriesView(origin='/counts', name='Mapped features by camera'),
                   rrb.TimeSeriesView(origin='/reprojection', name='Final-map reprojection RMS')]
    if selected_geometry is not None:
        image_views.append(rrb.TextDocumentView(origin='/evaluation_status', name='Evaluated-map coverage'))
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(origin='/world', name='Evaluation Sim3 — map + trajectory + GT' if args.evaluation_score else 'Growing map + trajectory + GT', contents='/world/**',
            eye_controls=rrb.EyeControls3D(position=(center+scale*np.array([.85, -.85, .65])).tolist(), look_target=center.tolist(), eye_up=[0, 0, 1]),
            line_grid=rrb.LineGrid3D(visible=True, spacing=1., color=[70, 70, 78, 140])),
        rrb.Vertical(*image_views, row_shares=[5, 1, 1, 1] if selected_geometry is not None else [5, 1, 1]), column_shares=[3, 2]),
        rrb.TimePanel(timeline='t', play_state='Following', playback_speed=1., loop_mode='Selection',
            time_selection=rr.datatypes.AbsoluteTimeRange(
                rr.datatypes.TimeInt(seconds=float(frame_times[0])), rr.datatypes.TimeInt(seconds=float(frame_times[-1]))))),
        make_active=True, make_default=True)
    rr.set_time('t', timestamp=float(frame_times[0]))
    rr.log('/world', rr.ViewCoordinates.RIGHT_HAND_Z_UP)
    rr.log('/metadata', rr.TextDocument(json.dumps(score, indent=2)))
    geometry_frame = score.get('evaluation_scores', {}).get('trajectory_selection', {}).get('selected_coordinate_frame', 'source_export')
    if len(gt) and score['gt_in_common_frame']:
        rr.log('/world/gt', rr.LineStrips3D([gt[:, 1:4]], colors=[[115, 200, 115]], radii=.001*scale))
    for cam in (0, 1):
        rr.log(f'/counts/cam{cam}', rr.SeriesLines(colors=[COLORS[cam]], names=[f'cam{cam} mapped']))
        rr.log(f'/reprojection/cam{cam}', rr.SeriesLines(colors=[COLORS[cam]], names=[f'cam{cam} RMS px']))
    groups = iter(keypoint_groups(raw['kp_']))
    pending = next(groups, None)
    cloud_cursor = 0; cloud_batch = 0; cloud_points_logged = 0
    histories = [defaultdict(lambda: deque(maxlen=12)), defaultdict(lambda: deque(maxlen=12))]
    previous_images = [None, None]; previous_locations = [{}, {}]
    still_indices = {len(timestamps)//4, len(timestamps)//2, 3*len(timestamps)//4}
    trajectory_chunk = 0; trajectory_segment = []; trajectory_stamps = []; last_pose_index = -1
    missing_images = 0; reprojection_frames = 0; previous_geometry_status = None
    with (out / 'stats.csv').open('w') as stats_file:
        writer = csv.writer(stats_file)
        writer.writerow(['t', 'cam', 'features', 'mapped', 'reprojection_rms_px', 'reprojected_points', 'pose_available', 'keypoint_dump_available'])
        for frame_number, (stamp, t) in enumerate(zip(timestamps, frame_times)):
            while pending is not None and pending[0] < stamp-1000:
                pending = next(groups, None)
            observed = pending is not None and abs(pending[0]-stamp) <= 1000
            observations = pending[1] if observed else [[], []]
            if observed:
                pending = next(groups, None)
            rr.set_time('t', timestamp=float(t))
            if selected_geometry is not None:
                frame_name = selected_geometry[3]['selected_coordinate_frame']
                if pose_available[frame_number]:
                    geometry_status = f'Scored map: {frame_name}. Its saved CP Sim3 is applied to this map only.'
                elif int(stamp) in selected_geometry[2]:
                    geometry_status = f'This image belongs to another independent map. No scored 3D pose is available here. Only {frame_name} geometry is displayed; no stitching. Green points still show runtime image associations.'
                else:
                    geometry_status = f'No exported scored-map pose at this timestamp. Images continue; no 3D pose is filled in. Selected frame: {frame_name}.'
                if geometry_status != previous_geometry_status:
                    rr.log('/evaluation_status', rr.TextDocument(geometry_status))
                    previous_geometry_status = geometry_status
            end = int(np.searchsorted(cloud[:, 0], t+1e-6, side='right')) if len(cloud) else 0
            if end > cloud_cursor:
                selected = np.flatnonzero(keep[cloud_cursor:end])+cloud_cursor
                if len(selected):
                    colors = COLORS[np.clip(cloud[selected, 4].astype(int), 0, 2)]
                    rr.log(f'/world/map/part_{cloud_batch:06d}', rr.Points3D(points[selected], colors=colors, radii=.00025*scale),
                           rr.AnyValues(landmark_id=cloud[selected, 5].astype(np.int64), coordinate_frame=[geometry_frame]))
                    cloud_batch += 1
                    cloud_points_logged += len(selected)
                cloud_cursor = end
            pose = None
            if pose_available[frame_number]:
                pi = int(frame_pose[frame_number]); pose = (rotations[pi], positions[pi])
                if pi != last_pose_index:
                    if last_pose_index >= 0 and (times[pi]-times[last_pose_index] > 1.5*dt or len(trajectory_segment) >= 128):
                        trajectory_chunk += 1
                        trajectory_segment = [trajectory_segment[-1]] if times[pi]-times[last_pose_index] <= 1.5*dt else []
                        trajectory_stamps = [trajectory_stamps[-1]] if times[pi]-times[last_pose_index] <= 1.5*dt else []
                    trajectory_segment.append(positions[pi].tolist())
                    trajectory_stamps.append(int(round(times[pi]*1e9)))
                    rr.log(f'/world/trajectory/part_{trajectory_chunk:06d}', rr.LineStrips3D([trajectory_segment], colors=[[245, 185, 70]], radii=.0025*scale),
                           rr.AnyValues(source_timestamps_ns=np.array(trajectory_stamps, dtype=np.int64), coordinate_frame=[geometry_frame]))
                    last_pose_index = pi
                for cam, Tc in enumerate((np.eye(4), display_T01)):
                    Rcam = pose[0] @ Tc[:3, :3]; pcam = pose[1]+pose[0] @ Tc[:3, 3]
                    par = cameras[cam]['parameters']
                    rr.log(f'/world/cam{cam}', rr.Transform3D(translation=pcam, mat3x3=Rcam),
                           rr.Pinhole(focal_length=par[:2], principal_point=par[2:4], resolution=[cameras[cam]['width'], cameras[cam]['height']],
                                      camera_xyz=rr.ViewCoordinates.RDF, image_plane_distance=.005*scale, color=COLORS[cam]),
                           rr.AnyValues(source_timestamp_ns=[int(round(times[pi]*1e9))], coordinate_frame=[geometry_frame]))
            else:
                for cam in (0, 1):
                    rr.log(f'/world/cam{cam}', rr.Clear(recursive=True))
            for cam in (0, 1):
                rows = observations[cam]
                mapped = sum(bool(r[3]) for r in rows)
                rms, projected_count = reprojection_stats(rows, cam, pose, points, point_index, display_T01, cameras)
                reprojection_frames += int(projected_count > 0)
                writer.writerow([f'{t:.9f}', cam, len(rows), mapped, rms, projected_count, int(pose is not None), int(observed)])
                rr.log(f'/counts/cam{cam}', rr.Scalars(mapped))
                rr.log(f'/reprojection/cam{cam}', rr.Scalars(rms))
                raw_pixels = np.array([r[1:3] for r in rows], dtype=float).reshape(-1, 2)
                pixels = rotate_pixels(raw_pixels, cameras[cam]['width'], cameras[cam]['height'], turns[cam])
                colors = np.array([MAPPED_COLOR if r[3] else UNMATCHED_COLOR for r in rows], dtype=np.uint8).reshape(-1, 3)
                rr.log(f'/cam{cam}/image/keypoints', rr.Points2D(pixels, colors=colors, radii=1.5, draw_order=30.))
                segments, segment_colors = [], []
                locations = {}
                for row, pixel in zip(rows, pixels):
                    if row[0] < 0 or not row[3]:
                        continue
                    locations[row[0]] = pixel
                    history = histories[cam][row[0]]
                    if history and t-history[-1][0] > 1.5*dt:
                        history.clear()
                    history.append((t, pixel))
                    for h in range(1, len(history)):
                        segments.append([history[h-1][1], history[h][1]])
                        fade = 1-h/len(history)
                        segment_colors.append((MAPPED_COLOR*(1-.7*fade)+255*.7*fade).astype(np.uint8))
                rr.log(f'/cam{cam}/image/trails', rr.LineStrips2D(segments, colors=segment_colors, radii=.65, draw_order=20.))
                if frame_number % 100 == 0:
                    histories[cam] = defaultdict(lambda: deque(maxlen=12), {key: value for key, value in histories[cam].items() if value and t-value[-1][0] < 1.})
                if not args.no_images:
                    path = image_path(args.dataset, cam, stamp)
                    image = load_display_image(path, cameras[cam], turns[cam], args.config) if path else None
                    if image is None:
                        missing_images += 1
                        rr.log(f'/cam{cam}/image', rr.Clear(recursive=False))
                    else:
                        ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
                        if not ok:
                            raise RuntimeError(f'Cannot encode image {path}')
                        rr.log(f'/cam{cam}/image', rr.EncodedImage(contents=encoded.tobytes(), media_type='image/jpeg'))
                        if frame_number in still_indices and previous_images[cam] is not None:
                            pair = np.hstack([previous_images[cam], image]).copy(); width = image.shape[1]
                            common = sorted(set(locations) & set(previous_locations[cam]))
                            for identity in common[:150]:
                                a = tuple(np.rint(previous_locations[cam][identity]).astype(int)); q = locations[identity]+[width, 0]
                                cv2.line(pair, a, tuple(np.rint(q).astype(int)), tuple(int(v) for v in MAPPED_COLOR), 1, cv2.LINE_AA)
                            cv2.putText(pair, f'cam{cam}: {len(common)} persistent mapped IDs', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
                            cv2.imwrite(str(out / 'viz' / f'matches_cam{cam}_{stamp}.png'), cv2.cvtColor(pair, cv2.COLOR_RGB2BGR))
                            overlay = image.copy()
                            for pixel, color in zip(pixels, colors):
                                cv2.circle(overlay, tuple(np.rint(pixel).astype(int)), 2, tuple(int(v) for v in color), -1, cv2.LINE_AA)
                            cv2.putText(overlay, f'cam{cam}: green {mapped} mapped / red {len(rows)-mapped} unmapped', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
                            cv2.imwrite(str(out / 'viz' / f'overlay_cam{cam}_{stamp}.png'), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
                        previous_images[cam] = image; previous_locations[cam] = locations
            if frame_number % 1000 == 0:
                print(f'packaged {frame_number}/{len(timestamps)} frames', flush=True)
    score['map_display_filter'].update({'recorded_points': cloud_points_logged,
                                        'eligible_points_outside_recording_interval': int(keep.sum())-cloud_points_logged})
    score.update({'missing_images': missing_images, 'camera_frames_with_reprojection': reprojection_frames,
                  'map_points_recorded': cloud_points_logged})
    recording = rr.get_global_data_recording()
    if recording is not None:
        recording.flush(); recording.disconnect()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axis = plt.subplots(figsize=(8, 7))
    if len(gt) and score['gt_in_common_frame']: axis.plot(gt[:, 1], gt[:, 2], color='#75c875', label='GT')
    if len(positions): axis.plot(positions[:, 0], positions[:, 1], color='#cc9231', label='SLAM cam0')
    axis.set_aspect('equal', adjustable='datalim'); axis.set_xlabel('World X (m)'); axis.set_ylabel('World Y (m)')
    axis.set_title('Evaluation frame — saved control-point Sim3' if args.evaluation_score else 'Common world frame — SE3 alignment, scale unchanged')
    if (len(gt) and score['gt_in_common_frame']) or len(positions): axis.legend()
    fig.tight_layout(); fig.savefig(out / 'viz/path_topdown.png', dpi=160); plt.close(fig)
    executable = shutil.which('rerun', path=str(Path(sys.executable).parent)+os.pathsep+os.environ.get('PATH', ''))
    if executable:
        verified = subprocess.run([executable, 'rrd', 'verify', str(out / 'run.rrd')], capture_output=True, text=True)
        (out / 'run.rrd.verify.log').write_text(verified.stdout+verified.stderr)
        score['rrd_verification'] = 'passed' if verified.returncode == 0 else 'failed'
    else:
        score['rrd_verification'] = 'not run: rerun CLI unavailable'
    (out / 'score.txt').write_text('\n'.join(f'{key}: {json.dumps(value)}' for key, value in score.items())+'\n')
    (out / 'output_metadata.json').write_text(json.dumps(score, indent=2)+'\n')
    print(json.dumps(score, indent=2), flush=True)
    if score['rrd_verification'] != 'passed':
        raise RuntimeError('Rerun recording did not pass verification; inspect run.rrd.verify.log')
    if missing_images:
        raise RuntimeError(f'Output is incomplete: {missing_images} requested camera images could not be read')


if __name__ == '__main__':
    main()
