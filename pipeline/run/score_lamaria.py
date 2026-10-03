#!/usr/bin/env python3
"""Score one native LaMAria VI run with the unchanged official toolkit.

The input is the raw all-atlas export, not the display-aligned traj.csv. One
surviving map is scored: independently initialized maps are never stitched.
For a fragmented run, --map-scope largest selects the most populated pose
segment without consulting GT and counts all omitted timestamps as missing.
Outputs stay under the run directory; no estimates or toolkit files are edited.
"""
import argparse
from bisect import bisect_left
import csv
from decimal import Decimal, ROUND_HALF_UP
import hashlib
import importlib.metadata
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path
import shutil
import subprocess

import numpy as np


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1048576), b""):
            digest.update(chunk)
    return digest.hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def seconds_to_ns(value):
    return int((Decimal(value) * Decimal(1000000000)).to_integral_value(rounding=ROUND_HALF_UP))


def timestamp_list(path):
    values = [int(line.strip()) for line in path.read_text().splitlines()
              if line.strip() and not line.lstrip().startswith('#')]
    if not values or any(b <= a for a, b in zip(values, values[1:])):
        raise ValueError(f"Timestamps must be nonempty, strictly increasing integer ns: {path}")
    return values


def input_coverage(processed, source, scored_stamps):
    """Count omitted source boundary frames without changing evaluation poses.

    Both input lists use exact integer ns. The exported seconds may differ by
    a few ns from float serialization, so correspondence uses the coverage
    audit's 1 us tolerance. Processed inputs must be a contiguous source subset;
    an interior omission must not be mislabeled an unsupported boundary.
    """
    source_ids = {stamp: index for index, stamp in enumerate(source)}
    if not processed or any(stamp not in source_ids for stamp in processed):
        raise ValueError("Run input timestamps must be a nonempty subset of source timestamps")
    ids = [source_ids[stamp] for stamp in processed]
    if ids != list(range(ids[0], ids[0] + len(ids))):
        raise ValueError("Processed inputs omit interior source frames; these are not boundary exclusions")
    matched = []
    differences = []
    for stamp in scored_stamps:
        upper = bisect_left(processed, stamp)
        candidates = range(max(0, upper - 1), min(len(processed), upper + 1))
        nearest = min(candidates, key=lambda index: abs(processed[index] - stamp))
        difference = abs(processed[nearest] - stamp)
        if difference > 1000:
            raise ValueError("A scored pose does not correspond to a processed/source image timestamp")
        matched.append(nearest)
        differences.append(difference)
    if len(set(matched)) != len(matched):
        raise ValueError("Multiple scored poses correspond to the same source image timestamp")
    return {
        'native_input_frames': len(source), 'processed_input_frames': len(processed),
        'unsupported_boundary_frames': len(source) - len(processed),
        'unsupported_boundary_prefix_frames': ids[0],
        'unsupported_boundary_suffix_frames': len(source) - ids[-1] - 1,
        'missing_native_input_poses': len(source) - len(scored_stamps),
        'missing_processed_input_poses': len(processed) - len(scored_stamps),
        'pose_coverage_fraction': len(scored_stamps) / len(source),
        'processed_pose_coverage_fraction': len(scored_stamps) / len(processed),
        'pose_to_input_max_difference_ns': max(differences, default=0),
    }


def load_raw_rows(path, map_scope='single'):
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("No exported poses to score")
    groups = {}
    for row in rows:
        key = (row['map_id'], row['map_init_kf_id'], row['coordinate_frame'])
        groups.setdefault(key, []).append(row)
    total_count = len(rows)
    if len(groups) != 1 and map_scope == 'single':
        raise ValueError("Cannot score independent map frames as one trajectory; no stitching is performed")
    selected = min(groups, key=lambda key: (-len(groups[key]), key))
    rows = groups[selected]
    if not rows[0]['coordinate_frame'].endswith('_body0'):
        raise ValueError("Expected the raw world_from_imu-right body0 export")
    stamps = [seconds_to_ns(r['t_s']) for r in rows]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Export timestamps must be strictly increasing and unique")
    values = np.array([[float(r[k]) for k in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')] for r in rows])
    if not np.all(np.isfinite(values)) or np.max(abs(np.linalg.norm(values[:, 3:], axis=1) - 1)) >= 1e-5:
        raise ValueError("Export contains nonfinite poses or nonunit quaternions")
    selection = {
        'policy': map_scope, 'selected_coordinate_frame': selected[2],
        'selected_map_id': selected[0], 'selected_map_init_kf_id': selected[1],
        'selected_pose_count': len(rows), 'atlas_pose_count': total_count,
        'atlas_map_epochs': len(groups), 'omitted_other_map_poses': total_count - len(rows),
        'available_maps': [{'coordinate_frame': key[2], 'poses': len(group)} for key, group in sorted(groups.items())],
        'uses_ground_truth_for_selection': False,
    }
    return rows, stamps, selection


def associate_dense(est, gt, matching_time_indices):
    """Use official timestamp matching, preserving sensor identity and coverage.

    The current upstream associate_trajectories aliases the two return values
    when the estimate is shorter than GT. Prefiltering both to their matches
    gives evaluate_wrt_pgt equal-length inputs and avoids that branch. Neither
    missing timestamps nor new poses are manufactured. The full GT denominator
    is retained before any in-place filtering.
    """
    original_gt = list(gt.timestamps)
    if len(est) >= len(gt):
        gt_ids, est_ids = matching_time_indices(np.asarray(gt.timestamps), np.asarray(est.timestamps))
    else:
        est_ids, gt_ids = matching_time_indices(np.asarray(est.timestamps), np.asarray(gt.timestamps))
    if not est_ids:
        raise ValueError("No estimate/GT timestamp matches within the official 1 ms tolerance")
    if len(set(est_ids)) != len(est_ids) or len(set(gt_ids)) != len(gt_ids):
        raise ValueError("Ambiguous duplicate timestamp associations")
    est.filter_from_indices(est_ids)
    gt.filter_from_indices(gt_ids)
    differences = [abs(a - b) for a, b in zip(est.timestamps, gt.timestamps)]
    if max(differences) > 1000000:
        raise ValueError("Association exceeded the official 1 ms tolerance")
    matched = set(gt.timestamps)
    return len(original_gt), [t for t in original_gt if t not in matched]


def asset_files(assets, sequence):
    """Support the local flat assets and the official downloader layout."""
    candidates = {
        'cp': [assets/'gt_sparse.json', assets/'ground_truth/control_points'/f'{sequence}.json'],
        'gt': [assets/'gt_dense.txt', assets/'ground_truth/pGT'/f'{sequence}.txt'],
        'calib': [assets/'aria_calib.json', assets/'aria_calibrations'/f'{sequence}.json'],
    }
    result = {}
    for name, paths in candidates.items():
        existing = [p for p in paths if p.is_file()]
        if len(existing) != 1:
            raise ValueError(f"Require exactly one {name} asset; checked {paths}")
        result[name] = existing[0]
    return result


def score(args):
    run = args.run.resolve(strict=True)
    assets = args.assets.resolve(strict=True)
    toolkit = args.toolkit.resolve(strict=True)
    out = (args.out or run/'lamaria_score').resolve()
    if not out.is_relative_to(run) or out == run:
        raise ValueError("Scoring outputs must be in a new subdirectory of the run")
    if out.exists():
        raise FileExistsError(f"Preserving existing score output; choose a new --out: {out}")
    raw_files = list(run.glob('atlas_*_trajectory.csv'))
    if len(raw_files) != 1:
        raise ValueError("Expected exactly one raw atlas trajectory export")
    raw = raw_files[0]
    rows, stamps, selection = load_raw_rows(raw, args.map_scope)
    paths = asset_files(assets, args.sequence)
    config = run/'config/settings.yaml'
    coverage = json.loads((run/'coverage.json').read_text())
    if selection['atlas_pose_count'] != coverage['unique_input_frames_with_exported_pose']:
        raise ValueError("Pose count disagrees with independently computed coverage")
    processed_file = run/'config/timestamps_ns.txt'
    source_file = (args.source_timestamps or processed_file).resolve(strict=True)
    processed_stamps = timestamp_list(processed_file)
    if len(processed_stamps) != coverage['input_frames']:
        raise ValueError("Processed timestamp count disagrees with independently computed coverage")
    frame_coverage = input_coverage(processed_stamps, timestamp_list(source_file), stamps)
    tracked_diff = subprocess.check_output(['git', '-C', str(toolkit), 'diff', 'HEAD', '--name-only'], text=True)
    if tracked_diff.strip():
        raise ValueError("Official evaluator tracked sources have changed")
    sys.path.insert(0, str(toolkit))
    import cv2
    import pycolmap
    from lamaria.structs.trajectory import Trajectory
    from lamaria.structs.control_point import load_cp_json, run_control_point_triangulation
    from lamaria.utils.aria import initialize_reconstruction_from_calibration_file, get_t_imu_camera_from_calibration_file
    from lamaria.eval.sparse_evaluation import evaluate_wrt_control_points
    from lamaria.eval.pgt_evaluation import evaluate_wrt_pgt
    from lamaria.utils.metrics import calculate_control_point_score, calculate_control_point_recall, calculate_pose_recall, calculate_error
    from lamaria.utils.timestamps import matching_time_indices

    t_imu_cam0 = get_t_imu_camera_from_calibration_file(paths['calib'], 'cam0')
    fs = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
    run_tbc = fs.getNode('IMU.T_b_c1').mat()
    fs.release()
    if run_tbc is None:
        raise ValueError("Run calibration is missing IMU.T_b_c1")
    calibration_difference = float(np.max(np.abs(t_imu_cam0.matrix() - run_tbc[:3, :])))
    if calibration_difference >= 1e-6:
        raise ValueError("Official and estimator IMU/cam0 extrinsics differ; verify the sequence assets")
    try:
        pycolmap.set_random_seed(0)
    except AttributeError:
        pass
    np.random.seed(0)
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), out/'score_local.py')
    imu_path, cam_path = out/'estimated_imu_ns.txt', out/'estimated_cam0_ns.txt'
    with imu_path.open('w') as handle:
        handle.write('# timestamp_ns tx ty tz qx qy qz qw; world_from_imu-right, raw map gauge\n')
        for stamp, row in zip(stamps, rows):
            handle.write(str(stamp) + ' ' + ' '.join(row[k] for k in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')) + '\n')
    imu = Trajectory.load_from_file(imu_path, invert_poses=False, corresponding_sensor='imu')
    with cam_path.open('w') as handle:
        handle.write('# timestamp_ns tx ty tz qx qy qz qw; world_from_cam0 = world_from_imu * imu_from_cam0\n')
        for stamp, pose in imu.as_tuples():
            camera = pose * t_imu_cam0
            values = np.r_[camera.translation, camera.rotation.quat]
            handle.write(str(stamp) + ' ' + ' '.join(format(float(v), '.17g') for v in values) + '\n')
    source_files = [raw, config, *paths.values(), Path(__file__), run/'coverage.json', run/'command.json', run/'result.json', processed_file, source_file]
    provenance = {
        'sequence': args.sequence, 'command': sys.argv, 'python': sys.executable,
        'toolkit_commit': subprocess.check_output(['git', '-C', str(toolkit), 'rev-parse', 'HEAD'], text=True).strip(),
        'toolkit_tracked_diff': tracked_diff, 'random_seed': 0,
        'files_sha256': {str(p): sha(p) for p in source_files},
        'official_python_sha256': {str(p.relative_to(toolkit)): sha(p) for p in sorted(toolkit.rglob('*.py'))},
        'versions': {name: importlib.metadata.version(name) for name in ('numpy', 'scipy', 'pycolmap', 'pyceres', 'projectaria-tools', 'lamaria')},
        'pose_input': 'raw world_from_imu-right, qxyzw, integer ns; no GT prealignment',
        'calibration_max_abs_difference_from_run_Tbc': calibration_difference,
        'imu_from_cam0': t_imu_cam0.matrix().tolist(),
        'CP_calibration': 'Official native Fisheye624 calibration; fixed throughout scoring',
        'pGT_pose_frame': 'cam0; converted before applying the same CP-derived Sim3',
        'GT_used_in_estimator': False, 'public_upload': False,
        'wrapper_corrections': ['IMU-to-cam0 sensor conversion', 'Full original GT denominator', 'Prefilter official timestamp matches to avoid upstream shorter-estimate aliasing'],
        'estimator_returncode': coverage.get('estimator_returncode'),
        'export_returncode': coverage.get('export_returncode'),
        'trajectory_selection': selection,
        'source_timestamps': str(source_file),
        'source_timestamp_scope': 'full source recording supplied' if args.source_timestamps else 'processed run inputs only',
        'input_coverage': frame_coverage,
    }
    save_json(out/'provenance.json', provenance)

    reconstruction = initialize_reconstruction_from_calibration_file(paths['calib'])
    control_points, timestamp_to_images = load_cp_json(paths['cp'])
    reconstruction = imu.add_estimate_poses_to_reconstruction(reconstruction, timestamp_to_images)
    run_control_point_triangulation(reconstruction, control_points)
    result = evaluate_wrt_control_points(reconstruction, control_points)
    if result is None:
        raise RuntimeError("Official control-point alignment failed; no score fabricated")
    result.save_as_npy(out/'sparse_eval_result.npy')
    cp_errors = calculate_error(result)
    cp_rows = [{'tag_id': int(tag), 'name': summary.name, 'triangulated': summary.is_triangulated(),
                'horizontal_error_m': float(error) if np.isfinite(error) else None,
                'inlier_observations': len(control_points[tag].inlier_detections),
                'triangulation_inlier_ratio': float(control_points[tag].inlier_ratio)}
               for (tag, summary), error in zip(result.cp_summary.items(), cp_errors)]
    save_json(out/'control_point_errors.json', cp_rows)
    est_cam = Trajectory.load_from_file(cam_path, invert_poses=False, corresponding_sensor='cam0')
    gt_cam = Trajectory.load_from_file(paths['gt'], invert_poses=False, corresponding_sensor='cam0')
    full_gt_count, unmatched = associate_dense(est_cam, gt_cam, matching_time_indices)
    # Independent numerical check that the official call applies exactly the
    # saved CP transform, without another fit or accidental GT/estimate alias.
    expected = np.linalg.norm(np.asarray([result.alignment * p for p in est_cam.positions])[:, :2] - gt_cam.positions[:, :2], axis=1)
    errors = evaluate_wrt_pgt(est_cam, gt_cam, result.alignment)
    if errors is None or len(errors) != len(expected) or not np.allclose(errors, expected, atol=1e-7, rtol=1e-9):
        raise RuntimeError("Official dense errors disagree with the single CP alignment")
    with (out/'pgt_horizontal_errors.csv').open('w') as handle:
        writer = csv.writer(handle)
        writer.writerow(['estimate_timestamp_ns', 'gt_timestamp_ns', 'difference_ns', 'horizontal_error_m'])
        for et, gt, error in zip(est_cam.timestamps, gt_cam.timestamps, errors):
            writer.writerow([et, gt, abs(et-gt), format(float(error), '.17g')])
    save_json(out/'unmatched_gt_timestamps_ns.json', unmatched)
    scores = {
        'sequence': args.sequence, 'run': str(run), 'evaluation': 'LaMAria official functions, local evaluation only',
        'Score2D': float(calculate_control_point_score(result)),
        'CPRecall@1m_percent': float(calculate_control_point_recall(result, 1.0)),
        'PoseRecall@5m_percent': float(calculate_pose_recall(errors, full_gt_count, 5.0)),
        'PoseRecall@1m_percent': float(calculate_pose_recall(errors, full_gt_count, 1.0)),
        'CP_total': len(cp_rows), 'CP_triangulated': sum(r['triangulated'] for r in cp_rows),
        'CP_within_1m': int(np.sum(cp_errors <= 1.0)), 'GT_total_denominator': full_gt_count,
        'GT_associated': len(errors), 'GT_unmatched_count': len(unmatched),
        'GT_unmatched_before_first_estimate': sum(t < stamps[0] for t in unmatched),
        'GT_unmatched_after_last_estimate': sum(t > stamps[-1] for t in unmatched),
        'pose_count_within_5m': int(np.sum(errors <= 5.0)), 'pose_count_within_1m': int(np.sum(errors <= 1.0)),
        'estimated_poses': len(rows), **frame_coverage, 'map_epochs': 1,
        'source_timestamp_scope': provenance['source_timestamp_scope'],
        'trajectory_selection': selection,
        'pGT_association_max_difference_ns': max(abs(a-b) for a, b in zip(est_cam.timestamps, gt_cam.timestamps)),
        'associated_horizontal_error_m': {'median': float(np.median(errors)), 'rmse': float(np.sqrt(np.mean(errors**2))),
                                          'p90': float(np.percentile(errors, 90)), 'maximum': float(np.max(errors))},
        'CP_sim3': {'scale': float(result.alignment.scale), 'rotation_xyzw': result.alignment.rotation.quat.tolist(),
                    'translation': result.alignment.translation.tolist()},
        'protocol_notes': ['Horizontal XY errors, not 3D ATE.', 'One CP-derived Sim3; no separate dense-GT fit.',
                           'Missing GT timestamps count as failures in the full PoseRecall denominator; they do not directly change Score2D.',
                           'Score2D averages the official piecewise control-point error score; missing control points contribute zero.',
                           'Missing input poses remain absent; no gap filling or independent-map stitching.',
                           'Local per-sequence score, not a leaderboard submission.'],
    }
    save_json(out/'scores.json', scores)
    (out/'README.txt').write_text(
        f"LaMAria local score: {args.sequence}\n\n"
        f"Score2D: {scores['Score2D']:.6f} / 100\n"
        f"CP Recall @ 1 m: {scores['CPRecall@1m_percent']:.6f}% ({scores['CP_within_1m']}/{scores['CP_total']})\n"
        f"Pose Recall @ 5 m: {scores['PoseRecall@5m_percent']:.6f}% ({scores['pose_count_within_5m']}/{full_gt_count})\n\n"
        f"Estimated poses: {len(rows)} / {frame_coverage['native_input_frames']}; unmatched GT: {len(unmatched)}.\n"
        f"Processed input frames: {frame_coverage['processed_input_frames']}; unsupported boundary frames: {frame_coverage['unsupported_boundary_frames']}.\n"
        f"Input denominator scope: {provenance['source_timestamp_scope']}.\n"
        f"Scored frame: {selection['selected_coordinate_frame']}; atlas maps: {selection['atlas_map_epochs']}.\n"
        f"Selection: {selection['policy']}; other-map poses omitted: {selection['omitted_other_map_poses']}.\n"
        "All metrics use horizontal XY error and one official control-point Sim3 alignment.\n"
        "Missing poses remain absent; missing GT timestamps count as failures for PoseRecall.\n"
        "Score2D averages the official piecewise control-point error score; missing control points contribute zero.\n"
        "The official toolkit is unchanged; the wrapper preserves the full GT count,\n"
        "converts IMU poses to cam0 and avoids the upstream shorter-estimate association alias.\n"
        "No GT enters the estimator; no public submission was made.\n\n"
        "Official protocol: https://lamaria.ethz.ch/slam_documentation\n"
        "Sequence categories: https://lamaria.ethz.ch/slam_datasets\n")
    provenance['generated_files_sha256'] = {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file() and p.name not in ('provenance.json', 'evaluation.log')}
    save_json(out/'provenance.json', provenance)
    print(json.dumps(scores, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=relocated_path, required=True)
    parser.add_argument('--assets', type=relocated_path, required=True, help='Sequence folder containing native calibration and both GT assets')
    parser.add_argument('--sequence', required=True, help='Official sequence identifier, e.g. sequence_1_19')
    parser.add_argument('--toolkit', type=relocated_path, default=Path(__file__).resolve().parents[3]/'third_party/lamaria_toolkit')
    parser.add_argument('--out', type=relocated_path, help='New scoring subdirectory of --run; default: lamaria_score')
    parser.add_argument('--source-timestamps', type=relocated_path,
                        help='Full original integer-ns image timestamp list, including unsupported IMU boundary frames; affects coverage metadata only')
    parser.add_argument('--map-scope', choices=('single', 'largest'), default='single',
                        help='Require one map, or explicitly score the map with the most poses; all omitted GT timestamps remain failures')
    score(parser.parse_args())


if __name__ == '__main__':
    main()
