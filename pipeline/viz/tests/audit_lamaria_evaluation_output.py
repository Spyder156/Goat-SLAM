#!/usr/bin/env python3
"""Independently compare an evaluation RRD with its saved official score."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
from rerun.experimental import RrdReader
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('output', Path(__file__).parents[1] / 'make_lamaria_output.py')
output = importlib.util.module_from_spec(spec)
spec.loader.exec_module(output)


def audit(run, folder, gt_path):
    metadata = json.loads((folder/'output_metadata.json').read_text())
    scores_path = output.relocated_path(metadata['evaluation_score_source'])
    scores = json.loads(scores_path.read_text())
    sim3 = scores['CP_sim3']; scale = sim3['scale']
    align = Rotation.from_quat(sim3['rotation_xyzw']).as_matrix()
    offset = np.array(sim3['translation'])
    camera = output.load_table(scores_path.parent/'estimated_cam0_ns.txt', 8)
    expected_positions = scale * (camera[:, 1:4] @ align.T) + offset
    expected_rotations = align @ Rotation.from_quat(camera[:, 4:8]).as_matrix()
    timestamps = np.rint(camera[:, 0]).astype(np.int64)
    indices = {int(stamp): i for i, stamp in enumerate(timestamps)}
    trajectory = output.load_table(folder/'traj.csv', 8, delimiter=',', skiprows=1)
    assert np.array_equal(np.rint(trajectory[:, 0]*1e9).astype(np.int64), timestamps)
    trajectory_error = float(np.max(np.abs(trajectory[:, 1:4]-expected_positions)))
    assert trajectory_error < 1e-8
    _, rig, _ = output.load_calibration(run/'config/settings.yaml')
    expected_cam1 = expected_positions + scale*np.einsum('nij,j->ni', expected_rotations, rig[:3, 3])
    expected_rot1 = expected_rotations @ rig[:3, :3]
    raw = output.raw_files(run)
    if metadata.get('geometry_selection'):
        selected = output.selected_evaluation_geometry(run, scores_path, raw)
        assert selected is not None
        cloud = selected[1]
        assert selected[3]['selected_coordinate_frame'] == scores['trajectory_selection']['selected_coordinate_frame']
    else:
        cloud, _ = output.map_reveal_times(output.load_table(raw['mp_'], 6, delimiter=',', skiprows=1), raw['mp_'])
    cloud = cloud[np.argsort(cloud[:, 0], kind='stable')]
    expected_points = scale*(cloud[:, 1:4] @ align.T)+offset
    points = output.load_table(folder/'points.csv', 3, delimiter=',', skiprows=1)
    points_error = float(np.max(np.abs(points-expected_points)))
    assert points_error < 1e-8
    diagonal = float(np.linalg.norm(np.ptp(expected_positions, axis=0)))
    finite_points = np.isfinite(expected_points).all(1)
    keep = finite_points.copy()
    finite = np.flatnonzero(keep)
    keep[finite] = cKDTree(expected_positions).query(expected_points[finite])[0] <= 1.5*diagonal
    display_filter_report = {'source_landmarks': len(cloud),
                             'nonfinite_points_omitted': int((~finite_points).sum()),
                             'distant_points_omitted': int((finite_points & ~keep).sum()),
                             'eligible_points': int(keep.sum()),
                             'downsampling': False, 'point_count_cap': None}
    keep &= cloud[:, 0] <= metadata['recording_interval_s'][1]+1e-6
    display_filter_report['eligible_points_outside_recording_interval'] = display_filter_report['eligible_points']-int(keep.sum())
    display_filter_report['expected_recorded_points'] = int(keep.sum())
    gt = output.load_table(gt_path, 8)
    gt_index = {int(stamp): i for i, stamp in enumerate(np.rint(gt[:, 0]).astype(np.int64))}
    error_differences = []
    with (scores_path.parent/'pgt_horizontal_errors.csv').open() as handle:
        for row in csv.DictReader(handle):
            p = expected_positions[indices[int(row['estimate_timestamp_ns'])]]
            truth = gt[gt_index[int(row['gt_timestamp_ns'])], 1:4]
            error_differences.append(abs(np.linalg.norm(p[:2]-truth[:2])-float(row['horizontal_error_m'])))
    assert len(error_differences) == scores['GT_associated']
    assert max(error_differences) < 1e-8
    observed_camera_rows = [0, 0]; camera_error = [0., 0.]; rotation_error = [0., 0.]
    frustum_error = 0.; map_parts = {}; map_id_parts = {}; image_rows = [0, 0]
    trajectory_chunks_checked = 0
    trajectory_geometry_error = 0.
    selected_frame = scores['trajectory_selection']['selected_coordinate_frame']
    input_times = np.loadtxt(run/'config/timestamps_ns.txt', dtype=np.int64)
    maximum_segment_step = 1.5 * float(np.median(np.diff(input_times)))
    for chunk in RrdReader(folder/'run.rrd').stream():
        entity = chunk.entity_path
        if entity in ('/cam0/image', '/cam1/image'):
            if 'EncodedImage:blob' in chunk.to_record_batch().schema.names:
                image_rows[int(entity[4])] += chunk.num_rows
        elif entity in ('/world/cam0', '/world/cam1'):
            cam = int(entity[-1]); batch = chunk.to_record_batch()
            if 'Transform3D:translation' not in batch.schema.names:
                continue
            for row in batch.to_pylist():
                if not row.get('Transform3D:translation'):
                    continue
                index = indices[row['t'].value]
                if metadata.get('trajectory_source_timestamps_logged'):
                    assert row['source_timestamp_ns'] == [row['t'].value]
                    assert row['coordinate_frame'] == [selected_frame]
                p = expected_positions[index] if cam == 0 else expected_cam1[index]
                r = expected_rotations[index] if cam == 0 else expected_rot1[index]
                camera_error[cam] = max(camera_error[cam], float(np.max(np.abs(np.array(row['Transform3D:translation'][0])-p))))
                if row.get('Transform3D:mat3x3'):
                    rotation_error[cam] = max(rotation_error[cam], float(np.max(np.abs(np.array(row['Transform3D:mat3x3'][0]).reshape(3, 3, order='F')-r))))
                if row.get('Pinhole:image_plane_distance'):
                    frustum_error = max(frustum_error, abs(row['Pinhole:image_plane_distance'][0]-.005*diagonal))
                observed_camera_rows[cam] += 1
        elif entity.startswith('/world/map/part_'):
            batch = chunk.to_record_batch()
            for row in batch.to_pylist():
                if row.get('Points3D:positions'):
                    map_parts[entity] = np.array(row['Points3D:positions'])
                    if metadata.get('map_landmark_ids_logged'):
                        assert row['coordinate_frame'] == [selected_frame]
                        map_id_parts[entity] = np.array(row['landmark_id'], dtype=np.int64)
        elif entity.startswith('/world/trajectory/part_') and metadata.get('trajectory_source_timestamps_logged'):
            for row in chunk.to_record_batch().to_pylist():
                segment_stamps = row.get('source_timestamps_ns')
                if segment_stamps:
                    assert row['coordinate_frame'] == [selected_frame]
                    assert all(stamp in indices for stamp in segment_stamps)
                    assert np.all(np.diff(segment_stamps) <= maximum_segment_step)
                    assert np.all(np.diff(segment_stamps) > 0)
                    strips = row['LineStrips3D:strips']
                    assert len(strips) == 1
                    logged_segment = np.asarray(strips[0])
                    expected_segment = expected_positions[[indices[stamp] for stamp in segment_stamps]]
                    assert logged_segment.shape == expected_segment.shape
                    trajectory_geometry_error = max(trajectory_geometry_error, float(np.max(np.abs(logged_segment-expected_segment))))
                    assert trajectory_geometry_error < 1e-4
                    trajectory_chunks_checked += 1
    logged_points = np.vstack([map_parts[key] for key in sorted(map_parts)])
    assert logged_points.shape == expected_points[keep].shape
    if metadata.get('map_display_filter'):
        declared_filter = metadata['map_display_filter']
        for name in ('nonfinite_points_omitted', 'distant_points_omitted', 'eligible_points',
                     'eligible_points_outside_recording_interval'):
            assert declared_filter[name] == display_filter_report[name]
        assert declared_filter['recorded_points'] == len(logged_points)
        assert declared_filter['downsampling'] is False and declared_filter['point_count_cap'] is None
    logged_point_error = float(np.max(np.abs(logged_points-expected_points[keep])))
    if metadata.get('map_landmark_ids_logged'):
        logged_ids = np.concatenate([map_id_parts[key] for key in sorted(map_id_parts)])
        assert np.array_equal(logged_ids, cloud[keep, 5].astype(np.int64))
    assert logged_point_error < 1e-4
    assert max(camera_error) < 1e-4 and max(rotation_error) < 1e-6 and frustum_error < 1e-6
    assert observed_camera_rows == [len(camera), len(camera)]
    assert image_rows == [metadata['input_frames'], metadata['input_frames']]
    assert metadata['missing_images'] == 0 and metadata['rrd_verification'] == 'passed'
    return {'passed': True, 'official_score': scores['Score2D'], 'saved_sim3_scale': scale,
            'geometry_selection': metadata.get('geometry_selection'),
            'trajectory_csv_max_difference_m': trajectory_error, 'points_csv_max_difference_m': points_error,
            'pgt_horizontal_errors_checked': len(error_differences),
            'pgt_horizontal_error_max_difference_m': max(error_differences),
            'rrd_camera_rows': observed_camera_rows, 'rrd_camera_position_max_difference_m': camera_error,
            'rrd_camera_rotation_max_difference': rotation_error, 'rrd_image_rows': image_rows,
            'rrd_map_points_checked': len(logged_points), 'rrd_map_max_difference_m': logged_point_error,
            'map_display_filter_audit': display_filter_report,
            'rrd_selected_landmark_ids_checked': len(logged_points) if metadata.get('map_landmark_ids_logged') else None,
            'rrd_trajectory_chunks_checked_for_gaps': trajectory_chunks_checked,
            'rrd_trajectory_geometry_max_difference_m': trajectory_geometry_error,
            'rrd_frustum_size_max_difference_m': frustum_error, 'missing_images': 0,
            'note': 'All recorded camera transforms, map positions, image rows and saved scorer errors checked; float32 RRD storage tolerance only.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gt', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run, args.output, args.gt)
    (args.output/'evaluation_geometry_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    metadata_path = args.output/'output_metadata.json'
    metadata = json.loads(metadata_path.read_text())
    metadata['independent_evaluation_audit'] = result
    metadata_path.write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(result, indent=2))
