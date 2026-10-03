#!/usr/bin/env python3
"""Extract existing camera payloads and compare online/final failure evidence.

This is a diagnostic replay, not a new SLAM run. Final map0 cam0 poses use the
saved official CP alignment. Live IMU poses remain in their own map/version;
they are never overlaid with final poses or joined across gauge changes.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from rerun.experimental import RrdReader
from scipy.spatial.transform import Rotation


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def csv_rows(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def nearest(stamps, value):
    hi = int(np.searchsorted(stamps, value))
    choices = [i for i in (hi-1, hi) if 0 <= i < len(stamps)]
    return min(choices, key=lambda i: abs(int(stamps[i])-value))


def stamp_lookup(rows, field, stamps):
    result = {}
    for row in rows:
        value = round(float(row[field])*1e9)
        i = nearest(stamps, value)
        if abs(int(stamps[i])-value) <= 1000:
            result[int(stamps[i])] = row
    return result


def rgb(packed):
    values = np.asarray(packed, dtype=np.uint32)
    return np.column_stack([(values >> shift) & 255 for shift in (24, 16, 8, 0)]).astype(np.uint8)


def cameras(run, stamps):
    """Decode only selected rows; retain original compressed image bytes."""
    store = RrdReader(run/'run.rrd').store()
    frames = {int(t): [{}, {}] for t in stamps}
    for cam in (0, 1):
        for chunk in store.stream().filter(content=f'/cam{cam}/**'):
            batch = chunk.to_record_batch()
            times = np.asarray(batch.column('t').cast('int64'))
            selected = np.flatnonzero((times >= stamps[0]-1) & (times <= stamps[-1]+1))
            for index in selected:
                value = int(times[index]); target = int(stamps[nearest(stamps, value)])
                if abs(target-value) > 1:
                    continue
                frame = frames[target][cam]
                names = batch.schema.names
                if 'EncodedImage:blob' in names:
                    if 'image' in frame:
                        raise ValueError('Duplicate image timestamp')
                    frame['image'] = bytes(batch.column('EncodedImage:blob')[index].as_py()[0])
                if 'Points2D:positions' in names:
                    frame['pixels'] = np.asarray(batch.column('Points2D:positions')[index].as_py(), dtype=np.float32).reshape(-1, 2)
                    frame['colors'] = rgb(batch.column('Points2D:colors')[index].as_py())
                if 'LineStrips2D:strips' in names:
                    frame['trails'] = batch.column('LineStrips2D:strips')[index].as_py()
                    frame['trail_colors'] = rgb(batch.column('LineStrips2D:colors')[index].as_py())
        for pair in frames.values():
            assert set(pair[cam]) == {'image', 'pixels', 'colors', 'trails', 'trail_colors'}
            assert len(pair[cam]['pixels']) == len(pair[cam]['colors'])
        print(f'Loaded cam{cam}: {len(stamps)} original images/overlays/trails', flush=True)
    return frames


def make(args):
    run, out = args.run.resolve(), args.out.resolve()
    if out.exists():
        raise FileExistsError(f'Choose a new diagnostics folder: {out}')
    all_stamps = np.loadtxt(run/'config/timestamps_ns.txt', dtype=np.int64)
    stamps = all_stamps[(all_stamps >= round(args.start*1e9)) & (all_stamps <= round(args.end*1e9))]
    assert len(stamps) > 1
    times = stamps.astype(float)*1e-9
    online_path = next(run.glob('online_*.csv'))
    online = stamp_lookup(csv_rows(online_path), 'input_t_s', stamps)
    assert set(online) == set(map(int, stamps))
    stats_rows = csv_rows(run/'stats.csv')
    stats = [stamp_lookup([r for r in stats_rows if int(r['cam']) == c], 't', stamps) for c in (0, 1)]
    assert all(set(s) == set(online) for s in stats)
    score = json.loads((run/'lamaria_score/scores.json').read_text())
    provenance = json.loads((run/'lamaria_score/provenance.json').read_text())
    gt_path = next(Path(p) for p in provenance['files_sha256'] if p.endswith('/gt_dense.txt'))
    final_path = run/'lamaria_score/estimated_cam0_ns.txt'
    final = np.loadtxt(final_path, ndmin=2)
    final_stamps = np.asarray([int(line.split()[0]) for line in final_path.read_text().splitlines() if line.strip() and not line.startswith('#')], dtype=np.int64)
    sim = score['CP_sim3']
    rotation = Rotation.from_quat(sim['rotation_xyzw']).as_matrix()
    final_xyz = sim['scale'] * (final[:, 1:4] @ rotation.T) + np.asarray(sim['translation'])
    gt = np.loadtxt(gt_path, ndmin=2)
    gt_times = gt[:, 0]*1e-9
    selected_gt = (gt_times >= times[0]) & (gt_times <= times[-1])
    origin = gt[selected_gt, 1:4].mean(0)
    final_xyz -= origin
    gt_xyz = gt[selected_gt, 1:4] - origin
    selected_final = (final_stamps >= stamps[0]) & (final_stamps <= stamps[-1])
    final_by_stamp = {int(t): p for t, p in zip(final_stamps[selected_final], final_xyz[selected_final])}
    points = np.concatenate([gt_xyz, final_xyz[selected_final]])
    scale = max(float(np.linalg.norm(np.ptp(points, axis=0))), 1.)
    center = (points.min(0)+points.max(0))/2
    errors = {}
    for row in csv_rows(run/'lamaria_score/pgt_horizontal_errors.csv'):
        t = int(row['estimate_timestamp_ns'])
        if stamps[0] <= t <= stamps[-1]:
            errors[t] = float(row['horizontal_error_m'])
    frames = cameras(run, stamps)
    out.mkdir(parents=True, exist_ok=False)
    (out/'viz').mkdir(); (out/'config').mkdir()
    (out/'config/timestamps_ns.txt').write_text(''.join(f'{t}\n' for t in stamps))
    (out/'config/settings.yaml').write_bytes((run/'config/settings.yaml').read_bytes())
    record = {
        'artifact_kind': 'Diagnostic replay of existing baseline; not a new estimator run',
        'source_run': str(run), 'source_rrd': str(run/'run.rrd'),
        'source_recording_bytes': (run/'run.rrd').stat().st_size,
        'frames_per_camera': len(stamps), 'window_native_seconds': [float(times[0]), float(times[-1])],
        'window_elapsed_seconds': [float((stamps[0]-all_stamps[0])*1e-9), float((stamps[-1]-all_stamps[0])*1e-9)],
        'image_scope': 'Both original RRD camera streams, exact compressed bytes and existing display rotations; full 20 Hz selected window',
        'keypoint_scope': 'Exact original per-frame associations: green mapped, red unmapped; original persistent-ID trails',
        'final_pose_scope': 'Final primary-map0 cam0 poses under saved CP Sim3. No new fit; no extrapolation after map0 ends.',
        'final_alignment': sim, 'display_origin_subtracted_from_final_and_gt': origin.tolist(),
        'online_pose_scope': 'Raw live IMU pose in current map/version only; trail clears at any map/init/world_version change. Not registered to final or GT.',
        'online_pose_caveat': 'Some backend pose corrections are not reflected in world_version. A live jump alone does not establish physical motion or tracking failure.',
        'reprojection_scope': 'Source stats.csv uses FINAL map/pose geometry; not online solver residuals.',
        'map_cloud_scope': 'Trajectory diagnostics only; full clouds remain in baseline run.rrd and atlas_overview.rrd.',
        'source_sha256': {str(p): digest(p) for p in [online_path, run/'stats.csv', final_path, gt_path, run/'lamaria_score/scores.json', run/'config/settings.yaml', run/'config/timestamps_ns.txt', Path(__file__)]},
        'independent_maps_joined': False, 'source_modified': False,
    }
    rr.init('lamaria_failure_'+args.label, spawn=False); rr.save(str(out/'run.rrd'))
    live_rows = [online[int(t)] for t in stamps if int(online[int(t)]['pose_available'])]
    live_xyz = np.array([[float(row[k]) for k in ('tx', 'ty', 'tz')] for row in live_rows])
    # Fix the eye to the longest single live gauge; other gauges never share geometry.
    gauges = {}
    for row, xyz in zip(live_rows, live_xyz):
        gauges.setdefault(tuple(row[k] for k in ('map_id', 'map_init_kf_id', 'world_version')), []).append(xyz)
    primary_live = np.array(max(gauges.values(), key=len))
    live_center = (primary_live.min(0)+primary_live.max(0))/2
    live_scale = max(float(np.linalg.norm(np.ptp(primary_live, axis=0))), 1.)
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
        rrb.Vertical(
            rrb.Spatial3DView(origin='/final', name='TOP VIEW: final map0 cam0 (orange) + reference (green)',
                eye_controls=rrb.EyeControls3D(position=center+[0, 0, 1.3*scale], look_target=center, eye_up=[0, 1, 0]),
                line_grid=rrb.LineGrid3D(visible=True, spacing=1.)),
            rrb.Spatial3DView(origin='/online', name='LIVE IMU pose: independent current map/version',
                eye_controls=rrb.EyeControls3D(position=live_center+live_scale*np.array([.7, -.7, .9]), look_target=live_center, eye_up=[0, 0, 1]),
                line_grid=rrb.LineGrid3D(visible=True, spacing=1.)), row_shares=[2, 1]),
        rrb.Vertical(
            rrb.Horizontal(rrb.Spatial2DView(origin='/cam0', name='Cam0: green mapped / red unmapped'),
                           rrb.Spatial2DView(origin='/cam1', name='Cam1: green mapped / red unmapped')),
            rrb.TextDocumentView(origin='/status', name='Current frame / coordinate-frame scope'),
            rrb.TimeSeriesView(origin='/counts', name='Association counts and ONLINE optimization inliers'),
            rrb.TimeSeriesView(origin='/quality', name='Final-map diagnostic error (m); not online residual'),
            rrb.TimeSeriesView(origin='/state', name='Tracking state / coasting / map ID'), row_shares=[5, 1, 1.5, 1, 1]),
        column_shares=[1, 1]),
        rrb.TimePanel(timeline='t', play_state='Paused', playback_speed=.5, loop_mode='Selection',
            time_selection=rr.datatypes.AbsoluteTimeRange(rr.datatypes.TimeInt(seconds=float(times[0])), rr.datatypes.TimeInt(seconds=float(times[-1]))))),
        make_active=True, make_default=True)
    rr.set_time('t', timestamp=np.datetime64(int(stamps[0]), 'ns'))
    rr.log('/metadata', rr.TextDocument(json.dumps(record, indent=2)))
    rr.log('/final', rr.ViewCoordinates.RIGHT_HAND_Z_UP)
    rr.log('/final/gt', rr.LineStrips3D([gt_xyz], colors=[115, 200, 115], radii=.001*scale))
    rr.log('/counts/cam0_mapped', rr.SeriesLines(colors=[60, 140, 255], names=['cam0 mapped']))
    rr.log('/counts/cam1_mapped', rr.SeriesLines(colors=[210, 90, 245], names=['cam1 mapped']))
    rr.log('/counts/online_inliers', rr.SeriesLines(colors=[255, 190, 60], names=['online total inliers']))
    rr.log('/quality/final_horizontal_m', rr.SeriesLines(colors=[255, 110, 60], names=['final cam0 error in metres']))
    rr.log('/state/tracking', rr.SeriesLines(names=['state: 2 OK, 3 recently lost, 4 lost']))
    rr.log('/state/coasting', rr.SeriesLines(names=['IMU coasting: 0/1']))
    rr.log('/state/map_id', rr.SeriesLines(names=['map ID']))
    final_trail = []; live_trail = []; gauge = None; events = []
    still_targets = args.still or list(np.linspace(times[0], times[-1], 5))
    still_stamps = {int(stamps[nearest(stamps, round(t*1e9))]) for t in still_targets}
    panels = []
    diagnostic_rows = []
    for index, stamp in enumerate(map(int, stamps)):
        t = stamp*1e-9; row = online[stamp]; s = [v[stamp] for v in stats]
        rr.set_time('t', timestamp=np.datetime64(stamp, 'ns'))
        for cam, frame in enumerate(frames[stamp]):
            rr.log(f'/cam{cam}/image', rr.EncodedImage(contents=frame['image'], media_type='image/jpeg'))
            rr.log(f'/cam{cam}/image/keypoints', rr.Points2D(frame['pixels'], colors=frame['colors'], radii=1.5, draw_order=30.))
            rr.log(f'/cam{cam}/image/trails', rr.LineStrips2D(frame['trails'], colors=frame['trail_colors'], radii=.65, draw_order=20.))
            mapped = np.count_nonzero(np.all(frame['colors'][:, :3] == [40, 230, 80], axis=1))
            assert mapped == int(s[cam]['mapped'])
            assert len(frame['pixels']) == int(s[cam]['features'])
            rr.log(f'/counts/cam{cam}_mapped', rr.Scalars(mapped))
        rr.log('/counts/online_inliers', rr.Scalars(float(row['inliers'])))
        for name, key in [('tracking', 'state'), ('coasting', 'coasting'), ('map_id', 'map_id')]:
            rr.log('/state/'+name, rr.Scalars(int(row[key])))
        if stamp in errors:
            rr.log('/quality/final_horizontal_m', rr.Scalars(errors[stamp]))
        if stamp in final_by_stamp:
            final_trail.append(final_by_stamp[stamp])
            rr.log('/final/trajectory', rr.LineStrips3D([final_trail], colors=[245, 185, 70], radii=.0015*scale))
            rr.log('/final/current', rr.Points3D([final_by_stamp[stamp]], colors=[245, 185, 70], radii=.005*scale))
        else:
            rr.log('/final/current', rr.Clear(recursive=True))
        next_gauge = tuple(row[k] for k in ('map_id', 'map_init_kf_id', 'world_version'))
        if next_gauge != gauge:
            rr.log('/online', rr.Clear(recursive=True)); live_trail = []; gauge = next_gauge
            rr.log('/online', rr.ViewCoordinates.RIGHT_HAND_Z_UP)
            events.append({'t_s': t, 'map_id': int(gauge[0]), 'map_init_kf_id': int(gauge[1]), 'world_version': int(gauge[2])})
        if int(row['pose_available']):
            p = [float(row[k]) for k in ('tx', 'ty', 'tz')]
            live_trail.append(p)
            rr.log('/online/trajectory', rr.LineStrips3D([live_trail], colors=[90, 180, 255], radii=.0015*live_scale))
            rr.log('/online/current', rr.Points3D([p], colors=[255, 80, 60] if int(row['coasting']) else [90, 180, 255], radii=.005*live_scale))
        else:
            rr.log('/online/current', rr.Clear(recursive=True))
            live_trail = []
        text = f"t={t:.3f}s | elapsed={(stamp-int(all_stamps[0]))*1e-9:.3f}s | state={row['state']} | coasting={row['coasting']} | inliers={row['inliers']}\n"
        text += f"ONLINE map={gauge[0]}, initKF={gauge[1]}, version={gauge[2]}; body pose in its own gauge.\nFINAL map0 cam0: {'available' if stamp in final_by_stamp else 'ABSENT (not extrapolated)'}. Green image dots are associations, not guaranteed inliers."
        rr.log('/status', rr.TextDocument(text))
        diagnostic_rows.append({'t_s': f'{t:.9f}', **{k: row[k] for k in ('map_id', 'map_init_kf_id', 'world_version', 'state', 'coasting', 'inliers', 'imu_initialized', 'pose_available')}, 'cam0_mapped': s[0]['mapped'], 'cam1_mapped': s[1]['mapped'], 'cam0_detected': s[0]['features'], 'cam1_detected': s[1]['features'], 'final_map0_pose_available': int(stamp in final_by_stamp), 'final_horizontal_error_m': errors.get(stamp, '')})
        if stamp in still_stamps:
            tiles = []
            for cam, frame in enumerate(frames[stamp]):
                im = cv2.imdecode(np.frombuffer(frame['image'], np.uint8), cv2.IMREAD_COLOR)
                for p, color in zip(frame['pixels'], frame['colors']):
                    cv2.circle(im, tuple(np.rint(p).astype(int)), 2, tuple(map(int, color[:3][::-1])), -1, cv2.LINE_AA)
                im = cv2.copyMakeBorder(im, 58, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25,25,25))
                cv2.putText(im, f"cam{cam}  t={t:.3f}s  mapped={s[cam]['mapped']}/{s[cam]['features']}", (8,22), cv2.FONT_HERSHEY_SIMPLEX, .48, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(im, f"state={row['state']} coast={row['coasting']} map={row['map_id']} total inliers={row['inliers']}", (8,46), cv2.FONT_HERSHEY_SIMPLEX, .43, (255,255,255), 1, cv2.LINE_AA)
                tiles.append(im)
            # Upright physical camera displays can have different dimensions.
            height = max(im.shape[0] for im in tiles)
            tiles = [cv2.copyMakeBorder(im, 0, height-im.shape[0], 0, 0, cv2.BORDER_CONSTANT) for im in tiles]
            panel = np.hstack(tiles); panels.append(panel)
            assert cv2.imwrite(str(out/'viz'/f'overlays_{stamp}.jpg'), panel)
        if index % 400 == 0:
            print(f'Logged {index}/{len(stamps)} frames', flush=True)
    recording = rr.get_global_data_recording(); recording.flush(); recording.disconnect()
    width = max(panel.shape[1] for panel in panels)
    panels = [cv2.copyMakeBorder(im, 0, 0, 0, width-im.shape[1], cv2.BORDER_CONSTANT) for im in panels]
    assert cv2.imwrite(str(out/'viz/failure_contact_sheet.jpg'), np.vstack(panels))
    with (out/'stats.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(diagnostic_rows[0])); writer.writeheader(); writer.writerows(diagnostic_rows)
    np.savetxt(out/'traj.csv', np.column_stack([final_stamps[selected_final]*1e-9, final_xyz[selected_final]]), delimiter=',', header='t,score_frame_cam0_x,score_frame_cam0_y,score_frame_cam0_z', comments='')
    verify = subprocess.run([str(Path(sys.executable).parent/'rerun'), 'rrd', 'verify', str(out/'run.rrd')], capture_output=True, text=True)
    (out/'run.rrd.verify.log').write_text(verify.stdout+verify.stderr)
    assert verify.returncode == 0, verify.stdout+verify.stderr
    # Independently decode the saved payload and compare every image and overlay.
    check = RrdReader(out/'run.rrd').store(); counts = {}
    for cam in (0, 1):
        seen_images = set(); seen_pixels = set(); seen_trails = set()
        for chunk in check.stream().filter(content=f'/cam{cam}/**'):
            batch = chunk.to_record_batch(); ts = batch.column('t').cast('int64').to_pylist()
            for i, stamp in enumerate(ts):
                original = frames[stamp][cam]
                if 'EncodedImage:blob' in batch.schema.names:
                    assert stamp not in seen_images; seen_images.add(stamp)
                    assert bytes(batch.column('EncodedImage:blob')[i].as_py()[0]) == original['image']
                if 'Points2D:positions' in batch.schema.names:
                    assert stamp not in seen_pixels; seen_pixels.add(stamp)
                    actual = np.asarray(batch.column('Points2D:positions')[i].as_py(), dtype=np.float32).reshape(-1, 2)
                    assert np.array_equal(actual, original['pixels'])
                    assert np.array_equal(rgb(batch.column('Points2D:colors')[i].as_py()), original['colors'])
                if 'LineStrips2D:strips' in batch.schema.names:
                    assert stamp not in seen_trails; seen_trails.add(stamp)
                    assert batch.column('LineStrips2D:strips')[i].as_py() == original['trails']
                    assert np.array_equal(rgb(batch.column('LineStrips2D:colors')[i].as_py()), original['trail_colors'])
        assert seen_images == seen_pixels == seen_trails == set(map(int, stamps))
        counts[str(cam)] = {'images': len(seen_images), 'overlays': len(seen_pixels), 'trail_frames': len(seen_trails), 'exact_source_payload_match': True}
    record.update({'passed': True, 'rrd_verify_returncode': verify.returncode, 'validated_camera_payloads': counts, 'online_gauge_events': events, 'final_map0_frames': len(final_by_stamp), 'rrd_bytes': (out/'run.rrd').stat().st_size})
    (out/'validation.json').write_text(json.dumps(record, indent=2)+'\n')
    (out/'score.txt').write_text('Diagnostic clip only; not independently scored. Baseline whole-sequence Score2D='+str(score['Score2D'])+'\n')
    (out/'README.txt').write_text('Failure-window replay of an existing run, not a new estimator result.\nOpen run.rrd and scrub through the selected native-time interval.\nFinal map0 camera geometry and raw live IMU geometry have SEPARATE views and frames.\nThe final pane uses the original whole-sequence CP Sim3; it does not fit this clip.\nThe online trail clears at map/init/world-version changes.\nGreen means mapped association, not necessarily a pose-optimization inlier.\nNo full map cloud is included; see the source baseline recordings.\nContact sheet: viz/failure_contact_sheet.jpg\nSource: '+str(run)+'\n')
    print(json.dumps({'out': str(out), 'passed': True, 'frames_per_camera': len(stamps), 'bytes': record['rrd_bytes']}, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--start', type=float, required=True)
    parser.add_argument('--end', type=float, required=True)
    parser.add_argument('--still', type=float, nargs='*')
    make(parser.parse_args())
