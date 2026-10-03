#!/usr/bin/env python3
"""Compare completed LaMAria observation-repair runs with registered baselines.

This creates diagnostic artifacts only. It never changes an estimate, joins
independent maps, fills missing poses, or feeds GT-derived fits into SLAM.
Scale calculations reproduce audit_lamaria_scale.py on identical associations.
"""
import argparse
import base64
import csv
import hashlib
import html
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


REPO = Path(__file__).resolve().parents[2]
REGISTRY = REPO/'configs/orbslam3_lamaria/suite_20261002.json'
SCALE_REFERENCE = REPO/'experiments/lamaria_structural_audit_20261002/scale/audit_lamaria_scale.py'
GRAPH_ERRORS = ('missing_backlinks', 'wrong_reverse_slots', 'invalid_observation_indices',
                'bad_observation_keyframes', 'observation_count_mismatches', 'bad_point_slots')
GRAPH_COUNTS = ('cam0_slots', 'cam1_slots', 'cam0_observations', 'cam1_observations')


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def read_rows(path):
    with Path(path).open() as handle:
        return list(csv.DictReader(handle))


def one(run, pattern):
    paths = list(run.glob(pattern))
    if len(paths) != 1:
        raise ValueError(f'Expected one {pattern} in {run}; found {len(paths)}')
    return paths[0]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def nearest(source, targets):
    ids = np.clip(np.searchsorted(source, targets), 0, len(source)-1)
    prev = np.maximum(ids-1, 0)
    ids = np.where(abs(source[prev]-targets) <= abs(source[ids]-targets), prev, ids)
    return ids, abs(source[ids]-targets)


def fit(x, y, scale=False):
    """Trusted audit's proper-rotation Umeyama fit; no reflection or pose editing."""
    require(len(x) >= 3 and len(x) == len(y), 'Insufficient paired positions')
    xc, yc = x-x.mean(axis=0), y-y.mean(axis=0)
    require(np.sum(xc*xc) > 1e-12, 'Degenerate position spread')
    u, d, vt = np.linalg.svd(yc.T @ xc)
    correction = np.diag([1., 1., np.linalg.det(u @ vt)])
    r = u @ correction @ vt
    s = float(np.sum(d*np.diag(correction))/np.sum(xc*xc)) if scale else 1.
    require(s > 0 and abs(np.linalg.det(r)-1) < 1e-9, 'Invalid proper similarity')
    t = y.mean(axis=0)-s*r@x.mean(axis=0)
    aligned = s*x@r.T+t
    error = np.linalg.norm(aligned-y, axis=1)
    return {'scale_estimate_to_gt': s, 'estimate_stretch': 1/s,
            'rmse_m': float(np.sqrt(np.mean(error**2))), 'median_m': float(np.median(error)),
            'p90_m': float(np.quantile(error, .9)), 'max_m': float(error.max()),
            'rotation': r.tolist(), 'translation': t.tolist()}, aligned


def summarize(t, x, y, mask):
    if np.count_nonzero(mask) < 3 or np.sum((x[mask]-x[mask].mean(0))**2) <= 1e-12:
        return None
    fixed, _ = fit(x[mask], y[mask], False)
    scaled, _ = fit(x[mask], y[mask], True)
    return {'poses': int(mask.sum()), 'first_s': float(t[mask][0]), 'last_s': float(t[mask][-1]),
            'se3': fixed, 'sim3': scaled}


def windows(t, x, y, good, width):
    result = []
    for start in np.arange(t[0], t[-1]-width, width/2):
        mask = good & (t >= start) & (t < start+width)
        if mask.sum() < 30 or np.max(np.diff(t[mask])) > 2.:
            continue
        spread = float(np.sqrt(np.mean(np.sum((y[mask]-y[mask].mean(0))**2, axis=1))))
        if spread < 3:
            continue
        row = summarize(t, x, y, mask)
        if row:
            row['gt_rms_spread_m'] = spread
            row['gt_singular_values'] = np.linalg.svd(y[mask]-y[mask].mean(0), compute_uv=False).tolist()
            result.append(row)
    return result


def graph_audit(run):
    paths = list(run.glob('atlas_*_observation_audit.csv'))
    if not paths:
        return {'available': False, 'reason': 'This run did not export the exact observation-graph audit.'}, []
    require(len(paths) == 1, f'Ambiguous graph audit in {run}')
    error_path = one(run, 'atlas_*_observation_errors.csv')
    by_map = {}
    for row in read_rows(paths[0]):
        values = by_map.setdefault(row['map_id'], {})
        require(row['metric'] not in values, 'Duplicate graph-audit metric')
        values[row['metric']] = int(row['count'])
    for values in by_map.values():
        require(set(GRAPH_ERRORS+GRAPH_COUNTS) <= set(values), 'Graph audit lacks required metrics')
        require(min(values.values()) >= 0, 'Negative graph-audit count')
    totals = {k: sum(m[k] for m in by_map.values()) for k in GRAPH_ERRORS+GRAPH_COUNTS}
    errors = read_rows(error_path)
    # bad_point_slots is an aggregate-only metric in the production exporter.
    for kind in GRAPH_ERRORS[:-1]:
        require(sum(r['kind'] == kind for r in errors) == totals[kind], f'{kind} detail count differs')
    return {'available': True, 'by_map': by_map, 'totals': totals,
            'zero_reported_violations': all(totals[k] == 0 for k in GRAPH_ERRORS),
            'live_graph_zero_violations': all(totals[k] == 0 for k in GRAPH_ERRORS[:-1]),
            'observation_minus_slot_counts': {f'cam{cam}': totals[f'cam{cam}_observations']-totals[f'cam{cam}_slots'] for cam in (0, 1)},
            'stale_bad_point_slots': totals['bad_point_slots'],
            'error_detail_rows': len(errors), 'source_csv': str(paths[0]),
            'source_errors_csv': str(error_path), 'scope': 'Final graph after backend shutdown; not every transient graph state.'}, [paths[0], error_path]


def load_case(entry, run):
    """Same raw body->cam0 conversion and nearest-GT association as scale audit."""
    run = run.resolve(strict=True)
    raw, online = one(run, 'atlas_*_trajectory.csv'), one(run, 'online_*.csv')
    maps_path = one(run, 'atlas_*_maps.csv')
    config = run/'config/settings.yaml'
    gtfile, calibfile = Path(entry['assets'])/'gt_dense.txt', Path(entry['assets'])/'aria_calib.json'
    score_path, coverage_path = run/'lamaria_score/scores.json', run/'coverage.json'
    score, coverage, result = read_json(score_path), read_json(coverage_path), read_json(run/'result.json')
    require(result.get('returncode') == 0, f'Estimator did not finish successfully: {run}')
    require(result.get('coverage_returncode') == 0, f'Coverage is not complete: {run}')
    require(score['sequence'] == entry['sequence'], 'Sequence does not match suite entry')
    fs = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
    tbc = fs.getNode('IMU.T_b_c1').mat().astype(float)
    fs.release()
    official = read_json(calibfile)['cam0']['T_b_s']
    official_tbc = np.eye(4)
    official_tbc[:3, :3] = Rotation.from_quat(official['qvec']).as_matrix()
    official_tbc[:3, 3] = official['tvec']
    difference = float(np.max(abs(tbc-official_tbc)))
    require(difference < 1e-6, 'Effective cam0 extrinsics differ: extend export/scoring before calibration tests')
    source = np.loadtxt(entry['source_timestamps'], dtype=np.int64, ndmin=1)
    supported = np.loadtxt(entry['supported_timestamps'], dtype=np.int64, ndmin=1)
    inputs = np.loadtxt(run/'config/timestamps_ns.txt', dtype=np.int64, ndmin=1)
    require(np.all(np.diff(source) > 0) and np.all(np.diff(supported) > 0), 'Unordered source timestamps')
    require(np.array_equal(inputs, supported), 'Candidate/baseline must use the full supported input, not a prefix')
    require(np.isin(supported, source).all(), 'Supported frames must belong to original source')
    require(score['native_input_frames'] == len(source) and score['processed_input_frames'] == len(supported),
            'Scoring denominator is not the full original source')
    require(score['GT_total_denominator'] == score['GT_associated']+score['GT_unmatched_count'], 'GT denominator mismatch')
    require(coverage['input_frames'] == len(supported), 'Coverage input denominator mismatch')
    groups = {}
    for row in read_rows(raw):
        groups.setdefault(row['coordinate_frame'], []).append(row)
    frame = min(groups, key=lambda k: (-len(groups[k]), k))
    require(frame == score['trajectory_selection']['selected_coordinate_frame'], 'Scale and score primary-map selection disagree')
    rows = groups[frame]
    require(frame.endswith('_body0'), 'Expected explicit independent body-frame export')
    stamps = np.rint(np.array([float(r['t_s']) for r in rows])*1e9).astype(np.int64)
    require(np.all(np.diff(stamps) > 0), 'Primary poses are not strictly ordered')
    pos = np.array([[float(r[k]) for k in ('tx', 'ty', 'tz')] for r in rows])
    quats = np.array([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in rows])
    require(np.max(abs(np.linalg.norm(quats, axis=1)-1)) < 1e-5, 'Non-unit exported quaternion')
    pos += Rotation.from_quat(quats).apply(np.broadcast_to(official_tbc[:3, 3], pos.shape).copy())
    gt = np.loadtxt(gtfile, ndmin=2)
    gtstamps = np.rint(gt[:, 0]).astype(np.int64)
    require(np.all(np.diff(gtstamps) > 0), 'GT timestamps are not strictly ordered')
    ei, delta = nearest(stamps, gtstamps)
    gi = np.flatnonzero(delta <= 1000000)
    ei = ei[gi]
    require(len(ei) == len(np.unique(ei)) and len(ei) >= 3, 'Invalid/insufficient GT associations')
    x, y, t = pos[ei], gt[gi, 1:4], stamps[ei]/1e9
    online_rows = read_rows(online)
    ot = np.rint(np.array([float(r['input_t_s']) for r in online_rows])*1e9).astype(np.int64)
    require(np.array_equal(ot, supported), 'Online trace does not cover every supported input exactly once')
    oi, od = nearest(ot, stamps[ei])
    require(od.max() <= 1000, 'Missing online state for final pose')
    good = np.array([int(online_rows[i]['state']) == 2 and int(online_rows[i]['coasting']) == 0 for i in oi])
    scorecsv = run/'lamaria_score/pgt_horizontal_errors.csv'
    scored = read_rows(scorecsv)
    require([int(r['gt_timestamp_ns']) for r in scored] == gtstamps[gi].tolist(), 'Scale and official scoring associations differ')
    require(len(scored) == score['GT_associated'], 'Score count differs from association CSV')
    audit, graph_paths = graph_audit(run)
    maps = read_rows(maps_path)
    exported = int(coverage['unique_input_frames_with_exported_pose'])
    require(exported == sum(len(v) for v in groups.values()), 'Atlas coverage and pose rows differ')
    metrics = {'source_frames': len(source), 'supported_frames': len(supported),
               'unsupported_boundary_frames': len(source)-len(supported),
               'primary_poses': len(rows), 'primary_source_fraction': len(rows)/len(source),
               'primary_missing_source_frames': len(source)-len(rows),
               'all_independent_maps_exported_poses': exported,
               'all_independent_maps_source_fraction': exported/len(source),
               'all_independent_maps_missing_source_frames': len(source)-exported,
               'visual_ok_frames': int(coverage['logged_visual_ok_frames']),
               'visual_ok_source_fraction': coverage['logged_visual_ok_frames']/len(source),
               'coasted_frames': int(coverage['logged_coasted_frames']),
               'independent_maps': int(coverage['independent_segments_exported']),
               'new_map_events': int(coverage['new_map_events']),
               'completed_active_map_resets': int(coverage['completed_active_map_resets'])}
    files = [raw, online, maps_path, config, gtfile, calibfile, score_path, coverage_path,
             scorecsv, run/'result.json', run/'config/timestamps_ns.txt',
             Path(entry['source_timestamps']), Path(entry['supported_timestamps']), *graph_paths]
    for optional in ('command.json', 'config/build_manifest.json', 'lamaria_score/independent_metric_check.json'):
        if (run/optional).exists():
            files.append(run/optional)
    info = {'run': str(run), 'sequence': entry['sequence'], 'coordinate_frame': frame,
            'atlas_maps': {k: len(v) for k, v in groups.items()}, 'map_manifest': maps,
            'coverage': coverage, 'metrics': metrics, 'score': score, 'graph_audit': audit,
            'gt_associations': len(t), 'visual_ok_associations': int(good.sum()),
            'max_timestamp_difference_ns': int(delta[gi].max()),
            'calibration_max_abs_difference': difference,
            'sensor_conversion': 'world_from_cam0 = world_from_imu_right * imu_right_from_cam0; xyzw',
            'fits': {'all_primary': summarize(t, x, y, np.ones(len(t), dtype=bool)),
                     'visual_ok': summarize(t, x, y, good)},
            'windows_60s': windows(t, x, y, good, 60.),
            'sha256': {str(p): sha(p) for p in files}}
    return {'t': t, 'x': x, 'y': y, 'good': good, 'gt_stamps': gtstamps[gi],
            'info': info, 'inputs': inputs, 'online': online_rows}


def line_with_gaps(axis, t, xy, **kwargs):
    # Missing timestamps remain visible as gaps; never join separate maps.
    cut = np.flatnonzero(np.diff(t) > 2.)+1
    for number, ids in enumerate(np.split(np.arange(len(t)), cut)):
        axis.plot(xy[ids, 0], xy[ids, 1], label=kwargs.pop('label', None) if number == 0 else None, **kwargs)


def figure_pair(pair, name, out):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), layout='constrained')
    for row, role in enumerate(('baseline', 'candidate')):
        data = pair[role]
        t, x, y, mask = (data[k] for k in ('t', 'x', 'y', 'common'))
        for col, scale in enumerate((False, True)):
            summary, aligned = fit(x[mask], y[mask], scale)
            axis = axes[row, col]
            line_with_gaps(axis, t[mask], y[mask], color='#208447', lw=1.7, label='Reference cam0')
            line_with_gaps(axis, t[mask], aligned, color='#db732a', lw=1.1, label=role.title()+' primary cam0')
            axis.set_aspect('equal', adjustable='datalim')
            axis.set_title(f'{role.title()} | '+('Diagnostic Sim3' if scale else 'Metric SE3 (scale = 1)')+
                           f'\n3D RMSE {summary["rmse_m"]:.3f} m'+(f' | scale {summary["scale_estimate_to_gt"]:.5f}' if scale else ''))
            axis.set_xlabel('Reference world x (m)'); axis.set_ylabel('Reference world y (m)')
            axis.grid(alpha=.2); axis.legend(fontsize=8)
    fig.suptitle(f'{name.title()}: identical common visual-OK timestamps, primary map only\n'
                 'Rigid/similarity fits are evaluation diagnostics; no corrected trajectory is produced', fontsize=12)
    path = out/f'{name}_top_views.png'
    fig.savefig(path, dpi=150); plt.close(fig)
    return path


def figure_history(pair, name, out):
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), layout='constrained')
    for role, color in (('baseline', '#b66064'), ('candidate', '#317eb4')):
        data = pair[role]
        ws = data['info']['windows_60s']
        axes[0].plot([(w['first_s']+w['last_s'])/2 for w in ws],
                     [w['sim3']['scale_estimate_to_gt'] for w in ws], '.-', color=color, label=role.title())
        offset = .1 if role == 'candidate' else -.1
        for row, segment in enumerate(data['info']['coverage']['segments']):
            for span in segment['contiguous_spans']:
                axes[1].plot([span['start_s'], span['end_s']], [row+offset]*2, lw=5, color=color,
                             label=role.title() if row == 0 and span == segment['contiguous_spans'][0] else None)
        online = data['online']
        coast = [r for r in online if int(r['coasting'])]
        axes[1].scatter([float(r['input_t_s']) for r in coast], [-.5+offset]*len(coast), s=2, color=color)
    axes[0].axhline(1., color='#444', ls='--', lw=1)
    axes[0].set_ylabel('Estimate → reference scale'); axes[0].set_title('Independent 60 s scale diagnostics (primary map)')
    axes[1].set_ylabel('Independent segment ordinal'); axes[1].set_title('Exported time spans; points below map 0 mark coasting')
    for axis in axes:
        axis.set_xlabel('Native sensor time (s)'); axis.grid(alpha=.2); axis.legend()
    path = out/f'{name}_scale_and_coverage.png'
    fig.savefig(path, dpi=140); plt.close(fig)
    return path


def camera_stills(run, stamps, out):
    # Reuse the production-RRD payload decoder used by MATCH reports; no feature
    # extraction, rematching or new association classification is performed.
    from make_lamaria_failure_diagnostics import cameras
    payloads = cameras(run, stamps)
    stats = {}
    for row in read_rows(run/'stats.csv'):
        stamp = round(float(row['t'])*1e9)
        i = int(np.argmin(abs(stamps-stamp)))
        if abs(int(stamps[i])-stamp) <= 1:
            stats[int(stamps[i]), int(row['cam'])] = row
    out.mkdir(parents=True, exist_ok=False)
    panels, evidence = [], []
    for stamp in map(int, stamps):
        views, records = [], []
        for cam, frame in enumerate(payloads[stamp]):
            raw = frame['image']
            picture = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            require(picture is not None, 'Invalid recorded image')
            for strip, color in zip(frame['trails'], frame['trail_colors']):
                points = np.rint(strip).astype(np.int32)
                if len(points) > 1:
                    cv2.polylines(picture, [points], False, tuple(map(int, color[:3][::-1])), 1, cv2.LINE_AA)
            for point, color in zip(frame['pixels'], frame['colors']):
                cv2.circle(picture, tuple(np.rint(point).astype(int)), 2, tuple(map(int, color[:3][::-1])), -1, cv2.LINE_AA)
            mapped = int(np.count_nonzero(np.all(frame['colors'][:, :3] == [40, 230, 80], axis=1)))
            expected = stats[stamp, cam]
            require(mapped == int(expected['mapped']) and len(frame['pixels']) == int(expected['features']),
                    f'Recorded overlay/stat count mismatch at {stamp}, cam{cam}')
            ok, encoded = cv2.imencode('.jpg', picture, [cv2.IMWRITE_JPEG_QUALITY, 94])
            require(ok, 'Failed to encode still')
            path = out/f'{stamp}_cam{cam}.jpg'
            path.write_bytes(encoded.tobytes())
            views.append({'camera': cam, 'mapped': mapped, 'features': len(frame['pixels']), 'path': str(path)})
            records.append({'camera': cam, 'mapped': mapped, 'features': len(frame['pixels']),
                            'source_image_sha256': hashlib.sha256(raw).hexdigest(),
                            'overlay_float32_sha256': hashlib.sha256(frame['pixels'].tobytes()).hexdigest(),
                            'color_uint8_sha256': hashlib.sha256(frame['colors'].tobytes()).hexdigest(),
                            'still_path': str(path), 'still_sha256': sha(path)})
        panels.append({'timestamp_ns': stamp, 'views': views})
        evidence.append({'timestamp_ns': stamp, 'cameras': records})
    return panels, evidence


def verify_recording_export(run, inputs):
    """Validate the actual export, including renders deferred by the launcher.

    render.sh deliberately does not rewrite the launcher's immutable result.
    A null export_returncode therefore means deferred, not failed or finished.
    """
    result = read_json(run/'result.json')
    require(result.get('export_returncode') in (None, 0), 'Launcher recorded an export failure')
    metadata_path, log_path = run/'output_metadata.json', run/'run.rrd.verify.log'
    metadata = read_json(metadata_path)
    require(metadata.get('rrd_verification') == 'passed', 'Recording verification did not pass')
    require(metadata.get('missing_images') == 0, 'Recording contains missing camera images')
    require(metadata.get('images_enabled') is True, 'Recording camera streams were disabled')
    require(metadata.get('input_frames') == len(inputs) and metadata.get('source_input_frames') == len(inputs),
            'Rendered frame count differs from the full requested input')
    require(metadata.get('selection') == 'Full requested input', 'Recording is a selected segment')
    require(relocated_path(metadata['source_run']).resolve() == run.resolve(), 'Recording belongs to another run')
    interval = metadata.get('recording_interval_s', [])
    require(len(interval) == 2 and
            max(abs(round(float(value)*1e9)-int(stamp)) for value, stamp in zip(interval, (inputs[0], inputs[-1]))) <= 1,
            'Recording interval differs from the full requested timestamps')
    require('1 file verified without error.' in log_path.read_text(), 'Rerun verification log lacks a successful result')
    require((run/'run.rrd').is_file() and (run/'run.rrd').stat().st_size > 0, 'Recording file is missing or empty')
    return {'metadata': str(metadata_path), 'verification_log': str(log_path),
            'launcher_export_returncode': result.get('export_returncode'),
            'rendered_frames_per_camera': len(inputs), 'missing_images': 0,
            'rrd_verification': 'passed',
            'sha256': {str(p): sha(p) for p in (metadata_path, log_path)}}


def image_tag(path, alt):
    suffix = 'png' if path.suffix == '.png' else 'jpeg'
    payload = base64.b64encode(path.read_bytes()).decode('ascii')
    return f'<img src="data:image/{suffix};base64,{payload}" alt="{html.escape(alt)}">'


def display(value):
    if value is None:
        return 'Unavailable'
    if isinstance(value, float):
        return f'{value:,.3f}'
    return html.escape(str(value))


def table(headers, rows):
    return '<table><tr>'+''.join('<th>'+html.escape(h)+'</th>' for h in headers)+'</tr>'+''.join(
        '<tr>'+''.join('<td>'+display(v)+'</td>' for v in row)+'</tr>' for row in rows)+'</table>'


def section(name, pair, figures, stills):
    infos = [pair[r]['info'] for r in ('baseline', 'candidate')]
    metric_keys = [('Source frames (full denominator)', 'source_frames'), ('Processed supported frames', 'supported_frames'),
                   ('Excluded IMU-boundary frames', 'unsupported_boundary_frames'), ('Primary-map poses', 'primary_poses'),
                   ('Missing primary-map source poses', 'primary_missing_source_frames'),
                   ('Exported poses across separate maps', 'all_independent_maps_exported_poses'),
                   ('Missing source poses across all maps', 'all_independent_maps_missing_source_frames'),
                   ('Visual-OK input frames', 'visual_ok_frames'), ('Coasted input frames', 'coasted_frames'),
                   ('Independent retained maps', 'independent_maps'), ('New-map events', 'new_map_events'),
                   ('Completed active-map resets', 'completed_active_map_resets')]
    rows = [[label, *[i['metrics'][key] for i in infos]] for label, key in metric_keys]
    rows += [[label, *[100*i['metrics'][key] for i in infos]] for label, key in
             (('Primary-map source coverage (%)', 'primary_source_fraction'),
              ('Separate-map source coverage (%)', 'all_independent_maps_source_fraction'),
              ('Visual-OK source coverage (%)', 'visual_ok_source_fraction'))]
    text = '<section><h2>'+html.escape(name.title())+' · '+html.escape(infos[0]['sequence'])+'</h2>'
    baseline_fit, candidate_fit = [i['fits']['common_visual_ok'] for i in infos]
    def direction(before, after):
        return 'increased' if after > before+1e-10 else ('decreased' if after < before-1e-10 else 'was unchanged')
    metric_direction = direction(baseline_fit['se3']['rmse_m'], candidate_fit['se3']['rmse_m'])
    scale_direction = direction(abs(baseline_fit['sim3']['estimate_stretch']-1), abs(candidate_fit['sim3']['estimate_stretch']-1))
    text += (f'<p class="notice">Retained independent maps: {infos[0]["metrics"]["independent_maps"]} → {infos[1]["metrics"]["independent_maps"]}. '
             f'LaMAria local score (higher is better): {infos[0]["score"]["Score2D"]:.3f} → {infos[1]["score"]["Score2D"]:.3f}. '
             f'On common visual-OK poses, metric SE3 error {metric_direction}: {baseline_fit["se3"]["rmse_m"]:.3f} → {candidate_fit["se3"]["rmse_m"]:.3f} m. '
             f'The magnitude of fitted metric-scale error {scale_direction}. Continuity and metric accuracy are separate outcomes.</p>')
    text += table(['Full-sequence measurement', 'Baseline', 'Observation repair'], rows)
    score_keys = [('LaMAria local score (higher is better)', 'Score2D'), ('Control points: full denominator', 'CP_total'),
                  ('Control points triangulated', 'CP_triangulated'), ('Control points within 1 m', 'CP_within_1m'),
                  ('Control-point recall within 1 m (%)', 'CPRecall@1m_percent'),
                  ('Pose recall within 5 m (%)', 'PoseRecall@5m_percent'), ('Pose recall within 1 m (%)', 'PoseRecall@1m_percent'),
                  ('GT pose timestamps: full denominator', 'GT_total_denominator'), ('Associated GT poses', 'GT_associated'),
                  ('Missing GT poses', 'GT_unmatched_count')]
    text += '<h3>LaMAria score with the full original denominator</h3>'
    text += table(['Local evaluation', 'Baseline', 'Observation repair'],
                  [[label, *[i['score'][key] for i in infos]] for label, key in score_keys])
    text += '<p>Missing GT timestamps remain failures in the full scoring denominator. The score uses one control-point-derived Sim3, including a fitted global scale, and reuses it for pose errors. Uniform global rescaling is already absorbed by this alignment; score regression cannot be attributed to a single scale factor alone. The dense-GT SE3/Sim3 diagnostics below are separate fits, not replacements for that score alignment. Poses from independent maps count toward exported coverage but are never joined or added to the primary-map score.</p>'
    text += '<h3>Metric scale on exactly the same visually tracked timestamps</h3>'
    text += table(['Run', 'Common poses', 'SE3 RMSE (m)', 'Diagnostic scale', 'Stretch (%)', 'Sim3 RMSE (m)'],
                  [[role, info['fits']['common_visual_ok']['poses'], info['fits']['common_visual_ok']['se3']['rmse_m'],
                    info['fits']['common_visual_ok']['sim3']['scale_estimate_to_gt'],
                    100*(info['fits']['common_visual_ok']['sim3']['estimate_stretch']-1),
                    info['fits']['common_visual_ok']['sim3']['rmse_m']] for role, info in zip(('Baseline', 'Observation repair'), infos)])
    text += '<p>SE3 rotates and translates the raw metric estimate without changing scale. Sim3 also fits one scale to dense reference positions: this is a diagnostic only, distinct from the official score and never fed into the estimator. This overlap-only comparison excludes coasting; full-primary and all visual-OK fits are saved in comparison.json.</p>'
    text += '<h3>All available primary-map poses: different time coverage</h3>'
    text += table(['Run', 'GT-associated poses', 'First s', 'Last s', 'SE3 RMSE (m)', 'Diagnostic scale', 'Sim3 RMSE (m)'],
                  [[role, info['fits']['all_primary']['poses'], info['fits']['all_primary']['first_s'], info['fits']['all_primary']['last_s'],
                    info['fits']['all_primary']['se3']['rmse_m'], info['fits']['all_primary']['sim3']['scale_estimate_to_gt'],
                    info['fits']['all_primary']['sim3']['rmse_m']] for role, info in zip(('Baseline', 'Observation repair'), infos)])
    text += '<p>This second diagnostic includes available coasting poses and each run’s entire primary-map interval. The intervals differ when the baseline lost its first map. These values are not the equal-time comparison above and do not replace the full-denominator score.</p>'
    text += image_tag(figures[0], name+' primary map top views')
    text += image_tag(figures[1], name+' scale and independent coverage')
    text += '<p>Local scale fits use separate 60 s windows with at least 30 visual-OK matches, no gap over 2 s, and reference RMS spread at least 3 m. Window fits are not a piecewise corrected trajectory. Segment ordinals identify separate coordinate systems, not positions on a common map.</p>'
    text += '<h3>Measured observation-graph audit</h3>'
    text += table(['Live graph status', 'Baseline', 'Observation repair'],
                  [['All exported live-error counters zero', *[i['graph_audit'].get('live_graph_zero_violations') for i in infos]],
                   ['Excluded stale slots referencing bad points', *[i['graph_audit'].get('stale_bad_point_slots') for i in infos]],
                   *[[f'Cam{cam}: observations minus live slots', *[i['graph_audit'].get('observation_minus_slot_counts', {}).get(f'cam{cam}') for i in infos]] for cam in (0, 1)]])
    text += table(['Final graph counter', 'Baseline', 'Observation repair'],
                  [[k, *[i['graph_audit'].get('totals', {}).get(k) for i in infos]] for k in GRAPH_COUNTS+GRAPH_ERRORS[:-1]])
    text += '<p>The baseline did not export this exact audit, so absent counters are unavailable, never zero. These checks measure final live observation-graph consistency after backend shutdown. Stale keyframe slots referencing points already flagged bad are counted separately; they are excluded from valid matching and BA. Zero live violations alone does not establish accurate geometry.</p>'
    if any(any(v != 0 for v in i['graph_audit'].get('observation_minus_slot_counts', {}).values()) for i in infos):
        text += '<p class="notice">Observation totals and live feature-slot totals do not exactly match in this run, despite the listed per-edge error counters. The nonzero deltas above remain an audit discrepancy; this report does not claim exact bidirectional equality.</p>'
    text += '<details><summary>Independent map scopes</summary>'+table(['Run', 'Coordinate frame', 'Exported poses', 'First s', 'Last s'],
             [[role, m['coordinate_frame'], int(m['exported_poses']), float(m['first_t_s']), float(m['last_t_s'])]
              for role, info in zip(('Baseline', 'Observation repair'), infos) for m in info['map_manifest']])+'</details>'
    if stills:
        text += '<h3>Same camera frames, actual recorded associations</h3><p>Green = mapped association; red = unmapped feature. A green association is not necessarily an optimization inlier. The displayed images and feature coordinates come from each run’s recording; no features were rematched or recolored. Original camera display rotations are retained.</p>'
        for index, sample in enumerate(stills['baseline']):
            text += f'<h4>Native time {sample["timestamp_ns"]*1e-9:.9f} s</h4><div class="pair">'
            for role in ('baseline', 'candidate'):
                text += '<div><h4>'+('Baseline' if role == 'baseline' else 'Observation repair')+'</h4><div class="cameras">'
                for view in stills[role][index]['views']:
                    text += '<div>'+image_tag(Path(view['path']), f'{role} camera {view["camera"]}')+f'<small>cam{view["camera"]}: {view["mapped"]}/{view["features"]} mapped</small></div>'
                text += '</div></div>'
            text += '</div>'
    text += '<p class="scope">Recording scope: run.rrd shows the selected largest map’s final geometry revealed over time. Independent maps remain separate in atlas_overview.rrd; this report never registers or joins them.</p>'
    text += '<details><summary>Source run paths</summary>'+''.join('<pre>'+html.escape(i['run'])+'</pre>' for i in infos)+'</details></section>'
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', action='append', required=True, metavar='SUITE_NAME=RUN_PATH')
    parser.add_argument('--suite', type=relocated_path, default=REGISTRY)
    parser.add_argument('--out', type=relocated_path, required=True)
    parser.add_argument('--no-stills', action='store_true', help='Explicit incomplete numerical preview; final reports require RRD stills')
    args = parser.parse_args()
    registry = read_json(args.suite)
    candidates = {}
    for spec in args.candidate:
        name, separator, path = spec.partition('=')
        require(separator and name in registry['sequences'] and name not in candidates, 'Use unique registry_name=run_path entries')
        candidates[name] = Path(path)
    out = args.out.resolve()
    require(not out.exists(), f'Output exists; preserve it and choose a new path: {out}')
    loaded = {}
    # Validate source inputs before creating an output directory.
    for name, candidate in candidates.items():
        entry = registry['sequences'][name]
        pair = {'baseline': load_case(entry, Path(entry['baseline'])), 'candidate': load_case(entry, candidate)}
        a, b = pair['baseline'], pair['candidate']
        if not args.no_stills:
            require(b['info']['graph_audit']['available'], 'Candidate final observation-graph audit is missing')
        require(a['info']['score']['GT_total_denominator'] == b['info']['score']['GT_total_denominator'], 'Different GT denominators')
        common = np.intersect1d(a['gt_stamps'][a['good']], b['gt_stamps'][b['good']])
        require(len(common) >= 3, 'No meaningful common visual-OK primary-map interval')
        for data in pair.values():
            data['common'] = np.isin(data['gt_stamps'], common) & data['good']
            data['info']['fits']['common_visual_ok'] = summarize(data['t'], data['x'], data['y'], data['common'])
            require(data['info']['fits']['common_visual_ok'] is not None, 'Degenerate common trajectory')
        require(np.array_equal(a['y'][a['common']], b['y'][b['common']]), 'Common reference positions differ')
        loaded[name] = pair
    out.mkdir(parents=True)
    report = {'artifact_kind': 'Offline candidate-vs-baseline diagnostics; not an estimator run',
              'complete_with_camera_evidence': not args.no_stills, 'estimator_outputs_modified': False,
              'independent_maps_joined': False, 'scale_method_reference': str(SCALE_REFERENCE),
              'fit_scope': 'Raw cam0 poses. Identical dense-GT associations, proper rotation, optional one positive global scale.',
              'cases': {}, 'camera_evidence': {}}
    sections = []
    for name, pair in loaded.items():
        figures = [figure_pair(pair, name, out), figure_history(pair, name, out)]
        stills, evidence = {}, {}
        if not args.no_stills:
            entry = registry['sequences'][name]
            events = entry.get('events', [])
            center = events[0]['start_ns'] if events else int(pair['baseline']['inputs'][len(pair['baseline']['inputs'])//2])
            targets = np.array([center-10_000_000_000, center, center+10_000_000_000], dtype=np.int64)
            ids, _ = nearest(pair['baseline']['inputs'], targets)
            selected = np.unique(pair['baseline']['inputs'][ids])
            for role, data in pair.items():
                run = Path(data['info']['run'])
                export = verify_recording_export(run, data['inputs'])
                stills[role], evidence[role] = camera_stills(run, selected, out/f'{name}_{role}_stills')
                data['info']['sha256'][str(run/'stats.csv')] = sha(run/'stats.csv')
                data['info']['sha256'].update(export['sha256'])
                data['info']['recording'] = {'path': str(run/'run.rrd'), 'bytes': (run/'run.rrd').stat().st_size,
                                           'sample_source_hashes': 'camera_evidence; original compressed images and overlays',
                                           'export_validation': export}
            require([[r['source_image_sha256'] for r in p['cameras']] for p in evidence['baseline']] ==
                    [[r['source_image_sha256'] for r in p['cameras']] for p in evidence['candidate']],
                    'Before/after recording images differ at the selected native times')
            report['camera_evidence'][name] = evidence
        report['cases'][name] = {role: data['info'] for role, data in pair.items()}
        sections.append(section(name, pair, figures, stills))
        for role, data in pair.items():
            with (out/f'{name}_{role}_matches.csv').open('w') as handle:
                writer = csv.writer(handle)
                writer.writerow(['time_s', 'gt_timestamp_ns', 'visual_ok', 'common_visual_ok', 'cam0_x', 'cam0_y', 'cam0_z', 'gt_x', 'gt_y', 'gt_z'])
                writer.writerows([float(t), int(gt), int(ok), int(common), *x, *y]
                                 for t, gt, ok, common, x, y in zip(data['t'], data['gt_stamps'], data['good'], data['common'], data['x'], data['y']))
    provenance = {'generator': str(Path(__file__).resolve()), 'generator_sha256': sha(__file__),
                  'suite': str(args.suite.resolve()), 'suite_sha256': sha(args.suite),
                  'scale_reference': str(SCALE_REFERENCE),
                  'scale_reference_sha256': sha(SCALE_REFERENCE) if SCALE_REFERENCE.exists() else None,
                  'camera_decoder': str(REPO/'pipeline/viz/make_lamaria_failure_diagnostics.py'),
                  'camera_decoder_sha256': sha(REPO/'pipeline/viz/make_lamaria_failure_diagnostics.py'),
                  'source_hashes': {name: {role: data['info']['sha256'] for role, data in pair.items()} for name, pair in loaded.items()}}
    title = 'Observation repair: baseline comparison'
    document = '<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>body{font:16px system-ui;max-width:1300px;margin:25px auto;padding:0 20px;color:#24303d;background:#f6f8fa}h1,h2,h3{line-height:1.25}section{padding:22px;background:white;border:1px solid #d8dfe8;border-radius:9px;margin:24px 0}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border-bottom:1px solid #dfe4ec;padding:8px;text-align:right}td:first-child,th:first-child{text-align:left}img{width:100%;height:auto}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}.pair,.cameras{display:grid;grid-template-columns:1fr 1fr;gap:10px}.notice{background:#fff1d6;padding:14px;border-left:4px solid #cb8412}.scope,small{color:#617083;font-size:13px}summary{cursor:pointer}h4{margin-bottom:8px}@media(max-width:850px){.pair{grid-template-columns:1fr}body{padding:0 10px}section{padding:12px}}</style><h1>'+title+'</h1><p class="notice">'+('NUMERICAL PREVIEW: camera comparisons omitted. ' if args.no_stills else '')+'These are completed-run measurements, not a GT-corrected estimator or a claim that all geometry is fixed. Compare coverage and fragmentation together with error and final graph consistency.</p>'+''.join(sections)+'<p>Full source provenance, fit matrices, per-map counters and sample image hashes: comparison.json and provenance.json beside this report. Sources are read-only. The HTML opens offline; plots and camera evidence are embedded.</p></html>'
    (out/'report.html').write_text(document)
    (out/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (out/'provenance.json').write_text(json.dumps(provenance, indent=2, allow_nan=False)+'\n')
    validation = {'passed': True, 'full_source_denominators_equal': True,
                  'full_supported_inputs_equal': True, 'same_common_visual_ok_gt_associations': True,
                  'factory_cam0_extrinsics_verified': True, 'score_matches_scale_associations': True,
                  'camera_evidence_verified': not args.no_stills, 'same_native_images_verified': not args.no_stills,
                  'independent_maps_joined': False, 'camera_pairs_per_run': 0 if args.no_stills else 3,
                  'live_graph_zero_violations': {name: pair['candidate']['info']['graph_audit'].get('live_graph_zero_violations') for name, pair in loaded.items()},
                  'stale_bad_point_slots': {name: pair['candidate']['info']['graph_audit'].get('stale_bad_point_slots') for name, pair in loaded.items()},
                  'artifacts_sha256': {str(p.relative_to(out)): sha(p) for p in out.rglob('*') if p.is_file()}}
    (out/'validation.json').write_text(json.dumps(validation, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'report': str(out/'report.html'), 'cases': list(loaded), 'validation': str(out/'validation.json')}))


if __name__ == '__main__':
    main()
