#!/usr/bin/env python3
"""Report metric-initializer telemetry and evaluate saved poses offline.

GT is never fed to SLAM. Top views use rigid SE3 alignment with scale fixed to
one. Separate diagnostic Sim3 fits measure scale; they do not correct outputs.
Online fits never cross either run's map/init/world-version boundary.
"""
import argparse
import csv
from decimal import Decimal
import hashlib
import html
import io
import json
from pathlib import Path
import re

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from audit_lamaria_scale import fit, read_rows


def timestamp_ns(value):
    return int((Decimal(str(value)) * 1_000_000_000).to_integral_value())


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def one_file(directory, pattern):
    paths = sorted(Path(directory).glob(pattern))
    if len(paths) != 1:
        raise ValueError(f'Expected one {pattern} in {directory}; found {len(paths)}')
    return paths[0]


def scalar(value):
    if re.fullmatch(r'[+-]?\d+', value):
        return int(value)
    try:
        number = float(value)
    except ValueError:
        return value
    return number if np.isfinite(number) else None


def parse_metric_records(path):
    """Keep proposals, completed stages and rejected commits distinct."""
    records, malformed, bootstrap = [], [], []
    marker_count = 0
    with Path(path).open(errors='replace') as handle:
        for line_number, line in enumerate(handle, 1):
            rig = re.search(r'\[METRIC-RIG\] scale=(\S+) inlier_landmarks=(\d+) '
                            r'reprojection_median_px=(\S+) log_scale_sigma=(\S+)', line)
            if rig:
                bootstrap.append({'line': line_number, 'bootstrap_units_to_metres': scalar(rig[1]),
                                  'landmarks': int(rig[2]), 'median_reprojection_px': scalar(rig[3]),
                                  'log_scale_sigma': scalar(rig[4])})
            for match in re.finditer(r'\[(METRIC-INIT|JOINT-INIT)\]([^\n]*?)(?=\[(?:METRIC-INIT|JOINT-INIT)\]|$)', line.rstrip('\n')):
                marker_count += 1
                payload = match[2].strip()
                prefix, separator, reason = payload.partition(' reason=')
                fields = {key: scalar(value) for key, value in re.findall(r'([A-Za-z_][A-Za-z_0-9]*)=([^\s]+)', prefix)}
                if separator:
                    fields['reason'] = reason
                fields['line'] = line_number
                if match[1] == 'JOINT-INIT':
                    kind, required = 'joint', ('t', 'stage', 'refined', 'accepted', 'points')
                elif 'completed_stage' in fields:
                    kind, required = 'completed', ('t', 'map', 'completed_stage', 'committed_scale', 'world_version')
                elif 'accepted' in fields:
                    kind, required = 'proposal', ('t', 'map', 'stage', 'accepted', 'scale')
                elif 'commit' in fields:
                    kind, required = 'commit', ('t', 'commit')
                else:
                    kind, required = 'unknown', ('t',)
                if kind == 'unknown' or any(key not in fields for key in required):
                    malformed.append({'line': line_number, 'text': payload[:600]})
                    continue
                if not isinstance(fields['t'], (int, float)):
                    malformed.append({'line': line_number, 'text': payload[:600]})
                    continue
                fields['kind'] = kind
                if kind == 'proposal' and fields.get('sigma') is None:
                    fields['conditioning_status'] = ('not computed: rejected by earlier stereo-consistency gate'
                        if fields.get('reason') == 'stereo consistency rejected metric correction'
                        else 'unavailable; nonfinite telemetry does not by itself establish unobservability')
                records.append(fields)
    proposals = [row for row in records if row['kind'] == 'proposal']
    return {'records': records, 'bootstrap_records': bootstrap,
            'parse_report': {'markers': marker_count, 'parsed': len(records), 'malformed': malformed,
                             'proposals': len(proposals),
                             'accepted_proposals': sum(row['accepted'] == 1 for row in proposals),
                             'rejected_proposals': sum(row['accepted'] == 0 for row in proposals),
                             'completed_stages': sum(row['kind'] == 'completed' for row in records),
                             'joint_records': sum(row['kind'] == 'joint' for row in records),
                             'commit_records': sum(row['kind'] == 'commit' for row in records)}}


def infer_assets(run):
    for name in ('result.json', 'command.json'):
        path = run / name
        if not path.exists():
            continue
        data = json.loads(path.read_text())
        command = data.get('export_command') or []
        if '--gt' in command:
            return Path(command[command.index('--gt') + 1]).parent
        vrs = data.get('imu_manifest', {}).get('vrs')
        if vrs:
            return Path(vrs).parent
    raise ValueError('Cannot infer sequence assets from result/command.json; supply --assets')


def input_provenance(run, online):
    """Read saved input identities; never infer equal input from fitted poses."""
    manifest_path = run / 'command.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    timestamps_path = run / 'config/timestamps_ns.txt'
    stamps = None
    if timestamps_path.exists():
        stamps = np.loadtxt(timestamps_path, dtype=np.int64, ndmin=1)
        if not len(stamps) or np.any(np.diff(stamps) <= 0):
            raise ValueError(f'Input timestamps must be nonempty, unique and ordered: {timestamps_path}')
    command = manifest.get('command', [])
    environment = {}
    for index, token in enumerate(command[:-1]):
        if token == '-e' and '=' in command[index+1]:
            key, value = command[index+1].split('=', 1)
            environment[key] = value
    export = manifest.get('export_command', [])
    dataset = export[export.index('--dataset')+1] if '--dataset' in export else None
    build = manifest.get('build_manifest', {})
    result_path = run / 'result.json'
    result = json.loads(result_path.read_text()) if result_path.exists() else {}
    return {'input_timestamps_sha256': hashlib.sha256(stamps.astype('<i8').tobytes()).hexdigest() if stamps is not None else None,
            'input_frames': len(stamps) if stamps is not None else None,
            'first_timestamp_ns': int(stamps[0]) if stamps is not None else None,
            'last_timestamp_ns': int(stamps[-1]) if stamps is not None else None,
            'online_covers_exact_input': bool(stamps is not None and np.array_equal(stamps, list(online))),
            'feature_directories': manifest.get('feature_directories'), 'image_dataset': dataset,
            'recorded_calibrated_imu_sha256': manifest.get('imu_manifest', {}).get('output_sha256'),
            'settings_sha256': sha256(run/'config/settings.yaml'),
            'library_sha256': build.get('library_sha256'), 'runner_sha256': build.get('runner_sha256'),
            'environment': environment,
            'cpu_limit': command[command.index('--cpus')+1] if '--cpus' in command else None,
            'returncode': result.get('returncode'), 'raw_export_returncode': result.get('raw_export_returncode'),
            'manifest_sha256': sha256(manifest_path) if manifest_path.exists() else None,
            'evidence_limit': 'Image and feature source paths identify shared inputs; individual image/feature bytes are not rehashed here. IMU hash is the recorded preparation hash.'}


def compare_input_controls(run, baseline):
    """Classify the input-length confound separately from estimator changes."""
    def same_known(key):
        return run.get(key) is not None and baseline.get(key) is not None and run[key] == baseline[key]
    same_timestamps = same_known('input_timestamps_sha256') and same_known('input_frames')
    same_sources = all(bool(run.get(key)) and same_known(key) for key in
                       ('feature_directories', 'image_dataset', 'recorded_calibrated_imu_sha256'))
    complete = all(data.get('online_covers_exact_input') and data.get('returncode') == 0 and
                   data.get('raw_export_returncode') == 0 for data in (run, baseline))
    if same_timestamps and same_sources and complete:
        label = 'Equal processed input scope and recorded input sources'
        note = ('Both completed runs consumed the same full timestamp sequence and share recorded image/feature sources and calibrated IMU hash. '
                'The full-run-versus-prefix future-BA confound is absent; thread scheduling and run-to-run variation remain uncontrolled.')
    elif same_timestamps:
        label = 'Equal requested timestamps; complete equal-input control not established'
        note = 'Requested timestamps match, but source identity or successful processing of every input is unverified.'
    else:
        label = 'Unequal or unverified processed input scope'
        note = ('Exact evaluation timestamps do not establish equal input length. A longer run\'s final prefix can benefit from future BA; '
                'this comparison must not be called an equal-input control.')
    return {'label': label, 'interpretation': note, 'same_requested_timestamps': same_timestamps,
            'same_recorded_input_sources': same_sources, 'both_complete_input_traces': bool(complete),
            'equal_input_control': bool(same_timestamps and same_sources and complete),
            'same_settings_bytes': same_known('settings_sha256'),
            'same_environment': same_known('environment'), 'same_cpu_limit': same_known('cpu_limit'),
            'same_library': same_known('library_sha256'), 'same_runner': same_known('runner_sha256'),
            'run': run, 'baseline': baseline}


def load_saved_run(run, assets):
    run = Path(run)
    config = run / 'config/settings.yaml'
    calib = json.loads((assets / 'aria_calib.json').read_text())['cam0']['T_b_s']
    tbc = np.eye(4)
    tbc[:3, :3] = Rotation.from_quat(calib['qvec']).as_matrix()
    tbc[:3, 3] = calib['tvec']
    settings = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
    configured = settings.getNode('IMU.T_b_c1').mat()
    settings.release()
    if configured is None or not np.allclose(configured, tbc, atol=1e-6, rtol=0):
        raise ValueError(f'Run IMU-to-cam0 calibration disagrees with supplied assets: {run}')

    def camera_position(row):
        translation = np.array([float(row[key]) for key in ('tx', 'ty', 'tz')])
        quat = np.array([float(row[key]) for key in ('qx', 'qy', 'qz', 'qw')])
        if not np.isfinite(translation).all() or not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1) > 1e-5:
            raise ValueError(f'Invalid body pose in {run}: {row}')
        return translation + Rotation.from_quat(quat).apply(tbc[:3, 3])

    online_path = one_file(run, 'online_*.csv')
    online = {}
    for row in read_rows(online_path):
        stamp = timestamp_ns(row['input_t_s'])
        if stamp in online:
            raise ValueError(f'Duplicate online timestamp: {stamp}')
        available = row['pose_available'] == '1'
        if available and row['pose_kind'] != 'body_to_current_map':
            raise ValueError(f'Unsupported online pose convention: {row["pose_kind"]}')
        online[stamp] = {'gauge': tuple(int(row[key]) for key in ('map_id', 'map_init_kf_id', 'world_version')),
                         'state': int(row['state']), 'coasting': int(row['coasting']),
                         'imu_initialized': int(row['imu_initialized']), 'available': available,
                         'good': available and row['state'] == '2' and row['coasting'] == '0',
                         'position': camera_position(row) if available else None}
    final_path = one_file(run, 'atlas_*_trajectory.csv')
    frames = {}
    for row in read_rows(final_path):
        frames.setdefault(row['coordinate_frame'], []).append(row)
    if not frames:
        raise ValueError(f'No final exported poses: {run}')
    selected = min(frames, key=lambda name: (-len(frames[name]), name))
    if not selected.endswith('_body0'):
        raise ValueError(f'Unsupported final pose convention: {selected}')
    final = {}
    for row in frames[selected]:
        stamp = timestamp_ns(row['t_s'])
        if stamp in final:
            raise ValueError(f'Duplicate final timestamp in selected map: {stamp}')
        final[stamp] = camera_position(row)
    telemetry = parse_metric_records(run / 'run.log')
    return {'run': str(run), 'online': online, 'final': final, 'telemetry': telemetry,
            'metadata': {'coordinate_frame': selected, 'final_map_pose_counts': {key: len(value) for key, value in frames.items()},
                         'selected_final_poses': len(final), 'native_online_rows': len(online),
                         'imu_initialized_rows': sum(row['imu_initialized'] for row in online.values()),
                         'first_input_timestamp_ns': min(online), 'last_input_timestamp_ns': max(online),
                         'input_provenance': input_provenance(run, online),
                         'calibration_max_abs_difference': float(np.max(np.abs(configured-tbc))),
                         'files_sha256': {str(path): sha256(path) for path in (config, online_path, final_path, run/'run.log')}}}


def select_common(run, baseline, gt, start=None, end=None):
    """Only exact common native timestamps, visual OK in both selected maps."""
    stamps = sorted(set(run['online']) & set(baseline['online']) & set(run['final']) & set(baseline['final']) & set(gt))
    selected = []
    counts = {'exact_common_before_state_filter': len(stamps), 'excluded_state_or_coasting': 0, 'excluded_time_range': 0}
    for stamp in stamps:
        a, b = run['online'][stamp], baseline['online'][stamp]
        if not a['good'] or not b['good']:
            counts['excluded_state_or_coasting'] += 1
            continue
        if (start is not None and stamp < timestamp_ns(start)) or (end is not None and stamp > timestamp_ns(end)):
            counts['excluded_time_range'] += 1
            continue
        selected.append(stamp)
    if len(selected) < 3:
        raise ValueError(f'Need at least three exact common visual-OK GT timestamps; got {len(selected)}')
    return np.array(selected, dtype=np.int64), counts


def summary_fit(x, y, stamps):
    spread = float(np.sqrt(np.mean(np.sum((y-y.mean(axis=0))**2, axis=1)))) if len(y) else 0.
    result = {'poses': len(y), 'first_timestamp_ns': int(stamps[0]) if len(stamps) else None,
              'last_timestamp_ns': int(stamps[-1]) if len(stamps) else None,
              'gt_rms_spread_m': spread, 'motion_support_sufficient': bool(len(y) >= 10 and spread >= .5)}
    if len(y) < 3 or spread < 1e-4 or np.sum((x-x.mean(axis=0))**2) < 1e-8:
        result['unavailable'] = 'Insufficient independent position samples or displacement'
        return result
    result['se3_scale_fixed_1'], _ = fit(x, y, False)
    result['diagnostic_sim3'], _ = fit(x, y, True)
    result['gt_singular_values'] = np.linalg.svd(y-y.mean(axis=0), compute_uv=False).tolist()
    return result


def comparison_groups(stamps, run, baseline):
    """Partition on changes in either run's online map/init/world version."""
    groups = []
    for index, stamp in enumerate(stamps):
        key = (run['online'][int(stamp)]['gauge'], baseline['online'][int(stamp)]['gauge'])
        if not groups or groups[-1]['key'] != key:
            groups.append({'key': key, 'indices': []})
        groups[-1]['indices'].append(index)
    return groups


def build_comparison(stamps, run, baseline, gt):
    y = np.array([gt[int(stamp)] for stamp in stamps])
    arrays = {}
    for label, data in (('run', run), ('baseline', baseline)):
        arrays[label+'_online'] = np.array([data['online'][int(stamp)]['position'] for stamp in stamps])
        arrays[label+'_final'] = np.array([data['final'][int(stamp)] for stamp in stamps])
    groups = comparison_groups(stamps, run, baseline)
    epochs, windows = [], []
    for group_index, group in enumerate(groups):
        indices = np.array(group['indices'])
        common = {'group': group_index, 'run_gauge': list(group['key'][0]), 'baseline_gauge': list(group['key'][1])}
        epochs.append({**common, 'fits': {label: summary_fit(x[indices], y[indices], stamps[indices]) for label, x in arrays.items()}})
        seconds = stamps[indices]/1e9
        for width in (10, 30):
            for first in np.arange(seconds[0], seconds[-1]-width, width/2):
                subset = indices[(seconds >= first) & (seconds < first+width)]
                if len(subset) < 10 or np.max(np.diff(stamps[subset])) > 2_000_000_000:
                    continue
                spread = float(np.sqrt(np.mean(np.sum((y[subset]-y[subset].mean(axis=0))**2, axis=1))))
                if spread < (.5 if width == 10 else 3.):
                    continue
                windows.append({**common, 'width_s': width, 'mid_timestamp_s': float(np.mean(stamps[subset][[0, -1]])/1e9),
                                'fits': {label: summary_fit(x[subset], y[subset], stamps[subset]) for label, x in arrays.items()}})
    final_summary = {label: summary_fit(arrays[label+'_final'], y, stamps) for label in ('run', 'baseline')}
    return arrays, y, epochs, windows, final_summary


def save_figure(figure, output, name):
    figure.savefig(output / (name+'.png'), dpi=150)
    svg = io.StringIO()
    figure.savefig(svg, format='svg')
    plt.close(figure)
    (output / (name+'.svg')).write_text(svg.getvalue())
    return svg.getvalue()


def make_figures(stamps, arrays, y, epochs, windows, telemetry, output):
    figures = {}
    figure, axes = plt.subplots(2, 2, figsize=(14, 10), layout='constrained')
    for label, color in (('run', '#d95f02'), ('baseline', '#1b6ca8')):
        info, aligned = fit(arrays[label+'_final'], y, False)
        axes[0, 0].plot(aligned[:, 0], aligned[:, 1], label=f'{label}: SE3 scale=1', color=color)
        axes[0, 1].plot(stamps/1e9, np.linalg.norm(aligned-y, axis=1), label=f'{label}: same final poses', color=color)
    axes[0, 0].plot(y[:, 0], y[:, 1], color='#238443', label='GT cam0', linewidth=1.5)
    axes[0, 0].set_aspect('equal', adjustable='datalim')
    axes[0, 0].set(xlabel='GT world X (m)', ylabel='GT world Y (m)', title='Final trajectories: rigid alignment only, no GT rescaling')
    axes[0, 1].set(xlabel='Native timestamp (s)', ylabel='3D error (m)', title='SE3-aligned error on exact common timestamps')
    for axis, width in zip(axes[1], (10, 30)):
        for label, color in (('run', '#d95f02'), ('baseline', '#1b6ca8')):
            for source, style in (('online', '-'), ('final', '--')):
                for index, group in enumerate(sorted({row['group'] for row in windows})):
                    rows = [row for row in windows if row['width_s'] == width and row['group'] == group]
                    axis.plot([row['mid_timestamp_s'] for row in rows],
                              [row['fits'][label+'_'+source]['diagnostic_sim3']['scale_estimate_to_gt'] for row in rows],
                              style, color=color, label=f'{label} {source}' if index == 0 else None)
        axis.axhline(1., color='black', linestyle=':', linewidth=1)
        axis.set(xlabel='Native timestamp (s)', ylabel='Scale multiplying estimate to match GT',
                 title=f'Diagnostic {width}s fits; separate map/version groups')
    for axis in axes.flat:
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    figure.suptitle('Offline evaluation only — final and online estimates remain unchanged')
    figures['trajectory_comparison'] = save_figure(figure, output, 'trajectory_comparison')

    figure, axes = plt.subplots(2, 2, figsize=(14, 9), layout='constrained')
    proposals = [row for row in telemetry['records'] if row['kind'] == 'proposal']
    completions = [row for row in telemetry['records'] if row['kind'] == 'completed']
    joint = [row for row in telemetry['records'] if row['kind'] == 'joint']
    for accepted, color, label in ((0, '#c22d35', 'Rejected proposal'), (1, '#238443', 'Accepted proposal (not necessarily committed)')):
        rows = [row for row in proposals if row['accepted'] == accepted and isinstance(row.get('scale'), (int, float))]
        axes[0, 0].scatter([row['t'] for row in rows], [row['scale'] for row in rows], c=color, s=25, label=label)
    axes[0, 0].scatter([row['t'] for row in completions], [row['committed_scale'] for row in completions],
                       c='#252525', marker='x', s=60, label='Completed stage')
    axes[0, 0].axhline(1, color='black', ls=':', lw=1)
    axes[0, 0].set(title='Scalar seed / global map correction' if joint else 'Proposed incremental metric correction',
                   xlabel='Native timestamp (s)', ylabel='Scale correction applied to current map')
    for key, label in (('sigma', 'Conditional log-scale sigma'), ('imu_sigma', 'IMU-only log-scale sigma')):
        rows = [row for row in proposals if isinstance(row.get(key), (float, int)) and row[key] > 0]
        axes[0, 1].plot([row['t'] for row in rows], [row[key] for row in rows], '.-', label=label)
    rows = [row for row in joint if isinstance(row.get('baseline_sigma'), (float, int)) and row['baseline_sigma'] > 0]
    axes[0, 1].plot([row['t'] for row in rows], [row['baseline_sigma'] for row in rows], 'x-', label='Joint log-baseline sigma')
    if proposals and not any(isinstance(row.get('sigma'), (int, float)) for row in proposals):
        note = ('Not computed: proposals rejected at stereo gate' if all(row.get('reason') ==
                'stereo consistency rejected metric correction' for row in proposals)
                else 'Conditioning unavailable; see each proposal rejection reason')
        axes[0, 1].text(.5, .5, note,
                        transform=axes[0, 1].transAxes, ha='center', va='center')
    axes[0, 1].set_yscale('log')
    axes[0, 1].set(title='Conditioning estimates; nonfinite values omitted', xlabel='Native timestamp (s)', ylabel='Modeled log-scale standard deviation')
    for key, label in (('stereo_landmarks', 'Independent stereo landmarks'), ('stereo_inliers', 'Stereo inlier landmarks'), ('stereo_kfs', 'Stereo keyframes')):
        rows = [row for row in proposals if isinstance(row.get(key), (float, int))]
        axes[1, 0].plot([row['t'] for row in rows], [row[key] for row in rows], '.-', label=label)
    axes[1, 0].set(title='Available stereo constraints', xlabel='Native timestamp (s)', ylabel='Count')
    for key, label in (('stereo_before', 'Stereo median before'), ('stereo_after', 'Stereo median after')):
        rows = [row for row in proposals if isinstance(row.get(key), (float, int))]
        axes[1, 1].plot([row['t'] for row in rows], [row[key] for row in rows], '.-', label=label)
    axes[1, 1].set(title='Scalar-seed stereo residuals (not final joint geometry)' if joint else 'Stereo consistency of each proposal',
                   xlabel='Native timestamp (s)', ylabel='Median reprojection error (px)')
    for axis in axes.flat:
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
        if telemetry['records']:
            times = [row['t'] for row in telemetry['records']]
            margin = max(.5, (max(times)-min(times))*.03)
            axis.set_xlim(min(times)-margin, max(times)+margin)
        if not proposals:
            axis.text(.5, .5, 'No METRIC-INIT proposal records in this run', transform=axis.transAxes, ha='center', wrap=True)
    figure.suptitle('Initializer telemetry — acceptance gates are reported, not inferred from GT')
    figures['initializer_telemetry'] = save_figure(figure, output, 'initializer_telemetry')
    return figures


def render_html(report, figures):
    def display(value):
        return 'unavailable' if value is None else html.escape(str(value))
    proposal_rows = []
    joint_rows = []
    for row in report['run_telemetry']['records']:
        if row['kind'] == 'joint':
            joint_rows.append('<tr>'+''.join(f'<td>{display(row.get(key))}</td>' for key in
                ('t', 'stage', 'refined', 'accepted', 'points', 'boundary_kfs', 'obs0', 'obs1',
                 'before0', 'after0', 'before1', 'after1', 'inliers_before0', 'inliers_after0',
                 'inliers_before1', 'inliers_after1', 'baseline_sigma', 'reason'))+'</tr>')
            continue
        proposal_rows.append('<tr>'+''.join(f'<td>{display(row.get(key))}</td>' for key in
            ('t', 'kind', 'map', 'stage', 'accepted', 'scale', 'sigma', 'imu_sigma', 'completed_stage', 'committed_scale', 'reason'))+'</tr>')
    epoch_rows = []
    for epoch in report['epochs']:
        source = epoch['fits']['run_online']
        columns = [str(epoch['group']), str(epoch['run_gauge']), str(epoch['baseline_gauge']), str(source['poses']),
                   f'{source["first_timestamp_ns"]/1e9:.9f} – {source["last_timestamp_ns"]/1e9:.9f}',
                   f'{source["gt_rms_spread_m"]:.3f}']
        for key in ('run_online', 'baseline_online', 'run_final', 'baseline_final'):
            scale = epoch['fits'][key].get('diagnostic_sim3', {}).get('scale_estimate_to_gt')
            columns.append(f'{scale:.5f}' if scale is not None else 'unavailable')
        columns.append('Adequate count/motion' if source['motion_support_sufficient'] else 'Weak: small motion/sample count')
        epoch_rows.append('<tr>'+''.join(f'<td>{html.escape(value)}</td>' for value in columns)+'</tr>')
    warnings = ''.join(f'<li>{html.escape(value)}</li>' for value in report['limitations'])
    joint_table = ('<h2>Joint refinement</h2><p>The scalar scale record is the initializer seed and global background transform. '
                   'Joint refinement can subsequently deform individual poses and landmarks; its net metric geometry is measured in the trajectory plots. '
                   'These per-camera residuals compare the unchanged source map to the final joint proposal. Accepted proposals still require a completed commit record.</p>'
                   '<div class="scroll"><table><tr>'+''.join(f'<th>{key}</th>' for key in
                   ('Time s', 'Stage', 'Refined', 'Accepted', 'Points', 'Boundary KFs', 'cam0 obs', 'cam1 obs',
                    'cam0 before px', 'cam0 after px', 'cam1 before px', 'cam1 after px', 'cam0 inliers before',
                    'cam0 inliers after', 'cam1 inliers before', 'cam1 inliers after', 'Joint log-baseline σ', 'Reason'))+
                   '</tr>'+''.join(joint_rows)+'</table></div>') if joint_rows else ''
    return ('<!doctype html><html><head><meta charset="utf-8"><title>Metric initialization report</title>'
            '<style>body{font:15px system-ui;max-width:1500px;margin:24px auto;padding:0 18px;color:#202124}'
            'button{padding:10px;margin:4px}table{border-collapse:collapse;font-size:12px;max-width:100%}'
            'td,th{padding:7px;border:1px solid #ddd;text-align:right}td:last-child{text-align:left}'
            '.scroll{overflow:auto}svg{width:100%;height:auto}pre{white-space:pre-wrap}.warning{background:#fff4df;padding:14px}</style></head><body>'
            '<h1>Metric initialization: telemetry and saved geometry</h1>'
            f'<p><strong>Run:</strong> {html.escape(report["run"])}</p><p><strong>Baseline:</strong> {html.escape(report["baseline"])}</p>'
            f'<p><strong>Control: {html.escape(report["input_control"]["label"])}</strong> '
            f'{html.escape(report["input_control"]["interpretation"])}</p>'
            '<p>Ground truth is used only in this offline report. Top views use SE3 with scale fixed at 1. '
            'Separate diagnostic scale fits never alter trajectories or official scores. No independent maps are stitched.</p>'
            f'<p>Selected {report["selection"]["poses"]} exact common native timestamps, visual OK and not coasting in both runs. '
            'Their nanosecond timestamps and map/version identities are saved in selected_timestamps.csv.</p>'
            '<div class="warning"><strong>Interpretation limits</strong><ul>'+warnings+'</ul></div>'
            '<button onclick="show(\'trajectory\')">Trajectory and scale</button><button onclick="show(\'telemetry\')">Initializer gates</button>'
            '<button onclick="show(\'epochs\')">Exact epoch comparison</button>'
            '<section id="trajectory">'+figures['trajectory_comparison']+'</section>'
            '<section id="telemetry">'+figures['initializer_telemetry']+
            '<p>An accepted proposal is distinct from a completed commit. Missing or nonfinite telemetry is unavailable, never zero.</p>'
            '<p>In particular, a stereo-consistency rejection occurs before the uncertainty calculation. Its default infinite sigma means <strong>not computed</strong>, not measured unobservability.</p>'
            '<div class="scroll"><table><tr><th>Time s</th><th>Record</th><th>Map</th><th>Stage</th><th>Accepted</th><th>Scale</th>'
            '<th>Conditional σ</th><th>IMU σ</th><th>Completed stage</th><th>Committed scale</th><th>Reason</th></tr>'+
            ''.join(proposal_rows)+'</table></div>'+joint_table+'</section><section id="epochs"><p>Epoch fits split whenever either run changes map, initialization ID, or world version. '
            'Each final fit uses exactly the same timestamps as its online counterpart.</p><div class="scroll"><table><tr>'
            '<th>Group</th><th>Run gauge</th><th>Baseline gauge</th><th>Poses</th><th>Native times s</th><th>GT spread m</th>'
            '<th>Run online scale</th><th>Baseline online scale</th><th>Run final scale</th><th>Baseline final scale</th><th>Support</th></tr>'+
            ''.join(epoch_rows)+'</table></div></section><details><summary>Parser and provenance</summary><pre>'+
            html.escape(json.dumps({'run_parser': report['run_telemetry']['parse_report'],
                                  'baseline_parser': report['baseline_telemetry']['parse_report'],
                                  'input_control': report['input_control'],
                                  'selection': report['selection'], 'source_hashes': report['source_hashes']}, indent=2))+
            '</pre></details><script>function show(id){document.querySelectorAll("section").forEach(s=>s.hidden=s.id!==id)}show("trajectory")</script></body></html>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--assets', type=Path, help='Sequence directory containing aria_calib.json and gt_dense.txt; inferred from run manifest by default')
    parser.add_argument('--start', type=float, help='Optional native start time in seconds')
    parser.add_argument('--end', type=float, help='Optional native end time in seconds')
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError(f'Preserving existing report: choose a new --out: {args.out}')
    if args.start is not None and args.end is not None and args.end <= args.start:
        raise ValueError('--end must be after --start')
    assets = args.assets or infer_assets(args.run)
    run, baseline = load_saved_run(args.run, assets), load_saved_run(args.baseline, assets)
    input_control = compare_input_controls(run['metadata']['input_provenance'], baseline['metadata']['input_provenance'])
    gt_raw = np.loadtxt(assets/'gt_dense.txt')
    gt = {int(round(row[0])): row[1:4] for row in gt_raw}
    if len(gt) != len(gt_raw):
        raise ValueError('Duplicate GT timestamps')
    stamps, counts = select_common(run, baseline, gt, args.start, args.end)
    arrays, y, epochs, windows, final_summary = build_comparison(stamps, run, baseline, gt)
    args.out.mkdir(parents=True)
    with (args.out/'selected_timestamps.csv').open('w') as handle:
        writer = csv.writer(handle)
        writer.writerow(['timestamp_ns', 'native_time_s', 'run_map', 'run_init_kf', 'run_world_version',
                         'baseline_map', 'baseline_init_kf', 'baseline_world_version',
                         'run_online_x', 'run_online_y', 'run_online_z', 'baseline_online_x', 'baseline_online_y', 'baseline_online_z',
                         'run_final_x', 'run_final_y', 'run_final_z', 'baseline_final_x', 'baseline_final_y', 'baseline_final_z', 'gt_x', 'gt_y', 'gt_z'])
        for i, stamp in enumerate(stamps):
            writer.writerow([int(stamp), format(stamp/1e9, '.9f'), *run['online'][int(stamp)]['gauge'], *baseline['online'][int(stamp)]['gauge'],
                             *arrays['run_online'][i], *arrays['baseline_online'][i], *arrays['run_final'][i], *arrays['baseline_final'][i], *y[i]])
    report = {'run': str(args.run), 'baseline': str(args.baseline), 'assets': str(assets),
              'input_control': input_control,
              'evaluation_only': True, 'estimator_modified': False, 'official_score': False,
              'selection': {**counts, 'poses': len(stamps), 'first_timestamp_ns': int(stamps[0]), 'last_timestamp_ns': int(stamps[-1]),
                            'policy': 'Exact common native timestamp and GT intersection; visual OK/noncoasting in both selected largest final maps',
                            'gt_timestamp_tolerance_ns': 0, 'start_requested_s': args.start, 'end_requested_s': args.end},
              'run_metadata': run['metadata'], 'baseline_metadata': baseline['metadata'],
              'run_telemetry': run['telemetry'], 'baseline_telemetry': baseline['telemetry'],
              'epochs': epochs, 'windows': windows, 'final_common_fits': final_summary,
              'limitations': ['Conditioning estimates assume the captured visual geometry/calibration; they are not full joint-BA uncertainty.',
                             'Small-motion epochs (<10 GT samples or <0.5m RMS spread) are weak scale evidence.',
                             'world_version marks whole-world re-expression; ordinary BA can still move poses within one version.',
                             'Online snapshots are historical and are not retrospectively optimized; final poses include subsequent BA.',
                             input_control['interpretation'],
                             run['metadata']['input_provenance']['evidence_limit'],
                             'Each run selects its largest retained map without using GT. Omitted maps and excluded frames remain excluded, not stitched.',
                             'This is an offline diagnostic, not the official CP score or proof that initialization caused every later error.'],
              'source_hashes': {str(path): sha256(path) for path in (Path(__file__), Path(__file__).with_name('audit_lamaria_scale.py'), assets/'aria_calib.json', assets/'gt_dense.txt')}}
    figures = make_figures(stamps, arrays, y, epochs, windows, run['telemetry'], args.out)
    (args.out/'report.html').write_text(render_html(report, figures))
    (args.out/'metric_init_report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.out/'parse_report.json').write_text(json.dumps({'run': run['telemetry']['parse_report'], 'baseline': baseline['telemetry']['parse_report']}, indent=2)+'\n')
    checks = {'exact_timestamp_selection': True, 'camera_conversion_matches_config': True,
              'same_online_final_timestamp_scopes': True, 'topview_alignment_scale': 1.0,
              'online_groups_split_both_gauges': True, 'selected_timestamps_sha256': sha256(args.out/'selected_timestamps.csv'),
              'metric_log_malformed_records': len(run['telemetry']['parse_report']['malformed'])}
    (args.out/'validation.json').write_text(json.dumps(checks, indent=2)+'\n')
    print(json.dumps({'report': str(args.out/'report.html'), 'poses': len(stamps),
                      'run_telemetry': run['telemetry']['parse_report'], 'final_common_fits': final_summary}, indent=2))


if __name__ == '__main__':
    main()
