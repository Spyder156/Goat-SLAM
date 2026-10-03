#!/usr/bin/env python3
"""Compare two already-scored LaMAria runs with unchanged timestamp coverage.

Uses each run's saved official control-point Sim3 for display. Performs no fit,
scoring, interpolation, optimization, trajectory repair, or estimator rerun.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_run(path):
    path = path.resolve()
    files = {name: path / 'lamaria_score' / name for name in (
        'scores.json', 'control_point_errors.json', 'estimated_cam0_ns.txt', 'provenance.json')}
    score = json.loads(files['scores.json'].read_text())
    provenance = json.loads(files['provenance.json'].read_text())
    cp = json.loads(files['control_point_errors.json'].read_text())
    poses = np.loadtxt(files['estimated_cam0_ns.txt'], ndmin=2)
    if poses.shape[1] != 8 or not np.isfinite(poses).all():
        raise ValueError('Expected finite native camera poses with eight fields')
    stamps = poses[:, 0].astype(np.int64)
    if not np.array_equal(poses[:, 0], stamps) or np.any(np.diff(stamps) <= 0):
        raise ValueError('Scored camera timestamps must be unique increasing integer nanoseconds')
    if len(stamps) != score['estimated_poses'] or len(cp) != score['CP_total']:
        raise ValueError('Saved score counts disagree with scored artifacts')
    if score['trajectory_selection']['atlas_map_epochs'] != 1:
        raise ValueError('This native refinement comparison requires one existing map')
    sim3 = score['CP_sim3']
    xyz = sim3['scale'] * (poses[:, 1:4] @ Rotation.from_quat(sim3['rotation_xyzw']).as_matrix().T)
    xyz += np.array(sim3['translation'])
    return dict(path=path, score=score, provenance=provenance, cp=cp, stamps=stamps,
                xyz=xyz, files=files)


def metrics(run):
    score = run['score']
    return {
        'Score2D': score['Score2D'],
        'estimated_poses': score['estimated_poses'],
        'native_input_frames': score['native_input_frames'],
        'pose_coverage_percent': 100 * score['pose_coverage_fraction'],
        'CP_triangulated': score['CP_triangulated'], 'CP_total': score['CP_total'],
        'CP_within_1m': score['CP_within_1m'],
        'GT_associated': score['GT_associated'], 'GT_total_denominator': score['GT_total_denominator'],
        'PoseRecall@1m_percent': score['PoseRecall@1m_percent'],
        'PoseRecall@5m_percent': score['PoseRecall@5m_percent'],
        'horizontal_median_m': score['associated_horizontal_error_m']['median'],
        'horizontal_RMSE_m': score['associated_horizontal_error_m']['rmse'],
        'horizontal_p90_m': score['associated_horizontal_error_m']['p90'],
        'horizontal_maximum_m': score['associated_horizontal_error_m']['maximum'],
        'CP_Sim3_scale': score['CP_sim3']['scale'],
        'retained_maps': score['trajectory_selection']['atlas_map_epochs'],
    }


def compare(args):
    output = args.out.resolve()
    if output.exists():
        raise FileExistsError(f'Preserving previous diagnostics: {output}')
    baseline, candidate = read_run(args.baseline), read_run(args.candidate)
    if baseline['path'] == candidate['path']:
        raise ValueError('Baseline and candidate must be different runs')
    if not np.array_equal(baseline['stamps'], candidate['stamps']):
        raise ValueError('Candidate changed exported timestamp coverage; refusing a matched refinement comparison')
    for key in ('sequence', 'native_input_frames', 'processed_input_frames', 'CP_total',
                'GT_total_denominator', 'GT_associated', 'missing_native_input_poses'):
        if baseline['score'][key] != candidate['score'][key]:
            raise ValueError(f'Comparison denominator/scope changed: {key}')
    gt_hash = sha(args.gt)
    for run in (baseline, candidate):
        matching = [value for name, value in run['provenance']['files_sha256'].items()
                    if Path(name).name == 'gt_dense.txt']
        if matching != [gt_hash]:
            raise ValueError('Plot GT differs from the GT artifact actually scored')
        if run['provenance'].get('GT_used_in_estimator') is not False:
            raise ValueError('Missing declaration of evaluation-only ground truth')
    if baseline['provenance']['official_python_sha256'] != candidate['provenance']['official_python_sha256']:
        raise ValueError('Official scoring implementation changed between runs')
    source_names = {int(row['tag_id']): row['name'] for row in baseline['cp']}
    candidate_by_id = {int(row['tag_id']): row for row in candidate['cp']}
    if source_names != {key: row['name'] for key, row in candidate_by_id.items()}:
        raise ValueError('Control point identities changed')
    cp_rows = []
    for before in baseline['cp']:
        after = candidate_by_id[int(before['tag_id'])]
        a, b = before['horizontal_error_m'], after['horizontal_error_m']
        cp_rows.append({
            'tag_id': before['tag_id'], 'name': before['name'],
            'baseline_triangulated': before['triangulated'], 'candidate_triangulated': after['triangulated'],
            'baseline_error_m': a, 'candidate_error_m': b,
            'candidate_minus_baseline_m': None if a is None or b is None else b - a,
            'baseline_inlier_observations': before['inlier_observations'],
            'candidate_inlier_observations': after['inlier_observations'],
        })
    before_metrics, after_metrics = metrics(baseline), metrics(candidate)
    gt = np.loadtxt(args.gt, ndmin=2)
    if gt.shape[1] != 8 or not np.isfinite(gt).all():
        raise ValueError('Expected finite native dense GT')
    origin = gt[:, 1:4].mean(axis=0)
    gt_xyz = gt[:, 1:4] - origin
    output.mkdir(parents=True)
    with (output / 'metrics.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['metric', 'baseline', 'candidate', 'candidate_minus_baseline'])
        for name, value in before_metrics.items():
            writer.writerow([name, value, after_metrics[name], after_metrics[name] - value])
    with (output / 'control_points.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(cp_rows[0]))
        writer.writeheader(); writer.writerows(cp_rows)
    fig = plt.figure(figsize=(16, 12), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, height_ratios=[1.4, 1])
    axes = [fig.add_subplot(grid[0, i]) for i in (0, 1)]
    all_xy = [gt_xyz[:, :2], baseline['xyz'][:, :2] - origin[:2], candidate['xyz'][:, :2] - origin[:2]]
    combined = np.concatenate(all_xy)
    low, high = combined.min(axis=0), combined.max(axis=0)
    margin = .04 * np.maximum(high - low, 1)
    for axis, run, label, color in zip(axes, (baseline, candidate),
                                      ('Saved BabyFeatures Long', 'Fixed-calibration global VI refinement'),
                                      ('darkorange', 'royalblue')):
        axis.plot(gt_xyz[:, 0], gt_xyz[:, 1], c='0.55', lw=2, label='Full GT route')
        xyz = run['xyz'] - origin
        spans = np.split(np.arange(len(xyz)), np.flatnonzero(np.diff(run['stamps']) > 500000000) + 1)
        for index, ids in enumerate(spans):
            axis.plot(xyz[ids, 0], xyz[ids, 1], c=color, lw=1.1,
                      label='Saved official CP alignment' if index == 0 else None)
        score = run['score']
        axis.set_title(f'{label}\nScore2D {score["Score2D"]:.4f}; CP <1m {score["CP_within_1m"]}/{score["CP_total"]}')
        axis.set(xlabel='East offset [m]', ylabel='North offset [m]',
                 xlim=(low[0]-margin[0], high[0]+margin[0]), ylim=(low[1]-margin[1], high[1]+margin[1]))
        axis.set_aspect('equal', adjustable='box'); axis.grid(alpha=.2); axis.legend(fontsize=8)
    axis = fig.add_subplot(grid[1, :])
    ids = np.arange(len(cp_rows))
    for offset, name, color, label in ((-.2, 'baseline_error_m', 'darkorange', 'Baseline'),
                                      (.2, 'candidate_error_m', 'royalblue', 'Refined')):
        values = [np.nan if row[name] is None else row[name] for row in cp_rows]
        axis.bar(ids + offset, values, width=.38, color=color, label=label)
        for index, value in enumerate(values):
            if not np.isfinite(value):
                axis.annotate('missing', (index+offset, 0), xytext=(0, 8), textcoords='offset points',
                              rotation=90, fontsize=7, color=color)
    axis.axhline(1, color='black', ls=':', lw=1, label='1m')
    axis.set_xticks(ids, [row['name'] for row in cp_rows], rotation=65, ha='right')
    axis.set(ylabel='Official horizontal control-point error [m]', ylim=(0, None),
             title='Every original control point; lower bars are better')
    axis.grid(axis='y', alpha=.2); axis.legend(fontsize=9)
    fig.suptitle(f'{baseline["score"]["sequence"]}: identical {len(baseline["stamps"]):,} exported pose timestamps; '
                 f'{before_metrics["native_input_frames"]:,} full native inputs\n'
                 'Each view uses its saved official evaluation Sim3. No new fit, interpolation, or GT optimization.')
    fig.savefig(output / 'comparison.png', dpi=170); plt.close(fig)
    delta = after_metrics['Score2D'] - before_metrics['Score2D']
    summary = {
        'baseline': {'run': str(baseline['path']), 'metrics': before_metrics},
        'candidate': {'run': str(candidate['path']), 'metrics': after_metrics},
        'Score2D_delta': delta, 'timestamp_coverage_identical': True,
        'new_alignment_fitted': False, 'ground_truth_used_for_optimization': False,
        'control_points': cp_rows,
        'source_hashes': {f'{label}/{name}': sha(path) for label, run in (
            ('baseline', baseline), ('candidate', candidate)) for name, path in run['files'].items()},
        'gt_dense_sha256': gt_hash, 'comparison_script_sha256': sha(Path(__file__)),
        'visualization': str(output / 'comparison.png'),
        'interpretation': 'A positive score delta is a result on this frozen Long run; it is not evidence of generalization or a guarantee of improved online tracking.',
    }
    (output / 'comparison.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    rows = ['# Fixed native-graph VI comparison', '',
            f'Score2D: {before_metrics["Score2D"]:.6f} → {after_metrics["Score2D"]:.6f} ({delta:+.6f}).', '',
            '| Metric | Baseline | Refined |', '|---|---:|---:|']
    for name, value in before_metrics.items():
        rows.append(f'| {name} | {value:.6g} | {after_metrics[name]:.6g} |')
    rows += ['', 'Inspect comparison.png: a better candidate follows the GT route more closely and lowers the control-point bars.',
             'The full pose timestamp set is unchanged. Missing startup frames remain missing. GT is evaluation-only.',
             '', 'Full precision: comparison.json; per-point evidence: control_points.csv.']
    (output / 'comparison.md').write_text('\n'.join(rows) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--gt', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    result = compare(parser.parse_args())
    print(json.dumps({'Score2D_delta': result['Score2D_delta'], 'visualization': result['visualization']}, indent=2))


if __name__ == '__main__':
    main()
