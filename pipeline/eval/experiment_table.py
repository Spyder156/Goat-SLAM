#!/usr/bin/env python3
"""One table across scored LaMAria runs: official scores plus ATE-style diagnostics.

Rows come from each run's saved lamaria_score/scores.json (official local
evaluation, unchanged). Diagnostics are computed read-only from the scored cam0
poses and the sequence's dense GT: SE3 (scale fixed at 1) RMSE and one dense-GT
Sim3 RMSE/scale, both labelled diagnostics, never scores. Unscorable runs (no
scores.json) are listed with their coverage from provenance/coverage.json.
Writes markdown, CSV and JSON into --out (refused if it exists).
"""
import argparse
import csv
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / 'viz'))
sys.path.insert(0, str(HERE.parents[1] / 'run'))
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from audit_lamaria_scale import fit, nearest  # noqa: E402
from score_lamaria import sha  # noqa: E402

ASSETS = Path('/media/raghav/HardDrive1/Mecka/lamaria')
REFERENCES = {'sequence_1_19': ('Short', 90.7), 'sequence_2_11': ('Medium', 78.5), 'sequence_3_17': ('Long', 70.9)}
COLUMNS = ['label', 'sequence', 'Score2D', 'CPRecall@1m_%', 'PoseRecall@5m_%', 'PoseRecall@1m_%', 'CP_within_1m', 'CP_total',
           'GT_assoc', 'GT_total', 'maps', 'coverage_%', 'horiz_median_m', 'horiz_rmse_m', 'horiz_p90_m', 'horiz_max_m',
           'CP_sim3_scale', 'ATE_SE3_rmse_m', 'ATE_Sim3_rmse_m', 'Sim3_scale_diag', 'runtime_s', 'run']


def gt_for(sequence):
    gt = np.loadtxt(ASSETS / sequence / 'gt_dense.txt', ndmin=2)
    return np.rint(gt[:, 0]).astype(np.int64), gt[:, 1:4]


def diagnostics(run, sequence):
    cam = np.loadtxt(run / 'lamaria_score/estimated_cam0_ns.txt', ndmin=2)
    stamps = cam[:, 0].astype(np.int64)
    gt_stamps, gt_p = gt_for(sequence)
    ei, delta = nearest(stamps, gt_stamps)
    keep = delta <= 1_000_000
    x, y = cam[ei[keep], 1:4], gt_p[keep]
    if len(x) < 3:
        return None, None, None
    se3, _ = fit(x, y, scale=False)
    sim3, _ = fit(x, y, scale=True)
    return se3['rmse_m'], sim3['rmse_m'], sim3['scale_estimate_to_gt']


def runtime_seconds(run):
    for candidate in (run.parent.parent / 'launch.json', run.parent.parent / 'status.json'):
        if candidate.exists():
            data = json.loads(candidate.read_text())
            if 'elapsed_seconds' in data:
                return data['elapsed_seconds']
            if isinstance(data.get('replay'), dict) and 'elapsed_seconds' in data['replay']:
                return data['replay']['elapsed_seconds']
    return None


def row_for(label, run):
    run = run.resolve()
    score_file = run / 'lamaria_score/scores.json'
    if not score_file.exists():
        prov = json.loads((run / 'lamaria_score/provenance.json').read_text()) if (run / 'lamaria_score/provenance.json').exists() else {}
        sel = prov.get('trajectory_selection', {})
        return {'label': label, 'sequence': prov.get('sequence'), 'Score2D': None, 'maps': sel.get('atlas_map_epochs'),
                'coverage_%': 100 * prov.get('input_coverage', {}).get('pose_coverage_fraction', float('nan')) if prov else None,
                'run': str(run), 'note': 'unscorable: official CP alignment failed'}
    s = json.loads(score_file.read_text())
    se3, sim3, scale = diagnostics(run, s['sequence'])
    h = s.get('associated_horizontal_error_m', {})
    # Older score files (2026-10-02 register) lack the selection/coverage fields; derive what is derivable.
    selection = s.get('trajectory_selection', {})
    maps = selection.get('atlas_map_epochs', s.get('map_epochs'))
    coverage = s.get('pose_coverage_fraction')
    if coverage is None and s.get('native_input_frames'):
        coverage = s['estimated_poses'] / s['native_input_frames']
    return {'label': label, 'sequence': s['sequence'], 'Score2D': s['Score2D'], 'CPRecall@1m_%': s.get('CPRecall@1m_percent'),
            'PoseRecall@5m_%': s.get('PoseRecall@5m_percent'), 'PoseRecall@1m_%': s.get('PoseRecall@1m_percent'),
            'CP_within_1m': s.get('CP_within_1m'), 'CP_total': s.get('CP_total'), 'GT_assoc': s.get('GT_associated'), 'GT_total': s.get('GT_total_denominator'),
            'maps': maps, 'coverage_%': None if coverage is None else 100 * coverage,
            'horiz_median_m': h.get('median'), 'horiz_rmse_m': h.get('rmse'), 'horiz_p90_m': h.get('p90'), 'horiz_max_m': h.get('maximum'),
            'CP_sim3_scale': s.get('CP_sim3', {}).get('scale'), 'ATE_SE3_rmse_m': se3, 'ATE_Sim3_rmse_m': sim3, 'Sim3_scale_diag': scale,
            'runtime_s': runtime_seconds(run), 'run': str(run)}


def fmt(value):
    if value is None:
        return '—'
    if isinstance(value, float):
        return f'{value:.3f}' if abs(value) < 1000 else f'{value:.0f}'
    return str(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', nargs=2, metavar=('LABEL', 'RUN_DIR'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--title', default='LaMAria experiment table')
    args = parser.parse_args()
    out = args.out.resolve()
    if out.exists():
        raise FileExistsError(f'Preserving previous table: {out}')
    rows = [row_for(label, Path(path)) for label, path in args.run]
    out.mkdir(parents=True)
    with (out / 'table.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS + ['note'])
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in COLUMNS + ['note']})
    (out / 'table.json').write_text(json.dumps({'title': args.title, 'rows': rows, 'meta_reference_Score2D': {k: v[1] for k, v in REFERENCES.items()},
                                                 'notes': ['Score2D, recalls, CP and GT counts are the official local evaluation (unchanged).',
                                                           'ATE_SE3 (scale 1) and ATE_Sim3/Sim3_scale_diag are dense-GT diagnostics, not scores.']}, indent=1) + '\n')
    short = ['label', 'sequence', 'Score2D', 'CPRecall@1m_%', 'PoseRecall@5m_%', 'PoseRecall@1m_%', 'CP_within_1m', 'maps', 'coverage_%',
             'horiz_median_m', 'horiz_rmse_m', 'ATE_SE3_rmse_m', 'ATE_Sim3_rmse_m', 'Sim3_scale_diag', 'runtime_s']
    lines = [f'# {args.title}', '', '| ' + ' | '.join(short) + ' |', '|' + '---|' * len(short)]
    for row in rows:
        lines.append('| ' + ' | '.join(fmt(row.get(k)) for k in short) + ' |')
    lines += ['', 'Meta/Aria category references: Short 90.7, Medium 78.5, Long 70.9 (not directly comparable to single training recordings).',
              'ATE columns are dense-GT diagnostics (SE3 scale 1; one Sim3), not scores. Unscorable rows show no Score2D.', '',
              'Runs:'] + [f'- {row["label"]}: {row["run"]}' for row in rows]
    (out / 'table.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
