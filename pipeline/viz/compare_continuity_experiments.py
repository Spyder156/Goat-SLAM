#!/usr/bin/env python3
"""Compare completed continuity trials using saved official scores only.

Incomplete runs are reported as incomplete, never assigned zero scores. The
selected map and evaluator's saved CP Sim3 are retained for every trajectory.
"""
import argparse
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = (ROOT / 'experiments').resolve()
BASELINES = {
    'medium': ARTIFACTS / 'lamaria_online_full_v2_medium_full_20261003/runs/lamaria_online_full_v2_medium_full_20261003_medium_full',
    'long': ARTIFACTS / 'lamaria_online_full_v2_long_native_full_20261003/runs/lamaria_online_full_v2_long_native_full_20261003_long_full',
}
VARIANTS = ('frozen_v2', 'recovery_direct', 'tracking_memory', 'periodic_vi')
LABELS = ('Frozen v2', 'Direct recovery', 'Landmark memory', 'Fixed-camera VI')


def read_score(run, sequence, variant):
    import numpy as np
    path = run / 'lamaria_score/scores.json'
    score = json.loads(path.read_text())
    poses = np.loadtxt(path.parent / 'estimated_cam0_ns.txt', ndmin=2)
    dt = np.diff(poses[:, 0]) / 1e9
    distance = np.linalg.norm(np.diff(poses[:, 1:4], axis=0), axis=1)
    adjacent = (dt >= .025) & (dt <= .1)
    speeds = distance[adjacent] / dt[adjacent]
    gaps = np.flatnonzero(dt > .5)
    selection = score.get('trajectory_selection')
    if selection is None:
        # Older single-map score schema: independently check coverage before
        # interpreting map_epochs, which is selected-map-only in newer files.
        coverage = json.loads((run / 'coverage.json').read_text())
        assert coverage['independent_segments_exported'] == score['map_epochs'] == 1
        selection = {'atlas_map_epochs': 1, 'selected_pose_count': score['estimated_poses'],
                     'atlas_pose_count': coverage['unique_input_frames_with_exported_pose']}
    return {
        'sequence': sequence, 'variant': variant, 'run': str(run),
        'score_path': str(path), 'Score2D': score['Score2D'],
        'CP_triangulated': score['CP_triangulated'], 'CP_total': score['CP_total'],
        'CP_within_1m': score['CP_within_1m'],
        'GT_associated': score['GT_associated'], 'GT_total': score['GT_total_denominator'],
        'native_coverage': score['pose_coverage_fraction'],
        'retained_maps': selection['atlas_map_epochs'],
        'selected_poses': selection['selected_pose_count'],
        'atlas_poses': selection['atlas_pose_count'],
        'CP_sim3_scale': score['CP_sim3']['scale'],
        'horizontal_errors_m': score['associated_horizontal_error_m'],
        'adjacent_pose_speed_m_s': {
            'note': 'Raw metric final-map exported poses, dt 25–100 ms. Spikes are diagnostics, not proof of a particular cause.',
            'median': float(np.median(speeds)) if len(speeds) else None,
            'p99': float(np.percentile(speeds, 99)) if len(speeds) else None,
            'maximum': float(speeds.max()) if len(speeds) else None,
            'above_10_m_s': int((speeds > 10).sum()),
        },
        'internal_pose_gaps_over_half_second': [
            {'start_native_s': float(poses[i, 0] / 1e9), 'duration_s': float(dt[i])}
            for i in gaps],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, default=ARTIFACTS / 'lamaria_continuity_batch_20261003')
    args = parser.parse_args()
    batch = args.batch.resolve()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.spatial.transform import Rotation
    rows, pending = [], []
    for sequence in ('medium', 'long'):
        rows.append(read_score(BASELINES[sequence], sequence, 'frozen_v2'))
        for variant in VARIANTS[1:]:
            case = batch / f'{variant}_{sequence}'
            state_path = case / 'status.json'
            state = json.loads(state_path.read_text()) if state_path.exists() else {'state': 'not started'}
            if state['state'] not in ('complete', 'complete_unscorable'):
                pending.append({'sequence': sequence, 'variant': variant, 'state': state['state']})
                continue
            if state['state'] == 'complete':
                row = read_score(Path(state['result']['run']), sequence, variant)
            else:
                diagnostic = state['result']
                row = {key: diagnostic[key] for key in ('run', 'Score2D', 'retained_maps', 'selected_poses', 'atlas_poses', 'evaluation_status')}
                row.update(sequence=sequence, variant=variant,
                           native_coverage=diagnostic['input_coverage']['pose_coverage_fraction'],
                           CP_triangulated=None, CP_total=None)
            row['online_diagnostics'] = state['result']
            row['runtime_seconds'] = state['replay']['elapsed_seconds']
            manifest = json.loads((Path(row['run']) / 'config/build_manifest.json').read_text())
            row['build_manifest'] = manifest
            rows.append(row)

    winners = {}
    for sequence in ('medium', 'long'):
        available = [row for row in rows if row['sequence'] == sequence and row['Score2D'] is not None]
        winners[sequence] = max(available, key=lambda row: row['Score2D'])['variant']
    historical_long = read_score(ARTIFACTS / 'lamaria_history_full_20261002', 'long', 'historical_continuity')
    historical_long['comparison_note'] = 'Different earlier initial calibration; not a matched ablation.'
    for row in rows + [historical_long]:
        counts = {'full_calibration_trials': 0, 'full_calibration_accepted': 0,
                  'fixed_vi_trials': 0, 'fixed_vi_accepted': 0}
        with (Path(row['run']) / 'run.log').open(errors='replace') as handle:
            for line in handle:
                family = ('full_calibration' if '[CALIBRATION-TRIAL]' in line else
                          'fixed_vi' if '[PERIODIC-VI-TRIAL]' in line else None)
                if family:
                    accepted = re.search(r'\baccepted=(\d+)', line)
                    if accepted:
                        counts[family + '_trials'] += 1
                        counts[family + '_accepted'] += int(accepted.group(1))
        row['periodic_refinement_counts'] = counts
    strongest = {}
    for sequence in ('medium', 'long'):
        available = [row for row in rows if row['sequence'] == sequence and row['Score2D'] is not None]
        if sequence == 'long':
            available.append(historical_long)
        strongest[sequence] = max(available, key=lambda row: row['Score2D'])['variant']
    result = {'results': rows, 'incomplete': pending, 'highest_observed_score_in_batch': winners,
              'historical_long_reference': historical_long, 'strongest_recorded_including_history': strongest,
              'caveats': ['One replay per candidate; asynchronous mapping/BA can affect results.',
                          'Frozen v2 references are retained earlier runs, not same-session repeats.',
                          'A and B also share the transactional relatch repair; they are not single-line ablations.',
                          'The earlier continuous Long scored 49.881534 with different initial calibration.',
                          'All scores use the same official local evaluator and largest-map selection.',
                          'No ground truth, CP timing, or evaluation alignment enters the estimator.']}
    (batch / 'comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    by_key = {(row['sequence'], row['variant']): row for row in rows}
    colors = ('0.55', '#4772c4', '#21875e', '#b56f1c')
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    for col, sequence in enumerate(('medium', 'long')):
        for i, variant in enumerate(VARIANTS):
            row = by_key.get((sequence, variant))
            if row is None:
                for ax in axes[:, col]:
                    ax.text(i, 5, 'Incomplete', ha='center', fontsize=9, color='0.4', rotation=90)
                continue
            if row['Score2D'] is not None:
                axes[0, col].bar(i, row['Score2D'], color=colors[i])
                axes[0, col].text(i, row['Score2D'] + 1, f'{row["Score2D"]:.2f}', ha='center')
            else:
                axes[0, col].text(i, 5, 'Unscorable\nCP alignment failed', ha='center', fontsize=9, rotation=90)
            axes[1, col].bar(i, 100 * row['native_coverage'], color=colors[i])
            cp_label = f'CP {row["CP_triangulated"]}/{row["CP_total"]}' if row['Score2D'] is not None else 'No CP alignment'
            axes[1, col].text(i, 100 * row['native_coverage'] + 1,
                             f'{row["retained_maps"]} maps\n{cp_label}',
                             ha='center', fontsize=9)
        axes[0, col].set(title=sequence.title(), ylabel='Official local Score2D', ylim=(0, 105))
        axes[1, col].set(ylabel='Native poses in scored map [%]', ylim=(0, 116))
        if sequence == 'long':
            axes[0, col].axhline(historical_long['Score2D'], color='#814998', ls='--', lw=1,
                                label='Earlier Long 49.88 (different calibration)')
            axes[0, col].legend(fontsize=8, loc='upper left')
        for ax in axes[:, col]:
            ax.set_xticks(range(len(VARIANTS)), LABELS, rotation=15)
            ax.grid(axis='y', alpha=.2)
    fig.suptitle('Continuity experiments — unchanged score rules; independent maps remain separate')
    fig.savefig(batch / 'comparison.png', dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 4, figsize=(18, 10), constrained_layout=True)
    for row_index, sequence in enumerate(('medium', 'long')):
        plan_paths = sorted(batch.glob(f'*_{sequence}/{sequence}_full/plan.json'))
        if not plan_paths:
            continue
        plan = json.loads(plan_paths[0].read_text())
        command = plan['render_command']
        gt = np.loadtxt(command[command.index('--gt') + 1], ndmin=2)[:, 1:4]
        origin = gt.mean(axis=0)
        gt -= origin
        for col, variant in enumerate(VARIANTS):
            ax = axes[row_index, col]
            ax.plot(gt[:, 0], gt[:, 1], color='0.7', lw=2, label='Full GT')
            row = by_key.get((sequence, variant))
            if row is not None and row['Score2D'] is not None:
                path = Path(row['score_path'])
                score = json.loads(path.read_text())
                sim = score['CP_sim3']
                poses = np.loadtxt(path.parent / 'estimated_cam0_ns.txt', ndmin=2)
                xyz = sim['scale'] * (poses[:, 1:4] @ Rotation.from_quat(sim['rotation_xyzw']).as_matrix().T)
                xyz += np.asarray(sim['translation']) - origin
                spans = np.split(np.arange(len(poses)), np.flatnonzero(np.diff(poses[:, 0]) > 5e8) + 1)
                for i, ids in enumerate(spans):
                    ax.plot(xyz[ids, 0], xyz[ids, 1], color=colors[col], lw=1,
                            label='Scored map' if i == 0 else None)
                ax.set_title(f'{sequence.title()} / {LABELS[col]}\n{row["Score2D"]:.2f}, {row["retained_maps"]} maps')
            elif row is not None:
                ax.set_title(f'{sequence.title()} / {LABELS[col]}\nNo CP alignment; {row["retained_maps"]} maps')
            else:
                ax.set_title(f'{sequence.title()} / {LABELS[col]}\nIncomplete')
            ax.set_aspect('equal', adjustable='datalim')
            ax.set(xlabel='East offset [m]', ylabel='North offset [m]')
            ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle('Saved evaluation alignment — gray route shows missing coverage; no new fit or map stitching')
    fig.savefig(batch / 'trajectories.png', dpi=160)
    plt.close(fig)
    print(json.dumps({'highest_observed_score_in_batch': winners, 'incomplete': pending}, indent=2))


if __name__ == '__main__':
    main()
