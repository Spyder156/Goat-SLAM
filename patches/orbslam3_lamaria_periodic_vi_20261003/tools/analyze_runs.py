#!/usr/bin/env python3
"""Read saved C logs only; summarize fixed-VI proposals without rescoring/replay."""
from pathlib import Path
import argparse
import json
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = None

def values(line):
    out = {}
    for name, raw in re.findall(r'(\w+)=([-+0-9.e]+)(?:\s|$)', line):
        out[name] = float(raw) if '.' in raw or 'e' in raw.lower() else int(raw)
    return out

def audit(sequence):
    case = ROOT / ('periodic_vi_' + sequence)
    run = case / 'runs' / ('periodic_vi_' + sequence + '_' + sequence + '_full')
    text = (run / 'run.log').read_text(errors='replace')
    first_time = int((case / (sequence + '_full') / 'timestamps_ns.txt').read_text().splitlines()[0]) / 1e9
    trials = []
    pending = None
    for lineno, line in enumerate(text.splitlines(), 1):
        if '[PERIODIC-VI-SCHEDULE]' in line:
            val = values(line)
            pending = {'timestamp_s': val['t'], 'map_id': val['map'],
                       'init_kf': val['init_kf'], 'populated_maps': val['populated_maps'],
                       'mode': val['mode'], 'fixed_intrinsics': val['fixed_intrinsics'],
                       'schedule_log_line': lineno}
        elif '[PERIODIC-VI-HELDOUT]' in line and pending is not None:
            pending.setdefault('heldout', []).append(values(line))
        elif '[PERIODIC-VI-TRIAL]' in line and pending is not None:
            val = values(line)
            pending.update(accepted=bool(val['accepted']),
                           reason=line.split('reason=', 1)[-1].strip(),
                           result_log_line=lineno)
            trials.append(pending)
            pending = None
    by_map = {}
    for trial in trials:
        item = by_map.setdefault(f"map_{trial['map_id']}_init_{trial['init_kf']}",
                                 {'attempts': 0, 'accepted': 0, 'native_trial_timestamps_s': []})
        item['attempts'] += 1
        item['accepted'] += int(trial['accepted'])
        item['native_trial_timestamps_s'].append(trial['timestamp_s'])
    status = json.loads((case / 'status.json').read_text())
    report = {'sequence': sequence, 'run': str(run),
              'first_input_capture_time_s': first_time,
              'attempts': len(trials), 'accepted': sum(t['accepted'] for t in trials),
              'trials_with_heldout_validation': sum(bool(t.get('heldout')) for t in trials),
              'trials_rejected_before_heldout_validation': sum(not bool(t.get('heldout')) for t in trials),
              'fixed_vi_pre_solve_comparator_trials': sum(t['fixed_intrinsics'] == 1 for t in trials),
              'free_calibration_fixed_ba_comparator_trials': text.count('[CALIBRATION-TRIAL]'),
              'heldout_bins_evaluated': sum(len(t.get('heldout', [])) for t in trials),
              'heldout_bins_over_median_gate': sum(row['final_px'] > row['fixed_px'] + .03 for t in trials for row in t.get('heldout', [])),
              'heldout_bins_below_inlier_gate': sum(row['final_inliers'] < .95 * row['fixed_inliers'] for t in trials for row in t.get('heldout', [])),
              'intrinsic_commit_log_rows': text.count('[CALIBRATION-COMMIT]'),
              'attempts_with_multiple_populated_maps': sum(t['populated_maps'] > 1 for t in trials),
              'by_map': by_map, 'trials': trials,
              'replay_status': status['state'], 'result': status.get('result'),
              'acceptance_reference': 'Fixed VI compares candidate held-out residuals with pre-solve live geometry. In this log fixed_px denotes that live baseline, unlike v2 CALIBRATION-HELDOUT where it means separately optimized fixed-BA branch.',
              'interpretation': 'Rejected proposals never commit. A zero-accept run does not show that accepted fixed VI updates damaged the route. Held-out observations may previously have been used by local BA, so this is proposal validation against live geometry, not unseen-data cross-validation.'}
    (case / 'periodic_vi_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return report

def plot(reports):
    fig, axes = plt.subplots(2, len(reports), figsize=(15, 8), sharex='col', squeeze=False)
    colors = ['#386cb0', '#6baed6', '#bd3786', '#e995c8']
    for col, report in enumerate(reports):
        ax, lower = axes[:, col]
        t0 = report['first_input_capture_time_s']
        for i, (cam, outer) in enumerate([(0, 0), (0, 1), (1, 0), (1, 1)]):
            data = [(trial['timestamp_s'] - t0, row['final_px'] - row['fixed_px'])
                    for trial in report['trials'] for row in trial.get('heldout', [])
                    if row['cam'] == cam and row['outer'] == outer]
            if data:
                ax.plot(*zip(*data), '.-', color=colors[i], linewidth=1,
                        label=f"cam{cam} {'periphery' if outer else 'centre'}")
        ax.axhline(0, color='k', linewidth=.6)
        ax.axhline(.03, color='#cc1111', linestyle='--', linewidth=1, label='+0.03 px maximum increase')
        ax.set_ylabel('Candidate median minus live median (px)')
        ax.set_title(f"{report['sequence'].capitalize()}: {report['accepted']}/{report['attempts']} accepted")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
        for trial in report['trials']:
            lower.scatter(trial['timestamp_s'] - t0, trial['populated_maps'],
                          color='#228833' if trial['accepted'] else '#cc3311',
                          marker='o' if trial.get('heldout') else 'x', s=35)
        lower.set_ylabel('Populated maps at proposal')
        lower.set_xlabel('Seconds since first selected input frame')
        lower.set_ylim(bottom=0)
        lower.grid(alpha=.2)
    fig.suptitle('Fixed-intrinsic periodic VI: proposals versus the live map\nPositive residual differences are worse. Red markers = rejected; x = rejected before held-out solve.', fontsize=12)
    fig.tight_layout(rect=(0, .06, 1, .92))
    fig.text(.01, .015, 'Proposal-only diagnostics. No GT enters optimization. These held-out observations may have previously been used by local BA.\nNo rejected proposal changes live poses, points, intrinsics or IMU state; this plot does not isolate all causes of trajectory fragmentation.', fontsize=9)
    path = ROOT / 'periodic_vi_proposals.png'
    fig.savefig(path, dpi=160)
    print(path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True, help='Root containing completed periodic_vi_medium and periodic_vi_long case folders.')
    args = parser.parse_args()
    ROOT = args.suite.resolve(strict=True)
    reports = [audit(sequence) for sequence in ['medium', 'long']]
    plot(reports)
    print(json.dumps([{k: report[k] for k in ['sequence', 'attempts', 'accepted', 'attempts_with_multiple_populated_maps', 'replay_status']} for report in reports]))
