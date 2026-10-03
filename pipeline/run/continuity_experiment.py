#!/usr/bin/env python3
"""Run one recorded continuity variant through existing full-sequence tools.

Two filesystem slots bound concurrent replays. Every case gets a fresh suite,
logs, unchanged official scoring, and an evaluation-aligned diagnostic plot.
This launcher does not change estimator inputs, score rules, or thresholds.
"""
import argparse
import contextlib
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
MATRIX = ROOT / 'configs/orbslam3_lamaria/continuity_experiments_20261003/experiment.json'
DEFAULT_BATCH = ROOT / 'experiments/lamaria_continuity_batch_20261003'


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


@contextlib.contextmanager
def replay_slot(batch):
    folder = batch / 'slots'
    folder.mkdir(parents=True, exist_ok=True)
    handles = [open(folder / f'{i}.lock', 'a+') for i in range(2)]
    selected = None
    try:
        while selected is None:
            for i, handle in enumerate(handles):
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    selected = i
                    break
                except BlockingIOError:
                    continue
            if selected is None:
                time.sleep(5)
        yield selected
    finally:
        if selected is not None:
            fcntl.flock(handles[selected], fcntl.LOCK_UN)
        for handle in handles:
            handle.close()


def command(case, label, argv):
    start = time.monotonic()
    with (case / f'{label}.log').open('x') as log:
        result = subprocess.run(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    record = {'command': argv, 'returncode': result.returncode,
              'elapsed_seconds': time.monotonic() - start}
    save(case / f'{label}.json', record)
    if result.returncode:
        raise RuntimeError(f'{label} failed ({result.returncode}): {case / (label + ".log")}')
    return record


def diagnostics(case, plan, label):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.spatial.transform import Rotation

    run = Path(plan['run_directory'])
    score = json.loads((run / 'lamaria_score/scores.json').read_text())
    with next(run.glob('online_*.csv')).open() as handle:
        online = list(csv.DictReader(handle))
    ts = np.array([float(row['input_t_s']) for row in online])
    elapsed = ts - ts[0]
    inliers = np.array([int(row['inliers']) for row in online])
    states = np.array([int(row['state']) for row in online])
    coast = np.array([int(row['coasting']) for row in online])
    keys = [(row['map_id'], row['map_init_kf_id']) for row in online]
    unique = {key: i for i, key in enumerate(dict.fromkeys(keys))}
    epochs = np.array([unique[key] for key in keys])
    pose = np.loadtxt(run / 'lamaria_score/estimated_cam0_ns.txt', ndmin=2)
    args = plan['render_command']
    gt_path = Path(args[args.index('--gt') + 1])
    gt = np.loadtxt(gt_path, ndmin=2)
    sim = score['CP_sim3']
    xyz = sim['scale'] * (pose[:, 1:4] @ Rotation.from_quat(sim['rotation_xyzw']).as_matrix().T)
    xyz += np.asarray(sim['translation'])
    origin = gt[:, 1:4].mean(axis=0)
    xyz -= origin
    gt_xyz = gt[:, 1:4] - origin
    fig = plt.figure(figsize=(15, 10), constrained_layout=True)
    layout = fig.add_gridspec(3, 2, width_ratios=[1.2, 1])
    ax = fig.add_subplot(layout[:, 0])
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], color='0.65', lw=2, label='Full GT route')
    # Preserve genuine gaps; never draw a synthetic connection through missing poses.
    spans = np.split(np.arange(len(pose)), np.flatnonzero(np.diff(pose[:, 0]) > 5e8) + 1)
    for i, ids in enumerate(spans):
        ax.plot(xyz[ids, 0], xyz[ids, 1], color='darkorange', lw=1.3,
                label='Scored map, saved CP Sim3' if i == 0 else None)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set(xlabel='East offset [m]', ylabel='North offset [m]')
    ax.legend(fontsize=8)
    ax.grid(alpha=.2)
    a = fig.add_subplot(layout[0, 1])
    a.plot(elapsed, inliers, lw=.6)
    a.axhline(15, color='red', ls=':', label='15 inliers (context only)')
    a.set(ylabel='Online inliers', ylim=(0, None))
    a.legend(fontsize=8)
    a = fig.add_subplot(layout[1, 1])
    a.step(elapsed, epochs, where='post')
    a.set(ylabel='Online map epoch (incl. resets)')
    a = fig.add_subplot(layout[2, 1])
    a.plot(elapsed, states, lw=.7, label='Tracking state')
    a.fill_between(elapsed, 0, coast, alpha=.3, label='Coasting')
    a.set(xlabel='Elapsed input time [s]', ylabel='State / coast')
    a.legend(fontsize=8)
    selection = score['trajectory_selection']
    fig.suptitle(f'{label}: Score2D {score["Score2D"]:.2f}; '
                 f'{selection["atlas_map_epochs"]} retained maps; '
                 f'CP {score["CP_triangulated"]}/{score["CP_total"]}; '
                 f'scored native coverage {100 * score["pose_coverage_fraction"]:.1f}%')
    fig.savefig(case / 'diagnostics.png', dpi=160)
    plt.close(fig)
    counters = {'recovery_calls': 0, 'recovery_successes': 0,
                'recovery_hypotheses': 0, 'loop_detection_markers': 0,
                'merge_detection_markers': 0}
    for line in (run / 'run.log').open(errors='replace'):
        if '[RECOVERY_DIAG]' in line:
            counters['recovery_calls'] += 1
            values = dict(re.findall(r'(\w+)=(-?\d+)', line))
            counters['recovery_successes'] += int(values.get('success', 0))
            counters['recovery_hypotheses'] += int(values.get('hypotheses', 0))
        counters['loop_detection_markers'] += int('*Loop detected' in line)
        counters['merge_detection_markers'] += int('*Merge detected' in line)
    result = {'label': label, 'run': str(run), 'score_path': str(run / 'lamaria_score/scores.json'),
              'Score2D': score['Score2D'], 'CP_triangulated': score['CP_triangulated'],
              'CP_total': score['CP_total'], 'CP_within_1m': score['CP_within_1m'],
              'GT_associated': score['GT_associated'], 'GT_total': score['GT_total_denominator'],
              'retained_maps': selection['atlas_map_epochs'],
              'selected_poses': selection['selected_pose_count'],
              'atlas_poses': selection['atlas_pose_count'],
              'native_input_frames': score['native_input_frames'],
              'scored_pose_coverage': score['pose_coverage_fraction'],
              'CP_sim3_scale': sim['scale'], 'horizontal_errors_m': score['associated_horizontal_error_m'],
              'online_rows': len(online), 'online_map_epochs': len(unique),
              'coasted_frames': int(coast.sum()), 'tracking_ok_frames': int((states == 2).sum()),
              'log_counters': counters, 'diagnostic_plot': str(case / 'diagnostics.png')}
    save(case / 'summary.json', result)
    return result


def run_case(args, sequence):
    matrix = json.loads(MATRIX.read_text())
    variant = matrix['variants'][args.variant]
    config = ROOT / variant['configs'][sequence]['path']
    assert hashlib.sha256(config.read_bytes()).hexdigest() == variant['configs'][sequence]['sha256']
    manifest = json.loads((ROOT / 'configs/orbslam3_lamaria/suite_20261002.json').read_text())
    case = args.batch.resolve() / f'{args.variant}_{sequence}'
    if case.exists():
        raise FileExistsError(f'Case already exists; never reuse an experiment output: {case}')
    prepare = [manifest['python'], str(ROOT / 'pipeline/run/lamaria_suite.py'), 'prepare',
               '--sequence', sequence, '--profile', 'full', '--out', str(case),
               '--build', str(ROOT / 'build' / variant['build']), '--config', str(config),
               '--cpus', '4', '--match-diagnostics', '--debugger']
    subprocess.run(prepare, check=True, cwd=ROOT)
    plan = json.loads((case / f'{sequence}_full/plan.json').read_text())
    state = {'variant': args.variant, 'sequence': sequence, 'case': str(case), 'state': 'queued'}
    save(case / 'status.json', state)
    print(f'QUEUED {args.variant} {sequence}', flush=True)
    try:
        with replay_slot(args.batch.resolve()) as slot:
            state.update(state='running', slot=slot, started=time.time())
            save(case / 'status.json', state)
            print(f'START {args.variant} {sequence} slot={slot}', flush=True)
            state['replay'] = command(case, 'launch', plan['run_command'])
            state.update(state='scoring')
            save(case / 'status.json', state)
            state['scoring'] = command(case, 'score', plan['score_command'])
        state['result'] = diagnostics(case, plan, f'{args.variant} / {sequence}')
        state.update(state='complete', finished=time.time())
        save(case / 'status.json', state)
        print('COMPLETE ' + json.dumps(state['result']), flush=True)
    except Exception as error:
        state.update(state='failed', error=str(error), finished=time.time())
        save(case / 'status.json', state)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=('recovery_direct', 'tracking_memory', 'periodic_vi'), required=True)
    parser.add_argument('--sequence', choices=('medium', 'long', 'both'), default='both')
    parser.add_argument('--batch', type=Path, default=DEFAULT_BATCH)
    args = parser.parse_args()
    for sequence in (('medium', 'long') if args.sequence == 'both' else (args.sequence,)):
        run_case(args, sequence)


if __name__ == '__main__':
    main()
