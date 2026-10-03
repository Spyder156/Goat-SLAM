#!/usr/bin/env python3
"""Summarize completed full solves without treating usable as converged."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import numpy as np


def summarize(suite, mode):
    arm = suite / 'offline' / mode
    model = arm / 'model'
    command = json.loads((arm / 'solve.command_result.json').read_text())
    result = json.loads((model / 'result.json').read_text())
    if command['returncode']:
        raise ValueError('Cannot summarize a failed solve as an output')
    report = dict(mode=mode, usable=result['usable'], converged=result['converged'],
                  accepted_iterations=result['accepted_iterations'],
                  linear_capped_steps=result['linear_capped_steps'],
                  linear_solver=result['linear_solver'],
                  explicit_schur=result.get('explicit_schur', False),
                  runtime_seconds=command['elapsed_s'], frames=result['frames'],
                  points=result['points'], observations=result['observations'],
                  dropped_observations=result['dropped_observations'],
                  unprojectable_final_observations=result['unprojectable_final_observations'],
                  reprojection_rms_pixels=result['final_projectable_pixel_rms'])
    report['costs'] = dict(initial_global_after_state_warmup=result['initial_cost'],
                           final_global_linearized=result['final_cost'],
                           initial_visual=result['initial_visual_cost'],
                           final_visual=result['final_visual_cost'])
    report['gravity'] = None
    report['biases'] = None
    report['final_imu_audit'] = None
    metadata = model / 'observation_metadata.json'
    report['observation_metadata_audit'] = json.loads(metadata.read_text()) if metadata.exists() else None
    if mode != 'visual_rig':
        report['costs'].update(initial_imu_before_state_warmup=result['initial_imu_cost'],
                               final_imu_linearized=result['final_imu_cost'],
                               final_imu_exact=result.get('final_imu_exact_cost'),
                               final_global_exact=result.get('final_exact_cost'))
        report['costs']['final_priors_and_bias_random_walk'] = (
            result['final_cost'] - result['final_visual_cost'] - result['final_imu_cost'])
        report['costs']['other_constraint_cost_description'] = (
            'Global minus visual minus 9D IMU residual cost: initial bias prior, '
            'bias random walks, and camera priors for vi_calib; subtraction may include roundoff.')
        gravity = np.asarray(result['gravity'], dtype=float)
        report['gravity'] = dict(vector_m_s2=gravity.tolist(), norm_m_s2=float(np.linalg.norm(gravity)),
                                 unit_direction=(gravity / np.linalg.norm(gravity)).tolist())
        states = np.genfromtxt(model / 'imu_states.csv', delimiter=',', names=True)
        report['biases'] = {}
        for name, columns, units in [('gyro', ('bgx', 'bgy', 'bgz'), 'rad/s'),
                                     ('accelerometer', ('bax', 'bay', 'baz'), 'm/s^2')]:
            values = np.column_stack([states[c] for c in columns])
            norms = np.linalg.norm(values, axis=1)
            report['biases'][name] = dict(units=units, axis_min=values.min(axis=0).tolist(),
                                         axis_max=values.max(axis=0).tolist(),
                                         norm_min=float(norms.min()), norm_median=float(np.median(norms)),
                                         norm_p95=float(np.percentile(norms, 95)), norm_max=float(norms.max()))
        report['final_imu_audit'] = {k: result.get(k) for k in (
            'final_imu_linearized_cost', 'final_imu_exact_cost', 'final_exact_cost',
            'final_imu_cost_relative_shift', 'final_max_gyro_bias_linearization_displacement',
            'final_max_accel_bias_linearization_displacement', 'final_exact_reintegrations')}
    with (model / 'frame_support.csv').open() as stream:
        support = list(csv.DictReader(stream))
    report['pose_support_categories'] = dict(Counter(row['pose_support'] for row in support))
    report['visual_associations_per_frame'] = {}
    for camera in ('cam0', 'cam1'):
        counts = np.array([int(row[camera + '_observations']) for row in support])
        report['visual_associations_per_frame'][camera] = dict(min=int(counts.min()),
            median=float(np.median(counts)), max=int(counts.max()), zero_frames=int((counts == 0).sum()))
    rss_path = suite / 'process_rss_samples.jsonl'
    samples = [json.loads(line) for line in rss_path.read_text().splitlines()] if rss_path.exists() else []
    finished_at = (arm / 'solve.command_result.json').stat().st_mtime
    started_at = finished_at - command['elapsed_s']
    selected = [sample for sample in samples if ('--mode ' + mode + ' ') in sample['cmdline']
                and ('--output_path ' + str(model) + ' ') in sample['cmdline']
                and started_at - 1 <= sample['time'] <= finished_at + 1]
    report['resource_sample_time_window'] = dict(start=started_at, end=finished_at,
                                                tolerance_seconds=1)
    report['peak_process_rss_gib'] = max((s['peak_rss_kib'] / 1024**2 for s in selected), default=None)
    containers = {s['container'] for s in selected}
    resource_path = suite / 'canonical_resource_samples.jsonl'
    resources = [json.loads(line) for line in resource_path.read_text().splitlines()] if resource_path.exists() else []
    report['peak_cgroup_memory_gib'] = max((s['memory_peak'] / 1024**3 for s in resources
                                           if s['container'] in containers), default=None)
    execution = model / 'execution_manifest.json'
    report['actual_execution_manifest'] = str(execution) if execution.exists() else None
    if execution.exists():
        report['effective_execution'] = json.loads(execution.read_text())
    else:
        report['execution_provenance'] = 'Original frozen offline_experiment.json and solve.command_result.json'
    destination = arm / 'scientific_summary.json'
    destination.write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True)
    parser.add_argument('--mode', choices=('visual_rig', 'vi_fixed', 'vi_calib'), required=True)
    args = parser.parse_args()
    print(json.dumps(summarize(args.suite.resolve(), args.mode), indent=2))


if __name__ == '__main__':
    main()
