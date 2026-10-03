#!/usr/bin/env python3
"""Replay one fresh full Long BabyFeatures experiment, then score and diagnose.

Inputs, native camera calibration, calibrated IMU, temporal associations, four
CPU allocation and official largest-map scoring match the frozen v2 transfer.
BabyFeatures are explicitly enabled by the launcher; no GT reaches SLAM.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from continuity_experiment import command, diagnostics, save, unscorable_diagnostics

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = ROOT / 'experiments/lamaria_babyfeats_long_20261003'
DEFAULT_BUILD = ROOT / 'build/orbslam3_lamaria_babyfeats_20261003'
CONFIG = ROOT / 'configs/orbslam3_lamaria/babyfeats_20261003/long_native.yaml'
BASE_CONFIG = ROOT / 'configs/orbslam3_lamaria/online_full_v2_transfer_20261003/long_native.yaml'
SUITE = ROOT / 'configs/orbslam3_lamaria/suite_20261002.json'


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(build):
    manifest = json.loads((build / 'build_manifest.json').read_text())
    paths = {'config': CONFIG, 'base_config': BASE_CONFIG,
             'build_manifest': build / 'build_manifest.json',
             'library': build / 'lib/libORB_SLAM3.so',
             'runner': build / 'bin/stereo_lamaria_euroc', 'suite_manifest': SUITE}
    hashes = {key: {'path': str(path.resolve()), 'sha256': sha256(path)}
              for key, path in paths.items()}
    if hashes['config']['sha256'] != hashes['base_config']['sha256']:
        raise ValueError('BabyFeatures Long config must exactly preserve frozen v2 native settings')
    for key, manifest_key in [('library', 'library_sha256'), ('runner', 'runner_sha256')]:
        if hashes[key]['sha256'] != manifest[manifest_key]:
            raise ValueError(f'{key} differs from recorded build manifest')
    return hashes


def git_value(*argv):
    return subprocess.check_output(['git', *argv], cwd=ROOT, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT,
                        help='Fresh suite directory; existing experiments are never overwritten')
    parser.add_argument('--build', type=Path, default=DEFAULT_BUILD)
    parser.add_argument('--render', action='store_true',
                        help='After scoring, generate complete evaluation-aligned Rerun (adds runtime/storage)')
    args = parser.parse_args()
    out, build = args.out.resolve(), args.build.resolve(strict=True)
    if out.exists():
        raise FileExistsError(f'Experiment already exists; choose a fresh --out: {out}')
    before = fingerprint(build)
    suite = json.loads(SUITE.read_text())
    python = suite['python']
    prepare = [python, str(ROOT / 'pipeline/run/lamaria_suite.py'), 'prepare',
               '--sequence', 'long', '--profile', 'full', '--out', str(out),
               '--build', str(build), '--config', str(CONFIG), '--cpus', '4',
               '--match-diagnostics', '--debugger', '--baby-features']
    subprocess.run(prepare, cwd=ROOT, check=True)
    plan = json.loads((out / 'long_full/plan.json').read_text())
    run = Path(plan['run_directory'])
    docker = plan['runner_dry_run']['command']
    if 'LAMARIA_BABY_FEATURES=1' not in docker or '--baby-features' not in plan['run_command']:
        raise ValueError('Prepared run did not explicitly enable BabyFeatures')
    if plan['runner_dry_run']['ground_truth_used_by_estimator'] is not False:
        raise ValueError('Ground truth must remain evaluation-only')
    provenance = {'experiment': 'BabyFeatures SOS / full Long',
                  'git_commit': git_value('rev-parse', 'HEAD'),
                  'git_branch': git_value('branch', '--show-current'),
                  'working_tree_diff_sha256': hashlib.sha256(subprocess.check_output(
                      ['git', 'diff', 'HEAD'], cwd=ROOT)).hexdigest(),
                  'estimator_files_before': before,
                  'asset_validation': plan['asset_validation'],
                  'prepare_command': prepare,
                  'estimator_environment': {'LAMARIA_BABY_FEATURES': '1'},
                  'settings_equal_frozen_v2': True,
                  'uses_ground_truth_for_estimation': False,
                  'scoring': 'Existing official functions, largest map without GT selection, full denominators',
                  'reference': {'native_v2_score': 34.458900755473394, 'native_v2_maps': 5,
                                'historical_long_score': 49.881533645273585,
                                'historical_caveat': 'Different earlier calibration; not a matched ablation'}}
    save(out / 'provenance.json', provenance)
    status = {'state': 'running', 'experiment': 'BabyFeatures SOS', 'sequence': 'sequence_3_17',
              'started': time.time(), 'run': str(run), 'out': str(out)}
    save(out / 'status.json', status)
    print(f'START BabyFeatures full Long: {out}', flush=True)
    try:
        status['replay'] = command(out, 'launch', plan['run_command'])
        after = fingerprint(build)
        provenance['estimator_files_after'] = after
        provenance['estimator_files_unchanged_during_replay'] = before == after
        save(out / 'provenance.json', provenance)
        if before != after:
            raise ValueError('Estimator artifacts or inputs changed during replay; inspect provenance')
        status['state'] = 'scoring'
        save(out / 'status.json', status)
        scorable = True
        try:
            status['scoring'] = command(out, 'score', plan['score_command'])
        except RuntimeError:
            if 'Official control-point alignment failed; no score fabricated' not in (out / 'score.log').read_text():
                raise
            scorable = False
            status['scoring'] = json.loads((out / 'score.json').read_text())
        status['result'] = (diagnostics if scorable else unscorable_diagnostics)(
            out, plan, 'BabyFeatures SOS / Long')
        status['baby_diagnostics'] = command(out, 'baby_diagnostics', [python,
            str(ROOT / 'pipeline/viz/babyfeats_diagnostics.py'), '--case', str(out)])
        baby = json.loads((out / 'babyfeats_diagnostics/summary.json').read_text())
        status['result'].update(baby_event_counts=baby['baby_event_counts'],
                                sos_episodes=baby['sos_episodes'],
                                continuity_plot=str(out / 'babyfeats_diagnostics/continuity.png'))
        save(out / 'summary.json', status['result'])
        if args.render and scorable:
            status['state'] = 'rendering'
            save(out / 'status.json', status)
            render = plan['render_command'] + ['--output-dir', str(out / 'evaluation'),
                                              '--evaluation-score', str(run / 'lamaria_score/scores.json')]
            status['render'] = command(out, 'render', render)
            status['evaluation_rrd'] = str(out / 'evaluation/run.rrd')
        status.update(state='complete' if scorable else 'complete_unscorable', finished=time.time())
        save(out / 'status.json', status)
        print('COMPLETE ' + json.dumps(status['result']), flush=True)
        print('BabyFeatures diagnostics: ' + str(out / 'babyfeats_diagnostics'), flush=True)
    except Exception as error:
        status.update(state='failed', error=str(error), finished=time.time())
        save(out / 'status.json', status)
        raise


if __name__ == '__main__':
    main()
