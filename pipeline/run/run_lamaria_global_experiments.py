#!/usr/bin/env python3
"""Run the full three-arm native-camera batch experiment, score, and visualize.

All arms start from the same immutable SIFT reconstruction. No evaluation data
are supplied to the optimizer. Completed stages are retained with commands and
hashes, and failed stages stop rather than silently publishing a partial run.
"""
import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path, relocated_record
import subprocess
import time

from lamaria_colmap_refinement import DEFAULT_SOURCE, DEFAULT_WORK, ROOT, save, sha

MODES = ('visual_rig', 'vi_fixed', 'vi_calib')
DATA = Path('/media/raghav/HardDrive1/Mecka/lamaria/bench_basalt/orbslam/current_pipeline_20261002/sequence_1_19')
ASSETS = Path('/media/raghav/HardDrive1/Mecka/lamaria/sequence_1_19')


def run_stage(folder, label, argv):
    folder.mkdir(parents=True, exist_ok=True)
    argv = list(map(str, argv))
    marker = folder / (label + '.command_result.json')
    if marker.exists():
        previous = json.loads(marker.read_text())
        if relocated_record(previous['command']) == relocated_record(argv) and previous['returncode'] == 0:
            print(label + ': retained successful output', flush=True)
            return
    started = time.time()
    print(label + ': started', flush=True)
    with (folder / (label + '.log')).open('a') as log:
        log.write(json.dumps(argv) + '\n'); log.flush()
        rc = subprocess.call(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                             env={**os.environ, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '4'})
    save(marker, dict(command=argv, returncode=rc, elapsed_s=time.time()-started))
    if rc:
        raise RuntimeError(f'{label} failed ({rc}): {folder / (label + ".log")}')
    print(f'{label}: completed in {time.time()-started:.1f}s', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('solve', 'export', 'score', 'viz', 'all'))
    parser.add_argument('--source-run', type=relocated_path, default=DEFAULT_SOURCE)
    parser.add_argument('--sift-work', type=relocated_path, default=DEFAULT_WORK)
    parser.add_argument('--suite', type=relocated_path, default=DEFAULT_WORK.parent)
    parser.add_argument('--mode', choices=MODES, action='append')
    parser.add_argument('--iterations', type=int, default=30)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--linear-solver', choices=('iterative_schur', 'sparse_schur'), default='iterative_schur')
    parser.add_argument('--linear-eta', type=float, default=.001)
    parser.add_argument('--max-linear-iterations', type=int, default=1000)
    args = parser.parse_args()
    args.source_run=args.source_run.resolve(strict=True)
    work=args.sift_work.resolve(strict=True); suite=args.suite.resolve()
    model=work/'triangulated'; binary=ROOT/'build/colmap_lamaria_ba/colmap_lamaria_ba'
    graph=json.loads((work/'graph_summary.json').read_text())
    files=[model/n for n in ('cameras.txt','rigs.txt','frames.txt','images.txt','points3D.txt')]
    # COLMAP prefers binary when both serializations exist. Hash every input
    # representation, not just the text used by the Python output adapter.
    files += [model/n for n in ('cameras.bin','rigs.bin','frames.bin','images.bin','points3D.bin') if (model/n).exists()]
    provenance={str(p):sha(p) for p in files}
    provenance[str(binary)]=sha(binary)
    manifest=dict(source_run=str(args.source_run),sift_work=str(work),graph=graph,
                  files_sha256=provenance, modes=list(MODES),
                  metric_rig_fixed=True, ground_truth_used_by_optimization=False,
                  camera_model='native RAD_TAN_THIN_PRISM_FISHEYE / Aria Fisheye624',
                  calibration_free_parameters_per_camera=15,
                  solver=dict(linear_solver=args.linear_solver, linear_eta=args.linear_eta,
                              max_linear_iterations=args.max_linear_iterations,
                              max_iterations=args.iterations, threads=args.threads,
                              memory_limit=os.environ.get('LAMARIA_BA_MEMORY', '26g'),
                              cpu_limit=os.environ.get('LAMARIA_BA_CPUS', '4')),
                  alignment='Raw estimator output; CP Sim3 only during evaluation and evaluation Rerun')
    manifest_path=suite/'offline_experiment.json'
    if manifest_path.exists():
        if relocated_record(json.loads(manifest_path.read_text())) != relocated_record(manifest):
            raise ValueError('Input graph or executable changed; choose a new experiment suite')
        # Preserve the exact original execution record, including its old paths.
    else:
        save(manifest_path,manifest)
    for mode in args.mode or MODES:
        arm=suite/'offline'/mode; output=arm/'model'; run=suite/'runs'/mode
        if args.stage in ('solve','all'):
            run_stage(arm,'solve',['bash',ROOT/'pipeline/vi_ba_lamaria/run.sh',
                '--input_path',model,'--output_path',output,
                '--imu_csv',DATA/'calibrated_imu.csv',
                '--settings_yaml',args.source_run/'config/settings.yaml',
                '--mode',mode,'--max_iterations',args.iterations,'--max_threads',args.threads,
                '--linear_solver',args.linear_solver,'--linear_eta',args.linear_eta,
                '--max_linear_iterations',args.max_linear_iterations])
            result=json.loads((output/'result.json').read_text())
            if not result['usable'] or result['frames']!=graph['frames']:
                raise ValueError('Solver did not produce a usable full result')
            if mode!='visual_rig' and result['imu_intervals']!=graph['frames']-1:
                raise ValueError('Not all native IMU intervals were optimized')
        if args.stage in ('export','all'):
            run_stage(arm,'export',[sys.executable,ROOT/'pipeline/run/export_lamaria_colmap_refinement.py',
                '--source-run',args.source_run,'--model',output,'--run-dir',run,'--sift-work',work,
                '--native-timestamps',DATA/'euroc/timestamps.txt'])
        if args.stage in ('score','all'):
            run_stage(arm,'score',[sys.executable,ROOT/'pipeline/run/score_lamaria.py',
                '--run',run,'--assets',ASSETS,'--sequence','sequence_1_19',
                '--source-timestamps',DATA/'euroc/timestamps.txt'])
        if args.stage in ('viz','all'):
            viz=suite/(mode+'_evaluation')
            run_stage(arm,'visualize',[sys.executable,ROOT/'pipeline/viz/make_lamaria_output.py',
                '--run-dir',run,'--output-dir',viz,'--dataset',DATA/'euroc',
                '--timestamps',run/'config/native_timestamps_ns.txt','--config',run/'config/settings.yaml',
                '--gt',ASSETS/'gt_dense.txt','--evaluation-score',run/'lamaria_score/scores.json'])
            run_stage(arm,'verify_rrd',[Path(sys.executable).with_name('rerun'),'rrd','verify',viz/'run.rrd'])
            link=suite/(mode+'_evaluation.rrd')
            if not link.exists(): link.symlink_to(viz/'run.rrd')
        print(f'{mode}: requested stages complete',flush=True)


if __name__=='__main__':
    main()
