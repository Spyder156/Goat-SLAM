#!/usr/bin/env python3
"""Run the actual fixed-intrinsic FullCalibrationBA on an observable synthetic rig."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[3]
NAME = 'orbslam3_lamaria_periodic_vi_20261003'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=ROOT/'build'/NAME)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve(strict=True)
    config = args.config.resolve(strict=True)
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'include').mkdir()
    header = (build/'sources/include/System.h').read_text()
    if header.count('\nprivate:') != 1:
        raise RuntimeError('Unexpected System visibility declaration')
    (out/'include/System.h').write_text(header.replace('\nprivate:', '\npublic: // fixture visibility only; no layout or behavior change'))
    vendor = ROOT/'third_party/ORB_SLAM3'
    source = Path(__file__).with_name('periodic_vi_contract.cpp')
    mounts = ['docker', 'run', '--rm', '--network', 'none', '--cpus', '1', '--user', f'{os.getuid()}:{os.getgid()}',
              '-v', f'{out}:/audit', '-v', f'{source.parent}:/tests:ro', '-v', f'{build}:/build:ro',
              '-v', f'{vendor}:/orb:ro', '-v', f'{ROOT}/third_party/ELSED:/elsed:ro', '-v', f'{config}:/config.yaml:ro']
    compiler = mounts+['--entrypoint', '/usr/bin/c++', 'insv/orbslam3:gdb']
    flags = ['-O1', '-march=native', '-std=c++11', '-DCOMPILEDWITHC11', '-I/audit/include',
             '-I/build/sources', '-I/build/sources/include', '-I/build/sources/include/CameraModels',
             '-I/elsed/src', '-I/orb', '-I/orb/Thirdparty/Sophus', '-isystem', '/usr/include/eigen3',
             '-isystem', '/usr/local/include/opencv4']
    link = shlex.split((vendor/'build/CMakeFiles/mono_rig_euroc.dir/link.txt').read_text())[1:]
    for i, value in enumerate(link):
        if value.startswith('CMakeFiles/'):
            link[i] = '/audit/fixture.o'
        elif value == '../Examples/Monocular-Inertial/mono_rig_euroc':
            link[i] = '/audit/fixture'
        elif value == '../lib/libORB_SLAM3.so':
            link[i] = '/build/lib/libORB_SLAM3.so'
        elif value.startswith('../Thirdparty/'):
            link[i] = '/orb/'+value[3:]
    commands = {'compile': compiler+flags+['-c', '/tests/periodic_vi_contract.cpp', '-o', '/audit/fixture.o'], 'link': compiler+link}
    with (out/'build.log').open('w') as log:
        for cmd in commands.values():
            subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
    runtime = mounts+['-e', 'LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                      '--entrypoint', '/audit/fixture', 'insv/orbslam3:gdb', '/config.yaml', '/audit/vocabulary.txt']
    with (out/'fixture.log').open('w') as log:
        result = subprocess.run(runtime, stdout=log, stderr=subprocess.STDOUT, timeout=180)
    lines = (out/'fixture.log').read_text().splitlines()
    checks = [line for line in lines if line.startswith(('CHECK ', 'GEOMETRY ', 'RESULT '))]
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    report = {'returncode': result.returncode, 'checks': checks, 'runtime_command': runtime,
              'commands': commands, 'production_build': str(build), 'config': str(config),
              'sha256': {str(p): sha(p) for p in (source, Path(__file__), config, build/'lib/libORB_SLAM3.so', build/'build_manifest.json')}}
    (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print('\n'.join(checks))
    if result.returncode:
        raise SystemExit('Production periodic VI contract failed; see fixture.log')


if __name__ == '__main__':
    main()
