#!/usr/bin/env python3
"""Compile the native BabyFeatures fixture plus the bias-hold and Baby-assist fixtures against the candidate library."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[3]
FIXTURES = [('baby_solver_contract.cpp', 'fixture', None),
            ('bias_hold_contract.cpp', 'bias_hold', 'PASS 9 bias-hold checks'),
            ('baby_assist_contract.cpp', 'baby_assist', 'PASS 10 baby-assist checks')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve(strict=True)
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    vendor = ROOT/'third_party/ORB_SLAM3'
    tests = Path(__file__).resolve().parent
    manifest = json.loads((build/'build_manifest.json').read_text())
    sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
    if sha(build/'lib/libORB_SLAM3.so') != manifest['library_sha256']:
        raise SystemExit('Candidate library differs from build manifest')
    mounts = ['docker', 'run', '--rm', '--network', 'none', '--cpus', '1',
              '--user', f'{os.getuid()}:{os.getgid()}', '-v', str(out)+':/audit',
              '-v', str(tests)+':/tests:ro', '-v', str(build)+':/build:ro',
              '-v', str(vendor)+':/orb:ro',
              '-v', str(ROOT/'third_party/ELSED')+':/elsed:ro']
    compiler = mounts+['--entrypoint', '/usr/bin/c++', manifest['image']]
    flags = ['-O1', '-march=native', '-std=c++11', '-DCOMPILEDWITHC11',
             '-I/build/sources', '-I/build/sources/include',
             '-I/build/sources/include/CameraModels', '-I/build/sources/src',
             '-I/elsed/src', '-I/orb', '-I/orb/Thirdparty/Sophus',
             '-isystem', '/usr/include/eigen3', '-isystem', '/usr/local/include/opencv4']
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
    runtime = mounts+['-e', 'LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                      '--entrypoint', '/audit/fixture', manifest['image']]
    record = {'library_sha256': manifest['library_sha256'], 'build_manifest_sha256': sha(build/'build_manifest.json'), 'fixtures': {}}
    failed = []
    with (out/'build.log').open('w') as log:
        for source, name, expect in FIXTURES:
            this_link = [v.replace('/audit/fixture.o', f'/audit/{name}.o').replace('/audit/fixture', f'/audit/{name}') if v.startswith('/audit/') else v for v in link]
            subprocess.run(compiler+flags+['-c', f'/tests/{source}', '-o', f'/audit/{name}.o'], stdout=log, stderr=subprocess.STDOUT, check=True)
            subprocess.run(compiler+this_link, stdout=log, stderr=subprocess.STDOUT, check=True)
    for source, name, expect in FIXTURES:
        this_runtime = [v.replace('/audit/fixture', f'/audit/{name}') if v == '/audit/fixture' else v for v in runtime]
        result = subprocess.run(this_runtime, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
        (out/f'{name}.log').write_text(result.stdout)
        record['fixtures'][name] = {'returncode': result.returncode, 'output': result.stdout, 'fixture_sha256': sha(tests/source)}
        print(result.stdout, end='', flush=True)
        if result.returncode or (expect and expect not in result.stdout):
            failed.append(name)
    (out/'results.json').write_text(json.dumps(record, indent=2)+'\n')
    if failed:
        raise SystemExit('Contract failed: '+', '.join(failed))


if __name__ == '__main__':
    main()
