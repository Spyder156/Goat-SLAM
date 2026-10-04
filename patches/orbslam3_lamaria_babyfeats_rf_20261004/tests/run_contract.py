#!/usr/bin/env python3
"""Compile a native BabyFeatures numerical fixture against the candidate library."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[3]


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
    with (out/'build.log').open('w') as log:
        subprocess.run(compiler+flags+['-c', '/tests/baby_solver_contract.cpp',
                       '-o', '/audit/fixture.o'], stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(compiler+link, stdout=log, stderr=subprocess.STDOUT, check=True)
    runtime = mounts+['-e', 'LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                      '--entrypoint', '/audit/fixture', manifest['image']]
    result = subprocess.run(runtime, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=120)
    (out/'fixture.log').write_text(result.stdout)
    record = {'returncode': result.returncode, 'output': result.stdout,
              'library_sha256': manifest['library_sha256'],
              'fixture_sha256': sha(tests/'baby_solver_contract.cpp'),
              'build_manifest_sha256': sha(build/'build_manifest.json')}
    (out/'results.json').write_text(json.dumps(record, indent=2)+'\n')
    print(result.stdout, end='', flush=True)
    if result.returncode:
        raise SystemExit('BabyFeatures contract failed; see fixture.log')


if __name__ == '__main__':
    main()
