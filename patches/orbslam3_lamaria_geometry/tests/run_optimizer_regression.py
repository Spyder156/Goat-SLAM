#!/usr/bin/env python3
"""Build and run actual mixed-camera optimizer regressions and negative controls.

Requires pipeline/run/build_lamaria.py's completed build. Artifacts remain in
build/orbslam3_lamaria/tests/optimizer; vendor and test sources are read-only.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

TESTS = Path(__file__).resolve().parent
PROJECT = TESTS.parents[2]
sys.path.insert(0, str(PROJECT / 'pipeline/run'))
from build_lamaria import rewrite_link, sha256

SIGNATURES = (
    'int Optimizer::OptimizeSim3(',
    'void Optimizer::LocalBundleAdjustment(KeyFrame* pMainKF,',
    'void Optimizer::MergeInertialBA(',
)


def extract_functions(source):
    text = source.read_text()
    bodies = []
    for signature in SIGNATURES:
        start = text.index(signature)
        # These existing functions have their outer closing brace at column 0.
        end = text.index('\n}', start) + 2
        bodies.append(text[start:end])
    includes = text[:text.index('/// Withhold point edges')]
    helper = 'static bool sortByVal(const pair<MapPoint*,int>&a,const pair<MapPoint*,int>&b){return a.second<b.second;}\n'
    return includes + helper + '\n\n'.join(bodies) + '\n}\n', {
        'source_sha256': sha256(source),
        'function_sha256': dict(zip(SIGNATURES, (hashlib.sha256(b.encode()).hexdigest() for b in bodies))),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=PROJECT / 'build/orbslam3_lamaria')
    parser.add_argument('--config', type=Path, default=PROJECT / 'configs/orbslam3_lamaria/stereo_inertial.yaml')
    parser.add_argument('--without-baseline', action='store_true', help='Skip the expected pre-fix ASan failures')
    args = parser.parse_args()
    build = args.build.resolve()
    manifest = json.loads((build / 'build_manifest.json').read_text())
    vendor = Path(manifest['vendor']).resolve()
    image = manifest['image']
    if sha256(build / 'lib/libORB_SLAM3.so') != manifest['library_sha256']:
        raise ValueError('Library no longer matches its build manifest')
    fixed = build / 'sources/src/Optimizer.cc'
    if sha256(fixed) != manifest['patched_sources_sha256']['src/Optimizer.cc']:
        raise ValueError('Compiled Optimizer source no longer matches its build manifest')
    baseline = vendor / 'src/Optimizer.cc'
    expected = json.loads((TESTS.parent / 'base_manifest.json').read_text())['files']['src/Optimizer.cc']
    if not args.without_baseline and sha256(baseline) != expected:
        raise ValueError('Original Optimizer differs from the frozen negative-control baseline')
    image_id = subprocess.check_output(['docker', 'image', 'inspect', '--format', '{{.Id}}', image], text=True).strip()
    if image_id != manifest['image_id']:
        raise ValueError('Compiler image no longer matches the production build')
    out = build / 'tests/optimizer'
    out.mkdir(parents=True, exist_ok=True)
    config = out / 'settings.yaml'
    config.write_bytes(args.config.read_bytes())
    container_out = '/work/tests/optimizer/'
    docker = ['docker', 'run', '--rm', '--network', 'none', '--ulimit', 'core=0',
              '--user', f'{os.getuid()}:{os.getgid()}',
              '-v', f'{vendor}:/orb:ro', '-v', f'{PROJECT / "third_party/ELSED"}:/elsed:ro',
              '-v', f'{build}:/work', '-v', f'{TESTS}:/tests:ro']
    flags = ['-I/work/sources', '-I/work/sources/include', '-I/work/sources/include/CameraModels']
    definitions = (vendor / 'build/CMakeFiles/ORB_SLAM3.dir/flags.make').read_text()
    for key in ('CXX_FLAGS', 'CXX_DEFINES', 'CXX_INCLUDES'):
        match = re.search(r'^' + key + r'\s*=\s*(.*)$', definitions, re.M)
        if not match:
            raise ValueError('Missing compiler flag group: ' + key)
        flags += shlex.split(match.group(1))
    # Keep the library's Eigen SIMD ABI. Override optimization only, instrument
    # vector accesses inside the changed optimizer functions, preserve frames.
    flags += ['-O1', '-g1', '-fsanitize=address', '-fno-omit-frame-pointer']
    compiler = docker + ['--entrypoint', '/usr/bin/c++', image]
    kinds = ['library', 'fixed'] + ([] if args.without_baseline else ['baseline'])
    source_hashes = {}
    with (out / 'build.log').open('w') as log:
        subprocess.run(compiler + flags + ['-c', '/tests/optimizer_mixed_camera_probe.cpp',
                       '-o', container_out + 'fixture.o'], stdout=log, stderr=subprocess.STDOUT, check=True)
        for kind in kinds:
            if kind != 'library':
                code, hashes = extract_functions(baseline if kind == 'baseline' else fixed)
                unit = out / ('Optimizer.' + kind + '.cc')
                unit.write_text(code)
                source_hashes[kind] = hashes
                subprocess.run(compiler + flags + ['-c', container_out + unit.name,
                               '-o', container_out + kind + '.o'], stdout=log, stderr=subprocess.STDOUT, check=True)
            old_link = shlex.split((vendor / 'build/CMakeFiles/mono_rig_euroc.dir/link.txt').read_text())
            replacement = {a: container_out + 'fixture.o' for a in old_link if a.startswith('CMakeFiles/') and a.endswith('.o')}
            link = rewrite_link(old_link, replacement, container_out + 'probe_' + kind, runner=True)
            # GCC ASan PIE startup is unreliable in this existing container;
            # non-PIE changes the harness only, leaving the shared library intact.
            link += ['-fsanitize=address', '-no-pie']
            if kind != 'library':
                link += [container_out + kind + '.o']
            subprocess.run(compiler + link, stdout=log, stderr=subprocess.STDOUT, check=True)
            print('BUILT', kind, flush=True)
    results = []
    for kind in kinds:
        for mode in ('sim3', 'visual', 'inertial'):
            log = out / (kind + '_' + mode + '.log')
            name = 'lamaria-optimizer-test-' + str(os.getpid()) + '-' + kind + '-' + mode
            command = docker + ['--name', name,
                '-e', 'LD_LIBRARY_PATH=/work/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                # The fixture intentionally uses process-lifetime objects. Keep
                # address checks; disable leak checks and recursive SEGV handler.
                '-e', 'ASAN_OPTIONS=detect_leaks=0:handle_segv=0',
                '--entrypoint', container_out + 'probe_' + kind, image, container_out + 'settings.yaml', mode]
            with log.open('w') as handle:
                try:
                    completed = subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, timeout=60)
                    code = completed.returncode
                except subprocess.TimeoutExpired:
                    subprocess.run(['docker', 'kill', name], stdout=subprocess.DEVNULL, check=False)
                    code = 124
            text = log.read_text()
            passed = (code != 0 and 'AddressSanitizer: heap-buffer-overflow' in text) if kind == 'baseline' else (code == 0 and 'PASS ' + mode in text)
            results.append({'kind': kind, 'mode': mode, 'exit_code': code, 'expected_result_observed': passed,
                            'output': text[:32000], 'log': str(log)})
            print(('PASS' if passed else 'FAIL'), kind, mode, 'exit', code, flush=True)
    record = {'library_sha256': manifest['library_sha256'], 'fixture_sha256': sha256(TESTS / 'optimizer_mixed_camera_probe.cpp'),
              'config_sha256': sha256(config), 'image_id': image_id, 'sources': source_hashes, 'runs': results}
    (out / 'results.json').write_text(json.dumps(record, indent=2) + '\n')
    print(out / 'results.json')
    return 0 if all(r['expected_result_observed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
