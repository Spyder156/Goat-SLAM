#!/usr/bin/env python3
"""Build the isolated BabyFeatures candidate using verified frozen v2 objects."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path(__file__).resolve().parent
CHANGED = ['src/Tracking.cc', 'src/Frame.cc', 'src/Optimizer.cc']
ADDED = ['src/LamariaBabySolver.cc']


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path,
                        default=ROOT/'build/orbslam3_lamaria_online_full_v2')
    parser.add_argument('--out', type=Path,
                        default=ROOT/'build/orbslam3_lamaria_biashold_kin_20261004')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 4:
        parser.error('--jobs must be 1 through 4')
    base, out = args.base.resolve(strict=True), args.out.resolve()
    if out == base or base in out.parents or out in base.parents:
        raise SystemExit('A separate candidate build directory is required')
    vendor = ROOT/'third_party/ORB_SLAM3'
    manifest = json.loads((base/'build_manifest.json').read_text())
    for table in ('patched_sources_sha256', 'headers_sha256'):
        for relative, expected in manifest[table].items():
            if sha(base/'sources'/relative) != expected:
                raise SystemExit('Changed frozen input: '+relative)
    for relative, field in [('lib/libORB_SLAM3.so', 'library_sha256'),
                            ('bin/stereo_lamaria_euroc', 'runner_sha256')]:
        if sha(base/relative) != manifest[field]:
            raise SystemExit('Changed frozen product: '+relative)
    image = manifest['image']
    image_id = subprocess.check_output(
        ['docker', 'image', 'inspect', '--format', '{{.Id}}', image], text=True).strip()
    if image_id != manifest['image_id']:
        raise SystemExit('Frozen build image differs')
    spec = importlib.util.spec_from_file_location(
        'build_lamaria', ROOT/'pipeline/run/build_lamaria.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    builder.verify_baseline(vendor, PACKAGE/'base_manifest.json')
    for name in ('sources', 'objects', 'lib', 'bin', 'logs'):
        (out/name).mkdir(parents=True, exist_ok=True)
    (out/'build_manifest.json').unlink(missing_ok=True)
    source = out/'sources'
    shutil.rmtree(source)
    shutil.copytree(base/'sources', source)
    subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1',
                    '-d', str(source), '-i', str(PACKAGE/'delta.patch')], check=True)
    # The package also preserves v2's added files for reproduction from vendor.
    shutil.copytree(PACKAGE/'new_files', source, dirs_exist_ok=True)
    # Existing class layouts must be identical when their objects are reused.
    for relative, expected in manifest['headers_sha256'].items():
        if sha(source/relative) != expected:
            raise SystemExit('Existing header changed; full rebuild required: '+relative)
    replacements, reused = {}, {}
    for relative in manifest['compile_sources']:
        filename = relative.replace('/', '_')+'.o'
        obj = base/'objects'/filename
        record = json.loads((base/'objects'/(filename+'.json')).read_text())
        if sha(obj) != record['sha256']:
            raise SystemExit('Changed frozen object: '+relative)
        if relative not in CHANGED:
            if not relative.startswith('elsed/') and sha(source/relative) != sha(base/'sources'/relative):
                raise SystemExit('Unlisted source change: '+relative)
            shutil.copy2(obj, out/'objects'/filename)
            shutil.copy2(base/'objects'/(filename+'.json'), out/'objects'/(filename+'.json'))
            reused[relative] = record['sha256']
        replacements['CMakeFiles/ORB_SLAM3.dir/'+relative+'.o'] = '/work/objects/'+filename
    flags = ['-I/work/sources', '-I/work/sources/include',
             '-I/work/sources/include/CameraModels', '-I/work/sources/src']
    flag_text = (vendor/'build/CMakeFiles/ORB_SLAM3.dir/flags.make').read_text()
    for key in ('CXX_FLAGS', 'CXX_DEFINES', 'CXX_INCLUDES'):
        flags += shlex.split(re.search(r'^'+key+r'\s*=\s*(.*)$', flag_text, re.M).group(1))
    mounts = ['docker', 'run', '--rm', '--network', 'none', '--cpus', '1',
              '--user', f'{os.getuid()}:{os.getgid()}',
              '-v', str(vendor)+':/orb:ro',
              '-v', str(ROOT/'third_party/ELSED')+':/elsed:ro',
              '-v', str(out)+':/work', '--entrypoint', '/usr/bin/c++', image]
    dependency_hashes = {str(p.relative_to(source)): sha(p)
                         for p in source.rglob('*')
                         if p.is_file() and p.suffix in ('.h', '.hpp')}
    compile_commands = {}

    def compile_one(relative):
        filename = relative.replace('/', '_')+'.o'
        obj = out/'objects'/filename
        metadata = obj.with_name(obj.name+'.json')
        fingerprint = hashlib.sha256(json.dumps({
            'source': sha(source/relative), 'dependencies': dependency_hashes,
            'flags': flags, 'image_id': image_id}, sort_keys=True).encode()).hexdigest()
        previous = json.loads(metadata.read_text()) if metadata.exists() else {}
        command = mounts+flags+['-c', '/work/sources/'+relative,
                                '-o', '/work/objects/'+filename]
        compile_commands[relative] = command
        if not (previous.get('fingerprint') == fingerprint and obj.exists()
                and previous.get('sha256') == sha(obj)):
            print('Compiling '+relative, flush=True)
            with (out/'logs'/(filename+'.log')).open('w') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            metadata.write_text(json.dumps({'fingerprint': fingerprint, 'sha256': sha(obj)})+'\n')
        else:
            print('Reusing candidate '+relative, flush=True)
        return '/work/objects/'+filename

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        products = list(pool.map(compile_one, CHANGED+ADDED))
    link = shlex.split((vendor/'build/CMakeFiles/ORB_SLAM3.dir/link.txt').read_text())
    rewritten = builder.rewrite_link(link, replacements, '/work/lib/libORB_SLAM3.so')
    rewritten += products[len(CHANGED):]
    link_command = mounts+rewritten
    with (out/'logs/incremental_link.log').open('w') as log:
        subprocess.run(link_command, stdout=log, stderr=subprocess.STDOUT, check=True)
    shutil.copy2(base/'bin/stereo_lamaria_euroc', out/'bin/stereo_lamaria_euroc')
    result = dict(manifest)
    result.update({
        'output': str(out), 'patch_dir': str(PACKAGE),
        'incremental_base': str(base),
        'base_build_manifest_sha256': sha(base/'build_manifest.json'),
        'incremental_recompiled_sources': CHANGED+ADDED,
        'compile_sources': list(manifest['compile_sources'])+ADDED,
        'reused_objects_sha256': reused,
        'patch_sha256': sha(PACKAGE/'geometry.patch'),
        'delta_patch_sha256': sha(PACKAGE/'delta.patch'),
        'compile_commands': compile_commands, 'link_command': link_command,
        'library_sha256': sha(out/'lib/libORB_SLAM3.so'),
        'runner_sha256': sha(out/'bin/stereo_lamaria_euroc'),
        'patched_sources_sha256': {
            relative: sha(source/relative)
            for relative in list(manifest['patched_sources_sha256'])+ADDED},
        'headers_sha256': {str(p.relative_to(source)): sha(p)
                           for p in (source/'include').rglob('*') if p.is_file()},
        'helper_dependencies_sha256': dependency_hashes,
    })
    (out/'build_manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    print('Built '+str(out), flush=True)


if __name__ == '__main__':
    main()
