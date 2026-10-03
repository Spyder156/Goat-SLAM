#!/usr/bin/env python3
"""Launch one capped solver, preserving actual per-arm binary/options provenance."""
import hashlib
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path
import time

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    argv = sys.argv[1:]
    if len(argv) % 2:
        raise ValueError('Solver arguments require --key value pairs')
    args = dict(zip(argv[::2], argv[1::2]))
    for key in ('--input_path', '--output_path', '--imu_csv', '--settings_yaml'):
        if key in args:
            args[key] = str(relocated_path(args[key]))
    output = Path(args['--output_path']).resolve()
    override_path = output.parent / 'solver_override.json'
    override = json.loads(override_path.read_text()) if override_path.exists() else {}
    allowed = {'binary', 'binary_sha256', 'explicit_schur', 'memory_limit',
               'cpu_limit', 'source_manifest_path', 'min_available_memory_gib',
               'runtime_libraries', 'release_observation_metadata'}
    if set(override) - allowed:
        raise ValueError('Unknown solver override keys: ' + str(set(override) - allowed))
    binary = relocated_path(override.get('binary', os.environ.get('LAMARIA_BA_BINARY',
                  ROOT / 'build/colmap_lamaria_ba/colmap_lamaria_ba'))).resolve(strict=True)
    binary_hash = sha(binary)
    if override and override.get('binary_sha256') != binary_hash:
        raise ValueError('Per-arm binary does not match its frozen override hash')
    runtime_libraries = {}
    for library, expected_hash in override.get('runtime_libraries', {}).items():
        path = relocated_path(library).resolve(strict=True)
        actual_hash = sha(path)
        if actual_hash != expected_hash:
            raise ValueError('Frozen runtime library hash changed: ' + str(path))
        runtime_libraries[str(path)] = actual_hash
    if 'explicit_schur' in override:
        args['--explicit_schur'] = str(int(override['explicit_schur']))
    if 'release_observation_metadata' in override:
        args['--release_observation_metadata'] = str(int(override['release_observation_metadata']))
    memory = str(override.get('memory_limit', os.environ.get('LAMARIA_BA_MEMORY', '26g')))
    cpus = str(override.get('cpu_limit', os.environ.get('LAMARIA_BA_CPUS', '4')))
    minimum_available = float(override.get('min_available_memory_gib', 0))
    output.mkdir(parents=True, exist_ok=True)
    wait_started = time.time()
    memory_checks = []
    if minimum_available:
        while True:
            fields = {line.split(':')[0]: int(line.split()[1])
                      for line in Path('/proc/meminfo').read_text().splitlines()}
            available = fields['MemAvailable'] / 1024 ** 2
            sample = dict(time=time.time(), available_gib=available,
                          required_available_gib=minimum_available,
                          reason='reserve solver cap plus host/export/render headroom')
            memory_checks.append(sample)
            with (output / 'resource_wait.jsonl').open('a') as stream:
                stream.write(json.dumps(sample) + '\n')
            if available >= minimum_available:
                break
            if len(memory_checks) % 6 == 1:
                print(f'WAIT_RESOURCE available_gib={available:.3f} required_gib={minimum_available} '
                      f'planned_cap={memory}; waiting for preceding export/render or other owned work', flush=True)
            time.sleep(10)
    effective_args = [str(x) for pair in args.items() for x in pair]
    command = ['docker', 'run', '--rm', '--user', f'{os.getuid()}:{os.getgid()}',
               '--entrypoint', '', '--label', 'mecka.task=lamaria-global-ba',
               '--memory', memory, '--memory-swap', memory, '--cpus', cpus,
               '-e', 'OPENBLAS_NUM_THREADS=1', '-e', 'OMP_NUM_THREADS=4',
               '-v', f'{ROOT}:{ROOT}', '-v', '/media:/media', '-w', str(ROOT),
               'fisheye-slam', str(binary), *effective_args]
    manifest = dict(authority='actual execution; overrides initial planned suite solver fields',
                    binary=str(binary), binary_sha256=binary_hash,
                    runtime_libraries=runtime_libraries,
                    original_solver_arguments=argv, effective_solver_arguments=effective_args,
                    memory_limit=memory, memory_plus_swap_limit=memory, cpu_limit=cpus,
                    minimum_available_memory_gib=minimum_available,
                    resource_wait_seconds=time.time()-wait_started,
                    available_memory_gib_at_launch=memory_checks[-1]['available_gib'] if memory_checks else None,
                    docker_command=command,
                    override_path=str(override_path) if override else None,
                    override_sha256=sha(override_path) if override else None)
    suite_manifest = output.parent.parent.parent / 'offline_experiment.json'
    if suite_manifest.exists():
        manifest['suite_manifest'] = str(suite_manifest)
        manifest['suite_manifest_sha256'] = sha(suite_manifest)
    if override.get('source_manifest_path'):
        source_manifest = relocated_path(override['source_manifest_path']).resolve(strict=True)
        manifest['source_manifest_path'] = str(source_manifest)
        manifest['source_manifest_sha256'] = sha(source_manifest)
    manifest['launcher_path'] = str(Path(__file__).resolve())
    manifest['launcher_sha256'] = sha(Path(__file__))
    target = output / 'execution_manifest.json'
    if target.exists() and json.loads(target.read_text()) != manifest:
        raise ValueError('Execution changed; archive the existing arm before retrying')
    target.write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'RESOURCE_LIMITS memory={memory} memory_plus_swap={memory} cpus={cpus}', flush=True)
    print(f'EXECUTION_MANIFEST {target} binary_sha256={binary_hash}', flush=True)
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
