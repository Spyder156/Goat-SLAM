#!/usr/bin/env python3
"""Build only a regression harness, then test the selected production library."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

PROJECT=Path(__file__).resolve().parents[3]
CASES=('mapper_existing','mapper_new_stereo','add_idempotent','replace_complementary',
       'replace_conflict','fuse_right','fuse_left','ba_stereo_depth',
       'add_conflict','add_conflict_other','erase_last','fuse_sim3_left',
       'ba_mapper_depth','ba_monocular_control')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--expect-baseline-failures',action='store_true')
    args=parser.parse_args();build=args.build.resolve(strict=True);out=args.out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    source=Path(__file__).with_name('observation_contract.cpp')
    vendor=PROJECT/'third_party/ORB_SLAM3'
    mounts=['docker','run','--rm','--network','none','--cpus','2','--user',f'{os.getuid()}:{os.getgid()}',
            '-v',f'{out}:/audit','-v',f'{source.parent}:/tests:ro','-v',f'{build}:/build:ro',
            '-v',f'{vendor}:/orb:ro','-v',f'{PROJECT}/third_party/ELSED:/elsed:ro']
    compiler=mounts+['--entrypoint','/usr/bin/c++','insv/orbslam3:gdb']
    flags=['-Wall','-O1','-march=native','-std=c++11','-DCOMPILEDWITHC11',
           '-I/elsed/src','-I/build/sources','-I/build/sources/include',
           '-I/build/sources/include/CameraModels','-I/orb','-I/orb/Thirdparty/Sophus',
           '-isystem','/usr/include/eigen3','-isystem','/usr/local/include/opencv4']
    link=shlex.split((vendor/'build/CMakeFiles/mono_rig_euroc.dir/link.txt').read_text())[1:]
    for i,arg in enumerate(link):
        if arg.startswith('CMakeFiles/'):link[i]='/audit/fixture.o'
        elif arg=='../Examples/Monocular-Inertial/mono_rig_euroc':link[i]='/audit/fixture'
        elif arg=='../lib/libORB_SLAM3.so':link[i]='/build/lib/libORB_SLAM3.so'
        elif arg.startswith('../Thirdparty/'):link[i]='/orb/'+arg[3:]
    commands=[compiler+flags+['-c','/tests/observation_contract.cpp','-o','/audit/fixture.o'],compiler+link]
    with (out/'build.log').open('w') as log:
        for command in commands:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    runtime=mounts+['-e','LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                    '--entrypoint','/audit/fixture','insv/orbslam3:gdb']
    results={}
    for case in CASES:
        completed=subprocess.run(runtime+[case],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
        (out/(case+'.log')).write_text(completed.stdout)
        results[case]={'returncode':completed.returncode,'checks':[line for line in completed.stdout.splitlines() if line.startswith(('CHECK ','OBS ','RECENT ','FUSED ','BA ','RESULT ','[AUDIT] GBA edges:'))]}
        print(case,completed.returncode,flush=True)
        for line in results[case]['checks']:print('  '+line,flush=True)
    provenance={'build':str(build),'library_sha256':sha(build/'lib/libORB_SLAM3.so'),
                'manifest_sha256':sha(build/'build_manifest.json'),'fixture_sha256':sha(source),
                'runner_sha256':sha(Path(__file__)),'compile_commands':commands,'runtime_command':runtime,
                'results':results,'estimator_sources_modified':False}
    (out/'results.json').write_text(json.dumps(provenance,indent=2)+'\n')
    failures=[name for name,result in results.items() if result['returncode']!=0]
    if args.expect_baseline_failures:
        expected=set(CASES)-{'ba_stereo_depth','ba_monocular_control','erase_last'}
        if not expected.issubset(failures) or any(results[name]['returncode']!=1 for name in expected) or any(results[name]['returncode'] for name in ('ba_stereo_depth','ba_monocular_control')):
            raise SystemExit('Baseline did not show the expected contract failures and passing BA controls; inspect logs')
        # Empty-map iterator dereference is UB in baseline: record whatever it
        # does without requiring a crash. Candidate must pass its strict checks.
    elif failures:raise SystemExit('Failed observation contracts: '+', '.join(failures))


if __name__=='__main__':main()
