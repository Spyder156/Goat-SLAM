#!/usr/bin/env python3
"""Exercise metric initialization using production classes and optimizer sources."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

PROJECT=Path(__file__).resolve().parents[3]
CASES=('jacobians','lever_arm','scale_small','scale_large','stationary','weak_stereo')

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--staging',type=Path,help='Compile changed production objects ahead of existing library')
    parser.add_argument('--edge-baseline-only',action='store_true',help='Expect old production scale/lever-arm contracts to fail')
    parser.add_argument('--joint',action='store_true',help='Include joint geometry and boundary-observer tests')
    parser.add_argument('--boundary-conflict-diagnostic',action='store_true',help='Also record the known metric-accuracy limitation with deliberately wrong fixed boundary geometry')
    args=parser.parse_args();build=args.build.resolve(strict=True);out=args.out.resolve()
    out.mkdir(parents=True,exist_ok=True)
    source=Path(__file__).with_name('metric_inertial_contract.cpp');vendor=PROJECT/'third_party/ORB_SLAM3'
    staging=args.staging.resolve(strict=True) if args.staging else None
    mounts=['docker','run','--rm','--network','none','--cpus','2','--user',f'{os.getuid()}:{os.getgid()}',
            '-v',f'{out}:/audit','-v',f'{source.parent}:/tests:ro','-v',f'{build}:/build:ro',
            '-v',f'{vendor}:/orb:ro','-v',f'{PROJECT}/third_party/ELSED:/elsed:ro']
    if staging:mounts+=['-v',f'{staging}:/staging:ro']
    compiler=mounts+['--entrypoint','/usr/bin/c++','insv/orbslam3:gdb']
    flags=['-Wall','-O1','-march=native','-std=c++11','-DCOMPILEDWITHC11']
    if staging:flags+=['-I/staging/include']
    flags+=['-I/elsed/src','-I/build/sources','-I/build/sources/include','-I/build/sources/include/CameraModels',
            '-I/orb','-I/orb/Thirdparty/Sophus','-isystem','/usr/include/eigen3','-isystem','/usr/local/include/opencv4']
    if args.edge_baseline_only:flags+=['-DMETRIC_EDGE_BASELINE_ONLY']
    link=shlex.split((vendor/'build/CMakeFiles/mono_rig_euroc.dir/link.txt').read_text())[1:]
    for i,arg in enumerate(link):
        if arg.startswith('CMakeFiles/'):link[i]='/audit/fixture.o'
        elif arg=='../Examples/Monocular-Inertial/mono_rig_euroc':link[i]='/audit/fixture'
        elif arg=='../lib/libORB_SLAM3.so':link[i]='/build/lib/libORB_SLAM3.so'
        elif arg.startswith('../Thirdparty/'):link[i]='/orb/'+arg[3:]
    commands=[]
    if staging and not args.edge_baseline_only:
        objects=[]
        for name in ('G2oTypes','Optimizer'):
            if not (staging/f'src/{name}.cc').exists():continue
            commands.append(compiler+flags+['-c',f'/staging/src/{name}.cc','-o',f'/audit/{name}.o'])
            objects.append(f'/audit/{name}.o')
        link[link.index('/audit/fixture.o')+1:link.index('/audit/fixture.o')+1]=objects
    commands += [compiler+flags+['-c','/tests/metric_inertial_contract.cpp','-o','/audit/fixture.o'],compiler+link]
    with (out/'build.log').open('w') as log:
        for command in commands:
            print('BUILD',command[-3:],flush=True)
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    runtime=mounts+['-e','LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                    '--entrypoint','/audit/fixture','insv/orbslam3:gdb']
    results={}
    cases=CASES[:2] if args.edge_baseline_only else CASES
    if args.joint:cases+=('joint_scale','joint_distorted','joint_boundary','joint_stationary')
    if args.boundary_conflict_diagnostic:cases+=('joint_boundary_conflict',)
    for case in cases:
        completed=subprocess.run(runtime+[case],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120)
        (out/(case+'.log')).write_text(completed.stdout)
        results[case]={'returncode':completed.returncode,'checks':[line for line in completed.stdout.splitlines() if line.startswith(('CHECK ','JACOBIAN ','LEVER_ARM ','PROPOSAL '))]}
        print(case,completed.returncode,flush=True)
        for line in results[case]['checks']:print('  '+line,flush=True)
    provenance={'build':str(build),'staging':str(staging) if staging else None,'library_sha256':sha(build/'lib/libORB_SLAM3.so'),
                'fixture_sha256':sha(source),'runner_sha256':sha(Path(__file__)),'compile_commands':commands,
                'runtime_command':runtime,'results':results}
    if staging:provenance['staging_sha256']={name:sha(staging/name) for name in ('src/G2oTypes.cc','src/Optimizer.cc','include/MetricInertialInitialization.h') if (staging/name).exists()}
    (out/'results.json').write_text(json.dumps(provenance,indent=2)+'\n')
    failures=[name for name,result in results.items() if result['returncode']!=0]
    if args.boundary_conflict_diagnostic and 'joint_boundary_conflict' in failures:
        diagnostic=results['joint_boundary_conflict']
        failed_checks=[line for line in diagnostic['checks'] if line.startswith('CHECK FAIL ')]
        if diagnostic['returncode']==1 and failed_checks==['CHECK FAIL joint output camera baseline recovers metric length within 2 percent']:
            failures.remove('joint_boundary_conflict')
            print('KNOWN LIMITATION: incorrect fixed boundary geometry biases metric length; accuracy failure retained in results.json',flush=True)
    if args.edge_baseline_only:
        if any(result['returncode']!=1 for result in results.values()):raise SystemExit('Baseline must exhibit contract failures, not crash')
    elif failures:raise SystemExit('Failed metric initialization contracts: '+', '.join(failures))

if __name__=='__main__':main()
