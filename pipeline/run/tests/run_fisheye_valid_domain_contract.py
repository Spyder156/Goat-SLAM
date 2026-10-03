#!/usr/bin/env python3
"""Build a production-linked domain/ingestion regression, with SDK references."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import cv2
import numpy as np
from projectaria_tools.core import calibration, sophus

PROJECT=Path(__file__).resolve().parents[3]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--expect-baseline-failures',action='store_true')
    args=parser.parse_args();build=args.build.resolve(strict=True);out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    config=args.config.resolve(strict=True);fs=cv2.FileStorage(str(config),cv2.FILE_STORAGE_READ)
    names=['fx','fy','cx','cy']+[f'k{i}' for i in range(1,7)]+['p1','p2','s1','s2','s3','s4']; rows=[]
    for cam in range(2):
        p=np.asarray([fs.getNode(f'Camera{cam+1}.{name}').real() for name in names],np.float32).astype(float)
        radius=fs.getNode(f'Camera{cam+1}.validRadius').real();angle=fs.getNode(f'Camera{cam+1}.maxSolidAngle').real()
        assert radius>0 and 0<angle<np.pi/2
        sdk=calibration.CameraCalibration('fixture',calibration.CameraModelType.FISHEYE624,np.r_[p[0],p[2:]],sophus.SE3(),640,480,radius,angle,'')
        for v in range(0,480,8):
            for u in range(0,640,8):
                adjusted=[u,p[3]+(v-p[3])*p[0]/p[1]];ray=sdk.unproject(adjusted)
                valid=ray is not None
                if valid:ray=ray/np.linalg.norm(ray);valid=np.arccos(ray[2])<=angle
                rows.append([cam,u,v,int(valid),*(ray if valid else [0,0,0])])
    fs.release();np.savetxt(out/'sdk_reference.txt',np.asarray(rows),fmt='%.17g')
    source=Path(__file__).with_name('fisheye_valid_domain_contract.cpp');vendor=PROJECT/'third_party/ORB_SLAM3'
    mounts=['docker','run','--rm','--network','none','--cpus','2','--user',f'{os.getuid()}:{os.getgid()}',
            '-v',f'{out}:/audit','-v',f'{source.parent}:/tests:ro','-v',f'{build}:/build:ro',
            '-v',f'{vendor}:/orb:ro','-v',f'{PROJECT}/third_party/ELSED:/elsed:ro','-v',f'{config}:/config.yaml:ro']
    compiler=mounts+['--entrypoint','/usr/bin/c++','insv/orbslam3:gdb']
    flags=['-O1','-march=native','-std=c++11','-DCOMPILEDWITHC11','-I/elsed/src','-I/build/sources',
           '-I/build/sources/include','-I/build/sources/include/CameraModels','-I/orb','-I/orb/Thirdparty/Sophus',
           '-isystem','/usr/include/eigen3','-isystem','/usr/local/include/opencv4']
    link=shlex.split((vendor/'build/CMakeFiles/mono_rig_euroc.dir/link.txt').read_text())[1:]
    for i,value in enumerate(link):
        if value.startswith('CMakeFiles/'):link[i]='/audit/fixture.o'
        elif value=='../Examples/Monocular-Inertial/mono_rig_euroc':link[i]='/audit/fixture'
        elif value=='../lib/libORB_SLAM3.so':link[i]='/build/lib/libORB_SLAM3.so'
        elif value.startswith('../Thirdparty/'):link[i]='/orb/'+value[3:]
    with (out/'build.log').open('w') as log:
        subprocess.run(compiler+flags+['-c','/tests/fisheye_valid_domain_contract.cpp','-o','/audit/fixture.o'],stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(compiler+link,stdout=log,stderr=subprocess.STDOUT,check=True)
    runtime=mounts+['-e','LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                    '--entrypoint','/audit/fixture','insv/orbslam3:gdb']
    results={}
    for case in ('cached','orb','empty','empty_left','empty_right'):
        (out/case).mkdir()
        completed=subprocess.run(runtime+['/config.yaml','/audit/sdk_reference.txt',case,'/audit/'+case],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
        (out/(case+'.log')).write_text(completed.stdout)
        checks=[line for line in completed.stdout.splitlines() if line.startswith(('CHECK ','DOMAIN ','RESULT ','[FISHEYE-DOMAIN]'))]
        results[case]={'returncode':completed.returncode,'checks':checks};print(case,completed.returncode,flush=True)
        for line in checks:print(line,flush=True)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    report={'build':str(build),'config':str(config),'library_sha256':sha(build/'lib/libORB_SLAM3.so'),
            'config_sha256':sha(config),'fixture_sha256':sha(source),'runner_sha256':sha(Path(__file__)),
            'sdk_reference_sha256':sha(out/'sdk_reference.txt'),'results':results,'estimator_sources_modified':False}
    (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.expect_baseline_failures:
        if not all(r['returncode']==1 for r in results.values()):raise SystemExit('Baseline did not reproduce expected domain/filter contract failures')
    elif any(r['returncode'] for r in results.values()):raise SystemExit('Candidate domain/filter regression failed')


if __name__=='__main__':main()
