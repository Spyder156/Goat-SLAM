#!/usr/bin/env python3
import hashlib,json,os,shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
PACKAGE=Path(__file__).resolve().parents[1]
BUILD=ROOT/'build/orbslam3_lamaria_tracking_memory_20261003'
OUT=BUILD/'tests/tracking_memory';OUT.mkdir(parents=True,exist_ok=True)
VENDOR=ROOT/'third_party/ORB_SLAM3'
base=['docker','run','--rm','--network','none','--cpus','1','--user',f'{os.getuid()}:{os.getgid()}',
      '-v',str(BUILD)+':/build:ro','-v',str(OUT)+':/audit','-v',str(PACKAGE/'tests')+':/tests:ro',
      '-v',str(VENDOR)+':/orb:ro','-v',str(ROOT/'third_party/ELSED')+':/elsed:ro']
flags=['-Wall','-O1','-march=native','-std=c++11','-DCOMPILEDWITHC11',
       '-I/build/sources/src','-I/build/sources','-I/build/sources/include','-I/build/sources/include/CameraModels',
       '-I/elsed/src','-I/orb','-I/orb/Thirdparty/Sophus','-isystem','/usr/include/eigen3','-isystem','/usr/local/include/opencv4']
compiler=base+['--entrypoint','/usr/bin/c++','insv/orbslam3:gdb']
link=shlex.split((VENDOR/'build/CMakeFiles/mono_rig_euroc.dir/link.txt').read_text())[1:]
for i,arg in enumerate(link):
    if arg.startswith('CMakeFiles/'):link[i]='/audit/fixture.o'
    elif arg=='../Examples/Monocular-Inertial/mono_rig_euroc':link[i]='/audit/fixture'
    elif arg=='../lib/libORB_SLAM3.so':link[i]='/build/lib/libORB_SLAM3.so'
    elif arg.startswith('../Thirdparty/'):link[i]='/orb/'+arg[3:]
commands=[compiler+flags+['-c','/tests/tracking_memory_contract.cpp','-o','/audit/fixture.o'],compiler+link]
with (OUT/'build.log').open('w') as log:
    for command in commands: subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
cases=['normal','short_gap','descriptor_copy','rejected','expired','map_boundary','map_epoch','owner_boundary','time_boundary',
       'outside_projection','scale_gate','ratio_gate','camera_owner','occupied_feature','occupied_landmark','erased_point','bad_point']
runtime=base+['-e','LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
              '--entrypoint','/audit/fixture','insv/orbslam3:gdb']
results={}
for case in cases:
    result=subprocess.run(runtime+[case],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=60)
    (OUT/(case+'.log')).write_text(result.stdout)
    results[case]={'returncode':result.returncode,'checks':[x for x in result.stdout.splitlines() if x.startswith(('CHECK ','RESULT '))]}
    print(case,result.returncode,results[case]['checks'],flush=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
report={'library_sha256':sha(BUILD/'lib/libORB_SLAM3.so'),'helper_sha256':sha(BUILD/'sources/src/LamariaTrackingMemory.h'),
        'contract_sha256':sha(PACKAGE/'tests/tracking_memory_contract.cpp'),'commands':commands,'runtime':runtime,'results':results}
(OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
if any(v['returncode'] for v in results.values()):raise SystemExit('Contract failed')
