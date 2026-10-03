#!/usr/bin/env python3
"""Build standalone solver against existing COLMAP 4.2 libraries; no vendor edits."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT=Path(__file__).resolve().parents[2]
def main():
    p=argparse.ArgumentParser();kind=p.add_mutually_exclusive_group();kind.add_argument('--contracts',action='store_true');kind.add_argument('--metadata-contracts',action='store_true');p.add_argument('--out',type=Path,default=ROOT/'build/colmap_lamaria_ba');p.add_argument('--ceres-prefix',type=Path);a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    name='metadata_contracts' if a.metadata_contracts else ('contracts' if a.contracts else 'colmap_lamaria_ba')
    source=ROOT/'pipeline/vi_ba_lamaria/src'/(name+'.cc')
    binary=out/name
    build=ROOT/'build/colmap_build'
    libs=['scene/libcolmap_scene.a','scene/libcolmap_scene_types.a','geometry/libcolmap_geometry.a','sensor/libcolmap_sensor.a','math/libcolmap_math.a','util/libcolmap_util.a','optim/libcolmap_optim.a','feature/libcolmap_feature_types.a']
    private_ceres=[]
    if a.ceres_prefix:
        prefix=a.ceres_prefix.resolve(strict=True)
        private_ceres=['-I'+str(prefix/'include'),'-L'+str(prefix/'lib'),
                       '-Wl,--disable-new-dtags','-Wl,-rpath,'+str(prefix/'lib')]
    command=['g++','-O2','-DNDEBUG','-std=c++17','-DCOLMAP_HASH_STD','-DGLOG_VERSION_MAJOR=0','-DGLOG_VERSION_MINOR=6',*private_ceres,
             '-I'+str(ROOT/'third_party/colmap_src/src'),'-I/usr/include/eigen3','-I/usr/include/opencv4',str(source),'-o',str(binary),
             '-Wl,--start-group',*[str(build/'src/colmap'/x) for x in libs],'-Wl,--end-group',
             '-lceres','-lglog','-lgflags','-lopencv_core','-lsqlite3','-lOpenImageIO','-lOpenImageIO_Util','-lboost_graph','-lboost_regex','-lcurl','-lcrypto','-lmetis','-lcholmod','-fopenmp','-lpthread']
    docker=['docker','run','--rm','--user',f'{os.getuid()}:{os.getgid()}','--entrypoint','',
            '-v',f'{ROOT}:{ROOT}','-v','/media:/media','-w',str(ROOT),'fisheye-slam',*command]
    (out/(name+'_build_command.json')).write_text(json.dumps(docker,indent=2)+'\n')
    print(shlex.join(docker),flush=True);subprocess.run(docker,check=True)
    print(binary)
if __name__=='__main__':main()
