#!/usr/bin/env python3
"""Hash-validated private build of Tracking.cc against frozen native v2 objects."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=Path(__file__).resolve().parent

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--base',type=Path,default=ROOT/'build/orbslam3_lamaria_online_full_v2');ap.add_argument('--out',type=Path,default=ROOT/'build/orbslam3_lamaria_tracking_memory_20261003');ap.add_argument('--jobs',type=int,default=1)
    a=ap.parse_args();base=a.base.resolve(strict=True);out=a.out.resolve();vendor=ROOT/'third_party/ORB_SLAM3'
    if a.jobs!=1: raise SystemExit('This builder deliberately uses one compiler job')
    if out==base or base in out.parents or out in base.parents: raise SystemExit('Private output required')
    manifest=json.loads((base/'build_manifest.json').read_text())
    for table in ('patched_sources_sha256','headers_sha256'):
        for rel,wanted in manifest[table].items():
            if sha(base/'sources'/rel)!=wanted: raise SystemExit('Changed frozen source: '+rel)
    for filename,key in [('lib/libORB_SLAM3.so','library_sha256'),('bin/stereo_lamaria_euroc','runner_sha256')]:
        if sha(base/filename)!=manifest[key]:raise SystemExit('Changed frozen product: '+filename)
    image=manifest['image'];image_id=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()
    if image_id!=manifest['image_id']:raise SystemExit('Frozen compiler image changed')
    for name in ('sources','objects','lib','bin','logs'): (out/name).mkdir(parents=True,exist_ok=True)
    (out/'build_manifest.json').unlink(missing_ok=True)
    source=out/'sources';shutil.rmtree(source);shutil.copytree(base/'sources',source)
    subprocess.run(['patch','--batch','--forward','--fuzz=0','-p1','-d',str(source),'-i',str(PACKAGE/'delta.patch')],check=True)
    shutil.copy2(PACKAGE/'new_files/src/LamariaTrackingMemory.h',source/'src/LamariaTrackingMemory.h')
    changed='src/Tracking.cc';object_hashes={};replacements={}
    for rel in manifest['compile_sources']:
        filename=rel.replace('/','_')+'.o';obj=base/'objects'/filename;record=json.loads((base/'objects'/(filename+'.json')).read_text())
        if sha(obj)!=record['sha256']: raise SystemExit('Changed frozen object: '+filename)
        object_hashes[rel]=record['sha256']
        if rel!=changed:shutil.copy2(obj,out/'objects'/filename)
        replacements['CMakeFiles/ORB_SLAM3.dir/'+rel+'.o']='/work/objects/'+filename
    flags=['-I/work/sources','-I/work/sources/include','-I/work/sources/include/CameraModels']
    text=(vendor/'build/CMakeFiles/ORB_SLAM3.dir/flags.make').read_text()
    for key in ('CXX_FLAGS','CXX_DEFINES','CXX_INCLUDES'): flags+=shlex.split(re.search(r'^'+key+r'\s*=\s*(.*)$',text,re.M).group(1))
    mounts=['docker','run','--rm','--network','none','--cpus','1','--user',f'{os.getuid()}:{os.getgid()}',
            '-v',str(vendor)+':/orb:ro','-v',str(ROOT/'third_party/ELSED')+':/elsed:ro','-v',str(out)+':/work',
            '--entrypoint','/usr/bin/c++',image]
    spec=importlib.util.spec_from_file_location('build_lamaria',ROOT/'pipeline/run/build_lamaria.py');builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
    command=mounts+flags+['-c','/work/sources/'+changed,'-o',replacements['CMakeFiles/ORB_SLAM3.dir/'+changed+'.o']]
    print('Compiling only '+changed,flush=True)
    with (out/'logs/incremental_compile.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    link=shlex.split((vendor/'build/CMakeFiles/ORB_SLAM3.dir/link.txt').read_text())
    link_command=mounts+builder.rewrite_link(link,replacements,'/work/lib/libORB_SLAM3.so')
    with (out/'logs/incremental_link.log').open('w') as log:subprocess.run(link_command,stdout=log,stderr=subprocess.STDOUT,check=True)
    shutil.copy2(base/'bin/stereo_lamaria_euroc',out/'bin/stereo_lamaria_euroc')
    result=dict(manifest)
    result.update({'output':str(out),'patch_dir':str(PACKAGE),'incremental_base':str(base),'base_build_manifest_sha256':sha(base/'build_manifest.json'),
        'incremental_recompiled_sources':[changed],'reused_objects_sha256':{k:v for k,v in object_hashes.items() if k!=changed},
        'patch_sha256':sha(PACKAGE/'geometry.patch'),'delta_patch_sha256':sha(PACKAGE/'delta.patch'),
        'private_helper_sha256':sha(source/'src/LamariaTrackingMemory.h'),'compile_command':command,'link_command':link_command,
        'library_sha256':sha(out/'lib/libORB_SLAM3.so'),'runner_sha256':sha(out/'bin/stereo_lamaria_euroc'),
        'patched_sources_sha256':{rel:sha(source/rel) for rel in manifest['patched_sources_sha256']},
        'headers_sha256':{str(p.relative_to(source)):sha(p) for p in (source/'include').rglob('*') if p.is_file()}})
    (out/'build_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Built '+str(out),flush=True)
if __name__=='__main__':main()
