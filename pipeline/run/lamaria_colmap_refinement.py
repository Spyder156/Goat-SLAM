#!/usr/bin/env python3
"""Build native two-camera SIFT tracks for full LaMAria batch refinement.

Estimator inputs are raw SLAM poses, native images, physical calibration and
calibrated IMU. Ground truth is never read here. All original exported pose
timestamps are retained. COLMAP coordinates use a +0.5 pixel centre offset;
the native SDK/SLAM convention is restored when exporting image observations.
"""
import argparse
import csv
import hashlib
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path, relocated_record
import sqlite3
import subprocess
import time

import cv2
import numpy as np
import pycolmap
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = ROOT/'experiments/lamaria_radial_fixed_short_full_20261003/runs/lamaria_radial_fixed_short_full_20261003_short_full'
DEFAULT_IMAGES = Path('/media/raghav/HardDrive1/Mecka/lamaria/bench_basalt/orbslam/current_pipeline_20261002/sequence_1_19/euroc/mav0')
DEFAULT_WORK = ROOT/'experiments/lamaria_global_refinement_20261003/sift'
MODEL = 'RAD_TAN_THIN_PRISM_FISHEYE'


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data, indent=2)+'\n')
    temp.replace(path)


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def calibration(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    names = ['fx','fy','cx','cy']+[f'k{i}' for i in range(1,7)]+['p1','p2']+[f's{i}' for i in range(1,5)]
    cameras = []
    for n in (1,2):
        cameras.append(dict(params=[fs.getNode(f'Camera{n}.{k}').real() for k in names],
                            width=int(fs.getNode('Camera.width').real()),
                            height=int(fs.getNode('Camera.height').real()),
                            radius=fs.getNode(f'Camera{n}.validRadius').real(),
                            angle=fs.getNode(f'Camera{n}.maxSolidAngle').real()))
    transforms = [fs.getNode(k).mat().astype(float) for k in ('IMU.T_b_c1','Rig.T_c0_c1')]
    fs.release()
    for T in transforms:
        u,_,vt=np.linalg.svd(T[:3,:3]); T[:3,:3]=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    return transforms[0],transforms[1],cameras


def source_poses(source):
    paths=list(source.glob('atlas_*_trajectory.csv'))
    if len(paths)!=1:
        raise ValueError('Expected one raw atlas trajectory')
    with paths[0].open() as f:
        rows=list(csv.DictReader(f))
    if len({r['coordinate_frame'] for r in rows})!=1:
        raise ValueError('Independent maps cannot be silently joined')
    stamps=np.array([round(float(r['t_s'])*1e9) for r in rows],dtype=np.int64)
    if len(np.unique(stamps))!=len(stamps) or np.any(np.diff(stamps)<=0):
        raise ValueError('Duplicate or unordered source poses')
    xyz=np.array([[float(r[k]) for k in ('tx','ty','tz')] for r in rows])
    rot=Rotation.from_quat([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows]).as_matrix()
    return paths[0],stamps,xyz,rot


def sdk_camera(c, label):
    from projectaria_tools.core import calibration as aria
    from projectaria_tools.core.sophus import SE3
    p=np.asarray(c['params'])
    if abs(p[0]-p[1])>1e-8:
        raise ValueError('Native Aria shared-axis focal contract violated')
    return aria.CameraCalibration(label,aria.FISHEYE624,np.r_[p[0],p[2:]],SE3(),c['width'],c['height'],c['radius'],c['angle'],'')


def projection_contract(cameras, work):
    report=[]
    rng=np.random.default_rng(20261003)
    for cam,c in enumerate(cameras):
        sdk=sdk_camera(c,f'cam{cam}')
        p=np.array(c['params']); p[2:4]+=0.5
        cc=pycolmap.Camera(model=MODEL,width=c['width'],height=c['height'],params=p)
        th=rng.uniform(0,1.39,3000); phi=rng.uniform(-np.pi,np.pi,3000)
        rays=np.c_[np.sin(th)*np.cos(phi),np.sin(th)*np.sin(phi),np.cos(th)]
        errors=[]; inverse=[]
        for ray in rays:
            uv=sdk.project(ray)
            if uv is None:
                continue
            q=cc.img_from_cam(ray)-0.5
            errors.append(np.linalg.norm(q-uv))
            r=cc.cam_from_img(np.asarray(uv)+0.5)
            b=np.r_[r,1.]; b/=np.linalg.norm(b)
            inverse.append(np.linalg.norm(b-ray))
        if not errors or max(errors)>1e-7 or max(inverse)>1e-6:
            raise AssertionError((cam,max(errors),max(inverse)))
        mask=np.zeros((c['height'],c['width']),dtype=np.uint8)
        for y in range(c['height']):
            for x in range(c['width']):
                bearing=sdk.unproject(np.array([x,y],float))
                if bearing is None:
                    continue
                uv=sdk.project(bearing)
                if uv is not None and np.linalg.norm(uv-[x,y])<0.01:
                    mask[y,x]=255
        cv2.imwrite(str(work/f'cam{cam}_valid_mask.png'),mask)
        report.append(dict(camera=cam,valid_ray_tests=len(errors),max_projection_error_px=max(errors),
                           max_inverse_ray_error=max(inverse),valid_mask_pixels=int((mask>0).sum()),
                           colmap_principal_point_offset_px=0.5))
    save(work/'projection_contract.json',{'passed':True,'cameras':report,'SDK_vs_COLMAP':True})


def command(work, label, cmd):
    marker=work/f'{label}.result.json'
    if marker.exists():
        previous=json.loads(marker.read_text())
        if previous['returncode']==0 and relocated_record(previous['command'])==relocated_record(list(map(str,cmd))):
            print(f'{label}: previous successful command retained',flush=True)
            return
    print(f'{label}: started',flush=True)
    start=time.time()
    with (work/f'{label}.log').open('a') as log:
        log.write('\n'+json.dumps(list(map(str,cmd)))+'\n'); log.flush()
        rc=subprocess.call(list(map(str,cmd)),stdout=log,stderr=subprocess.STDOUT,
                           env={**os.environ,'QT_QPA_PLATFORM':'offscreen'})
    save(marker,{'command':list(map(str,cmd)),'returncode':rc,'elapsed_s':time.time()-start})
    if rc:
        raise RuntimeError(f'{label} failed with exit {rc}; see {work}/{label}.log')
    print(f'{label}: finished in {time.time()-start:.1f}s',flush=True)


def prepare(args):
    work=args.work.resolve(); work.mkdir(parents=True,exist_ok=True)
    config=args.source/'config/settings.yaml'
    Tbc,T01,cameras=calibration(config)
    raw,stamps,xyz,rot=source_poses(args.source)
    if not (work/'projection_contract.json').exists():
        projection_contract(cameras,work)
    stage=work/'images'; stage.mkdir(exist_ok=True)
    for cam in (0,1):
        folder=stage/f'cam{cam}'; folder.mkdir(exist_ok=True)
        names=[]
        for stamp in stamps:
            name=f'{stamp}.png'; src=args.images/f'cam{cam}'/'data'/name
            if not src.is_file():
                raise FileNotFoundError(src)
            dst=folder/name
            if not dst.exists(): dst.symlink_to(src.resolve())
            names.append(f'cam{cam}/{name}')
        (work/f'cam{cam}_images.txt').write_text('\n'.join(names)+'\n')
    db_path=work/'database.db'
    if not db_path.exists():
        with pycolmap.Database.open(str(db_path)) as db:
            for cam,c in enumerate(cameras):
                p=np.array(c['params']); p[2:4]+=0.5
                camera=pycolmap.Camera(camera_id=cam+1,model=MODEL,width=c['width'],height=c['height'],params=p,has_prior_focal_length=True)
                db.write_camera(camera,use_camera_id=True)
    T10=np.linalg.inv(T01); q=Rotation.from_matrix(T10[:3,:3]).as_quat()
    rig=[{'cameras':[{'image_prefix':'cam0/','ref_sensor':True},
                     {'image_prefix':'cam1/','cam_from_rig_rotation':q[[3,0,1,2]].tolist(),
                      'cam_from_rig_translation':T10[:3,3].tolist()}]}]
    save(work/'rig_config.json',rig)
    # Temporal offsets supply short continuity and longer parallax. Nearby
    # revisits are proposed from SLAM only and still require independent SIFT
    # matching/geometric verification; GT is not loaded.
    pairs=set()
    def add(a,b):
        if a!=b: pairs.add(tuple(sorted((a,b))))
    for i,stamp in enumerate(stamps):
        add(f'cam0/{stamp}.png',f'cam1/{stamp}.png')
        for d in (1,5,20,60):
            j=i+d
            if j>=len(stamps): continue
            for cam in (0,1):
                add(f'cam{cam}/{stamp}.png',f'cam{cam}/{stamps[j]}.png')
        if i%5==0 and i+5<len(stamps):
            add(f'cam0/{stamp}.png',f'cam1/{stamps[i+5]}.png')
            add(f'cam1/{stamp}.png',f'cam0/{stamps[i+5]}.png')
    tree=cKDTree(xyz)
    for i in range(0,len(stamps),20):
        candidates=tree.query_ball_point(xyz[i],8.)
        candidates=[j for j in candidates if abs(int(stamps[j])-int(stamps[i]))>30_000_000_000 and j%20==0]
        for j in sorted(candidates,key=lambda j:np.linalg.norm(xyz[j]-xyz[i]))[:4]:
            for a in (0,1):
                for b in (0,1): add(f'cam{a}/{stamps[i]}.png',f'cam{b}/{stamps[j]}.png')
    (work/'pairs.txt').write_text(''.join(f'{a} {b}\n' for a,b in sorted(pairs)))
    save(work/'manifest.json',{'source_run':str(args.source.resolve()),'raw_trajectory':str(raw.resolve()),
         'raw_trajectory_sha256':sha(raw),'settings_sha256':sha(config),'pose_timestamps':len(stamps),
         'images':2*len(stamps),'pairs':len(pairs),'camera_model':MODEL,
         'T_body_cam0':Tbc.tolist(),'T_cam0_cam1':T01.tolist(),'source_cameras':cameras,
         'colmap_pixel_offset':0.5,'ground_truth_used':False,'feature_maximum':args.features,
         'source_timestamp_start_ns':int(stamps[0]),'source_timestamp_end_ns':int(stamps[-1])})
    np.savez(work/'seed_body_poses.npz',timestamps_ns=stamps,xyz=xyz,rotation=rot)
    print(f'Prepared {len(stamps)} paired timestamps, {len(pairs)} image pairs',flush=True)


def extract(args):
    work=args.work.resolve()
    for cam in (0,1):
        command(work,f'extract_cam{cam}',[args.colmap,'feature_extractor',
            '--database_path',work/'database.db','--image_path',work/'images',
            '--image_list_path',work/f'cam{cam}_images.txt',
            '--ImageReader.existing_camera_id',str(cam+1),
            '--ImageReader.camera_mask_path',work/f'cam{cam}_valid_mask.png',
            '--FeatureExtraction.type','SIFT','--FeatureExtraction.use_gpu','1',
            '--FeatureExtraction.num_threads',str(args.threads),
            '--SiftExtraction.max_num_features',str(args.features)])
    command(work,'rig_configuration',[args.colmap,'rig_configurator','--database_path',work/'database.db',
                                      '--rig_config_path',work/'rig_config.json'])


def match(args):
    work=args.work.resolve()
    command(work,'match',[args.colmap,'matches_importer','--database_path',work/'database.db',
        '--match_list_path',work/'pairs.txt','--match_type','pairs',
        '--FeatureMatching.use_gpu','1','--FeatureMatching.num_threads',str(args.threads),
        '--FeatureMatching.skip_image_pairs_in_same_frame','0',
        '--FeatureMatching.guided_matching','0','--TwoViewGeometry.compute_relative_pose','1'])


def seed_model(args):
    work=args.work.resolve()
    Tbc,T01,_=calibration(args.source/'config/settings.yaml')
    _,stamps,xyz,rot=source_poses(args.source)
    poses={int(t):(p,R) for t,p,R in zip(stamps,xyz,rot)}
    rec=pycolmap.Reconstruction()
    # Read database with matching library version through stable SQLite columns
    # for image identity; camera/rig values remain our explicit physical input.
    con=sqlite3.connect(f'file:{work}/database.db?mode=ro',uri=True)
    cameras=con.execute('SELECT camera_id,model,width,height,params FROM cameras').fetchall()
    rows=con.execute('SELECT image_id,name,camera_id FROM images').fetchall()
    membership={iid:(fid,rid) for iid,fid,rid in con.execute(
        'SELECT d.data_id,d.frame_id,f.rig_id FROM frame_data d JOIN frames f ON f.frame_id=d.frame_id WHERE d.sensor_type=0')}
    con.close()
    for cid,mid,w,h,blob in cameras:
        c=pycolmap.Camera(camera_id=cid,model=MODEL,width=w,height=h,params=np.frombuffer(blob,dtype=np.float64).copy())
        rec.add_camera(c)
    ids={cam:next(cid for _,name,cid in rows if name.startswith(f'cam{cam}/')) for cam in (0,1)}
    if ids[0]==ids[1]: raise ValueError('Two physical cameras unexpectedly share one intrinsic block')
    rig_ids={membership[iid][1] for iid,_,_ in rows}
    if len(rig_ids)!=1:
        raise ValueError('Database rig configuration has not paired the two cameras')
    rig_id=rig_ids.pop()
    rig=pycolmap.Rig(rig_id=rig_id)
    rig.add_ref_sensor(pycolmap.sensor_t(type=pycolmap.SensorType.CAMERA,id=ids[0]))
    rig.add_sensor(pycolmap.sensor_t(type=pycolmap.SensorType.CAMERA,id=ids[1]),pycolmap.Rigid3d(np.linalg.inv(T01)[:3]))
    rec.add_rig(rig)
    by_stamp={}
    for iid,name,cid in rows:
        by_stamp.setdefault(int(Path(name).stem),[]).append((iid,name,cid))
    if set(by_stamp)!=set(map(int,stamps)):
        raise ValueError('Database does not retain all source timestamps')
    for stamp in stamps:
        p,R=poses[int(stamp)]; T=np.eye(4); T[:3,:3]=R; T[:3,3]=p
        Tcw=np.linalg.inv(T@Tbc)
        frame_ids={membership[iid][0] for iid,_,_ in by_stamp[int(stamp)]}
        if len(frame_ids)!=1:
            raise ValueError('Simultaneous cameras do not share one database frame')
        fid=frame_ids.pop()
        frame=pycolmap.Frame(frame_id=fid,rig_id=rig_id)
        for iid,name,cid in by_stamp[int(stamp)]:
            frame.add_data_id(pycolmap.data_t(sensor_id=pycolmap.sensor_t(type=pycolmap.SensorType.CAMERA,id=cid),id=iid))
        frame.rig_from_world=pycolmap.Rigid3d(Tcw[:3])
        rec.add_frame(frame)
        for iid,name,cid in by_stamp[int(stamp)]:
            im=pycolmap.Image(name=name,camera_id=cid,image_id=iid); im.frame_id=fid
            rec.add_image(im)
    out=work/'seed_model'; out.mkdir(exist_ok=True); rec.write_text(str(out))
    save(work/'seed_model_contract.json',{'frames':rec.num_reg_frames(),'images':rec.num_reg_images(),
        'rigs':rec.num_rigs(),'camera0_reference':ids[0],'camera1':ids[1],
        'physical_baseline_m':float(np.linalg.norm(T01[:3,3])),'no_scale_alignment':True})


def triangulate(args):
    work=args.work.resolve(); seed_model(args)
    out=work/'triangulated'; out.mkdir(exist_ok=True)
    command(work,'triangulate',[args.colmap,'point_triangulator','--database_path',work/'database.db',
        '--image_path',work/'images','--input_path',work/'seed_model','--output_path',out,
        '--Mapper.ba_refine_focal_length','0','--Mapper.ba_refine_principal_point','0',
        '--Mapper.ba_refine_extra_params','0','--Mapper.ba_refine_sensor_from_rig','0',
        '--Mapper.fix_existing_frames','1','--Mapper.num_threads',str(args.threads),
        '--Mapper.ba_global_max_num_iterations','5','--Mapper.extract_colors','0'])
    command(work,'triangulated_text',[args.colmap,'model_converter','--input_path',out,
                                      '--output_path',out,'--output_type','TXT'])
    rec=pycolmap.Reconstruction(); rec.read_text(str(out))
    expected=json.loads((work/'manifest.json').read_text())
    if rec.num_reg_frames()!=expected['pose_timestamps'] or rec.num_points3D()<200:
        raise ValueError(f'Incomplete/empty triangulated graph: {rec.summary()}')
    save(work/'graph_summary.json',{'frames':rec.num_reg_frames(),'images':rec.num_reg_images(),
        'landmarks':rec.num_points3D(),'observations':rec.compute_num_observations(),
        'mean_reprojection_px':rec.compute_mean_reprojection_error(),
        'mean_track_length':rec.compute_mean_track_length(),'GT_used':False})
    print(rec.summary(),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['prepare','extract','match','triangulate','all'])
    p.add_argument('--source',type=relocated_path,default=DEFAULT_SOURCE)
    p.add_argument('--images',type=relocated_path,default=DEFAULT_IMAGES)
    p.add_argument('--work',type=relocated_path,default=DEFAULT_WORK)
    p.add_argument('--colmap',default='/usr/local/bin/colmap')
    p.add_argument('--features',type=int,default=1024)
    p.add_argument('--threads',type=int,default=6)
    args=p.parse_args()
    for name,func in [('prepare',prepare),('extract',extract),('match',match),('triangulate',triangulate)]:
        if args.stage in (name,'all'): func(args)


if __name__=='__main__': main()
