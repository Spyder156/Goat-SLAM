#!/usr/bin/env python3
"""Independent synthetic metric rig/IMU regression, including arbitrary gravity."""
import json
import argparse
from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
import pycolmap

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/colmap_lamaria_ba/end_to_end_contracts_v2'
def values(a):return ' '.join(f'{float(x):.17g}' for x in np.asarray(a).ravel())
def pose(R,t):return values(np.r_[Rotation.from_matrix(R).as_quat()[[3,0,1,2]],t])
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,default=OUT)
    parser.add_argument('--linear-solver',choices=('sparse_schur','iterative_schur'),default='sparse_schur')
    parser.add_argument('--explicit-schur',action='store_true')
    parser.add_argument('--release-observation-metadata',action='store_true')
    args=parser.parse_args();OUT=args.out.resolve()
    OUT.mkdir(parents=True,exist_ok=True);model=OUT/'input';model.mkdir(exist_ok=True)
    Rbc=Rotation.from_rotvec([.2,-.3,.1]).as_matrix();tbc=np.array([.03,-.07,.09])
    Rc1c0=Rotation.from_rotvec([0,.3,0]).as_matrix();tc1c0=np.array([-.137,0,0])
    Rc0c1=Rc1c0.T;tc0c1=-Rc0c1@tc1c0
    Tbc=np.eye(4);Tbc[:3,:3]=Rbc;Tbc[:3,3]=tbc
    T01=np.eye(4);T01[:3,:3]=Rc0c1;T01[:3,3]=tc0c1
    settings=OUT/'settings.yaml'
    settings.write_text('%YAML:1.0\n'+''.join(f'{key}: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n   data: [{", ".join(map(str,T.ravel()))}]\n' for key,T in [('IMU.T_b_c1',Tbc),('Rig.T_c0_c1',T01)])+'IMU.NoiseGyro: 0.0005\nIMU.NoiseAcc: 0.0024\nIMU.GyroWalk: 0.00024\nIMU.AccWalk: 0.0064\n')
    pars=np.array([242,242,319.3,241.,-.0277657,.104662,-.075221,.0142486,.000849837,-.000390873,.000328355,.000291279,-.00114684,-.000284116,-.000549598,-.000060646])
    camera=pycolmap.Camera(model='RAD_TAN_THIN_PRISM_FISHEYE',width=640,height=480,params=pars)
    (model/'cameras.txt').write_text(''.join(f'{i} RAD_TAN_THIN_PRISM_FISHEYE 640 480 {values(pars)}\n' for i in [1,2]))
    (model/'rigs.txt').write_text('1 2 CAMERA 1 CAMERA 2 1 '+pose(Rc1c0,tc1c0)+'\n')
    w=np.array([.02,.06,-.03]);g=np.array([1,2,-np.sqrt(9.81**2-5)])
    def body(t):
        R=Rbc@Rotation.from_rotvec(w*t).as_matrix()
        p=np.array([.7*t+.18*(1-np.cos(1.7*t)),.15*np.sin(1.2*t),.08*(1-np.cos(2.1*t))])
        a=np.array([.18*1.7**2*np.cos(1.7*t),-.15*1.2**2*np.sin(1.2*t),.08*2.1**2*np.cos(2.1*t)])
        return R,p,a
    ns=1000000000+np.arange(21,dtype=np.int64)*150000000
    rng=np.random.default_rng(42)
    points=np.c_[rng.uniform(-2,4,80),rng.uniform(-2,2,80),rng.uniform(6,12,80)]
    Cfirst=Rbc@tbc;scale=1.12;scaled_points=Cfirst+scale*(points-Cfirst)
    frames=[];images=[];truth=[];tracks=[[] for _ in points]
    for i,t_ns in enumerate(ns):
        t=(t_ns-1000000000)*1e-9;Rwb,pwb,a=body(t);truth.append(pwb)
        Rwc=Rwb@Rbc;C=pwb+Rwb@tbc
        Rcw=Rwc.T;tcw=-Rcw@C;Cscaled=Cfirst+scale*(C-Cfirst);tscaled=-Rcw@Cscaled
        frames.append(f'{i+1} 1 '+pose(Rcw,tscaled)+f' 2 CAMERA 1 {2*i+1} CAMERA 2 {2*i+2}\n')
        for cam in [0,1]:
            Rcr=np.eye(3) if cam==0 else Rc1c0;tcr=np.zeros(3) if cam==0 else tc1c0
            R=Rcr@Rcw;timage=Rcr@tcw+tcr;tinput=Rcr@tscaled+tcr
            xy=camera.img_from_cam(points@R.T+timage)
            image_id=2*i+cam+1
            images.append(f'{image_id} '+pose(R,tinput)+f' {cam+1} cam{cam}/{t_ns}.png\n')
            images.append(' '.join(f'{x:.17g} {y:.17g} {j+1}' for j,(x,y) in enumerate(xy))+'\n')
            for j in range(len(points)):tracks[j].append((image_id,j))
    (model/'frames.txt').write_text(''.join(frames));(model/'images.txt').write_text(''.join(images))
    (model/'points3D.txt').write_text(''.join(f'{j+1} '+values(X)+' 100 150 200 1 '+ ' '.join(f'{i} {k}' for i,k in tracks[j])+'\n' for j,X in enumerate(scaled_points)))
    csv=OUT/'imu.csv'
    with csv.open('w') as f:
        f.write('#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n')
        for t_ns in range(int(ns[0])-1000000,int(ns[-1])+1000001,1000000):
            R,p,a=body((t_ns-1000000000)*1e-9);f.write(str(t_ns)+','+','.join(map(str,np.r_[w,R.T@(a-g)]))+'\n')
    results={}
    for mode in ['visual_rig','vi_fixed','vi_calib']:
        target=OUT/mode
        cmd=['bash',str(ROOT/'pipeline/vi_ba_lamaria/run.sh'),'--input_path',str(model),'--output_path',str(target),'--imu_csv',str(csv),'--settings_yaml',str(settings),'--mode',mode,'--max_iterations','60','--max_threads','2','--linear_solver',args.linear_solver]
        if args.explicit_schur:cmd += ['--explicit_schur','1']
        if args.release_observation_metadata:cmd += ['--release_observation_metadata','1']
        with (OUT/f'{mode}.log').open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
        result=json.loads((target/'result.json').read_text())
        final=pycolmap.Reconstruction(str(target));errors=[]
        for i,t_ns in enumerate(ns):
            im=next(x for x in final.images.values() if x.name==f'cam0/{t_ns}.png')
            Tcw=im.cam_from_world().matrix();Rcw=Tcw[:3,:3];tcw=Tcw[:3,3]
            Rwb=(Rbc@Rcw).T;pwb=-Rwb@(Rbc@tcw+tbc);errors.append(np.linalg.norm(pwb-truth[i]))
        result['body_rmse']=float(np.sqrt(np.mean(np.square(errors))))
        assert result['frames']==len(ns) and result['images']==2*len(ns)
        assert result['body_rmse']<.015,(mode,result)
        assert result['final_visual_cost']<result['initial_visual_cost']*.02,(mode,result)
        if mode!='visual_rig':assert result['imu_intervals']==len(ns)-1
        if mode=='vi_calib':
            hist=np.genfromtxt(target/'optimization.csv',delimiter=',',names=True)
            assert hist['max_intrinsic_step_sigma'].max()<=.150000001
        results[mode]=result
        print(mode,'PASS RMSE',result['body_rmse'],flush=True)
    (OUT/'results.json').write_text(json.dumps({'passed':True,'results':results},indent=2)+'\n')
if __name__=='__main__':main()
