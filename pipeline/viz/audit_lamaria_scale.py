#!/usr/bin/env python3
"""Offline metric-scale diagnosis. Never edits or feeds results to SLAM.

Compare SE3 and one global Sim3 on identical raw cam0/GT associations. Local
window fits diagnose scale variation; they do not create a corrected trajectory.
"""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

BASE = Path(__file__).resolve().parents[2] / 'experiments'
ASSETS = Path('/media/raghav/HardDrive1/Mecka/lamaria')
CASES = {
    'short_baseline': ('sequence_1_19', BASE/'lamaria_history_short_1_19_20261002'),
    'short_replay': ('sequence_1_19', BASE/'lamaria_reliability_replay_20261002/runs/lamaria_reliability_replay_20261002_short_loss_prefix'),
    'medium_baseline': ('sequence_2_11', BASE/'lamaria_history_medium_2_11_20261002'),
    'medium_replay': ('sequence_2_11', BASE/'lamaria_reliability_replay_20261002/runs/lamaria_reliability_replay_20261002_medium_loss_prefix'),
    'long_baseline': ('sequence_3_17', BASE/'lamaria_history_full_20261002'),
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fit(x, y, scale=False):
    """Least-squares proper rotation, optional positive scalar, translation."""
    xc, yc = x - x.mean(axis=0), y - y.mean(axis=0)
    u, d, vt = np.linalg.svd(yc.T @ xc)
    correction = np.diag([1., 1., np.linalg.det(u @ vt)])
    r = u @ correction @ vt
    s = float(np.sum(d * np.diag(correction)) / np.sum(xc*xc)) if scale else 1.
    t = y.mean(axis=0) - s*r@x.mean(axis=0)
    aligned = s*x@r.T + t
    error = np.linalg.norm(aligned-y, axis=1)
    return {'scale_estimate_to_gt': s, 'estimate_stretch': 1/s,
            'rmse_m': float(np.sqrt(np.mean(error**2))),
            'median_m': float(np.median(error)), 'p90_m': float(np.quantile(error,.9)),
            'max_m': float(error.max()), 'rotation': r.tolist(), 'translation': t.tolist()}, aligned


def nearest(source, targets):
    ids = np.clip(np.searchsorted(source, targets), 0, len(source)-1)
    prev = np.maximum(ids-1, 0)
    ids = np.where(abs(source[prev]-targets) <= abs(source[ids]-targets), prev, ids)
    return ids, abs(source[ids]-targets)


def read_rows(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def load_case(sequence, run):
    raw = next(run.glob('atlas_*_trajectory.csv'))
    online = next(run.glob('online_*.csv'))
    config = run/'config/settings.yaml'
    gtfile, calibfile = ASSETS/sequence/'gt_dense.txt', ASSETS/sequence/'aria_calib.json'
    fs = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
    tbc = fs.getNode('IMU.T_b_c1').mat().astype(float)
    fs.release()
    calib = json.loads(calibfile.read_text())['cam0']['T_b_s']
    official_tbc = np.eye(4)
    official_tbc[:3,:3] = Rotation.from_quat(calib['qvec']).as_matrix()
    official_tbc[:3,3] = calib['tvec']
    calibration_difference=float(np.max(abs(tbc-official_tbc)))
    assert calibration_difference < 1e-6
    # Use official double-precision lever arm; verify estimator calibration above.
    tbc = official_tbc
    rows = read_rows(raw)
    groups = {}
    for row in rows:
        groups.setdefault(row['coordinate_frame'], []).append(row)
    frame = min(groups, key=lambda k: (-len(groups[k]), k))
    rows = groups[frame]
    assert frame.endswith('_body0')
    stamps = np.rint(np.array([float(r['t_s']) for r in rows])*1e9).astype(np.int64)
    pos = np.array([[float(r[k]) for k in ('tx','ty','tz')] for r in rows])
    quats = np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows])
    assert np.max(abs(np.linalg.norm(quats,axis=1)-1)) < 1e-5
    pos += Rotation.from_quat(quats).apply(np.broadcast_to(tbc[:3,3],pos.shape).copy())
    gt = np.loadtxt(gtfile)
    gtstamps = np.rint(gt[:,0]).astype(np.int64)
    ei, delta = nearest(stamps, gtstamps)
    gi = np.flatnonzero(delta <= 1000000)
    ei = ei[gi]
    assert len(ei) == len(np.unique(ei))
    x, y, t = pos[ei], gt[gi,1:4], stamps[ei]/1e9
    orows = read_rows(online)
    ot = np.rint(np.array([float(r['input_t_s']) for r in orows])*1e9).astype(np.int64)
    oi, od = nearest(ot, stamps[ei])
    assert od.max() <= 1000
    state = np.array([int(orows[i]['state']) for i in oi])
    coast = np.array([int(orows[i]['coasting']) for i in oi])
    good = (state == 2) & (coast == 0)
    # Official score wrapper and this audit must use the same primary-map matches.
    scorecsv = run/'lamaria_score/pgt_horizontal_errors.csv'
    if scorecsv.exists():
        scored = read_rows(scorecsv)
        assert [int(r['gt_timestamp_ns']) for r in scored] == gtstamps[gi].tolist()
    info = {'run': str(run), 'sequence': sequence, 'coordinate_frame':frame,
            'atlas_maps':{k:len(v) for k,v in groups.items()},
            'primary_poses':len(rows), 'gt_associations':len(t),
            'max_timestamp_difference_ns':int(delta[gi].max()),
            'calibration_max_abs_difference':calibration_difference,
            'sensor_conversion':'world_from_cam0 = world_from_imu_right * imu_right_from_cam0; xyzw',
            'first_timestamp_s':float(t[0]), 'last_timestamp_s':float(t[-1]),
            'visual_ok_associations':int(good.sum()),
            'hashes':{str(p):sha(p) for p in (raw, online, config, gtfile, calibfile)}}
    return t,x,y,good,info


def summarize(t,x,y,mask):
    fixed, _ = fit(x[mask],y[mask],False)
    scaled, _ = fit(x[mask],y[mask],True)
    return {'poses':int(mask.sum()), 'first_s':float(t[mask][0]), 'last_s':float(t[mask][-1]),
            'se3':fixed, 'sim3':scaled,
            'rmse_reduction_percent':100*(1-scaled['rmse_m']/fixed['rmse_m'])}


def windows(t,x,y,good,width):
    rows=[]
    for start in np.arange(t[0],t[-1]-width,width/2):
        mask=good & (t>=start) & (t<start+width)
        if mask.sum()<30 or np.max(np.diff(t[mask]))>2.0:
            continue
        spread=float(np.sqrt(np.mean(np.sum((y[mask]-y[mask].mean(axis=0))**2,axis=1))))
        if spread<3:
            continue
        row=summarize(t,x,y,mask)
        row['gt_rms_spread_m']=spread
        row['gt_singular_values']=np.linalg.svd(y[mask]-y[mask].mean(axis=0),compute_uv=False).tolist()
        rows.append(row)
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    loaded={name:load_case(*case) for name,case in CASES.items()}
    report={'evaluation_only':True,'estimator_modified':False,
            'warning':'GT-derived SE3/Sim3 fits are diagnostics, not official scores or estimator outputs. Raw geometry is never overwritten. All compared fits use identical cam0 poses.',
            'window_rule':'30/60 second windows, >=30 matched visual-OK poses, no GT gap >2s, GT RMS spread >=3m; proper rotations, no reflection; no terminal coasting poses in local scale fits.',
            'cases':{}}
    comparisons={}
    for prefix in ('short','medium'):
        a,b=loaded[prefix+'_baseline'],loaded[prefix+'_replay']
        # Compare exact same GT-associated timestamps that were visually tracked in both.
        common=np.intersect1d(np.rint(a[0][a[3]]*1e9).astype(np.int64),np.rint(b[0][b[3]]*1e9).astype(np.int64))
        comparisons[prefix]=common
    table=[]
    for name,(t,x,y,good,info) in loaded.items():
        allmask=np.ones(len(t),dtype=bool)
        scopes={'all_primary':allmask, 'visual_ok':good}
        prefix=name.split('_')[0]
        if prefix in comparisons:
            scopes['common_visual_ok']=np.isin(np.rint(t*1e9).astype(np.int64),comparisons[prefix])
        info['fits']={scope:summarize(t,x,y,m) for scope,m in scopes.items()}
        info['windows_30s']=windows(t,x,y,good,30.)
        info['windows_60s']=windows(t,x,y,good,60.)
        for width in (30,60):
            s=np.array([r['sim3']['scale_estimate_to_gt'] for r in info[f'windows_{width}s']])
            info[f'window_{width}s_scale_summary']={'n':len(s),'min':float(s.min()),'p10':float(np.quantile(s,.1)),
                'median':float(np.median(s)),'p90':float(np.quantile(s,.9)),'max':float(s.max())}
        report['cases'][name]=info
        mask=scopes.get('common_visual_ok',good)
        sf,asf=fit(x[mask],y[mask],False)
        ss,ass=fit(x[mask],y[mask],True)
        fig,axes=plt.subplots(2,2,figsize=(14,11),layout='constrained')
        for axis,aligned,title in ((axes[0,0],asf,f'Fixed metric scale (SE3): RMSE {sf["rmse_m"]:.2f} m'),
                                    (axes[0,1],ass,f'Diagnostic global Sim3: scale {ss["scale_estimate_to_gt"]:.4f}, RMSE {ss["rmse_m"]:.2f} m')):
            axis.plot(y[mask,0],y[mask,1],color='#228833',lw=2,label='GT cam0')
            axis.plot(aligned[:,0],aligned[:,1],color='#ee7733',lw=1.2,label='Raw SLAM cam0 after alignment')
            axis.set_aspect('equal',adjustable='datalim');axis.set_title(title);axis.legend(fontsize=8)
            axis.set_xlabel('GT world x (m)');axis.set_ylabel('GT world y (m)');axis.grid(alpha=.2)
        axis=axes[1,0]
        for width,marker in ((30,'.'),(60,'o')):
            ws=info[f'windows_{width}s']
            axis.plot([(r['first_s']+r['last_s'])/2 for r in ws], [r['sim3']['scale_estimate_to_gt'] for r in ws],marker=marker,label=f'{width}s independent windows')
        axis.axhline(1,color='black',ls='--',lw=1)
        axis.axhline(ss['scale_estimate_to_gt'],color='#ee7733',ls=':',label='Single global scale')
        axis.set_ylabel('Scale multiplying estimate to match GT');axis.set_xlabel('Original sensor time (s)');axis.legend(fontsize=8);axis.grid(alpha=.2)
        axis=axes[1,1]
        axis.plot(t[mask],np.linalg.norm(asf-y[mask],axis=1),label='SE3, scale=1',alpha=.85)
        axis.plot(t[mask],np.linalg.norm(ass-y[mask],axis=1),label='One global Sim3',alpha=.85)
        axis.set_xlabel('Original sensor time (s)');axis.set_ylabel('3D position error (m)');axis.legend(fontsize=8);axis.grid(alpha=.2)
        fig.suptitle(name+' | same visually tracked cam0 poses; GT-derived fits for evaluation only',fontsize=13)
        fig.savefig(args.out/(name+'.png'),dpi=150);plt.close(fig)
        with (args.out/(name+'_matches.csv')).open('w') as handle:
            writer=csv.writer(handle);writer.writerow(['time_s','visual_ok','cam0_x','cam0_y','cam0_z','gt_x','gt_y','gt_z'])
            writer.writerows([float(ti),int(ok),*xx,*yy] for ti,ok,xx,yy in zip(t,good,x,y))
        table.append(f'<tr><td>{name}</td><td>{int(mask.sum())}</td><td>{sf["rmse_m"]:.3f}</td><td>{ss["scale_estimate_to_gt"]:.5f}</td><td>{ss["rmse_m"]:.3f}</td></tr>')
        print(name, json.dumps({'fits':info['fits'],'local':info['window_60s_scale_summary']}))
    report['script_sha256']=sha(__file__)
    (args.out/'scale_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (args.out/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>LaMAria metric scale audit</title><style>body{font:16px system-ui;max-width:1300px;margin:24px auto;color:#222}img{width:100%}td,th{padding:8px;border-bottom:1px solid #ddd}table{border-collapse:collapse}</style><h1>Metric scale audit</h1><p>'+html.escape(report['warning'])+'</p><p>Short and Medium comparison plots use the exact common GT timestamps that have visual-OK states in both baseline and replay. Long uses its visual-OK states. Coasting poses are excluded from these plots; full-primary fits remain in scale_audit.json. Maps remain separate.</p><table><tr><th>Run</th><th>Matched poses</th><th>SE3 RMSE m</th><th>Single scale</th><th>Sim3 RMSE m</th></tr>'+''.join(table)+'</table>'+''.join(f'<h2>{name}</h2><img src="{name}.png">' for name in loaded))


if __name__=='__main__':
    main()
