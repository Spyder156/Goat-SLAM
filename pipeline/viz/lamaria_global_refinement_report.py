#!/usr/bin/env python3
"""Compare scored global refinements without changing their raw trajectories."""
import argparse
import csv
import html
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from audit_lamaria_scale import fit


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='append',required=True,help='label=absolute run path')
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--toolkit',type=Path,default=Path(__file__).resolve().parents[3]/'third_party/lamaria_toolkit')
    a=p.parse_args(); a.out.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(a.toolkit.resolve()))
    from lamaria.utils.metrics import piecewise_linear_scoring
    cp_scoring=piecewise_linear_scoring()
    gt=np.loadtxt(a.gt); gtlookup={int(r[0]):r[1:4] for r in gt}
    summary=[]; residual_fields=[];fig,axes=plt.subplots(1,3,figsize=(19,6))
    axes[0].plot(gt[:,1],gt[:,2],c='black',lw=2,label='Ground truth')
    axes[1].plot(gt[:,1],gt[:,2],c='black',lw=2,label='Ground truth')
    for item in a.run:
        label,path=item.split('=',1);run=Path(path).resolve();folder=run/'lamaria_score'
        score=json.loads((folder/'scores.json').read_text())
        result_file=run/'result.json'
        result=json.loads(result_file.read_text()) if result_file.exists() else {}
        solver=result.get('solver_result')
        control_points=sorted(json.loads((folder/'control_point_errors.json').read_text()),key=lambda r:r['tag_id'])
        if len(control_points)!=score['CP_total']:
            raise ValueError('Control-point count differs from saved evaluation')
        for cp_row in control_points:
            error=cp_row['horizontal_error_m']
            contribution=float(cp_scoring(np.inf if error is None else error))*5/len(control_points)
            cp_row['sequence_score_contribution']=contribution
        reconstructed_score=sum(row['sequence_score_contribution'] for row in control_points)
        if abs(reconstructed_score-score['Score2D'])>1e-9:
            raise ValueError('Official control-point scoring curve does not reproduce saved Score2D')
        poses=np.loadtxt(folder/'estimated_cam0_ns.txt'); pose_lookup={int(r[0]):r[1:4] for r in poses}
        with (folder/'pgt_horizontal_errors.csv').open() as f: pairs=list(csv.DictReader(f))
        x=np.array([pose_lookup[int(r['estimate_timestamp_ns'])] for r in pairs])
        y=np.array([gtlookup[int(r['gt_timestamp_ns'])] for r in pairs])
        metric,aligned=fit(x,y,False);diagnostic,_=fit(x,y,True)
        cp=score['CP_sim3'];R=Rotation.from_quat(cp['rotation_xyzw']).as_matrix()
        evaluated=cp['scale']*x@R.T+np.array(cp['translation'])
        err=np.linalg.norm(evaluated[:,:2]-y[:,:2],axis=1)
        if np.max(abs(err-np.array([float(r['horizontal_error_m']) for r in pairs])))>1e-6:
            raise ValueError('Saved evaluation cannot be reproduced')
        axes[0].plot(aligned[:,0],aligned[:,1],lw=1,label=label)
        axes[1].plot(evaluated[:,0],evaluated[:,1],lw=1,label=f'{label} ({score["Score2D"]:.2f})')
        axes[2].plot([int(r['gt_timestamp_ns'])*1e-9 for r in pairs],err,lw=.8,label=label)
        residual_fields.append((label, y, evaluated, score['Score2D']))
        summary.append(dict(label=label,run=str(run),Score2D=score['Score2D'],
            CP_within_1m=score['CP_within_1m'],CP_total=score['CP_total'],
            GT_associated=score['GT_associated'],GT_total=score['GT_total_denominator'],
            estimated_poses=score['estimated_poses'],native_input_frames=score['native_input_frames'],
            missing_native_poses=score['missing_native_input_poses'],
            PoseRecall_at_1m_percent=score['PoseRecall@1m_percent'],
            PoseRecall_at_5m_percent=score['PoseRecall@5m_percent'],
            control_points=control_points,
            official_CP_scale=cp['scale'],official_horizontal_error=score['associated_horizontal_error_m'],
            diagnostic_metric_SE3=metric,diagnostic_dense_Sim3=diagnostic,
            batch_solver=None if solver is None else {key: solver.get(key) for key in (
                'converged','accepted_iterations','linear_capped_steps','max_linear_iterations',
                'linear_solver','explicit_schur','final_imu_exact_cost','final_imu_cost_relative_shift')}))
    axes[0].set_title('Metric geometry: rigid SE3, scale remains 1')
    axes[1].set_title('Evaluated geometry: saved CP-derived Sim3')
    axes[2].set_title('Official horizontal error; same saved CP Sim3')
    for axis in axes[:2]:axis.set_aspect('equal',adjustable='datalim');axis.set_xlabel('x (m)');axis.set_ylabel('y (m)')
    axes[2].set_xlabel('Native timestamp (s)');axes[2].set_ylabel('Horizontal error (m)')
    for axis in axes:axis.legend(fontsize=8);axis.grid(alpha=.2)
    fig.tight_layout();fig.savefig(a.out/'trajectory_comparison.png',dpi=170);plt.close(fig)
    columns=min(3,len(residual_fields));rows=(len(residual_fields)+columns-1)//columns
    fig,field_axes=plt.subplots(rows,columns,figsize=(6*columns,6*rows),squeeze=False)
    for axis,(label,truth,estimate,score_value) in zip(field_axes.flat,residual_fields):
        axis.plot(gt[:,1],gt[:,2],color='black',lw=1,label='Ground truth')
        axis.plot(estimate[:,0],estimate[:,1],color='steelblue',lw=.8,label='Evaluated trajectory')
        selected=[0];distance=0.
        for i in range(1,len(truth)):
            distance+=np.linalg.norm(truth[i,:2]-truth[i-1,:2])
            if distance>=12:
                selected.append(i);distance=0.
        selected=np.asarray(selected);delta=estimate[selected,:2]-truth[selected,:2]
        axis.quiver(truth[selected,0],truth[selected,1],10*delta[:,0],10*delta[:,1],
                    angles='xy',scale_units='xy',scale=1,color='firebrick',width=.004,
                    label='Remaining error, magnified 10x')
        axis.set_title(f'{label}: score {score_value:.2f}')
        axis.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',
                 xlim=(gt[:,1].min()-25,gt[:,1].max()+25),
                 ylim=(gt[:,2].min()-25,gt[:,2].max()+25))
        axis.grid(alpha=.2);axis.legend(fontsize=7)
    for axis in list(field_axes.flat)[len(residual_fields):]:axis.set_visible(False)
    fig.suptitle('Remaining error after the saved official CP alignment; arrows are 10x actual displacement')
    fig.tight_layout();fig.savefig(a.out/'alignment_residual_vectors.png',dpi=160);plt.close(fig)
    names=[(r['tag_id'],r['name']) for r in summary[0]['control_points']]
    if any([(r['tag_id'],r['name']) for r in item['control_points']]!=names for item in summary):
        raise ValueError('Runs do not use the same control points')
    cp_errors=np.array([[np.nan if r['horizontal_error_m'] is None else r['horizontal_error_m']
                         for r in item['control_points']] for item in summary])
    cp_contributions=np.array([[r['sequence_score_contribution'] for r in item['control_points']]
                               for item in summary])
    delta=cp_contributions-cp_contributions[0]
    fig,cp_axes=plt.subplots(3,1,figsize=(16,3.3+1.5*len(summary)),layout='constrained')
    lim=max(.01,float(np.max(np.abs(delta))))
    panels=[(cp_errors,'Horizontal control-point error (m); missing points are grey','RdYlGn_r',0,2.5),
            (cp_contributions,'Contribution to Score2D; each row sums to its reported score','viridis',0,100/len(names)),
            (delta,'Score-point gain/loss relative to the first run','RdYlGn',-lim,lim)]
    for axis,(values,title,cmap,vmin,vmax) in zip(cp_axes,panels):
        colors=plt.get_cmap(cmap).copy();colors.set_bad('#d0d0d0')
        graphic=axis.imshow(values,aspect='auto',cmap=colors,vmin=vmin,vmax=vmax)
        axis.set_yticks(range(len(summary)),[f'{r["label"]} ({r["Score2D"]:.2f})' for r in summary])
        axis.set_xticks(range(len(names)),[name for _,name in names],rotation=35,ha='right')
        axis.set_title(title)
        for row in range(len(summary)):
            for col in range(len(names)):
                value=values[row,col]
                axis.text(col,row,'missing' if not np.isfinite(value) else f'{value:.2f}',
                          ha='center',va='center',fontsize=8,
                          bbox=dict(facecolor='white',alpha=.65,edgecolor='none',pad=.5))
        fig.colorbar(graphic,ax=axis,shrink=.8)
    fig.savefig(a.out/'control_point_score_breakdown.png',dpi=160);plt.close(fig)
    report=dict(runs=summary,notes=[
        'Local official-function scores; no public leaderboard submission.',
        'GT is used only for scoring and this diagnostic report.',
        'SE3 and dense Sim3 are auxiliary fits on each run\'s scored timestamp associations; they do not modify outputs.',
        'Score2D averages the official piecewise control-point error score. Missing control points contribute zero; dense trajectory errors do not directly enter Score2D.',
        'PoseRecall uses the full GT denominator: missing GT timestamps count as failures. Inspect coverage before comparing matched-only diagnostics.',
        'A usable batch result can be iteration-limited. A negative score from such a result does not establish the best achievable global-BA accuracy.',
        'The evaluation plot and Reruns reuse the saved official control-point transform; no second fitting step.'])
    (a.out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    lines=['Full Short global refinement comparison','']
    rows=[]
    for r in summary:
        lines.append(f'{r["label"]}: Score2D={r["Score2D"]:.4f}; CP scale={r["official_CP_scale"]:.6f}; '
                     f'metric 3D RMSE={r["diagnostic_metric_SE3"]["rmse_m"]:.4f} m; '
                     f'CP within 1m={r["CP_within_1m"]}/{r["CP_total"]}; '
                     f'GT={r["GT_associated"]}/{r["GT_total"]}; poses={r["estimated_poses"]}/{r["native_input_frames"]}')
        solver_status='Not a batch run' if r['batch_solver'] is None else (
            'Yes' if r['batch_solver']['converged'] else 'No; iteration-limited')
        if r['batch_solver'] is not None:
            lines.append(f'  Batch solver: {solver_status}; accepted updates={r["batch_solver"]["accepted_iterations"]}; '
                         f'linear-capped attempts={r["batch_solver"]["linear_capped_steps"]}')
        rows.append('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in
            [r['label'],f'{r["Score2D"]:.2f}',f'{r["official_CP_scale"]:.6f}',
             f'{r["diagnostic_metric_SE3"]["rmse_m"]:.3f}',f'{r["CP_within_1m"]}/{r["CP_total"]}',
             f'{r["GT_associated"]}/{r["GT_total"]}',solver_status])+'</tr>')
    (a.out/'SUMMARY.txt').write_text('\n'.join(lines)+'\n\n'+'\n'.join(report['notes'])+'\n')
    (a.out/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Full Short refinement</title>'
        '<style>body{font:16px system-ui;margin:32px;color:#222}td,th{padding:9px;border-bottom:1px solid #bbb}img{max-width:100%}</style>'
        '<h1>Full Short refinement</h1><p>Raw metric geometry and evaluation-aligned geometry are shown separately.</p>'
        '<table><tr><th>Run</th><th>Score2D</th><th>CP scale</th><th>Metric 3D RMSE (m)</th>'
        '<th>CP within 1 m</th><th>Matched GT</th><th>Batch solver converged?</th></tr>'+
        ''.join(rows)+'</table><img src="trajectory_comparison.png">'
        '<p>Red arrows below magnify remaining horizontal error by 10x; the estimated trajectory itself is unmodified.</p>'
        '<img src="alignment_residual_vectors.png">'
        '<p>Score2D comes from the control points below, using the official scoring curve. '
        'A perfect result places every control point within 5 cm and recovers every required pose.</p>'
        '<img src="control_point_score_breakdown.png"><ul>'+''.join('<li>'+html.escape(x)+'</li>' for x in report['notes'])+'</ul>')
    print('\n'.join(lines))


if __name__=='__main__':main()
