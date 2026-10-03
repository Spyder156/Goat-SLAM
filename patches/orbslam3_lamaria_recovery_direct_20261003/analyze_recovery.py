#!/usr/bin/env python3
"""Read-only recovery funnel and split analysis; outputs a new diagnostics folder."""
import argparse
import csv
import json
from pathlib import Path
import re


def describe(values):
    if not values:
        return None
    values=sorted(values)
    def quantile(fraction):
        index=(len(values)-1)*fraction
        lo=int(index);hi=min(lo+1,len(values)-1)
        return values[lo]+(values[hi]-values[lo])*(index-lo)
    return {'count':len(values),'minimum':values[0],'median':quantile(.5),
            'p90':quantile(.9),'maximum':values[-1]}


def plot_handoff(summary,out,label):
    """Plot saved native diagnostics; displacement is disagreement, not GT error."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    events=[row for row in summary['transaction_events'] if row.get('local_confirmation')]
    figure,axes=plt.subplots(3,1,figsize=(15,10),sharex=True,layout='constrained')
    x=np.arange(len(events));width=.19
    for camera,colors in ((0,('#9ecae1','#2171b5')),(1,('#fdd0a2','#d94801'))):
        for side,key in enumerate(('before_opt','after_opt')):
            counts=[event['local_confirmation']['cams'][camera][key] for event in events]
            axes[0].bar(x+(camera*2+side-1.5)*width,counts,width=width,color=colors[side],
                        label=f'Cam{camera}: '+('input associations' if side==0 else 'retained after VI'))
    axes[0].set_ylabel('Observations entering / leaving local VI')
    axes[0].legend(ncol=2,fontsize=9)
    colors=['#238b45' if row['committed'] else '#cb181d' for row in events]
    axes[1].bar(x,[row['local_confirmation']['opt_camera_displacement_m'] for row in events],color=colors)
    axes[1].set_ylabel('VI pose displacement from\nvisual hypothesis [m]')
    axes[1].set_title('Displacement measures disagreement between estimates; it is not ground-truth error.',fontsize=11)
    axes[2].bar(x,[row['local_confirmation']['inliers'] for row in events],color=colors)
    axes[2].axhline(10,color='black',ls=':',label='RECENTLY_LOST accepts >10 (unchanged default)')
    axes[2].axhline(50,color='gray',ls='--',label='Earlier visual recovery required >=50')
    axes[2].set_ylabel('Final local-map inliers');axes[2].legend(fontsize=9)
    labels=[f"{row['t']:.3f}\n"+('COMMIT' if row['committed'] else 'ROLLBACK') for row in events]
    axes[2].set_xticks(x,labels,rotation=45,ha='right',fontsize=9)
    axes[2].set_xlabel('Native sequence time [s] / transaction outcome')
    for axis in axes:axis.grid(axis='y',alpha=.2)
    figure.suptitle(label+': visual recovery → local visual-inertial handoff\n'
                   'Green commits the optimized state; red restores the pre-relatch IMU prediction and preserves grace',fontsize=13)
    if not events:axes[1].text(.5,.5,'No successful visual relatch transactions',transform=axes[1].transAxes,ha='center')
    figure.savefig(out/'relatch_handoff.png',dpi=160);plt.close(figure)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--label',help='Figure label; otherwise inferred from the run directory')
    args=parser.parse_args();run=args.run.resolve(strict=True);out=args.out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    recovery=[];transactions=[];local=[];relatches=[]
    with (run/'run.log').open(errors='replace') as handle:
        for number,line in enumerate(handle,1):
            marker=next((tag for tag in ('RECOVERY_DIAG','RELATCH_TRANSACTION','LM_DIAG','RELATCH_DIAG') if '['+tag+']' in line),None)
            if marker is None:continue
            if marker in ('LM_DIAG','RELATCH_DIAG'):
                try:row=json.loads(line[line.index('{'):])
                except (ValueError,json.JSONDecodeError):continue
                row['line']=number;(local if marker=='LM_DIAG' else relatches).append(row);continue
            row={'line':number}
            for key,value in re.findall(r'(\w+)=([^\s]+)',line):
                try:row[key]=float(value) if any(c in value for c in '.eE') else int(value)
                except ValueError:row[key]=value
            (recovery if marker=='RECOVERY_DIAG' else transactions).append(row)
    local_by_time={round(row['time'],6):row for row in local}
    visual_by_time={round(row['time'],6):row for row in relatches if row.get('success')}
    for row in transactions:
        confirmation=local_by_time.get(round(row.get('t',-1),6))
        if confirmation is not None:
            row['local_confirmation']={key:confirmation.get(key) for key in
                ('line','frame','time','map','state','success','inliers','cams','optimizer',
                 'prior_exists','opt_camera_displacement_m','opt_velocity_delta_mps')}
        visual=visual_by_time.get(round(row.get('t',-1),6))
        if visual is not None:row['visual_recovery']=visual
    with next(run.glob('online_*.csv')).open() as handle:online=list(csv.DictReader(handle))
    epochs=[]
    for row in online:
        key=(row['map_id'],row['map_init_kf_id'])
        if not epochs or tuple(epochs[-1]['key'])!=key:
            epochs.append({'key':list(key),'first_t':float(row['input_t_s']),'last_t':float(row['input_t_s']),
                           'rows':0,'tracking_ok':0})
        epochs[-1]['last_t']=float(row['input_t_s']);epochs[-1]['rows']+=1
        epochs[-1]['tracking_ok']+=int(row['state'])==2
    splits=[]
    for previous,current in zip(epochs,epochs[1:]):
        begin,end=previous['last_t']-8.,current['first_t']
        attempts=[r for r in recovery if begin<=r.get('t',-1)<=end]
        txns=[r for r in transactions if begin<=r.get('t',-1)<=end]
        failed=[row for row in local if begin<=row['time']<=end and not row.get('success')]
        splits.append({'previous_epoch':previous,'next_epoch':current,
                       'first_failed_local_update_in_last_eight_seconds':failed[0] if failed else None,
                       'last_eight_seconds_recovery_calls':len(attempts),
                       'hypotheses':sum(r.get('hypotheses',0) for r in attempts),
                       'geometry_accepted':sum(r.get('success',0) for r in attempts),
                       'local_confirmation_committed':sum(r.get('committed',0) for r in txns),
                       'local_confirmation_rolled_back':sum(not r.get('committed',0) for r in txns)})
    accepted=[r for r in recovery if r.get('success')]
    summary={
        'run':str(run),'recovery_calls':len(recovery),
        'tested_candidates':sum(r.get('tested_candidates',0) for r in recovery),
        'below_total15_gate_candidates':sum(r.get('below_total_gate',0) for r in recovery),
        'hypotheses':sum(r.get('hypotheses',0) for r in recovery),
        'solved_cam0':sum(r.get('solved_cam0',0) for r in recovery),
        'solved_cam1':sum(r.get('solved_cam1',0) for r in recovery),
        'geometry_accepted_calls':len(accepted),
        'confirmed_relatch_transactions':sum(r.get('committed',0) for r in transactions),
        'rolled_back_relatch_transactions':sum(not r.get('committed',0) for r in transactions),
        'recovery_milliseconds':describe([r['milliseconds'] for r in recovery if 'milliseconds' in r]),
        'matching':{f'{kind}_cam{camera}':describe([r[f'{kind}_cam{camera}'] for r in recovery if f'{kind}_cam{camera}' in r])
                    for kind in ('bow','direct') for camera in (0,1)},
        'calls_with_direct_improvement_either_lens':sum(any(r.get(f'direct_cam{c}',0)>r.get(f'bow_cam{c}',0) for c in (0,1)) for r in recovery),
        'geometry_accepted_events':accepted,'transaction_events':transactions,
        'online_epochs':epochs,'split_events':splits,
        'interpretation':'Per-camera best counts are maxima over candidates, not a single joint match set. Do not sum them to infer the15-total gate. The total gate count is logged separately. Geometry acceptance is provisional until local confirmation. solved_cam counts successful solver returns; repeated iterations can count the same hypothesis more than once.'}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if recovery:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        offset=float(online[0]['input_t_s']);times=np.array([r['t']-offset for r in recovery])
        figure,axes=plt.subplots(3,1,figsize=(14,9),sharex=True,layout='constrained')
        for camera,color in ((0,'tab:blue'),(1,'tab:orange')):
            axes[0].scatter(times,[r.get(f'bow_cam{camera}',0) for r in recovery],s=7,marker='x',alpha=.6,color=color,label=f'Cam{camera} raw BoW')
            axes[0].scatter(times,[r.get(f'direct_cam{camera}',0) for r in recovery],s=7,alpha=.6,color=color,label=f'Cam{camera} direct')
        axes[0].axhline(10,color='gray',ls=':',label='10 per lens required for hypothesis')
        axes[0].set_ylabel('Best candidate matches / lens');axes[0].legend(ncol=3,fontsize=8)
        axes[1].scatter(times,[r.get('hypotheses',0) for r in recovery],s=8,label='PnP hypotheses')
        axes[1].scatter(times,[r.get('solved_cam0',0)+r.get('solved_cam1',0) for r in recovery],s=8,label='PnP solve returns (repeatable)')
        axes[1].set_ylabel('Recovery hypothesis count');axes[1].legend()
        axes[2].step([float(r['input_t_s'])-offset for r in online],[int(r['inliers']) for r in online],lw=.5,label='Online inliers')
        for item in transactions:
            axes[2].axvline(item['t']-offset,color='green' if item.get('committed') else 'red',alpha=.45,lw=.8)
        for item in epochs[1:]:
            for axis in axes:axis.axvline(item['first_t']-offset,color='black',ls='--',alpha=.3)
        axes[2].set(xlabel='Elapsed supported input time [s]',ylabel='Online inliers')
        for axis in axes:axis.grid(alpha=.2)
        figure.suptitle('Recovery funnel: green = confirmed relatch, red = rolled back, dashed = map epoch change')
        figure.savefig(out/'recovery_funnel.png',dpi=160);plt.close(figure)
    label=args.label or ('Long' if '_long_' in run.name else 'Medium' if '_medium_' in run.name else run.name)
    plot_handoff(summary,out,label)
    print(json.dumps({key:value for key,value in summary.items() if key not in ('geometry_accepted_events','transaction_events','online_epochs','split_events')},indent=2))


if __name__=='__main__':main()
