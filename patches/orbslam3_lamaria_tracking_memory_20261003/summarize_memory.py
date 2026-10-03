#!/usr/bin/env python3
"""Summarize observed B proposals/accepted associations; never modifies estimator outputs."""
import argparse
import json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('case',type=Path);args=ap.parse_args()
    case=args.case.resolve(strict=True)
    plans=list(case.glob('*_full/plan.json'))
    if len(plans)!=1:raise SystemExit('Expected one sequence plan')
    run=Path(json.loads(plans[0].read_text())['run_directory'])
    records=[];failures=[]
    for line in (run/'run.log').open(errors='replace'):
        if '[TRACK_MEMORY_DIAG] ' in line:
            try:records.append(json.loads(line.split('[TRACK_MEMORY_DIAG] ',1)[1]))
            except json.JSONDecodeError:pass
        if '[LM_DIAG] ' in line:
            try:
                row=json.loads(line.split('[LM_DIAG] ',1)[1])
                if not row['success']:failures.append(row['time'])
            except (json.JSONDecodeError,KeyError):pass
    seeded=[sum(r['seeded']) for r in records]
    retained=[sum(r['retained']) for r in records]
    total_seeded=sum(seeded);total_retained=sum(retained)
    result={'run':str(run),'logged_frames':len(records),
            'frames_with_memory_proposals':sum(x>0 for x in seeded),
            'successful_frames_with_retained_memory':sum(x>0 for x in retained),
            'seeded_camera_associations':total_seeded,'accepted_camera_associations':total_retained,
            'accepted_fraction':total_retained/total_seeded if total_seeded else None,
            'max_cache_entries':max((r['cache'] for r in records),default=0),
            'logged_local_map_failures':len(failures),
            'interpretation':'Counts describe this run. They do not prove the counterfactual tracker would fail without the seeded associations.'}
    (case/'tracking_memory_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n')
    if records:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        time=np.asarray([r['time'] for r in records]);origin=time[0]
        fig,ax=plt.subplots(3,1,figsize=(14,9),sharex=True,constrained_layout=True)
        ax[0].plot(time-origin,seeded,lw=.6,label='Memory proposals')
        ax[0].plot(time-origin,retained,lw=.6,label='Accepted after optimizer')
        ax[0].set(ylabel='Added camera associations');ax[0].legend()
        ax[1].plot(time-origin,[r['cache'] for r in records],lw=.7)
        ax[1].set(ylabel='Cached camera / landmark IDs',ylim=(0,None))
        ax[2].plot(time-origin,[r['total_inliers'] for r in records],lw=.7,label='All optimizer inliers at diagnostic frames')
        if failures:ax[2].scatter(np.asarray(failures)-origin,[0]*len(failures),c='red',s=4,label='Rejected local-map update')
        ax[2].set(xlabel='Input seconds since first memory diagnostic',ylabel='Inliers');ax[2].legend()
        fig.suptitle(f'Bounded accepted-track memory: {total_retained}/{total_seeded} proposed associations survived accepted frames')
        fig.savefig(case/'tracking_memory_diagnostic.png',dpi=160);plt.close(fig)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
