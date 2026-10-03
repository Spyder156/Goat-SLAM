#!/usr/bin/env python3
"""Report accepted native-intrinsic history and fair held-out VI-BA trials."""
import argparse,csv,json,re,html as html_module
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 fs=cv2.FileStorage(str(a.run/'config/settings.yaml'),cv2.FILE_STORAGE_READ)
 native=['fx','fy','cx','cy']+[f'k{i}' for i in range(1,7)]+['p1','p2']+[f's{i}' for i in range(1,5)]
 factory=np.array([[fs.getNode(f'Camera{c+1}.{k}').real() for k in native] for c in range(2)]);fs.release()
 names=['f','cx','cy']+[f'k{i}' for i in range(6)]+['p0','p1']+[f's{i}' for i in range(4)]
 indices=[0]+list(range(2,16));base=factory[:,indices]
 bounds=np.array([[.05*f[0],4,4,.02,.01,.003,.001,.0003,.0001,.001,.001,.001,.001,.001,.001] for f in factory])
 text=(a.run/'run.log').read_text(errors='replace');commits={};candidates={};trials=[];held=[]
 for line in text.splitlines():
  m=re.search(r'\[CALIBRATION-COMMIT\] t=(\S+) map=(\d+) cam=([01]) params=(\S+)',line)
  if m:commits.setdefault(float(m[1]),{})[int(m[3])]=[float(x) for x in m[4].split(',')]
  m=re.search(r'\[CALIBRATION-CANDIDATE\] t=(\S+) cam=([01]).* params=(\S+)',line)
  if m:candidates.setdefault(float(m[1]),{})[int(m[2])]=[float(x) for x in m[3].split(',')]
  if line.startswith('[CALIBRATION-TRIAL]'):trials.append(line)
  if line.startswith('[CALIBRATION-HELDOUT]'):
   pairs=dict(re.findall(r'(\w+)=([^ ]+)',line));held.append({k:float(v) for k,v in pairs.items()})
 times=sorted(t for t,c in commits.items() if len(c)==2)
 with next(a.run.glob('online_*.csv')).open() as f:online=list(csv.DictReader(f))
 origin=float(online[0]['input_t_s']);accepted=np.array([[commits[t][c] for c in range(2)] for t in times]) if times else np.zeros((0,2,16))
 result=json.loads((a.run/'result.json').read_text()) if (a.run/'result.json').exists() else None
 metrics={'run':str(a.run.resolve()),'native_origin_s':origin,'last_processed_s':float(online[-1]['input_t_s']),'full_sequence_complete':result is not None and result.get('returncode')==0 and len(online)==result.get('expected_frames'),'estimator_result':result,'processed_frames':len(online),'map_ids':sorted(set(int(row['map_id']) for row in online)),'accepted_updates':len(times),'attempts':len(trials),'trials':trials,'heldout':held,'factory_native_parameters':factory.tolist(),'parameter_names':names,'lifetime_bounds':bounds.tolist(),'commits':[{'t':t,'parameters':commits[t]} for t in times],'candidates':[{'t':t,'parameters':candidates[t]} for t in sorted(candidates)], 'interpretation':'All 15 native intrinsic DOFs per camera; factory priors and 5% of lifetime-bound per LM step; fixed/free branches each20iterations from same10iteration warm start. Full-map VI fixes initial pose and world gravity. Frame lifecycle repairs are also included, so score differences are not an isolated calibration ablation.'}
 score=a.run/'lamaria_score/scores.json'
 if score.exists():metrics['score']=json.loads(score.read_text())
 (a.out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
 if times:
  fig,axes=plt.subplots(2,1,figsize=(max(10,len(times)*.6),8),constrained_layout=True)
  for c,ax in enumerate(axes):
   normal=(accepted[:,c,indices]-base[c])/bounds[c];im=ax.imshow(normal.T,vmin=-1,vmax=1,cmap='coolwarm',aspect='auto');ax.set_yticks(range(15),names);ax.set_xticks(range(len(times)),[f'{t-origin:.0f}' for t in times]);ax.set_title(f'Cam{c}: accepted change / fixed factory bound');ax.set_xlabel('Seconds since first input (accepted updates only)');fig.colorbar(im,ax=ax,label='Fraction of allowed lifetime change')
  fig.savefig(a.out/'intrinsics_history.png',dpi=160);plt.close(fig)
 if candidates:
  ct=sorted(t for t,c in candidates.items() if len(c)==2);values=np.array([[candidates[t][c] for c in range(2)] for t in ct]);fig,axes=plt.subplots(2,1,figsize=(max(10,len(ct)*.65),8),constrained_layout=True)
  for c,ax in enumerate(axes):
   normal=(values[:,c,indices]-base[c])/bounds[c];im=ax.imshow(normal.T,vmin=-1,vmax=1,cmap='coolwarm',aspect='auto');ax.set_yticks(range(15),names);ax.set_xticks(range(len(ct)),[f'{t-origin:.0f}'+(' A' if t in times else ' R') for t in ct]);ax.set_title(f'Cam{c}: all proposals; A=accepted, R=rejected (never applied)');ax.set_xlabel('Seconds since first input');fig.colorbar(im,ax=ax,label='Fraction of fixed factory lifetime bound')
  fig.savefig(a.out/'proposals_history.png',dpi=160);plt.close(fig)
 if held:
  fig,axes=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
  for c in range(2):
   for b in range(2):
    rows=[r for r in held if r['cam']==c and r['outer']==b];ax=axes[c,b];t=[r['t']-origin for r in rows];ax.plot(t,[r['fixed_px'] for r in rows],'o-',label='Fixed intrinsics');ax.plot(t,[r['final_px'] for r in rows],'o-',label='Free intrinsics candidate');ax.set_title(f'Cam{c}, '+('periphery ≥250 px' if b else 'centre <250 px'));ax.set_xlabel('Seconds since first input');ax.set_ylabel('Held-out median reprojection error (px)');ax.grid(alpha=.2);ax.legend()
  fig.savefig(a.out/'heldout_history.png',dpi=160);plt.close(fig)
 title='Recurring full native calibration: Short'
 status='Full replay finished' if metrics['full_sequence_complete'] else ('Replay ended unsuccessfully; no full result claimed' if result else 'Replay in progress; no final accuracy claim')
 html=f'''<!doctype html><html><meta charset="utf-8"><title>{title}</title><style>body{{font:17px system-ui;max-width:1150px;margin:30px auto;padding:15px;color:#18202a;background:#fafbfc}}img{{max-width:100%}}pre{{white-space:pre-wrap;font-size:13px}}table{{border-collapse:collapse}}td,th{{padding:8px;border:1px solid #bbb}}</style><h1>{title}</h1><p>{status}. Accepted updates: {len(times)} / {len(trials)} completed trials.</p><p>{metrics['interpretation']}</p><h2>What a healthy result should show</h2><p>Bounded camera parameters should settle; held-out errors should improve in both lenses and at the image periphery, while full-route coverage, metric geometry and official score improve. Lower reprojection error alone is not success.</p>'''
 for image in ['intrinsics_history.png','proposals_history.png','heldout_history.png']:
  if (a.out/image).exists():html+=f'<img src="{image}">'
 if (a.out/'recovery_diagnostic.json').exists():
  recovery=json.loads((a.out/'recovery_diagnostic.json').read_text())
  html+='<h2>Recovery transaction diagnostic</h2><p>'+html_module.escape(recovery['explanation'])+'</p><img src="recovery_diagnostic.png">'
 if (a.out/'commit_to_loss.json').exists():
  loss=json.loads((a.out/'commit_to_loss.json').read_text())
  html+='<h2>Global update and later tracking loss</h2><p>'+html_module.escape(loss['interpretation'])+'</p><img src="commit_to_loss.png">'
 if (a.out/'rollback_audit.json').exists():
  html+='<h2>Full replay rollback audit</h2><pre>'+html_module.escape((a.out/'rollback_audit.json').read_text())+'</pre>'
 if (a.out/'camera_stills/contact_sheet.png').exists():
  html+='<h2>Exact recorded camera images and associations</h2><p>Green points have map associations; red points do not. Nearby moving hands can supply many detections without supplying stable world landmarks.</p><img src="camera_stills/contact_sheet.png">'
 if 'score' in metrics:html+='<h2>Official local evaluation</h2><pre>'+json.dumps(metrics['score'],indent=2)+'</pre>'
 html+='<h2>Trial decisions</h2><pre>'+ '\n'.join(trials)+'</pre><p>metrics.json contains exact parameter values, held-out counts and input coverage. Ground truth is excluded from estimation.</p></html>'
 (a.out/'report.html').write_text(html)
 print(json.dumps({k:metrics[k] for k in ['accepted_updates','attempts','last_processed_s','full_sequence_complete']}))
if __name__=='__main__':main()
