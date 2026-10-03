#!/usr/bin/env python3
"""Read-only coverage accounting; no GT stitching or gap filling. Writes --out only."""
import argparse,csv,json,re
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np

def rows(path):
    with path.open() as f:return list(csv.DictReader(f))
def one(run,pattern):
    files=sorted(run.glob(pattern))
    if len(files)>1:raise ValueError(f'Ambiguous {pattern}: {files}')
    return files[0] if files else None

def indices(stamps,native):
    stamps=np.asarray(stamps,dtype=float);hi=np.searchsorted(native,stamps).clip(0,len(native)-1);lo=(hi-1).clip(0,len(native)-1)
    j=np.where(abs(native[lo]-stamps)<=abs(native[hi]-stamps),lo,hi)
    return j,abs(native[j]-stamps)<=1e-6

def spans(ids,native):
    ids=np.unique(ids);out=[]
    for part in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
        if len(part):out.append({'frames':len(part),'start_s':float(native[part[0]]),'end_s':float(native[part[-1]]),'duration_s':float(native[part[-1]]-native[part[0]])})
    return sorted(out,key=lambda x:x['frames'],reverse=True)

def summarize(run,native):
    result={'run':str(run),'input_frames':len(native),'input_start_s':float(native[0]),'input_end_s':float(native[-1]),'input_span_s':float(native[-1]-native[0])}
    log=(run/'run.log').read_text(errors='replace') if (run/'run.log').exists() else ''
    result['completed_active_map_resets']=log.count('LM: Active map reset, Done!!!')
    result['new_map_events']=log.count('Creation of new map with id:')
    result['retained_map_keyframes']=[{'map_id':int(i),'keyframes':int(n)} for i,n in re.findall(r'^  Map (\d+) has (\d+) KFs$',log,re.M)]
    if (run/'result.json').exists():
        status=json.loads((run/'result.json').read_text());result['estimator_returncode']=status.get('returncode');result['export_returncode']=status.get('export_returncode')
    atlas=one(run,'atlas_*_trajectory.csv');legacy=one(run,'f_*.txt');exported_ids=set();segments=[]
    if atlas:
        groups=defaultdict(list)
        for r in rows(atlas):groups[r['coordinate_frame']].append(r)
        for frame,group in sorted(groups.items()):
            j,ok=indices([float(r['t_s']) for r in group],native);valid=np.unique(j[ok]);exported_ids.update(map(int,valid))
            segments.append({'coordinate_frame':frame,'map_id':int(group[0]['map_id']),'map_init_kf_id':int(group[0]['map_init_kf_id']),'exported_rows':len(group),'unique_input_frames':len(valid),'contiguous_spans':spans(valid,native),'global_anchor_available':False})
        result['pose_export_scope']='All surviving atlas maps, each in its own explicitly named frame'
    elif legacy:
        data=np.loadtxt(legacy,ndmin=2);j,ok=indices(data[:,0]*1e-9,native);valid=np.unique(j[ok]);exported_ids.update(map(int,valid))
        segments.append({'coordinate_frame':'legacy_largest_map_only','exported_rows':len(data),'unique_input_frames':len(valid),'contiguous_spans':spans(valid,native),'global_anchor_available':False})
        result['pose_export_scope']='Largest map only; additional retained map poses were not saved'
    else:result['pose_export_scope']='No trajectory export found'
    result['segments']=segments;result['independent_segments_exported']=len(segments);result['unique_input_frames_with_exported_pose']=len(exported_ids);result['exported_pose_fraction']=len(exported_ids)/len(native)
    result['longest_independently_framed_contiguous_segment']=max((s for m in segments for s in m['contiguous_spans']),key=lambda x:x['frames'],default=None)
    online=one(run,'online_*.csv');conf=run/'f_conf.csv';ok_ids=set();coast_ids=set();logged_ids=set();epoch_ids=defaultdict(set)
    if online:
        data=rows(online);j,valid=indices([float(r['input_t_s']) for r in data],native)
        for r,i,validrow in zip(data,j,valid):
            if not validrow:continue
            i=int(i);logged_ids.add(i)
            has=int(r['pose_available'])==1;state=int(r['state']);coast=int(r['coasting'])==1
            if has and state==2 and not coast:ok_ids.add(i)
            if has and (coast or state==3):coast_ids.add(i)
            if has and state in (2,3):epoch_ids[(r['map_id'],r['map_init_kf_id'])].add(i)
        result['online_pose_rows']=len(data);result['online_pose_rows_cover_every_input']=len(logged_ids)==len(native)
        result['online_quality_source']='Explicit per-input pose/state trace, including initialization and early returns'
    elif conf.exists():
        data=rows(conf);j,valid=indices([float(r['t']) for r in data],native)
        for r,i,validrow in zip(data,j,valid):
            if not validrow:continue
            i=int(i);logged_ids.add(i)
            if int(r['state'])==2 and not int(r['coasting']):ok_ids.add(i)
            if int(r['coasting']) or int(r['state'])==3:coast_ids.add(i)
        result['online_quality_source']='Legacy f_conf; initialization, early returns and unflushed tail are unknown'
    result['logged_native_frames']=len(logged_ids);result['unlogged_native_frames']=len(native)-len(logged_ids)
    result['logged_visual_ok_frames']=len(ok_ids);result['logged_visual_ok_fraction']=len(ok_ids)/len(native)
    result['logged_coasted_frames']=len(coast_ids)
    result['logged_visual_ok_without_final_export']=len(ok_ids-exported_ids)
    result['exported_pose_with_logged_visual_ok']=len(ok_ids&exported_ids)
    result['exported_pose_with_logged_coasting']=len(coast_ids&exported_ids)
    result['online_map_epochs']=[{'map_id':int(k[0]),'map_init_kf_id':int(k[1]),'pose_frames':len(v),'contiguous_spans':spans(list(v),native)} for k,v in epoch_ids.items()]
    history=one(run,'atlas_*_history.csv')
    if history:result['final_history_status_counts']=dict(Counter(r['status'] for r in rows(history)))
    result['interpretation']=['A retained final pose can include IMU-only coasting; pose coverage is not visual success.','Independent-map pose coverage is not a continuous global trajectory. No transform between independent maps is invented.','Visual OK without final export can mean either largest-only suppression or a later cleared map; legacy outputs cannot disambiguate.','Online snapshots preserve estimates before reset, but do not retrospectively follow optimization and may change gauge at world_version updates.']
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--timestamps',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    stamps=np.loadtxt(a.timestamps or a.run_dir/'config/timestamps_ns.txt',ndmin=1)*1e-9
    assert np.all(np.diff(stamps)>0),'Native input timestamps must be unique and sorted'
    result=summarize(a.run_dir,stamps);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('input_frames','independent_segments_exported','unique_input_frames_with_exported_pose','exported_pose_fraction','logged_visual_ok_frames','logged_visual_ok_without_final_export','completed_active_map_resets')},indent=2))
if __name__=='__main__':main()
