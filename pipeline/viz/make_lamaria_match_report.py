#!/usr/bin/env python3
"""Build an offline MATCH diagnostic report from an existing replay and RRD.

No estimator, matching, geometry or scoring code is run or changed. LM_DIAG
observations remain sampled observations, not a new estimate of all frames.
"""
import argparse
import base64
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from plotly.offline import get_plotlyjs

from make_lamaria_failure_diagnostics import cameras


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def finite_json(value):
    """Reject non-standard NaN JSON; missing diagnostic values stay null."""
    return json.dumps(value, allow_nan=False, separators=(',', ':')).replace('</', '<\\/')


def load_json(path):
    return json.loads(path.read_text())


def select_window(rows, summary, start=None, end=None):
    lo, hi = min(r['time'] for r in rows), max(r['time'] for r in rows)
    episodes = summary.get('sustained_low_episodes', [])
    usable = [e for e in episodes if isinstance(e, dict) and 'first_t' in e and 'last_t' in e]
    # The first sustained-low episode is a reproducible selection, not a claim
    # that it caused a map failure. Explicit bounds override this focus window.
    if usable:
        first = min(usable, key=lambda e: e['first_t'])
        default_start, default_end = first['first_t'] - 12., first['last_t'] + 12.
        basis = 'First sustained-low episode reported by the log parser, with 12 seconds of context.'
    else:
        default_start, default_end = max(lo, hi - 60.), hi
        basis = 'Last 60 seconds of available MATCH diagnostics; parser reported no sustained-low episode.'
    if start is not None or end is not None:
        basis = 'Explicit diagnostic focus bounds supplied to the report generator.'
    start = max(lo, float(default_start if start is None else start))
    end = min(hi, float(default_end if end is None else end))
    if end <= start:
        raise ValueError('Selected diagnostic window has no duration')
    return [start, end], basis, usable


def make_stills(run, window, out, extra_times=None, extra_focus_times=None):
    stamps = np.loadtxt(run/'config/timestamps_ns.txt', dtype=np.int64, ndmin=1)
    eligible = stamps[(stamps >= round(window[0]*1e9)) & (stamps <= round(window[1]*1e9))]
    if not len(eligible):
        raise ValueError('No native image timestamps in the selected diagnostic window')
    chosen = eligible[np.unique(np.rint(np.linspace(0, len(eligible)-1, min(6, len(eligible)))).astype(int))]
    if extra_focus_times:
        chosen = np.unique(np.r_[chosen, [int(stamps[np.argmin(np.abs(stamps-round(t*1e9)))]) for t in extra_focus_times]]).astype(np.int64)
    focus_stamps = set(map(int, chosen))
    if extra_times:
        extra = [int(stamps[np.argmin(np.abs(stamps-round(t*1e9)))]) for t in extra_times]
        chosen = np.unique(np.r_[chosen, extra]).astype(np.int64)
    payloads = cameras(run, chosen)
    selected_stats = {}
    with (run/'stats.csv').open() as handle:
        for row in csv.DictReader(handle):
            stamp = round(float(row['t'])*1e9)
            closest = int(chosen[np.argmin(np.abs(chosen-stamp))])
            if abs(closest-stamp) <= 1:
                selected_stats[closest, int(row['cam'])] = row
    panels, records = [], []
    out.mkdir(parents=True, exist_ok=False)
    for stamp in map(int, chosen):
        cam_panels, cam_records = [], []
        for cam, frame in enumerate(payloads[stamp]):
            raw = frame['image']
            image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f'Invalid source JPEG at {stamp}, camera {cam}')
            height, width = image.shape[:2]
            for strip, color in zip(frame['trails'], frame['trail_colors']):
                points = np.rint(np.asarray(strip)).astype(np.int32)
                if len(points) >= 2:
                    cv2.polylines(image, [points], False, tuple(map(int, color[:3][::-1])), 1, cv2.LINE_AA)
            for point, color in zip(frame['pixels'], frame['colors']):
                cv2.circle(image, tuple(np.rint(point).astype(int)), 2, tuple(map(int, color[:3][::-1])), -1, cv2.LINE_AA)
            mapped = int(np.count_nonzero(np.all(frame['colors'][:, :3] == [40, 230, 80], axis=1)))
            expected = selected_stats[stamp, cam]
            if mapped != int(expected['mapped']) or len(frame['pixels']) != int(expected['features']):
                raise ValueError(f'Recorded overlays disagree with source statistics at {stamp}, camera {cam}')
            name = f'frame_{stamp}_cam{cam}.jpg'
            path = out/name
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 94])
            if not ok:
                raise ValueError('Failed to encode overlay still')
            path.write_bytes(encoded.tobytes())
            cam_panels.append({'cam': cam, 'width': width, 'height': height, 'mapped': mapped,
                               'detected': len(frame['pixels']),
                               'image': 'data:image/jpeg;base64,' + base64.b64encode(encoded).decode('ascii')})
            cam_records.append({'camera': cam, 'source_encoded_image_sha256': hashlib.sha256(raw).hexdigest(),
                                'source_overlay_float32_sha256': hashlib.sha256(frame['pixels'].tobytes()).hexdigest(),
                                'source_color_uint8_sha256': hashlib.sha256(frame['colors'].tobytes()).hexdigest(),
                                'detected': len(frame['pixels']), 'mapped': mapped, 'display_size': [width, height],
                                'rendered_still': str(path), 'rendered_still_sha256': sha(path)})
        panels.append({'timestamp_ns': stamp, 'time': stamp*1e-9,
                       'scope': 'focus' if stamp in focus_stamps else 'comparison', 'cameras': cam_panels})
        records.append({'timestamp_ns': stamp, 'cameras': cam_records})
    return panels, records


HTML = r'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>LaMAria · matching diagnostics</title>
<style>
:root{color-scheme:dark;--bg:#10161d;--panel:#18212d;--line:#2b3b4f;--fg:#e8f1fb;--muted:#9eb0c6;--accent:#75d8b0}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}main{max-width:1480px;margin:auto;padding:28px}h1{font-size:29px;margin:8px 0}h2{font-size:19px;margin:0 0 8px}h3{font-size:16px;margin:0 0 8px}p{line-height:1.55;color:var(--muted);margin:8px 0 14px}.eyebrow{font-size:12px;letter-spacing:2px;color:var(--accent)}.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:17px;margin:16px 0}.two{display:grid;grid-template-columns:1fr 1fr;gap:16px}.two>.panel{margin:0}.plot{height:310px}.small{font-size:12px}.notice{border-left:3px solid #f4be69;background:#302d26;padding:10px 14px;color:#f3d29f}.cards{display:flex;gap:12px;flex-wrap:wrap}.card{flex:1;min-width:180px;border:1px solid var(--line);background:var(--panel);border-radius:8px;padding:13px}.card strong{font-size:22px;color:var(--accent);display:block}.card span{font-size:12px;color:var(--muted)}code{font:12px ui-monospace,monospace;overflow-wrap:anywhere;white-space:normal;display:block;color:#a8c9ee}.stills{display:grid;grid-template-columns:1fr 1fr;gap:12px}.still{background:#0c131e;border:1px solid var(--line);padding:12px;border-radius:8px}.images{display:flex;gap:8px;align-items:flex-start}.camera{flex:1;min-width:0}.camera img{width:100%;height:auto;border-radius:4px}.caption{font-size:12px;color:var(--muted);padding:6px 0}button{background:#294635;color:#d9f6e8;border:1px solid #4c8261;border-radius:5px;padding:8px 15px;cursor:pointer}table{width:100%;border-collapse:collapse;font-size:12px}th,td{text-align:right;padding:7px;border-bottom:1px solid var(--line)}th:first-child,td:first-child{text-align:left}summary{cursor:pointer;color:var(--accent)}@media(max-width:900px){main{padding:15px}.two,.stills{grid-template-columns:1fr}}
</style><script>__PLOTLY__</script></head><body><main>
<div class="eyebrow">FRESH REPLAY · MATCH INSTRUMENTATION</div><h1>Where do the mapped matches disappear?</h1>
<p id="runname"></p><div class="notice">Measured matching and optimization observations from this replay. These plots identify when and where counts change; they do not establish a causal root cause.</div>
<div class="cards" id="cards"></div>
<div class="panel"><h2>Actual camera observations in the focus window</h2><p class="small">Native timestamps sampled from the fresh recording, including any explicitly selected diagnostic frame. Original displayed camera images, with recorded keypoints and persistent-ID trails redrawn at their recorded pixel positions. Green means mapped association; it does not guarantee an optimization inlier. Camera display rotations are inherited unchanged from the recording.</p><div id="stills" class="stills"></div></div>
<div class="panel"><h2>Tracking state through this replay</h2><p class="small">Full-rate online state/coasting/map ID when available. MATCH markers record local-map function outcomes before the outer tracking state is updated: state OK with success=0 can initiate a failure, while RECENTLY_LOST with success=1 can mark recovery. Healthy-frame logging frequency is controlled by the replay configuration. Shading marks parser-reported sustained-low episodes.</p><div id="stateplot" class="plot"></div><button id="focusButton">Zoom plots to focus window</button></div>
<div class="panel"><h2>Focus window</h2><p id="focusnote"></p><div id="episodeTable"></div></div>
<div class="panel" id="comparisonPanel" style="display:none"><h2>The earlier baseline's loss window</h2><p id="comparisonNote"></p><div id="comparisonplot" class="plot"></div><div id="comparisonTable"></div><p class="small">The following two image pairs are from the fresh replay in this comparison interval, outside the main focus window. All other detailed plots below retain the main focus interval.</p><div id="comparisonStills" class="stills"></div></div>
<div class="two"><div class="panel"><h2>Cam0: visibility and optimization</h2><div id="cam0plot" class="plot"></div></div><div class="panel"><h2>Cam1: visibility and optimization</h2><div id="cam1plot" class="plot"></div></div></div>
<p class="small">“Visible” counts newly checked local landmarks in the camera frustum. “Before optimization” also includes incoming associations and any companion matches, so these curves are different populations, not successive steps of one funnel. “After optimization” counts surviving associations; total inliers can use additional map-observation requirements.</p>
<div class="two"><div class="panel"><h2>Cam0: projection rejection funnel</h2><div id="funnel0" class="plot"></div></div><div class="panel"><h2>Cam1: projection rejection funnel</h2><div id="funnel1" class="plot"></div></div></div>
<p class="small">Funnel totals sum only emitted projection calls inside the focus window, not every video frame. Direct accepted assignments exclude separate stereo companion writes. Candidate evaluations and occupied candidates are not landmark counts. Missing projection records remain missing.</p>
<div class="panel"><h2>Rejection outcomes over time</h2><div id="rejectplot" class="plot"></div><div id="projectionTable"></div></div>
<div class="two"><div class="panel"><h2>Velocity before and after optimization</h2><div id="speedplot" class="plot"></div></div><div class="panel"><h2>Velocity update proposed and retained</h2><div id="deltaplot" class="plot"></div></div></div>
<p class="small">“Predicted” means the state immediately before the local-map optimizer, potentially after a relatch. Predicted and optimized speeds are measured norms. The accepted velocity delta compares the returned state with that pre-optimizer state; a zero delta can mean that a proposed update was restored. Accepted speed and accepted bias are not emitted and are not inferred.</p>
<div class="two"><div class="panel"><h2>Gyroscope bias update</h2><div id="gyroplot" class="plot"></div></div><div class="panel"><h2>Accelerometer bias update</h2><div id="accelplot" class="plot"></div></div></div>
<div class="two"><div class="panel"><h2>Proposed camera-position update</h2><div id="poseplot" class="plot"></div></div><div class="panel"><h2>IMU initialization and optimizer path</h2><div id="optimizerplot" class="plot"></div></div></div>
<div class="panel"><p class="small">3D recording scope: the accompanying run.rrd shows final geometry from its selected largest map, revealed over time. It is not the raw live optimizer map. Where atlas_overview.rrd is provided, its raw independent maps occupy separate views and are not joined.</p><details><summary>Sources, sampling and interpretation</summary><div id="sources"></div><p class="small">This report performs no SLAM, scoring, alignment, recalibration or map joining. It uses no ground truth. It does not turn null diagnostics into zeros. Final geometry in the main recording and live optimizer observations can describe different stages of the estimator. A backend correction or a changed map gauge can affect state values; a jump alone is not proof of a physical movement or a specific geometry defect.</p></details></div>
</main><script>
const data=__DATA__,rows=data.rows,win=data.window,colors=['#7eb0fc','#74d7ad','#f1bd6e','#d48edb','#e97e84'];
const config={responsive:true,displaylogo:false,toImageButtonOptions:{format:'png',scale:2}};
function esc(s){return String(s).replace(/[&<>\"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[c]));}
const finite=v=>typeof v==='number'&&Number.isFinite(v)?v:null;
const focus=rows.filter(r=>r.time>=win[0]&&r.time<=win[1]);
const bands=data.episodes.map(e=>({type:'rect',xref:'x',yref:'paper',x0:e.first_t,x1:e.last_t,y0:0,y1:1,fillcolor:'#d3576a',opacity:.14,line:{width:0},layer:'below'}));
function layout(extra={}){return {paper_bgcolor:'#18212d',plot_bgcolor:'#18212d',font:{color:'#dce7f5',family:'system-ui'},margin:{l:58,r:18,t:12,b:60},legend:{orientation:'h',y:-.23,font:{size:11}},xaxis:{title:'Native time (s)',gridcolor:'#2b3b4f',range:win},yaxis:{gridcolor:'#2b3b4f',rangemode:'tozero'},shapes:bands,...extra};}
const plotIds=[];
function plot(id,traces,extra={}){plotIds.push(id);return Plotly.newPlot(id,traces,layout(extra),config);}
function series(name,get,color,index=focus){return {type:index.length>3000?'scattergl':'scatter',mode:'lines+markers',name,x:index.map(r=>r.time),y:index.map(r=>finite(get(r))),line:{color,width:1.3},marker:{size:3,color},connectgaps:false,hovertemplate:'t=%{x:.9f}s<br>%{y:.5g}<extra>'+name+'</extra>'};}
function diagnosticSeries(key,name,color){return series(name,r=>r[key],color);}
document.getElementById('runname').textContent=data.run;
document.getElementById('cards').innerHTML=[['Logged calls',rows.length],['Focus window',win.map(v=>v.toFixed(3)).join(' – ')+' s'],['Focus call failures',focus.filter(r=>!r.success).length],['Focus camera samples',data.stills.filter(s=>s.scope==='focus').length+' pairs']].map(([a,b])=>`<div class="card"><strong>${esc(b)}</strong><span>${a}</span></div>`).join('');
document.getElementById('focusnote').textContent=data.window_basis+' Detailed plots contain only this focus window; the state overview above covers the replay. Red shading is a measured low-inlier episode, not an automatically diagnosed cause. Plot lines connect emitted observations; normal-frame sampling can be sparse.';
document.getElementById('episodeTable').innerHTML='<table><tr><th>Reported episode</th><th>Start (s)</th><th>End (s)</th><th>Map</th><th>Logged calls</th></tr>'+data.episodes.map((e,i)=>`<tr><td>${i+1}</td><td>${e.first_t.toFixed(6)}</td><td>${e.last_t.toFixed(6)}</td><td>${esc(e.map)}</td><td>${esc(e.frames)}</td></tr>`).join('')+'</table>';
function stillHTML(samples){return samples.map(s=>`<div class="still"><h3>t=${s.time.toFixed(9)} s</h3><div class="images">${s.cameras.map(c=>`<div class="camera"><img src="${c.image}" width="${c.width}" height="${c.height}" alt="Camera ${c.cam} recorded overlays at ${s.time}"><div class="caption">cam${c.cam} · ${c.mapped}/${c.detected} mapped</div></div>`).join('')}</div></div>`).join('');}
document.getElementById('stills').innerHTML=stillHTML(data.stills.filter(s=>s.scope==='focus'));
const actual=data.online.length?data.online:rows;
plot('stateplot',[series('Tracking state',r=>r.state,colors[0],actual),series('Coasting',r=>r.coasting,colors[3],actual),series('Map ID',r=>r.map??r.map_id,colors[2],actual),{type:'scattergl',mode:'markers',name:'Local-map call failed',x:rows.filter(r=>!r.success).map(r=>r.time),y:rows.filter(r=>!r.success).map(r=>r.state),marker:{color:colors[4],size:7,symbol:'x'}}],{xaxis:{title:'Native time (s)',gridcolor:'#2b3b4f'},yaxis:{title:'Raw enum / flag / map ID',gridcolor:'#2b3b4f'}});
if(data.comparison){
 const c=data.comparison,inside=values=>values.filter(r=>r.time>=c.window[0]&&r.time<=c.window[1]);
 const replay=inside(data.online),baseline=inside(c.online);
 document.getElementById('comparisonPanel').style.display='block';
 document.getElementById('comparisonNote').textContent=c.note+' This compares matching observations at the same native times; it is not evidence of a repaired algorithm.';
 plot('comparisonplot',[series('Fresh replay: online inliers',r=>r.inliers,colors[1],replay),series('Earlier baseline: online inliers',r=>r.inliers,colors[4],baseline)],{xaxis:{title:'Native time (s)',range:c.window,gridcolor:'#2b3b4f'},yaxis:{title:'Online reported inliers',gridcolor:'#2b3b4f'},shapes:[]});
 document.getElementById('comparisonTable').innerHTML='<table><tr><th>Same interval</th><th>Map IDs observed</th><th>Coasting frames</th><th>Recorded frames</th></tr>'+[['Fresh replay',replay],['Earlier baseline',baseline]].map(([name,values])=>`<tr><td>${name}</td><td>${[...new Set(values.map(r=>r.map_id))].join(', ')}</td><td>${values.filter(r=>r.coasting).length}</td><td>${values.length}</td></tr>`).join('')+'</table>';
 document.getElementById('comparisonStills').innerHTML=stillHTML(data.stills.filter(s=>s.scope==='comparison'));
}
for(let cam=0;cam<2;cam++){
 plot('cam'+cam+'plot',[['visible','New local landmarks visible'],['incoming','Incoming associations'],['before_opt','Before optimization'],['after_opt','After optimization'],['outliers','Flagged outliers']].map(([k,n],i)=>series(n,r=>r.cams?.[cam]?.[k],colors[i])),{yaxis:{title:'Count per logged local-map call',gridcolor:'#2b3b4f'}});
 const p=focus.filter(r=>r.projection?.cams?.[cam]).map(r=>r.projection.cams[cam]);
 const sum=k=>p.reduce((s,c)=>s+(finite(c[k])??0),0);
 const visible=sum('visible'),invalid=sum('invalid_level'),empty=sum('empty'),occupied=sum('occupied_only'),geometry=sum('geometry_only'),descriptor=sum('descriptor'),ratio=sum('ratio'),accepted=sum('accepted');
 const stages=['Visible in projection matcher','Valid scale level','Has nearby candidates','Has eligible candidates','Passes descriptor threshold','Passes ratio / accepted'];
 const values=[visible,visible-invalid,visible-invalid-empty,visible-invalid-empty-occupied-geometry,visible-invalid-empty-occupied-geometry-descriptor,accepted];
 plot('funnel'+cam,[{type:'funnel',y:stages,x:values,textinfo:'value+percent initial',marker:{color:colors[cam]},hovertemplate:'%{y}<br>%{x} landmark attempts<extra></extra>'}],{xaxis:{},yaxis:{},shapes:[],margin:{l:195,r:18,t:12,b:35},showlegend:false,annotations:[{xref:'paper',yref:'paper',x:1,y:1.08,text:p.length+' emitted projection calls',showarrow:false,font:{size:11,color:'#9eb0c6'}}]});
}
const outcomeKeys=[['empty','No candidates'],['occupied_only','Only occupied candidates'],['geometry_only','Only geometry-rejected candidates'],['descriptor','Descriptor rejection'],['ratio','Ratio rejection'],['invalid_level','Invalid predicted level']];
plot('rejectplot',outcomeKeys.map(([k,n],i)=>series(n,r=>r.projection?.cams? r.projection.cams.reduce((s,c)=>s+(finite(c[k])??0),0):null,[...colors,'#a7aabc'][i])),{yaxis:{title:'Rejected landmark attempts, both cameras',gridcolor:'#2b3b4f'}});
const projectionKeys=['visible','empty','occupied_only','geometry_only','descriptor','ratio','invalid_level','accepted','candidates','occupied','stereo_error','companion_written','companion_conflict'];
document.getElementById('projectionTable').innerHTML='<p class="small">Totals over emitted projection calls in the focus window. “Candidates”, “occupied”, and “stereo_error” count candidate evaluations; the other rejection categories count landmark attempts.</p><table><tr><th>Counter</th><th>Cam0 total</th><th>Cam1 total</th></tr>'+projectionKeys.map(k=>'<tr><td>'+k+'</td>'+[0,1].map(c=>'<td>'+focus.reduce((s,r)=>s+(finite(r.projection?.cams?.[c]?.[k])??0),0).toLocaleString()+'</td>').join('')+'</tr>').join('')+'</table>';
plot('speedplot',[diagnosticSeries('pred_speed_mps','Predicted speed',colors[0]),diagnosticSeries('opt_speed_mps','Optimized speed',colors[2])],{yaxis:{title:'Speed norm (m/s)',gridcolor:'#2b3b4f'}});
plot('deltaplot',[diagnosticSeries('opt_velocity_delta_mps','Proposed velocity delta',colors[2]),diagnosticSeries('accepted_velocity_delta_mps','Accepted velocity delta',colors[1])],{yaxis:{title:'Norm of velocity change (m/s)',gridcolor:'#2b3b4f'}});
plot('gyroplot',[diagnosticSeries('pred_gyro_bias_norm','Predicted bias norm',colors[0]),diagnosticSeries('opt_gyro_bias_norm','Optimized bias norm',colors[2]),diagnosticSeries('opt_gyro_bias_delta','Proposed bias delta norm',colors[3])],{yaxis:{title:'Gyro bias (rad/s)',gridcolor:'#2b3b4f'}});
plot('accelplot',[diagnosticSeries('pred_accel_bias_norm','Predicted bias norm',colors[0]),diagnosticSeries('opt_accel_bias_norm','Optimized bias norm',colors[2]),diagnosticSeries('opt_accel_bias_delta','Proposed bias delta norm',colors[3])],{yaxis:{title:'Accelerometer bias (m/s²)',gridcolor:'#2b3b4f'}});
plot('poseplot',[diagnosticSeries('opt_camera_displacement_m','Optimized minus predicted camera position',colors[2])],{yaxis:{title:'Position-delta norm (m)',gridcolor:'#2b3b4f'}});
const optNames=[...new Set(rows.map(r=>r.optimizer))];
plot('optimizerplot',[diagnosticSeries('imu_initialized','IMU initialized',colors[1]),diagnosticSeries('ba2','Second VI-BA complete',colors[2]),series('Optimizer path',r=>optNames.indexOf(r.optimizer)+2,colors[3])],{yaxis:{tickvals:[0,1,...optNames.map((v,i)=>i+2)],ticktext:['0','1',...optNames],gridcolor:'#2b3b4f'},margin:{l:150,r:18,t:12,b:60}});
document.getElementById('focusButton').onclick=()=>plotIds.filter(id=>!id.startsWith('funnel')&&id!=='comparisonplot').forEach(id=>Plotly.relayout(id,{'xaxis.range':win}));
document.getElementById('sources').innerHTML=data.source_paths.map(p=>'<code>'+esc(p)+'</code>').join('')+'<p class="small">Parser errors: '+esc(JSON.stringify(data.parse_report.parse_errors_by_tag||{}))+'. Dropped records: '+esc(JSON.stringify(data.parse_report.dropped_records||{}))+'.</p>';
window.matchReportValidation={ready:true,loggedCalls:rows.length,focusCalls:focus.length,stills:data.stills.length,comparison:!!data.comparison,plotCount:plotIds.length,window:win};
</script></body></html>'''


def build(args):
    run = args.run.resolve(strict=True)
    viz = run/'viz'
    destination = viz/'report.html'
    provenance_path = viz/'match_report_provenance.json'
    validation_path = viz/'match_report_validation.json'
    still_path = viz/'match_report_stills'
    if any(p.exists() for p in [destination, provenance_path, validation_path, still_path]):
        raise FileExistsError('Report outputs already exist; preserve them and choose a fresh replay directory')
    diagnostic_path = viz/'match_diagnostics.json'
    parse_path = viz/'parse_report.json'
    summary_path = viz/'match_diagnostics_summary.json'
    rows = load_json(diagnostic_path)
    parse_report = load_json(parse_path)
    if not parse_report.get('finished'):
        raise ValueError('Wait until the parser marks this replay finished before creating its final report')
    summary = load_json(summary_path) if summary_path.exists() else {}
    if not isinstance(rows, list) or not rows:
        raise ValueError('Expected a nonempty top-level list of LM_DIAG observations')
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get('time'), (float, int)) or len(row.get('cams', [])) != 2:
            raise ValueError('Malformed LM_DIAG observation')
    if not all(a['time'] <= b['time'] for a, b in zip(rows, rows[1:])):
        raise ValueError('MATCH observations are not in timestamp order')
    window, basis, episodes = select_window(rows, summary, args.start, args.end)
    if args.window_note:
        basis += ' ' + args.window_note
    focus = [r for r in rows if window[0] <= r['time'] <= window[1]]
    if not focus:
        raise ValueError('No diagnostics in focus window')
    outcome_keys = ['empty', 'occupied_only', 'geometry_only', 'descriptor', 'ratio', 'invalid_level', 'accepted']
    projection_checked = [0, 0]
    for row in rows:
        projection = row.get('projection')
        if projection is None:
            continue
        for cam, counts in enumerate(projection['cams']):
            total = sum(counts[k] for k in outcome_keys)
            if counts['visible'] != total:
                raise ValueError(f"Projection outcomes do not partition visible attempts: t={row['time']}, cam={cam}, {counts['visible']} != {total}")
            projection_checked[cam] += 1
    online = []
    online_files = list(run.glob('online_*.csv'))
    if len(online_files) == 1:
        with online_files[0].open() as handle:
            for row in csv.DictReader(handle):
                online.append({'time': float(row['input_t_s']), **{k: int(row[k]) for k in ('state', 'coasting', 'map_id', 'inliers')}})
    comparison, comparison_files = None, []
    if args.comparison_window:
        if not args.comparison_run or not args.comparison_note:
            raise ValueError('Comparison window requires --comparison-run and --comparison-note')
        comparison_files = list(args.comparison_run.resolve(strict=True).glob('online_*.csv'))
        if len(comparison_files) != 1:
            raise ValueError('Expected one baseline online CSV for the comparison')
        with comparison_files[0].open() as handle:
            previous = [{'time': float(row['input_t_s']), **{k: int(row[k]) for k in ('state', 'coasting', 'map_id', 'inliers')}} for row in csv.DictReader(handle)
                        if args.comparison_window[0] <= float(row['input_t_s']) <= args.comparison_window[1]]
        comparison = {'window': args.comparison_window, 'note': args.comparison_note,
                      'run': str(args.comparison_run.resolve()), 'online': previous}
    stills, still_records = make_stills(run, window, still_path, args.comparison_stills, args.extra_focus_still)
    source_paths = [diagnostic_path, parse_path, summary_path, run/'stats.csv', run/'config/timestamps_ns.txt', run/'config/settings.yaml', *online_files, *comparison_files]
    source_paths = [p for p in source_paths if p.exists()]
    for name in ['result.json', 'command.json', 'output_metadata.json', 'run.rrd.verify.log']:
        if (run/name).exists():
            source_paths.append(run/name)
    data = {'run': str(run), 'rows': rows, 'online': online, 'window': window, 'window_basis': basis,
            'episodes': episodes, 'stills': stills, 'parse_report': parse_report, 'comparison': comparison,
            'source_paths': [str(p) for p in source_paths] + [str(run/'run.rrd')]}
    text = HTML.replace('__PLOTLY__', get_plotlyjs()).replace('__DATA__', finite_json(data))
    destination.write_text(text)
    provenance = {'artifact_kind': 'Offline MATCH diagnostic report from an existing replay',
                  'run': str(run), 'builder': str(Path(__file__).resolve()), 'builder_sha256': sha(Path(__file__)),
                  'source_sha256': {str(p): sha(p) for p in source_paths},
                  'recording': {'path': str(run/'run.rrd'), 'bytes': (run/'run.rrd').stat().st_size,
                                'note': 'The multi-GB recording is not hashed again; exact sampled image payload hashes are recorded below.'},
                  'selected_window_native_seconds': window, 'window_selection': basis,
                  'additional_focus_still_native_seconds': args.extra_focus_still,
                  'comparison_window_native_seconds': args.comparison_window,
                  'comparison_source_run': str(args.comparison_run.resolve()) if args.comparison_run else None,
                  'stills': still_records, 'plotly_bundled': True, 'network_dependencies': [],
                  'GT_used': False, 'new_estimator_or_scoring_runs': 0, 'estimator_or_source_files_modified': False}
    provenance_path.write_text(json.dumps(provenance, indent=2, allow_nan=False)+'\n')
    validation = {'passed': True, 'logged_calls': len(rows), 'focus_calls': len(focus),
                  'projection_outcome_partitions_checked_per_camera': projection_checked,
                  'still_pairs': len(stills), 'all_stills_use_exact_native_input_timestamps': True,
                  'all_missing_diagnostic_scalars_preserved_as_null': True,
                  'source_images_and_overlay_positions_reused_without_registration_changes': True,
                  'sampled_mapped_and_detected_counts_match_source_stats': True,
                  'browser_check': 'Pending independent browser rendering; data validation only.'}
    validation_path.write_text(json.dumps(validation, indent=2)+'\n')
    print(json.dumps({'report': str(destination), **validation}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--start', type=float, help='Optional focus-window start in native seconds')
    parser.add_argument('--end', type=float, help='Optional focus-window end in native seconds')
    parser.add_argument('--window-note', help='Evidence-based description of this selected interval; displayed verbatim as text')
    parser.add_argument('--comparison-window', type=float, nargs=2, metavar=('START', 'END'))
    parser.add_argument('--comparison-run', type=Path, help='Earlier baseline with an online CSV, for same-time inlier comparison')
    parser.add_argument('--comparison-note', help='Evidence-based description of whether the baseline loss reproduced')
    parser.add_argument('--comparison-stills', type=float, nargs=2, metavar=('FIRST', 'SECOND'))
    parser.add_argument('--extra-focus-still', type=float, action='append', help='Additional native timestamp to show among the main focus camera samples')
    build(parser.parse_args())
