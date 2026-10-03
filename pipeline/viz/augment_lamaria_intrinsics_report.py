#!/usr/bin/env python3
"""Join independent image tracks to saved live map IDs; never changes estimates."""
import argparse
import json
from pathlib import Path
import numpy as np
import cv2
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import audit_lamaria_intrinsics as audit


def render_residuals(out, summary):
    """Render cached measurements with sample counts; never refit geometry."""
    for group in summary['groups']:
        name = f'{group["sequence"]}_c{group["cam"]}'
        arrays = np.load(out/f'{name}_results.npz')
        pairs = [p for p in group['pairs'] if not p['training_window']]
        xy = np.concatenate([arrays[f'w{p["window"]}_xy'] for p in pairs])
        radii = np.concatenate([arrays[f'w{p["window"]}_radius'] for p in pairs])
        bins = [(0, 150), (150, 200), (200, 250), (250, 290), (290, 330)]
        counts = [int(((radii >= lo)&(radii < hi)).sum()) for lo, hi in bins]
        fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
        for index, model in enumerate(('fixed', 'adjusted')):
            error = np.concatenate([arrays[f'w{p["window"]}_{model}_errors'] for p in pairs])
            vectors = np.concatenate([arrays[f'w{p["window"]}_{model}_vectors'] for p in pairs])
            scatter = axs[index].scatter(xy[:, 0], xy[:, 1], c=np.minimum(abs(error), 3), s=8, cmap='magma', vmin=0, vmax=3)
            for y in range(40, 480, 80):
                for x in range(40, 640, 80):
                    take = (abs(xy[:, 0]-x) < 40)&(abs(xy[:, 1]-y) < 40)&(abs(error) < 5)
                    if take.sum() >= 5:
                        v = np.median(vectors[take], axis=0)
                        axs[index].arrow(x, y, 15*v[0], 15*v[1], color='cyan', head_width=4)
            axs[index].set(xlim=(0, 640), ylim=(480, 0), title=f'{model}: held-out times + tracks', aspect='equal')
            fig.colorbar(scatter, ax=axs[index], label='Absolute epipolar residual (px)', shrink=.7)
            medians = [np.median(abs(error[(radii >= lo)&(radii < hi)])) if n >= 10 else np.nan for (lo, hi), n in zip(bins, counts)]
            axs[2].plot([(lo+hi)/2 for lo, hi in bins], medians, 'o-', label=model)
        for (lo, hi), n in zip(bins, counts):
            axs[2].text((lo+hi)/2, .93, f'n={n}', ha='center', va='top', fontsize=8, color='black' if n >= 10 else 'red')
        axs[2].set(xlabel='Radius from principal point (px)', ylabel='Median held-out residual (px)', ylim=(0, 1),
                   title='Bins with fewer than 10 tracks omitted')
        axs[2].legend(loc='upper left', bbox_to_anchor=(0, .84)); axs[2].grid(alpha=.2)
        fig.suptitle(f'{name}: focal {group["focal_change_percent"]:+.2f}%, k1 {group["k1_change"]:+.5f}; cyan vectors ×15')
        fig.tight_layout(); fig.savefig(out/f'{name}_residuals.png', dpi=150); plt.close(fig)


def nearby_rows(path, start, end):
    result = {}
    with path.open('rb', buffering=1024*1024) as handle:
        lo, hi = 0, path.stat().st_size
        while hi-lo > 262144:
            mid = (lo+hi)//2
            handle.seek(mid); handle.readline(); position = handle.tell(); line = handle.readline()
            if not line: hi = mid
            elif float(line.split(b',', 1)[0]) < start: lo = position
            else: hi = mid
        handle.seek(lo)
        if lo == 0: handle.readline()
        for line in handle:
            fields = line.split(b','); stamp = float(fields[0])
            if stamp < start: continue
            if stamp > end: break
            camera = int(fields[1]); key = (round(stamp*1e9), camera)
            result.setdefault(key, []).append([float(fields[3]), float(fields[4]), int(fields[2]), int(fields[5])])
    return {key: np.asarray(rows) for key, rows in result.items()}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(); out = args.out.resolve()
    suite = json.loads((audit.PROJECT/'configs/orbslam3_lamaria/suite_20261002.json').read_text())
    experiment = Path(__file__).resolve().parents[2] / 'experiments'
    results = {'method': 'Independent image-only0.5s tracks joined by native pixel coordinates to saved live feature/mapID exports; no pose/geometry fit used for correspondence selection.',
               'limitations': ['MapID replacement/merging can change IDs without losing the physical landmark. Green means a current association, not verified optimizer inlier.',
                              'Conditional on image tracks surviving0.5s; this cannot explain image features that disappear or fail descriptor/KLT checks.',
                              'Stratification by image radius does not control scene depth, texture, blur or viewpoint.'], 'rows': [], 'sources': []}
    for label in ('short', 'medium'):
        name = f'lamaria_observations_{label}_20261002_{label}_full'
        run = experiment/f'lamaria_observations_{label}_20261002/runs'/name
        export = run/f'kp_{name}.csv'
        results['sources'].append({'path': str(export), 'bytes': export.stat().st_size, 'mtime_ns': export.stat().st_mtime_ns})
        seq = suite['sequences'][label]
        windows = [(window, 11) for window in audit.WINDOWS[label]]
        if label == 'short': windows.append((804.6, 2))
        for window, frames in windows:
            example_images = []
            for cam in (0, 1):
                raw = audit.associations(seq, cam, window, label, out, frames)
                stamps = [int(raw['stamps'][0]), int(raw['stamps'][-1])]
                runtime = nearby_rows(export, stamps[0]*1e-9-1e-6, stamps[1]*1e-9+1e-6)
                joined = []
                for index, stamp in enumerate(stamps):
                    rows = runtime[(stamp, cam)]
                    distance, match = cKDTree(rows[:, :2]).query(raw[f'xy{index}'])
                    assert len(distance) == 0 or distance.max() < .009, (label, cam, window, distance.max())
                    joined.append(rows[match, 2:4])
                start, end = joined
                initial = (start[:, 0] >= 0) & (start[:, 1] > 0)
                final = (end[:, 0] >= 0) & (end[:, 1] > 0)
                same = initial & final & (start[:, 0] == end[:, 0])
                p = audit.parameters(seq['config'], cam)
                radius = np.linalg.norm(raw['xy1']-p[2:4], axis=1)
                record = {'sequence': label, 'cam': cam, 'window': window, 'start_s': stamps[0]*1e-9, 'end_s': stamps[1]*1e-9,
                          'frame_count': frames,
                          'detections_first': int(raw['counts'][0]), 'detections_last': int(raw['counts'][1]),
                          'descriptor_proposals': int(raw['counts'][2]), 'image_verified_matches': int(raw['counts'][3]),
                          'initially_mapped': int(initial.sum()), 'end_mapped': int(final.sum()), 'same_mapID': int(same.sum()),
                          'mapped_to_unmapped': int((initial & ~final).sum()), 'unmapped_to_mapped': int((~initial & final).sum()),
                          'both_unmapped': int((~initial & ~final).sum()), 'bins': []}
                for lo, hi in ((0, 200), (200, 250), (250, 330)):
                    take = (radius >= lo) & (radius < hi)
                    record['bins'].append({'radius_range': [lo, hi], 'matched': int(take.sum()),
                              'initially_mapped': int((take & initial).sum()), 'same_mapID': int((take & same).sum()),
                              'mapped_to_unmapped': int((take & initial & ~final).sum()), 'both_unmapped': int((take & ~initial & ~final).sum())})
                results['rows'].append(record)
                np.savez_compressed(out/f'{label}_c{cam}_{window}_runtime.npz', xy0=raw['xy0'], xy1=raw['xy1'], runtime0=start, runtime1=end)
                if window in (804.6, 804.65, 850):
                    selected = np.arange(len(initial))[::max(1, len(initial)//45)]
                    colors = np.where(same, 'lime', np.where(initial & ~final, 'orange', np.where(final, 'cyan', 'red')))
                    for k, stamp in enumerate(stamps):
                        image = cv2.imread(str(Path(seq['dataset'])/f'mav0/cam{cam}/data/{stamp}.png'), 0)
                        example_images.append((image, raw[f'xy{k}'][selected], colors[selected], f'cam{cam}, {stamp*1e-9:.3f}s'))
            if example_images:
                fig, axs = plt.subplots(2, 2, figsize=(12, 9))
                for ax, (image, points, colors, title) in zip(axs.ravel(), example_images):
                    ax.imshow(image, cmap='gray'); ax.scatter(points[:, 0], points[:, 1], c=colors, s=16)
                    ax.set_title(title); ax.axis('off')
                context = 'doorway' if label == 'short' else 'tracking-loss context'
                duration = '0.05 s' if frames == 2 else '0.5 s'
                fig.suptitle(f'{label} {context}: image-verified {duration} tracks; native image orientation\nGreen=same mapID, orange=mapped→unmapped, cyan=end mapped, red=both unmapped')
                suffix = '_adjacent' if frames == 2 else ''
                fig.tight_layout(rect=(0, 0, 1, .95)); fig.savefig(out/f'{label}_doorway_runtime{suffix}.png', dpi=150); plt.close(fig)
    (out/'runtime_persistence.json').write_text(json.dumps(results, indent=2)+'\n')
    summary = json.loads((out/'summary.json').read_text())
    validation = json.loads((out/'validation/validation.json').read_text())
    render_residuals(out, summary)
    html = (out/'report.html').read_text()
    # Keep this augmentation idempotent when rerendering cached measurements.
    if '<h2>Coverage and interpretation</h2>' in html:
        begin = html.index('<h2>Coverage and interpretation</h2>')
        end = html.index('<h2>short_c0</h2>', begin)
        html = html[:begin]+html[end:]
    html = html.replace('other tracks at four disjoint later windows', 'other tracks at disjoint later windows')
    note = '<h2>Coverage and interpretation</h2><p><strong>The sampled Short data do not support a large general peripheral-calibration error.</strong> Fixed intrinsics give median held-out residuals around0.15px, with about98% below1px. A learned focal/radial correction barely changes this. This is a one-dimensional epipolar check, not a proof of correct depth, scale or complete calibration.</p>'
    note += '<p>Medium is less conclusive: only47 held-out tracks (7 outer) in cam0 and128 (16 outer) in cam1. Correction helps cam0 modestly but harms cam1. At850s both cameras have almost no0.5s image-verified correspondences, so this audit cannot estimate lens accuracy there. These omissions remain explicit below.</p>'
    note += '<p>Positive control passed: an injected+3% focal/+0.015 radial error in official-SDK synthetic image pairs was recovered to+0.102% focal/+0.000070 radial; held-out residual improved0.208→0.055px (true calibration0.055px). This checks that the fitting procedure can detect a calibration error under informative motion.</p>'
    note += '<p>Production matches the official SDK to less than 5e-8 radians throughout its declared valid domain. Invalid extreme grid corners can trigger the existing inverse fallback. Two actual Medium cam1 features (at119.987s and499.987s, radius about362px, beyond the declared330px radius) produced about122px inverse round-trip errors. One was image-trackable but excluded by this diagnostic\'s valid-domain gate. This is a real invalid-domain handling edge case, not evidence that physically valid lens bearings are generally wrong. No such errors occurred in the sampled Short frames or sampled doorway frames.</p>'
    note += '<p>At the exact Short doorway interval 804.642–805.142s, cam0 provides 66 image-verified matches and cam1 provides 30. Independent held-out fixed-calibration residuals are 0.109px (33 tracks) and 0.240px (15 tracks). Cam0 has 10 held-out peripheral tracks with median 0.148px and all below 1px; cam1 has no peripheral held-out coverage there. The latest observation candidate changes from 17 visual inliers to coasting with 5 during this interval, in the same map/world gauge.</p>'
    note += '<p>In the adjacent 804.592–804.642s pair, 305/457 image tracks survive but only 5/8 start with a map ID. All 238 matched features at radius 250–330px started unmapped; the four mapped-to-unmapped losses occur inside radius 200px. This does not show a general peripheral loss mechanism. It shows that most detectable, trackable features never carried a landmark into this comparison.</p><h2>Doorway image evidence and actual map IDs</h2>'
    note += '<p>Visual inspection: the Short doorway contains nearby doorframes, a large viewpoint change, and people occluding much of cam1. Image-only KLT can follow moving people; these are not guaranteed static scene points. Robust pose fitting and held-out checks reduce sensitivity to outliers but do not prove that every matched point belongs to the static scene. In a healthy view, visible static mapped corners should retain associations; red here means unmapped at both endpoints, not a calibration outlier.</p>'
    for label, window in [('short', 804.6), ('short', 804.65), ('medium', 850)]:
        duration = '0.05 s' if window == 804.6 else '0.5 s'
        note += f'<h3>{label}: {window}s, {duration} separation</h3><table><tr><th>Camera</th><th>Detections first/last</th><th>Descriptor proposals</th><th>Image-verified</th><th>Initially mapped</th><th>Same mapID</th><th>Mapped→unmapped</th><th>Both unmapped</th></tr>'
        for r in results['rows']:
            if r['sequence'] == label and r['window'] == window:
                note += f'<tr><td>{r["cam"]}</td><td>{r["detections_first"]}/{r["detections_last"]}</td><td>{r["descriptor_proposals"]}</td><td>{r["image_verified_matches"]}</td><td>{r["initially_mapped"]}</td><td>{r["same_mapID"]}</td><td>{r["mapped_to_unmapped"]}</td><td>{r["both_unmapped"]}</td></tr>'
        note += '</table>'
        suffix = '_adjacent' if window == 804.6 else ''
        note += f'<img src="{label}_doorway_runtime{suffix}.png">'
    note += '<p>The illustrated map IDs come from the latest observation-bookkeeping candidate, not a new SLAM run. Do not interpret different point IDs as proof of physical loss: fusion/replacement can change them. Pose/point errors and missing landmark creation remain plausible.</p>'
    html = html.replace('<h2>short_c0</h2>', note+'<h2>short_c0</h2>')
    (out/'report.html').write_text(html)
    print(json.dumps([r for r in results['rows'] if r['window'] in (803, 804.6, 804.65, 850)], indent=2), flush=True)


if __name__ == '__main__': main()
