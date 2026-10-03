#!/usr/bin/env python3
"""Cross-validated native-Fisheye624 calibration diagnostic; never edits SLAM.

Uses production-linked bearings, independent image associations, and fitted
relative poses. No ground-truth poses, exported map coordinates, or SLAM poses
enter fitting. Small lens changes learned on disjoint time windows are evaluated
with relative pose re-estimated on training tracks and errors on held-out tracks.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import struct
import subprocess
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix
from projectaria_tools.core import calibration, sophus

PROJECT = Path(__file__).resolve().parents[2]
NAMES = ['fx', 'fy', 'cx', 'cy'] + [f'k{i}' for i in range(1, 7)] + ['p1', 'p2', 's1', 's2', 's3', 's4']
WINDOWS = {'short': [110, 450, 650, 700, 770, 795, 803, 804.65, 825],
           'medium': [120, 400, 500, 700, 820, 839, 850, 863]}
cv2.setNumThreads(2)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Production:
    def __init__(self, build, out):
        self.build, self.out = build, out
        self.mounts = ['docker', 'run', '--rm', '--network', 'none', '--user', f'{os.getuid()}:{os.getgid()}',
                       '-v', f'{out}:/audit', '-v', f'{PROJECT}/pipeline/run/tests:/tests:ro',
                       '-v', f'{build}:/build:ro', '-v', f'{PROJECT}/third_party/ORB_SLAM3:/orb:ro',
                       '-v', f'{PROJECT}/third_party/ELSED:/elsed:ro']
        compiler = self.mounts + ['--entrypoint', '/usr/bin/c++', 'insv/orbslam3:gdb']
        flags = ['-O2', '-std=c++11', '-DCOMPILEDWITHC11', '-I/elsed/src', '-I/build/sources',
                 '-I/build/sources/include', '-I/build/sources/include/CameraModels', '-I/orb',
                 '-I/orb/Thirdparty/Sophus', '-isystem', '/usr/include/eigen3', '-isystem', '/usr/local/include/opencv4']
        link = shlex.split((PROJECT/'third_party/ORB_SLAM3/build/CMakeFiles/mono_rig_euroc.dir/link.txt').read_text())[1:]
        for i, value in enumerate(link):
            if value.startswith('CMakeFiles/'): link[i] = '/audit/bearings.o'
            elif value == '../Examples/Monocular-Inertial/mono_rig_euroc': link[i] = '/audit/bearings'
            elif value == '../lib/libORB_SLAM3.so': link[i] = '/build/lib/libORB_SLAM3.so'
            elif value.startswith('../Thirdparty/'): link[i] = '/orb/' + value[3:]
        if not (out/'bearings').exists():
            with (out/'build.log').open('w') as log:
                subprocess.run(compiler+flags+['-c', '/tests/fisheye_bearing_server.cpp', '-o', '/audit/bearings.o'], stdout=log, stderr=subprocess.STDOUT, check=True)
                subprocess.run(compiler+link, stdout=log, stderr=subprocess.STDOUT, check=True)
        self.process = subprocess.Popen(self.mounts+['-i', '-e', 'LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib',
                 '--entrypoint', '/audit/bearings', 'insv/orbslam3:gdb'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=(out/'server.log').open('w'))
        self.calls = 0

    def evaluate(self, p, xy):
        xy = np.asarray(xy, '<f8').reshape(-1, 2)
        if not len(xy): return np.empty((0, 5))
        self.process.stdin.write(struct.pack('<I', len(xy)) + np.asarray(p, '<f8').tobytes() + xy.tobytes())
        self.process.stdin.flush()
        n = 40 * len(xy)
        buffer = self.process.stdout.read(n)
        if len(buffer) != n: raise RuntimeError('Production server terminated')
        self.calls += 1
        return np.frombuffer(buffer, '<f8').reshape(-1, 5).copy()

    def rays_jacobian(self, p, xy):
        xy = np.asarray(xy)
        delta = .1
        all_xy = np.concatenate([xy, xy+[delta, 0], xy-[delta, 0], xy+[0, delta], xy-[0, delta]])
        evaluated = self.evaluate(p, all_xy).reshape(5, len(xy), 5)
        rays = evaluated[0, :, :3]
        jac = np.stack([(evaluated[1, :, :3]-evaluated[2, :, :3])/(2*delta),
                        (evaluated[3, :, :3]-evaluated[4, :, :3])/(2*delta)], axis=2)
        return rays, jac

    def close(self):
        self.process.stdin.write(struct.pack('<I', 0)); self.process.stdin.flush()
        self.process.wait(timeout=10)


def parameters(path, cam):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    p = np.asarray([fs.getNode(f'Camera{cam+1}.{name}').real() for name in NAMES], np.float32).astype(float)
    fs.release()
    return p


def load_features(folder, stamp):
    data = (Path(folder)/f'{stamp}.kp').read_bytes()
    count = int(np.frombuffer(data, '<i4', 1)[0])
    assert len(data) == 4+44*count
    return np.frombuffer(data, '<f4', 2*count, 4).reshape(-1, 2).copy(), np.frombuffer(data, np.uint8, 32*count, 4+12*count).reshape(-1, 32).copy()


def associations(sequence, cam, window, label, out, frames=11):
    suffix = '' if frames == 11 else f'_f{frames}'
    cached = out/f'{label}_c{cam}_{window}{suffix}_pairs.npz'
    if cached.exists(): return dict(np.load(cached))
    stamps = np.loadtxt(sequence['supported_timestamps'], dtype=np.int64)
    first = int(np.argmin(abs(stamps*1e-9-window)))
    selected = stamps[first:first+frames]
    xy0, d0 = load_features(sequence[f'kp{cam}'], selected[0])
    xy1, d1 = load_features(sequence[f'kp{cam}'], selected[-1])
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = bf.knnMatch(d0, d1, k=2); reverse = bf.match(d1, d0)
    matches = [(a.queryIdx, a.trainIdx) for a, b in forward if a.distance <= 70 and a.distance < .8*b.distance and reverse[a.trainIdx].trainIdx == a.queryIdx]
    pairs = np.asarray(matches, dtype=int).reshape(-1, 2)
    images = [cv2.imread(str(Path(sequence['dataset'])/f'mav0/cam{cam}/data/{s}.png'), 0) for s in selected]
    assert all(im is not None for im in images)
    points = xy0[pairs[:, 0]].reshape(-1, 1, 2).copy()
    good = np.ones(len(points), bool)
    traces = [points[:, 0].copy()]
    kwargs = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 30, .01), minEigThreshold=1e-4)
    for a, b in zip(images[:-1], images[1:]):
        pred, ok, err = cv2.calcOpticalFlowPyrLK(a, b, points, None, **kwargs)
        pred = np.nan_to_num(pred)
        back, back_ok, _ = cv2.calcOpticalFlowPyrLK(b, a, pred, None, **kwargs)
        good &= ok.ravel().astype(bool) & back_ok.ravel().astype(bool)
        good &= np.linalg.norm(back[:, 0]-points[:, 0], axis=1) < .75
        good &= err.ravel() < 20
        good &= (pred[:, 0, 0] >= 10) & (pred[:, 0, 0] < 630) & (pred[:, 0, 1] >= 10) & (pred[:, 0, 1] < 470)
        points = pred
        traces.append(points[:, 0].copy())
    good &= np.linalg.norm(points[:, 0]-xy1[pairs[:, 1]], axis=1) <= 2
    raw = dict(xy0=xy0[pairs[good, 0]], xy1=xy1[pairs[good, 1]], indices=pairs[good],
               traces=np.asarray(traces)[:, good], stamps=selected,
               all_xy0=xy0, counts=np.asarray([len(xy0), len(xy1), len(pairs), int(good.sum())]))
    np.savez_compressed(cached, **raw)
    print('matches', label, cam, window, raw['counts'].tolist(), flush=True)
    return raw


def skew(t):
    x, y, z = t
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def pose_pack(R, t):
    t = t.ravel()/np.linalg.norm(t)
    return np.r_[Rotation.from_matrix(R).as_rotvec(), np.arctan2(t[1], t[0]), np.arcsin(t[2])]


def pose_unpack(x):
    R = Rotation.from_rotvec(x[:3]).as_matrix()
    t = np.array([np.cos(x[4])*np.cos(x[3]), np.cos(x[4])*np.sin(x[3]), np.sin(x[4])])
    return R, t


def residual(pose, rays0, rays1, jac0, jac1, vectors=False):
    R, t = pose_unpack(pose)
    E = skew(t)@R
    n1 = rays0@E.T
    n0 = rays1@E
    error = np.einsum('ni,ni->n', rays1, n1)
    g0 = np.einsum('ni,nij->nj', n0, jac0)
    g1 = np.einsum('ni,nij->nj', n1, jac1)
    denom = np.sqrt(np.sum(g0*g0+g1*g1, axis=1)).clip(1e-9)
    if vectors:
        return error/denom, error[:, None]*g1/(np.sum(g1*g1, axis=1).clip(1e-9)[:, None])
    return error/denom


def initialize_pose(raw, p, production):
    if len(raw['xy0']) < 30: return None
    b0, j0 = production.rays_jacobian(p, raw['xy0'])
    b1, j1 = production.rays_jacobian(p, raw['xy1'])
    domain = (np.linalg.norm(raw['xy0']-p[2:4], axis=1) < 330) & (np.linalg.norm(raw['xy1']-p[2:4], axis=1) < 330)
    domain &= (b0[:, 2] > np.cos(1.4)) & (b1[:, 2] > np.cos(1.4))
    domain &= np.linalg.norm(production.evaluate(p, raw['xy0'])[:, 3:]-raw['xy0'], axis=1) < .01
    domain &= np.linalg.norm(production.evaluate(p, raw['xy1'])[:, 3:]-raw['xy1'], axis=1) < .01
    selected = np.flatnonzero(domain)
    # Stable random split is independent of calibration and image residual.
    rng = np.random.default_rng(int(raw['stamps'][0] % 2147483647))
    rng.shuffle(selected)
    selected = selected[:400]
    train = selected[:len(selected)//2]; test = selected[len(selected)//2:]
    if len(train) < 15: return None
    cv2.setRNGSeed(42)
    a = b0[train, :2]/b0[train, 2, None]; b = b1[train, :2]/b1[train, 2, None]
    E, mask = cv2.findEssentialMat(a, b, np.eye(3), method=cv2.RANSAC, prob=.999, threshold=.003)
    if E is None: return None
    _, R, t, _ = cv2.recoverPose(E[:3], a, b, np.eye(3), mask=mask)
    pose = pose_pack(R, t)
    fitted = least_squares(lambda x: residual(x, b0[train], b1[train], j0[train], j1[train]), pose, loss='soft_l1', f_scale=1., max_nfev=120)
    R, t = pose_unpack(fitted.x)
    rotated = b0@R.T
    angles = np.degrees(np.arccos(np.einsum('ni,ni->n', rotated, b1).clip(-1, 1)))
    all_evaluated = production.evaluate(p, raw['all_xy0'])
    all_roundtrip = np.linalg.norm(all_evaluated[:, 3:]-raw['all_xy0'], axis=1)
    all_angles = np.degrees(np.arccos(all_evaluated[:, 2].clip(-1, 1)))
    all_radius = np.linalg.norm(raw['all_xy0']-p[2:4], axis=1)
    return dict(raw=raw, train=train, test=test, pose=fitted.x, parallax_median_deg=float(np.median(angles[selected])),
                all_feature_count=len(raw['all_xy0']), all_feature_roundtrip_gt1px=int((all_roundtrip > 1).sum()),
                all_feature_outside_declared_domain=int(((all_radius >= 330) | (all_angles > np.degrees(1.4))).sum()),
                ransac_inliers=int(mask.sum()), domain_count=int(domain.sum()), pair_count=len(b0))


def changed_parameters(p, correction):
    result = p.copy()
    # Shift dimensionless variables away from zero so relative finite
    # differences survive the production model's float storage.
    result[:2] *= np.exp(.01*(correction[0]-1))
    result[4] += .005*(correction[1]-1)
    return result


def fit_shared(pairs, base, production):
    learned = pairs[:4]
    x0 = np.r_[np.ones(2), np.concatenate([pair['pose'] for pair in learned])]
    xy = np.concatenate([pair['raw'][key][pair['train']] for pair in learned for key in ('xy0', 'xy1')])
    cache = {'key': None}
    def evaluate(x):
        key = tuple(changed_parameters(base, x[:2]).astype(np.float32))
        if cache['key'] != key:
            cache['rays'], cache['jac'] = production.rays_jacobian(np.asarray(key), xy)
            cache['key'] = key
        pos = 0; results = []
        for i, pair in enumerate(learned):
            n = len(pair['train']); b = cache['rays'][pos:pos+2*n]; j = cache['jac'][pos:pos+2*n]
            results.append(residual(x[2+5*i:7+5*i], b[:n], b[n:], j[:n], j[n:]))
            pos += 2*n
        # Explicit, modest physical priors: focal sigma3%, k1 sigma0.015.
        return np.r_[np.concatenate(results), (x[:2]-1)/3]
    m = sum(len(pair['train']) for pair in learned)
    sparsity = lil_matrix((m+2, len(x0)), dtype=int)
    start = 0
    for i, pair in enumerate(learned):
        n = len(pair['train']); sparsity[start:start+n, :2] = 1; sparsity[start:start+n, 2+5*i:7+5*i] = 1; start += n
    sparsity[m:, :2] = 1
    bounds = (np.r_[[-4., -3.], np.full(len(x0)-2, -np.inf)], np.r_[[6., 5.], np.full(len(x0)-2, np.inf)])
    # Explicit derivative step because production camera parameters/rays are float.
    fit = least_squares(evaluate, x0, bounds=bounds, loss='soft_l1', f_scale=1., jac_sparsity=sparsity,
                        diff_step=1e-3, max_nfev=120, ftol=1e-7, xtol=1e-7, gtol=1e-7)
    return fit


def stats(values):
    a = np.asarray(values); a = a[np.isfinite(a)]
    return {'n': int(len(a)), 'median_px': float(np.median(abs(a))), 'p90_px': float(np.percentile(abs(a), 90)),
            'fraction_below_1px': float(np.mean(abs(a) < 1)), 'fraction_below_2px': float(np.mean(abs(a) < 2))} if len(a) else {'n': 0}


def grid_check(production, p, out, name):
    xy = np.asarray([(u, v) for v in range(0, 480, 8) for u in range(0, 640, 8)], float)
    evaluated = production.evaluate(p, xy)
    sdk = calibration.CameraCalibration('audit', calibration.CameraModelType.FISHEYE624, np.r_[p[0], p[2:]],
                                        sophus.SE3(), 640, 480, 330., 1.4, '')
    sdk_rays = np.full((len(xy), 3), np.nan)
    for i, pixel in enumerate(xy):
        adjusted = [pixel[0], p[3] + (pixel[1]-p[3])*p[0]/p[1]]
        ray = sdk.unproject(adjusted)
        if ray is not None: sdk_rays[i] = ray/np.linalg.norm(ray)
    valid = np.isfinite(sdk_rays).all(axis=1)
    prod_rays = evaluated[:, :3]/np.linalg.norm(evaluated[:, :3], axis=1)[:, None]
    angle = np.arctan2(np.linalg.norm(np.cross(prod_rays, sdk_rays), axis=1), np.sum(prod_rays*sdk_rays, axis=1))
    error = np.linalg.norm(evaluated[:, 3:]-xy, axis=1)
    np.savez_compressed(out/f'{name}_grid.npz', xy=xy, production=evaluated, sdk_rays=sdk_rays, valid=valid, roundtrip=error)
    return dict(grid_points=len(xy), valid=int(valid.sum()), max_sdk_bearing_difference_rad=float(np.nanmax(angle)),
                max_valid_roundtrip_px=float(np.max(error[valid])), invalid_domain_gt1px=int(np.sum(~valid & (error > 1))),
                max_valid_angle_deg=float(np.degrees(np.arccos(prod_rays[valid, 2])).max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--build', type=Path, default=PROJECT/'build/orbslam3_lamaria_observations')
    args = parser.parse_args(); out = args.out.resolve(); out.mkdir(parents=True, exist_ok=True)
    suite = json.loads((PROJECT/'configs/orbslam3_lamaria/suite_20261002.json').read_text())
    production = Production(args.build.resolve(), out)
    summary = {'method': __doc__, 'production_build': str(args.build.resolve()), 'library_sha256': sha(args.build/'lib/libORB_SLAM3.so'),
               'script_sha256': sha(__file__), 'cpp_sha256': sha(PROJECT/'pipeline/run/tests/fisheye_bearing_server.cpp'),
               'GT_used': False, 'estimator_modified': False, 'matching_bounds_changed': False, 'groups': [],
               'limitations': ['Epipolar residual measures a one-dimensional geometric constraint, not full3D reprojection or scale.',
                    'Relative pose is re-fitted for each calibration on one half of each pair; only the other half measures generalization.',
                    'Pure rotation, low parallax, planar scenes and short baselines weaken lens observability.',
                    'Passing this diagnostic cannot prove hardware calibration perfect. ALIKED/KLT tracks can still contain correlated errors.',
                    'Model correction only includes focal scale and first radial coefficient; other intrinsic/extrinsic defects are not tested.']}
    panels = []
    for label, seq in suite['sequences'].items():
        if label not in WINDOWS: continue
        for cam in (0, 1):
            name = f'{label}_c{cam}'
            base = parameters(seq['config'], cam)
            group = {'sequence': label, 'cam': cam, 'parameters': base.tolist(), 'grid': grid_check(production, base, out, name)}
            pairs = []
            skipped = []
            for window in WINDOWS[label]:
                raw = associations(seq, cam, window, label, out)
                pair = initialize_pose(raw, base, production)
                if pair is None:
                    all_evaluated = production.evaluate(base, raw['all_xy0'])
                    roundtrip = np.linalg.norm(all_evaluated[:, 3:]-raw['all_xy0'], axis=1)
                    skipped.append({'window': window, 'counts': raw['counts'].tolist(),
                                    'all_feature_roundtrip_gt1px': int((roundtrip > 1).sum()),
                                    'reason': 'Fewer than30 image-verified valid-domain correspondences before any geometric fit'})
                    print('skip insufficient geometry', name, window, flush=True); continue
                pair['window'] = window; pairs.append(pair)
            group['skipped_pairs'] = skipped
            if len(pairs) < 5 or [p['window'] for p in pairs[:4]] != WINDOWS[label][:4]:
                raise RuntimeError(f'Insufficient prescribed training or held-out windows: {name}, got {[p["window"] for p in pairs]}')
            fit = fit_shared(pairs, base, production)
            adjusted = changed_parameters(base, fit.x[:2])
            group.update({'focal_change_percent': float(100*(adjusted[0]/base[0]-1)), 'k1_change': float(adjusted[4]-base[4]),
                          'fit_success': bool(fit.success), 'fit_message': fit.message, 'fit_evaluations': fit.nfev, 'pairs': []})
            arrays = {}
            for index, pair in enumerate(pairs):
                raw = pair['raw']; train, test = pair['train'], pair['test']
                result = {'window': pair['window'], 'training_window': index < 4, 'counts': raw['counts'].tolist(),
                          'domain_count': pair['domain_count'], 'train_tracks': len(train), 'heldout_tracks': len(test),
                          'all_feature_count': pair['all_feature_count'],
                          'all_feature_roundtrip_gt1px': pair['all_feature_roundtrip_gt1px'],
                          'all_feature_outside_declared_domain': pair['all_feature_outside_declared_domain'],
                          'parallax_median_deg': pair['parallax_median_deg']}
                for model, params in [('fixed', base), ('adjusted', adjusted)]:
                    b0, j0 = production.rays_jacobian(params, raw['xy0']); b1, j1 = production.rays_jacobian(params, raw['xy1'])
                    refined = least_squares(lambda x: residual(x, b0[train], b1[train], j0[train], j1[train]), pair['pose'],
                                            loss='soft_l1', f_scale=1., max_nfev=200)
                    errors, vectors = residual(refined.x, b0[test], b1[test], j0[test], j1[test], vectors=True)
                    radius = np.linalg.norm(raw['xy1'][test]-base[2:4], axis=1)
                    result[model] = {'all': stats(errors), 'inner_r_lt200': stats(errors[radius < 200]),
                                     'outer_r_ge250': stats(errors[radius >= 250]), 'pose_fit_success': bool(refined.success)}
                    arrays[f'w{pair["window"]}_{model}_errors'] = errors
                    arrays[f'w{pair["window"]}_{model}_vectors'] = vectors
                    arrays[f'w{pair["window"]}_xy'] = raw['xy1'][test]
                    arrays[f'w{pair["window"]}_radius'] = radius
                group['pairs'].append(result)
                print(name, pair['window'], 'heldout median fixed/adjusted', result['fixed']['all']['median_px'], result['adjusted']['all']['median_px'], flush=True)
            for subset, indices in [('training_times', range(4)), ('heldout_times', range(4, len(pairs)))]:
                group[subset] = {}
                for model in ('fixed', 'adjusted'):
                    errors = np.concatenate([arrays[f'w{pairs[i]["window"]}_{model}_errors'] for i in indices])
                    radii = np.concatenate([arrays[f'w{pairs[i]["window"]}_radius'] for i in indices])
                    group[subset][model] = {'all': stats(errors), 'inner_r_lt200': stats(errors[radii < 200]), 'outer_r_ge250': stats(errors[radii >= 250])}
            np.savez_compressed(out/f'{name}_results.npz', **arrays)
            (out/f'{name}_summary.json').write_text(json.dumps(group, indent=2)+'\n')
            summary['groups'].append(group)
            fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
            for i, model in enumerate(('fixed', 'adjusted')):
                xy = np.concatenate([arrays[f'w{p["window"]}_xy'] for p in pairs[4:]])
                error = np.concatenate([arrays[f'w{p["window"]}_{model}_errors'] for p in pairs[4:]])
                vectors = np.concatenate([arrays[f'w{p["window"]}_{model}_vectors'] for p in pairs[4:]])
                scatter = axs[i].scatter(xy[:, 0], xy[:, 1], c=np.minimum(abs(error), 3), s=8, cmap='magma', vmin=0, vmax=3)
                for y in range(40, 480, 80):
                    for x in range(40, 640, 80):
                        take = (abs(xy[:, 0]-x) < 40) & (abs(xy[:, 1]-y) < 40) & (abs(error) < 5)
                        if take.sum() >= 5:
                            v = np.median(vectors[take], axis=0)
                            axs[i].arrow(x, y, 15*v[0], 15*v[1], color='cyan', head_width=4)
                axs[i].set(xlim=(0, 640), ylim=(480, 0), title=f'{model}: held-out times + tracks', aspect='equal')
                fig.colorbar(scatter, ax=axs[i], label='Absolute epipolar residual (px)', shrink=.7)
            for model in ('fixed', 'adjusted'):
                errors = np.concatenate([arrays[f'w{p["window"]}_{model}_errors'] for p in pairs[4:]])
                radius = np.concatenate([arrays[f'w{p["window"]}_radius'] for p in pairs[4:]])
                bins = [(0, 150), (150, 200), (200, 250), (250, 290), (290, 330)]
                axs[2].plot([(a+b)/2 for a, b in bins], [np.median(abs(errors[(radius >= a)&(radius < b)])) if np.any((radius >= a)&(radius < b)) else np.nan for a, b in bins], 'o-', label=model)
            axs[2].set(xlabel='Radius from principal point (px)', ylabel='Median held-out residual (px)', ylim=(0, 1)); axs[2].legend(); axs[2].grid(alpha=.2)
            fig.suptitle(f'{name}: focal {group["focal_change_percent"]:+.2f}%, k1 {group["k1_change"]:+.5f}; cyan vectors ×15')
            fig.tight_layout(); fig.savefig(out/f'{name}_residuals.png', dpi=150); plt.close(fig)
            # Native image examples: exact same image-only tracks, no poses/map involved.
            fig, axs = plt.subplots(2, 2, figsize=(12, 9))
            for row, pair in enumerate((pairs[4], pairs[-1])):
                raw = pair['raw']; chosen = pair['test'][::max(1, len(pair['test'])//50)]
                for col, frame in enumerate((0, -1)):
                    stamp = int(raw['stamps'][frame]); image = cv2.imread(str(Path(seq['dataset'])/f'mav0/cam{cam}/data/{stamp}.png'), 0)
                    axs[row, col].imshow(image, cmap='gray')
                    xy = raw['xy0' if frame == 0 else 'xy1'][chosen]
                    axs[row, col].scatter(xy[:, 0], xy[:, 1], s=20, facecolors='none', edgecolors='lime')
                    for point, number in zip(xy, range(len(chosen))): axs[row, col].text(*point, str(number), color='yellow', fontsize=6)
                    axs[row, col].set_title(f'{name} {stamp*1e-9:.3f}s; matched IDs'); axs[row, col].axis('off')
            fig.tight_layout(); fig.savefig(out/f'{name}_tracks.png', dpi=150); plt.close(fig)
            panels.append(name)
            (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    summary['production_calls'] = production.calls
    production.close()
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    html = '<!doctype html><meta charset="utf-8"><title>LaMAria intrinsics audit</title><style>body{font:16px sans-serif;background:#171c24;color:#ddd;margin:30px;max-width:1600px}img{width:100%;margin:15px 0}table{border-collapse:collapse}td,th{padding:10px;border:1px solid #555}</style><h1>Does lens calibration explain peripheral tracking loss?</h1>'
    html += '<p>Offline diagnostic only. Both lenses, Short and Medium. Production Fisheye624 bearings; image-only ALIKED/Hamming + 10-step forward/back KLT. No GT, SLAM poses or landmark coordinates enter the fit. Green circles below mean image correspondences, not mapped SLAM points.</p>'
    html += '<p>Focal and first radial correction learned on four earlier windows. Each calibration has relative pose re-fitted on half the tracks in each pair; other tracks at four disjoint later windows measure generalization. Cyan arrows show median epipolar-normal error multiplied by15. A healthy model should show small, directionally unsystematic errors across the valid image.</p><table><tr><th>Group</th><th>Focal correction</th><th>Held-out median fixed→adjusted</th><th>Outer median fixed→adjusted</th></tr>'
    for g in summary['groups']:
        a, b = g['heldout_times']['fixed'], g['heldout_times']['adjusted']
        html += f'<tr><td>{g["sequence"]} cam{g["cam"]}</td><td>{g["focal_change_percent"]:+.2f}%</td><td>{a["all"]["median_px"]:.3f} → {b["all"]["median_px"]:.3f}px</td><td>{a["outer_r_ge250"].get("median_px",float("nan")):.3f} → {b["outer_r_ge250"].get("median_px",float("nan")):.3f}px</td></tr>'
    html += '</table><p>Interpretation: consistent held-out improvement supports a calibration defect; training-only improvement does not. Epipolar consistency cannot establish metric scale or full3D correctness. This test does not claim intrinsic perfection and does not isolate every possible lens or rig error.</p>'
    for name in panels: html += f'<h2>{name}</h2><img src="{name}_residuals.png"><img src="{name}_tracks.png">'
    html += '<p>Machine-readable summary: summary.json. Saved per-track arrays and original feature IDs make each result reproducible.</p>'
    (out/'report.html').write_text(html)
    print('COMPLETE', out, flush=True)


if __name__ == '__main__': main()
