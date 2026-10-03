#!/usr/bin/env python3
"""Positive-control validation for the offline intrinsics diagnostic."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from projectaria_tools.core import calibration, sophus
import audit_lamaria_intrinsics as audit


def evaluate(pairs, parameters, production):
    errors = []
    for pair in pairs:
        a, ja = production.rays_jacobian(parameters, pair['raw']['xy0'])
        b, jb = production.rays_jacobian(parameters, pair['raw']['xy1'])
        train, test = pair['train'], pair['test']
        fitted = least_squares(lambda x: audit.residual(x, a[train], b[train], ja[train], jb[train]), pair['pose'],
                               loss='soft_l1', f_scale=1, max_nfev=200)
        errors.extend(audit.residual(fitted.x, a[test], b[test], ja[test], jb[test]))
    return audit.stats(errors)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(); out = args.out.resolve(); out.mkdir(parents=True, exist_ok=True)
    suite = json.loads((audit.PROJECT/'configs/orbslam3_lamaria/suite_20261002.json').read_text())
    truth = audit.parameters(suite['sequences']['short']['config'], 0)
    wrong = truth.copy(); wrong[:2] *= 1.03; wrong[4] += .015
    production = audit.Production(audit.PROJECT/'build/orbslam3_lamaria_observations', out)
    sdk = calibration.CameraCalibration('synthetic', calibration.CameraModelType.FISHEYE624,
              np.r_[truth[0], truth[2:]], sophus.SE3(), 640, 480, 330., 1.4, '')
    rng = np.random.default_rng(42)
    pairs = []
    for k in range(8):
        pixel0 = rng.uniform([30, 30], [610, 450], (700, 2))
        R = Rotation.from_rotvec(rng.uniform(-.15, .15, 3)).as_matrix()
        t = rng.normal(size=3); t = t/np.linalg.norm(t)*.65
        a, b = [], []
        for pixel in pixel0:
            ray = sdk.unproject(pixel)
            if ray is None: continue
            X = ray/np.linalg.norm(ray)*rng.uniform(2, 12)
            projected = sdk.project(R@X+t)
            if projected is not None and np.all(projected >= [10, 10]) and np.all(projected < [630, 470]):
                a.append(pixel); b.append(projected)
        a = np.asarray(a)+rng.normal(0, .08, (len(a), 2)); b = np.asarray(b)+rng.normal(0, .08, (len(b), 2))
        raw = dict(xy0=a, xy1=b, all_xy0=a, counts=np.array([len(a)]*4), stamps=np.array([int((k+1)*1e9)]))
        pair = audit.initialize_pose(raw, wrong, production)
        assert pair is not None
        pair['window'] = k; pairs.append(pair)
    fit = audit.fit_shared(pairs, wrong, production)
    recovered = audit.changed_parameters(wrong, fit.x[:2])
    result = {'synthetic_definition': 'OfficialAriaSDK generated8 independent known-R/t nonplanar pointcloud pairs;0.08px independent pixel noise; first4 timewindows calibrationtrain, last4 heldout. Pose independently refitted on halftracks perpair.',
              'truth_parameters': truth.tolist(), 'injected_parameters': wrong.tolist(), 'recovered_parameters': recovered.tolist(),
              'focal_injected_error_percent': 3., 'k1_injected_error': .015,
              'focal_remaining_error_percent': float((recovered[0]/truth[0]-1)*100),
              'k1_remaining_error': float(recovered[4]-truth[4]),
              'wrong_heldout': evaluate(pairs[4:], wrong, production),
              'truth_heldout': evaluate(pairs[4:], truth, production),
              'recovered_heldout': evaluate(pairs[4:], recovered, production), 'fit_success': bool(fit.success), 'fit_nfev': fit.nfev}
    result['passed'] = abs(result['focal_remaining_error_percent']) < 1.5 and abs(result['k1_remaining_error']) < .0075 and result['recovered_heldout']['median_px'] < result['wrong_heldout']['median_px']*.8
    (out/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    production.close()
    print(json.dumps(result, indent=2), flush=True)
    if not result['passed']: raise SystemExit('Positive control did not meet its predefined accuracy/recovery criteria')


if __name__ == '__main__': main()
