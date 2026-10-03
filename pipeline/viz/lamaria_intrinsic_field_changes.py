#!/usr/bin/env python3
"""Visualize intrinsic-only projection changes using COLMAP's native camera model."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pycolmap


def camera(native):
    parameters = np.asarray(native, dtype=float).copy()
    parameters[2:4] += .5
    return pycolmap.Camera(model='RAD_TAN_THIN_PRISM_FISHEYE', width=640,
                          height=480, params=parameters)


def stats(values):
    return dict(count=len(values), median_px=float(np.median(values)),
                p95_px=float(np.percentile(values, 95)), maximum_px=float(values.max()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--v2', type=Path, required=True)
    parser.add_argument('--v3', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    a, b = [json.loads(path.read_text()) for path in (args.v2, args.v3)]
    args.output.mkdir(parents=True, exist_ok=True)
    if len(b['commits']) < 2:
        raise ValueError('The failed replay needs two commits to inspect its final update')
    fig, axes = plt.subplots(2, 4, figsize=(21, 8), constrained_layout=True)
    u, v = np.meshgrid(np.arange(4, 640, 8), np.arange(4, 480, 8))
    pixels = np.column_stack((u.ravel(), v.ravel()))
    results, panels = [], []
    for cam in (0, 1):
        native = np.asarray(a['factory_native_parameters'][cam])
        np.testing.assert_allclose(native, b['factory_native_parameters'][cam], atol=1e-12, rtol=0)
        factory = camera(native)
        tangent = factory.cam_from_img(pixels + .5)
        rays = np.column_stack((tangent, np.ones(len(tangent))))
        theta = np.arctan(np.linalg.norm(tangent, axis=1))
        # Use the frozen physical mask and angular domain from this experiment.
        valid = (np.linalg.norm(pixels-native[2:4], axis=1) <= 330) & (theta <= 1.4)
        valid &= np.isfinite(rays).all(axis=1)
        rays, sampled, theta = rays[valid], pixels[valid], theta[valid]
        projected = [factory.img_from_cam(rays) - .5]
        commits = [a['commits'][-1], b['commits'][-1], b['commits'][-2]]
        for commit in commits:
            projected.append(camera(commit['parameters'][str(cam)]).img_from_cam(rays) - .5)
        error = np.max(np.linalg.norm(projected[0] - sampled, axis=1))
        if error > 1e-6:
            raise ValueError(f'Native camera round trip failed: {error}')
        for col, (first, second, label) in enumerate([
                (0, 1, 'v2 final vs factory'), (0, 2, 'v3 before loss vs factory'),
                (1, 2, 'v3 vs v2'), (3, 2, 'v3 final accepted update')]):
            delta = projected[second] - projected[first]
            magnitude = np.linalg.norm(delta, axis=1)
            if not np.isfinite(magnitude).all():
                raise ValueError('Nonfinite projection change')
            result = dict(camera=cam, comparison=label, all=stats(magnitude),
                          central=stats(magnitude[theta <= .5]),
                          peripheral=stats(magnitude[theta >= 1.1]),
                          factory_roundtrip_max_px=float(error))
            results.append(result)
            panels.append((axes[cam, col], sampled, delta, magnitude, label, cam))
    vmax = max(float(panel[3].max()) for panel in panels)
    for axis, sampled, delta, magnitude, label, cam in panels:
        scatter = axis.scatter(sampled[:, 0], sampled[:, 1], c=magnitude,
                               s=14, marker='s', cmap='magma', vmin=0, vmax=vmax)
        take = np.arange(len(sampled))[::24]
        axis.quiver(sampled[take, 0], sampled[take, 1], delta[take, 0], delta[take, 1],
                    color='cyan', angles='xy', scale_units='xy', scale=.15, width=.002)
        axis.set(xlim=(0, 640), ylim=(480, 0), xlabel='Native u (px)', ylabel='Native v (px)',
                 title=f'cam{cam}: {label}')
        axis.set_aspect('equal')
    fig.colorbar(scatter, ax=axes, label='Projection change (pixels); arrows enlarged 6.67x')
    fig.suptitle('Same factory rays through accepted lens models: change, not ground-truth error')
    fig.savefig(args.output/'projection_changes.png', dpi=150)
    plt.close(fig)
    report = dict(v2_metrics=str(args.v2), v3_metrics=str(args.v3),
                  v2_commit_s=a['commits'][-1]['t'], v3_commit_s=b['commits'][-1]['t'],
                  model='Production pycolmap RAD_TAN_THIN_PRISM_FISHEYE; +0.5 pixel convention',
                  grid_step_px=8, physical_valid_radius_px=330, max_angle_rad=1.4,
                  central_angle_max_rad=.5, peripheral_angle_min_rad=1.1,
                  interpretation='Intrinsic-only projection displacement on identical rays. '
                  'This is not calibration error or evidence that a particular update caused tracking loss.',
                  measurements=results)
    (args.output/'projection_changes.json').write_text(json.dumps(report, indent=2)+'\n')
    for row in results:
        print(f"cam{row['camera']} {row['comparison']}: centre median "
              f"{row['central']['median_px']:.3f}px, periphery median "
              f"{row['peripheral']['median_px']:.3f}px, maximum {row['all']['maximum_px']:.3f}px")


if __name__ == '__main__':
    main()
