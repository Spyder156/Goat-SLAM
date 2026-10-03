#!/usr/bin/env python3
"""Append native VRS camera-domain metadata to an isolated SLAM settings copy."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from projectaria_tools.core import data_provider


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vrs', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists() or args.out.resolve() == args.config.resolve():
        raise ValueError('Choose a fresh output; input settings are immutable')
    source = args.config.read_text()
    if '.validRadius:' in source or '.maxSolidAngle:' in source:
        raise ValueError('Input already contains camera-domain metadata')
    settings = cv2.FileStorage(str(args.config), cv2.FILE_STORAGE_READ)
    if settings.getNode('Camera.type').string() != 'Fisheye624':
        raise ValueError('Only native Fisheye624 settings are supported')
    provider = data_provider.create_vrs_data_provider(str(args.vrs))
    calibration = provider.get_device_calibration()
    keys = ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'k3', 'k4', 'k5', 'k6',
            'p1', 'p2', 's1', 's2', 's3', 's4']
    records, additions = [], []
    for index, label in enumerate(('camera-slam-left', 'camera-slam-right'), 1):
        camera = calibration.get_camera_calib(label)
        native = np.asarray(camera.projection_params(), dtype=float)
        expected = np.r_[native[0], native[0], native[1:]]
        actual = np.array([settings.getNode(f'Camera{index}.{k}').real() for k in keys])
        if len(expected) != 16 or not np.allclose(actual, expected, rtol=0, atol=1e-8):
            raise ValueError(f'{label}: settings intrinsics differ from this VRS')
        size = np.asarray(camera.get_image_size(), dtype=int)
        config_size = [int(settings.getNode('Camera.width').real()),
                       int(settings.getNode('Camera.height').real())]
        if config_size != size.tolist():
            raise ValueError('Settings native resolution differs; resize through explicit settings')
        radius = camera.get_valid_radius()
        angle = float(camera.get_max_solid_angle())
        if radius is None or not np.isfinite(radius) or radius <= 0 or not 0 < angle < np.pi:
            raise ValueError(f'{label}: missing/invalid native domain')
        additions.extend((f'Camera{index}.validRadius: {float(radius):.17g}',
                          f'Camera{index}.maxSolidAngle: {angle:.17g}'))
        records.append({'label': label, 'camera': index, 'resolution': size.tolist(),
                        'principal_point': camera.get_principal_point().tolist(),
                        'valid_radius_native_px': float(radius),
                        'max_solid_angle_rad': angle,
                        'intrinsics_max_abs_difference': float(np.max(np.abs(actual-expected)))})
    settings.release()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(source.rstrip()+'\n\n# Native calibrated domain; lens parameters unchanged.\n'+'\n'.join(additions)+'\n')
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    record = {'vrs': str(args.vrs.resolve()), 'source_config': str(args.config.resolve()),
              'source_config_sha256': digest(args.config), 'output_config': str(args.out.resolve()),
              'output_config_sha256': digest(args.out), 'cameras': records,
              'intrinsics_modified': False, 'ground_truth_used': False,
              'circle_center': 'principal point', 'radius_units': 'native image pixels'}
    args.out.with_suffix('.domain.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record, indent=2))


if __name__ == '__main__':
    main()
