#!/usr/bin/env python3
"""Fuse the two Aria IMUs into one virtual IMU expressed at the right IMU (W14a).

Inputs are the two factory-rectified EuRoC CSVs (right: 1 kHz, left: 800 Hz; each in its own sensor
frame, corrected device time) and imu_calibration_both.json (T_device_imu for both). Output: one CSV on
the right IMU's timestamp grid, in the right IMU frame, so the estimator's IMU extrinsic (T_b_c1, body =
right IMU) stays valid.

  omega_fused = 0.5 * (omega_R + R_RL omega_L)
  f_R_from_L  = R_RL f_L + alpha x r + omega x (omega x r)      r = p_R - p_L in the right frame
  f_fused     = 0.5 * (f_R + f_R_from_L)

alpha is the smoothed time derivative of omega_fused (Savitzky-Golay). Conventions: EuRoC columns
"#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z"; rad/s and m/s^2; T_device_imu maps IMU frame -> device frame.
The script writes a manifest with validation numbers. The lever-arm model is falsified if the compensated
residual between the right accelerometer and the transformed left one is not smaller than the
uncompensated one. GT is never read.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def load_csv(path):
    with open(path) as f:
        head = f.readline()
    data = np.loadtxt(path, delimiter=',', skiprows=0 if head.strip()[0].isdigit() else 1)
    t = data[:, 0].astype(np.int64)
    return t, data[:, 1:4], data[:, 4:7]


def interp_rows(t_src, x_src, t_dst):
    return np.stack([np.interp(t_dst, t_src, x_src[:, k]) for k in range(3)], axis=1)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--right', type=Path, required=True)
    p.add_argument('--left', type=Path, required=True)
    p.add_argument('--calibration', type=Path, required=True, help='imu_calibration_both.json')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--savgol-window', type=int, default=21, help='samples on the 1 kHz grid (odd)')
    args = p.parse_args()
    if args.out.exists():
        raise SystemExit(f'refusing to overwrite {args.out}')
    calib = json.loads(args.calibration.read_text())
    T_dR = np.array(calib['imus']['imu-right']['T_device_imu'], dtype=float)
    T_dL = np.array(calib['imus']['imu-left']['T_device_imu'], dtype=float)
    R_dR, p_dR = T_dR[:3, :3], T_dR[:3, 3]
    R_dL, p_dL = T_dL[:3, :3], T_dL[:3, 3]
    R_RL = R_dR.T @ R_dL                              # left IMU frame -> right IMU frame
    r_LR_R = R_dR.T @ (p_dR - p_dL)                   # from left IMU to right IMU, right frame, metres
    tR, wR, aR = load_csv(args.right)
    tL, wL, aL = load_csv(args.left)
    tRs, tLs = tR * 1e-9, tL * 1e-9
    keep = (tRs >= tLs[0]) & (tRs <= tLs[-1])         # overlap only; no extrapolation
    tR, tRs, wR, aR = tR[keep], tRs[keep], wR[keep], aR[keep]
    wL_R = interp_rows(tLs, wL, tRs) @ R_RL.T         # rows are vectors: v_R = R_RL v_L
    aL_R = interp_rows(tLs, aL, tRs) @ R_RL.T
    w_fused = 0.5 * (wR + wL_R)
    dt = np.median(np.diff(tRs))
    alpha = savgol_filter(w_fused, args.savgol_window, 2, deriv=1, delta=dt, axis=0)
    lever = np.cross(alpha, r_LR_R) + np.cross(w_fused, np.cross(w_fused, r_LR_R))
    f_from_L = aL_R + lever
    f_from_L_wrong_sign = aL_R - lever
    a_fused = 0.5 * (aR + f_from_L)
    rms = lambda x: float(np.sqrt(np.mean(np.sum(x ** 2, axis=1))))
    validation = {
        'gyro_residual_rms_rad_s': rms(wR - wL_R),
        'accel_residual_rms_uncompensated_m_s2': rms(aR - aL_R),
        'accel_residual_rms_compensated_m_s2': rms(aR - f_from_L),
        'accel_residual_rms_wrong_sign_m_s2': rms(aR - f_from_L_wrong_sign),
        'lever_term_rms_m_s2': rms(lever),
        'lever_arm_model_supported': bool(rms(aR - f_from_L) < rms(aR - aL_R) and rms(aR - f_from_L) < rms(aR - f_from_L_wrong_sign)),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, 'w') as f:
        f.write('#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n')
        for i in range(len(tR)):
            f.write(f'{tR[i]},' + ','.join(f'{v:.12g}' for v in (*w_fused[i], *a_fused[i])) + '\n')
    right_manifest = json.loads(Path(str(args.right) + '.manifest.json').read_text()) if Path(str(args.right) + '.manifest.json').exists() else {}
    manifest = dict(right_manifest)
    manifest.update({
        'imu_label': 'imu-fused', 'stream_id': '1202-1+1202-2', 'nominal_rate_hz': 1000.0,
        'fusion': {'method': 'average of right and lever-arm-compensated left, right IMU frame, right timestamp grid',
                   'R_RL': R_RL.tolist(), 'lever_arm_L_to_R_right_frame_m': r_LR_R.tolist(),
                   'savgol_window_samples': args.savgol_window, 'left_source': str(args.left), 'right_source': str(args.right),
                   'left_sha256': sha256(args.left), 'right_sha256': sha256(args.right), 'calibration': str(args.calibration)},
        'validation': validation,
        'output_csv': str(args.out), 'output_sha256': sha256(args.out), 'output_rows': int(len(tR)),
        'first_timestamp_ns': int(tR[0]), 'last_timestamp_ns': int(tR[-1]), 'ground_truth_used': False,
    })
    Path(str(args.out) + '.manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'rows': int(len(tR)), 'dropped_outside_left_range': int((~keep).sum()), **validation}, indent=2))


if __name__ == '__main__':
    main()
