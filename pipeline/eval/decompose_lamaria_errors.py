#!/usr/bin/env python3
"""Read-only decomposition of one scored LaMAria run: startup, drift, tilt.

Stages 0-2 of the 2026-10-04 diagnostic plan:
  0  preflight: input hashes match the saved score provenance; the scored cam0
     poses, the saved CP Sim3 and the official per-timestamp errors are
     reproduced from raw files before any new number is derived.
  1  heading-drift integral: does the estimator's own heading error (from
     ORIENTATIONS, which the position Sim3 never uses) predict the observed
     horizontal error field?  Separates start deformation from alignment
     compromise and splits the late error into yaw and local-scale parts.
  2  GT-free localisation of the late scale error: online (live) versus final
     (post-BA) stretch, internal tracking/IMU correlates, event ledger.

Nothing here runs SLAM, the offline solver, the scorer or a renderer, and
nothing under the run directory is written.  Ground truth is read only on the
evaluation side.  Every GT-derived number is a diagnostic, not a score; the
official Score2D of the inputs is unchanged.

Conventions (stated once, used everywhere):
  timestamps     : integer nanoseconds in files, native Aria seconds in plots
  quaternions    : xyzw
  trajectory.csv : T_export_body, body(IMU-right) -> export frame, metres.
                   The export frame is the body frame of the first surviving
                   keyframe (KF 29); it is NOT gravity aligned.
  cam0           : T_exp_cam0 = T_exp_body * T_body_cam0, T_body_cam0 = IMU.T_b_c1
  gt_dense.txt   : T_world_cam0, survey frame, z up (ASSUMPTION: gravity aligned)
  CP Sim3        : p_gt = s * R_cp * p_cam0 + t   (scores.json CP_sim3)
  R_rel(t)       : R_gt_cam0(t) * R_est_cam0(t)^T, maps export-frame vectors
                   into the GT frame; near-constant if orientations agree
  u_exp          : -g/|g| from the offline VI solver gravity, export frame
"""
import argparse
import csv
import importlib.metadata
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))
sys.path.insert(0, str(HERE.parents[1] / 'run'))
sys.path.insert(0, str(HERE.parents[1] / 'viz'))
from artifact_paths import relocated_path  # noqa: E402
from score_lamaria import sha, save_json  # noqa: E402
# audit_lamaria_scale.fit is reserved for the Stage 3-5 pass (not used in Stages 0-2)
from babyfeats_diagnostics import read_baby_events, episodes  # noqa: E402
from make_lamaria_output import (load_table, load_calibration, body_to_cam0,  # noqa: E402
                                 load_imu, apply_committed_radial_calibration)

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

TOOLKIT = Path('/home/raghav/workspace/MeckaAI/third_party/lamaria_toolkit')
BIN_EDGES = np.arange(150., 2000., 50.)          # native seconds, left-closed
SEGMENTS = {'start': (163., 400.), 'mid': (400., 1600.), 'end': (1600., 1950.)}
REFERENCE_WINDOW = (400., 1400.)                 # mid-route window for R_ref
FLAGS = {'evaluation_only': True, 'GT_used_in_estimator': False, 'new_official_score': False,
         'official_score_changed': False, 'estimator_modified': False, 'slam_or_solver_rerun': False}


# ----------------------------------------------------------------------------- helpers
def read_rows(path):
    with Path(path).open() as handle:
        return list(csv.DictReader(handle))


def load_export_trajectory(path):
    """Raw atlas export: single body0 frame, strictly increasing stamps."""
    rows = read_rows(path)
    frames = {row['coordinate_frame'] for row in rows}
    if len(frames) != 1 or not next(iter(frames)).endswith('_body0'):
        raise ValueError(f'Expected one raw world_from_imu-right body0 export: {path}')
    stamps = np.rint(np.array([float(r['t_s']) for r in rows]) * 1e9).astype(np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('Export timestamps must be strictly increasing')
    p = np.array([[float(r[k]) for k in ('tx', 'ty', 'tz')] for r in rows])
    q = np.array([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in rows])
    if np.max(abs(np.linalg.norm(q, axis=1) - 1)) >= 1e-5:
        raise ValueError('Export contains non-unit quaternions')
    return stamps, p, q, rows


def load_pose_table(path):
    """'timestamp_ns tx ty tz qx qy qz qw' (one '#' header allowed) -> stamps, p, R."""
    data = load_table(Path(path), 8)
    if not len(data) or not np.isfinite(data).all():
        raise ValueError(f'Expected finite 8-column poses: {path}')
    stamps = data[:, 0].astype(np.int64)
    if not np.array_equal(data[:, 0], stamps):
        raise ValueError(f'Timestamps are not integer nanoseconds: {path}')
    return stamps, data[:, 1:4], Rotation.from_quat(data[:, 4:8]).as_matrix()


def official_match(est_stamps, gt_stamps):
    """Official 1 ms matcher, ordered exactly as score_lamaria.associate_dense."""
    from lamaria.utils.timestamps import matching_time_indices
    est_stamps, gt_stamps = np.asarray(est_stamps), np.asarray(gt_stamps)
    if len(est_stamps) >= len(gt_stamps):
        gt_ids, est_ids = matching_time_indices(gt_stamps, est_stamps)
    else:
        est_ids, gt_ids = matching_time_indices(est_stamps, gt_stamps)
    est_ids, gt_ids = np.asarray(est_ids, dtype=int), np.asarray(gt_ids, dtype=int)
    if len(np.unique(est_ids)) != len(est_ids) or len(np.unique(gt_ids)) != len(gt_ids):
        raise ValueError('Ambiguous duplicate timestamp associations')
    if len(est_ids) and np.max(abs(est_stamps[est_ids] - gt_stamps[gt_ids])) > 1000000:
        raise ValueError('Association exceeded the official 1 ms tolerance')
    matched = set(gt_stamps[gt_ids].tolist())
    unmatched = [int(s) for s in gt_stamps if int(s) not in matched]
    return est_ids, gt_ids, unmatched


def sim3_from_score(score):
    sim3 = score['CP_sim3']
    return float(sim3['scale']), Rotation.from_quat(sim3['rotation_xyzw']).as_matrix(), np.asarray(sim3['translation'], float)


def apply_sim3(s, R, t, p):
    # p_gt = s * R * p + t, applied row-wise
    return s * (p @ R.T) + t


def chordal_mean(Rs):
    """Nearest proper rotation to the arithmetic mean of rotation matrices."""
    U, _, Vt = np.linalg.svd(np.sum(Rs, axis=0))
    return U @ np.diag([1., 1., np.linalg.det(U @ Vt)]) @ Vt


def twist_about_z(Rs):
    """Swing-twist split about the GT z axis.

    Returns (twist_deg, swing_deg): twist is the yaw component (signed, about
    +z, right-handed), swing the remaining tilt magnitude.  Works on (N,3,3).
    """
    q = Rotation.from_matrix(np.asarray(Rs).reshape(-1, 3, 3)).as_quat()   # xyzw
    twist = 2.0 * np.arctan2(q[:, 2], q[:, 3])
    twist = (twist + np.pi) % (2 * np.pi) - np.pi
    Rz_inv = Rotation.from_euler('z', -twist[:, None]).as_matrix()
    swing = Rotation.from_matrix(np.asarray(Rs).reshape(-1, 3, 3) @ Rz_inv).magnitude()
    return np.degrees(twist), np.degrees(swing)


def rolling_median(t, x, width_s):
    """Centred rolling median over a time window; t must be sorted."""
    lo = np.searchsorted(t, t - width_s / 2, side='left')
    hi = np.searchsorted(t, t + width_s / 2, side='right')
    return np.array([np.median(x[a:b]) for a, b in zip(lo, hi)])


def rotate2d(psi_rad, d):
    """Apply R_z(psi) to 2-D vectors d (N,2) with per-row angles."""
    c, s = np.cos(psi_rad), np.sin(psi_rad)
    return np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)


def heading_integral_prediction(t, p_gt_xy, psi_rad, e_obs_xy, pivot):
    """Yaw-only prediction of the horizontal error field.

    Model: aligned estimated displacement = R_z(psi) * GT displacement, scale 1.
    e_pred(t) = e_obs(pivot) + sum over the path from the pivot of (R_z(psi)-I) d_gt.
    psi is the heading error of the aligned estimate relative to GT; the angle
    used on each interval is the mean of its two endpoint angles.
    """
    n = len(t)
    e_pred = np.zeros((n, 2))
    e_pred[pivot] = e_obs_xy[pivot]
    d = np.diff(p_gt_xy, axis=0)                       # d[i] = p[i+1] - p[i]
    psi_mid = 0.5 * (psi_rad[1:] + psi_rad[:-1])
    inc = rotate2d(psi_mid, d) - d                     # (R_z(psi)-I) d, per interval
    for i in range(pivot + 1, n):
        e_pred[i] = e_pred[i - 1] + inc[i - 1]
    for i in range(pivot - 1, -1, -1):
        e_pred[i] = e_pred[i + 1] - inc[i]
    return e_pred


def windowed_displacement_ratio(t, p_a, p_b, path_m=15.0, max_dt_s=30.0):
    """|d_a|/|d_b| and signed heading angle (deg, a from b) over windows path_m
    long along trajectory b; returns (t_mid, ratio, heading_deg).  Positional
    diagnostic: no orientations involved."""
    seg = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p_b[:, :2], axis=0), axis=1))]
    j = np.searchsorted(seg, seg + path_m, side='left')
    valid = (j < len(t)) & ((t[np.minimum(j, len(t) - 1)] - t) <= max_dt_s)
    i = np.flatnonzero(valid)
    j = j[valid]
    da, db = p_a[j, :2] - p_a[i, :2], p_b[j, :2] - p_b[i, :2]
    ratio = np.linalg.norm(da, axis=1) / np.linalg.norm(db, axis=1)
    heading = np.degrees(np.arctan2(db[:, 0] * da[:, 1] - db[:, 1] * da[:, 0], np.sum(da * db, axis=1)))
    return 0.5 * (t[i] + t[j]), ratio, heading


def bin_indices(t, edges=BIN_EDGES):
    return [(float(a), float(b), np.flatnonzero((t >= a) & (t < b))) for a, b in zip(edges[:-1], edges[1:])]


def fnum(x):
    return None if x is None or (isinstance(x, float) and not np.isfinite(x)) else float(x)


def git_head(path):
    return subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()


# ----------------------------------------------------------------------------- context
class Context:
    """Everything the stages share; built once, read-only."""

    def __init__(self, args):
        self.run = args.run.resolve(strict=True)
        self.score_dir = self.run / 'lamaria_score'
        self.score = json.loads((self.score_dir / 'scores.json').read_text())
        self.provenance = json.loads((self.score_dir / 'provenance.json').read_text())
        self.gt_dense = args.gt_dense.resolve(strict=True)
        self.gt_sparse = args.gt_sparse.resolve(strict=True)
        self.solver_result = args.solver_result.resolve(strict=True)
        self.refinement = args.refinement_run.resolve(strict=True) if args.refinement_run else None
        self.settings = self.run / 'config/settings.yaml'
        self.trajectory_csv = next(self.run.glob('atlas_*_trajectory.csv'))
        self.keyframes_csv = next(self.run.glob('atlas_*_keyframes.csv'))
        self.history_csv = next(self.run.glob('atlas_*_history.csv'))
        self.maps_csv = next(self.run.glob('atlas_*_maps.csv'))
        self.online_csv = next(self.run.glob('online_*.csv'))
        self.log = self.run / 'run.log'
        self.imu_csv = (self.run / 'input/euroc/mav0/imu0/data.csv')
        # T_body_cam0 (body_from_cam0), SVD re-orthonormalised as the viz tooling does
        self.Tbc, self.T01, self.cameras = load_calibration(self.settings)
        gt = np.loadtxt(self.gt_dense, ndmin=2)
        if gt.shape[1] != 8 or not np.isfinite(gt).all():
            raise ValueError('Expected finite 8-column dense GT')
        self.gt_stamps = np.rint(gt[:, 0]).astype(np.int64)
        self.gt_p = gt[:, 1:4]
        self.gt_R = Rotation.from_quat(gt[:, 4:8]).as_matrix()     # R_world_cam0
        solver = json.loads(self.solver_result.read_text())
        g = np.asarray(solver['gravity'], float)
        self.gravity = g
        self.u_exp = -g / np.linalg.norm(g)                         # estimated 'up', export frame
        sparse = json.loads(self.gt_sparse.read_text())
        windows = {}
        for name, image in sparse['images'].items():
            cp = image['control_point']
            stamp = int(image['timestamp']) * 1e-9
            lo, hi = windows.get(cp, (np.inf, -np.inf))
            windows[cp] = (min(lo, stamp), max(hi, stamp))
        self.cp_windows = {name: {'first_s': lo, 'last_s': hi} for name, (lo, hi) in windows.items()}
        self.cp_errors = json.loads((self.score_dir / 'control_point_errors.json').read_text())
        baby_events, malformed = read_baby_events(self.log)
        self.baby_events, self.baby_malformed = baby_events, malformed
        self.sos = episodes(baby_events)
        self.commits = apply_committed_radial_calibration(self.run, [dict(c, parameters=c['parameters'].copy()) for c in self.cameras])
        self.online = read_rows(self.online_csv)
        wv_changes, imu_init_time, previous = [], None, None
        for row in self.online:
            wv = int(row['world_version'])
            if wv != previous:
                wv_changes.append({'t_s': float(row['input_t_s']), 'world_version': wv})
                previous = wv
            if imu_init_time is None and row['imu_initialized'] == '1':
                imu_init_time = float(row['input_t_s'])
        self.world_versions, self.imu_init_time = wv_changes, imu_init_time


# ----------------------------------------------------------------------------- stage 0
def preflight(ctx, out):
    """Hash lock + convention self-test.  Raises on any failure."""
    sub = out / '00_preflight'
    sub.mkdir()
    mismatches, checked = [], {}
    for recorded, digest in ctx.provenance['files_sha256'].items():
        path = relocated_path(recorded)
        actual = sha(path)
        checked[str(path)] = actual
        if actual != digest:
            mismatches.append({'file': str(path), 'recorded': digest, 'actual': actual})
    for name, digest in ctx.provenance.get('generated_files_sha256', {}).items():
        path = ctx.score_dir / name
        actual = sha(path)
        checked[str(path)] = actual
        if actual != digest:
            mismatches.append({'file': str(path), 'recorded': digest, 'actual': actual})
    extra = [ctx.keyframes_csv, ctx.history_csv, ctx.maps_csv, ctx.online_csv, ctx.log,
             ctx.imu_csv.resolve(), ctx.solver_result, ctx.gt_sparse]
    for path in extra:
        checked[str(path)] = sha(path)
    official = {name: sha(TOOLKIT / name) for name in ctx.provenance['official_python_sha256']}
    toolkit_changed = official != ctx.provenance['official_python_sha256']
    if toolkit_changed:
        mismatches.append({'file': str(TOOLKIT), 'recorded': 'official_python_sha256', 'actual': 'differs'})
    if mismatches:
        save_json(sub / 'HASH_MISMATCH.json', mismatches)
        raise RuntimeError(f'Input hashes differ from the saved score provenance: {sub / "HASH_MISMATCH.json"}')

    # Convention self-test 1: rebuild cam0 from the raw body export.
    stamps, p_body, q_body, _ = load_export_trajectory(ctx.trajectory_csv)
    raw = np.c_[stamps.astype(float), p_body, q_body]
    _, p_cam0, R_cam0, _ = body_to_cam0(raw, ctx.Tbc)
    s_stamps, s_p, s_R = load_pose_table(ctx.score_dir / 'estimated_cam0_ns.txt')
    if not np.array_equal(s_stamps, stamps):
        raise RuntimeError('Scored cam0 timestamps differ from the raw export')
    cam0_dp = float(np.max(np.linalg.norm(p_cam0 - s_p, axis=1)))
    cam0_dR = float(np.max(abs(R_cam0 - s_R)))
    if cam0_dp > 1e-6 or cam0_dR > 1e-6:
        raise RuntimeError(f'cam0 rebuild disagrees with the scored poses: dp {cam0_dp} m, dR {cam0_dR}')

    # Convention self-test 2: saved Sim3 + official matcher reproduce the official errors.
    s, R, t = sim3_from_score(ctx.score)
    est_ids, gt_ids, unmatched = official_match(s_stamps, ctx.gt_stamps)
    aligned = apply_sim3(s, R, t, s_p[est_ids])
    errors = np.linalg.norm(aligned[:, :2] - ctx.gt_p[gt_ids, :2], axis=1)
    saved = read_rows(ctx.score_dir / 'pgt_horizontal_errors.csv')
    saved_gt = np.array([int(r['gt_timestamp_ns']) for r in saved])
    saved_err = np.array([float(r['horizontal_error_m']) for r in saved])
    if not np.array_equal(saved_gt, ctx.gt_stamps[gt_ids]):
        raise RuntimeError('Official matched GT timestamps differ from the saved csv')
    err_diff = float(np.max(abs(errors - saved_err)))
    if err_diff > 1e-9:
        raise RuntimeError(f'Saved Sim3 does not reproduce the official errors: max diff {err_diff} m')
    saved_unmatched = json.loads((ctx.score_dir / 'unmatched_gt_timestamps_ns.json').read_text())
    if sorted(unmatched) != sorted(saved_unmatched):
        raise RuntimeError('Unmatched GT set differs from the saved list')
    before_first = sum(u < int(stamps[0]) for u in unmatched)

    # Convention self-test 3: the solver gravity frame is the export frame (identity at KF 29).
    body_traj = ctx.solver_result.parent / 'body_trajectory_ns.txt'
    first = np.loadtxt(body_traj, ndmin=2)[0]
    gravity_frame_ok = bool(abs(first[0] - stamps[0]) <= 250000000 and np.linalg.norm(first[1:4]) < 1e-6
                            and min(np.linalg.norm(first[4:8] - [0, 0, 0, 1]), np.linalg.norm(first[4:8] - [0, 0, 0, -1])) < 1e-6)
    if not gravity_frame_ok:
        raise RuntimeError('Solver trajectory does not start at identity at the export origin; gravity frame unverified')

    versions = {name: importlib.metadata.version(name) for name in ('numpy', 'scipy', 'matplotlib', 'pycolmap', 'lamaria')}
    selftest = {
        'matched_gt': int(len(gt_ids)), 'unmatched_gt': len(unmatched), 'unmatched_before_first_estimate': int(before_first),
        'max_association_difference_ns': int(np.max(abs(s_stamps[est_ids] - ctx.gt_stamps[gt_ids]))),
        'cam0_rebuild_max_dp_m': cam0_dp, 'cam0_rebuild_max_dR': cam0_dR,
        'official_error_reproduction_max_diff_m': err_diff,
        'solver_gravity_export_frame': ctx.gravity.tolist(), 'gravity_norm': float(np.linalg.norm(ctx.gravity)),
        'u_exp': ctx.u_exp.tolist(), 'solver_first_pose_identity_at_export_origin': gravity_frame_ok,
        'T_body_cam0': ctx.Tbc.tolist(), 'CP_sim3': ctx.score['CP_sim3'],
        'export_z_vs_u_exp_deg': float(np.degrees(np.arccos(np.clip(ctx.u_exp[2], -1, 1)))),
        'passed': True,
    }
    save_json(sub / 'selftest.json', selftest)
    save_json(sub / 'provenance.json', {
        **FLAGS, 'run': str(ctx.run), 'score_provenance_checked': str(ctx.score_dir / 'provenance.json'),
        'input_sha256': checked, 'official_python_sha256_unchanged': not toolkit_changed,
        'toolkit_commit': git_head(TOOLKIT), 'toolkit_commit_recorded': ctx.provenance['toolkit_commit'],
        'versions': versions, 'python': sys.executable, 'official_Score2D': ctx.score['Score2D'],
    })
    return {'s_stamps': s_stamps, 's_p': s_p, 's_R': s_R, 'est_ids': est_ids, 'gt_ids': gt_ids,
            'errors': errors, 'selftest': selftest}


# ----------------------------------------------------------------------------- stage 1
def heading_integral(ctx, score, s_stamps, s_p, s_R, out, label):
    """Stage 1 for one scored trajectory (scored cam0 poses + its own CP Sim3)."""
    sub = out
    sub.mkdir()
    s, R_cp, t_cp = sim3_from_score(score)
    est_ids, gt_ids, _ = official_match(s_stamps, ctx.gt_stamps)
    t = s_stamps[est_ids] * 1e-9
    p_est, R_est = s_p[est_ids], s_R[est_ids]
    p_gt, R_gt = ctx.gt_p[gt_ids], ctx.gt_R[gt_ids]
    aligned = apply_sim3(s, R_cp, t_cp, p_est)
    e_obs = aligned[:, :2] - p_gt[:, :2]
    err = np.linalg.norm(e_obs, axis=1)

    # Orientations only: R_rel maps export-frame vectors into the GT frame.
    R_rel = np.einsum('nij,nkj->nik', R_gt, R_est)                   # R_gt @ R_est^T
    ref_mask = (t >= REFERENCE_WINDOW[0]) & (t < REFERENCE_WINDOW[1])
    R_ref = chordal_mean(R_rel[ref_mask])
    D = np.einsum('nij,kj->nik', R_rel, R_ref)                      # R_rel @ R_ref^T : drift of orientation vs mid-route
    psi_D, swing_D = twist_about_z(D)
    M = np.einsum('ij,nkj->nik', R_cp, R_rel)                       # R_cp @ R_rel^T : heading error of the ALIGNED estimate vs GT
    psi_M, _ = twist_about_z(M)
    cp_vs_ref_twist, cp_vs_ref_swing = twist_about_z(R_cp @ R_ref.T)
    tilt_orientation = float(np.degrees(np.arccos(np.clip((R_ref @ ctx.u_exp)[2], -1, 1))))
    tilt_cp = float(np.degrees(np.arccos(np.clip((R_cp @ ctx.u_exp)[2], -1, 1))))
    psi_s = rolling_median(t, np.radians(psi_M), 10.0)              # smoothed heading error, radians

    pivot_cp = 'AA3814'
    w = ctx.cp_windows[pivot_cp]
    t_pivot = 0.5 * (w['first_s'] + w['last_s'])
    pivot = int(np.argmin(abs(t - t_pivot)))
    e_pred = heading_integral_prediction(t, p_gt[:, :2], psi_s, e_obs, pivot)
    resid = e_obs - e_pred

    def angle_between(a, b):
        return np.degrees(abs(np.arctan2(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0], np.sum(a * b, axis=1))))

    direction = angle_between(e_obs, e_pred)
    big = (np.linalg.norm(e_obs, axis=1) > 0.5) & (np.linalg.norm(e_pred, axis=1) > 0.5)

    # Positional (not orientation-derived) local displacement ratio and heading, 15 m windows.
    t_w, ratio_w, heading_w = windowed_displacement_ratio(t, aligned, p_gt)

    rows = []
    for a, b, ids in bin_indices(t):
        if not len(ids):
            continue
        wmask = (t_w >= a) & (t_w < b)
        de = e_obs[ids[-1]] - e_obs[ids[0]]
        dr = resid[ids[-1]] - resid[ids[0]]
        dp = p_gt[ids[-1], :2] - p_gt[ids[0], :2]
        k_eff = fnum(1 + np.dot(dr, dp) / np.dot(dp, dp)) if np.linalg.norm(dp) > 5 else None
        rows.append({
            'bin_start_s': a, 'bin_end_s': b, 'n': int(len(ids)),
            'median_abs_e_obs_m': fnum(np.median(err[ids])),
            'median_abs_e_pred_m': fnum(np.median(np.linalg.norm(e_pred[ids], axis=1))),
            'median_abs_residual_m': fnum(np.median(np.linalg.norm(resid[ids], axis=1))),
            'median_direction_diff_deg': fnum(np.median(direction[ids][big[ids]])) if big[ids].any() else None,
            'mean_e_obs_xy': e_obs[ids].mean(0).tolist(), 'mean_e_pred_xy': e_pred[ids].mean(0).tolist(),
            'mean_residual_xy': resid[ids].mean(0).tolist(),
            'psi_M_mean_deg': fnum(np.degrees(psi_s[ids]).mean()), 'psi_D_mean_deg': fnum(psi_D[ids].mean()),
            'swing_D_mean_deg': fnum(swing_D[ids].mean()),
            'gt_net_displacement_m': fnum(np.linalg.norm(dp)),
            'effective_scale_from_residual': k_eff,
            'positional_ratio_15m_median': fnum(np.median(ratio_w[wmask])) if wmask.any() else None,
            'positional_heading_15m_median_deg': fnum(np.median(heading_w[wmask])) if wmask.any() else None,
            'delta_e_obs_xy_over_bin': de.tolist(),
        })

    segments = {}
    for name, (a, b) in SEGMENTS.items():
        m = (t >= a) & (t < b)
        if m.sum() < 10:
            continue
        x, y = e_pred[m].ravel(), e_obs[m].ravel()
        slope0 = float(np.dot(x, y) / np.dot(x, x)) if np.dot(x, x) > 0 else None
        A = np.c_[x, np.ones_like(x)]
        slope, intercept = np.linalg.lstsq(A, y, rcond=None)[0]
        r = float(np.corrcoef(x, y)[0, 1]) if np.std(x) > 0 and np.std(y) > 0 else None
        rms_obs = float(np.sqrt(np.mean(np.sum(e_obs[m] ** 2, axis=1))))
        rms_res = float(np.sqrt(np.mean(np.sum(resid[m] ** 2, axis=1))))
        segments[name] = {
            'start_s': a, 'end_s': b, 'n': int(m.sum()),
            'median_abs_e_obs_m': fnum(np.median(err[m])),
            'median_abs_residual_m': fnum(np.median(np.linalg.norm(resid[m], axis=1))),
            'median_direction_diff_deg': fnum(np.median(direction[m][big[m]])) if big[m].any() else None,
            'regression_slope_through_origin': slope0, 'regression_slope': float(slope), 'regression_intercept_m': float(intercept),
            'pearson_r': r, 'rms_e_obs_m': rms_obs, 'rms_residual_m': rms_res,
            'fraction_of_rms_explained_by_yaw': fnum(1 - rms_res / rms_obs) if rms_obs > 0 else None,
            'psi_M_median_deg': fnum(np.degrees(np.median(psi_s[m]))),
            'positional_ratio_15m_median': fnum(np.median(ratio_w[(t_w >= a) & (t_w < b)])) if ((t_w >= a) & (t_w < b)).any() else None,
        }

    # Start arithmetic: heading needed to produce the first error over the path to the pivot.
    path_to_pivot = float(np.sum(np.linalg.norm(np.diff(p_gt[:pivot + 1, :2], axis=0), axis=1)))
    start_arith = {
        'first_error_m': fnum(err[0]), 'pivot_cp': pivot_cp, 'pivot_time_s': float(t[pivot]), 'pivot_error_m': fnum(err[pivot]),
        'gt_path_first_to_pivot_m': path_to_pivot,
        'heading_needed_deg': fnum(np.degrees(np.arctan2(err[0] - err[pivot], path_to_pivot))),
        'measured_psi_M_median_163_300_deg': fnum(np.degrees(np.median(psi_s[(t >= 163) & (t < 300)]))),
        'measured_psi_M_median_mid_deg': fnum(np.degrees(np.median(psi_s[(t >= 400) & (t < 1600)]))),
    }
    yaw_elapsed = []
    t0 = float(ctx.online[0]['input_t_s'])
    for k in range(18):
        m = (t - t0 >= 100 * k) & (t - t0 < 100 * (k + 1))
        yaw_elapsed.append({'elapsed_bin_start_s': 100 * k, 'n': int(m.sum()),
                            'psi_D_mean_deg': fnum(psi_D[m].mean()) if m.any() else None,
                            'swing_D_mean_deg': fnum(swing_D[m].mean()) if m.any() else None})
    st, en = segments.get('start'), segments.get('end')
    end_bins = [r for r in rows if r['bin_start_s'] >= 1700 and r['effective_scale_from_residual'] is not None
                and r['positional_ratio_15m_median'] is not None]
    scale_agree = [abs(r['effective_scale_from_residual'] - r['positional_ratio_15m_median']) for r in end_bins]
    verdict = {
        'start_heading_explains_error': bool(st and st['median_abs_residual_m'] is not None and st['median_abs_residual_m'] < 1.0
                                            and (st['median_direction_diff_deg'] or 99) < 20 and 0.8 <= (st['regression_slope'] or 0) <= 1.2
                                            and (st['pearson_r'] or 0) > 0.9),
        'start_broken_prediction_near_zero_or_wrong_sign': bool(st and ((st['regression_slope'] or 0) < 0.3)),
        'end_fraction_explained_by_yaw': en['fraction_of_rms_explained_by_yaw'] if en else None,
        'end_yaw_explains_more_than_80pct_BROKEN': bool(en and (en['fraction_of_rms_explained_by_yaw'] or 0) > 0.8),
        'end_effective_scale_matches_positional_ratio_within_0p03': bool(scale_agree) and bool(np.median(scale_agree) < 0.03),
        'end_effective_scale_vs_positional_ratio_median_abs_diff': fnum(np.median(scale_agree)) if scale_agree else None,
    }
    result = {
        **FLAGS, 'label': label, 'official_Score2D_unchanged': score['Score2D'],
        'conventions': (__doc__ or '').split('Conventions')[1],
        'reference_window_s': list(REFERENCE_WINDOW), 'n_matched': int(len(t)),
        'R_ref_quat_xyzw': Rotation.from_matrix(R_ref).as_quat().tolist(),
        'R_cp_vs_R_ref': {'twist_yaw_deg': fnum(cp_vs_ref_twist[0]), 'swing_tilt_deg': fnum(cp_vs_ref_swing[0])},
        'tilt_of_u_exp_deg': {'orientation_based_R_ref': tilt_orientation, 'cp_sim3_based_R_cp': tilt_cp},
        'R_rel_deviation_from_ref_deg': {'median': fnum(np.median(Rotation.from_matrix(D).magnitude() * 180 / np.pi)),
                                         'p90': fnum(np.percentile(Rotation.from_matrix(D).magnitude() * 180 / np.pi, 90)),
                                         'max': fnum(np.max(Rotation.from_matrix(D).magnitude() * 180 / np.pi))},
        'pivot': {'cp': pivot_cp, 'window_s': w, 'index': pivot, 'time_s': float(t[pivot])},
        'start_arithmetic': start_arith, 'segments': segments, 'bins_50s': rows,
        'yaw_drift_by_elapsed_100s': yaw_elapsed, 'verdict': verdict,
        'notes': ['psi_M is the twist about GT z of R_cp R_rel^T: heading error of the Sim3-aligned estimate relative to GT, from orientations only.',
                  'psi_D is the twist of R_rel R_ref^T: slow heading drift of the estimator relative to its mid-route mean.',
                  'effective_scale_from_residual projects the residual increment per 50 s bin onto the GT displacement: 1 + (dr.dp)/|dp|^2.',
                  'positional_ratio_15m is |d_aligned|/|d_gt| over 15 m GT-path windows (dt <= 30 s): a positional diagnostic, not orientation-derived.',
                  'All numbers are diagnostics; the official Score2D is unchanged.'],
    }
    save_json(sub / 'heading_integral.json', result)
    plot_heading_integral(ctx, sub, label, t, e_obs, e_pred, resid, err, psi_D, np.degrees(psi_s), aligned, p_gt, pivot, rows)
    return result, {'t': t, 'aligned': aligned, 'p_gt': p_gt, 'err': err, 'bins': rows, 't_w': t_w, 'ratio_w': ratio_w}


def shade_events(ax, ctx, cp_names=True):
    for name, w in ctx.cp_windows.items():
        ax.axvspan(w['first_s'], w['last_s'], color='0.85', alpha=0.6, lw=0)
        if cp_names:
            ax.text(0.5 * (w['first_s'] + w['last_s']), ax.get_ylim()[1], name, rotation=90, fontsize=5,
                    ha='center', va='top', color='0.35')
    for ep in ctx.sos:
        ax.axvline(ep['start_native_s'], color='goldenrod', lw=0.8, alpha=0.8)
    for commit in (ctx.commits.get('history') or []):
        ax.axvline(commit['timestamp_s'], color='purple', lw=0.8, ls=':', alpha=0.9)
    for change in ctx.world_versions[1:]:
        ax.axvline(change['t_s'], color='green', lw=0.8, ls='--', alpha=0.7)


def plot_heading_integral(ctx, sub, label, t, e_obs, e_pred, resid, err, psi_D, psi_M_deg, aligned, p_gt, pivot, rows):
    fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=True, constrained_layout=True)
    for k, (ax, comp) in enumerate(zip(axes[:2], ('east (x)', 'north (y)'))):
        ax.plot(t, e_obs[:, k], c='k', lw=0.9, label='observed error (saved CP Sim3)')
        ax.plot(t, e_pred[:, k], c='tab:blue', lw=0.9, label='yaw-only prediction from orientations')
        ax.plot(t, resid[:, k], c='tab:red', lw=0.8, alpha=0.8, label='residual = observed - predicted')
        ax.axhline(0, c='0.5', lw=0.5); ax.set_ylabel(f'{comp} error [m]'); ax.grid(alpha=0.2)
        ax.axvline(t[pivot], c='tab:green', lw=1.2, label='pivot (AA3814)' if k == 0 else None)
        ax.legend(fontsize=8, loc='upper left')
        shade_events(ax, ctx, cp_names=(k == 0))
    ax = axes[2]
    ax.plot(t, err, c='k', lw=0.9, label='|observed|')
    ax.plot(t, np.linalg.norm(resid, axis=1), c='tab:red', lw=0.8, label='|residual|')
    ax.set_ylabel('horizontal error [m]'); ax.grid(alpha=0.2); ax.legend(fontsize=8, loc='upper left')
    shade_events(ax, ctx, cp_names=False)
    ax = axes[3]
    ax.plot(t, psi_M_deg, c='tab:blue', lw=0.8, label='psi_M: heading error of aligned estimate vs GT (10 s median)')
    ax.plot(t, psi_D, c='tab:gray', lw=0.5, alpha=0.7, label='psi_D: heading drift vs mid-route mean (per frame)')
    ax.axhline(0, c='0.5', lw=0.5); ax.set_ylabel('heading [deg]'); ax.set_xlabel('native time [s]'); ax.grid(alpha=0.2)
    ax.legend(fontsize=8, loc='upper left')
    shade_events(ax, ctx, cp_names=False)
    fig.suptitle(f'{label}: heading-drift integral test. Grey = CP observation windows, gold = SOS starts, '
                 'purple dotted = calibration commits, green dashed = IMU-init gauge changes.\n'
                 'Healthy start: prediction tracks observed. Healthy end: residual carries the error (scale, not yaw).', fontsize=10)
    fig.savefig(sub / 'error_components_vs_time.png', dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 13), constrained_layout=True)
    origin = p_gt[:, :2].mean(0)
    ax.plot(p_gt[:, 0] - origin[0], p_gt[:, 1] - origin[1], c='0.6', lw=2, label='GT route')
    ax.plot(aligned[:, 0] - origin[0], aligned[:, 1] - origin[1], c='darkorange', lw=1, label='estimate, saved CP Sim3')
    step = max(1, len(t) // 120)
    ids = np.arange(0, len(t), step)
    ax.quiver(p_gt[ids, 0] - origin[0], p_gt[ids, 1] - origin[1], 10 * e_obs[ids, 0], 10 * e_obs[ids, 1],
              color='k', angles='xy', scale_units='xy', scale=1, width=0.002, label='observed error x10')
    ax.quiver(p_gt[ids, 0] - origin[0], p_gt[ids, 1] - origin[1], 10 * e_pred[ids, 0], 10 * e_pred[ids, 1],
              color='tab:blue', angles='xy', scale_units='xy', scale=1, width=0.002, alpha=0.8, label='yaw-only prediction x10')
    ax.set_aspect('equal'); ax.grid(alpha=0.2); ax.legend(fontsize=9)
    ax.set(xlabel='East offset [m]', ylabel='North offset [m]', title=f'{label}: observed vs orientation-predicted error along the route (arrows x10)')
    fig.savefig(sub / 'route_quiver_obs_vs_pred.png', dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 4.5), constrained_layout=True)
    ax.plot(t, psi_D, c='tab:gray', lw=0.5, alpha=0.6, label='psi_D per frame')
    bt = [0.5 * (r['bin_start_s'] + r['bin_end_s']) for r in rows]
    ax.plot(bt, [r['psi_D_mean_deg'] for r in rows], 'o-', c='tab:blue', ms=3, label='psi_D 50 s mean')
    ax.plot(bt, [r['swing_D_mean_deg'] for r in rows], 's-', c='tab:red', ms=3, label='tilt part (swing) 50 s mean')
    ax.axhline(0, c='0.5', lw=0.5); ax.set(xlabel='native time [s]', ylabel='deg', title=f'{label}: orientation drift relative to the 400-1400 s mean (yaw vs tilt)')
    ax.grid(alpha=0.2); ax.legend(fontsize=8)
    shade_events(ax, ctx, cp_names=False)
    fig.savefig(sub / 'yaw_vs_time.png', dpi=150); plt.close(fig)


# ----------------------------------------------------------------------------- stage 2
def read_lm_diag(log):
    """[LM_DIAG] JSON lines (same parse as make_lamaria_recovery_diagnostic.py)."""
    rows = []
    prefix = '[LM_DIAG] '
    with log.open(errors='replace') as handle:
        for line in handle:
            if line.startswith(prefix):
                row = json.loads(line[len(prefix):])
                cams = row.get('cams') or [{}, {}]
                rows.append((row['time'], row['frame'], row['state'], row['success'], row['inliers'],
                             row.get('local_points', np.nan), row.get('local_keyframes', np.nan),
                             cams[0].get('detected', np.nan), cams[1].get('detected', np.nan),
                             cams[0].get('before_opt', np.nan), cams[1].get('before_opt', np.nan),
                             cams[0].get('after_opt', np.nan), cams[1].get('after_opt', np.nan)))
    return np.array(rows, dtype=float)


def count_patterns(log, patterns):
    counts = {p: 0 for p in patterns}
    regexes = {p: re.compile(p, re.IGNORECASE) for p in patterns}
    with log.open(errors='replace') as handle:
        for line in handle:
            for p, rx in regexes.items():
                if rx.search(line):
                    counts[p] += 1
    return counts


def late_scale_internal(ctx, out, evaluation):
    sub = out / '02_late_scale_internal'
    sub.mkdir()
    stamps, p_exp, q_exp, _ = load_export_trajectory(ctx.trajectory_csv)
    R_exp = Rotation.from_quat(q_exp).as_matrix()
    t_exp = stamps * 1e-9
    maps = read_rows(ctx.maps_csv)[0]
    # maps.csv map_T_export: documented 'export coordinates to internal map', p_map = R_s p_export + t_s.
    # Direction verified empirically below against the online internal-frame snapshots.
    R_s = Rotation.from_quat([float(maps[k]) for k in ('map_T_export_qx', 'map_T_export_qy', 'map_T_export_qz', 'map_T_export_qw')]).as_matrix()
    t_s = np.array([float(maps[k]) for k in ('map_T_export_tx', 'map_T_export_ty', 'map_T_export_tz')])
    by_stamp = {int(s): i for i, s in enumerate(stamps)}
    final_wv = max(c['world_version'] for c in ctx.world_versions)
    joined = []
    for row in ctx.online:
        if int(row['world_version']) != final_wv or row['pose_available'] != '1' or row['state'] != '2' or row['coasting'] != '0':
            continue
        key = int(round(float(row['input_t_s']) * 1e9))
        i = by_stamp.get(key)
        if i is None:
            continue
        joined.append((i, *[float(row[k]) for k in ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')]))
    joined = np.array(joined)
    ids = joined[:, 0].astype(int)
    p_on, R_on = joined[:, 1:4], Rotation.from_quat(joined[:, 4:8]).as_matrix()
    p_pred = p_exp[ids] @ R_s.T + t_s
    pos_res = np.linalg.norm(p_pred - p_on, axis=1)
    rot_res = Rotation.from_matrix(np.einsum('nij,nkj->nik', R_s @ R_exp[ids], R_on)).magnitude() * 180 / np.pi
    alt = np.linalg.norm((p_exp[ids] - t_s) @ R_s + 0 - p_on, axis=1)   # reverse hypothesis p_map = R_s^T (p_exp - t_s)
    transform = {'documented_direction_median_residual_m': fnum(np.median(pos_res)), 'documented_direction_median_rotation_deg': fnum(np.median(rot_res)),
                 'reverse_direction_median_residual_m': fnum(np.median(alt)), 'joined_rows': int(len(ids)), 'world_version': int(final_wv),
                 'ok': bool(np.median(pos_res) < 1.0 and np.median(rot_res) < 1.0)}
    t_j = t_exp[ids]
    stretch_rows = []
    if transform['ok']:
        tw, ratio, heading = windowed_displacement_ratio(t_j, p_on @ R_s, p_exp[ids])   # lengths frame-free; heading in export frame
        for a, b, _ in bin_indices(t_exp):
            m = (tw >= a) & (tw < b)
            if m.sum() < 5:
                continue
            stretch_rows.append({'bin_start_s': a, 'bin_end_s': b, 'n_windows': int(m.sum()),
                                 'online_over_final_ratio_median': fnum(np.median(ratio[m])),
                                 'online_over_final_ratio_iqr': [fnum(np.percentile(ratio[m], 25)), fnum(np.percentile(ratio[m], 75))],
                                 'heading_online_minus_final_median_deg': fnum(np.median(heading[m]))})
    save_json(sub / 'online_vs_final_stretch.json', {**FLAGS, 'transform_check': transform, 'bins_50s': stretch_rows,
                                                    'note': 'online.csv poses are live internal-map snapshots before later BA; final export is post-BA. '
                                                            'A ratio of 1 everywhere means BA did not change local scale; a late-only deviation means BA rewrote the late segment.'})

    # Internal correlates.
    lm = read_lm_diag(ctx.log)
    kf_t = np.array([float(r['t_s']) for r in read_rows(ctx.keyframes_csv)])
    online_t = np.array([float(r['input_t_s']) for r in ctx.online])
    online_coast = np.array([int(r['coasting']) for r in ctx.online])
    online_state = np.array([int(r['state']) for r in ctx.online])
    imu = load_imu(ctx.run / 'input/euroc')
    gyro_deg = np.degrees(np.linalg.norm(imu[:, 1:4], axis=1))
    acc_norm = np.linalg.norm(imu[:, 4:7], axis=1)
    baby_acc_t = np.array([e['time'] for e in ctx.baby_events if e['event'] == 'accepted'])
    step = np.linalg.norm(np.diff(p_exp, axis=0), axis=1)
    eval_by_bin = {(r['bin_start_s'], r['bin_end_s']): r for r in evaluation['bins']}
    stretch_by_bin = {(r['bin_start_s'], r['bin_end_s']): r for r in stretch_rows}
    signals = []
    for a, b, _ in bin_indices(t_exp):
        m = (lm[:, 0] >= a) & (lm[:, 0] < b)
        if not m.any():
            continue
        im = (imu[:, 0] >= a) & (imu[:, 0] < b)
        sm = (t_exp[1:] >= a) & (t_exp[1:] < b) & (np.diff(t_exp) < 0.5)
        om = (online_t >= a) & (online_t < b)
        ev = eval_by_bin.get((a, b), {})
        st = stretch_by_bin.get((a, b), {})
        signals.append({
            'bin_start_s': a, 'bin_end_s': b, 'frames': int(m.sum()),
            'keyframes': int(((kf_t >= a) & (kf_t < b)).sum()),
            'inliers_median': fnum(np.nanmedian(lm[m, 4])), 'local_points_median': fnum(np.nanmedian(lm[m, 5])),
            'local_keyframes_median': fnum(np.nanmedian(lm[m, 6])),
            'cam0_detected_median': fnum(np.nanmedian(lm[m, 7])), 'cam1_detected_median': fnum(np.nanmedian(lm[m, 8])),
            'cam0_before_opt_median': fnum(np.nanmedian(lm[m, 9])), 'cam1_before_opt_median': fnum(np.nanmedian(lm[m, 10])),
            'cam0_after_opt_median': fnum(np.nanmedian(lm[m, 11])), 'cam1_after_opt_median': fnum(np.nanmedian(lm[m, 12])),
            'tracking_failures': int((lm[m, 3] == 0).sum()), 'state3_frames': int((online_state[om] == 3).sum()),
            'coasting_frames': int(online_coast[om].sum()),
            'speed_mps': fnum(step[sm].sum() / np.diff(t_exp)[sm].sum()) if sm.any() else None,
            'accel_norm_std_mps2': fnum(acc_norm[im].std()) if im.any() else None,
            'gyro_mean_deg_s': fnum(gyro_deg[im].mean()) if im.any() else None,
            'baby_accepted_updates': int(((baby_acc_t >= a) & (baby_acc_t < b)).sum()),
            'evaluation_positional_ratio_15m_median': ev.get('positional_ratio_15m_median'),
            'evaluation_median_horizontal_error_m': ev.get('median_abs_e_obs_m'),
            'online_over_final_ratio_median': st.get('online_over_final_ratio_median'),
        })
    loop_patterns = [r'loop detected', r'merge detected', r'\[LOOP\]', r'\[MERGE\]', r'loop closure', r'MergeLocal', r'CorrectLoop']
    ledger = {
        'calibration_commits_s': [c['timestamp_s'] for c in (ctx.commits.get('history') or [])],
        'loop_or_merge_marker_counts': count_patterns(ctx.log, loop_patterns),
        'sos_episodes': [{'start_s': e['start_native_s'], 'end_s': e.get('end_native_s'), 'accepted': e['accepted_updates'],
                          'rejected': e['rejected_updates'], 'outcome': e.get('outcome')} for e in ctx.sos],
        'world_version_changes': ctx.world_versions, 'imu_initialized_first_s': ctx.imu_init_time,
        'baby_malformed_events': len(ctx.baby_malformed),
    }
    # Ranking / uniqueness test for the 1700-1750 s bin.
    def rank(key, reverse=False, subset=None):
        pool = [s for s in signals if s[key] is not None and (subset is None or subset(s))]
        pool.sort(key=lambda s: s[key], reverse=reverse)
        return pool
    target = next((s for s in signals if s['bin_start_s'] == 1700.), None)
    uniqueness = None
    if target:
        late = lambda s: s['bin_start_s'] >= 1650
        by_points, by_points_late = rank('local_points_median'), rank('local_points_median', subset=late)
        by_speed = rank('speed_mps', reverse=True)
        starved = [s for s in signals if s['local_points_median'] is not None and s['local_points_median'] <= target['local_points_median'] and s['bin_start_s'] < 1650]
        uniqueness = {
            'bin_1700_1750_local_points_median': target['local_points_median'],
            'rank_by_local_points_all_bins_ascending': 1 + [s['bin_start_s'] for s in by_points].index(1700.),
            'rank_by_local_points_late_bins_ascending': 1 + [s['bin_start_s'] for s in by_points_late].index(1700.),
            'rank_by_speed_all_bins_descending': 1 + [s['bin_start_s'] for s in by_speed].index(1700.),
            'mid_route_bins_equally_or_more_starved': [{'bin_start_s': s['bin_start_s'], 'local_points_median': s['local_points_median'],
                                                        'evaluation_positional_ratio_15m_median': s['evaluation_positional_ratio_15m_median'],
                                                        'evaluation_median_horizontal_error_m': s['evaluation_median_horizontal_error_m']} for s in starved],
        }
    mid = [s['online_over_final_ratio_median'] for s in signals if 200 <= s['bin_start_s'] < 1650 and s['online_over_final_ratio_median'] is not None]
    late_r = [s['online_over_final_ratio_median'] for s in signals if s['bin_start_s'] >= 1700 and s['online_over_final_ratio_median'] is not None]
    pattern = 'inconclusive'
    if transform['ok'] and mid and late_r:
        if max(abs(np.array(mid + late_r) - 1)) < 0.01:
            pattern = 'A_scale_error_born_in_tracking'
        elif max(abs(np.array(mid) - 1)) < 0.01 and max(abs(np.array(late_r) - 1)) > 0.03:
            pattern = 'B_BA_rewrote_late_segment'
        elif max(abs(np.array(mid) - 1)) >= 0.01:
            pattern = 'BROKEN_online_reference_wanders_mid_route'
    save_json(sub / 'internal_signals.json', {**FLAGS, 'bins_50s': signals, 'event_ledger': ledger, 'uniqueness_test_1700_1750': uniqueness,
                                              'online_vs_final_pattern': pattern,
                                              'online_vs_final_mid_route_max_abs_dev': fnum(max(abs(np.array(mid) - 1))) if mid else None,
                                              'online_vs_final_late_max_abs_dev': fnum(max(abs(np.array(late_r) - 1))) if late_r else None})
    plot_internal(ctx, sub, signals, stretch_rows, transform)
    return {'transform': transform, 'pattern': pattern, 'ledger': ledger, 'uniqueness': uniqueness, 'signals': signals}


def plot_internal(ctx, sub, signals, stretch_rows, transform):
    bt = np.array([0.5 * (s['bin_start_s'] + s['bin_end_s']) for s in signals])
    def col(key):
        return np.array([np.nan if s[key] is None else s[key] for s in signals], dtype=float)
    fig, axes = plt.subplots(7, 1, figsize=(16, 18), sharex=True, constrained_layout=True)
    panels = [('keyframes', 'keyframes / 50 s'), ('inliers_median', 'inliers (median)'), ('local_points_median', 'local map points (median)'),
              ('local_keyframes_median', 'local keyframes (median)'), ('speed_mps', 'walking speed [m/s]'),
              ('accel_norm_std_mps2', 'accel |a| std [m/s^2]'), ('gyro_mean_deg_s', 'gyro |w| mean [deg/s]')]
    for ax, (key, ylabel) in zip(axes, panels):
        ax.plot(bt, col(key), 'o-', ms=3, lw=1, c='tab:blue'); ax.set_ylabel(ylabel, fontsize=8); ax.grid(alpha=0.2)
        shade_events(ax, ctx, cp_names=False)
    ax2 = axes[2].twinx()
    ax2.plot(bt, col('evaluation_positional_ratio_15m_median'), 's--', ms=3, lw=1, c='tab:red', label='evaluation: |d_est|/|d_gt| 15 m (comparison only)')
    ax2.axhline(1, c='tab:red', lw=0.5, ls=':'); ax2.set_ylabel('est/GT displacement ratio', color='tab:red', fontsize=8); ax2.legend(fontsize=7, loc='upper left')
    ax3 = axes[1].twinx()
    ax3.bar(bt, col('baby_accepted_updates'), width=40, color='goldenrod', alpha=0.5, label='accepted Baby updates / 50 s')
    ax3.set_ylabel('Baby updates', color='goldenrod', fontsize=8); ax3.legend(fontsize=7, loc='upper left')
    axes[-1].set_xlabel('native time [s]')
    fig.suptitle('Internal tracking/IMU signals per 50 s (GT-free) with the evaluation scale ratio overlaid for comparison only.\n'
                 'Grey = CP windows, gold = SOS starts, purple dotted = calibration commits, green dashed = IMU-init gauge changes.', fontsize=10)
    fig.savefig(sub / 'internal_signals_vs_time.png', dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 4.5), constrained_layout=True)
    if stretch_rows:
        sbt = [0.5 * (r['bin_start_s'] + r['bin_end_s']) for r in stretch_rows]
        med = np.array([r['online_over_final_ratio_median'] for r in stretch_rows])
        lo = np.array([r['online_over_final_ratio_iqr'][0] for r in stretch_rows]); hi = np.array([r['online_over_final_ratio_iqr'][1] for r in stretch_rows])
        ax.fill_between(sbt, lo, hi, color='tab:blue', alpha=0.2, label='IQR')
        ax.plot(sbt, med, 'o-', ms=3, c='tab:blue', label='online/final displacement ratio, 15 m windows, 50 s median')
    ax.plot(bt, col('evaluation_positional_ratio_15m_median'), 's--', ms=3, c='tab:red', label='evaluation est/GT ratio (comparison only)')
    ax.axhline(1, c='0.4', lw=0.6); ax.set(xlabel='native time [s]', ylabel='ratio',
                                          title=f'Live (pre-BA) versus final (post-BA) local stretch. Transform check: median residual '
                                                f'{transform["documented_direction_median_residual_m"]:.3f} m, {transform["documented_direction_median_rotation_deg"]:.3f} deg')
    ax.grid(alpha=0.2); ax.legend(fontsize=8)
    shade_events(ax, ctx, cp_names=False)
    fig.savefig(sub / 'online_vs_final_stretch_vs_time.png', dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 7), constrained_layout=True)
    x, y = col('local_points_median'), col('evaluation_positional_ratio_15m_median')
    late = np.array([s['bin_start_s'] >= 1650 for s in signals])
    ax.scatter(x[~late], y[~late], c='tab:blue', s=30, label='bins before 1650 s')
    ax.scatter(x[late], y[late], c='tab:red', s=40, label='bins from 1650 s')
    for xi, yi, s in zip(x, y, signals):
        if np.isfinite(xi) and np.isfinite(yi):
            ax.annotate(f'{int(s["bin_start_s"])}', (xi, yi), fontsize=6, xytext=(2, 2), textcoords='offset points')
    ax.axhline(1, c='0.4', lw=0.6); ax.grid(alpha=0.2); ax.legend(fontsize=8)
    ax.set(xlabel='local map points (median per 50 s bin)', ylabel='evaluation est/GT displacement ratio (15 m windows)',
           title='Starvation versus local scale error, one point per 50 s bin')
    fig.savefig(sub / 'starvation_vs_scale_scatter.png', dpi=150); plt.close(fig)


# ----------------------------------------------------------------------------- main
def write_readme(out, ctx, pre, stage1, stage1_ref, stage2):
    lines = [
        '# Startup / drift / tilt decomposition (read-only, Stages 0-2)', '',
        f'Source run: {ctx.run}', f'Official Score2D (unchanged): {ctx.score["Score2D"]}',
        f'Ground truth (evaluation only): {ctx.gt_dense}', f'Sparse GT / CP windows: {ctx.gt_sparse}',
        f'Solver gravity source: {ctx.solver_result}', '',
        '## Conventions', (__doc__ or '').split('Conventions')[1].strip(), '',
        '## Stage 0 preflight', f'- all input hashes match the saved score provenance; official toolkit sources unchanged',
        f'- cam0 rebuild max |dp| {pre["selftest"]["cam0_rebuild_max_dp_m"]:.2e} m; official error reproduction max diff {pre["selftest"]["official_error_reproduction_max_diff_m"]:.2e} m',
        f'- matched GT {pre["selftest"]["matched_gt"]}, unmatched {pre["selftest"]["unmatched_gt"]} (all before first estimate: {pre["selftest"]["unmatched_before_first_estimate"]})', '',
        '## Stage 1 heading-drift integral (01_heading_integral/, 01_heading_integral_refined/)',
        'Healthy start: the yaw-only prediction (orientations only) tracks the observed error; healthy end: the residual carries the error (scale, not yaw).',
        f'- source run verdict: {json.dumps(stage1["verdict"])}',
    ]
    if stage1_ref:
        lines.append(f'- refinement verdict: {json.dumps(stage1_ref["verdict"])}')
    lines += ['', '## Stage 2 late scale, GT-free (02_late_scale_internal/)',
              f'- export-to-internal transform check: {json.dumps(stage2["transform"])}',
              f'- online vs final pattern: {stage2["pattern"]}',
              f'- calibration commits (s): {stage2["ledger"]["calibration_commits_s"]}; loop/merge markers: {stage2["ledger"]["loop_or_merge_marker_counts"]}', '',
              '## Files', *[f'- {p}' for p in sorted(str(p) for p in out.rglob('*') if p.is_file())], '',
              'All alternative quantities are labelled diagnostics; no scores.json was written and the official score is unchanged.']
    (out / 'README.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run', type=Path, required=True, help='Scored run directory (contains lamaria_score/)')
    parser.add_argument('--refinement-run', type=Path, help='Optional second scored run with identical timestamps (Stage 1 only)')
    parser.add_argument('--gt-dense', type=Path, default=Path('/media/raghav/HardDrive1/Mecka/lamaria/sequence_3_17/gt_dense.txt'))
    parser.add_argument('--gt-sparse', type=Path, default=Path('/media/raghav/HardDrive1/Mecka/lamaria/sequence_3_17/gt_sparse.json'))
    parser.add_argument('--solver-result', type=Path, required=True, help='Offline VI result.json carrying the export-frame gravity')
    parser.add_argument('--out', type=Path, required=True, help='Fresh output directory; refused if it exists')
    args = parser.parse_args()
    out = args.out.resolve()
    if out.exists():
        raise FileExistsError(f'Preserving previous diagnostics: {out}')
    for forbidden in ('baselines', 'vendor', 'third_party', 'lamaria_score', 'INSV_STITCHING'):
        if forbidden in out.parts:
            raise ValueError(f'Refusing to write diagnostics under {forbidden}')
    sys.path.insert(0, str(TOOLKIT))
    ctx = Context(args)
    if out.is_relative_to(ctx.run):
        raise ValueError('Diagnostics must not be written inside the scored run directory')
    out.mkdir(parents=True)
    shutil.copy2(HERE, out / 'decompose_lamaria_errors.py')
    pre = preflight(ctx, out)
    stage1, evaluation = heading_integral(ctx, ctx.score, pre['s_stamps'], pre['s_p'], pre['s_R'], out / '01_heading_integral', 'BabyFeatures Long 43.73 (source run)')
    stage1_ref = None
    if ctx.refinement:
        r_score = json.loads((ctx.refinement / 'lamaria_score/scores.json').read_text())
        r_stamps, r_p, r_R = load_pose_table(ctx.refinement / 'lamaria_score/estimated_cam0_ns.txt')
        if not np.array_equal(r_stamps, pre['s_stamps']):
            raise ValueError('Refinement run timestamps differ from the source run')
        stage1_ref, _ = heading_integral(ctx, r_score, r_stamps, r_p, r_R, out / '01_heading_integral_refined', 'fixed-camera VI refinement 45.03')
    stage2 = late_scale_internal(ctx, out, evaluation)
    write_readme(out, ctx, pre, stage1, stage1_ref, stage2)
    save_json(out / 'provenance.json', {
        **FLAGS, 'script_sha256': sha(HERE), 'script_copy': str(out / 'decompose_lamaria_errors.py'),
        'run': str(ctx.run), 'refinement_run': str(ctx.refinement) if ctx.refinement else None,
        'preflight': str(out / '00_preflight/provenance.json'), 'official_Score2D': ctx.score['Score2D'],
        'stage1_verdict': stage1['verdict'], 'stage1_refined_verdict': stage1_ref['verdict'] if stage1_ref else None,
        'stage2_pattern': stage2['pattern'], 'stage2_transform_check': stage2['transform'],
    })
    print(json.dumps({'out': str(out), 'preflight': pre['selftest']['passed'], 'stage1': stage1['verdict'],
                      'stage1_refined': stage1_ref['verdict'] if stage1_ref else None,
                      'stage2_pattern': stage2['pattern'], 'stage2_transform': stage2['transform']}, indent=2))


if __name__ == '__main__':
    main()
