#!/usr/bin/env python3
"""Compare two runs of the same configuration: are they byte-identical, pose-identical, or merely close?

Reads each run's atlas trajectory CSV (t_s, map_id, ..., tx, ty, tz, qx, qy, qz, qw) and official scores.
Reports: common timestamps, max position difference (m), max rotation difference (deg), map counts,
keyframe counts, Score2D of each. Read-only."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def load(run):
    path = next(Path(run).glob('atlas_*_trajectory.csv'))
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r['t_s']) for r in rows]); m = np.array([int(r['map_id']) for r in rows])
    p = np.array([[float(r[k]) for k in ('tx', 'ty', 'tz')] for r in rows])
    q = np.array([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in rows])
    kf = sum(1 for _ in open(next(Path(run).glob('atlas_*_keyframes.csv')))) - 1
    score = json.load(open(Path(run) / 'lamaria_score/scores.json'))['Score2D'] if (Path(run) / 'lamaria_score/scores.json').exists() else None
    return t, m, p, q, kf, score


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run_a'); ap.add_argument('run_b')
    a = ap.parse_args()
    ta, ma, pa, qa, kfa, sa = load(a.run_a); tb, mb, pb, qb, kfb, sb = load(a.run_b)
    common, ia, ib = np.intersect1d(np.round(ta, 6), np.round(tb, 6), return_indices=True)
    dp = np.linalg.norm(pa[ia] - pb[ib], axis=1)
    dr = (Rotation.from_quat(qa[ia]).inv() * Rotation.from_quat(qb[ib])).magnitude() * 180 / np.pi
    report = {
        'poses_a': int(len(ta)), 'poses_b': int(len(tb)), 'common_timestamps': int(len(common)),
        'maps_a': int(len(np.unique(ma))), 'maps_b': int(len(np.unique(mb))), 'keyframes_a': kfa, 'keyframes_b': kfb,
        'max_position_difference_m': float(dp.max()) if len(dp) else None, 'median_position_difference_m': float(np.median(dp)) if len(dp) else None,
        'max_rotation_difference_deg': float(dr.max()) if len(dr) else None,
        'byte_identical_trajectories': bool(len(ta) == len(tb) and np.array_equal(pa, pb) and np.array_equal(qa, qb)),
        'score2d_a': sa, 'score2d_b': sb,
    }
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
