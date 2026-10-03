"""Validated access to the estimator's poses, for every offline analysis.

WHY THIS EXISTS. Offline scripts used to rebuild the world->lens transform from
the trajectory file and pair it with an image by "nearest pose within 50 ms".
Both halves were wrong:

  * `f_orb.txt` holds only the SURVIVING map. ORB-SLAM3's atlas resets (7-17
    times in these runs, "IMU is not or recently initialized") throw away
    everything before the last reset, so whole seconds have NO pose. A
    nearest-match then silently returns a pose from a DISCARDED map, and the
    map projected with it lands anywhere. That produced a "broken line map"
    that was entirely an artefact of the analysis.
  * The estimator's own frame ids do not equal dataset frame numbers, so
    keying on frame id mispaired images with poses.

So: key on TIMESTAMP exactly, refuse frames with no pose, and -- when the run
carries `frame_pose_dump.csv` (the pose the estimator actually used, per lens)
-- VERIFY the reconstruction against it before any figure is trusted.

CONVENTIONS
  f_orb.txt      : t[ns] tx ty tz qx qy qz qw, BODY pose T_b0_b (the b0 frame is
                   the first keyframe of the biggest map, gravity-aligned).
                   ml_orb.csv / mp_orb.csv are in the SAME b0 frame.
  T_b_c0         : IMU.T_b_c1 in the settings file, camera -> body.
  T_b_c1         : T_b_c0 @ Rig.T_c0_c1  (rear lens).
  returned Tcw   : 4x4, b0 world -> OBSERVING LENS frame, right-handed, metres.
  frame_pose_dump: T_cw in the estimator's RAW world frame. It differs from the
                   b0 frame by a pure rotation, so RELATIVE motion between two
                   frames is directly comparable and absolute rotation is not.
"""
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

TS_TOL_S = 1e-6      # timestamps are written from the same double; demand equality


def _mat4(txt, key):
    m = re.search(rf"{re.escape(key)}:.*?data:\s*\[(.*?)\]", txt, re.S)
    if not m:
        return None
    v = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", m.group(1))]
    return np.array(v).reshape(4, 4)


def T_b_lens(cfg, cam):
    """camera -> body, for lens `cam` (0 front, 1 rear)."""
    txt = Path(cfg).read_text()
    T = _mat4(txt, "IMU.T_b_c1")
    if T is None:
        raise ValueError(f"{cfg}: no IMU.T_b_c1")
    if cam == 1:
        R = _mat4(txt, "Rig.T_c0_c1")
        if R is None:
            raise ValueError(f"{cfg}: no Rig.T_c0_c1")
        T = T @ R
    return T


class Poses:
    """Timestamp -> T_cw, with no silent nearest-match."""

    def __init__(self, run, cfg):
        run = Path(run)
        tj = np.loadtxt(run / "f_orb.txt")
        self.t = tj[:, 0] / 1e9
        self.xyz = tj[:, 1:4]
        self.quat = tj[:, 4:8]                    # qx qy qz qw
        self.cfg = cfg
        self.span = (self.t.min(), self.t.max())
        # gaps > 1.5 frame periods mean a reset chopped the trajectory
        d = np.diff(self.t)
        self.gaps = int((d > 0.05).sum())

    def index(self, ts):
        """Row for this exact timestamp, or None. NEVER a nearest match."""
        j = int(np.argmin(np.abs(self.t - ts)))
        return j if abs(self.t[j] - ts) <= TS_TOL_S else None

    def Twb(self, ts):
        j = self.index(ts)
        if j is None:
            return None
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(self.quat[j]).as_matrix()
        T[:3, 3] = self.xyz[j]
        return T

    def Tcw(self, ts, cam):
        """b0 world -> lens `cam`. None when this time has no pose."""
        Twb = self.Twb(ts)
        if Twb is None:
            return None
        return np.linalg.inv(Twb @ T_b_lens(self.cfg, cam))

    def covered(self, times):
        """Subset of `times` that actually has a pose."""
        return np.array([t for t in times if self.index(t) is not None])


def validate(run, cfg, verbose=True):
    """Diff this reconstruction against the pose the estimator actually used.

    Compares RELATIVE motion between consecutive dumped frames, which is
    immune to the b0-vs-raw-world rotation and to any later BA drift. Returns
    (n_compared, median_rot_deg, median_trans_m). Raises if the run has no
    pose dump to check against.
    """
    run = Path(run)
    pf = run / "frame_pose_dump.csv"
    if not pf.exists():
        raise FileNotFoundError(f"{pf} missing: rerun with Lines.dumpFrom/dumpTo set")
    D = np.atleast_1d(np.genfromtxt(pf, delimiter=",", names=True))
    P = Poses(run, cfg)
    out = {}
    for cam in (0, 1):
        rows = sorted([r for r in D if int(r["cam"]) == cam], key=lambda r: r["t"])
        dR, dT = [], []
        for a, b in zip(rows[:-1], rows[1:]):
            Ma, Mb = P.Tcw(a["t"], cam), P.Tcw(b["t"], cam)
            if Ma is None or Mb is None:
                continue                      # no pose: correctly skipped
            def slam(r):
                M = np.eye(4)
                M[:3, :3] = [[r["r00"], r["r01"], r["r02"]],
                             [r["r10"], r["r11"], r["r12"]],
                             [r["r20"], r["r21"], r["r22"]]]
                M[:3, 3] = [r["tx"], r["ty"], r["tz"]]
                return M
            E = (Mb @ np.linalg.inv(Ma)) @ np.linalg.inv(slam(b) @ np.linalg.inv(slam(a)))
            dR.append(np.degrees(np.arccos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1))))
            dT.append(np.linalg.norm(E[:3, 3]))
        out[cam] = (len(dR), float(np.median(dR)) if dR else np.nan,
                    float(np.median(dT)) if dT else np.nan)
        if verbose:
            n, r, t = out[cam]
            print(f"  cam{cam}: {n:5d} frames validated   "
                  f"rel-rotation err median {r:.5f} deg   "
                  f"rel-translation err median {t*1000:.4f} mm")
    if verbose:
        n_dump = len(set(D["t"]))
        cov = len(P.covered(sorted(set(D["t"]))))
        print(f"  trajectory covers {cov}/{n_dump} dumped frames "
              f"({100*cov/max(n_dump,1):.1f}%); internal gaps: {P.gaps}")
    return out


if __name__ == "__main__":
    import sys
    print(f"validating {sys.argv[1]}")
    validate(sys.argv[1], sys.argv[2])
