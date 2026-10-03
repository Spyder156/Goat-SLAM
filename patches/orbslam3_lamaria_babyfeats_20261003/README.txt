BabyFeatures SOS / frozen online-full v2 parent
================================================

Branch: experiments/BabyFeats
Purpose: one full Long continuity experiment; preserve the successful v2
normal estimator. This is a bounded first implementation, not a guarantee
of permanent continuity or a replacement for the preserved baseline.

Behavior
--------
LAMARIA_BABY_FEATURES=1 enables a private passive track cache. It uses the
same cached ALIKED-derived binary descriptors, per-camera mutual Hamming
matching (distance <=70, ratio <0.8), and at most seven historical Frames.
The cache owns its interval preintegrations; it owns no live MapPoints or
marginal priors. Existing normal map association and estimator factors are
unchanged. Background work can change runtime/thread scheduling.

Only a failed ordinary tracking update on an initialized, coast-armed map
activates the temporary VI graph. An at-most-eight-frame graph uses native
Fisheye624 projection/unprojection, general-ray DLT, EdgeMono, EdgeInertial,
gyro/accelerometer random walks, temporary XYZ points, and pose/velocity/
bias states. Already accepted normal states and the oldest rolling state
are fixed anchors. Healthy-state optimization does not use Baby factors.

Temporary XYZ admission needs >=0.25 degree ray parallax. After robust
optimization, >=15 distinct current tracks must have at least two verified
observations at the native 5.991 normalized residual bound. Finite-state,
gross-motion, rotation and active-IMU residual checks precede state commit.
Rejected solves do not mutate caller frames. These are new SOS acceptance
checks, not relaxed ordinary-map acceptance thresholds.

Accepted SOS poses remain RECENTLY_LOST/coasting, with separate BABY_DIAG
records. Only visually accepted SOS updates refresh ordinary five-second
grace. An episode has a ten-second hard limit. Pure inertial propagation
does not count as BabyFeature success or indefinitely renew that timer.

For forward routes, >=50 verified temporary points with >=3 views and
>=1 degree parallax, after >=3 consecutive accepted solves and >=0.15 s
in SOS, can seed a standard temporal keyframe in the same map. The existing
keyframe/IMU chain is used, and point ownership is checked before insertion.
Actual next-frame normal map tracking must confirm the return. Frame-only
observations are not fabricated as keyframe observations; initially each
promoted point has its current keyframe observation and the separately
verified temporary frame history.

On accepted normal-map return, the retained window can be refined with
the returning map pose fixed. Revised retained SOS poses update native
frame history before export. Different atlas origins are never joined by
the exporter or by GT. The normal recent-loss acceptance policy is inherited
unchanged; reports must show the actual mapped inlier count on return.

Limitations
-----------
- A fixed oldest state approximates the rolling prior; this is not a full
  uncertainty-preserving marginalization backend.
- At most eight frames are smoothed. An entire longer blind interval is not
  retrospectively optimized merely because its endpoint returns to the map.
- No optical-flow frontend, semantic motion mask, or new feature extraction
  is added. Pure rotation/insufficient parallax supplies no temporary XYZ
  constraints. Complete visual blindness remains IMU-only.
- Backend map/calibration changes flush the private window to avoid using
  stale gauges/camera geometry. This can temporarily remove SOS support.
- Normal v2's recovery acceptance policy and periodic calibration schedule
  are retained; this experiment does not claim to repair every prior issue.
- Early failures before valid initialized/coast-armed inertial tracking still
  follow the existing initialization/reset policy.

Reproduction
------------
geometry.patch = frozen v2 vendor patch followed by the Baby Tracking delta.
new_files retains v2's added files plus the new Baby headers/solver. The custom
builder also compiles the extra solver translation unit, which is not in the
historical vendor CMake object list. Use this builder, not the unextended
generic build_lamaria.py, for the candidate:

python3 patches/orbslam3_lamaria_babyfeats_20261003/build_incremental.py --jobs 2

It verifies original vendor hashes, frozen v2 source/header/product/object
hashes, and Docker image identity. Existing headers/class layouts must remain
identical. It recompiles Tracking.cc and LamariaBabySolver.cc and links only
inside build/orbslam3_lamaria_babyfeats_20261003. Baseline products are untouched.

Native numerical contract (fresh output directory required):

python3 patches/orbslam3_lamaria_babyfeats_20261003/tests/run_contract.py \
  --build build/orbslam3_lamaria_babyfeats_20261003 \
  --out build/orbslam3_lamaria_babyfeats_20261003/tests/NEW_CONTRACT_NAME

Full Long experiment, official score, and diagnostic plots:

/home/raghav/miniconda3/envs/lamaria/bin/python -B \
  pipeline/run/babyfeats_experiment.py

The harness refuses existing output; pass --out for a separately authorized
repeat. --render optionally creates a full evaluation-aligned Rerun. Long
settings are byte-identical to the frozen v2 native Long settings. The runtime
switch is explicit in the saved command and provenance. GT is evaluation-only.

Evaluate against native v2 Long 34.458901 / five retained maps. Historical
continuous Long 49.881534 is an additional preservation reference, with different
starting lens calibration; it is not a matched ablation. Process completion,
SOS acceptance, mature-map return, map continuity and score are separate claims.
