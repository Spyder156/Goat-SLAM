Full native online-calibration candidate v2 (experimental; not default)

Base: orbslam3_lamaria_online_radial with fixed-intrinsics mode 1 and 12 s
initialization (the 67.19-scoring control configuration). Third-party source
is read-only; candidate_provenance.json pins staging and base hashes.

Changes:
- Failed inertial tracking transactions restore the captured pose, velocity
  and bias together, discard the rejected marginal prior, and flag rejected
  image associations. The v2 capture boundary is optimizer entry; a remaining
  visual-relatch boundary defect is documented below. Rejected frames cannot
  create a new keyframe/fixed inertial anchor from fresh stereo points.
- After VI-BA2, every 60 s of native input time, optimize a private full map
  using native Fisheye624 visual factors and native IMU + bias random walks.
  Input pauses while this past-only solve runs; no frames are skipped.
- Both cameras release all native 15 intrinsic DOFs: f, cx/cy, six radial,
  two tangential and four prism terms. fx=fy within each camera, independent
  focal lengths between cameras. Physical stereo/body rig stays fixed.
- Factory-centred priors remain fixed for the entire run. Every LM update
  clips physical increments to 5% of the per-parameter lifetime bound.
  See configs/orbslam3_lamaria/online_full_20261003/experiment.json for values.
- After ten warm-up iterations, twenty-iteration fixed-camera and free-camera
  branches from the exact same graph snapshot use identical held-out measurements. Acceptance requires centre and
  periphery residual/inlier retention, improvement, valid inverse domain,
  finite states/biases and parameters away from hard bounds.
- All camera parameters and full geometry commit atomically while image
  consumption, loop readers/GBA and map writers are quiesced. Intrinsic and
  stereo caches refresh; old frame marginal priors are invalidated, and the
  next tracking optimizer rebuilds from valid keyframe/preintegration.
- Bootstrap two-view K cache recreates on each bootstrap after calibration.

Contract validation against final built library:
  build/orbslam3_lamaria_online_full_v2/tests/domain_admission/results.json
42 checks pass. Intrinsic/pose Jacobian relative error below 2.5e-9.
986 unused synthetic projections: median 0.67777 -> 0.004757 pixels.
These synthetic checks validate mechanics, not real sequence accuracy.

Full Short replay:
  experiments/lamaria_online_full_v2_short_full_20261003
Launched under GDB, 18,351 supported inputs, 917.5 seconds, four CPUs.
No ground truth enters estimator. Completed all18,351 inputs, exited0.
Score2D80.199161; CP14/14 and GT2999/2999 recovered, one map;
18,343 poses, eight supported startup inputs without estimates.
CP Sim3scale1.019443; horizontal median0.401453m/RMSE0.463521m.
Runtime29min46s. Two accepted updates in15 trials.
Online arm contains both calibration and tracking lifecycle repairs, so its
score difference cannot be attributed exclusively to intrinsic calibration.

Additional v2 corrections before replay:
- Keep valid two-observation tracks, including same-frame metric stereo.
- Preserve fixed factory physical mask centre while cx/cy are optimized.
  Distinguish numerical inverse failures from angular-admission changes; test
  full-cone monotonic radial mapping, tangential/prism determinant, and native
  project/unproject consistency. Explicit rejection gates are recorded.
- Frame velocity transports optimized anchor velocity, not only rotation;
  temporal preintegration bias caches follow updated anchor states.
- Unsupported single-view points retain range but receive updated native
  bearings from observations, rather than retaining old-calibration rays.
- v1 original source/build/run remain preserved and are not relabeled v2.

Known remaining rollback boundary defect in this preserved v2:
A successful visual relatch can replace pose before the local-map snapshot.
On failure, v2 may restore that rejected relatch pose rather than the IMU
prediction (actual2.72m/2.98m double jump around549.74s). The isolated v3
repair moves capture to PredictStateIMU. The14.44m diagnostic at554.99s
was an internal relatch/optimizer undo, not an actual trajectory jump.
See the suite diagnostics/recovery_diagnostic.png and.json for evidence.
