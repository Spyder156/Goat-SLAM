Full native online-calibration candidate (experimental; not default)

Base: orbslam3_lamaria_online_radial with fixed-intrinsics mode 1 and 12 s
initialization (the 67.19-scoring control configuration). Third-party source
is read-only; candidate_provenance.json pins staging and base hashes.

Changes:
- Failed inertial tracking transactions restore propagated pose, velocity and
  bias together, discard the rejected marginal prior, and flag rejected image
  associations. Rejected frames retain their propagated history but cannot
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
- A ten-iteration fixed-camera reference and twenty-iteration free-camera
  solve use identical held-out measurements. Acceptance requires centre and
  periphery residual/inlier retention, improvement, valid inverse domain,
  finite states/biases and parameters away from hard bounds.
- All camera parameters and full geometry commit atomically while image
  consumption, loop readers/GBA and map writers are quiesced. Intrinsic and
  stereo caches refresh; old frame marginal priors are invalidated, and the
  next tracking optimizer rebuilds from valid keyframe/preintegration.
- Bootstrap two-view K cache recreates on each bootstrap after calibration.

Contract validation against final built library:
  build/orbslam3_lamaria_online_full/tests/full_final/results.json
30 checks pass. Intrinsic/pose Jacobian relative error below 2.5e-9.
986 unused synthetic projections: median 0.67777 -> 0.004757 pixels.
These synthetic checks validate mechanics, not real sequence accuracy.

Full Short replay:
  experiments/lamaria_online_full_short_full_20261003
Launched under GDB, 18,351 supported inputs, 917.5 seconds, four CPUs.
No ground truth enters estimator. Final diagnostic score:48.068151090242395, CP11/14, GT2424/2999;
two atlas maps. Largest-map export omits575 GT timestamps.
No calibration candidate was accepted; this is a negative control, not an
implemented-calibration accuracy success. It is superseded by v2.
Online arm contains both calibration and tracking lifecycle repairs, so its
score difference cannot be attributed exclusively to intrinsic calibration.
