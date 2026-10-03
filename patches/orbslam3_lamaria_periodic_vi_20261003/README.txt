Periodic full-map fixed-intrinsic VI candidate (2026-10-03)

Forked from preserved orbslam3_lamaria_online_full_v2. Not a new default.
Neither third_party, the frozen v2 source/build nor prior run artifacts changed.

Config: Calibration.periodicVIMode
  0 (default): original v2 single-map full calibration and global cadence.
  1: full calibration while one populated map exists; fixed-intrinsic full VI
     on the active map once multiple maps exist (also fixed if fullOnline=0).
  2: periodic fixed-intrinsic full VI from initial BA2 onward, including with
     one map. This is the experiment C control requested for Medium and Long.
Calibration.intervalSeconds retains its original value and validation.
New modes use a last-trial clock keyed by (map ID, initial keyframe ID), so a
newly initialized/reset map is not starved by a different map's recent solve.

Exact existing native visual/IMU/bias graph, fixed first-pose gauge and rig.
The fixed branch performs the existing 10 warm + 20 fixed-camera iterations;
it never releases the camera vertices or mutates live intrinsic parameters.
It compares held-out centre/periphery errors against the pre-solve geometry,
using the existing +0.03px median, 95% inlier-retention and summed 0.004px
improvement gates. Existing finite-state and physical bias gates remain.
Acceptance requires reduced robust training objective. No tracking gates,
input calibration, extrinsics, image/features, or GT constraints are changed.

Input, loop/GBA-reader and active-map locks quiesce live state during solve
and commit. Rejection leaves the private graph discarded. Acceptance updates
poses/points/velocities/biases, transports live frames and invalidates shared
obsolete priors. Fixed mode preserves stereo/intrinsic caches; nonoptimized
single-view points preserve their previous camera-space coordinates while
following their anchor update. The shared-camera calibration guard remains.

Logs: PERIODIC-VI-CONFIG / SCHEDULE / START / HELDOUT / INERTIAL / TRIAL.
A successful VI trial is not an intrinsic-calibration commit.

Build: build/orbslam3_lamaria_periodic_vi_20261003 (jobs=1), completed clean.
Production contract: tests/exact_solver/result.json under that build.
The fixture normally constructs a System then joins its worker threads,
builds 12 inertial keyframes, 272 points, 3900 native observations and 11 IMU
edges with stereo overlap/acceleration, and invokes the actual compiled
Optimizer::FullCalibrationBA(..., false). All checks passed: accepted solve,
bit-identical 32 camera scalars, fixed gauge/rig/cache/other-map state,
shared prior invalidation, one map publication, improved metric landmarks,
and atomic rejection of an invalid IMU covariance.

Synthetic initial mean point error is 9.5mm; the accepted solution reduces
it below the fixture's 1.9mm threshold. Held-out medians fall from roughly
0.61--0.69px to 1e-5px. This validates mechanics, not real-sequence performance.
The first test fixture lacked metric observability (constant velocity,
disjoint camera tracks): its metric assertion failed despite tiny residuals.
Its log is retained as fixture_unobservable_initial.log. The final fixture
adds real stereo correspondences and acceleration rather than relaxing the
assertion. No estimator thresholds were changed in response to that test.

Reproduce the production contract with:
  python patches/orbslam3_lamaria_periodic_vi_20261003/tests/run_contract.py \
    --config configs/orbslam3_lamaria/online_full_v2_transfer_20261003/medium.yaml \
    --out build/orbslam3_lamaria_periodic_vi_20261003/tests/new_contract

Only the initial authorized full Medium and Long cases are planned; full
trajectory scores/continuity are not known at this implementation handoff.
