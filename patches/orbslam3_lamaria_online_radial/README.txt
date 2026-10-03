orbslam3_lamaria_online_radial: isolated experimental overlay on orbslam3_lamaria_valid_domain.
Vendor is read-only. See candidate_provenance.json for exact staging hashes and docs/LAMARIA_RELIABILITY.txt for experiment outcomes.
Build with pipeline/run/build_lamaria.py --patch-dir patches/orbslam3_lamaria_online_radial --out build/orbslam3_lamaria_online_radial --jobs 4.
This candidate is not the default.

2026-10-03 experiment: native-domain repair + joint metric initializer + bounded
two-coefficient radial calibration in the first inertial initialization.

Init.radialCalibrationMode: 0=off, 1=fixed-camera control, 2=online radial trial.
Both paired configs use Init.metricMinSeconds=12.0 and the same observation
holdout. The first initialization pauses input while solving and committing;
it does not skip frames. Camera poses, landmarks, velocities, gravity and IMU
biases are optimized on a copied graph. Only Fisheye624 parameters 4 and 5 per
camera may change, bounded to +/-0.02 and +/-0.01 with Gaussian priors. Focal
length, principal point, higher distortion terms, rig and IMU extrinsics remain
fixed. Matching bounds are unchanged. No ground truth enters the estimator.

Calibration requires coverage of every live keyframe in the sole populated map,
stable source state, observable bounded parameters, valid inverse geometry and
held-out central/peripheral reprojection checks. A rejected calibration restores
the fixed-camera candidate. An accepted calibration commits camera coefficients
and geometry together under tracking/calibration and map locks, refreshes stereo
depth/XYZ caches through production triangulation, and is applied once per run.
Scope: first initialization of the tested two-camera IMU_STEREO pipeline only.
This is not a general asynchronous or repeated online-calibration implementation.

The holdout excludes selected observations from this refinement, including its
scalar stereo seed, but the existing map was constructed from the input images.
It is an optimization holdout, not an independent previously unseen sequence.

Final-library checks: 26 factor/cache contracts, 10 metric/joint cases and two
proposal/fallback cases passed. Results are under build/orbslam3_lamaria_online_radial
/tests/ and build/lamaria_online_radial_staging/radial_contract_production_v1/.

Full Short A/B suites:
experiments/lamaria_radial_fixed_short_full_20261003
experiments/lamaria_radial_online_short_full_20261003
Config/provenance: configs/orbslam3_lamaria/online_radial_20261003/experiment.json
Comparison: experiments/lamaria_radial_full_comparison_20261003

The visualization exporter reads accepted RADIAL-COMMIT records to project the
final map using the estimated camera model; source configs and official scoring
calibration remain unchanged. No display or scoring scale enters the estimator.
Full-run outcomes and remaining limitations belong in docs/LAMARIA_RELIABILITY.txt.

Full Short outcome: both runs completed and exported all 18,343 available poses
in one map. Fixed-camera mode scored 67.1884 (previous domain-only 55.3499),
but retained a global estimate-to-GT scale of 0.8862. Online mode corrected
global scale to 1.0012, yet scored 28.9521 due to lasting route deformation
around native 528-530 seconds. The trial is mixed, not a calibration promotion.
The independently verified scores use identical official calibration and all
14 control points / 2,999 GT timestamps. History remains the default build.
