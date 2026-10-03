# Version 2: unique target ownership during triangulation

Includes the previous LaMAria geometry repairs and the missing `vbMatched2[bestIdx2] = true` assignment in `SearchForTriangulation`. Reserving each matched target once prevents separate landmarks from overwriting the same keyframe observation slot.

Six real keyframe-pair tests in the production matching direction reproduced 72 duplicate claims among 596 accepted triangulations. Both the candidate and the rebuilt production library eliminated all duplicates; valid unique targets changed from 524 to 528. These fixtures start with unmapped features, so they establish the mechanism rather than its frequency during live tracking.

The independent ten-frame geometry test uses the production Fisheye624 model, GeometricTools::Triangulate and EdgeMono/g2o. First-five and last-five reconstructions use disjoint observations and fixed measured camera poses. Across three predeclared windows and both cameras, 241 persistent tracks gave 1.025 px median held-out reprojection error (2.159 px p90), with outliers retained up to 14.865 px. The longest window had no cam0 track surviving all ten images. This validates these visual geometry paths conditional on poses; it does not validate online pose estimation or IMU integration.

Reproduction source, exact library provenance, raw results and visual contact sheets: `/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_tenframe/`. The duplicate-ownership regression is in `/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_green/triangulation_production_v2_results.log`.

The completed replay is `/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_unique_full_20261002/`. Estimator and exporter exited 0; all 35,842 camera frames were processed, and the full Rerun recording passed verification with no missing images. Relative to `lamaria_geometry_full_20261001_v2`, completed resets increased from 26 to 36 and exported coverage stayed at 8,764 poses (24.45%). On the same 1,293 reference timestamps, scale-1 SE3 ATE RMSE changed from 1.303 m to 1.238 m. Over the shared saved-map interval, median mapped points changed from 110/102 to 115/107 for cam0/cam1. The saved map still loses tracking around 1529 seconds. This single replay establishes neither improved robustness nor that this fix caused the reset-count increase. The association invariant is repaired; the remaining tracking failure is unresolved. Exact comparison and validation records are in the replay directory.

Build: `python3 pipeline/run/build_lamaria.py --patch-dir patches/orbslam3_lamaria_geometry_v2 --out build/orbslam3_lamaria_v2`. Run with `--build build/orbslam3_lamaria_v2`.

# LaMAria metric rig geometry repair

The old monocular rig bootstrap normalized a temporal map to arbitrary units while camera1 projection used the calibrated inter-camera translation in metres. Camera0 reprojection could remain correct because jointly scaling its map and translation cancels in projection; the fixed metric baseline does not cancel. This broke the relationship between the two cameras, map points and subsequent inertial optimization.

The repaired runner uses `IMU_STEREO` with temporal initialization. Calibrated stereo observations determine the temporal map's metric scale, with independent-landmark consensus, reprojection and observability checks. Initialization waits when scale is unobservable. The inter-camera baseline is retained in metres; ground truth is never used by the estimator. Stereo inertial optimization then preserves metric scale.

Related fixes restore the overlapping ALIKED feature ranges, enforce reciprocal unique stereo matches, and resolve left/right pooled indices in mapping, fusion and merge optimization. Sim3 keeps its existing camera0 contract and uses camera pixels for synthesized missing-neighbor observations. Inertial pose optimization selects a valid frame prior, a valid keyframe preintegration, or visual fallback.

The workflow also factory-rectifies `imu-right` values and aligns gyro/accelerometer timing to the camera clock using VRS calibration. The raw channels were already rad/s and m/s²; factory correction is separate from unit conversion. Existing Fisheye624 unit-bearing unprojection remains in the recorded baseline. This package does not replace camera extrinsics with an estimated or ground-truth transform.

## Build and run

Run from the repository root. The build needs the recorded existing ORB-SLAM3 CMake build and local Docker image `insv/orbslam3:gdb`. It verifies `base_manifest.json`, applies the patch to a source overlay, and writes artifacts under `build/orbslam3_lamaria_v2`; `third_party` stays unchanged. Both build and replay commands select this version by default.

```sh
python3 pipeline/run/build_lamaria.py --jobs 2
python3 pipeline/run/run_lamaria.py \
  --dataset /path/to/euroc-layout-sequence \
  --timestamps /path/to/chronological-camera-timestamps-ns.txt \
  --kp0 /path/to/aliked/cam0 --kp1 /path/to/aliked/cam1 \
  --vrs /path/to/recording.vrs \
  --imu-python /path/to/python-with-projectaria-tools \
  --viz-python /path/to/python-with-rerun \
  --out experiments/lamaria_geometry_check
```

Camera timestamps must be strictly increasing integer nanoseconds. The default configuration is `configs/orbslam3_lamaria/stereo_inertial.yaml`, with `Rig.non_overlapping: 0`. Instead of rectifying again, supply `--calibrated-imu corrected.csv --imu-manifest corrected.csv.manifest.json`. `--dry-run` validates inputs and prints the command. Each run requires a new output directory and records configuration, build/input hashes, estimator logs, trajectory and visualization exports. Optional `--gt` is for scoring/display only and expects a cam0 TUM trajectory in seconds.

## Regression evidence

```sh
python3 -m unittest discover -s pipeline/run -p 'test_lamaria_workflow.py'
python3 patches/orbslam3_lamaria_geometry/tests/run_optimizer_regression.py
```

The optimizer regression constructs 11 real KeyFrame objects, 60 synthetic landmarks and 923 observations using LaMAria's Fisheye624 calibration, unequal feature pools (48 left, 64 right), and left-only, right-only and paired observations. It calls the actual `OptimizeSim3`, visual merge BA and inertial merge BA implementations. Right-only landmarks are deliberately perturbed so a pass requires their camera-specific constraints to correct them.

| Check | Repaired result | Original-code control |
| --- | --- | --- |
| Sim3 | 47 valid camera0 inliers; scale 1; missing-neighbor pixel projection retained | AddressSanitizer heap-buffer-overflow |
| Visual merge BA | Max reprojection error 0.768 → 0.00094 px; all 923 observations retained | AddressSanitizer heap-buffer-overflow |
| Inertial merge BA | Max reprojection error 0.768 → 0.00591 px; all 923 observations retained; finite velocities | AddressSanitizer heap-buffer-overflow |

All three passed both direct calls into the production library and separately instrumented, verbatim copies of the modified functions. The original-code controls use the frozen vendor source with the same fixture and library dependencies. Test sources, source hashes, logs and executables are generated under `build/orbslam3_lamaria/tests/optimizer`, with a machine-readable `results.json`. No dataset or ground truth is needed for these synthetic tests.

Additional packaged fixtures cover metric scale estimation and stereo match uniqueness. These focused tests establish the repaired geometry/data-access behavior; they do not establish full-sequence tracking quality or exercise the complete loop-detection/merge lifecycle. Full replay results must be assessed from a completed run's logs and coverage, independently of these regression checks.
