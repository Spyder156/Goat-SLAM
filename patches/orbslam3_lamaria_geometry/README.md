# LaMAria metric rig geometry repair

The old monocular rig bootstrap normalized a temporal map to arbitrary units while camera1 projection used the calibrated inter-camera translation in metres. Camera0 reprojection could remain correct because jointly scaling its map and translation cancels in projection; the fixed metric baseline does not cancel. This broke the relationship between the two cameras, map points and subsequent inertial optimization.

The repaired runner uses `IMU_STEREO` with temporal initialization. Calibrated stereo observations determine the temporal map's metric scale, with independent-landmark consensus, reprojection and observability checks. Initialization waits when scale is unobservable. The inter-camera baseline is retained in metres; ground truth is never used by the estimator. Stereo inertial optimization then preserves metric scale.

Related fixes restore the overlapping ALIKED feature ranges, enforce reciprocal unique stereo matches, and resolve left/right pooled indices in mapping, fusion and merge optimization. Sim3 keeps its existing camera0 contract and uses camera pixels for synthesized missing-neighbor observations. Inertial pose optimization selects a valid frame prior, a valid keyframe preintegration, or visual fallback.

The workflow also factory-rectifies `imu-right` values and aligns gyro/accelerometer timing to the camera clock using VRS calibration. The raw channels were already rad/s and m/s²; factory correction is separate from unit conversion. Existing Fisheye624 unit-bearing unprojection remains in the recorded baseline. This package does not replace camera extrinsics with an estimated or ground-truth transform.

## Build and run

Run from the repository root. The build needs the recorded existing ORB-SLAM3 CMake build and local Docker image `insv/orbslam3:gdb`. It verifies `base_manifest.json`, applies the patch to a source overlay, and writes artifacts under `build/orbslam3_lamaria`; `third_party` stays unchanged.

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
