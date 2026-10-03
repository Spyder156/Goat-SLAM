# BabyFeatures Long: final native-graph VI refinement

Completed 2026-10-04 on `experiments/BabyFeats`. One offline refinement of the
saved Long trajectory; no SLAM replay, new feature extraction, or parameter sweep.

The user then stopped the session for their usage limit and requested documentation
only. Their latest visual observations and proposed diagnostic priorities are
recorded in `docs/RESUME_20261004.md`: confirmed missing startup poses, apparent
end drift, and possible tilt/alignment compromise. No tilt diagnosis was measured
and no additional experiment was launched.

## Result

**Modest positive result: Score2D 43.729292 → 45.027586 (+1.298294).** The same
35,619 / 35,842 native timestamps, one map and 27 / 27 control points remain.
Historical Long 49.881534 and Short 80.199161 remain preserved and stronger.

| Measurement | Saved Baby Long | Fixed-camera global VI |
|---|---:|---:|
| Official local Score2D | 43.729292 | 45.027586 |
| CP within 1 m | 4 / 27 | 5 / 27 |
| Dense horizontal median | 1.895925 m | 1.807317 m |
| Dense horizontal RMSE | 2.933179 m | 2.754464 m |
| Dense horizontal maximum | 10.843229 m | 8.590225 m |
| CP evaluation Sim3 scale | 1.008174 | 1.007453 |
| Largest adjacent exported step | 1.618828 m | 1.622238 m |
| Native pose coverage | 99.377825% | 99.377825% |
| Dense GT associations | 6078 / 6118 | 6078 / 6118 |

Sixteen CP errors decreased and eleven increased under each output's official
saved CP alignment. The remaining large jump was not repaired. This is an offline
accuracy improvement, not evidence that online tracking or SOS was improved.

## What changed

The inherited periodic calibration builds both a fixed-camera VI solution and a
free-intrinsics candidate, but commits only the accepted free-intrinsics candidate.
Rejected calibration trials therefore do not commit their fixed-camera solution.
This experiment tests the value of a separate final fixed-calibration VI solve.

The adapter exported the retained native graph: **2,401 keyframes, 4,802 camera
images, 156,557 landmarks and 1,119,060 observations**, including 82,368 points seen
by both cameras. No SIFT graph or additional matches were introduced. Both native
Fisheye624 lenses use the last paired accepted calibration at 1262.649294925 s;
camera, rig and camera/IMU extrinsics stay fixed. The rig baseline is 0.137749094 m.
Native pixel centres and COLMAP's centres differ by 0.5 pixels; both observations
and principal points receive that offset, and export reverses it.

The Ceres/COLMAP-backed VI solver jointly adjusts keyframe poses, XYZ, velocities,
biases and gravity direction with native calibrated IMU intervals. It first fits
inertial states with visual geometry fixed. First-pose gauge and gravity norm are
fixed. Saved online velocities/biases were unavailable, so this is reconstruction
of inertial states from the saved graph and raw calibrated IMU, not a byte-for-byte
continuation of the live optimizer. Native ALIKED observations are octave zero.

Ground truth is read only by the subsequent official evaluation and comparison.
Neither the native-graph adapter nor VI optimizer uses it.

## Solver and validation

The fresh private binary uses current corrected midpoint IMU integration and
exact final reintegration. It passed the existing synthetic rig/VI contract:
21 frames, 3,360 observations, convergence and unchanged lenses. Eight adapter
and history-transport tests passed, including nonidentity rig/body poses, native
pixel centres, accepted calibration selection, association integrity, reference
correction and missing-reference rejection. A separate source review checked the
body/camera transform and reference-history convention.

The real graph solve used sparse Schur, at most 100 outer iterations, 8 CPUs and
a 26 GiB container limit. Wall time was **78.52 s**, including model loading and
export. It reported convergence after 39 outer iterations / 15 accepted updates.
There were 23 rejected trial steps before the final tolerance stop; the last
reported gradient maximum was about 16.6. This is solver-reported convergence at
a cost plateau, not evidence of a fully stationary optimum. Total objective
after inertial initialization decreased **14.96%**, from
433167.3197 to an exact-reintegrated 368373.2927. Final projectable-observation
pixel RMS was **0.803250**. Final IMU exact/linearized objective differed by only
2.96e-7 relative. The `initial_imu_cost` result field precedes inertial warmup;
do not compare that field directly to the post-warmup total objective.

The solver removed 35 observations and 5 landmarks due to invalid final projection
or resulting track support. It retained 1,119,025 observations and 156,552 points.
Matching and robust residual thresholds were not loosened. Termination is the
solver's local numerical criterion, not proof of global geometric accuracy.

Every original frame is exported using its surviving reference keyframe:

`T_world_body_new = T_world_reference_new * inverse(T_world_reference_old) * T_world_body_old`

No timestamps are invented or interpolated. Non-keyframe states retain their
original relative pose; they were not individually reoptimized. No-op export
preserves position exactly and orientation within 2.61e-16 radians. Updated
references move frames by a median 0.333176 m and maximum 3.007264 m. Existing
online keypoint associations remain labelled as online evidence, not offline
accepted residuals. Official scoring confirmed the exact original coverage.

## Artifacts and reproduction

All artifacts are under:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_native_vi_20261004
```

- `input/manifest.json`: native graph counts, IDs, calibration, source/model hashes.
- `offline_experiment.json`: source, calibrated IMU hash, command and timing.
- `offline/vi_fixed/model/execution_manifest.json`: actual binary, limits and build hashes.
- `offline/vi_fixed/model/optimization.csv` and `result.json`: solver evidence.
- `runs/vi_fixed/`: full-history candidate, coverage, geometry checks and official score.
- `comparison/comparison.png`: equal-scale route views and all 27 CP errors.
- `comparison/control_points.csv`: exact per-CP changes.
- `comparison/comparison.json`: score/coverage comparison and source hashes.

Build: `build/colmap_lamaria_babyfeats_vi_20261004/`; binary SHA256:
`36d3d76f746c60a7a868efda41990a94be06efbd2063d1849c9f6e8e89be81a1`.
The launcher command in `offline_experiment.json` reproduces the solve with a fresh
output directory. Native export and comparison commands are recorded alongside
their outputs. Original runs and baseline implementations were not overwritten.

In the comparison, a healthy improvement lowers CP bars across the route while
keeping complete coverage. Here the route barely changes at full-route scale,
and metre-scale errors remain visible in the bars. This explains why an attractive
route with subpixel native reprojection still scores only 45.03.

## Decision

Keep the 45.03 artifact as a better refinement of this Baby Long run, without
promoting it over the historical Long baseline. Repeating global BA on the same
fixed graph is not supported as a route to 70–80 by this result. The next meaningful
change must improve constraints or the sensor model, rather than only extending
the same solve. SOS return-window continuity and longer verified static tracks
remain concrete targets; joint calibration needs its own held-out validation.
Stop for visual review before starting another implementation family.

## Requested original-run Rerun

The full **43.729292 source run** recording completed and passed `rrd verify`:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_long_20261003/evaluation_baby_20261004/run.rrd
```

All 35,842 frames have both normal and Baby camera panels. The 3D map is amber
for the 72 logged accepted Baby+IMU updates, magenta during the 11 other logged
fallback frames, and its original colour during normal tracking. First retained
SOS: native 408.099–408.349 s; the remaining jump is near 1607.199 s.

Per-feature solver inlier IDs were not recorded. Baby panels therefore show
reconstructed passive descriptor tracks, with actual aggregate solver counts
and activation from logs. All 453 logged pair-count comparisons matched;
71,684 camera descriptor mappings had no issues. This is neither a solver replay
nor a claim that every displayed Baby point was an accepted factor. The map uses
final optimized positions revealed at earliest surviving observations, not an
unrecorded history of BA positions. One distant point is omitted by the existing
display filter; full source geometry remains saved. Recording size: 10,847,526,424
bytes. `baby_visualization.json` documents semantics and all colour transitions.
This Rerun belongs to 43.73, not the later 45.03 offline candidate.
