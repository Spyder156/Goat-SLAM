# LaMAria experiment ledger and research backlog

Status recorded on 2026-10-04. Read GOAL.md first for the objective and ROADMAP.md for the implementation order. This file records what actually happened, including negative results, correctness-only tests, incomplete runs, and ideas that have not been tested. It is deliberately separate from the roadmap so that a hypothesis cannot quietly become an established result.

The project is Raghav's ORB-SLAM3 fork adapted to native dual-camera Aria Fisheye624 images and calibrated IMU. The goal is a reliable, accurate metric SLAM system that can outperform Meta/Aria under the actual LaMAria benchmark protocol. The best observed result below, **80.199161 on one full Short sequence**, is promising; it is not a category-average result or a leaderboard victory.

## 1. How to read the results

- **Official local Score2D** means the released LaMAria evaluation functions were run locally. No result here was submitted to the public leaderboard. Score2D averages the official piecewise control-point error contributions, with missing control points contributing zero. It is not dense-trajectory ATE and it is not PoseRecall.
- **CP triangulated** and **CP within 1 m** are different. Recovering 14/14 control points does not imply a high score if their positions remain inaccurate.
- **PoseRecall** uses the full GT timestamp denominator. Missing GT associations count as recall failures. They do not directly enter Score2D; missing control points do.
- The official evaluator estimates **one control-point Sim3** after SLAM. It reuses that transform for the trajectory. SLAM exports raw metric poses and does not get this evaluation transform. A uniform scale error can therefore be absorbed by scoring while still indicating an incorrect metric estimator. Local shape errors and drift remain.
- **Metric SE3 RMSE** allows translation and rotation alignment, with scale fixed at one. **Diagnostic dense-GT Sim3 RMSE** allows scale too. Neither should be relabelled as Score2D.
- A full input replay can still export only one fragment. Always inspect the selected map, omitted maps, timestamp coverage, initialization gaps, and coasted versus visually constrained poses.
- Green Rerun points mean a mapped association under that recording's definition. They are not automatically optimizer inliers. In offline COLMAP Reruns they are final batch-map associations, not causal online tracking successes.
- Most estimator variants were run once in a multithreaded pipeline. Changed keyframe/BA timing and map snapshots can confound a one-variable source comparison. Tiny score differences are not established improvements.
- Historical reports retain their contemporary findings and defaults. Their statements such as “not yet implemented” or “next step” are time-local; this ledger includes subsequent outcomes.

All absolute paths below are plain paths. Project root:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3
```

Main artifact root (`experiments/` in the repository resolves here):

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments
```

## 2. Verified official score register

Scores were checked against saved `lamaria_score/scores.json` artifacts when this ledger was written. Numbers are rounded here; JSON retains full precision. “GT” means associated / full GT denominator. “Maps” means atlas epochs, including maps excluded from the single-map score.

| ID | Sequence and variant | Score2D | CP triangulated | CP within 1 m | GT | Maps | Verdict |
|---|---|---:|---:|---:|---:|---:|---|
| S01 | Long `sequence_3_17`, history/continuity | 49.881534 | 27/27 | 11/27 | 6113/6118 | 1 | First strong full-route baseline; substantial drift remains |
| S02 | Medium `sequence_2_11`, history/continuity | 55.752448 | 13/18 | 12/18 | 2761/4083 | 3 | Accurate retained portion; incomplete selected trajectory |
| S03 | Short `sequence_1_19`, history/continuity | 39.656698 | 11/14 | 2/14 | 2452/2999 | 2 | Scale/shape error plus missing route |
| S04 | Short, dual-camera observation repair | 9.627402 | 14/14 | 0/14 | 2999/2999 | 1 | Better coverage, much worse score |
| S05 | Medium, dual-camera observation repair | 39.921623 | 18/18 | 3/18 | 4080/4083 | 1 | Better coverage, worse accuracy |
| S06 | Short, native valid-domain repair | 55.349924 | 14/14 | 5/14 | 2999/2999 | 1 | Strong improvement over direct observation-build parent |
| S07 | Short, joint initializer, fixed-camera control | 67.188379 | 14/14 | 10/14 | 2999/2999 | 1 | Reference “67.19 run”; raw metric scale still wrong |
| S08 | Short, first-initialization radial calibration | 28.952104 | 14/14 | 1/14 | 2999/2999 | 1 | Scale improved; route deformation made score worse |
| S09 | Short, recurring full online VI/calibration v1 | 48.068151 | 11/14 | 6/14 | 2424/2999 | 2 | Calibration-validation bug; unsuccessful full-route result |
| S10 | Short, recurring full online VI/calibration v2 | **80.199161** | **14/14** | **14/14** | **2999/2999** | **1** | Best observed Short result; transient jumps remain |
| S11 | Short, online v3 rollback variant | 42.593277 | 7/14 | 7/14 | 1384/2999 | 3 | Correct rollback contract; severe continuity regression |
| S12 | Short S07 + offline SIFT visual rig BA | 64.705686 | 14/14 | 10/14 | 2999/2999 | 1 | Worse than 67.19 |
| S13 | Short S07 + offline SIFT full VI-BA, fixed intrinsics | 67.194208 | 14/14 | 10/14 | 2999/2999 | 1 | Essentially unchanged: +0.005828 |
| S14 | Short S07 + offline SIFT full VI-BA, all intrinsics | 67.183330 | 14/14 | 10/14 | 2999/2999 | 1 | Essentially unchanged: -0.005050 |
| S15 | Medium, unchanged online VI/calibration v2 method | 62.764799 | 13/18 | 13/18 | 2499/4083 | 3 | Accurate selected segment; fragmentation still removes much of the route |
| S16 | Long, online VI/calibration v2 with verified native input | 34.458901 | 15/27 | 9/27 | 3163/6118 | 5 | Completed replay, negative continuity result; only 47.88% in the scored map |
| S17 | Medium, direct recovery | Unscorable | — | — | — | 7 | No official CP alignment |
| S18 | Long, direct recovery | 38.941196 | 18/27 | 6/27 | 3857/6118 | 8 | Above native v2; below historical Long; still fragmented |
| S19 | Medium, landmark memory | 45.874750 | 12/18 | 10/18 | 2115/4083 | 4 | Below unchanged Medium |
| S20 | Long, landmark memory | 42.904563 | 19/27 | 8/27 | 4163/6118 | 6 | Above native v2; below historical Long; still fragmented |
| S21 | Medium, periodic fixed VI | Unscorable | — | — | — | 6 | No official CP alignment |
| S22 | Long, periodic fixed VI | 37.630917 | 16/27 | 11/27 | 3545/6118 | 13 | Above native v2; below historical Long; still fragmented |
| S23 | Medium, unchanged v2 repeat | 63.086620 | 13/18 | 12/18 | 2497/4083 | 4 | Reproduces saved v2 score; no estimator change |
| S24 | Long, BabyFeatures SOS | **43.729292** | **27/27** | 4/27 | 6078/6118 | **1** | No split after startup; 15 SOS returns; residual jumps and metre-scale error remain |
| S25 | S24 + final native-graph fixed-camera global VI | **45.027586** | **27/27** | 5/27 | 6078/6118 | **1** | +1.298294, identical coverage; 1.62 m jump remains |

The chronological history is not a monotonic leaderboard ladder. Different sequences, coverage, feature graphs, and optimizer budgets matter. Do not compare Long 49.88 with Short 80.20 as the effect of one change.

### Exact run paths for the score register

Each scored directory below contains `lamaria_score/scores.json`. Unscorable
S17/S21 retain evaluation provenance and the failed official alignment logs;
they have no fabricated score file.

```text
S01 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_history_full_20261002
S02 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_history_medium_2_11_20261002
S03 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_history_short_1_19_20261002
S04 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_observations_short_20261002/runs/lamaria_observations_short_20261002_short_full
S05 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_observations_medium_20261002/runs/lamaria_observations_medium_20261002_medium_full
S06 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_domain_short_full_20261003/runs/lamaria_domain_short_full_20261003_short_full
S07 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_radial_fixed_short_full_20261003/runs/lamaria_radial_fixed_short_full_20261003_short_full
S08 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_radial_online_short_full_20261003/runs/lamaria_radial_online_short_full_20261003_short_full
S09 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_short_full_20261003/runs/lamaria_online_full_short_full_20261003_short_full
S10 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/runs/lamaria_online_full_v2_short_full_20261003_short_full
S11 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v3_short_full_20261003/runs/lamaria_online_full_v3_short_full_20261003_short_full
S12 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/runs/visual_rig
S13 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/runs/vi_fixed
S14 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/runs/vi_calib
S15 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_medium_full_20261003/runs/lamaria_online_full_v2_medium_full_20261003_medium_full
S16 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003/runs/lamaria_online_full_v2_long_native_full_20261003_long_full
S17 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/recovery_direct_medium/runs/recovery_direct_medium_medium_full
S18 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/recovery_direct_long/runs/recovery_direct_long_long_full
S19 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/tracking_memory_medium/runs/tracking_memory_medium_medium_full
S20 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/tracking_memory_long/runs/tracking_memory_long_long_full
S21 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/periodic_vi_medium/runs/periodic_vi_medium_medium_full
S22 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/periodic_vi_long/runs/periodic_vi_long_long_full
S23 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/frozen_v2_repeat_medium/runs/frozen_v2_repeat_medium_medium_full
S24 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_long_20261003/runs/lamaria_babyfeats_long_20261003_long_full
S25 /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_native_vi_20261004/runs/vi_fixed
```

## 3. The experiments, one by one

### E01. Initial Fisheye624 bearing and index repairs

**Question:** Why can two images reproject plausibly while accumulated map points, rig geometry, and tracking fail?

The early conversation records `Fisheye624::unproject` changing from z=1 tangent-plane rays to unit bearings, with the private DLT changed to the fork's general-ray form. This convention correction is already present in the later geometry baseline. Pooled left/right indexing and lifetime errors were also repaired during the early debugging phase.

**Evidence status:** the conversation reports Run A with 31 maps, a guarded Run B with 802 maps and changing crash sites, `rig_aliked_02` completing but producing 935 resets, and Run 04 hitting a dangling `mpcpi` in `EdgePriorPoseImu`. These are historical user-reported counters, not newly verified official scores. Do not turn them into a numerical benchmark or conclude “threading race” from changing crash sites. There is no isolated verified score attributable solely to unit-bearing normalization or null-after-delete.

**Follow-up principle:** retain exact production-model projection, unprojection, Jacobian, pooled-index, and prior-ownership tests. A ray round trip alone does not validate a whole SLAM pipeline.

### E02. Bootstrap scale and vocabulary diagnostic, two 30-second windows

**Tried:** two repetitions in each of four arms: stock versus a qualified GT metric-bootstrap oracle, each with the vocabulary gate on/off; windows 170–200 and 450–480 seconds. All 16 runs exited normally.

**Result:** stock/vocabulary-on had 57 tracking failures and 61 initializations; oracle/on 36/40. Removing the vocabulary gate worsened both: stock/off 132/135, oracle/off 109/112. The oracle was only applied at qualifying initializations; it was not a complete GT trajectory feed.

**Interpretation:** a map-unit/metric-baseline inconsistency was demonstrated, and correcting scale can help, but it did not explain every reset. Vocabulary removal was not a validated cure. **These oracle runs are deliberately GT-assisted diagnostics and are ineligible benchmark candidates. Score2D: not measured.**

Evidence:

```text
/home/raghav/workspace/MeckaAI/lamaria_audit_20261001/experiment_summary.json
/home/raghav/workspace/MeckaAI/lamaria_audit_20261001/bootstrap
/home/raghav/workspace/MeckaAI/lamaria_audit_20261001/runs/summary.json
```

### E03. Classic ORB versus binarized ALIKED image matching

**Tried:** actual fork ORB extraction at 1,500 and 2,500 features versus saved 1,500-feature ALIKED random-hyperplane binary descriptors, with common mutual-nearest, ratio 0.8, Hamming 70 gates. Native images were used; no extra displacement filter was imposed. GT was only for geometric checking.

**Result:** across 68 adjacent pairs, 1,500-feature ALIKED had median 646.5 geometrically consistent matches/pair and 98.11% consistency, versus ORB-1500's 431 and 93.35%. ORB-2500 gave 749.5 and 94.17%, at higher feature count. At a five-frame separation ALIKED consistency was 96.33%, versus ORB-1500 73.60% and ORB-2500 76.17%. Existing ORB vocabulary bins retained a smaller fraction of ALIKED matches.

**Interpretation:** ALIKED was a defensible frontend choice on sampled data; ORB is not categorically unusable, and increasing its feature count changes the comparison. Epipolar consistency is not proof of exact identity. Binarized ALIKED also has a descriptor/vocabulary mismatch worth testing. **No full SLAM Score2D from this probe.**

```text
/home/raghav/workspace/MeckaAI/lamaria_audit_20261001/frontend/summary.json
/home/raghav/workspace/MeckaAI/lamaria_audit_20261001/frontend/README.md
```

### E04. Metric rig bootstrap, native IMU correction, and camera-specific geometry

**Implemented:** use `IMU_STEREO` with temporal initialization; estimate temporal-map scale from calibrated stereo with independent-landmark consensus and observability checks; preserve the physical baseline. Restore overlapping feature ranges, reciprocal stereo ownership, correct pooled indices in mapping/fusion/merge, and valid prior/preintegration fallback in inertial optimization. Factory-rectify IMU values and apply calibrated timing relative to cameras. Raw channels already had rad/s and m/s² units; this was not a unit conversion.

**Result:** production-linked camera-specific optimizer tests passed. The old Sim3 test triggered an ASan heap-buffer-overflow; corrected paths used the proper camera/index contracts. Full geometry replays still fragmented. The later geometry-v2 baseline retained only 8,764 / 35,842 input poses in its largest map despite processing the entire Long input.

**Interpretation:** real correctness defects were fixed, but “builds, round-trips, and doesn't crash” did not mean full tracking was solved. No verified official Score2D was found for these early geometry variants.

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/patches/orbslam3_lamaria_geometry/README.md
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_geometry_full_20261001
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_geometry_full_20261001_v2
```

### E05. Ten-frame, independent five-plus-five reconstruction

**Tried:** three predeclared windows, both cameras, identical features tracked through ten time-ordered images; reconstruct from images 0–4 and 5–9 independently, then project each 3D estimate into the unseen half. Linked production `Fisheye624`, `GeometricTools::Triangulate`, `ImuCamPose`, `EdgeMono`, and g2o. Measured **GT camera poses are diagnostic inputs** that isolate visual geometry; this is not an eligible benchmark estimator run. The tested frames were not necessarily ten consecutive 20 Hz images.

**Result:** 241 descriptor tracks; all produced two finite positive-depth estimates. Aggregate held-out median was 1.025 px, p90 2.159 px, worst 14.865 px. Typical split-3D median disagreements were about 1.5–5 cm; seven far tracks in the longest window had 41.49 cm median discrepancy at 14.37 m median range. That window retained zero cam0 tracks, which was not hidden.

**Interpretation:** strong evidence against a gross convention failure in these visual paths conditional on correct poses. It does not validate online IMU state, map ownership, tracking, all image edges, or physical calibration everywhere. **Score2D: not applicable.**

```text
/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_tenframe/README.txt
/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_tenframe/summary.json
```

### E06. Unique target feature ownership in triangulation

**Implemented:** mark the selected second-image feature as matched after acceptance; previously multiple landmarks could claim it.

**Result:** full Long run exited normally, but resets increased 26 → 36, atlas maps 9 → 11, and largest-map coverage stayed 8,764 / 35,842 (24.45%). On 1,293 common GT timestamps metric SE3 RMSE changed 1.303 → 1.238 m. Median mapped counts in the shared interval changed cam0 110 → 115, cam1 102 → 107. Tracking still failed near native 1529 s.

**Interpretation:** ownership invariant repaired, no demonstrated robustness win. Reset changes from single multithreaded runs cannot be causally attributed with confidence. **No official Score2D.**

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_unique_full_20261002/comparison.json
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/patches/orbslam3_lamaria_geometry_v2/README.md
```

### E07. Loosening local-map descriptor ratio 0.8 → 0.9

**Tried:** a controlled 9,400-frame clip, then a full Long replay; no reprojection floor or tracking floor was lowered.

**Result:** in the clip, resets 4 → 2, exported coverage 93.36% → 96.61%, median green counts 125/113 → 149/141, and the terminal loss was avoided. The full trial still fragmented and was worse on its short shared export interval: metric SE3 RMSE 0.0868 → 1.4005 m over 83 common GT samples. The full comparison also included diagnostic instrumentation, unlike the exact clip setting comparison.

**Decision:** not promoted. More green dots alone are not the goal. Broad threshold loosening can hide the structural cause or admit bad constraints. **No official Score2D.**

```text
/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_leniency/README.txt
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_leniency_baseline_20261002
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_leniency_ratio09_20261002
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_leniency_ratio09_full_20261002
```

### E08. Temporal descriptor seeding alone; optical-flow build attempt

**Tried:** seed existing landmarks by immediately previous/current mutual descriptor matches before local-map projection, preserving ownership and unchanged geometric tests.

**Result:** more retained matches near 208 s, but temporal-only prefix still lost tracking at 208.349 s and ended in two maps. A separate optical-flow integration build was unsuccessful because the pinned OpenCV lacked its video module; it produced no SLAM result. An independent image probe favored temporal descriptor seeding over strict KLT on this sample.

**Interpretation:** use temporal information, but landmark replenishment and recovery still matter. **No official Score2D.**

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_temporal_first240_20261002
/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_continuity/frontend_probe
```

### E09. Rig-aware recovery, recent-keyframe traversal, mapping during bounded coasting

**Implemented:** PnP proposals from either lens with the calibrated transform back to cam0; dual-camera verification; recent-keyframe traversal that does not stall on an already selected node; keyframe insertion during the existing armed five-second inertial recovery window, clearing rejected associations before promotion.

**Correctness results:** cam1-only fixture 0 → 64 true inliers with zero planted mismatches; recovery projection 0 → 99 correct unique matches; intended neighborhood 2 → 5 keyframes.

**Replay results:** combined prefix crossed the 208 s failure in one map, metric SE3 RMSE 0.4315 m. On common timestamps temporal-only 1.0534 → combined 0.2927 m; fixed pre-loss anchored five-second recovery RMSE 2.3697 → 0.00844 m. Full candidate still reset five times early, then retained 34,401 / 35,842 poses (95.98%), with full-route metric RMSE 8.641 m.

**Interpretation:** genuine local recovery without relaxed geometric gates; early initialization/history retention remained unsolved. **No official Score2D.**

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_first240_20261002
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_full_20261002
/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_continuity/README.txt
```

### E10. Permit confident IMU recovery before BA2 completion

**Implemented:** arm the existing recovery interval after actual IMU initialization plus the existing 30-frame/30-inlier confidence streak, without waiting for BA2. Keep the 15-inlier tracking floor and five-second limit.

**Result:** full Long zero resets, original map, 35,772 / 35,842 poses (99.8047%). Metric SE3 RMSE 10.3445 m. Fixed-anchor recovery errors were centimetric near early and historical failure sites; a late long coast had 25.70 cm RMSE. Mapped medians improved over the older fragmented baseline from 89/84 → 141/134.

**Interpretation:** major continuity gain, not a global drift solution. **No official Score2D for this intermediate variant.**

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_early_recovery_first240_20261002
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_early_recovery_full_20261002
```

### E11. Native projection in coarse loop matching

**Implemented:** replace pinhole equations in a coarse loop-closure overload with the actual native camera `project()`.

**Correctness result:** 100 native wide-angle observations matched 0 → 100; pinhole control stayed 100. Old median pixel discrepancy was 90.44 px.

**Full result:** continuous Long had zero resets, 35,772 poses, metric RMSE 9.2546 m. **Zero coarse-loop invocations were logged**, so improved replay numbers cannot be credited to this fix. The route has no measured GT pose pairs within 5 m after 60 s separation; a useful loop is not guaranteed. Medium `sequence_2_11` is the intended loop test.

**Score2D:** not measured for this intermediate variant.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuous_full_20261002
/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_continuity/loop_projection_candidate
```

### E12. Correct frame-history attachment and keyframe-culling rebases

**Implemented:** pair each saved frame-relative pose with a recent temporal keyframe, while keeping its visual matching reference separate; rebase history when keyframes are culled.

**Correctness result:** synthetic/actual-class wrong-reference step 3.05496 m → 0.0500003 m, preserved through repeated culling. Reconstructed pose preservation error <5.4e-7 in matrix norm.

**Full Long result, S01:** zero resets, 35,772 / 35,842 poses, 168,031 finite unique landmarks. Final-only large adjacent jumps 115 → 2 compared with the prior continuous run; late-window maximum step 3.1467 → 0.2258 m. Full metric RMSE 9.1420 m. Official Score2D **49.881534**, CP scale 0.9640021. All 70 missing poses are startup; five GT timestamps are unmatched. The shared old short segment did worsen 1.238 → 1.767 m, so continuity was not universal accuracy improvement.

**Interpretation:** this became the historical continuity baseline. The remaining two jumps were associated with actual online drift, not the repaired history composition defect.

### E13. Native Fisheye624 Basalt with corrected IMU

**Tried:** independent existing Basalt backend, previous vision-refined native intrinsics, same factory-rectified/time-corrected IMU, fixed factory rig. Disabled duplicate correction in Basalt configuration.

**Result:** 35,832 / 35,842 poses, no gaps after ten startup frames; full metric SE3 RMSE **14.7817 m**, versus prior raw-IMU/refined Basalt 14.2018 m on the same reference samples. At the old ORB failure, fixed-anchor RMSE 0.1045 m through the next interval. On ORB's old retained segment, ORB 1.2380 m versus Basalt 2.0940 m.

**Interpretation:** valid continuity reference, not an accuracy upgrade. Its accumulated cloud includes latest snapshots of marginalized points and is not a jointly optimized global map. **No verified official Score2D recorded for this trial.**

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_basalt_native_corrected_refined_20261002/README.txt
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_basalt_native_corrected_refined_20261002/viz/run.rrd
```

### E14. Carry the history baseline to full Short and Medium

**Results:** S02 Medium **55.752448**, S03 Short **39.656698**. Medium atlas has three independent maps, but only 15,709 of 23,748 input poses belong to the selected map. Short has two maps, with 15,304 selected poses of 18,352. Their missing route was not stitched using GT.

**Interpretation:** the long-route success did not establish reliable startup/recovery on all data. Medium's low surviving-segment error cannot substitute for complete coverage. A feature-rich image and a visually good retained path can coexist with low benchmark performance.

### E15. Freeze the suite and reproduce the first failing stage

**Prepared:** Short/Medium/Long × startup/loss-context/loss-prefix/full, 12 cases with preserved native timestamps, settings, commands, scoring, and rendering. Startup is the original first 120 seconds. A cold-start loss-context crop has a different history from an original-start loss-prefix and must not be treated as equivalent.

**Diagnostic replays:** Short first failed local-map update at 804.642200762 s detected 708/1,288 features but only **2+11=13** map associations entered optimization; only 2+3 survived. The floor was 15, so optimizer tuning alone could not rescue that call. Medium's old terminal loss did not reproduce; it differed in inliers/IMU timing much earlier, showing variability rather than an implemented repair.

**Score2D:** no new full-sequence scores; these were diagnostic prefixes.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_suite_20261002
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_reliability_step1_20261002/baseline_dashboard.html
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_reliability_replay_20261002
```

### E16. Dual-camera observation bookkeeping diagnosis and repair

**Found:** keyframe-wide membership tests suppressed a landmark's second-camera observation; fusion/replacement had related assumptions. Short saved state had 2,012 contradictory camera slots, 2,010 from cam1. At the failing transition, 1,099 descriptor proposals existed, 775 passed image-flow checks, but only seven started from already mapped landmarks. Most good red-red image matches never had a usable 3D landmark.

**Implemented:** per-camera insertion/fusion, idempotent observation counts, complementary camera retention on replacement, correct recent-landmark membership, bidirectional graph audit. Production fixture baseline failed 11/14 contracts; candidate passed 14/14. Direct mapper-to-BA synthetic depth stayed incorrectly at 8 m with one missing stereo factor, then recovered 5.00001 m for a true 5 m point after repair.

**Full results:** S04 Short **9.627402**, S05 Medium **39.921623**. Both ran to completion in one surviving map. Short recovered all CPs/GT but coasted 491 frames; its first loss now had 61 associations entering optimization and only 12 surviving. Medium also gained coverage, but score worsened. Correct factor bookkeeping did not by itself fix raw scale or inaccurate/coasted geometry.

**Interpretation:** retain correctness repair, reject the claim that it alone is an accuracy win. It created a better-defined basis for subsequent geometry changes.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_structural_audit_20261002
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_observations_comparison_20261002
/home/raghav/workspace/MeckaAI/lamaria_observation_regression_20261002
```

### E17. Scalar metric initializer and scale/lever-arm contracts

**Implemented:** multiplicative-scale Jacobian correction; preserve physical camera/IMU lever arms during map scaling; copied stereo-inertial scalar proposal with observability/consistency checks and retries.

**Correctness result:** six production cases passed, including 0.86 → 0.8599658 and 1.15 → 1.1499428 scale recovery and no-mutation rejection. Old-library controls reproduced the targeted defects. Because the old stereo path held scale=1, dormant scale-update bugs alone do not explain its entire old scale error.

**120-second results:** Short rejected all **227** proposals at stereo-consistency checks and never initialized IMU; Medium accepted initial and both early VI stages. Short metric RMSE 4.42937 m, Medium 0.150114 m on their startup scopes. Comparisons against a full-run optimized historical baseline were initially unequal in available future data and must not be used as an isolated causal A/B.

**Verdict:** functional failure on Short; no full-run score. Frozen poses/landmarks made the scalar proposal too restrictive, motivating joint refinement.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_metric_init_short_startup_20261003
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_metric_init_medium_startup_20261003
```

### E18. Image-only held-out peripheral intrinsics audit

**Tried:** production Fisheye624 bearings, descriptor and forward/backward image-flow tracks; relative pose treated as a nuisance; focal/first-radial fit on earlier windows, tested on disjoint later windows. No existing SLAM poses or landmark XYZ were used for fitting.

**Result:** sampled Short held-out medians about 0.150/0.148 px, roughly 98% below 1 px. Outer radius ≥250 px had 112/554 cam0 and 86/587 cam1 held-out tracks, medians 0.158/0.239 px, reaching ~77° off-axis. Tiny focal changes gave no meaningful overall benefit. Medium support was sparse/uneven; exact Short-doorway cam1 had no outer held-out support. An injected bad focal/radial positive control was recovered.

**Interpretation:** the test was not centre-only, but it did not cover every edge, direction, or failure. It argues against a large universal calibration error on the sampled Short tracks; it does not certify factory intrinsics or establish the cause of every tracking collapse. Two real Medium detections outside the native calibrated radius did expose a separate inverse-fallback bug.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_intrinsics_periphery_20261002/report.html
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_intrinsics_coverage_audit_20261003/report.html
```

### E19. Reject invalid native Fisheye624 domain / inverse failure

**Implemented:** use per-sequence native radius/angle metadata; explicit inverse failure instead of fabricated bearings; filter before feature indexing for cached ALIKED and native ORB. Lens coefficients and matcher bounds stayed fixed.

**Contracts:** 17,732 SDK-valid grid pixels retained, 1,468 invalid rejected; all 1,893 sampled valid peripheral pixels at radius ≥300 px retained. Two actual outside-domain pixels with ~122 px round-trip error were rejected. This validates implementation/domain agreement, not physical calibration truth.

**Full Short, S06:** **55.349924**, versus direct parent 9.627402. Coasted frames 491 → 12; doorway 790–840 s coasting 489 → 9, median inliers 15 → 219.5. One surviving map, 18,343 poses. CP scale **0.8768755** means metric scale remained wrong.

**Full Medium:** exit **139** after native 665.437393837 s, no final trajectory/map export, no valid full score or candidate Rerun. No stack/core was captured; the site/cause remained unproven. Do not substitute historical Medium scores for this failed run.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_domain_joint_20261003/SUMMARY.txt
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_domain_full_comparison_20261003/report.html
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_domain_medium_full_20261003
```

### E20. Joint stereo-inertial initializer, controlled startup windows

**Implemented:** use scalar solution as a seed, then jointly refine copied poses, landmarks, velocities, biases, and gravity before validation/commit; keep physical rig and IMU lever arms. Scale culled keyframe-relative translations consistently during map-scale updates.

**Contracts:** ten normal production cases passed. A deliberately wrong fixed-boundary fixture still had a metric error despite low visual residuals; it remains an explicit limitation. Actual real startup windows had no fixed boundary keyframes, so that fixture did not explain their error.

**Matched 120-second results:** both Short/Medium accepted initial IMU and both early VI stages. Short metric RMSE 4.347502 → 4.158946 m; Sim3 shape RMSE 0.453423 → 0.219004 m, but diagnostic scale 0.8590951 still meant ~16.4% oversizing. Medium 0.200021 → 0.137589 m. Domain-only and joint-only changes were separate in these startup tests.

**Verdict:** initialization acceptance improved; metric scale not solved. No full-run Score2D for these isolated startup arms.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_init_control_short_20261003
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_init_control_medium_20261003
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_joint_short_startup_20261003
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_joint_medium_startup_20261003
```

### E21. Umeyama Sim3 diagnostic requested from the visual trajectory

**Tried:** one proper-rotation positive-scale fit of the domain Short output to all 2,999 associated GT camera poses, then apply that same transform to its triangulated CPs with no second fit.

**Result:** scale 0.8769552, rigid 3D RMSE 18.10839 → Sim3 1.71343 m. Diagnostic CP score **54.02407** under this dense-GT fit. The official CP-aligned score remains **55.349924**.

**Interpretation:** this is a GT-fitted diagnostic, not a new SLAM estimate or valid score improvement. The official evaluation already permits global scale, so adding another Umeyama alignment cannot be presented as fixing initialization.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_short_umeyama_20261003/report.html
```

### E22. Full Short joint initialization, fixed intrinsic control: the 67.19 run

**Tried:** combine native-domain repair and joint metric initializer, 12-second initial window, fixed cameras, deterministic observation holdout. Both radial experiment arms share this setup; it is not simply the older domain-only run.

**Result, S07:** **67.188379**, all 2,999 GT samples and 14 CPs recovered, 10 CPs within 1 m, one map, 18,343 poses, 14 coasted frames. Native recording is ~917.5 s, 18,352 input frames; eight supported startup frames plus one unsupported tail frame have no pose. Metric SE3 RMSE **16.5321 m**, dense-Sim3 shape RMSE **1.0953 m**, official CP scale **0.8861150**.

**Verdict:** much better evaluated shape/coverage than early Short versions, but the raw route remains globally oversized. This is the exact input used for all three offline COLMAP experiments below.

### E23. First-initialization online calibration of only k0/k1

**Tried:** adjust two leading radial coefficients per camera, bounded ±0.02/±0.01 with priors, together with joint geometry/inertial states; one commit at native 77.0922 s. Focal, principal point, higher terms, rig, and camera/IMU extrinsics stayed fixed. No GT in estimation.

**Result, S08:** **28.952104**, full coverage and one map, but only 1/14 CP within 1 m. Metric scale improved dramatically (CP scale 0.9990901; dense scale 1.0011970), metric SE3 RMSE **4.7258 m**, while shape RMSE worsened to **4.7233 m**. Held-out peripheral errors on the same copied graph improved cam0 1.003 → 0.854 px and cam1 1.156 → 0.988 px. Early 20–300 s metric error improved strongly; the final route did not.

**Failure:** outdoors near bridge/railing 528–530 s, green associations disappeared and a rejected/recovery optimization could leave pose, velocity/bias, and marginal prior inconsistent. A 22.349 m optimizer correction with 13 inliers was accepted by a recovery exception. The original later doorway improved, so one repaired passage did not ensure the whole route.

**Verdict:** positive metric scale evidence, negative benchmark result. It supports investigating calibration and state consistency jointly, not the claim that radial calibration alone solved the system.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_radial_full_comparison_20261003/report.html
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_radial_full_comparison_20261003/fixed.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_radial_full_comparison_20261003/online.rrd
```

### E24. Recurring full online VI-BA + all native intrinsics, v1

**Implemented:** recurring causal VI refinement of past keyframes, all 15 independent native intrinsic parameters per camera, factory priors, bounded changes, inverse-domain and held-out checks, consistent accepted state updates. This includes focal length. Rig and camera/IMU extrinsics remain fixed; “full calibration” here does not mean extrinsic/time-offset calibration.

**Result, S09:** **48.068151**, two maps, selected 14,933 poses, 2,424/2,999 GT associations. All calibration proposals were rejected because the validator incorrectly conflated a mutable principal point with the fixed physical valid-image mask.

**Verdict:** failed candidate; useful implementation bug found. It is not evidence that fitting all intrinsics cannot help. No complete Rerun was produced for this negative control.

### E25. Corrected recurring online full calibration, v2: 80.20

**Changed:** keep the physical mask centred on its immutable factory geometry while evaluating proposed native intrinsics correctly. Periodic calibration uses only past keyframes, every 60 native seconds when eligible. Factory priors and per-step limits restrain all native DOFs; held-out checks decide commits. Matching thresholds are unchanged.

**Result, S10:** **80.199161**, +13.010782 over the 67.19 reference. One map, 18,343 poses, 14/14 CPs within 1 m, 2,999/2,999 GT associations. Metric SE3 RMSE **16.5321 → 2.4818 m**; CP scale **0.8861150 → 1.0194435**. Evaluated horizontal median error **0.8437 → 0.4015 m**, RMSE **1.0784 → 0.4635 m**. Runtime ~1,786 s for ~917.5 s of input.

**Calibration evidence:** two accepted commits from 15 trials, at native 81.8922 and 382.7922 s. First held-out peripheral medians: cam0 1.5049 → 0.8949 px, cam1 1.1475 → 0.6316 px. Final focal changes about -0.2355/-0.2459 px. All 15 intrinsic variables were allowed to move, rather than fitting radial terms only.

**Remaining limitations:** brief output spikes of roughly 2.7–3 m near 550 s; one successful run is not repeatability. The score gain cannot be assigned solely to intrinsics because recurring VI optimization and lifecycle handling also changed. A periodic-VI/frozen-intrinsics arm is still required. Do not discard or overwrite this successful snapshot while repairing its weaknesses.

**Visualization expectation:** near-complete GT overlap across the full route, broad valid associations in both lenses, brief losses through occlusion rather than sustained divergence; inspect the 550 s spikes honestly.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/online_full_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/diagnostics/report.html
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/comparison_online_v2/report.html
```

### E26. Move rollback snapshot before visual relatching, v3

**Changed:** preserve the IMU-predicted pose/velocity/bias before a visual relatch; rejected visual updates restore that prediction and clear the incompatible marginal prior. The preceding v2 fallback could retain a discarded relatch state.

**Correctness result:** six production regression checks passed; 205/205 logged rejected initialized updates restored the intended state and null prior.

**Full result, S11:** **42.593277**, three maps, selected 7,977 poses and 1,384/2,999 GT samples. Independent maps hold 10,351 additional poses which were correctly omitted rather than stitched. Resets near 463.6922 and 835.8922 s. The first sustained failure starts at 458.6422 s with 14 inliers, before the old bridge failure. Three calibration commits from seven trials.

**Interpretation:** a valid rollback invariant did not make recovery robust. Calibration/keyframe schedules differed before the first rejected rollback, so the full score comparison is not a perfectly deterministic ablation. Small retained-map error is a partial-route statistic, not evidence v3 is more accurate overall. Keep this negative result; do not promote it over v2.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v3_short_full_20261003/online_full_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v3_short_full_20261003/diagnostics/commit_to_loss.png
```

### E27. Intrinsic field and cache audit after the online regressions

**Tried:** project identical native rays through factory, v2, and v3 camera models over the full valid field. Read-only check of active projection/bearing caches after commits.

**Result:** v3's last accepted calibration changed maximum projected pixels by ~0.185 px cam0 and ~0.513 px cam1. v3-versus-v2 differences were also small. Shared camera objects, frame/keyframe intrinsics, stereo depth caches, map depth bounds, projection caches, PnP rays, and bootstrap matrices used refreshed values in the active paths. A stale unused matrix had no consumers.

**Interpretation:** no large sudden lens discontinuity or established stale-intrinsic cache bug was found in those active paths. These fields measure **model displacement**, not error against calibration truth, and do not rule out coupled geometry/state effects. **No new SLAM run or Score2D.**

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/intrinsic_fields/projection_changes.png
```

### E28. Build one full native dual-camera SIFT graph from the 67.19 run

**Implemented:** COLMAP extraction/matching/triangulation on both native images at all 18,343 source poses, retaining Fisheye624. Stereo, temporal offsets, cross-camera temporal pairs, and pose-proposed verified revisits use estimated poses only. Native pixels convert once by +0.5 px to COLMAP convention.

**Result:** 36,686 images, 174,535 matched pairs, **1,715,595 landmarks**, **26,564,121 observations**, including 310,108 cross-camera landmarks. Initial mean track length 15.48; initial mean reprojection error 0.94496 px. Fixed-pose triangulation preserved input poses. Native SDK/COLMAP peripheral projection parity was within 2.34e-13 px.

**Interpretation:** this shared immutable graph makes the three offline objectives comparable. It is a different feature graph from online ALIKED, and its green association counts do not establish online trackability. **Graph construction itself has no new trajectory score.**

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/sift/graph_summary.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/sift/viz
```

### E29. Offline global optical/visual rig BA on that graph

**Tried:** optimize every supplied rig pose and landmark; fixed native intrinsics and physical stereo baseline; first-pose gauge fixed. Start independently from S07's common graph.

**Result, S12:** **64.705686**, -2.482693. 27 accepted updates / 30 attempts; **not converged**, 26 steps hit the linear iteration cap. Visual objective fell 22.097 M → 13.052 M, but CP score worsened. Full coverage retained; 10/14 CP within 1 m. CP scale 0.8823890 and metric SE3 RMSE 17.1446 m. Runtime ~147.6 min, peak process RSS ~20.38 GiB.

**Interpretation:** reduced reprojection cost is not a benchmark guarantee. This finite-budget trial does not establish the best attainable visual BA solution, nor prove every visual-global method will hurt scale. Preserving a metric stereo rig is necessary but was insufficient here.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/visual_rig_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/offline/visual_rig/scientific_summary.json
```

### E30. Offline full global VI-BA with fixed intrinsics

**Tried:** same initial graph plus native IMU across all 18,342 consecutive optimized-pose intervals; loaded 920,016 native samples and retained every intervening native sample within those intervals, with exact boundary handling, per-frame velocity/bias states, bias random walks, and gravity direction on a fixed 9.81 m/s² norm. Loading the full file does not imply integrating its unused prefix/tail. Midpoint covariance/Jacobian contracts were checked before full execution. No GT in optimization.

**Result, S13:** **67.194208**, +0.005828, effectively unchanged. 13 accepted updates / 30 attempts; **not converged**, 18 linear-capped steps. CP scale 0.8861126, metric SE3 RMSE 16.5324 m: scale was not repaired. Final objective ~15.709 M. Final-bias exact IMU reintegration changed IMU cost by only 1.05e-7 relative. Runtime ~60.2 min, peak RSS ~23.45 GiB.

**Interpretation:** this full VI graph did not automatically repair the baseline shape/scale within its budget. Tiny score change is not a meaningful win. Noise weighting, conditioning, state initialization, convergence, and graph quality remain experimental questions, not proven explanations.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/vi_fixed_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/offline/vi_fixed/scientific_summary.json
```

### E31. Offline full global VI-BA with all native intrinsics

**Tried:** same full graph and inertial factors plus each camera's 15 independent native parameters: shared focal, cx/cy, six radial, two tangential, four thin-prism. COLMAP stores 16 values because fx=fy is duplicated. Factory-centred Gaussian priors; ±3σ lifetime limits, ±0.15σ per attempted calibration step, native inverse validity checks. Physical rig and camera/IMU extrinsics stayed fixed.

**Result, S14:** **67.183330**, -0.005050, effectively unchanged. All 30 attempts accepted; **not converged**, no linear-capped steps. Final reprojection RMS 0.88544 px; visual objective 22.097 M → 9.786 M, but trajectory score/scale hardly moved. CP scale 0.8861252. Bounds were respected; at least one final parameter reached the allowed 3σ boundary. Final-bias exact IMU audit shifted cost by only 2.03e-7 relative. Runtime ~57.2 min; peak RSS ~28.93 GiB.

**Interpretation:** the online 80.20 improvement is not reproduced by simply freeing intrinsics in a final offline solve of this graph. The offline and online graphs/history differ, and all batch arms are budget-limited, so this does not establish a universal verdict against calibration or BA. Parameters at a prior bound need a physical/observability check, not automatic wider bounds.

**Output status:** estimator solve, export, and official local score are complete. **The calibration Rerun was stopped at the user's request and is partial/unverified. Do not present it as a complete result or silently resume it.** Diagnostics and numerical outputs are preserved.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/offline/vi_calib/scientific_summary.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/calibration_diagnostics/metrics.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/vi_calib_evaluation/run.PARTIAL_STOPPED.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/vi_calib_evaluation/STOPPED_INCOMPLETE.json
```

### E32. Numerical infrastructure and failed attempts supporting the offline suite

These are **not additional independent algorithm wins or scored trajectories**:

- Native Fisheye624/IMU contracts, midpoint covariance checks, and synthetic global scale-recovery fixtures. Seventeen contracts passed; full synthetic visual/fixed-VI/calibrated-VI cases recovered a planted scale error to tiny numerical residuals. This establishes implementation behavior on controlled inputs, not real-data conditioning.
- A private parallel Ceres sparse matrix-vector product improved the measured exact-pattern kernel ~3.31×, preserving numerical parity and ABI. It did not change factors or claim a score improvement.
- The implicit-Schur calibrated attempt was deliberately stopped after eight accepted updates due to expensive iterations. No completed geometry checkpoint existed, so it was never scored.
- A subsequent explicit-Schur calibrated attempt exceeded its 30 GiB cap and was killed by confirmed Docker OOM before a joint update. Again, no scored result.
- The successful calibrated execution temporarily released 1,523,300,232 bytes of duplicate immutable observation metadata while keeping optimization parameter storage stable. It restored all 36,686 image records, 40,962,102 features, and 26,564,121 track entries with matching hashes before final audit/export. No measurement graph was thinned for RAM.
- Final pruning of unprojectable observations/under-supported points is explicitly counted: visual 163 removed observations; fixed VI and calibrated VI 355 each. This is distinct from resource-related graph thinning.

**Lesson:** future large runs need a justified graph size, memory profile, checkpoint/restart, and convergence plan before spending hours. Preserve attempts for provenance but keep the scored ledger tied to completed canonical outputs.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/profiles
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/solver_source_metadata
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/pipeline/vi_ba_lamaria/EXPERIMENTS.txt
```

### E33. Exact evaluated Reruns and score audits

**Implemented:** display the saved official CP similarity, consistently applied to trajectory, both cameras/rig lever arms, landmarks, and histories; no separate alignment for cloud or dense GT. Reconstruct all score-associated errors from exported geometry. Keep missing poses absent and maps separate. Preserve final optimized map growth semantics explicitly.

**Result:** baseline, online v2/v3, visual BA, and fixed VI recordings are complete and verified. Offline visual/fixed VI include all 18,352 native images per lens, including the unsupported tail image; older online/baseline recordings contain 18,351 source-processed images. All retain original score denominators. The calibrated offline recording alone is partial as above. Adapter repairs preserve distinct SIFT feature indices even when one landmark has more than one observation in the same image; exact duplicate (image, feature) pairs remain invalid.

**Interpretation:** diagnostic integrity work, not an estimator improvement. It answers the user's concern that a Rerun should show what was actually evaluated. Final optimized landmarks revealed at their first retained observation are not a historical replay of every backend point update.

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/VISUALIZATIONS.txt
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/comparison_progress/control_point_score_breakdown.png
```

### E34. Explain the missing 19.80 points in the 80.20 Short result

**Question:** Why does the evaluated Rerun appear to lie directly over GT while S10 scores 80.199161 rather than 100?

**Tried:** read the preserved S10 control-point estimates and invoke the unchanged official error/score functions in memory. No SLAM replay, new alignment, calibration, or trajectory modification was performed. Recalculation exactly reproduced **80.19916100650815**.

**Result:** all 14 CPs are recovered and all 2,999 GT timestamps are associated. The entire **19.800839-point deficit** comes from nonzero horizontal control-point position errors, not missing CPs or missing GT poses. CP errors range **10.709–80.341 cm**, with **39.335 cm median** and **41.398 cm mean**. There are zero CPs within 5 cm, two above 5–20 cm, eight above 20–50 cm, and four above 50–100 cm. The worst four account for **8.234899** of the lost score points; the remaining loss is spread over the other ten.

The official per-CP credit is 20/20 through 5 cm, 18/20 at 20 cm, 15/20 at 50 cm, and 12/20 at 1 m, with interpolation between knots. Thus being within 1 m gives full CP recall but does not give full Score2D. The result earns 224.557651 of 280 available CP credits. On a whole-route view, tens of centimetres can be hidden by line thickness. The Rerun also applies the evaluator's saved CP Sim3, including scale **1.0194434596**; this removes one global scale discrepancy for display/evaluation, not local errors in the estimated route or CP geometry.

**Verdict:** the visually strong overlap is consistent with the saved score. Improving this Short score requires more accurate control-point geometry toward centimetres, rather than merely recovering more CPs. Dense trajectory XY RMSE 0.463521 m is a separate diagnostic and is not averaged into Score2D.

**Visual review:** a perfect result has every CP error bar at or below the 5 cm line and every lost-score bar at zero. The companion residual plot places each surveyed CP at the origin; every residual dot should lie inside its 5 cm circle. These diagnostics expose small offsets hidden at whole-route scale without changing the submitted trajectory.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/diagnostics_score_explanation_20261003/score_explanation.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/diagnostics_score_explanation_20261003/control_point_score_breakdown.png
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/diagnostics_score_explanation_20261003/route_and_control_point_residuals.png
```

### E35. Transfer the unchanged 80.20 method to full Medium

**Tried:** run the frozen online-full-v2 executable on `sequence_2_11`, preserving that sequence's native camera, rig, and IMU calibration. Copy only the same eight method settings from S10: each camera's valid radius 330 px and maximum angle 1.4 rad; fixed-camera initialization control mode 1; 12-second minimum metric window; full online calibration enabled; 60-second interval. Use the same cached feature/IMU inputs, temporal associations, four-CPU limit, diagnostics and debugger. No GT enters estimation.

**Provenance:** the executable, shared library and Docker image match the saved S10 hashes exactly. Runner SHA256 `a0efe826e73155b3f307882cb5176ec630eca9bd2147cba04cbcd395eec93f66`; library SHA256 `7664755942310a1513cde99aeb47d3c48c4cf17a5412196b970b6315018c3414`. This is a transfer of the complete v2 method, not an isolated calibration ablation.

**Result, S15:** **62.764799**, CP **13/18 triangulated and 13/18 within 1 m**, GT **2,499/4,083 associated**. All **23,746 supported frames** were processed normally in **746.75 seconds**, from native 68.437375 to 1255.687394 s. Four full-calibration updates were accepted from 12 completed trials. The atlas retains **three independently framed maps**, with 22,888 exported poses total (96.39% of supported inputs); these do not form one global trajectory. There were four completed active-map resets and three new-map events, including the initial map.

| Retained map | Poses | Native interval, seconds | Official score selection |
|---|---:|---|---|
| `map_0_init_0_body0` | 14,131 | 71.137375–777.637375 | Largest map, selected without GT |
| `map_1_init_2008_body0` | 8,558 | 799.587394–1227.437394 | Omitted; independent coordinate frame |
| `map_2_init_2999_body0` | 199 | 1245.787394–1255.687394 | Omitted; independent coordinate frame |

The score's top-level `map_epochs: 1` describes the selected trajectory, while `trajectory_selection.atlas_map_epochs: 3` describes the surviving atlas. The selected map covers **59.50% of the full native inputs**; 8,757 poses in other maps are not stitched into it. Missing GT associations are four before the selected trajectory and 1,580 after it. PoseRecall@5m is **61.2050%** and PoseRecall@1m **60.6417%**.

**Accuracy versus missing coverage:** the five missing CPs cost **27.777778 score points**. The remaining nonzero errors in the 13 recovered CPs cost **9.457423 points**. These account for the full **37.235201-point deficit**. The associated trajectory is accurate: official CP-aligned horizontal median **0.219637 m**, RMSE **0.338146 m**, with CP scale **1.010361083**. On identical largest-map GT associations, raw-metric SE3 3D RMSE is **0.758592 m**; a separately labelled diagnostic dense-GT Sim3 gives **0.403588 m**, scale **1.009803528**. That extra fit is not used for scoring or the evaluated Rerun.

**Verdict: mixed.** The score exceeds the old S02 Medium baseline by 7.012351 points, and the retained segment has good geometry. Continuity is still inadequate: three maps survive and five CPs are missing. Full input processing must not be reported as a full global trajectory. The method's 80.20 Short result has not transferred into equivalent full-route Medium reliability.

**Visual review:** the evaluated Rerun contains all **23,746 images per lens** with red/green feature overlays and the selected map revealed over time. It displays only the scored map in 3D, with an explicit coverage panel; both videos continue after that map ends. RRD verification and an independent audit of image rows, both camera transforms, trajectory timestamps, scorer errors, and **46,550 displayed landmarks** passed. Eight distant landmarks are omitted by the declared display filter; no point-count downsampling is applied. A perfect result would retain a single metric route through the later GT continuation, rather than only accurately overlaying its first section. The static top view makes this missing continuation visible. Rendering took 450.87 s; the independent audit took 4.30 s.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_medium_full_20261003/result_summary.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_medium_full_20261003/online_full_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_medium_full_20261003/evaluation/viz/path_topdown.png
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_medium_full_20261003/evaluation/evaluation_geometry_audit.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_medium_full_20261003/diagnostics/report.html
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/configs/orbslam3_lamaria/online_full_v2_transfer_20261003/verification.json
```

### E36. Long transfer: incompatible inherited calibration, then native-input correction

**Initial attempt:** transfer the same frozen v2 method to the existing Long `sequence_3_17` configuration. It aborted at its first full online calibration construction with `std::invalid_argument: Expected native single-focal Fisheye624`. **No official score is available for this failed attempt.** The last completed tracking diagnostic, frame 538 at native 179.249314 s, still had 272 inliers and completed inertial BA2; this was not observed tracking collapse.

**Cause, verified:** the initial configuration was inherited from an earlier calibration refinement, despite its native-image naming. All 32 lens scalars exactly match the prior COLMAP/SIFT `refined_native.json` from 2026-09-30. Its cameras have independently refined fx/fy: 242.148802/242.277555 and 241.644785/241.797036. The native Aria model has one focal parameter per camera; v2's 15-DOF calibration vertex rejects a factory-vector fx/fy difference above 1e-4. `Tracking::GrabImageStereo` snapshots those input parameters before optimization, so an online update did not create this mismatch. The missing early input validation allowed the incompatible configuration to reach the optimizer constructor.

**Corrected input:** create a fresh `long_native.yaml` using all 32 exact native VRS lens scalars, including equal focal pairs 241.6048925567284 and 241.3272717603338. Preserve every non-lens configuration line and the same eight v2 method settings. Verify rig/stereo and body-camera transforms against the native VRS: maximum element differences 2.07e-8 and 1.21e-8 respectively. Native 640×480 dimensions, 20 Hz cameras, 1,000 Hz IMU, radius 330 px, and 1.4 rad angular domain agree. The existing baseline 0.137749 m agrees with the native 0.137749093 m to its saved precision. Original inputs and the failed run remain preserved.

**Completed corrected run, S16:** **34.458901**, CP **15/27 triangulated**, **9/27 within 1 m**, GT **3,163/6,118 associated**. All **35,842 native frames** were processed normally in **1,192.70 seconds**, spanning native 152.349295–1944.399295 s (1,792.05 s of input). The estimator and coverage checks exited zero. There were **13 completed active-map resets** and **five surviving independently framed maps**, containing 33,306 poses in total (92.92% of input frames). Only **one full calibration trial** completed and was accepted, at native 179.149314 s; this is not a result with successful recurring calibration throughout all five maps.

| Retained map | Poses | Native interval, seconds | Official score selection |
|---|---:|---|---|
| `map_0_init_29_body0` | 997 | 163.499314–213.299295 | Omitted; independent coordinate frame |
| `map_1_init_203_body0` | 17,160 | 224.549314–1082.499295 | Largest map, selected without GT |
| `map_2_init_2298_body0` | 10,586 | 1082.649295–1611.899295 | Omitted; independent coordinate frame |
| `map_3_init_3586_body0` | 1,291 | 1663.299295–1727.799295 | Omitted; independent coordinate frame |
| `map_4_init_3837_body0` | 3,272 | 1780.849314–1944.399295 | Omitted; independent coordinate frame |

The selected map covers **47.88% of native inputs**; 16,146 poses from the other maps are not joined to it. `trajectory_selection.atlas_map_epochs` is five; top-level `map_epochs` is one because the score uses one map. The missing GT count is 2,955 (273 before and 2,682 after the selected map). PoseRecall@5m is **51.6999%**, and PoseRecall@1m **32.6741%**.

**Score loss and geometry:** twelve missing CPs cost **44.444444 score points**; errors in the fifteen recovered CPs cost **21.096655 points**. Official CP-aligned trajectory horizontal median error is **0.710571 m**, RMSE **1.331364 m**, p90 **1.960877 m**; CP scale is **0.994901829**. On the same selected-map GT associations, raw-metric SE3 3D RMSE is **1.499042 m**, while a separately labelled diagnostic dense-GT Sim3 gives **1.331182 m**, scale **0.993332217**. These selected-segment errors must not be compared as a full-route accuracy improvement over S01's complete Long metric error.

**Verdict: negative transfer result.** The input contract is corrected and the full replay completes, but the 80.20 Short method does not preserve a single Long trajectory. Score is below S01's 49.881534 and coverage is substantially worse. This comparison changes both the estimator variant and the old Long lens input: the historical baseline used a previous two-focal COLMAP refinement, while S16 starts from verified native factory intrinsics. Therefore it is not a pure calibration or algorithm ablation, and the result does not identify native calibration as the cause of fragmentation. No source patch, blind fy-to-old-fx assignment, GT-assisted map joining, or additional SLAM retry was used.

**Visual review:** the exact evaluation-aligned Rerun is complete and verified. It displays only the scored map in 3D, with explicit coverage information, all **35,842 images per lens**, **17,160 poses**, and **76,926 landmarks**. RRD verification and the independent geometry/image audit passed; there are zero missing images, no landmark downsampling, and no omitted landmarks from the selected map. Rendering took **779.87 s**, and the independent audit **33.08 s**. A healthy result would preserve all five portions in one measured global map and recover the missing CPs. The current top view shows the accurate and inaccurate parts of the selected segment against the full GT route; it does not present unaligned maps as a single trajectory.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_full_20261003
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003/result_summary.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003/online_full_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003/evaluation/viz/path_topdown.png
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003/evaluation/evaluation_geometry_audit.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003/diagnostics/report.html
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/configs/orbslam3_lamaria/online_full_v2_transfer_20261003/long_native.yaml
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/configs/orbslam3_lamaria/online_full_v2_transfer_20261003/long_native.audit.json
/media/raghav/HardDrive1/Mecka/lamaria/bench_basalt/calib_refine/native_sequence_3_17/refined_native.json
```

### E37. Three full Medium/Long continuity candidates

Implemented separate frozen-v2-derived patches: **A**, direct binary-descriptor
recovery without ORB vocabulary partitions; **B**, 0.5-second accepted-landmark
appearance memory under native projection gates; **C**, fixed-intrinsic periodic
VI with map-local scheduling after fragmentation. A/B share provisional-relatch
rollback and grace handling. No geometric gates were loosened; GT was evaluation
only. All six full replays exited normally; native production contracts passed.

Results S17–S22 above are **not promotion results**. B's Long 42.904563 beats the
native-v2 transfer by 8.445662, with 68.36% selected native coverage, but still
has six maps and falls below the older 49.881534 continuous Long. B regressed
Medium to 45.874750. A/C Medium lacked the height-supported CPs required for the
official alignment and are unscorable, not assigned fabricated zeros. C accepted
zero of 18/25 periodic proposals, so no periodic correction reached live state.

A exposed a recovery acceptance gap: >=50-inlier visual recovery is followed by
local VI that can discard support and commit through the >10 RECENTLY_LOST
branch. Medium committed 16/20/21-inlier solutions displaced 5.49–8.52 m from
visual hypotheses; Long committed an 11-inlier solution after 84 visual inliers.
This shows a support/acceptance mismatch, not which pose is GT-correct or that
all breaks share this cause. Existing VI already optimizes pose/velocity/bias;
valid temporal anchors are not intrinsically wrong. A dedicated recovery
confirmation/state transition remains the next implementation decision.

Full technical report: `docs/CONTINUITY_EXPERIMENTS_20261003.md`.
Exact source/log trace: `docs/RECOVERY_HANDOFF_20261003.md`.
Seven-case hashes/results: `docs/CONTINUITY_RESULTS_20261003.json`.
Diagnostics are under the project artifact batch `lamaria_continuity_batch_20261003`.
No new Rerun recording was rendered; prior evaluated recordings are preserved.

### E38. One unchanged Medium repeatability control

S23 used byte-identical frozen-v2 library, runner, settings and timestamps. It
scored **63.086620**, versus the saved 62.764799, with nearly identical selected
native coverage (59.4787% versus 59.5040%). The first split repeats at 777.387 s
versus 777.687 s. Four maps survive versus three; internal calibration and
recovery outcomes also vary (5 versus 4 calibration commits; 16 versus zero
visual recovery hypotheses accepted). This is not an algorithmic score gain,
and one repeat does not establish variance. It does confirm the broad Medium
failure location and does not explain away B's 45.87 regression. No candidate
was retried or selected for a favorable run. Stop for discussion before another
implementation family; knot-based refinement remains deferred.

### E39. BabyFeatures SOS, one full Long replay

**Tried:** on `experiments/BabyFeats`, keep a passive short-track cache from the
existing features. After ordinary tracking fails, jointly optimize temporary
native-camera points and recent pose/velocity/bias states with IMU. Baby factors
do not enter healthy tracking. Verified three-view tracks can seed a temporal
keyframe in the same map, followed by actual ordinary-map tracking confirmation.
Normal association/reprojection gates, native Long settings, and frozen v2
products were preserved. Fourteen native numerical solver checks passed.

**Positive continuity result:** the full 35,842-frame replay exited normally.
After one startup reset, one retained map covered 35,619 consecutive inputs
(99.377825%), from native 163.499314 to 1944.399295 s. The first 223 inputs are
absent from the final trajectory; no gap filling or independent-map stitching
was used. All 15 SOS episodes returned to normal tracking, with 72 accepted
Baby updates and 13 promotions totalling 1,648 landmarks. All 72 SOS poses reached
the scored export, still labelled as coasting rather than mature tracking.

**Mixed accuracy result:** S24 scored **43.729292**, +9.270391 over native v2 S16
(34.458901, five maps), but below historical Long S01 (49.881534, different
starting calibration). All 27 CPs were recovered, but only four were within 1 m.
Associated horizontal trajectory error has median 1.896 m and RMSE 2.933 m.
The official CP Sim3 scale is 1.008174. A 4.276 m online step at native
1607.199295 s shrinks to **1.619 m in the final export**, so map survival is not
equivalent to a smooth or accurate trajectory. Thirteen return-window
refinements succeeded; two lacked history after backend cache invalidation.

**Attribution limits:** this is one replay, not a repeatability result. Passive
matching changes asynchronous scheduling; this run survived the saved v2's
first split before the first Baby solve. Keeping a single map also enabled
29 inherited periodic calibration trials, four accepted. The score is the
result of the complete pipeline; it is not an isolated Baby-factor accuracy
gain. No Short or Medium Baby run was performed, and no frozen baseline was
replaced. Stop here for discussion before another implementation family.

Full report and exact evidence: `docs/BABYFEATURES_LONG_20261003.md` and
`docs/BABYFEATURES_LONG_20261003.json`. Diagnostic PNGs include the full evaluated
route, map continuity, first-SOS camera views, and the remaining late SOS jump.
No new full Rerun was rendered during the initial fast experiment. The later
requested full recording completed on 2026-10-04; see E40 and its report.

### E40. Final fixed-camera VI on the saved Baby Long native graph

Latest session ended for the user's usage limit. Documentation only after this
completed result; no new experiment follows it. User flags missing startup,
terminal drift and apparent tilt. Startup loss is confirmed, the tilt/drift
mechanism is not. Resume context: `docs/RESUME_20261004.md`.

**Tried:** one offline sparse-Schur VI solve on S24's 2,401 keyframes, 156,557
landmarks and 1,119,060 native observations. Last accepted lenses and physical
rig/IMU extrinsics remain fixed. No new SIFT extraction or SLAM replay. Calibrated
native IMU reconstructs velocity/bias/gravity states, followed by joint keyframe
pose/XYZ/inertial-state optimization. GT remains evaluation-only.

**Modest positive result:** S25 **45.027586**, +1.298294; identical 35,619 timestamps,
99.377825% native coverage, one map and 27/27 CPs. CP within 1 m increased 4→5;
dense horizontal RMSE 2.933179→2.754464 m. Sixteen CPs improved, eleven worsened.
The remaining 1.618828 m step became 1.622238 m: not repaired. Historical Long
49.881534 remains stronger and preserved.

Eight adapter/history tests plus the synthetic native VI contract passed. The
real solve took 78.52 s, reported convergence at a cost plateau after 39 outer
calls / 15 accepted updates / 23 rejected trials. Objective decreased 14.96%;
final projectable pixel RMS 0.803250. Exact IMU reintegration changed inertial
cost by only 2.96e-7 relative. This is local numerical termination, not proof of
optimal geometry. Invalid-projection/support pruning removed 35 observations
and five points. No acceptance thresholds were relaxed. Non-keyframe poses retain
their original relative transform to their corrected reference keyframe; they
were not separately optimized or interpolated.

**Visualization:** `comparison/comparison.png` under the S25 experiment shows the
matched routes and all CP bars. The separate full S24 Rerun is now verified at
`experiments/lamaria_babyfeats_long_20261003/evaluation_baby_20261004/run.rrd`, with
two normal and two Baby panels plus SOS map colours. Baby identities are passive
descriptor reconstructions, not recorded solver inlier IDs; 453 logged count
comparisons agree. See `docs/BABYFEATURES_NATIVE_VI_20261004.md` for absolute paths,
limits and provenance. Further runs are deferred for visual discussion.

### E41. Read-only decomposition of the Baby Long startup, drift and tilt

Before any new run, the user's three Rerun observations on S24 (missing beginning, end
drift, apparent tilt) were decomposed from the saved outputs only, with every number
re-derived on a second code path. New script `pipeline/eval/decompose_lamaria_errors.py`
(Stages 0-2; 15 synthetic contracts in `pipeline/eval/test_decompose_lamaria_errors.py`),
output `experiments/lamaria_babyfeats_long_20261003/diagnostics_startup_drift_tilt_20261004/`
with a provenance file (`evaluation_only`, `GT_used_in_estimator: false`,
`new_official_score: false`). No estimator, solver or Rerun change; no new score.

**Tried:** preflight (input hashes, cam0 rebuild to 6e-8 m, official per-timestamp errors
reproduced to 7e-13 m), a heading-integral test (do orientation-derived yaw errors predict
the position error?), swing-twist about GT z, 15 m windowed displacement ratios, an
online-vs-final stretch comparison and an event ledger (calibration commits, loops, SOS).

**Established:** the 223 missing start frames are 71 before any map, 148 of a visual-only
first map that lost tracking in a 0.5 s blur/detector-dropout burst at 163.25 s while the
12 s IMU gate was still open, and 4 re-init frames. The end drift is local scale error born
in live tracking (15 m-window est/GT displacement ratio 0.85-0.91 after 1800 s versus
0.97-1.02 mid-route; yaw explains about 10 %; no SOS step moves the error by more than
0.16 m). The start error is a 1.1 deg heading error over 282 m plus about 1 % local scale
excess. The estimator's gravity is within 0.3 deg of GT vertical from orientations alone;
the visible 4 deg tilt belongs to the 27-control-point Sim3, whose 13 height CPs are all
late and sit on the drifted segment. The viewer applies exactly `CP_sim3`.

**Decision:** the scale-born-in-tracking finding pointed at the inertial bias freedom and
led directly to the IMU random-walk arm of E42.

### E42. Mechanism arms after the decomposition: rollback, IMU bias walks, speed bound

User steering: many runs, each carrying a real fix or idea (config-only variants were rejected),
scored with the official local LaMAria evaluation and tabulated. Round report
`docs/BATCH_20261004.md`; cross-run table
`experiments/lamaria_batch2_20261004/table_r5/table.md` (25 rows, recalls and dense-GT ATE
diagnostics). Tooling: `pipeline/run/batch_runner.py` (parallel cases, CPU-aware stall
watchdog, attach mode), `pipeline/eval/experiment_table.py`,
`pipeline/datasets/extract_aliked_dense.py`. All arms: Baby SOS on, one container per arm,
4 CPUs, saved CP Sim3 only, no GT in the estimator, nothing frozen modified.

**Positive.** Medium **78.590** (`coastguard_medium`: rejected-update rollback
`Tracking.coastRestore: 1` plus a 3 m/s coast speed guard that never mattered): one map,
17/18 CPs within 1 m, median 0.35 m, level with the Meta Medium reference 78.5. Rerun
verified at `experiments/lamaria_batch2_20261004/coastguard_medium/evaluation_baby_20261004/run.rrd`.
Medium 74.408 (`coastguard_imuwalk10_medium`: rollback + IMU bias random walks /10): the only
Medium run whose maximum error stays under 2.1 m end to end (final coast tamed, last CP
1.70 m) at the cost of 1.5 m drift over the last 250 s. Long **49.186** (`imuwalk10_long`,
walks /10; Rerun verified at `.../imuwalk10_long/evaluation_baby_20261004/run.rrd`), 50.042
with 1x noise densities (`imuwalk10dens1x_long`), 49.415 with offline fixed-camera VI on the
49.19 graph; Long 45.411 with walks /30 (tightest tail, max 4.3 m, but mid-route 2.2-2.8 m).
Short 81.393 (`imuwalk10_short`, 14/14 CPs; within run noise of the frozen 80.20).

**Negative, recorded with the reason.** Gyro-only pre-init coasting (Medium 37.64, Long
18.29: the kept startup map initialises worse). Coast-age-widened projection search
(Medium 21.75: wrong relatches during a 57 s coast). Dense ALIKED caches, 3000 keypoints at
threshold 0.05 (Medium 44.66, Long 36.16: more keypoints, worse geometry). A soft pedestrian
speed bound (|v| <= 2.2 m/s, sigma 0.01, `patches/orbslam3_lamaria_speedprior_20261004`) in
pose-only inertial optimisation, local inertial BA, the Baby solver and IMU propagation
(Medium 19.98: the pose-only prior fights the fixed map whenever the estimate exceeds the
bound, the run coasts 840-940 s and a mid-route kink misfits the single Sim3; the tail was
bounded, 10 m instead of 222 m). Walks /10 alone on Medium split at 882 s (56.77). Rollback
on Long (44.19 and, with walks /10, 38.47) drew bad startup lotteries; its one 103 m tail came
from coasts at 3-7 m/s on a poisoned velocity.

**Mechanistic facts established.** GT walking speed never exceeds 1.84 m/s on any sequence
(read-only check); the estimator reached 2.5-5 m/s only where scale crept or a coast ran on a
bad velocity. The Medium 860-890 s split is an inertial-state failure (a rejected visual
update leaves 6.7 m/s in the frame; coasting then diverges to 41 m/s), cured by rollback. The
Long end drift is bias-absorbed scale creep, cured by /10 random walks; /30 over-constrains.
Long run-to-run spread (38.5-50.0 with one map each) follows the startup lottery (early CPs
4-8 m; worst with three pre-init resets). Periodic full-map calibration trials on 1,400-1,650
keyframes take 10-25 min each, single-threaded; the watchdog must be CPU-aware.

**Contracts.** 14 native Baby solver checks on every new library; 11 (12 for the tail variant)
speed-bound checks (analytic Jacobian versus central differences, bound reached along the
velocity direction, untouched below the bound, equal-information compromise, propagation
clamp); a 120 s Medium startup replay with zero firings and one map before the full arms.

**Tail-only variant** (`patches/orbslam3_lamaria_speedprior_tail_20261004`, bound only on
Baby-solver and IMU-propagated velocities): Medium 16.17, negative. Clamping the propagated
velocity after the position has been propagated with the unclamped one leaves an inconsistent
prediction; from 640 s matching fails, 2,159 frames coast and the error reaches 28-54 m by
600-800 s while the same configuration without the bound scored 78.59. The speed bound is
dropped in every form. Long 45.23 with the bound never firing: a repeat of the 49.19
configuration with identical startup (retained map from 163.35 s), so the Long run-to-run
spread is about ±2 even without the startup lottery. Table:
`experiments/lamaria_batch2_20261004/table_r5/table.md` (25 rows).

### E43. Round 3: reproducibility repeats, bias hold, Baby assist before loss

User steering: name the workers, what each implements, which problem it solves and what to expect;
Baby support may act before tracking is lost but main tracks must dominate and Baby landmarks stay out
of the map. Report `docs/BATCH3_20261004.md`; table
`experiments/lamaria_batch3_20261004/table_r1/table.md`.

**Repeats (W1-W3), byte-identical configs:** Short 79.79 (was 81.39): Short is 80 +-1 for every config
tried. Medium 25.35 (was 78.59, one map both times) and 61.37 with a split at 842 s (was 74.41): Medium
is decided between 640 and 944 s, where the open plaza thins tracking and Baby bridges; the 78.59 run
drew no mid-route blackout. Long 49.19 / 45.23 / 50.37 on one config: spread about +-2.5.

**Bias hold during visual starvation (W4 Medium 61.74, W5 Long 50.37;
`patches/orbslam3_lamaria_biashold_20261004`).** Biases held exactly (deviation 0 in all 39 and 14
windows) while coasting, RECENTLY_LOST or Baby-bridged; 9 contract checks. Neutral: the Medium runaway
coast (840-944 s, 17 m/s, split) happened with biases frozen. Its accepted Baby solves changed the frame
velocity by 1.14 m/s per frame on average (max 4.59, bound 5), against 0.036 m/s in a surviving bridge.

**Baby assist before loss (W6 Long 46.90, W7 Medium 82.88;
`patches/orbslam3_lamaria_babyassist_20261004`).** Monitor: previous-frame inliers below 80 or local-map
matches below 5 % of keypoints for five frames (thresholds calibrated on healthy frames: the 10 % ratio
fired on 12-16 % of frames and was rejected). One Baby window solve on a copy of the frame with
ordinary-tracked frames fixed; verified landmarks enter the pose-only inertial optimisation as
temporaries with information x 0.2 and Huber, never as MapPoints; 10 contract checks. Long: active on
4.3 % of frames, no effect on the 1700-1760 s step, within spread. **Medium 82.88, one map, 17/18 CPs
within 1 m, max error 4.0 m, above the Meta reference 78.5:** assist on 7.6 % of frames carrying 11 % of
the map cost; the 640 s and 760 s blackouts shrank to 10 and 5 frames (repeat without assist: 176 and
403), the 840 s coast and the 1220 s tail were survived at walking speed. One sample so far.

**Second half of the round (user: launch repeats, W8, W9; test the knot idea; try global VI).**
Two byte-identical repeats of the 82.88 run scored 56.79 and 61.33, both one map: with assist every
Medium run kept one map (4 of 4) but accuracy varies with the draw (a slow 2 % scale drift through
500 s of thin tracking; a 17 s blackout whose bridge deformed the map; the final-section loss running
away in two of four). W8 assist + hold + kinematic gate (`patches/orbslam3_lamaria_assist_kin_20261004`):
35.17, three maps, negative: 89 rejected bridge solves (steps 0.50-1.83 m/s) let two SOS episodes expire.
W9 assist + hold + gate + stereo-fused Baby tracks (`patches/orbslam3_lamaria_assist_stereo_20261004`,
17 solver checks): 62.95, one map, mid-route bridges consistent but only about 13 left-right pairs per
frame; neutral. Knot test (read-only): control-point error equals the trajectory error at the tag frames
(median ratio 0.96-1.00 on all sequences), so local accuracy at the tags cannot move the score; only
tags as metric scale anchors or loop closures (one revisited tag, Medium) could. Offline fixed-camera VI
on finished runs: Medium 82.88 -> 82.53 (18/18 CPs within 1 m), Long 50.37 -> 49.08; not a lever; the
export tool gained `--missing-reference nearest`. Table
`experiments/lamaria_batch3_20261004/table_r2/table.md` (18 rows).

## 4. What these results establish and what they do not

1. The user's insistence on end-to-end geometry was justified: bearing contracts, map units versus a metric baseline, camera-specific indices/observations, invalid-domain inversion, and recovery state consistency all contained real issues. The correct response is to trace complete geometry/state contracts, not move the cloud until it looks right.
2. The project moved from crashing/resetting and exporting fragments to a full Long baseline and then a full Short result of 80.20. This progress is real and measured, but not yet demonstrated across the benchmark or repeated seeds/schedules.
3. Correctness fixes can expose other problems or change the trajectory unfavorably. “This invariant is fixed” and “the benchmark improves” must remain separate statements.
4. More matches, green dots, or lower BA cost are supporting evidence. They are not success criteria by themselves. Retained static constraints, conditioning, scale, coverage, and final errors matter.
5. Factory calibration is neither certified perfect nor proven to be the sole cause. Held-out radial evidence, the negative one-shot radial replay, the positive online full replay, and the nearly unchanged offline calibration all have to be explained together.
6. The 80.20 run is the current best observed Short candidate. Its transient recovery jumps and the v3 regression mean it is not yet the final robust baseline for every sequence.
7. No documented DA3/depth initializer, MegaLoc loop integration, joint extrinsic calibration, time-offset calibration, or full benchmark-category submission has been completed in this ledger. Plans are not results.

## 5. Many-month experiment backlog, prioritized by evidence

These are proposed experiments, not additional runs authorized by this document. ROADMAP.md gives implementation detail. Every proposal needs a parent checkpoint, one main question, metrics, a stopping rule, and a visual review before widening scope.

| Priority / target | Proposed experiment | Why it may help | What could make it fail | Evidence required to continue |
|---|---|---|---|---|
| First: repeatability | Repeat frozen v2 and S07 on Short, then Medium and Long; capture scheduling/config provenance | Establish whether 80.20 survives ordinary run variation | Hidden mutable state, race-sensitive scheduling, scene-specific tuning | Stable score/coverage distributions; identical-input artifacts; no unexplained resets |
| First: attribute 80.20 | Periodic full VI with **fixed intrinsics**, same schedule/holdout/state code as v2; compare full calibration | Separate benefit of recurring optimization from lens freedom | Different KF selection means same nominal schedule is insufficient | Saved common snapshots for paired solves plus complete causal replays |
| First: recovery | Transactional pose/velocity/bias/prior handling and rig-aware relocalization from either camera | Eliminate jumps without throwing away recoverable maps | Conservative rejection creates v3-like fragmentation | No inconsistent prior; accurate fixed-anchor passage and full-sequence recovery |
| First: initialization | Adaptive multi-frame stereo-inertial initialization, information-based acceptance, gravity/bias uncertainty | Use metric stereo and motion together; avoid frozen-geometry scalar conflict | Weak stereo, little acceleration, poor overlap, correlated depth noise | Scale near one without GT feedback, bounded uncertainty, fast full coverage |
| High: learned depth | Confidence-weighted DA3 or another depth prior on native-consistent rays; metric anchoring from stereo/IMU | Add initial structure when native overlap/texture is weak | Learned mono scale bias, fisheye domain shift, dynamic objects, hallucinated depth | Improvement over strong stereo/IMU initialization on held-out routes; prior can be rejected |
| High: local visual/VI BA | Same captured graph, visual versus VI factors, window sizes, keyframe retention, robust weighting | Improve medium-range conditioning and prevent weak local geometry accumulating | Larger windows increase latency; poor IMU weights oppose valid visual constraints | Relative drift by distance/time, full score, latency/backlog, camera-specific residuals |
| High: genuine medium-range tracks | Retain/reacquire static landmark tracks over seconds and viewpoint changes; use descriptor histories and multi-view verification | Add constraints beyond frame-to-frame motion without waiting for a loop | Aliasing, dynamic tracks, descriptor aging, overconfident correlated observations | Track-age/support distributions, native projection overlays, controlled outlier rate |
| High: feature contract | Compare native float ALIKED matching, trained retrieval vocabulary, other learned matchers, and ORB at matched compute | Remove binarization/vocabulary/octave mismatch; strengthen revisits | More raw matches may be less geometrically useful; GPU/latency cost | Verified static inliers per compute budget, periphery support, full score |
| High: loop closure | Port existing MegaLoc retrieval from Mecka Basalt; native rig verification; gravity-consistent metric graph correction | Use real revisit constraints on Medium `sequence_2_11` | False loops catastrophically distort maps; appearance retrieval alone does not estimate pose | Top-view accepted/rejected loop edges, native pair overlays, pre/post scale and score |
| High: loop/global optimization | Compare metric SE3/4DOF graph correction plus VI refinement with constrained visual BA and carefully staged solves | Close long-range drift without breaking gravity/metric scale | Fixed bad priors, unobservable scale freedoms, poor loop weighting | Loop residuals reduce, local VI consistency remains, no route deformation |
| High: batch convergence | Checkpoint the same SIFT graph; improve preconditioning/window hierarchy; study meaningful convergence before huge budgets | Current three batch runs are not converged | More iterations optimize the wrong noise model or overfit bad matches | Cost components and held-out geometry improve; final CP score/metric error improve |
| Medium: observation weighting | Calibrate residual covariance by feature type, radius, blur, stereo geometry, and track correlations | Prevent marginal peripheral/dynamic evidence dominating BA | Hand-tuned weights may fit one sequence or suppress valuable edges | Residual calibration on independent windows, repeatability, better uncertainty |
| Medium: intrinsics observability | Release f/principal, low radial, higher radial, tangential/prism in staged blocks with priors and singular-value checks | Avoid weakly observed parameters absorbing pose/scale errors | Too-tight priors freeze real bias; too-loose terms mimic depth/extrinsic errors | Held-out directional/radial errors, stable parameter posterior, no central/peripheral tradeoff |
| Medium: rig/IMU extrinsics | Small manifold perturbations of cam0-cam1 and body-camera transforms, strong factory priors, excitation gates | Correct small real assembly/calibration offsets | Extrinsic/scale/bias/timing coupling under weak motion | Cross-camera reprojection and inertial residual gain on unseen motion; metric baseline preserved within justified uncertainty |
| Medium: temporal calibration | Audit residual camera/gyro/accelerometer offset, jitter, and exposure timestamp convention before adding time-offset states | Timing errors can mimic geometric calibration error during motion | Unobservable delay or double correction; hardware model mismatch | Motion-dependent residual signature and identifiable optimum on independent data |
| Medium: structural cues | Lines, vanishing directions, planes, and Manhattan/ground constraints only when supported | Stabilize texture-poor doorways, distant facades, bridges | Curved scenes or false semantic priors bend the reconstruction | Correct reject behavior and measurable benefit on weak-point intervals |
| Medium: scene dynamics | Hand/person masks, multi-view motion segmentation, static-confidence scoring | Avoid persistent but nonstatic tracks misleading the estimator | Masks remove too much of the two narrow useful views | More static inliers and fewer bad jumps, with coverage maintained under occlusion |
| Research: learned dense tracking | Multi-frame learned point tracking, dense correspondence or recurrent correspondence modules supplying native-ray factors | Survive blur and larger viewpoint change; longer tracks than pairwise descriptors | Correlated prediction errors, severe fisheye appearance shift, expensive inference | Geometry-verified independent tracks, stable uncertainty, real replay speed and score |
| Research: geometry priors | Confidence-calibrated multi-view foundation-model geometry fused as a temporary proposal, not unquestioned metric truth | Rescue weak startup/relocalization using broad visual context | Scale drift, domain shift, leakage, overconfident hallucination | Metric consistency against stereo/IMU, held-out generalization, safe fallback |
| Research: learned uncertainty and factor selection | Learn which tracks/factors improve geometric information under a fixed optimization budget | Spend compute on informative stable constraints | Dataset-specific shortcut learning or opaque failure | Better score/coverage per latency on held-out sequences with interpretable diagnostics |
| Research: continuous-time VI | Spline/continuous-time trajectory with native sensor timing where motion/exposure warrants it | Model asynchronous/within-exposure motion more faithfully | Unnecessary state complexity for this sensor; new gauge/covariance problems | Timing/exposure evidence first, then controlled improvement versus discrete-time baseline |
| Final: combined system | Combine only individually useful changes; benchmark full categories with frozen development/test split | Convert one strong Short result into a robust category result | Interactions erase isolated gains; tuning against evaluation GT | Full valid benchmark protocol, per-sequence failures, runtime, coverage, reproducible submissions |

The “research” ideas above are candidate directions, not claims that a particular current model is state of the art or already works on native Aria imagery. Verify primary documentation, licenses, inference assumptions, and current evidence when selecting a concrete model.

## 6. What to record after every future experiment

Use a new immutable output directory. Record: hypothesis; exact parent/source/binary/config/input hashes; sequence and temporal scope; changed variables; feature graph and calibration; exit status; solver convergence; runtime/memory; map epochs and selected-map policy; source/pose/GT/CP denominators; official Score2D; PoseRecall; metric SE3 and labelled Sim3 diagnostics; local tracking/recovery evidence; confounds; verdict; next decision.

Always provide the user a short completed-experiment summary: **what changed, positive/negative/mixed result, score and coverage, remaining failure, plain absolute visualization paths, and what a healthy result should look like.** Stop for discussion at major implementations, errors, or crossroads. Do not launch an expanding family of experiments before explaining the previous major result.

Never overwrite the 80.20 run or call a partial Rerun complete. Never turn missing output into interpolated poses to improve coverage. Never use GT to initialize, calibrate, pick matches, join maps, or choose a live pose/association hypothesis inside the estimator, except in explicitly labelled diagnostic-oracle tests that are excluded from benchmark claims. Development-set GT remains appropriate for post-run scoring, diagnostic comparison, and deciding which research direction to investigate; preserve a separate held-out evaluation set for final generalization claims.
