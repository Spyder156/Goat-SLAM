# Project goal: beat Meta's Aria SLAM on LaMAria

Last reconciled: **2026-10-03**. Project: `/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3`.

## Read this first if you have lost the thread

**Build a reliable, metrically accurate, native dual-fisheye visual-inertial SLAM system that beats Meta's Aria SLAM under the actual LaMAria benchmark protocol.** We want complete trajectories and coherent maps across Short, Medium and Long recordings, followed by broader challenge coverage. A system that occasionally produces an excellent fragment is insufficient. A pretty trajectory after alignment is insufficient. More matched dots, lower reprojection loss, a successful build, or another completed experiment is not the goal.

The current strongest demonstrated result is **80.199161 Score2D on the full Short training sequence `sequence_1_19`**, from an online replay with periodic VI optimization and bounded refinement of both cameras' intrinsics. The fixed-intrinsics reference on that same sequence scored **67.188379**. This is a meaningful improvement, but **we have not beaten Meta on the leaderboard**, and we have not yet shown that this configuration is robust across sequences or repeated runs.

If unsure what to do next, ask: **Will this action help us explain, reproduce, stabilize or generalize the 80.20 result, fix metric initialization/scale, preserve landmarks through difficult sections, or remove accumulated drift? What completed experiment would prove it?** If the answer is unclear, stop expanding the task and return to the roadmap.

Read these files together:

- `GOAL.md`: purpose, current position, lessons and decision rules.
- `ROADMAP.md`: technical hypotheses, implementation options and ordered experiments by subsystem.
- `EXPERIMENTS.md`: historical results, failures, evidence paths and follow-up ideas. A missing score stays missing; it is never inferred from a screenshot.

## What beating Meta actually means

The target is a defensible result on the **same benchmark split, challenge, sensor configuration and aggregation**, with broad coverage. On 2026-10-03, the official leaderboard lists the reproduced Aria SLAM category averages below. These are reference targets, not numbers directly comparable to our single training-sequence runs. [Official LaMAria leaderboard](https://lamaria.ethz.ch/leaderboard).

| Challenge | Published Aria SLAM Score2D reference |
|---|---:|
| Short | 90.7 |
| Medium | 78.5 |
| Long | 70.9 |
| Low light | 84.2 |
| Moving platform | 55.0 |

The earlier conversational shorthand “Meta has about 70” referred to the Long-level reference, not a universal overall score. Our **80.20 on one Short training recording does not outrank 70.9 on the Long test challenge**, nor establish parity with the Short average of 90.7. Recheck published references before making a future comparison; store its date and scope.

Our eventual deliverable is a reproducible system and benchmark result, supported by saved raw estimates, calibration histories, configuration/build provenance, coverage accounting and inspectable visualizations. Keep the causal online SLAM result separate from any offline refinement result. Both are useful research directions, but they answer different questions and have different latency and compute costs.

## Where we started

The initial system was Raghav's ORB-SLAM3 fork running LaMAria through a new native **Fisheye624** model and a dual-camera rig. The user repeatedly observed a mismatch between apparently plausible image matches and poor 3D map/trajectory behavior. The central request was to inspect the estimator's geometry and associations end to end: quaternion layout, transform direction, camera/body/IMU axes, native image orientation, ray normalization, rig baseline, map units and projection consumers.

Early runs suffered from crashes, map fragmentation, repeated initialization, low map-point support and exports that represented only a surviving map segment. The conversation records hundreds of resets and cases where frame-to-frame matches looked good while local-map associations collapsed. These observations motivated geometry and association audits, rather than treating the Rerun renderer as the assumed source of the problem.

The development history includes ray/triangulation convention corrections, pooled-camera index and ownership repairs, inertial-prior lifecycle work, temporal association/recovery changes, map/history export repairs and more careful metric/calibration handling. Their exact implementation and test status belong in `EXPERIMENTS.md`; this file does not assign a numerical score gain to each individual fix.

A major early milestone was a continuous Long run on **`sequence_3_17`** with **49.881534 Score2D**, one original map, zero resets, and 35,772 poses out of 35,842 inputs. That established useful full-route behavior; it did not establish accurate metric scale or solve Short/Medium reliability. The same historical pipeline scored **39.6567 on Short** and **55.7524 on Medium**, with fragmentation in both. These are different recordings and must not be treated as one numerical progression.

## The measured position now

All values below are **local evaluations**, not public leaderboard submissions. The detailed ledger records exact precision, source files and coverage.

| Recording and experiment | Score2D | What this establishes |
|---|---:|---|
| Long `sequence_3_17`, history/continuity milestone | 49.8815 | Full-route single-map baseline; scale-1 error still material. |
| Short `sequence_1_19`, historical history profile | 39.6567 | Fragmented; largest map did not cover the whole route. |
| Medium `sequence_2_11`, historical history profile | 55.7524 | Fragmented; useful loop-closure development recording. |
| Short, fixed-intrinsics reference for latest experiments | 67.1884 | Full-route, one-map reference for the following comparisons. |
| Same Short reference, offline SIFT visual rig BA | 64.7057 | Negative score outcome despite reduced visual fitting cost. |
| Same Short reference, offline SIFT global VI-BA, fixed intrinsics | 67.1942 | Essentially neutral; metric scale remained wrong. |
| Same Short reference, offline SIFT global VI-BA, all native intrinsics adjustable | 67.1833 | Essentially neutral; lower reprojection error did not improve the score. |
| Short, first full online calibration candidate | 48.0682 | Negative; calibration validation rejected proposals and trajectory fragmented. |
| Short, online full calibration v2 | **80.1992** | Best result in the latest requested experiment family; full GT coverage, one map. |
| Short, later online rollback variant v3 | 42.5933 | Negative: three maps. A locally correct fix did not yield robust continuity. |
| Medium, frozen online full calibration v2 transfer | 62.7648 | Three independent maps; 13/18 CPs recovered and 2499/4083 GT associations in the scored map. |
| Long, frozen v2 with native VRS lens input | 34.4589 | Five independent maps; 15/27 CPs recovered and 3163/6118 GT associations. Worse than the historical Long baseline. |
| Long, BabyFeatures SOS | 43.7293 | One retained map after startup; 27/27 CPs recovered, four within 1 m. Better continuity than native v2; below historical Long accuracy. |

The three offline arms used one common graph with **1,715,595 initial landmarks and 26,564,121 observations**, re-extracted and matched using SIFT. They started independently from the same source trajectory. Each completed its 30-attempt budget without declaring formal convergence. Those results say that **these tested refinements did not improve the score**; they do not prove global BA can never help. The offline and online experiments differ in their visual graph, execution history and optimization schedule, so their comparison does not isolate a single causal variable.

### Latest BabyFeatures result: continuity improved, accuracy still limited

The user's SOS proposal is implemented on `experiments/BabyFeats` and tested
once on full Long. Passive short tracks enter estimation only after normal
tracking fails. **All 15 SOS episodes returned to normal map tracking**, and
one retained map covers **99.38% of native inputs** after a startup reset.
Score2D is **43.729292**, above native v2 34.458901 but below historical Long
49.881534. All 27 CPs are recovered; only four are within 1 m. A 1.619 m final
pose jump remains. The next discussion must separate the demonstrated map
survival from remaining accuracy and state-correction problems. This single
replay does not establish repeatability or transfer to Medium/Short. Frozen
baselines remain untouched. See E39 and `docs/BABYFEATURES_LONG_20261003.md`.

### Earlier continuity batch: no candidate promoted

Three separate candidates were implemented and run fully on Medium/Long:
direct recovery, accepted-landmark memory, and fixed-intrinsic periodic VI.
None surpassed the strongest retained implementation on its sequence. The
best new Long score was memory's **42.904563**, above native-v2 34.458901 but
below historical Long **49.881534**; memory regressed Medium to **45.874750**.
Direct-recovery and fixed-VI Medium outputs were unscorable after fragmentation.
An unchanged Medium control scored **63.086620**, versus saved 62.764799, and
repeated the first map break within 0.3 s. That small gain is run variation,
not an algorithmic improvement. Frozen Short **80.199161** is unchanged.

The concrete new finding is a recovery acceptance gap: a >=50-inlier visual
recovery can lose most support during local VI and still commit through the
>10-inlier recent-loss path. Fix the explicit recovery confirmation/inertial
handoff without assuming which pose is correct or discarding valid IMU history.
Do not promote extra matches or lower optimizer cost as evidence of reliable
SLAM. Read E37–E38 in `EXPERIMENTS.md` and `docs/RECOVERY_HANDOFF_20261003.md`.

### What is real about the 80.20 result

- Full Short recording, approximately **917.5 seconds**; not a favorable time crop.
- **18,343 saved poses / 18,352 native input timestamps**, with eight startup poses and one unsupported tail pose absent. Missing poses were not invented.
- One map and **2,999 / 2,999 dense GT timestamp associations**. Full GT coverage is not identical to complete native-frame coverage.
- **14 / 14 control points within 1 m**, compared with 10 / 14 in the 67.19 reference.
- Pose recall at 1 m: **99.7666%**; at 5 m: **100%**.
- Metric 3D RMSE after a rigid, scale-preserving SE3 diagnostic fit: **16.53 m → 2.48 m** versus the 67.19 reference.
- Evaluation scale multiplier: **0.886115 → 1.019443**. Much closer to metric scale, but not an assertion of perfect scale everywhere.
- The estimator used only observations available up to its current time for periodic optimization. GT and surveyed control points did not enter SLAM.
- Its missing **19.800839 score points** are all residual control-point error, not missing CP coverage. Recovered CP errors have median **39.34 cm**, range **10.71–80.34 cm**; full credit requires **5 cm or less**. Whole-route overlap cannot resolve errors at this scale.

### What is still unresolved

- v2 contains brief pose jumps around native timestamp 550 s. Preserve this limitation alongside the good score.
- The gain is **not yet attributable exclusively to camera calibration**. Periodic VI optimization, calibration and associated state handling changed together. The fixed-intrinsic C control has now run, but accepted no periodic update and uses a different proposal comparator; a matched accepted-update comparison is still needed.
- v3's corrected rollback contract passed its checks, but the full run fragmented and scored 42.59. Its calibration/keyframe schedule differed before the first failure, so it is not a perfectly deterministic one-variable ablation.
- The first full Medium/Long transfers have now completed and **failed to preserve continuity**: scores 62.76/34.46 with three/five independent maps. Medium loses 27.78 points to missing CPs; Long loses 44.44. In v2, periodic full calibration only runs while one populated map exists, so splitting also prevents further calibration trials. Stabilize this behavior before promoting the candidate across the suite; repeatability and wider benchmark generalization remain unproved.
- Long's inherited lens input was a prior COLMAP fit with separate focal lengths, not factory-native calibration. That attempt failed the single-focal constructor contract. The completed transfer restored the sequence's native VRS lens parameters and retained its rig/IMU transforms; comparison against the old Long baseline therefore changes both method and starting lens calibration. See E36 and the input audit in `EXPERIMENTS.md`.
- Adjusting all camera intrinsics is not the same as calibrating rig extrinsics, camera-to-IMU extrinsics, time offsets or every IMU parameter. Those extrinsics remained fixed in the latest online and offline experiments.
- The final offline calibrated solve/export/score is complete. Its Rerun was stopped at the user's request and is **partial and unverified**. Do not advertise that recording as finished.

## The reasoning that should guide the project

### 1. Correct geometry before compensating thresholds

The user's core suspicion was that 2D success can hide a 3D convention or calibration failure. That is a valid diagnostic direction: paired image matches, camera projection/unprojection round trips, and low central-image errors do not establish metric multi-view consistency. A matching function can consistently use the wrong convention in both directions. A strong test must exercise the complete observation-to-bearing-to-rig-to-world-to-projection chain, both lenses and the image periphery.

Use the production SLAM geometry in tests. Keep camera-to-world versus world-to-camera, xyzw versus wxyz, camera versus IMU frame, scale units, image rotations and principal-point conventions explicit. Do not “fix” physical geometry through a display rotation or post-hoc trajectory alignment.

### 2. Initialization, scale and persistence are coupled

The trajectory often looked angularly correct but stretched. Metric initialization must make visual structure, rig translation, camera motion, IMU velocity and gravity mutually consistent. A known stereo baseline only helps when usable cross-camera geometry actually constrains the estimate. A depth network can offer structure or a prior; metric scale must be established and tested, not assumed from its name.

After initialization, good short-lived optical flow is not enough. Landmarks must remain associated as viewing angle, distance and image radius change. Low green counts need a stage-by-stage explanation: detected features, propagated tracks, projected map candidates, matched associations, optimization inliers and surviving landmarks. More green dots alone is not the objective.

### 3. Calibration is a serious hypothesis, not a foregone conclusion

The user specifically proposed that incorrect intrinsics make a feature lose its landmark as it moves toward the fisheye periphery. Test that across the full valid lens domain, with held-out observations and error stratified by ray angle, not only near the image center. Include focal length. Permit bounded, regularized updates, invalidate dependent caches correctly and make calibration/state commits atomic.

The 80.20 result makes the combined online approach worth pursuing. The neutral offline calibrated result means “free more parameters and reduce reprojection error” is not sufficient evidence of better geometry. Blur, occlusion, dynamic objects, association ownership, uncertainty, observability and state-update consistency remain alternative or interacting causes.

### 4. Refinement and loop closure must preserve a coherent metric solution

Global visual and VI refinement are worth investigating, but success is measured by coverage, score, metric diagnostics and stability. Visual optimization should retain physical rig constraints when claiming metric output. VI optimization must retain visual loop constraints strongly enough to correct drift without silently corrupting bias, gravity or scale.

Rebuild retrieval and loop verification using the available MegaLoc work in Mecka Basalt, then integrate only geometrically verified constraints. `sequence_2_11` is the planned looped test recording. A place-recognition hit is a proposal, not an accepted loop; a closed-looking top view does not prove correct metric closure.

### 5. Short accuracy deserves its own investigation

Short recordings should expose calibration, initialization, bias and refinement limits without needing kilometer-scale loop closure to explain every error. Aim for high accuracy throughout the route, not merely recovered control points. Use per-control-point score contributions to explain the score, and metric/relative trajectory diagnostics to explain the motion. Do not assume that success on Long implies Short is solved, or that Short's remaining gap has one universal “secret.”

## Evaluation and visualization contract

The estimator outputs raw world-from-body/IMU poses in its own map gauge. The official evaluator obtains a control-point-derived Sim3 afterward. The evaluation Rerun reuses that **same saved transform** for trajectory, landmarks and rig display. It does not silently fit a second transform to dense GT. A constant scale error can be absorbed by this alignment; time-varying scale and shape errors remain. Physical metric diagnostics must therefore accompany the aligned score.

**Score2D** averages the official piecewise scores of horizontal control-point errors; absent control points contribute zero. **PoseRecall** uses the full dense GT denominator, so missing GT timestamps count as failures there. Dense trajectory RMS is a separate diagnostic, not the Score2D formula. The native-frame pose count is another distinct coverage measure. The official functions and local wrapper are the executable source of truth:

```text
/home/raghav/workspace/MeckaAI/third_party/lamaria_toolkit/lamaria/utils/metrics.py
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/pipeline/run/score_lamaria.py
```

The website describes submission and evaluation requirements; keep the actual upload convention consistent with the sensor used. [Official evaluation documentation](https://lamaria.ethz.ch/slam_documentation).

Every major experiment should end with a concise handoff: hypothesis, change, dataset/coverage, positive/neutral/negative outcome, official score, metric scale/error, unresolved confounds, exact visualization paths and what the user should inspect. For SLAM recordings, green represents a retained association, not permanent correctness. For offline BA recordings, green represents the final batch map's associations, not historical online inliers. Final optimized point reveal is not a recording of every historical BA movement.

## Immediate priorities, in order

1. Preserve the 67.19 reference, 80.20 candidate and all failures with their actual build/config/input provenance. Do not overwrite the best run or promote a newer candidate solely because its code looks cleaner.
2. Make the 80.20 configuration reproducible and understand its discontinuities. Use a matched recurring-VI/frozen-intrinsics control to isolate calibration's contribution, with the same schedule and instrumentation where possible.
3. Stabilize initialization/metric scale and online state transitions without weakening geometric acceptance merely to increase counts. Validate on complete Short and Medium, then Long; report all outcomes.
4. Investigate landmark survival and difficult transitions using native peripheral geometry and calibrated uncertainty. Use the user's visual inspection to prioritize mechanisms, not to substitute for measurements.
5. Add verified MegaLoc loop closure and metric-consistent optimization on the looped Medium sequence, then evaluate broader transfer.
6. Expand to held-out training sequences, repeat-run variability and final challenge-wide evaluation. Pursue advanced frontend/backend/depth ideas when a concrete measured limitation justifies them.

The user explicitly requested the full Medium/Long transfers on 2026-10-03; both replays and scores are complete. Their artifacts and recording verification status belong in `EXPERIMENTS.md`. The priorities above remain proposed next work, not an instruction to start an unlimited experiment cascade.

## Working rules that prevent distraction

- **Advance the system, not the experiment count.** A focused production fix with a meaningful full-sequence test is preferable to a sequence of tiny probes that never addresses the user's hypothesis.
- Small tests are valuable for exact contracts, crash isolation and falsifying a mechanism. State what they cannot establish and move promptly to the relevant end-to-end test.
- Use the existing reusable Short/Medium/Long fixtures. Reuse inputs and feature caches only when compatible with the hypothesis and record the hashes.
- Do not loosen matching, triangulation or outlier gates merely to manufacture green features. If uncertainty warrants a changed gate, justify it statistically and measure false associations and accuracy.
- Do not introduce multiple unrelated estimator changes before measuring their contribution. A successful combined configuration can be preserved while its components are isolated.
- Never use GT poses, surveyed CP coordinates or evaluation alignments as hidden estimator constraints. Label oracle diagnostics explicitly and exclude them from performance claims.
- Do not join independent map origins without estimated geometric constraints, hide missing timestamps, overwrite logs, or present an unverified partial Rerun as complete.
- No unlimited experiment cascade. At a major implementation, failure, result or design crossroads, **stop, summarize, provide absolute paths, and discuss with the user**.
- The user wants absolute filesystem paths, not clickable local links. Give the exact Rerun name, a short result table and what a good visualization should show.
- Upstream `third_party/` is read-only by repository policy. Maintain changes in `patches/`, project code and isolated builds.
- A bug fix can be correct yet reduce a benchmark result by exposing another weakness. Preserve both correctness evidence and the actual negative system outcome.

## Start here: evidence and visual inspection

Best completed online Rerun, **80.20**, exact evaluator alignment:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/online_full_evaluation.rrd
```

Baseline **67.19**, exact evaluator alignment:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/baseline_evaluation.rrd
```

Calibration history and held-out peripheral diagnostics for the online candidate:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/diagnostics/report.html
```

Metric versus evaluated geometry and per-control-point scoring for the baseline and online candidate:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_global_refinement_20261003/comparison_online_v2/report.html
```

Authoritative visualization completion index, including the stopped calibration recording:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/VISUALIZATIONS.txt
```

Historical continuity evidence and reusable suite:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/docs/LAMARIA_CONTINUITY.txt
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_suite_20261002/suite.json
```

In an excellent result, both cameras contribute stable static-scene landmarks through large changes of viewpoint; the metric trajectory has the correct scale before evaluation; the evaluated map and trajectory agree with surveyed geometry; difficult sections do not create jumps or disconnected maps; and improvements repeat across recordings. **That system—not an isolated attractive score—is what we are building.**
