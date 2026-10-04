# Goat-SLAM handoff

Status: **2026-10-04**. Current local branch: **`experiments/BabyFeats`**.
**Session stopped at the user's usage limit.** Latest task is documentation and
handoff only; no experiments or render jobs remain running. Start with
`docs/RESUME_20261004.md` for exact commands, artifacts and the user's latest
missing-start / terminal-drift / apparent-tilt observations. The startup gap is
confirmed (223 frames, about 11.15 s); tilt and terminal-drift causes remain
unverified. `docs/NEXT_AGENT_PROMPT.txt` is the copy-paste session prompt. Discuss
the saved evidence before resuming implementation; do not automatically rerun.

Subsequent discussion: `IDEA_POOL.md` I13–I16 records the useful parts of a friend's
review of the old continuity arms, with mathematical and causal caveats. No code
or experiment changed: recovery-window confirmation, Medium777 s offline audit,
honest VI/calibration validation and bounded memory remain proposals.

**Latest completed work:** the requested full Baby Long Rerun now exists and
passed verification, including two additional reconstructed Baby-track panels
and logged SOS map colours. One authorized offline fixed-camera native-graph
global VI refinement scored **45.027586 versus 43.729292**, with identical one-map
coverage (35,619/35,842) and 27/27 CPs. It reported a cost-plateau convergence;
the 1.62 m jump remains. No new SLAM replay or calibration sweep was run. Read
`docs/BABYFEATURES_NATIVE_VI_20261004.md` / E40 / S25 for commands, limitations
and visual paths. No work is pending; pause for visual discussion before another
implementation family. The 43.73 Rerun is not relabelled as the 45.03 candidate.

The authorized BabyFeatures SOS implementation and one full Long replay are
complete: **43.729292 Score2D, one retained map, 99.38% native pose coverage,
15/15 SOS episodes returning to normal tracking**. One startup reset remains;
the retained map never splits. All 27 CPs were recovered, but only four are
within 1 m and a 1.619 m final trajectory step remains. This is a continuity
improvement, not a new accuracy baseline. Historical Long 49.881534 and frozen
Short 80.199161 are preserved. No replay is pending. Read
`docs/BABYFEATURES_LONG_20261003.md` and E39 in `EXPERIMENTS.md`, then discuss
the result before launching more experiments. Knot refinement remains deferred.
The earlier six-arm continuity batch and unchanged Medium control remain in
`docs/CONTINUITY_EXPERIMENTS_20261003.md` / `docs/CONTINUITY_RESULTS_20261003.json`.

## Start here

Build reliable, complete, metrically accurate dual-fisheye visual-inertial SLAM that beats Meta/Aria under the actual LaMAria protocol. Preserve the strong retained geometry while fixing fragmented trajectories. A good-looking partial map is not success.

**Implementation authorization, 2026-10-03, completed 2026-10-04:** implement BabyFeatures
and run one full Long experiment on `experiments/BabyFeats`. Good short-lived
correspondences are **tracked passively throughout**, but enter pose estimation
**only after normal map tracking fails**. The implementation supports temporary
VI constraints, guarded same-map landmark promotion, and verified ordinary
tracking return. All 72 accepted SOS poses reached the final scored export;
13 promotions added 1,648 landmarks. Fourteen native solver checks passed.
Read `IDEA_POOL.md` for the concept and the report for the bounded implementation.
This supersedes the older discussion-only and no-background-tracking instructions.
The experimental patch/build profile is `orbslam3_lamaria_babyfeats_20261003`;
use its custom `build_incremental.py`, not the generic historical builder.

Read `GOAL.md` for the objective and decision rules, `EXPERIMENTS.md` for verified outcomes and exact evidence, and `ROADMAP.md` for hypotheses and proposed work. `README.md` describes the layout; `docs/ARTIFACT_STORAGE.md` explains storage isolation; `docs/LAMARIA_CONTINUITY.txt` describes the historical workflow. **Default launch/build profiles still select the historical continuity baseline, not the best Short v2 candidate. Choose the intended profile explicitly.**

## What currently works

These are local official-function evaluations, not leaderboard submissions or category averages. Map counts include retained independent maps; GT coverage below belongs to the largest map selected without GT.

| Run | Score2D | CP recovered | GT associations | Maps | Assessment |
|---|---:|---:|---:|---:|---|
| Short `sequence_1_19`, online full v2 | **80.199161** | 14/14 | 2999/2999 | 1 | Best Short candidate; brief jumps remain |
| Medium `sequence_2_11`, frozen v2 transfer | **62.764799** | 13/18 | 2499/4083 | 3 | Accurate retained portion; incomplete global route |
| Long `sequence_3_17`, frozen v2/native input | **34.458901** | 15/27 | 3163/6118 | 5 | Negative transfer: fragmentation |
| Long, earlier history/continuity baseline | **49.881534** | 27/27 | 6113/6118 | 1 | Preserve this full-route reference |
| Medium, unchanged v2 repeat | **63.086620** | 13/18 | 2497/4083 | 4 | Same implementation; +0.322 is not an algorithmic gain |
| Long, BabyFeatures SOS | **43.729292** | 27/27 | 6078/6118 | 1 | 99.38% native coverage; one startup reset, no later split |
| Same Baby Long, offline fixed-camera native VI | **45.027586** | 27/27 | 6078/6118 | 1 | Identical timestamp coverage; modest improvement, jump remains |

BabyFeatures has not been evaluated on Short or Medium. Its completed four-panel
Rerun shows the original **43.73** result; no new **45.03** Rerun was generated.

The best Short method is **online SLAM plus periodic causal VI-BA and bounded intrinsic calibration**, using native Fisheye624, both cameras, calibrated IMU, and cached ALIKED-derived features. All 15 independent native intrinsic variables per camera, including focal length, can move with factory priors, step bounds and held-out checks. Physical rig and camera/IMU extrinsics remain fixed. Eligible full-calibration trials run every 60 native seconds; Short accepted 2/15 trials. This is not online calibration of extrinsics or time offsets.

Short exports 18,343/18,352 native poses across approximately 917.5 seconds; missing startup/tail poses remain absent. All CPs are recovered, but horizontal errors of 10.71–80.34 cm account for the remaining 19.80 score points. Full credit requires at most 5 cm. Its evaluation scale is 1.019443; rigid, scale-preserving 3D RMSE is 2.4818 m. Brief roughly 2.7–3 m spikes near native 550 s are unresolved. One good run does not establish repeatability or isolate calibration's contribution from recurring VI optimization.

Long's original inherited input had a prior two-focal COLMAP calibration incompatible with v2's native single-focal vertex. The completed transfer restored exact native VRS lens values and preserved rig/IMU transforms. Its comparison with the old Long baseline therefore changes both method and initial intrinsics; do not attribute the regression to one variable.

## Preserve the exact candidate

Project-relative implementation and workflow:

- `patches/orbslam3_lamaria_online_full_v2/`: best Short implementation, including baseline hashes and patch.
- `build/orbslam3_lamaria_online_full_v2/`: existing private build and `build_manifest.json`; local generated artifacts, not a portable source distribution.
- `pipeline/run/build_lamaria.py`, `run_lamaria.py`, `lamaria_suite.py`: build, launch and reusable fixtures.
- `configs/orbslam3_lamaria/online_full_v2_transfer_20261003/`: verified Medium/Long transfer settings and input audits.
- `pipeline/run/score_lamaria.py` and `pipeline/viz/`: scoring and evaluation-aligned visualization.

Frozen v2 runner SHA256:
`a0efe826e73155b3f307882cb5176ec630eca9bd2147cba04cbcd395eec93f66`

Frozen v2 library SHA256:
`7664755942310a1513cde99aeb47d3c48c4cf17a5412196b970b6315018c3414`

Patch SHA256 recorded in the build manifest:
`bd9f9c5eb0f664b08f322e928c4e7579b0624e84b901f8da2f2620df2e4aba2d`

Preserve each run's `command.json`, `config/settings.yaml`, `config/build_manifest.json`, logs, coverage and `lamaria_score/scores.json`. Do not overwrite the successful v2 build. The newer rollback v3 scored **42.593277 with three maps**; a correctness test passed, but the system regressed. Do not promote it just because it is newer.

Existing build tooling checks the recorded vendor baseline and uses an existing CMake baseline and local Docker image. A fresh public checkout also needs its documented third-party/build prerequisites, dataset, feature caches and environment. Do not imply that saved machine-local build paths make a clean clone immediately runnable.

## Completed continuity batch and next decision

1. **Three implementations were tested, none promoted.** Direct recovery: unscorable Medium / 38.941196 Long. Accepted-landmark memory: 45.874750 Medium / 42.904563 Long. Periodic fixed-intrinsic VI: unscorable Medium / 37.630917 Long. All full replays exited normally. Unscorable means the largest map could not support the official CP alignment, not a fabricated zero or a crash. No map was selected with GT or stitched to salvage a score.
2. **The unchanged Medium reference reproduces broadly.** Byte-identical library, runner, settings and timestamps produced 63.086620 versus saved 62.764799. The first retained-map split was 777.387 s versus 777.687 s. Internal calibration/recovery outcomes and retained-map count varied; one repeat does not estimate variance or identify its runtime cause. The latest control is the best observed Medium artifact, not a new algorithm.
3. **Recovery confirmation is the concrete next target.** Coast relatching skips normal relocalization bookkeeping. It can recover >=50 visual inliers, then commit a local VI pose supported by only 11–21 observations through the inherited >10 RECENTLY_LOST gate. Medium committed 5.49–8.52 m disagreements between the two estimates. This demonstrates an acceptance-policy gap, not which estimate is GT-correct. Existing VI already jointly optimizes current pose/velocity/bias; valid temporal anchors are not intrinsically wrong. Design an explicit recovery confirmation/window, retained geometric support and consistent inertial state before publishing a new prior. Merely raising 10 to 15 does not reject the Medium examples. Exact source/log evidence: `docs/RECOVERY_HANDOFF_20261003.md`.
4. **Additional matching alone did not solve continuity.** Direct matching generated some geometrically verified recoveries, but most candidates still lacked enough unambiguous matches. Memory supplied thousands of accepted associations; Long coverage improved versus native v2, but Medium regressed and historical Long remained stronger. A/B share the rollback repair, so these are not single-line causal ablations. Zero BoW recovery is not universal: the unchanged repeat accepted 16 visual hypotheses while the retained Medium accepted none; those counts do not certify VI handoff success.
5. **Map-local fixed VI is implemented experimentally, but accepted no periodic update.** It continued scheduling after fragmentation (18 Medium / 25 Long proposals), all rejected. C validates against the live pre-solve map, while v2 free calibration validates against its optimized fixed-camera branch. The latter allows a logical live-state validation gap, but old logs lack the initial held-out medians needed to establish an actual harmful commit. Do not claim accepted fixed-VI updates caused these scores.
6. **Loop retrieval and knots remain future work.** No loop/merge detection markers appeared in these batch logs. MegaLoc/CLoSeR integration is still proposed. The earlier motion-only knot cue overlapped 55/59 CP visits in an exploratory audit, not held-out validation; no CP coordinates/timing or zero-velocity assumptions entered this batch. Keep knot-based refinement deferred until the reliability decision is resolved.

Offline SIFT visual rig BA on the 67.188379 Short reference scored **64.705686**; full global VI-BA scored **67.194208** with fixed intrinsics and **67.183330** with adjustable intrinsics. These tested refinements were negative or neutral, not a demonstrated alternative to v2. The calibrated offline solve/export/score finished, but its Rerun was intentionally stopped and remains partial.

## Round of 2026-10-04: mechanism arms after the decomposition

See `docs/BATCH_20261004.md` for the full arm table. Established: the Medium 860-890 s split and the
end-of-route scale creep are inertial-state failures (a rejected visual update leaves a bad velocity;
biases absorb scale under weak vision). Rejected-update rollback gave Medium 78.59 with one map; IMU
bias random walks /10 gave Long 49.19-50.04 and Short 81.39. Negative, with the reason recorded:
gyro-only pre-init coasting, coast-age-widened search, dense ALIKED caches, a soft pedestrian speed
prior inside pose-only optimisation. New tooling: `pipeline/run/batch_runner.py`,
`pipeline/eval/experiment_table.py`, `pipeline/eval/decompose_lamaria_errors.py`,
`pipeline/datasets/extract_aliked_dense.py`. Frozen builds, baselines and the 80.20/62.76/49.88 artifacts
are untouched; new builds live under `build/orbslam3_lamaria_*_20261004` with their `patches/` packages.

## Evaluation and working contracts

- Score2D averages official horizontal CP-error credits; missing CPs receive zero. PoseRecall includes every dense-GT timestamp in its denominator. Neither is the native-frame coverage count or dense ATE.
- The evaluator, after SLAM, obtains a CP-derived Sim3. Evaluation Reruns use that exact saved transform for poses, points and cameras; they do not fit a second dense-GT transform. Scale-one metric diagnostics must remain separate.
- Largest-map evaluation excludes other independent origins. Never join maps without estimated geometric constraints, invent missing poses, or describe full video processing as a complete global trajectory. Medium/Long missing CPs cost 27.78/44.44 points respectively.
- No GT poses, surveyed CP coordinates, annotation timing or evaluation alignment enters the estimator. Label oracle diagnostics clearly.
- Keep upstream `third_party/` read-only. Implement changes in `patches/`, project-owned code and private builds. Preserve provenance and negative results.
- At each major result or design crossroads, stop and discuss: change, test, positive/neutral/negative outcome, limitations, exact visualization paths and what a perfect result should show. The user wants structured explanations and **plain absolute paths, not local links**.
- No broad threshold loosening, unnecessary experiment cascades, automatic restarts of stopped work, or unrequested estimator changes during publication. Use short commit messages without assistant attribution.

## Local storage and visual evidence

Project source is independent of the original project:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3
```

Dedicated artifact root (`experiments/` resolves here):

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments
```

**Do not read or write active pipeline dependencies in the original `INSV_STITCHING` tree.** Owned artifacts were moved out; unrelated original files were preserved. Historical command/provenance strings retain old paths intentionally, resolved by current readers. Never repoint storage to the original project or rewrite archived hashes. Dataset, shared Python/Docker environments and the LaMAria toolkit remain external prerequisites; this isolation is not a claim of a fully hermetic environment.

Complete evaluated Reruns:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/online_full_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_medium_full_20261003/online_full_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_long_native_full_20261003/online_full_evaluation.rrd
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_long_20261003/evaluation_baby_20261004/run.rrd
```

Latest read-only diagnostics:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_breaks_discussion_20261003/breaks.json
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_cp_motion_audit_20261003/README.md
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_cp_motion_audit_20261003/net_progress_knots.png
```

These are local evidence paths, not a promise that datasets, binaries or multi-gigabyte Reruns are included in the public source repository. For the next implementation discussion, retain the frozen v2 result and the earlier continuous Long baseline, then address recovery before replacing the successful geometry wholesale.
