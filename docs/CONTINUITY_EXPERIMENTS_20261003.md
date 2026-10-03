# Medium and Long map continuity experiments

The user authorized three separate implementations and full Medium/Long
replays. The frozen Short 80.199161 implementation remains unchanged. This
batch addresses map fragmentation; motion-knot/control-point refinement is
deferred. Ground truth enters evaluation and diagnostic plots only.

After the three Medium candidates regressed, one additional **unchanged v2
Medium control replay** was added to assess repeatability. Its library,
runner, settings and input timestamps are byte-identical to the saved Medium
reference. It uses the same two-slot harness and no estimator modification.
This was an explicit follow-up control, not a predeclared fourth method or a
search for a favorable retry. No candidate replay was repeated.

## Controlled batch

| Arm | Change relative to frozen v2 | Expected benefit | Important limitation |
|---|---|---|---|
| A, direct recovery | Match cached binary ALIKED descriptors without ORB vocabulary partitions; confirm the provisional recovered pose with local-map tracking | Recover sufficient correct correspondences to return to the existing map | Candidate retrieval is still the existing recent/covisible selection; no new global place-recognition system |
| B, landmark memory | Retain accepted per-camera appearance descriptors for 0.5 s; resolve landmark IDs in the current map before projection | Avoid losing useful landmark appearance after one weak frame | Runs only below 80 incoming associations; unchanged geometry may still reject every candidate |
| C, periodic VI | Full-map VI with fixed intrinsics from BA2 onward, scheduled independently per map, including after fragmentation | Continue geometric refinement when shared-intrinsic calibration must remain disabled | Refinement cannot invent missing visual correspondences or join independent maps |

A and B also share a transaction repair: a visually recovered pose remains
provisional until local-map confirmation. Failed confirmation restores the
pre-relatch IMU pose, velocity and bias, discards the rejected marginal prior,
and does not restart the lost timer. C retains v2 tracking. These are bounded
engineering candidates, not single-line causal ablations.

All arms retain native Fisheye624, both camera streams, the existing calibrated
IMU and cached features. Descriptor, reprojection, geometric acceptance and
tracking-inlier gates are not loosened. The original initialization settings
are retained; C freezes cameras during its **periodic** solve, not during all
startup code. No missing poses are filled and no independently framed maps
are joined for scoring.

## Implementation checks completed

- **A: 20 production-library checks passed.** The actual matcher recovers
  80 landmarks per camera despite disjoint vocabulary nodes; native per-camera
  MLPnP and rig pose optimization accept consistent geometry and reject an
  incorrect correspondence geometry. Ownership, descriptor gates, bounded
  work and relatch rollback/timer behavior are checked. A full-size single
  candidate required about 65 ms on one CPU in the fixture.
- **B: 17 contract cases passed, with 23 assertions.** Production camera/grid/
  scale geometry, per-lens ownership, descriptor copies, age and map-epoch
  expiry, and actual map-point deletion are exercised. The cache owns no raw
  landmark or marginal-prior pointers. Visibility statistics are updated for
  newly considered points.
- **C: 12 actual-solver checks passed.** A metrically observable synthetic
  dual-camera VI graph checks accepted optimization, unchanged 32 stored
  intrinsic scalars, unchanged rig/camera caches/other map, prior invalidation,
  improved geometry, and atomic rejection of an invalid IMU graph. The first
  fixture exposed scale unobservability; it was corrected with stereo overlap
  and acceleration, without changing estimator gates.
- Frozen preservation check still verifies **576 SHA256 entries**.

These checks establish implementation behavior, not full-sequence quality.

## Reproduction and evidence

Experiment branch: `experiments/map-continuity-20261003`.
Configuration and input hashes:
`configs/orbslam3_lamaria/continuity_experiments_20261003/experiment.json`.
Each arm has its own `patches/orbslam3_lamaria_*_20261003` package and private
build. Existing baseline, vendor and third-party sources were not modified.

Run the requested arm with the LaMAria environment:

```sh
/home/raghav/miniconda3/envs/lamaria/bin/python pipeline/run/continuity_experiment.py --variant recovery_direct --sequence both
/home/raghav/miniconda3/envs/lamaria/bin/python pipeline/run/continuity_experiment.py --variant tracking_memory --sequence both
/home/raghav/miniconda3/envs/lamaria/bin/python pipeline/run/continuity_experiment.py --variant periodic_vi --sequence both
/home/raghav/miniconda3/envs/lamaria/bin/python pipeline/run/continuity_experiment.py --variant frozen_v2_repeat --sequence medium
```

Outputs must be fresh; the harness refuses to overwrite an existing case.
For a repeat, supply a new `--batch` directory. A shared filesystem semaphore
limits the batch to two replays at four CPU cores each. Each completed replay
is scored with the unchanged official local evaluator using the largest map
selected without GT. Every run retains its inputs, build manifest, logs and
score artifacts.

Batch artifact directory:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003
```

Generate the saved-score comparison with:

```sh
/home/raghav/miniconda3/envs/lamaria/bin/python pipeline/viz/compare_continuity_experiments.py
```

`comparison.json`, `comparison.png` and `trajectories.png` contain completed
scores and explicitly label incomplete cases. Each case also has
`diagnostics.png`: scored trajectory versus full GT, online inliers, map
epochs including discarded resets, and tracking/coasting state. Every map
plot uses the evaluator's saved CP Sim3. No additional fit hides a regression.

An ideal result shows the whole gray GT route covered by one estimated map,
no jumps during recovery, fewer losses, and improved CP accuracy. Extra
accepted associations, fewer maps or a lower training loss alone do not
establish success. Retained atlas-map count and online reset-epoch count are
different and are reported separately.

## Results

All six candidate replays and the unchanged Medium control completed normally.
No candidate beat the strongest recorded implementation on its sequence.
Frozen references:

| Frozen reference | Score2D | CP triangulated | GT associated | Retained maps |
|---|---:|---:|---:|---:|
| Medium v2 | 62.764799 | 13/18 | 2499/4083 | 3 |
| Long v2, native calibration | 34.458901 | 15/27 | 3163/6118 | 5 |
| Earlier Long, different initial calibration | 49.881534 | 27/27 | 6113/6118 | 1 |

| New Medium arm | Score2D | Largest-map native coverage | Retained maps | Verdict |
|---|---:|---:|---:|---|
| A, direct recovery | Unscorable | 35.03% | 7 | Negative; largest map lacks sufficient height-supported CPs for official Sim3 |
| B, landmark memory | 45.874750 | 49.80% | 4 | Negative against frozen 62.764799 |
| C, periodic fixed VI | Unscorable | 34.78% | 6 | Negative; official Sim3 cannot be estimated |

All three Medium estimators processed the full supported input and exited
normally. Unscorable means the official alignment failed; it is neither a
fabricated zero score nor an estimator crash. The original failed scorer logs
are retained. No alternative map or alignment was chosen to salvage a score.

### What the Medium experiments established

B supplied 6,211 additional camera associations, of which 3,265 survived on
accepted frames. The memory mechanism works, but that did not deliver better
trajectory continuity or score.

A made 1,051 recovery calls. Of 11,136 tested keyframe candidates, 11,020 still
fell below the 15-total-correspondence gate. Nevertheless, the completed run
produced 152 pose hypotheses, 76 successful PnP solver returns (iterations can
revisit a hypothesis), 12 accepted visual relatches, and five locally committed
recoveries. Seven failures restored the pre-relatch
state and did not restart grace. This is more geometric recovery activity,
but not a good trajectory result.

Three locally committed A recoveries discarded most visual support while
moving 5.49–8.52 m away from the visual hypothesis. The source trace reveals
that coast relatching bypasses normal relocalization bookkeeping, so its
local VI solution can pass an inherited **>10-inlier RECENTLY_LOST gate**,
without the normal post-relocation 50-inlier protection. The optimizer also
has an existing weak-support outlier rescue path. These gates were not added
or loosened by this batch. The outer transaction repair handles declared
failures, not states this policy declares successful.

See `docs/RECOVERY_HANDOFF_20261003.md` for exact state, prior, preintegration,
source and log evidence. Large displacement measures disagreement between
estimates; it does not establish which pose is correct against GT. A coherent
recovery-state transition remains unimplemented and untested.

C attempted 18 periodic fixed-VI solves, including 15 after fragmentation;
all were rejected. Its per-map scheduling works. This does not prove all
global VI refinement is ineffective: C compares held-out residuals against
the original live map. In v2, the free-camera candidate is compared against
an already optimized fixed-camera branch. That comparator difference is
documented in C's README; v2 logs did not preserve the original medians, so an
actual harmful calibration commit cannot be established from those logs.

### Completed Long results

| Arm | Score2D | CP recovered / within 1 m | Native coverage | GT associated | Retained maps |
|---|---:|---:|---:|---:|---:|
| A, direct recovery | 38.941196 | 18/27 / 6 | 62.50% | 3857/6118 | 8 |
| B, landmark memory | **42.904563** | 19/27 / 8 | **68.36%** | 4163/6118 | 6 |
| C, periodic fixed VI | 37.630917 | 16/27 / 11 | 56.58% | 3545/6118 | 13 |

B is the strongest **new** Long arm, +8.445662 over native v2. It remains
6.976971 below the earlier continuous Long. The earlier baseline has different
initial intrinsics, so it is a preservation reference, not a matched ablation.
B held one map from approximately 413.35 to 1640.55 s (1227 s), compared with
the native-v2 largest span of about 858 s. However, six retained maps remain.
Its memory supplied 3,787 additional associations, 2,198 of which survived on
accepted frames. Four periodic full calibration/VI updates were accepted
before its first split, versus native v2's one; this interaction prevents a
pure attribution to descriptor memory.

A produced two visual recoveries and committed both. One went from 84 visual
inliers to 11 after local VI and still committed; the other went from 54 to
30. Its direct matcher still rejected 8,106/8,148 candidate checks below the
15-total gate. It did not solve map continuity. C attempted 25 periodic VI
proposals, including 24 after fragmentation, and accepted none. **No periodic
VI correction reached live state in C**, so its score cannot demonstrate
benefit or harm from an accepted correction.

### Unchanged Medium repeatability control

One follow-up replay of the frozen v2 library, runner, settings and timestamp
input was checked byte-for-byte against the saved Medium run. It scored
**63.086620**, versus **62.764799** (+0.321821). Native scored coverage was
59.4787% versus 59.5040%; GT associations were 2497/4083 versus 2499/4083.
Both recovered 13/18 CPs. The first retained-map split was at 777.387 s versus
777.687 s, closely reproducing the problematic location.

The repeat retained four maps rather than three, accepted 5/12 calibration
trials rather than 4/12, and reported 16/613 visual recovery successes rather
than 0/438. Those successes are geometric hypotheses, not independently
verified complete VI handoffs. Runtime outcomes vary despite identical input
and code; a single repeat neither estimates variance nor identifies its
cause. The small score difference is not an algorithmic improvement. B's
45.87 Medium remains substantially worse than either unchanged result.

### Decision and next crossroads

**Promote none of these three candidates.** Preserve v2 for Short/Medium and
the historical continuity implementation for Long. Best recorded scores on
these individual recordings are Short 80.199161, Medium 63.086620 (unchanged
repeat), and Long 49.881534. These are local evaluations, not category means
or leaderboard submissions.

The next concrete implementation target is an explicit recovery confirmation
and inertial-state handoff. Retain/recheck the recovered geometric support
and reconcile the recovery window's pose, velocity, biases and valid temporal
anchor before accepting a prior. Existing VI already jointly optimizes these
current states; it is not inherently wrong to inherit temporal information.
Nor is changing >10 to >=15 sufficient: the Medium examples pass 15 as well.
No such new recovery-window estimator was implemented in this batch. Stop
here for the requested discussion rather than expanding into more trials.

Machine-readable results and source/product/score hashes are in
`docs/CONTINUITY_RESULTS_20261003.json`. Full comparison artifacts:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/comparison.png
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/trajectories.png
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/recovery_direct_medium/recovery_diagnostics/relatch_handoff.png
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/recovery_direct_long/recovery_diagnostics/relatch_handoff.png
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003/periodic_vi_proposals.png
```

This batch produced diagnostic PNGs and JSON, not new Rerun recordings. The
existing evaluated baseline Reruns remain unchanged. In a healthy result,
one estimated map would cover the gray GT route; recovery would retain
verified support rather than accepting severe support collapse. Missing
sections and failed official alignments remain visible in these diagnostics.
