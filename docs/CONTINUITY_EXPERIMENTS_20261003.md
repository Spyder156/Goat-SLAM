# Medium and Long map continuity experiments

The user authorized three separate implementations and full Medium/Long
replays. The frozen Short 80.199161 implementation remains unchanged. This
batch addresses map fragmentation; motion-knot/control-point refinement is
deferred. Ground truth enters evaluation and diagnostic plots only.

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

Full replays are in progress. Preserve the measured frozen references until
the completed six-case score table is written here:

| Frozen reference | Score2D | CP triangulated | GT associated | Retained maps |
|---|---:|---:|---:|---:|
| Medium v2 | 62.764799 | 13/18 | 2499/4083 | 3 |
| Long v2, native calibration | 34.458901 | 15/27 | 3163/6118 | 5 |
| Earlier Long, different initial calibration | 49.881534 | 27/27 | 6113/6118 | 1 |

One replay per variant does not establish repeatability. Earlier frozen
references were not rerun in this batch; asynchronous mapping and BA can
change timing and accepted updates. Report the strongest observed result,
without attributing every difference to one source edit.
