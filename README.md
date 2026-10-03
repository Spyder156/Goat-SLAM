# Goat-SLAM

Native dual-fisheye visual-inertial SLAM for LaMAria, based on a modified
ORB-SLAM3 pipeline with ALIKED features, metric initialization, periodic VI
refinement, and bounded intrinsic calibration.

## Start here

Read `docs/HANDOFF.md` and `docs/RESUME_20261004.md` first, then `GOAL.md`,
`ROADMAP.md`, `EXPERIMENTS.md`, and `IDEA_POOL.md`. The resume note records the
latest stopping point and the next diagnostic question. **Work stopped at the
user's usage-limit request; this documentation update did not start another run.**
Measured results and proposed work must remain distinct.

## Preserved results

These are local evaluations on training recordings, not a leaderboard submission.
Scores from different sequences are not directly interchangeable.

| Implementation | Sequence | Score2D | Surviving maps |
|---|---|---:|---:|
| Online full v2 | Short `sequence_1_19` | 80.199161 | 1 |
| Online full v2 | Medium `sequence_2_11` | 62.764799 | 3 |
| Online full v2, native calibration | Long `sequence_3_17` | 34.458901 | 5 |
| Fixed-intrinsics reference | Short `sequence_1_19` | 67.188379 | 1 |
| History/continuity | Long `sequence_3_17` | 49.881534 | 1 |
| BabyFeatures SOS | Long `sequence_3_17` | 43.729292 | 1 |
| BabyFeatures plus final fixed-camera global VI | Long `sequence_3_17` | 45.027586 | 1 |

The BabyFeatures Long experiment retained **35,619 / 35,842 poses (99.38%)** in
one map after startup and recovered all **27 control points**. Its separate final
native-graph VI refinement improved the score by **1.298294** with exactly the
same timestamps and coverage; a **1.622 m adjacent-pose jump** remains. This is
improved continuity and a modest offline accuracy gain, not a solved Long route.
BabyFeatures has **not been tested on Medium or Short**. Short 80.20 and historical
Long 49.88 remain preserved; the historical Long used different initial
calibration, so these are not all matched algorithm comparisons. See
`docs/BABYFEATURES_LONG_20261003.md` and
`docs/BABYFEATURES_NATIVE_VI_20261004.md` for implementation, tests and limitations.

The frozen v2 Short method still fragments in its Medium and Long transfers.
Preserve the continuous Long baseline separately. The evaluator scores the selected map;
independent atlas maps must not be concatenated into a claimed full trajectory.
Evaluation Sim(3) alignment is separate from raw metric-scale accuracy.

The completed new Rerun shows the **43.73 BabyFeatures source run**, not the 45.03
offline refinement:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_long_20261003/evaluation_baby_20261004/run.rrd
```

The user reports a missing beginning, end drift and apparent tilt. The known
coverage loss is **223 startup frames**; missing GT associations are also all
before the first estimate. Causes of drift and the apparent tilt remain
unverified. Next inspect native/export/evaluation frames, gravity and local error
over time before attributing these observations to calibration or initialization.
Ground truth may diagnose/evaluate the result; it must not enter the estimator.

The initial preservation snapshot is tagged `baseline-short-80.20`,
`baseline-short-67.19`, and `baseline-long-49.88`. These tags retain the complete
repository; their names identify which frozen profile to select.

## Verify and restore

```bash
python3 tools/restore_baselines.py --verify
python3 tools/restore_baselines.py --destination /absolute/path/to/new-working-copy
```

The destination must not exist. Restoration preserves the current checkout and
recreates the historical private build layout from checked source and runtime
archives. See `docs/BASELINE_REPRODUCTION.md` for exact profiles, build/run
commands, external data requirements, and validation limits.

The frozen executables require the recorded Linux environment and compatible
CPU. The local Docker image is backed up separately; datasets, feature caches,
vocabulary, Python environments, and multi-gigabyte Reruns are not included in
Git. This is not a claim of a tested clean-machine rebuild or bitwise-repeatable
SLAM runs. The historical builder defaults to the history profile, so select
the v2 patch/build/config explicitly when reproducing the 80.20 result.

Validation: restoration from a fresh Git clone, all 576 snapshot hashes, all
three original-builder preflights, and loading all three restored executables
in the recorded Docker image passed. `docs/PRESERVATION_CHECK.json` records the
checks. No new full compilation or SLAM replay was performed for publication.

## Layout

| Path | Contents |
|---|---|
| `pipeline/` | Dataset processing, runners, evaluation, visualizations, and batch refinement |
| `configs/` | Recorded estimator configurations and experiment settings |
| `patches/` | Versioned implementation variants and regression probes |
| `vendor/` | Source snapshots and upstream provenance, including local modifications |
| `baselines/` | Frozen source overlays, build manifests, exact runtime bundles, and hashes |
| `tools/` | Snapshot verification and safe restoration |
| `docs/` | Handoff, reproduction, evaluation, storage, and licensing notes |
| `third_party/`, `build/` | Local working dependencies and builds; excluded from Git |
| `experiments/` | Local artifact storage; excluded from Git |

Keep the frozen snapshots unchanged when developing a new method. Use a new
patch/profile and output directory; retain scores and full-sequence coverage
alongside geometry diagnostics. Do not edit working upstream checkouts directly.
Ground truth is used for evaluation, never estimator initialization, calibration,
matching, or map stitching.

## License

ORB-SLAM3-derived code is GPL-3.0-or-later; see `LICENSE`. Dependencies retain
their own licenses and original authorship. See `docs/THIRD_PARTY.md` and the
notices distributed with each source snapshot.
