# Goat-SLAM

Native dual-fisheye visual-inertial SLAM for LaMAria, based on a modified
ORB-SLAM3 pipeline with ALIKED features, metric initialization, periodic VI
refinement, and bounded intrinsic calibration.

## Start here

Read `docs/HANDOFF.md` first, then `GOAL.md`, `ROADMAP.md`, and `EXPERIMENTS.md`.
The handoff distinguishes measured results from proposed work. This repository
preserves the current implementations; publishing it does not introduce a new
tracking or calibration change.

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

The strongest Short method still fragments Medium and Long. Preserve the
continuous Long baseline separately. The evaluator scores the selected map;
independent atlas maps must not be concatenated into a claimed full trajectory.
Evaluation Sim(3) alignment is separate from raw metric-scale accuracy.

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
