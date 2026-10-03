# BabyFeatures SOS: Long experiment

**Completed 2026-10-04:** Long43.729292, one map after startup, 99.38% native
pose coverage. A separate saved-graph VI refinement scored45.027586. Neither
replaces historical Long49.881534; Baby Medium/Short remain untested. The default
output below already exists and must not be reused. Full resume instructions and
the user's current documentation-only stop: `docs/RESUME_20261004.md`.

`long_native.yaml` is byte-identical to frozen v2's
`configs/orbslam3_lamaria/online_full_v2_transfer_20261003/long_native.yaml`
(SHA256 `6adf86a286d5cf4f9e037d19f1a323721965d0f5bd437b8f3eb9f4613139111c`).
BabyFeatures are enabled explicitly by `--baby-features`, which passes
`LAMARIA_BABY_FEATURES=1` to the isolated estimator container. Normal launchers
remain opt-out, and no existing acceptance threshold or calibration is edited.

After the private candidate build is ready, launch from the project root:

```bash
/home/raghav/miniconda3/envs/lamaria/bin/python -B pipeline/run/babyfeats_experiment.py
```

The default fresh output is `experiments/lamaria_babyfeats_long_20261003`.
The launcher preserves the original full Long timestamps, both ALIKED caches,
factory-corrected IMU, temporal associations, four CPUs and debugger diagnostics.
It runs the existing official local scorer on the largest retained map selected
without ground truth, retaining all missing-pose/control-point penalties.

Results include `status.json`, `provenance.json`, `summary.json`,
`diagnostics.png`, and `babyfeats_diagnostics/continuity.png`. SOS accepted
updates, confirmed recovery events, map creations, discarded-map resets and
retained independent maps remain separate measurements. The frozen native-v2
reference is 34.458901 with five retained maps. Historical Long 49.881534 uses
different initial calibration and is explicitly not a matched ablation.

Use `--render` to additionally create `evaluation/run.rrd` from the saved official
evaluation Sim3 after scoring. Rendering is deferred by default for fast feedback.
Existing output directories are never reused; choose a new `--out` for a retry.
