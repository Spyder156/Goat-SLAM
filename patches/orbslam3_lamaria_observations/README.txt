LaMAria dual-camera observation repair candidate, 2026-10-02

Cumulative overlay on the history/continuity baseline. Upstream vendor is read-only.
Registers each camera independently at keyframe insertion and fusion; counts repeated registration once; preserves complementary camera observations on replacement; avoids requeuing old landmarks as newly created points. Final shutdown audit checks both observation directions and weighted counts.

Matching thresholds, feature cache, camera/IMU calibration and metric bootstrap policy are unchanged. This build is an experimental candidate until full-sequence evaluation is complete.

Production-linked regression: pipeline/run/tests/run_observation_contract.py
Build: python pipeline/run/build_lamaria.py --patch-dir patches/orbslam3_lamaria_observations --out build/orbslam3_lamaria_observations --jobs 4
