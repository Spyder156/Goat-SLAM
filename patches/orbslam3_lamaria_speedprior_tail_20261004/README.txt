orbslam3_lamaria_speedprior_20261004
====================================
Base: orbslam3_lamaria_coastguard_20261004 (robust Frame + coast speed guard) on frozen v2 objects.
Tail variant: default stages baby,predict (pose-only and local-BA edges off). Adds a pedestrian speed-bound prior (see new_files/include/LamariaSpeedPrior.h):
  - one-sided soft prior e = max(0, |v| - 2.2 m/s), sigma 0.01 m/s, on body-velocity vertices in
    PoseInertialOptimizationLastKeyFrame / LastFrame (frame velocity), LocalInertialBA
    (optimisable keyframe velocities) and the Baby solver (non-fixed frame velocities);
  - pure IMU propagation in PredictStateIMU clamped to the same bound (log "[SPEED_PRIOR] stage=predict").
Evidence: GT 1 s-window speed never exceeds 1.84 m/s on any sequence; Baby Medium estimated
2.5-5.1 m/s from 1170 s (end-of-route scale creep, tail 27 m off).
Env: LAMARIA_SPEED_PRIOR_MAX (m/s, 0 disables), LAMARIA_SPEED_PRIOR_SIGMA (m/s).
Logs: "[SPEED_PRIOR] t=<s> stage=pose_lastkf|pose_lastframe speed_before= speed_after= chi2=" (only when
active), "stage=local_ba n_over_before= max_before= n_over_after= max_after=" (only when active).
Build: python3 build_incremental.py --jobs 3 ; tests: tests/run_contract.py (14 Baby + 11 speed-prior checks).
