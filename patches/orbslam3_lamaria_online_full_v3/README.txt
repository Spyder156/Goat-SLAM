Full native online-calibration candidate v3 (experimental; not default)

Base: frozen orbslam3_lamaria_online_full_v2. All calibration solver, priors,
bounds, matching thresholds, and no-coast-keyframe policy remain identical.
Only production behavior change: capture pose/velocity/bias at the successful
PredictStateIMU return, before any visual-relatch hypothesis changes pose.
Failure of local-map validation restores this complete propagated state and
invalidates the optimizer marginal prior. Accepted updates retain their state.
The per-input snapshot is cleared before each Track call.

Why: v2 at native549.7422 s accepted a visual relatch hypothesis, then rejected
its inertial/local-map optimization. Its snapshot had been captured too late,
so failure exported the relatch pose, causing a2.722269 m jump; the later
accepted optimization returned2.975665 m. v2 remains preserved for comparison.
At554.9922 s v2's14.44 m diagnostics instead describe relatch and optimizer
undo within one frame; actual output step was0.222910 m, not14.44 m.

The focused production Frame/rollback regression checks a visual relatch
followed by rejected optimization exports the pre-relatch propagated PVB and
no stale marginal prior. It also checks accepted updates preserve PVB/prior.
See build/orbslam3_lamaria_online_full_v3/tests/relatch_rollback/results.json.

The v2 calibration geometry/domain contracts remain applicable (42 passed).
No ground truth enters estimation. No vendor/default source is modified.
Full Short replay completed:18,351 supported inputs, exit0, runtime924.36s.
Score2D42.593277: three independent maps; the scorer selects map0 with7,977
poses, CP7/14, GT1384/2999, and counts1615 missing GT timestamps as failures.
Other maps retain7432 and2919 poses in explicit independent atlas frames;
none is stitched into the submitted/scored trajectory. Selected-map scale
1.022461, median horizontal error.226377m/RMSE.330388m on its covered subset.
This is a continuity REGRESSION versus v2's complete80.199161 result.

Live rollback audit: all205 rejected initialized inertial updates used the
pre-relatch prediction, restored zero pose/velocity delta and retained no
marginal prior. The implementation repair worked; overall SLAM is not robust.
The first loss began458.6422s and the5s grace expired463.6922s. Another map
started835.8922s. v3 accepted3calibration updates in7trials, including442.6922s
where v2 rejected its corresponding trial; asynchronous keyframe/optimization
scheduling changed the calibrated solution before the first failed rollback.
Do not attribute the entire score difference to the boundary repair alone.
See experiments/lamaria_online_full_v3_short_full_20261003/diagnostics for
rollback_audit.json and commit_to_loss.png/json. No new experiment follows
until the requested batch is discussed.
