orbslam3_lamaria_joint_init: isolated experimental overlay on orbslam3_lamaria_metric_init.
Vendor is read-only. See candidate_provenance.json for exact staging hashes and docs/LAMARIA_RELIABILITY.txt for experiment outcomes.
Build with pipeline/run/build_lamaria.py --patch-dir patches/orbslam3_lamaria_joint_init --out build/orbslam3_lamaria_joint_init --jobs 4.
This candidate is not the default.

2026-10-03 checkpoint:
Refines copied poses, landmarks, velocity, gravity and biases jointly with native
camera and IMU factors before validating and committing initialization. The
earlier scalar proposal is a seed, not the final metric geometry. Physical rig
and camera/IMU lever arms stay metric. Culled-keyframe relative translations are
also scaled once during Tracking::UpdateFrameIMU, to preserve export chains.

Production tests: 10 normal cases pass. The explicit conflicting-fixed-boundary
diagnostic remains an accuracy failure: small visual residuals do not guarantee
correct metric scale under wrong fixed geometry. Real startup windows here had
zero fixed boundary keyframes, so that fixture does not explain their error.

Equal-input 120s Short and Medium runs both completed; initial IMU and both early
VI-BA stages were accepted. Short SE3 RMSE4.347502->4.158946m and diagnostic
shape residual after Sim3 .453423->.219004m. However the trajectory is still
about16.40% oversized (estimate-to-GT scale.859095074). Medium SE3 RMSE
.200021->.137589m. Functional initialization improved; Short metric accuracy is
NOT fixed. No full-route joint run was started at this checkpoint.

The valid-domain patch is a separate candidate, not included in these runs.
Neither matching bounds nor supplied calibration coefficients were adjusted.
See docs/LAMARIA_RELIABILITY.txt and the experiment checkpoint for evidence.
