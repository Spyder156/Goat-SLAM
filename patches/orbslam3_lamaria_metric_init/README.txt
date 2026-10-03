LaMAria metric stereo-inertial initialization candidate, 2026-10-02

Cumulative overlay on orbslam3_lamaria_observations. Vendor remains read-only.
Corrects the multiplicative scale Jacobian and preserves the physical camera-to-IMU lever arm under map rescaling. Builds a copied multi-KF stereo-inertial proposal, marginalizes nuisance variables for a conditional scale-information check, and commits only an accepted proposal from an unchanged map snapshot. Stereo camera projection uses native Fisheye624. Refines metric scale at initial IMU initialization and both early VI-BA stages. Rejected proposals can retry; current-frame tracking continues while KF creation is held.

Matching bounds, feature extraction/cache, supplied calibration, and the existing provisional visual bootstrap are unchanged. No ground truth enters the estimator. Joint full inertial BA remains responsible for pose/landmark refinement after accepted scale initialization. This is a candidate, not the default, until end-to-end validation.

Build: python pipeline/run/build_lamaria.py --patch-dir patches/orbslam3_lamaria_metric_init --out build/orbslam3_lamaria_metric_init --jobs 4

2026-10-03 checkpoint: final-library mathematical regression passed all six cases. Original-start 120-second tests completed on Short and Medium without crashes or resets. Medium accepted initialization and both VI-BA stages; Short rejected all 227 proposals at the stereo-consistency guard and never initialized IMU. Do not promote this candidate or interpret it as a solved metric initializer. The fixed-pose/fixed-landmark scalar proposal needs joint visual-inertial geometry refinement before its commit decision. See docs/LAMARIA_RELIABILITY.txt and the per-run viz/metric_init_report/report.html for evidence and scope limitations.
