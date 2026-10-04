gyropred + preinit-coast (2026-10-04) on the BabyFeatures tree
================================================================
Mechanism 1 (gyropred): before IMU initialisation, predict the camera pose with
the frame-to-frame gyro rotation and constant-velocity translation instead of a
pure constant-velocity model. Env LAMARIA_GYRO_PREDICT=0 disables.
Mechanism 2 (preinit-coast): when a young map (>10 KFs) loses tracking before
IMU initialisation, do not reset immediately; coast up to 1.0 s on that
prediction, keep matching the temporal-keyframe local map, re-enter at the
normal 15-inlier stereo bar, reset only when the grace expires.
Env LAMARIA_PREINIT_COAST=0 disables. Evidence: Long first map lost at 163.25 s
in a 0.5 s blur/dropout burst (inliers 275 -> 9, detections 1490 -> 362/410),
tracking was strong again at 163.55 s (674 inliers); gyro prediction alone
reproduced the collapse frame for frame.
Build: python3 patches/orbslam3_lamaria_gyropred_20261004/build_incremental.py --jobs 2
