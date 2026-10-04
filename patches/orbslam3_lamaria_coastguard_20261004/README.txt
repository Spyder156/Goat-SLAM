coastguard (2026-10-04) = BabyFeatures tree + robust Frame + two coast-state mechanisms
1. Tracking.coastRestore: 1 in the arm configs: a rejected visual update restores the
   IMU-predicted pose/velocity/bias instead of leaving the failed optimum in the frame
   (mechanism already in the fork, off in every frozen config).
2. Pedestrian speed bound on inertial propagation while coasting (3 m/s), both
   PredictStateIMU branches. Evidence: imuwalk10 Medium 866-882 s: one failed update left
   6.7 m/s, pure IMU coasting then reached 41 m/s and the map split; the Baby run that
   passed the same section never coasted there. LAMARIA_COAST_GUARD=0 disables.
Build: python3 patches/orbslam3_lamaria_coastguard_20261004/build_incremental.py --jobs 3
