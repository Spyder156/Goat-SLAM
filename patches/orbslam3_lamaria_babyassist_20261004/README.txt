orbslam3_lamaria_babyassist_20261004
====================================
Base: orbslam3_lamaria_coastguard_20261004. Adds Baby assist before loss (new_files/include/LamariaBabyAssist.h):
a thin-tracking monitor (previous inliers < 80 or local-map matches < 10 % of keypoints, 5 consecutive
frames, IMU initialised, state OK) runs one Baby window solve on a copy of the frame with ordinary frames
fixed; verified landmarks (>= 3 views, >= 1 deg parallax, unassociated features) enter the pose-only
inertial optimisation as temporaries with information x 0.2 and Huber; never MapPoints; cleared per frame.
Logs "[BABY_ASSIST] t= prev_inliers= matches= keypoints= solve= temporaries= used= chi2_temporaries= chi2_map=".
Env: LAMARIA_BABY_ASSIST=0 disables; LAMARIA_BABY_ASSIST_WEIGHT.
Build: python3 build_incremental.py --jobs 3 ; tests: tests/run_contract.py (14 Baby + 10 assist checks).
