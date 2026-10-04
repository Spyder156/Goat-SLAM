orbslam3_lamaria_biashold_20261004
==================================
Base: orbslam3_lamaria_coastguard_20261004 (robust Frame + rollback-era tree + 3 m/s coast guard).
Adds bias hold during visual starvation (new_files/include/LamariaBiasHold.h): while the previous frame
was coasting, RECENTLY_LOST or Baby-bridged, IMU biases are held at the last observed value: Baby window
solve pins and fixes every bias vertex; pose-only inertial optimisation and local inertial BA scale the
random-walk edges by 1e4 (local BA only for keyframes inside a hold window). Logs "[BIAS_HOLD] t= begin/end".
Env: LAMARIA_BIAS_HOLD=0 disables; LAMARIA_BIAS_HOLD_GAIN sets the gain.
Build: python3 build_incremental.py --jobs 3 ; tests: tests/run_contract.py (14 Baby + 9 bias-hold checks).
