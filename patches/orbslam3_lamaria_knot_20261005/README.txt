orbslam3_lamaria_knot_20261005  (W10a)
=====================================
Base: orbslam3_lamaria_babyassist_20261004. Adds the knot regime (new_files/include/LamariaKnot.h): detector
(10 s medians: gyro RMS > 1.1 rad/s and speed < 0.8 m/s, 3 s hysteresis), knot keyframes at sweep reversals and
every 10 deg of rotation, Baby landmark promotion at knot keyframes while tracking is OK, time-based local
inertial BA window (5 s) with a whole-knot closure window on the first keyframe after a knot, assist forced on.
Logs: [KNOT] enter/exit, [KNOT_KF], [KNOT_PROMOTE], [KNOT_BA]. Env: LAMARIA_KNOT=0 disables the regime.
Tests: tests/run_contract.py (14 Baby + 10 assist + 8 knot checks).
Revision 2 (2026-10-05 03:00): knot keyframes every 15 deg / at most 5 per second; Baby promotion capped at 30 per keyframe and 300 per knot with at least 4 verified views; temporal-association matcher guarded.
