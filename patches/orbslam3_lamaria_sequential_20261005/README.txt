orbslam3_lamaria_sequential_20261005
====================================
Base: orbslam3_lamaria_babyassist_20261004 (Baby assist before loss). Adds the deterministic sequential mode:
after a keyframe insertion the tracker waits, at the start of the next frame and before taking the map-update mutex, until the local mapper has drained its queue and is idle
(poll 0.5 ms, skipped while the mapper initialises the IMU, 60 s timeout, logged). LAMARIA_SEQUENTIAL=0 disables. Log "[SEQUENTIAL] t= waits= total_wait_s=".
Tests: tests/run_contract.py (14 Baby + 10 assist checks). Determinism is verified by two full runs.
