# LaMAria leaderboard with Raghav-Slam, 2026-10-05

Raghav-Slam rows are our official local scores (same toolkit and control-point protocol as the website)
on the three controlled training recordings, one run each: Short `w11_assist_walk10_short` (Baby assist before
loss + rollback + IMU walks /10), Medium `w11_assist_walk10_medium_b` (same method), Long `w14a_fused_long`
(Baby + walks /10 + bias hold on the fused left+right virtual IMU). Updated 2026-10-05 after round 4
(previous entry: 81.4 / 82.9 / 50.4). The leaderboard rows are website test-set results; categories are not directly comparable to
one training recording. Low light and moving platform: not run. Sorted by Short score.

| Method | Sensors | Short score | Short R@5m | Medium score | Medium R@5m | Long score | Long R@5m |
|---|---|---:|---:|---:|---:|---:|---:|
| (r) Aria's SLAM | bino, imu | 90.7 | N.A. | 78.5 | N.A. | 70.9 | N.A. |
| **Raghav-Slam** | bino, imu | **89.2** | **100.0** | **83.9** | **99.9** | **54.3** | **99.2** |
| AnonSLAM | bino, imu | 80.2 | 99.9 | 61.6 | 96.2 | 59.9 | 99.3 |
| microSLAM | mono | 34.2 | 73.4 | 18.9 | 34.8 | 9.4 | 19.4 |
| Mighty Camera | mono, imu | 31.6 | 65.9 | 29.3 | 60.7 | 16.8 | 37.1 |
| (r) OpenVINS+Maplab | bino, imu | 27.7 | 60.8 | 23.4 | 52.3 | 12.8 | 26.1 |
| (r) ORB-SLAM3 | mono, imu | 23.0 | 61.2 | 10.9 | 26.0 | 11.2 | 28.7 |
| (r) OKVIS2 | bino, imu | 20.0 | 50.0 | 11.6 | 27.9 | 2.6 | 1.4 |
| (r) OpenVINS+Maplab | mono, imu | 19.7 | 43.8 | 12.7 | 29.0 | 5.5 | 10.2 |
| (r) OpenVINS | bino, imu | 18.9 | 54.2 | 15.9 | 46.8 | 12.5 | 28.1 |
| (r) OpenVINS | mono, imu | 16.0 | 40.3 | 10.6 | 24.0 | 5.5 | 13.7 |
| ViPE | mono | 15.8 | 32.8 | 9.0 | 19.0 | 2.6 | 5.8 |
| DROID-W | mono | 11.4 | 26.1 | 2.7 | 5.8 | 0.0 | 0.0 |
| (r) DPV-SLAM | mono | 8.0 | 14.1 | 8.8 | 15.6 | 3.9 | 8.7 |
| Test 4 | mono | 7.9 | 3.0 | N.A. | N.A. | N.A. | N.A. |
| (r) DPVO | mono | 7.6 | 18.0 | 7.5 | 15.4 | 1.7 | 2.9 |
| (r) Kimera VIO | mono, imu | 6.8 | 10.1 | 8.3 | 18.7 | 7.2 | 17.2 |
| LingBot-Map | mono | 1.6 | 2.4 | 0.7 | 0.9 | 0.3 | 0.3 |
| MVIO | bino, imu | N.A. | N.A. | N.A. | N.A. | N.A. | N.A. |

Raghav-Slam runs:
- Short 89.17 (14/14 control points within 1 m): `/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_batch4_20261005/w11_assist_walk10_short`
- Medium 83.92 (18/18 within 1 m): `/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_batch4_20261005/w11_assist_walk10_medium_b`
- Long 54.33 (8/27 within 1 m, 22/27 within 2 m): `/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_batch4_20261005/w14a_fused_long`
- Reruns: `/home/raghav/workspace/MeckaAI/LamariaRERUNS/` (Short_89.17_assist_walk10_short.rrd, Medium_83.92_assist_walk10_medium_b.rrd, Long_54.33_fused_imu_long.rrd)

Previous Raghav-Slam entry (2026-10-04): Short 81.39 `lamaria_batch2_20261004/imuwalk10_short`, Medium 82.88
`lamaria_batch3_20261004/babyassist_medium`, Long 50.37 `lamaria_batch3_20261004/biashold_long`.
