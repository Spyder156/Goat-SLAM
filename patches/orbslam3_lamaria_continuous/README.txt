Complete LaMAria continuity candidate.
Includes initialized-IMU recovery, temporal associations, two-camera
relocalization, recovery mapping, atlas export and coordinated shutdown.
The coarse loop matcher now uses the actual camera model for projection.
Enable LAMARIA_TEMPORAL_ASSOCIATIONS=1 and IMU.InsertKFsWhenLost: 1.
Full-run validation and reproduction: /home/raghav/workspace/MeckaAI/lamaria_audit_20261002_continuity/README.txt
