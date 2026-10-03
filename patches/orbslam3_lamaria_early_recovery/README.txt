LaMAria continuity repair
=========================
This package includes the prior calibrated geometry/indexing/landmark-ownership
repairs plus the following continuity changes:

* Camera-local temporal descriptor associations seed the existing geometric
  optimizer (LAMARIA_TEMPORAL_ASSOCIATIONS=1). They do not bypass outlier checks.
* Relocalization can solve from either physical camera and converts a cam1
  solution into the frame's cam0 pose. Pooled observation indices remain pooled.
* The temporal local-keyframe walk advances even when a keyframe is already in
  the local set.
* With IMU.InsertKFsWhenLost: 1, confidence-armed, initialized IMU recovery may
  insert keyframes and triangulate fresh points within the existing grace time.
* Recovery arming requires the existing 30-frame/30-inlier confidence streak and
  initialized IMU, without additionally waiting for BA2. No refinement flag is
  forged, and geometric gates, inlier floors, and five-second timeout remain.
* Shutdown waits for mapping, loop closing and outstanding global BA workers
  before saving. All retained map segments and per-input pose status are saved
  with explicit coordinate-frame identities; independent maps are not joined.

Build with pipeline/run/build_lamaria.py using this patch directory. The output
must stay outside third_party. Pinned vendor hashes are in base_manifest.json.
The complete candidate settings and controlled validation records are under:
/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_continuity

Completed prefix tests and real-camera solver tests pass. The full-sequence
candidate lamaria_early_recovery_full_20261002 is being evaluated; final results
will be recorded in the audit README.txt. This package alone does not enable
its opt-in temporal environment setting or keyframe-insertion configuration.
