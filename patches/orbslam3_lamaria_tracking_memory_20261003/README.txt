LaMAria experiment B: short accepted-landmark appearance memory (2026-10-03)

Base: exact frozen orbslam3_lamaria_online_full_v2 native-calibration source,
objects, runner and camera model. This package is a separate candidate.

Problem addressed:
Tracking.cc SeedTemporalLandmarks only uses the immediately previous image.
If an otherwise successfully tracked frame misses a landmark, the next image
cannot use that landmark's most recent accepted appearance; normal map search
uses the landmark's representative descriptor. This candidate retains a small
set of recent ACCEPTED observation descriptors to recover such associations
before the whole tracker fails. No synthetic pose extension is added.

Mechanism:
- Exact existing 32-byte binary ALIKED/ORB descriptor representation.
- 0.5-second age limit, at most 4096 entries, keyed by lens and landmark ID.
- Reset on tracker owner, map ID, initial-keyframe epoch, or time discontinuity.
- Copy descriptors; store no MapPoint pointer, Frame, pose, or marginal prior.
  Resolve IDs from the current live map under the existing map-update lock.
- Run after ordinary local projection matching, only while tracking is OK,
  and only below 80 incoming non-outlier associations.
- Mutual descriptor match with existing temporal distance <=70 and ratio<0.8.
- Require the production native-camera frustum, viewing-angle, distance policy,
  search radius, octave filtering and far-point policy. No wider search gate.
- Never overwrite occupied features or assign one landmark twice in one lens.
- Existing visual/VI optimizer and acceptance thresholds decide final inliers.
- Cache updates only on successful TrackLocalMap, using nonbad accepted inliers.
- Extra visible landmarks are charged once before optimizer found statistics;
  existing blind-frame refunds continue to apply.

Also includes the COMMON experiment transaction repair supplied by experiment A:
capture IMU state immediately before a provisional visual relatch, restore that
state on failed local confirmation, and refresh grace only after confirmation.
This repair is shared by experiments A and B; experiment C retains v2 tracking.
The distinguishing B mechanism is the accepted-descriptor memory.

Configuration:
Candidate enables the memory by default. LAMARIA_TRACKING_MEMORY=0 disables
only the memory for a control. It never changes stock/frozen v2 artifacts.
[TRACK_MEMORY_DIAG] records cache size, considered and projection-rejected
candidates, seeded and accepted surviving associations by lens, and final
tracking acceptance/inlier count. Recovered candidates are not claimed to be
correct until the normal optimizer accepts them.

Build (one compiler job):
  python patches/orbslam3_lamaria_tracking_memory_20261003/build_incremental.py --jobs 1

The incremental builder verifies frozen v2 source/header/product/object SHA256s
and compiler image identity. It recompiles only Tracking.cc, links 34 verified
unchanged objects, and copies the exact v2 runner. Class layouts are unchanged;
the changed inline transaction helper is consumed by Tracking.cc. delta.patch
is relative to frozen v2; geometry.patch plus new_files is vendor-relative and
can also be used by the existing full build_lamaria.py builder.

Contracts:
  python patches/orbslam3_lamaria_tracking_memory_20261003/tests/run_contract.py

The fixture uses the candidate helper and linked production Frame/MapPoint,
Fisheye624, grid, scale and matcher radius implementations. It checks recovery
across one missing association, descriptor copying, per-lens pooled indexing,
rejected observations, age/map/epoch/owner/time resets, exact native projection
and scale rejection, descriptor ambiguity, occupied feature/landmark rejection,
and lookup after actual map-point deletion. It does not establish real-run gain.

Build and contract output:
  build/orbslam3_lamaria_tracking_memory_20261003/
  build/orbslam3_lamaria_tracking_memory_20261003/tests/tracking_memory/results.json

Full Medium and Long run result paths (created by the coordinated harness):
  experiments/lamaria_continuity_batch_20261003/tracking_memory_medium/
  experiments/lamaria_continuity_batch_20261003/tracking_memory_long/
