Periodic full-map fixed-intrinsic VI candidate (2026-10-03)

Forked from preserved orbslam3_lamaria_online_full_v2. Not a new default.
Neither third_party, the frozen v2 source/build nor prior run artifacts changed.

Config: Calibration.periodicVIMode
  0 (default): original v2 single-map full calibration and global cadence.
  1: full calibration while one populated map exists; fixed-intrinsic full VI
     on the active map once multiple maps exist (also fixed if fullOnline=0).
  2: periodic fixed-intrinsic full VI from initial BA2 onward, including with
     one map. This is the experiment C control requested for Medium and Long.
Calibration.intervalSeconds retains its original value and validation.
New modes use a last-trial clock keyed by (map ID, initial keyframe ID), so a
newly initialized/reset map is not starved by a different map's recent solve.

Exact existing native visual/IMU/bias graph, fixed first-pose gauge and rig.
The fixed branch performs the existing 10 warm + 20 fixed-camera iterations;
it never releases the camera vertices or mutates live intrinsic parameters.
It compares held-out centre/periphery errors against the pre-solve geometry,
using the existing +0.03px median, 95% inlier-retention and summed 0.004px
improvement gates. Existing finite-state and physical bias gates remain.
Acceptance requires reduced robust training objective. No tracking gates,
input calibration, extrinsics, image/features, or GT constraints are changed.

Input, loop/GBA-reader and active-map locks quiesce live state during solve
and commit. Rejection leaves the private graph discarded. Acceptance updates
poses/points/velocities/biases, transports live frames and invalidates shared
obsolete priors. Fixed mode preserves stereo/intrinsic caches; nonoptimized
single-view points preserve their previous camera-space coordinates while
following their anchor update. The shared-camera calibration guard remains.

Logs: PERIODIC-VI-CONFIG / SCHEDULE / START / HELDOUT / INERTIAL / TRIAL.
A successful VI trial is not an intrinsic-calibration commit.

Build: build/orbslam3_lamaria_periodic_vi_20261003 (jobs=1), completed clean.
Production contract: tests/exact_solver/result.json under that build.
The fixture normally constructs a System then joins its worker threads,
builds 12 inertial keyframes, 272 points, 3900 native observations and 11 IMU
edges with stereo overlap/acceleration, and invokes the actual compiled
Optimizer::FullCalibrationBA(..., false). All checks passed: accepted solve,
bit-identical 32 camera scalars, fixed gauge/rig/cache/other-map state,
shared prior invalidation, one map publication, improved metric landmarks,
and atomic rejection of an invalid IMU covariance.

Synthetic initial mean point error is 9.5mm; the accepted solution reduces
it below the fixture's 1.9mm threshold. Held-out medians fall from roughly
0.61--0.69px to 1e-5px. This validates mechanics, not real-sequence performance.
The first test fixture lacked metric observability (constant velocity,
disjoint camera tracks): its metric assertion failed despite tiny residuals.
Its log is retained as fixture_unobservable_initial.log. The final fixture
adds real stereo correspondences and acceleration rather than relaxing the
assertion. No estimator thresholds were changed in response to that test.

Reproduce the production contract with:
  python patches/orbslam3_lamaria_periodic_vi_20261003/tests/run_contract.py \
    --config configs/orbslam3_lamaria/online_full_v2_transfer_20261003/medium.yaml \
    --out build/orbslam3_lamaria_periodic_vi_20261003/tests/new_contract

Full Medium and Long results (initial authorized pair, no replay retries):
  Medium: replay exited 0, all 23,746 supported frames processed. Six retained
    maps. Official largest-map CP alignment failed because fewer than three
    recovered CPs had nonzero surveyed height; Score2D is unavailable, not 0.
    Largest map: 8,259 poses (34.78% of 23,748 native input frames).
    Atlas: 22,818 poses across independent map frames; not one trajectory.
  Long: replay exited 0, all 35,842 inputs processed. Score2D 37.630917,
    16/27 CPs triangulated, 3,545/6,118 dense-GT poses associated. Thirteen
    retained maps. Largest map: 20,280 poses (56.58% input coverage).
    Atlas: 32,518 poses, still separate map coordinate systems.
  Frozen v2 comparison: Medium 62.764799 / 3 maps; native Long 34.458901 /
    5 maps. Older history Long was 49.881534 / 1 map. C is not a continuity
    winner: Long's modest score gain accompanies substantially more maps,
    and Medium cannot be officially aligned. No favorable-map reselection.

Mechanism evidence:
  Medium: 18 proposals, 17 reached held-out validation, one rejected for a
    disconnected temporal chain. 0 accepted, 0 intrinsic commits. Fifteen
    proposals occurred with multiple populated maps; four map-local clocks
    made 3/2/7/6 attempts (map IDs 0/2/3/5). Across 68 evaluated camera/radius
    bins, 62 exceeded the +0.03px median gate and 20 failed inlier retention.
  Long: 25 proposals, all reached held-out validation. 0 accepted, 0 intrinsic
    commits. Twenty-four proposals occurred after fragmentation; seven
    map-local clocks made 1/3/17/1/1/1/1 attempts (IDs 0/1/2/4/5/10/11).
    Across 100 evaluated bins, 99 exceeded the median gate and 42 failed
    inlier retention. No free-calibration/fixed-BA comparator trials ran.
  All 43 schedules requested fixed VI with the pre-solve live comparator;
    42 reached that validation. No accepted periodic correction reached the
    live estimator. These results do not show accepted fixed VI updates
    harmed tracking. Disabling free calibration and solver timing also
    distinguish this candidate from v2; one replay each is not a causal
    isolation of intrinsic quality or backend updates.

Artifacts under:
  /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/
  lamaria_continuity_batch_20261003/
    periodic_vi_medium/{status.json,summary.json,diagnostics.png,periodic_vi_audit.json}
    periodic_vi_long/{status.json,summary.json,diagnostics.png,periodic_vi_audit.json}
    periodic_vi_proposals.png
The proposal plot compares candidate minus live held-out median by camera
and centre/periphery, then shows populated maps at each proposal. A useful
proposal would improve residuals while preserving every bin's gates and
appear as an accepted green marker. Here every marker is rejected/red.
The x-axis is seconds since first selected input, not native capture time.

Reproduce the read-only log analysis (writes only diagnostic artifacts):
  python patches/orbslam3_lamaria_periodic_vi_20261003/tools/analyze_runs.py \
    --suite /media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_continuity_batch_20261003

Acceptance-comparator distinction (verified from source, not a new result):
  Frozen v2: patches/orbslam3_lamaria_online_full_v2/new_files/include/
  FullCalibrationOptimizerImpl.h:95 captures pre-solve held-out statistics
  only for coverage. Lines 97-104 run warm/fixed/free branches; lines 106-110
  compare the free-camera candidate with the separately optimized fixed
  branch, not with the live pre-solve map. Training cost is additionally
  checked against initial/fixed costs. This logic permits a free candidate
  whose held-out residual is better than the fixed branch yet worse than
  the live map. Existing v2 logs do NOT save the initial held-out medians,
  so they cannot establish that this actually occurred in an accepted trial.

  C: new_files/include/FullCalibrationOptimizerImpl.h:96 records the live
  summary; lines 104-110 choose that summary as the fixed-VI comparator.
  Lines 112-116 apply the same numerical gates to this different reference.
  The free-calibration mode retains v2's comparator unchanged (line 108).
  In PERIODIC-VI-HELDOUT logs, the inherited field name fixed_px denotes the
  PRE-SOLVE live-map median; final_px denotes the fixed-intrinsic BA result.
  In CALIBRATION-HELDOUT logs, fixed_px instead denotes the fixed-BA branch.

Earlier local BA may already have used the observations now held out of the
periodic graph. Therefore this is proposal validation against existing live
geometry, not genuinely unseen-data cross-validation. Zero accepted C trials
would mean no guarded fixed-VI updates reached live state; it would not show
that fixed-intrinsic global VI updates caused a trajectory failure. A future
calibration experiment could require improvement against both live and
fixed-BA references, but that guard is NOT implemented or tested here.
