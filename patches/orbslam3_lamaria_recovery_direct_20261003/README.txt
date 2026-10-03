Experiment A: direct binary-ALIKED recovery from frozen online-full v2

Purpose: test whether ORB-vocabulary partitions starve otherwise valid
post-loss map correspondences. This is an experimental candidate; the frozen
80.20 Short result belongs to v2, not this candidate. No trajectory result is
claimed here until the full Medium/Long replays finish.

Changes relative to frozen v2:
- RelocalizeWith directly matches existing cached256-bit ALIKED descriptors.
  These are seeded random-hyperplane binarizations of transient float128
  ALIKED descriptors. No float descriptors are present in these caches, and
  no feature re-extraction is performed.
- Exact OpenCV Hamming BF search spans all mapped descriptor rows in a
  candidate KF and each target camera. Existing TH_LOW70, nearest ratio0.75,
  and three-bin orientation filtering remain. Ambiguity is measured over
  the complete target lens, so this can be stricter than a vocabulary bucket.
- Lowest-distance deterministic assignment enforces both one image feature
  per landmark/camera and one landmark per image feature. Both source lenses
  can propose matches into both target lenses; geometric confirmation uses
  the selected lens's original native bearing model and camera transform.
- Existing candidate retrieval remains unchanged. Coast recovery considers
  its anchor and ten covisible KFs; the verifier explicitly caps work to11
  valid KFs. Each candidate and frame must have <=4096 features. Actual ALIKED
  inputs are <=1500 per lens, giving <=99million Hamming comparisons for11
  candidates. Oversized/malformed descriptors fail closed; no truncation.
- Existing MLPnP and pose-optimization acceptance is unchanged:15 total
  correspondences,10 per candidate camera, RANSAC(.99,10,300,6,.5,5.991),
  existing projection augmentation, and >=50 geometrically verified inliers.
  No pixel radius, chi-square, descriptor, or inlier threshold is loosened.
- Successful coast relatches remain provisional until local-map tracking
  confirms them. A pre-relatch IMU snapshot restores pose/velocity/bias and
  discards the rejected marginal prior if confirmation fails. The lost timer
  refreshes only after confirmation. This common safety correction is shared
  with experiment B; A is correspondence change plus this transaction fix.
- RECOVERY_DIAG reports raw BoW and direct counts before the15 gate, tested
  candidates, mapped rows, comparison budget, distance/ratio/ownership gates,
  hypotheses/solves, verified inliers, acceptance and elapsed time.
  RELATCH_TRANSACTION reports confirmation/rollback and unchanged grace time.

Build and provenance:
  python3 patches/orbslam3_lamaria_recovery_direct_20261003/build_incremental.py --jobs 1
The private builder verifies the frozen source/header/object/product/image
hashes and common original vendor manifest. Only Tracking.cc and ORBmatcher.cc
are recompiled using the exact original flags. No class data layout, vtable,
or existing method signature changes. ORBmatcher adds one nonvirtual method
and nested diagnostics type; the rollback inline is consumed only by the
recompiled Tracking.cc. The unchanged runner is copied byte-for-byte.
The cumulative geometry.patch + new_files also supports the ordinary full
pipeline/run/build_lamaria.py builder against the pinned original baseline.
Neither third_party nor the frozen v2 build is modified.

Contract:
  python3 patches/orbslam3_lamaria_recovery_direct_20261003/tests/run_contract.py \
    --build build/orbslam3_lamaria_recovery_direct_20261003 \
    --staging build/orbslam3_lamaria_recovery_direct_20261003/sources \
    --out build/orbslam3_lamaria_recovery_direct_20261003/tests/recovery_contract_v2
The fixture calls production SearchByDescriptor, MLPnPsolver, Fisheye624 and
Optimizer::PoseOptimization in the exact selected shared library. It covers
vocabulary-node starvation, both cameras and right-to-left pose conversion,
wrong-geometry rejection, exact Hamming equivalence, ratio/distance gates,
feature/landmark ownership, malformed input/work bounds, and native state
rollback plus grace-timer commit. It benchmarks one full-size candidate.
Synthetic contracts validate implementation mechanics, not sequence quality.

Authorized full replay harness (root coordinates the two global slots):
  /home/raghav/miniconda3/envs/lamaria/bin/python pipeline/run/continuity_experiment.py \
    --variant recovery_direct --sequence both
Expected outputs:
  experiments/lamaria_continuity_batch_20261003/recovery_direct_medium
  experiments/lamaria_continuity_batch_20261003/recovery_direct_long

Visual expectation if this helps: a verified recovery returns to the same
map after a dropout, with no pose jump from a rejected hypothesis and fewer
map epochs. The scored map should cover more of the real route. Good local
geometry alone or additional raw descriptor matches is not sufficient.

Completed native contract:20 checks PASS,0 failures. One3000-feature
candidate:65.4519ms /9million exact Hamming comparisons on one CPU.
Results:build/orbslam3_lamaria_recovery_direct_20261003/tests/recovery_contract_v2/results.json
Library SHA256:a9a93d595404894566a1742cda1194b9097429ecad05d54e766f55e5bd9c229e
The first contract fixture compile required const/reference setup corrections;
no estimator source changed in response. Final native checks all pass.
