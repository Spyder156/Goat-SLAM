orbslam3_lamaria_valid_domain: isolated experimental overlay on orbslam3_lamaria_observations.
Vendor is read-only. See candidate_provenance.json for exact staging hashes and docs/LAMARIA_RELIABILITY.txt for experiment outcomes.
Build with pipeline/run/build_lamaria.py --patch-dir patches/orbslam3_lamaria_valid_domain --out build/orbslam3_lamaria_valid_domain --jobs 4.
This candidate is not the default.

2026-10-03 checkpoint:
Fixes invalid native fisheye-domain input and false inverse fallback bearings.
Per-camera domain metadata comes from the corresponding native VRS, with exact
intrinsics/size checks. Camera coefficients and matcher bounds are unchanged.
Filtering occurs before feature indexing for cached ALIKED and native ORB.
Both Short and Medium production contract suites pass 5/5; the sampled SDK-valid
periphery is preserved. See docs/LAMARIA_RELIABILITY.txt for scope and evidence.

Equal-input 120s Short/Medium diagnostics both completed and improved observed
SE3 trajectory RMSE. Full Short completed: Score2D55.349924 versus its direct
observation-build baseline9.627402 and history default39.656698. Coasting12
frames versus491 for the direct baseline. This is a single run, not repeatability
evidence. Metric scale remains wrong (CP estimate-to-GT scale0.876875513).

Full Medium FAILED with exit139 after native timestamp665.437393837, before
final trajectory/map export. No candidate full Medium score or Rerun exists.
Crash location/cause is unproven: no stack/core was captured. Do not promote
this candidate based on its positive Short result. Joint initialization is a
separate candidate and was NOT enabled in these domain-only runs.
