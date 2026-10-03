# BabyFeatures SOS: completed Long experiment

Completed locally on **2026-10-04**; experiment identifier retains its launch date,
`20261003`. Branch: `experiments/BabyFeats`. This report covers **one full Long
replay**, not a benchmark-category average or leaderboard submission.

## Result and decision

**Positive for continuity, incomplete for accuracy.** The candidate retained one
map after startup and exported **35,619 / 35,842 poses (99.377825%)**. All 15 logged
SOS episodes returned to ordinary map tracking. Official local Score2D was
**43.72929195971007**, up **9.270391** from the saved native-v2 Long result, but
below the historical Long result of **49.881534**. A **1.618828 m displacement
between adjacent 50 ms poses** remains in the final trajectory.

Preserve the candidate as evidence that the SOS mechanism operates. **Do not
replace the strongest preserved baseline or claim that tracking never breaks.**
No Medium or Short replay was run for this candidate.

| Full Long result | Score2D | Retained maps | Native poses in scored map | CP recovered | Dense GT associated |
|---|---:|---:|---:|---:|---:|
| Saved frozen v2, native calibration | 34.458901 | 5 | 17,160 / 35,842 — 47.876793% | 15 / 27 | 3,163 / 6,118 |
| **BabyFeatures SOS** | **43.729292** | **1** | **35,619 / 35,842 — 99.377825%** | **27 / 27** | **6,078 / 6,118** |
| Historical continuity reference | 49.881534 | 1 | 35,772 / 35,842 — 99.804698% | 27 / 27 | 6,113 / 6,118 |

The historical reference starts from different camera calibration and is not a
matched ablation. The v2 and BabyFeatures rows share the native Long settings and
input caches, but their scored trajectories cover different portions of the route.

Only **4 / 27** BabyFeatures control points were within one metre. Associated
horizontal error was **1.895925 m median**, **2.933179 m RMSE**, and **4.708987 m
90th percentile**. The evaluator's saved CP-derived scale was **1.0081740367**.
These figures establish that full coverage has not solved the remaining geometric
accuracy problem. They do not isolate scale, calibration, or SOS as its cause.

## What was implemented

The frozen Short-80 v2 implementation remains the parent. Normal map association,
reprojection gates, calibration settings and visual-inertial optimization were
retained. New BabyFeature observations stay outside the estimator while ordinary
tracking succeeds.

1. **Passive tracking:** the same cached ALIKED-derived binary features are matched
   per camera using mutual Hamming matches, distance at most 70 and ratio below
   0.8. At most seven historical frames are retained. No feature extraction,
   optical-flow frontend, semantic mask or new camera model was introduced.
2. **SOS activation:** only failure of ordinary map tracking on an initialized,
   coast-armed inertial map activates the temporary visual-inertial solve. Its
   graph contains at most eight states: retained history plus the current frame.
3. **Temporary geometry:** native Fisheye624 projection/unprojection, the fork's
   general-ray DLT, `EdgeMono`, `EdgeInertial`, bias random walks, and temporary XYZ
   point vertices constrain pose, velocity and bias. Accepted normal states and
   the oldest rolling state anchor the solve. The temporary point variables are
   eliminated by the solver; they are not automatically persistent MapPoints.
4. **Continued map creation during SOS:** sufficiently verified temporary points
   can seed an ordinary temporal keyframe in the same existing map. This provides
   a way forward when the route does not revisit old landmarks. Native IMU links
   and point-slot ownership are preserved. Promotion initially records the actual
   current keyframe observation; temporary frame observations are not forged as
   keyframe observations.
5. **Return to normal:** ordinary map tracking must accept the returning frame.
   The retained window can then be refined with that returned map pose fixed.
   Revised retained SOS poses update native frame history before export. Baby
   factors become inactive again; passive correspondence caching continues.

Accepted SOS frames remain `RECENTLY_LOST` / coasting, rather than being labelled
ordinary visual success. Their acceptance refreshes the existing five-second
grace; pure IMU propagation does not. A ten-second episode limit prevents an
unbounded SOS extension. Early initialization failures retain the existing reset
policy. No independent Atlas origins are stitched together or aligned using GT.

## Acceptance and state protection

Temporary XYZ admission requires at least **0.25 degrees** of ray parallax.
Acceptance requires at least **15 distinct current tracks**, each supported by
at least two verified observations under the native normalized reprojection
bound **5.991**, with positive depth. Unsupported tracks are removed before the
second solve. Finite states, maximum active inertial chi-square **100**, rotation
change **0.5 rad**, position change **2 m**, velocity change **5 m/s**, and gross
speed/bias limits guard the transaction. Rejected solves do not commit caller
frame state. These are SOS checks; normal map gates were not loosened.

Promotion requires at least **50** eligible temporary points, each with at least
**three verified views** and **one degree of parallax**, after at least **three
consecutive accepted SOS solves** and **0.15 s** in SOS. Promotions are separated
by at least **0.5 s**, respect local-mapper availability, and recheck current
reprojection and feature-slot ownership. Ordinary next-frame map tracking, not
the promotion event itself, determines whether normal tracking has resumed.

The normal recent-loss return policy is inherited from v2. The observed return
support ranged from **17 to 182 mapped inliers**; this experiment does not repair
every previously identified recovery acceptance issue.

## Validation and execution

**Fourteen native numerical checks passed** against the actual candidate library.
They cover both fisheye cameras, metric geometry, fixed-anchor preservation,
temporary depth, visual correction of an erroneous IMU prediction, mismatched
correspondence rejection, transactional failure, no-visual/no-depth rejection,
IMU interval validation, two-view support, and endpoint-constrained interval
refinement. In the controlled correction case, position error changed from
**0.080 m to 0.00443047 m**. That case explicitly relaxed the synthetic IMU
covariance; it demonstrates the visual factors' operation, not measured production
drift correction.

The first fixture compilation used private Frame members and did not execute.
The fixture was corrected to construct the rig through the production public
constructor; the retained `native_solver_02` run passed all 14 checks. This was not
a second dataset experiment.

The full dataset replay processed all **35,842 inputs** and exited normally under
GDB, with **four CPUs**, existing temporal associations, calibrated IMU and matcher
diagnostics. Replay/runner duration was **2,339.72 s**, approximately 39 minutes.
The existing official scorer took **1.70 s**. Configuration and build fingerprints
matched before and after execution. Images and cached feature files were reused;
every expected image/feature path and feature-file length was validated, but the
entire feature-cache contents were not rehashed.

There was **one startup active-map reset**. The retained trajectory runs from
native **163.499313924 s to 1944.399294911 s**, spanning **1780.899981 s**. Its initial
223 native frames are absent from the final export. The full sequence begins at
152.349294899 s. The surviving map did not split afterward.

Coverage distinctions matter:

- The final export contains **35,537 normal-tracking poses plus 82 coasting poses**.
- The online trace logged **35,684 normal-tracking frames**, including 147 later
  discarded during startup, and 83 coasting frames, including one discarded.
- All **72 accepted BabyFeature updates** were explicitly state 3/coasting and all
  72 are present in the final scored map.
- The official scorer selected the largest map without GT and retained the full
  native/GT/control-point denominators. Missing poses were not interpolated.

The final observation audit found zero invalid observation indices, missing
backlinks, wrong reverse slots, or observation-count mismatches. There were 1,153
bad-point slots, compared with 1,116 across the saved v2 Long maps; this is not an
all-zero audit or proof that every map-lifecycle issue has disappeared.

## What the SOS logs establish

| Measurement | Completed result |
|---|---:|
| SOS episodes / ordinary-map returns | 15 / 15 |
| Accepted SOS updates | 72 |
| Rejected SOS attempts | 10, all `invalid_window` |
| Temporary-point promotions / points promoted | 13 / 1,648 |
| Accepted Baby inliers per update | 35–285 |
| Successful retained-window refinements on return | 13 / 15 |
| Maximum accepted SOS position correction | 0.000866003 m |
| Maximum accepted active inertial chi-square | 51.18247 |
| Periodic full-calibration trials / accepted | 29 / 4 |
| Loop / merge detection markers | 0 / 0 |

The first episode, **408.099–408.349 s**, demonstrates the intended mechanism:
mature-map support fell to two, four SOS updates retained 158–181 Baby inliers,
94 points were promoted, and the next ordinary update accepted 68 mapped inliers.
Map 0, initialization keyframe 29, and world version 3 remained unchanged. The
largest adjacent online displacement in that event was 3.54 cm.

The accepted position corrections were small throughout this run. The direct
evidence is sustained temporary visual support and a return through fresh map
landmarks, not large corrections to accumulated IMU drift. The two unsuccessful
return-window refinements had already lost their private cache to backend map
updates; ordinary map tracking still accepted the returns. All 4,628 `reset`
Baby log events were **cache resets for map updates**, not 4,628 Atlas map resets.

## Remaining jumps and attribution limits

| Native timestamp | Online adjacent body-pose step | Final exported adjacent step |
|---|---:|---:|
| 1607.199294886 s | 4.276225 m | **1.618828 m** |
| 1613.699294886 s | 0.924412 m | **0.173238 m** |

Both intervals are 50 ms and belong to the same retained map. The first occurs
inside SOS when a backend update clears the Baby cache; the second is a normal-map
return with 30 mapped inliers after another cache clear. Online snapshots record
the map at different optimization times, whereas final frame history follows its
optimized reference keyframes. `world_version` does not label every backend
deformation. Thus an online step alone is not proof of an equally large final
trajectory error; **the remaining 1.619 m final step is still a concrete defect**.
Five final adjacent steps exceed 10 m/s, at native 1605.349, 1605.849, 1606.349,
1607.199 and 1878.249 s.

One replay does not isolate the cause of the entire score/continuity change:

- Saved native v2 already left its first retained map at **213.299 s** and started
  another at **224.549 s**. BabyFeatures remains in map 0 before its first SOS at
  **408.099 s**. That earlier difference cannot be credited to active Baby factors.
  Passive matching adds work and changes asynchronous execution timing.
- Keeping a single populated map lets v2's own periodic refinement continue.
  This run accepted **4 / 29** calibration trials; saved native v2 accepted one.
  The final result therefore reflects the complete changed pipeline behavior,
  not an isolated short-track accuracy measurement.
- A recovered SOS episode is not automatically a map break proven to have been
  prevented: some brief gaps may also have recovered under the original grace.
- The eight-state window fixes its oldest state instead of carrying a complete
  marginalized uncertainty prior. It cannot retrospectively refine an arbitrarily
  long lost interval. Pure rotation, insufficient parallax, or total visual
  obstruction can still remove its temporary XYZ constraints.

The next discussion should separate **preserving this continuity gain** from
**repairing remaining pose/history handoff and whole-route geometric accuracy**.
This report does not authorize another sweep or silently promote the candidate.

## Visual evidence and what to inspect

Artifact root:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_long_20261003
```

Files relative to that root:

| File | What to inspect |
|---|---|
| `diagnostics.png` | Final scored trajectory uses the saved official CP Sim3. A perfect result would follow the full GT route with low residuals; continuity alone does not establish that. |
| `babyfeats_diagnostics/continuity.png` | One retained map covers the route after startup; SOS support and mature inliers are shown separately. Cache-reset counts must not be read as map deaths. |
| `babyfeats_diagnostics_live/first_sos_cameras.png` | Both native camera streams around the first bridge, preserving their original orientations. Nearby surfaces and strong shadow/exposure contrast coincide with lost map support. |
| `babyfeats_diagnostics_live/first_sos_motion.png` | Mature support falls, Baby support survives, and normal support returns without a large step in this first event. |
| `babyfeats_diagnostics/late_sos_audit.png` | The adverse late example: compare online and final pose steps. A perfect handoff would have no isolated spike; the green final trace still contains a 1.619 m spike. |
| `babyfeats_diagnostics/summary.json` | Exact episode counts and accepted-SOS export audit. |
| `babyfeats_diagnostics/late_sos_audit.json` | Exact online/final adjacent-step values and timestamps. |

The quick experiment deferred full Rerun rendering. The PNGs and raw exports above
are complete diagnostics, not a newly generated full camera/map `.rrd` recording.

## Reproduction and provenance

Run commands from:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3
```

The checked implementation at launch was commit
`2e989b48fabeafa6962ea3bfb4117935e26e8af2`, with no tracked working-tree diff. Inputs,
Docker image, frozen v2 objects and machine-local prerequisites are documented in
`docs/BASELINE_REPRODUCTION.md` and the candidate's `README.txt`. The candidate
builder adds the new solver translation unit; use it rather than an unextended
generic build command.

For a separately requested reproduction, choose unused output names:

```bash
python3 patches/orbslam3_lamaria_babyfeats_20261003/build_incremental.py \
  --out build/orbslam3_lamaria_babyfeats_REPRO --jobs 2

python3 patches/orbslam3_lamaria_babyfeats_20261003/tests/run_contract.py \
  --build build/orbslam3_lamaria_babyfeats_REPRO \
  --out build/orbslam3_lamaria_babyfeats_REPRO/tests/native_solver

/home/raghav/miniconda3/envs/lamaria/bin/python -B \
  pipeline/run/babyfeats_experiment.py \
  --build build/orbslam3_lamaria_babyfeats_REPRO \
  --out experiments/lamaria_babyfeats_long_REPRO
```

The last command runs, scores, and produces diagnostic PNGs. Optional `--render`
also creates `evaluation/run.rrd` using the already saved official evaluation
transform. Existing experiment outputs are refused. Original datasets, frozen
builds and the independent `INSV_STITCHING` project must remain untouched.

| Recorded item | SHA256 |
|---|---|
| Candidate library | `c3289348b0d0ac07caeb94d07524eef3992bc5ccdbb26fb0256513ed2b1c94c2` |
| Runner, unchanged from v2 | `a0efe826e73155b3f307882cb5176ec630eca9bd2147cba04cbcd395eec93f66` |
| Build manifest | `a424a28ccabe557d83138ca5e9c375222460693d1bf4cfee48ddbcd10ec54c26` |
| Native Long configuration | `6adf86a286d5cf4f9e037d19f1a323721965d0f5bd437b8f3eb9f4613139111c` |
| Combined vendor patch | `d0f5ed7b71087b3749196eacde3aed26994e3474f5a24d6dec0592ba4a7d566e` |
| Baby-only delta patch | `31f8acaac6a83677f95bd0fdab712fe0a771a372a5951b75d126c23a0713400c` |
| Native numerical fixture | `0984440e454b4891e29a8306f8da6832c433d9a0c0d7e924bc5aac3afb90aa00` |

Machine-readable source hashes, exact scores, compact coverage, test checks,
episode details, baseline references and artifact paths are in
`docs/BABYFEATURES_LONG_20261003.json`. Original evidence remains in the run's
`lamaria_score/scores.json`, `coverage.json`, `command.json`, and `run.log`, plus
the experiment's `provenance.json` and the native fixture's `results.json`.
