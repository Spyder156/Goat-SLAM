# BabyFeatures SOS: completed Long experiment

Completed locally on **2026-10-04**; identifier retains launch date `20261003`.
Branch: `experiments/BabyFeats`. **One full Long replay**, not a leaderboard submission.

## Result and decision

**Positive for continuity, incomplete for accuracy.** One map survived after startup,
with **35,619 / 35,842 poses (99.377825%)** and all 15 SOS episodes returning to normal
map tracking. Score2D was **43.72929195971007**, up **9.270391** from saved native v2,
but below historical Long **49.881534**. A **1.618828 m step over 50 ms** remains.
Preserve this candidate; **do not replace the strongest preserved baseline**.

| Full Long method | Score2D | Maps | Native poses in scored map | CP recovered | Dense GT associated |
|---|---:|---:|---:|---:|---:|
| Saved frozen v2, native calibration | 34.458901 | 5 | 17,160 / 35,842 — 47.88% | 15 / 27 | 3,163 / 6,118 |
| **BabyFeatures SOS** | **43.729292** | **1** | **35,619 / 35,842 — 99.38%** | **27 / 27** | **6,078 / 6,118** |
| Historical continuity reference | 49.881534 | 1 | 35,772 / 35,842 — 99.80% | 27 / 27 | 6,113 / 6,118 |

Historical Long uses different initial calibration and is not a matched ablation.
Only **4 / 27** BabyFeatures CPs were within one metre. Horizontal error was
**1.895925 m median**, **2.933179 m RMSE**, and **4.708987 m p90**; saved CP Sim3 scale
was **1.0081740367**. Full coverage has not solved whole-route geometric accuracy.
The v2/Baby scores cover different route portions, so their error statistics are
not a matched accuracy comparison. No Medium or Short candidate replay was run.

## Implementation and acceptance

Normal association, estimator factors, calibration settings and acceptance gates
remain v2's. A private passive cache tracks the existing ALIKED-derived binary
features per camera using mutual Hamming matches: distance ≤70, ratio <0.8.
Baby observations do not enter normal optimization. Only ordinary tracking failure
on an initialized, coast-armed map activates their temporary VI graph.

The graph has at most **eight states** and uses native Fisheye624 projection,
general-ray DLT, `EdgeMono`, `EdgeInertial`, bias random walks and temporary XYZ
points. It optimizes pose/velocity/bias, fixing mature-map states and the oldest
rolling state. Rejected solves do not commit caller state or modify calibration.

| SOS rule | Requirement |
|---|---|
| Temporary depth | ≥0.25° parallax; native triangulation and positive depth |
| Accepted visual support | ≥15 distinct current tracks, each with ≥2 verified views, normalized residual ≤5.991 |
| State protection | Finite state; active IMU chi-square ≤100; rotation change ≤0.5 rad, position change ≤2 m, velocity change ≤5 m/s; gross speed/bias limits |
| Bounded continuation | Accepted visual updates refresh five-second grace; episode limit ten seconds; pure IMU does not renew it |
| Landmark promotion | ≥50 points with ≥3 verified views and ≥1° parallax, ≥3 consecutive accepted solves, ≥0.15 s in SOS, ≥0.5 s between promotions |

Promotion seeds an ordinary temporal keyframe **inside the same existing map**,
with native IMU links and feature-slot checks. Temporary frame observations are
not forged as keyframe observations: promoted points initially have their actual
current-keyframe observation. Ordinary tracking must subsequently confirm return.
On return, the retained window can be refined with the returning pose fixed;
updated SOS poses are written into native history. Baby factors then switch off.
Accepted SOS frames remain explicitly `RECENTLY_LOST`/coasting. Atlas origins are
never joined by GT or invented transforms. Normal recovery acceptance is inherited.

## Validation and execution

**14 native numerical checks passed** against the production candidate library:
both fisheyes, metric depth/anchors, actual visual correction, transactional failure,
false/no/single-view matches, zero parallax, IMU intervals, two-view support and
return-endpoint refinement. A controlled wrong-IMU prediction improved from
**0.080 m to 0.00443047 m**, with explicitly relaxed synthetic IMU covariance;
this demonstrates factor operation, not production drift performance.
The initial fixture compile failed on private Frame access; the corrected public
constructor fixture `native_solver_02` passed. It was not a second dataset run.

The single replay processed all **35,842 inputs**, exited normally under GDB, and
used four CPUs, the same calibrated IMU, temporal associations and feature caches.
Runner duration: **2,339.72 s**; scoring: **1.70 s**. Build/config hashes matched
before and after. Every image/feature path and feature length was validated;
entire feature-cache contents were not rehashed. GT remained evaluation-only.

One startup active-map reset occurred. The retained map spans native
**163.499313924–1944.399294911 s (1780.899981 s)**. The first 223 native frames are
absent from final export; the input starts at 152.349294899 s. No later map split.
Final export comprises **35,537 normal poses + 82 coasting poses**. Online state 2
count was 35,684, including 147 later discarded startup frames; 83 online coast
frames included one discarded frame. All **72 accepted SOS frames** were state 3,
coasting, and present in the final scored map. No missing poses were interpolated.

Final graph audit: zero invalid indices, missing backlinks, wrong reverse slots or
observation-count mismatches. Bad-point slots remain: 1,153 versus saved v2's 1,116.

## Observed SOS behavior

| Measurement | Completed result |
|---|---:|
| SOS episodes / ordinary-map returns | 15 / 15 |
| Accepted / rejected SOS attempts | 72 / 10; all rejections `invalid_window` |
| Promotions / points promoted | 13 / 1,648 |
| Baby inliers / mapped inliers on return | 35–285 / 17–182 |
| Successful return-window refinements | 13 / 15; two caches had already been cleared |
| Maximum accepted SOS pose correction / active IMU chi-square | 0.000866003 m / 51.18247 |
| Periodic calibration trials / accepted | 29 / 4 |
| Loop / merge detection markers | 0 / 0 |

First episode **408.099–408.349 s**: mature support fell to two; four SOS updates
retained 158–181 Baby inliers; 94 points were promoted; next-frame normal tracking
accepted 68 mapped inliers. Same map/init/world version; largest online step 3.54 cm.
The direct benefit demonstrated is visual support and fresh-landmark handoff,
not large production IMU-drift correction. All 4,628 Baby `reset` events were
**private cache resets on map updates**, not Atlas resets.

## Remaining errors and causal limits

| Native time | Online adjacent step | Final adjacent step, 50 ms |
|---|---:|---:|
| 1607.199294886 s | 4.276225 m | **1.618828 m** |
| 1613.699294886 s | 0.924412 m | **0.173238 m** |

Both occur around backend updates, in the same retained map. Online snapshots and
final histories follow different optimization times; `world_version` does not
label every backend deformation. **The surviving 1.619 m final jump is a defect.**
Five final steps exceed 10 m/s: 1605.349, 1605.849, 1606.349, 1607.199 and 1878.249 s.

- Saved v2 left map 0 at **213.299 s**, before this run's first SOS at **408.099 s**.
  Improvement before SOS cannot be credited to active Baby factors. Passive work
  changes timing in a multithreaded pipeline; this batch has no unchanged Long repeat.
- A single map allows periodic VI/calibration to continue: **4/29** accepted trials
  versus saved native v2's one accepted update. Score changes reflect the complete pipeline.
- A brief recovered SOS is not automatically a map break proven to have been prevented.
- The fixed oldest state approximates a rolling prior; there is no full uncertainty
  marginalization. Only eight states are refined, not an arbitrarily long lost interval.
- Pure rotation/low parallax or complete obstruction cannot provide temporary XYZ
  constraints. Backend/calibration changes clear the cache. Early initialization
  and inherited recovery acceptance remain unchanged.

## Visual evidence

All paths below are relative to this absolute artifact root:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_long_20261003
```

| File | Inspect / ideal behavior |
|---|---|
| `diagnostics.png` | Final trajectory uses saved official CP Sim3; ideal full route closely follows GT. |
| `babyfeats_diagnostics/continuity.png` | One map after startup; mature and Baby support remain distinct. |
| `babyfeats_diagnostics_live/first_sos_cameras.png` | Both native camera streams, original orientation, around first bridge. |
| `babyfeats_diagnostics_live/first_sos_motion.png` | Positive example: support returns without an isolated pose jump. |
| `babyfeats_diagnostics/late_sos_audit.png` | Adverse example: compare online/final steps; ideal green final trace has no spike. |

Exact data accompany the PNGs in `summary.json` and `late_sos_audit.json`.
The quick harness deferred full `.rrd` rendering; it did not generate a new Rerun.

## Reproduction and hashes

Launch commit: `2e989b48fabeafa6962ea3bfb4117935e26e8af2`; tracked tree was clean.
Prerequisites: `docs/BASELINE_REPRODUCTION.md` and candidate `README.txt`.
For a separately requested repeat, run from the project root using unused names:

```bash
python3 patches/orbslam3_lamaria_babyfeats_20261003/build_incremental.py --out build/orbslam3_lamaria_babyfeats_REPRO --jobs 2
python3 patches/orbslam3_lamaria_babyfeats_20261003/tests/run_contract.py --build build/orbslam3_lamaria_babyfeats_REPRO --out build/orbslam3_lamaria_babyfeats_REPRO/tests/native_solver
/home/raghav/miniconda3/envs/lamaria/bin/python -B pipeline/run/babyfeats_experiment.py --build build/orbslam3_lamaria_babyfeats_REPRO --out experiments/lamaria_babyfeats_long_REPRO
```

Optional `--render` adds evaluation-aligned Rerun. Existing experiment outputs are
refused. Use the candidate builder, which includes the new solver translation unit.
Frozen builds, original inputs and independent `INSV_STITCHING` remain untouched.

| Item | SHA256 |
|---|---|
| Candidate library | `c3289348b0d0ac07caeb94d07524eef3992bc5ccdbb26fb0256513ed2b1c94c2` |
| Unchanged v2 runner | `a0efe826e73155b3f307882cb5176ec630eca9bd2147cba04cbcd395eec93f66` |
| Build manifest | `a424a28ccabe557d83138ca5e9c375222460693d1bf4cfee48ddbcd10ec54c26` |
| Native Long config | `6adf86a286d5cf4f9e037d19f1a323721965d0f5bd437b8f3eb9f4613139111c` |
| Combined patch | `d0f5ed7b71087b3749196eacde3aed26994e3474f5a24d6dec0592ba4a7d566e` |

`docs/BABYFEATURES_LONG_20261003.json` preserves exact results, source hashes, native
fixture evidence, episode details and original artifact paths. Original evidence
remains in the run's scores/coverage/command/log files and experiment provenance.
