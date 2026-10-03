# Roadmap: a reliable, accurate LaMAria visual-inertial SLAM system

Updated: 2026-10-03. This is the research and implementation plan, not a claim that every proposed feature exists. Read `GOAL.md` first for the objective and `EXPERIMENTS.md` for the experiment ledger. Paths below are relative to `/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3` unless an absolute path is given.

## 1. Direction and the decisions the evidence now supports

The goal is to outperform the Meta/Aria reference on comparable LaMAria evaluation, with a system that produces accurate, connected, metric trajectories and useful maps across short, medium, long, and difficult sequences. A visually good route, a lower reprojection cost, or one excellent sequence is not sufficient. We need coverage, repeatability, accurate geometry, and a verified score under the same protocol.

Our strongest observed result is **80.20 local Score2D on the complete Short `sequence_1_19`**, from periodic online VI bundle adjustment with bounded refinement of all native camera intrinsics. The fixed-camera reference scored **67.19**. This is a substantial local improvement, but the successful online run still contains brief pose jumps, and a subsequent rollback variant split into three maps. The result is promising; the system is not yet demonstrably robust.

The first full transfers of the frozen v2 executable are now measured: **Medium 62.76 with three independent maps**, and **Long 34.46 with five maps**. The largest scored maps cover 61.2% and 51.7% of dense GT timestamps, respectively. Medium's missing CPs cost 27.78 score points; Long's cost 44.44. V2 only attempts periodic full calibration while one populated map exists, so fragmentation also stops that refinement. These results make continuity and valid map reconnection concrete priorities. The corrected Long trial used native VRS lenses after inherited two-focal COLMAP parameters failed the native-model contract; it is not a pure calibration-only comparison with the older Long baseline. Exact records are E35–E36 in `EXPERIMENTS.md`.

Short's remaining 19.80 points have a different explanation: all 14 CPs are recovered, but median horizontal CP error is 39.34 cm and none meets the 5 cm full-credit threshold. Preserve this separation between sub-metre precision on Short and missing coherent map coverage on the transferred runs.

The official reproduced Aria SLAM category scores on the leaderboard checked on 2026-10-03 are Short 90.7, Medium 78.5, Long 70.9, Lowlight 84.2, and Moving 55.0. These are category results, whereas our 80.20 is a local result on one Additional Set sequence. We must not subtract them and announce a leaderboard gap or a win. A like-for-like suite comes later. [Official LaMAria leaderboard](https://lamaria.ethz.ch/leaderboard).

### Latest decision: BabyFeatures SOS implemented and tested on Long

E39 completed the user's requested SOS experiment on `experiments/BabyFeats`.
Short-lived tracks are maintained passively and only enter estimation after
ordinary tracking fails. One retained map survives all 15 SOS episodes after
a startup reset, covering 99.38% of native inputs. Score2D is 43.729292 versus
native v2 34.458901. This meets the immediate continuity objective in one replay,
but only four of 27 recovered CPs are within 1 m and a 1.619 m final step remains.
Historical Long 49.881534 stays preserved; Short 80.199161 is untouched.

Discuss the full-route and SOS diagnostics before another experiment. Candidate
next work is the backend state/history discontinuity exposed by the surviving
map, plus an authorized repeat/Medium transfer to establish reliability. Keep
the normal estimator protected. Do not silently broaden this into threshold
relaxation, knot tuning, or wholesale frontend changes. See
`docs/BABYFEATURES_LONG_20261003.md`; the following batch findings are historical.

### Earlier continuity batch and its recovery finding

E37–E38 are now complete: six full candidate replays and one unchanged Medium
control. Direct recovery, landmark memory and map-local fixed-intrinsic VI are
implemented as separate experimental packages. **Promote none.** Memory scored
45.87 Medium / 42.90 Long; direct recovery was unscorable Medium / 38.94 Long;
fixed VI was unscorable Medium / 37.63 Long and accepted no periodic correction.
The unchanged Medium repeat scored 63.09 and reproduced the first dropout
location. The retained Long 49.88 remains stronger than all new Long arms.

Next priority is an explicit recovery confirmation and VI handoff policy. The
current coast path can go from >=50 visual inliers to 11–21 after local VI and
still accept. Verify recovered support and state consistency over a recovery
window; do not merely change >10 to >=15, blindly trust visual poses, replace a
temporal IMU anchor with a retrieved keyframe, or join independent maps without
estimated constraints. Existing VI already optimizes pose, velocity and bias.
Keep high-quality temporal/appearance matching as a separate improvement, and
retain repeatability controls before claiming score gains. Knot refinement
remains deferred. The user requested discussion at this major crossroads.

Evidence and visual expectations: `docs/CONTINUITY_EXPERIMENTS_20261003.md`.
This update supersedes earlier wording below that calls these three candidates
unimplemented; research directions without recorded results remain proposals.

### What the latest experiments actually established

| Change / experiment | Local Short Score2D | What it supports | What it does not establish |
|---|---:|---|---|
| History/continuity reference | 39.66 | Useful historical reference, with incomplete selected-map coverage | Current best performance |
| Dual-camera observation bookkeeping repair | 9.63 | Structural observation contracts can be corrected while whole-system accuracy regresses | That more factors automatically improve geometry |
| Native fisheye validity repair | 55.35 | Correct validity handling materially improved continuity on this Short | That metric scale was fixed, or Medium was safe |
| Fixed-camera, 12-second joint initialization candidate | 67.19 | Best fixed-camera Short reference in this campaign | Perfect metric scale; its CP alignment scale was 0.8861 |
| One-time two-radial-term online calibration | 28.95 | Near-unit global scale and better early geometry can coexist with severe later deformation | Successful full-route calibration |
| Offline common-SIFT-graph visual rig BA | 64.71 | This tested batch refinement reduced objective but worsened score | That all visual BA must fail |
| Offline common-SIFT-graph global VI-BA, fixed lenses | 67.19 | This tested batch refinement left score and metric stretch essentially unchanged | That converged, differently structured VI refinement cannot help |
| Offline common-SIFT-graph global VI-BA, all intrinsics free within priors | 67.18 | This tested batch calibration did not improve score | That camera calibration cannot improve the online estimator |
| Periodic online full VI-BA + all-intrinsic calibration, v2 | **80.20** | Full-route score and raw metric accuracy both improved substantially | That the gain came only from intrinsics, or that the result repeats reliably |
| Online rollback-boundary repair, v3 | 42.59 | The rejected-state restoration contract worked, but continuity regressed | A safe candidate to promote or a clean one-variable causal result |

The three offline arms used the same original SIFT graph and each started independently from the 67.19 trajectory. They exhausted their 30-attempt budgets without formal convergence. The online arm re-extracted no SIFT graph; it replayed the original SLAM inputs with ALIKED observations and recurring past-keyframe optimization. Online and offline are therefore different experiments, not an isolated comparison of when identical BA ran.

### Immediate priority order

1. Preserve the exact 80.20 binary, inputs, configurations, model, score, and verified Rerun.
2. Explain and reproduce its improvement with a deliberately small, controlled comparison: periodic VI-BA with fixed intrinsics versus the existing bounded-calibration version.
3. Repair continuity as a coherent state-update operation, using the already captured failures. Do not restore a physically inconsistent state just because one lucky run scored better.
4. Validate the resulting candidate on full Short, Medium, and Long, including repeated runs where scheduling variability changes the result.
5. Integrate MegaLoc retrieval with native-camera geometric verification and a metric-consistent loop backend, primarily on `sequence_2_11`.
6. Only then spend the larger research budget on learned depth, generalized multi-view tracking, more calibration freedoms, and redesigned optimization.

This order is a proposal for the next campaign. Documentation work does not authorize silently restarting a stopped run. At a major implementation, failure, or choice between competing approaches, finish the current evidence package, present it, and discuss the next step with the user.

## 2. Measurement, scoring, and reusable test sequences

### What exists

The reusable registry is `configs/orbslam3_lamaria/suite_20261002.json`; the planner is `pipeline/run/lamaria_suite.py`. Prepared plans are at:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_suite_20261002
```

| Role | Sequence | Purpose |
|---|---|---|
| Short | `sequence_1_19` | Initialization, metric accuracy, doorway/bridge failures, and the 67.19/80.20 comparison |
| Medium | `sequence_2_11` | Full-route continuity and the known loop-closure opportunity |
| Long | `sequence_3_17` | Accumulated drift, long-range consistency, coasting, runtime, and map retention |

There are startup, loss-context, loss-prefix, and full profiles for each sequence. Startup is the first 120 seconds from the original start. A loss-context crop is a **cold start with a different map history**. Only the loss-prefix preserves all earlier inputs. A complete loop experiment requires the full relevant route. Never report a crop's good behavior as proof that the original failure is fixed.

### Scoring rules to retain

- Estimation outputs raw world-from-body poses. Ground truth does not enter tracking, calibration, matching, loop proposals, initialization, or BA.
- The official local evaluation derives a control-point Sim3 after estimation. It can absorb one uniform scale discrepancy. It cannot remove spatially varying stretch, a bad turn, a map break, or a pose jump.
- Score2D and dense trajectory coverage are distinct. Score2D is derived from control-point errors; missing control points contribute zero. Missing GT associations remain failures in the full PoseRecall denominator. Always report both metrics.
- Report control points triangulated **and** control points within 1 m. Fourteen recovered control points do not imply fourteen accurate control points.
- Keep the original input denominator. On the current Short there are 18,352 native timestamps, 18,351 inputs supported by the existing IMU interval, and 18,343 saved poses in the complete successful runs. Startup and unsupported-tail poses remain absent.
- An all-atlas export is useful evidence, but independently initialized maps cannot become one submitted route without measured geometric registration. Never use GT to stitch them.
- Rigid SE3 alignment is a separate metric-scale diagnostic; a dense-GT Sim3 is another diagnostic. Label both. Neither is a new SLAM solution or the official CP fit.

### Test design to add

Maintain a frozen development set and a separate held-out set, including different recordings/devices where available. Keep benchmark test-set feedback out of tuning. Use matched input manifests, immutable binaries, independent output directories, and complete parameter diffs. Pair short diagnostic windows with a direct route-level decision: a test exists to decide what implementation or run to do next.

For high-impact candidates, initially repeat the complete Short enough to expose observed scheduling sensitivity; three repeats is a practical first check, not a statistical guarantee. Report every attempt, the median, worst outcome, map count, and coverage. Expand repetition if variation is comparable to the apparent improvement. A deterministic keyframe/optimization replay can isolate numerical effects, but it must not replace eventual real scheduling tests.

**Success:** improvements survive full-route scoring and held-out sequences, with no hidden coverage loss. **Stop:** a change repeatedly improves only a small selected interval or reprojection cost while the full score/coverage regresses. Preserve the result and revise the hypothesis instead of launching near-identical trials indefinitely.

## 3. Coordinate conventions and native fisheye geometry

### Hypothesis and established repairs

The user's original concern was structural: persistent 2D matches can coexist with incorrect 3D bearings, rig transforms, or landmark coordinates. That was the right class of issue to investigate. Real convention, validity, indexing, and observation-lifecycle problems have been found during this campaign. It would be equally wrong now to assume every remaining failure is another quaternion-order error without evidence.

Current geometry must stay native Fisheye624. Its independent camera parameters are one shared focal, two principal-point values, six radial coefficients, two tangential coefficients, and four thin-prism coefficients: **15 per camera**. COLMAP stores 16 values because it has separate `fx` and `fy`; our native contract ties them. Native pixels and principal points are shifted by +0.5 pixel at the COLMAP boundary, and converted back exactly on export.

Relevant verified conventions in the current batch pipeline are `IMU.T_b_c1 = body_from_cam0`, `Rig.T_c0_c1 = cam0_from_cam1`, C++ quaternion memory in xyzw, COLMAP text quaternion order wxyz, and exported world-from-body poses in xyzw. The physical rig baseline in the current checked configuration is approximately **0.1375764865 m**. Do not copy an older rounded baseline from conversation into a new configuration.

### Required geometry invariants

1. Production `unproject` returns the bearing representation required by every consumer; normalization is explicit. Exact pixel round trips alone do not test depth, angular thresholds, or triangulation semantics.
2. Production project/unproject validity agrees across frontend, triangulation, local BA, global BA, calibration, and export, including the periphery and thin-prism inverse.
3. A display rotation never changes the measurement coordinates unless the camera model and keypoints undergo the same known transform. A rotated cam0 image is not by itself proof of a wrong extrinsic.
4. Map rescaling preserves physical rig and camera-to-IMU lever arms. Points, poses, relative keyframe translations, and exported pose chains must remain consistent.
5. Rig cameras retain their own origins. A generalized bearing is an origin plus a direction, not merely a central-camera unit ray.
6. A transform or calibration commit updates all dependent frame/keyframe matrices, stereo depth caches, landmark geometry summaries, and priors, or reconstructs them before use.

The completed online v2/v3 cache audit did not find stale intrinsics in active point tracking: shared camera objects, stereo caches, frame/keyframe matrices, and active projections were refreshed. That audit is evidence against one specific explanation, not a guarantee that all future calibration edits are safe.

### The user's 10-frame geometry experiment

This was already run using linked production geometry, on three windows with measured GT poses used only for this diagnostic: 241 tracks produced finite positive-depth estimates in both five-frame halves; held-out median error was 1.025 px and p90 was 2.159 px. The longest window retained no cam0 tracks, so the result did not demonstrate equal support across both cameras or every image region. This is useful evidence against a gross visual convention error conditional on correct poses, not proof that online inertial states, map ownership, or calibration everywhere are correct. These GT poses were not supplied to SLAM. See `EXPERIMENTS.md`, E05, and `/home/raghav/workspace/MeckaAI/lamaria_audit_20261002_tenframe/README.txt`.

Retain this as a focused acceptance test when triangulation or geometry changes. Extend support to static features across ten native frames in each camera and across the stereo overlap. Split observations into first five and last five, use the **actual production triangulator and camera model**, and compare resulting world points and held-out reprojections. Repeat with central and peripheral features, different depths, and appreciable parallax. Plot both 3D estimates and per-image residual arrows.

Avoid two traps: nearly pure rotation makes 3D depth unobservable even if pixels are exact; using identical optimized poses/observations everywhere can make a self-consistent wrong calibration look good. Separate code-contract tests from physical calibration tests, and add a held-out view or independent geometric constraint where possible. Do not claim this exact experiment has passed unless its artifact and production-code provenance are in the ledger.

**Success:** subpixel numerical consistency where exact synthetic geometry warrants it, finite well-conditioned real estimates, and no systematic outer-field or cross-camera disagreement. **Stop:** changing thresholds before the projection, ray, and unit contracts pass.

## 4. Metric initialization: stereo, IMU, and learned depth

### Why this remains important

The 67.19 trajectory had good shape but a substantial metric stretch. The online 80.20 run reduced rigidly aligned 3D RMSE from approximately 16.53 m to 2.48 m and required a CP scale correction of 1.0194 rather than 0.8861. This is evidence of a real metric improvement, not merely a nicer Sim3 display. It does not prove initialization is now optimal or solely responsible.

ORB-SLAM3 is a tightly integrated visual-inertial system; the IMU is not merely an optional late batch correction. Its initialization, tracking prediction, and inertial optimization interact. Our fork's actual mode, initialization state, and factor support must be logged rather than inferred from its name. [ORB-SLAM3 paper](https://arxiv.org/abs/2007.11898).

### Already tried

- The scalar metric proposal corrected the multiplicative scale Jacobian and preservation of the camera-to-IMU lever arm during rescaling. Synthetic contracts passed. In real Short startup, proposals were repeatedly rejected by stereo consistency; a correct scalar solve was not sufficient.
- The copied joint proposal refined poses, landmarks, velocities, gravity, and biases before a commit. It enabled Short initialization and early VI stages, but a 120-second comparison still showed roughly 16.4% oversizing. Functional initialization and correct scale are different outcomes.
- A 12-second startup window with paused input consumption, joint geometry, and fixed camera parameters produced the 67.19 full-route reference. Frames were delayed, not discarded.
- One-time radial calibration improved early metric geometry and near-unit global scale, but the full score fell to 28.95 after serious route deformation.

Sources: `patches/orbslam3_lamaria_metric_init/README.txt`, `patches/orbslam3_lamaria_joint_init/README.txt`, `docs/LAMARIA_RELIABILITY.txt`.

### Basic next implementation ideas

**A. Native metric stereo initialization.** Build a multi-frame generalized rig graph with real cross-camera tracks, temporal parallax, and fixed metric extrinsics. Require angular diversity and depth conditioning, not only a large number of points. Wide fisheyes may have limited useful overlap; a 0.138 m baseline provides weak depth for distant facades. If stereo has weak metric information, admit that uncertainty and wait for motion rather than force an apparently precise scale.

**B. Joint stereo-inertial startup.** Estimate poses, points, velocity, gravity direction, and biases together with realistic IMU covariance. Release metric scale only where the formulation requires it; do not duplicate a scale gauge already fixed by a physical stereo graph. Test observability after marginalizing nuisance states. A scale estimate can look well-conditioned if poses or landmarks are incorrectly treated as exact.

**C. Adaptive initialization duration.** Replace “exactly N seconds is enough” with a minimum evidence requirement plus a maximum waiting policy. Rotation helps bias/gravity calibration; translational excitation helps metric scale. Keep startup data until an accepted state can initialize subsequent tracking consistently. Evaluate startup delay separately from accuracy and full coverage.

**D. Early scale refinement without a discontinuity.** Revisit metric states after the first useful motion and again when adequate stereo support appears. Commit corrected poses, points, velocities, parent transforms, and marginal priors as one transaction. An accepted initializer must not leave the tracking frame in an older gauge.

### Depth Anything 3 and other learned geometry

DA3 has any-view, monocular, and metric-depth variants and supports pose-conditioned inference. Its documented extrinsics are world-to-camera; its input intrinsics are pinhole matrices. These are integration contracts, not proof of native F624 support or accurate metric depth on Aria grayscale. [Official DA3 API](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/docs/API.md).

Proposed use: generate calibrated virtual pinhole views from native fisheye rays, obtain depth/confidence, and transform the prediction back into the original camera ray representation. Verify whether the selected depth output means optical-axis depth or ray range. Fit or validate metric scale against reliable stereo and inertial evidence; do not use GT or assume a learned metric prediction is exact. Use robust, uncertainty-weighted depth factors for initialization, then reduce their influence as real multi-view evidence accumulates.

Expected benefit: useful depth seeds in low-parallax startup or a texture-poor scene. Failure modes: confident learned shape errors, domain shift to grayscale/fisheye, incorrect view conversion, moving objects, and a learned scale prior fighting real inertial measurements. Compare learned-depth assistance against the same stereo-inertial initializer. If it only makes the initial cloud prettier, it has not passed.

For a larger research effort, test VGGT-style multi-view geometry as a proposal generator. It predicts cameras, depth, points, and tracks; these predictions still need native-ray conversion, metric anchoring, and uncertainty checks. It is a candidate for initialization and difficult local reconstruction, not a replacement for measured metric consistency. [Official VGGT implementation](https://github.com/facebookresearch/vggt).

**Success:** consistent scale from the beginning, reduced rigid-SE3 error, healthy early inlier support, and repeatable full-route improvement. **Stop:** an initializer recovers only by allowing biases or lens parameters to absorb an unexplained geometry error.

## 5. Landmark persistence, feature matching, and medium-range tracking

### The exact problem to solve

At one recorded failed local-map update, only **13 associations entered optimization** even though hundreds of features were detected in each image. The required tracking floor was 15, so that optimizer could not possibly rescue the frame. The bottleneck existed before optimizer outlier rejection. However, this does not uniquely identify calibration: visibility selection, poor pose prediction, stale geometry, missing observations, descriptor failure, culling, or weak scene coverage can all starve the local map.

Green dots are a visualization of a particular association stage. In an offline final-map reconstruction they are final batch associations, not historical online inliers. Increasing their number is useful only if more correct, geometrically informative landmarks survive into accepted optimization.

### Already tried and learned

The dual-camera bookkeeping repair fixed missing per-camera observation records, idempotent registration, complementary-camera merges, and reverse-slot consistency. Live invariant audits became clean. Yet full Short score fell 39.66 to 9.63 and Medium 55.75 to 39.92 in that candidate. Correct bookkeeping exposed or changed the geometry being optimized; it was not a sufficient accuracy repair. Later valid-domain handling restored substantial Short continuity. Keep both lessons.

### Basic next work

1. **Trace the actual association funnel.** For each camera and angular sector: detected features → nearby landmarks → valid visible projections → candidates inside search bounds → descriptor/track matches → unique assignments → optimizer inliers → accepted frame. Record the exact production pixel/radius, local and pooled index, landmark ID, and rejection reason. Final-map reprojection is not a substitute for these online quantities.
2. **Retain one-to-one image ownership.** Multiple landmarks must not claim the same feature. Same-landmark measurements in both cameras are legitimate separate factors. Distinct SIFT orientations at nearly the same pixel require explicit correlation/weighting policy; their mere existence is not an indexing error.
3. **Replenish before starvation.** Trigger keyframes and triangulation using coverage, parallax, landmark age, camera balance, and survival probability. A fixed count threshold can miss that all surviving landmarks lie in one small area or on distant buildings.
4. **Maintain a medium-range landmark pool.** Keep geometrically useful older keyframes and tracks beyond immediate neighbors. Reassociate after viewpoint changes, with descriptor refresh or multiple appearance exemplars where measured useful. Evaluate 1–10 s, 10–60 s, and longer track survival separately.
5. **Use covariance-aware search.** Propagate pose/landmark uncertainty to predicted image uncertainty and use normalized gates. This is more principled than globally loosening a pixel threshold. Cap broad searches, enforce uniqueness, and report whether increased recall also increases false associations.
6. **Handle learned-feature scale honestly.** Existing ALIKED imports do not necessarily carry ORB-like scale-octave semantics. Audit remaining scale prediction, triangulation consistency, and fusion gates before interpreting an octave as a physical scale measurement.
7. **Separate difficult imagery from bad calibration.** Hands, moving people, water, blank walls, blur, and extreme tilt remove useful static constraints. Mask or downweight them based on evidence. Do not demand green static landmarks on a hand covering the camera.

### Advanced directions

- Track-assisted matching: combine a learned point tracker with descriptor re-identification and native-camera geometric validation. CoTracker is a concrete candidate for persistent 2D tracks; occlusion confidence and long-lived 3D consistency still need independent checks. [Official CoTracker implementation](https://github.com/facebookresearch/co-tracker).
- Add high-quality wider-baseline correspondences selectively, using LightGlue-compatible sparse features or dense matching for difficult keyframe pairs. LightGlue adapts its computation to matching difficulty; that is attractive for selective recovery or loop verification. [Official LightGlue implementation](https://github.com/cvg/LightGlue).
- Try RoMa-style dense correspondences for otherwise disconnected useful regions, then spatially subsample and geometrically verify them. Dense matches are correlated evidence, not thousands of independent precise constraints. [Official RoMa implementation](https://github.com/Parskatt/RoMa).
- Hybrid direct patches and sparse landmarks may improve tracking where descriptors are brittle. Add photometric/exposure models and native warp geometry; test against changes in lighting and blur before replacing the frontend.
- Investigate lines and planes for low-texture architecture, with switchable or robust factors. Do not impose a universal Manhattan world on outdoor scenes and stairs.

**Success:** better medium-range track survival, wider spatial/angular support, fewer weak frames, lower local relative error, and improved full-route coverage/score. **Stop:** more features or inliers accompanied by larger pose innovations, duplicated assignments, or worse trajectory accuracy.

## 6. Local visual BA and local VI-BA

### Why this is its own workstream

Short-range matching can look excellent while local estimation repeatedly creates small scale, orientation, or depth errors. Those errors then corrupt the map against which future frames are matched. Medium-range accuracy depends on maintaining a well-conditioned connected local graph, not just adding a loop closure at the end.

ORB-SLAM3 already contains local and inertial optimization. The new offline solver is not evidence that the online local window is optimal. Do not add a second optimizer that double-counts the same observations or fights a stale marginal prior.

### Proposed comparisons

**L1: local window support.** Hold factors and calibration fixed; compare time-based/covisibility-based windows and inclusion of older geometrically useful anchors. Measure frame-to-frame and 10–60 s drift, solver conditioning, tracking latency, and residual distribution per camera. A larger window is valuable only if it adds information and still commits in time.

**L2: optical versus VI local refinement.** Compare on one captured factor graph from the same starting state, then confirm the better design in online replay. Fixed-rig visual BA can be metric if sufficiently informative cross-camera constraints exist. It is not inherently scale-destroying. VI-BA adds physical information, but wrong uncertainty, bias linearization, or constraints can prevent a useful correction.

**L3: uncertainty and robust losses.** Calibrate observation variance by camera, angle, feature type, scale, and blur using independent residual evidence. Compare normalized innovations against expectations. An outer-field systematic bias should first prompt a camera-model investigation, not simply a larger variance to hide it.

**L4: prior consistency.** When calibration or map gauge changes, rebuild or correctly transform the marginal prior. When a visual update is rejected, its information must not leak into the next inertial prior. Audit pose, velocity, bias, gravity, calibration, and map version together.

### Longer-term backend options

Square-root marginalization is a relevant numerical direction for sliding-window BA; delayed marginalization is relevant when late scale or calibration corrections invalidate earlier linearization points. These are design ideas to prototype against the present backend, not claims that swapping libraries automatically improves score. [Square-root marginalization project](https://cvg.cit.tum.de/research/vslam/rootvo), [DM-VIO project](https://cvg.cit.tum.de/research/vslam/dm-vio).

An incremental factor-graph backend such as iSAM2/robust incremental smoothing is another possible research branch if repeated full optimization becomes the bottleneck. Require factor-equivalence and state-export tests before comparing runtime or accuracy. [GTSAM nonlinear optimization documentation](https://borglab.github.io/gtsam/nonlinear/).

**Success:** lower local error and better accepted-frame support without artificial overconfidence, unsafe latency, or degraded full-route coverage. **Stop:** tuning local cost until it looks better while global metric or control-point error worsens.

## 7. Global visual BA, global VI-BA, and offline reconstruction

### What we actually built and tested

The native dual-camera COLMAP refinement pipeline is in `pipeline/vi_ba_lamaria/`. The common graph contains 18,343 poses, 36,686 camera images, 1,715,595 initial landmarks, and 26,564,121 observations. It contains cross-camera, temporal, and geometrically proposed revisit matches. There was no GT-based match proposal.

The three arms were visual rig BA, full native-IMU VI-BA with fixed cameras, and VI-BA with all camera intrinsics under factory-centered priors and bounded updates. Results were **64.71, 67.19, and 67.18**, respectively, against **67.19**. They were usable iteration-limited results, not converged optima. Their complete scores are valid measurements of those outputs; their failure to improve is not a proof that no global refinement can help.

The fixed-intrinsic VI arm included every native IMU interval, per-frame velocities/biases and random walks, and a fixed-norm gravity direction. Exact final-bias reintegration changed its IMU objective negligibly, so stale final preintegration was not a useful explanation for that arm's unchanged score.

### What to try next, in order

1. **Explain information, not just size.** Inspect spatial support, triangulation angles, track correlations, cross-camera leverage, and factor residuals. Twenty-six million observations may heavily repeat the same weak geometry.
2. **Convergence study with checkpoints.** On representative connected graphs, compare accepted-step progress, normalized gradient, linear-solver residual, and held-out geometry. Then choose a resource budget for a full solve. Do not spend another several-hour run merely changing a solver flag without a predicted benefit.
3. **Better observations on the same geometry.** Refine keypoint localization, add verified wider-baseline tracks, remove truly inconsistent tracks using a documented rule, and compare against the same starting state. Track cleaning is a different experiment from changing the optimizer.
4. **Use original SLAM observations as a controlled alternative.** Compare batch optimization of the captured native online graph with the SIFT graph. This helps distinguish estimator-state problems from frontend correspondence differences.
5. **Hierarchical global refinement.** Optimize a well-connected keyframe graph and landmarks, then recover all frame poses with proper inertial and visual factors. Preserve uncertainty and evaluate whether compression changes the answer. Do not interpolate missing outputs and call it full optimization.
6. **Robust loop-aware refinement.** Add geometrically validated loop factors and shared tracks, then jointly relinearize visual and inertial states. An optical correction followed by a VI pass can undo itself when the two passes use inconsistent information or gauges; “two passes” alone is not a principled objective.
7. **Measure metric versus score gains separately.** A refinement can correct scale without improving CP Score2D because the evaluator already admits one global Sim3. That correction is still useful physically, but the score needs improved relative geometry and coverage.

**Success:** higher full-denominator score, lower residual trajectory deformation, and consistent metric states, with an auditable solve budget. **Stop:** repeated cost reductions without score/coverage gains; change the information or model, not only the iteration count.

## 8. MegaLoc loop closure and relocalization

### What exists and what needs porting

The Mecka Basalt implementation exists at:

```text
/home/raghav/workspace/MeckaAI/MECKA_GITHUB/mecka-basalt/scripts/lamaria/megaloc_wrapper.py
/home/raghav/workspace/MeckaAI/MECKA_GITHUB/mecka-basalt/scripts/lamaria/loop_closure.py
```

It uses both cameras in a pooled image gallery, MegaLoc retrieval, temporal exclusion, neighboring-keyframe consistency, DeDoDe/LightGlue verification, temporal triangulation, bidirectional pose consistency, and a drift-budget check. This is reusable experience, not a drop-in native-F624 loop backend. The inspected script explicitly assumes **pinhole** images and intrinsics for geometric verification. Its image orientation and feature coordinate handling must be ported deliberately.

MegaLoc is a retrieval/place-recognition model. It proposes places that may match; it does not by itself validate a metric relative pose or perform VI loop correction. [MegaLoc paper](https://arxiv.org/abs/2502.17237), [official implementation](https://github.com/gmberton/MegaLoc).

### Proposed integration stages

**C1: retrieval only.** Cache descriptors for both camera streams with exact timestamp, keyframe, camera, and preprocessing provenance. Retrieve same- and cross-camera revisits. Show top candidates on a trajectory and the paired images. Use `sequence_2_11` in full; a prefix that never reaches the return visit cannot test closure.

**C2: native geometric verification.** Use native F624 bearings, real rig origins, and production pose/triangulation conventions. Generalized PnP or a correctly constructed virtual-pinhole verification route are candidates. Require sufficient spatial support, temporal consistency, and bidirectional agreement. Repetitive facades and near-duplicate temporal neighbors are deliberate negative controls.

**C3: metric graph correction.** For an initialized VI map, keep scale fixed where metric observability justifies it and preserve gravity consistency. Use a gravity-aligned pose correction where appropriate, or a full formulation that retains inertial constraints. Sim3 may be appropriate for joining an uninitialized/unscaled component, but it must not silently stretch an already metric physical rig.

**C4: joint refinement.** After accepting a loop, jointly refine the affected visual geometry and inertial states, then propagate corrections coherently to live tracking and historical export chains. Reintegrate/relinearize as required. Log which measurements support the correction and whether it worsens local visual or IMU residuals.

**C5: recovery and map merging.** Reuse retrieval for relocalization after tracking loss, but require measured cross-map registration. A disconnected atlas is not a full trajectory. Keep uncertain loop proposals separate until their geometric evidence is sufficient.

### Advanced options and failure modes

Use robust or switchable loop constraints and consistency checks across cycles to limit false-positive damage. Broaden retrieval with virtual views or multi-camera aggregation if native fisheye appearance is difficult. Avoid an overly strict correction gate based solely on the drifting current trajectory: it can reject the true loop we need. Conversely, a high retrieval score must never override inconsistent geometry.

**Success:** correct loops are recovered and improve the full route while preserving metric scale and local alignment. **Stop:** an impressive top-view closure deforms other segments or reduces score. The user has already observed that optical and VI passes can disagree; test the factors and correction propagation that produce that disagreement.

**Required visualizations:** top view before/after, loop edges colored by accepted/rejected state, matched image pairs, per-edge residual/support, scale and gravity changes, and local camera overlays around the correction.

## 9. Online intrinsics: test the peripheral-error hypothesis properly

### The hypothesis

The user proposes that landmarks tracked near the center die as their image moves toward the fisheye periphery because the supplied lens calibration generates inaccurate bearing vectors. This is plausible and testable. Pixel round trips and a center-only fit cannot disprove it. But tracking starvation also occurred with occlusion and unsafe recovery; peripheral calibration is not established as the single universal cause.

### What the tests already say

- Earlier held-out calibration checks did contain outer-field observations, but coverage was uneven, and the exact Short doorway/cam1 region lacked outer held-out tracks. A sparse sector cannot rule out an error elsewhere.
- The native valid-domain bug lay outside the earlier valid-sample fit, so that fit could not detect it.
- The radial-only trial improved held-out errors and early scale but ended at **28.95** with route deformation. Near-unit scale was insufficient.
- Periodic full calibration v2 reached **80.20**, all 14 control points within 1 m, all 2,999 GT timestamps associated, and one map. It adjusted **all 15 parameters per camera, including focal length**, with priors and per-step/total limits. Rig and camera-to-IMU extrinsics stayed fixed.
- In v2, two calibration trials were accepted. Its final focal changes were small, about −0.24 pixel per camera; joint effects of all parameters produced larger image-field displacements. A small focal delta alone does not measure the importance of calibration.
- The later v3 variant scored **42.59** with three maps. Its final lenses were close to v2 in projection space; the final accepted update did not create a large lens discontinuity. Calibration/state interaction remains possible, but “one giant parameter jump” is not supported by that measured comparison.
- Offline bounded full intrinsic calibration on the common SIFT graph scored **67.18**. Online calibration may change future associations and map construction in ways a frozen-graph batch solve cannot recover.

### The next decisive comparison

Replay the same recurring VI optimization schedule with fixed intrinsics, then with the current bounded full-intrinsic estimator. Keep feature inputs, matching gates, state-update policy, and optimization budgets fixed. Record actual keyframe/commit times; thread scheduling can still change the graph. If necessary, first compare on frozen snapshots to isolate factor behavior and then assess the real online consequence.

This answers whether periodic VI refinement alone explains most of 80.20, whether intrinsics add a repeatable gain, and whether the gain occurs through improved map support after a commit. Do not start with fifteen independent coefficient sweeps.

### Safe calibration design

1. Keep factory-centered priors and hard lifetime bounds; a small per-step limit alone can drift arbitrarily far over many updates.
2. Optimize a normalized parameterization so one trust-region step has a meaningful size across focal, principal point, and distortion terms.
3. Use evidence-dependent activation. Weakly constrained high-order terms should remain near their prior; a large count of central observations is not peripheral observability.
4. Evaluate on held-out observations spanning both cameras, image angle, azimuth sector, depth, and motion. Record coverage where no reliable test is possible. The existing holdout is an optimization holdout from images that already helped form the map; add genuinely later observations for stronger validation.
5. Compare projected fields and bearing changes, not only coefficient deltas. Coefficients can trade off strongly while producing similar images.
6. Validate forward/inverse geometry over the complete physical domain. The sensor's valid pixel mask is fixed by acquisition geometry; do not move that mask merely because the estimated principal point moves.
7. Commit calibration, optimized states, stereo caches, point-depth summaries, and consistent priors together. Rejected calibration proposals restore the complete pre-proposal state in the current map gauge. This is distinct from a rejected tracking update, which must preserve the current valid IMU prediction.
8. After a commit, monitor accepted inliers, normalized innovations, periphery track survival, and pose discontinuities. An improved held-out cost does not authorize an unsafe live-state transition.

### Longer-term calibration research

Compare factory-fixed, one shared offline calibration per physical device, slowly updated shared online calibration, and per-session calibration. Learn repeatable device corrections on a calibration/training split, then evaluate frozen parameters on new recordings. This separates systematic lens error from a flexible model absorbing trajectory error.

Only after this separation consider temperature-dependent or slow temporal calibration. Use an explicit smooth model with tight priors; one free lens per frame would allow severe overfitting and destroy physical interpretability. Consider a small smooth residual bearing field only if the native F624 model leaves a stable, repeated held-out pattern that its coefficients cannot represent.

**Success:** reproducible peripheral reprojection and track-survival gains, improved full-route geometry, stable parameters, and transfer to held-out sequences. **Stop:** calibration reduces its own fit cost while biases, scale, or later tracking become less consistent.

## 10. Rig extrinsics, camera–IMU calibration, timing, and sensor modeling

### Current status

The successful 80.20 experiment calibrated intrinsics, not rig extrinsics or camera-to-IMU transforms. The checked input camera/IMU transform agreed with the supplied calibration at numerical precision. That supports correct ingestion of the supplied transform; it does not establish that the physical calibration itself is perfect.

Native SDK-calibrated IMU data and timestamps are used. Reapplying the same correction would create an error. The current LaMAria grayscale SLAM camera pair is global-shutter; the RGB camera is the rolling-shutter stream. Do not add a rolling-shutter model to the current grayscale pipeline without new acquisition evidence. [Official LaMAria sensor documentation](https://www.lamaria.ethz.ch/slam_documentation).

### Proposed staged freedoms

| Parameters | Expected benefit | Required evidence / risk |
|---|---|---|
| Rig relative rotation | Correct systematic cross-camera angular mismatch | Broad cross-camera overlap and diverse views; can trade off with lens distortion |
| Rig translation direction | Better triangulation for near objects | Reliable near-depth stereo support; weak for distant facades |
| Baseline magnitude | Correct a physical metric-scale mismatch | Strong metric/stereo/IMU observability and a physical prior; never casually release the only scale anchor |
| Camera-to-IMU rotation | Better rotational prediction and inertial consistency | Varied rotational motion, correct gyro calibration and timing |
| Camera-to-IMU translation | Correct lever-arm effects | Sufficient rotational excitation; often weakly constrained in simple walking |
| Small camera/IMU time offset | Correct motion-correlated projection error | Continuous-time or exact shifted-boundary integration, consistent timestamp units, and an explicit prior |
| Gyro/accel intrinsic residual corrections | Address a repeatable sensor model discrepancy | Independent evidence beyond bias drift; avoid absorbing bad gravity or camera scale |

Start with a captured, sufficiently excited graph and one parameter group. Assess marginal uncertainty and correlations with intrinsics, gravity, biases, and metric scale. Only move a group online after the offline identifiability and held-out behavior make sense. All groups need bounded steps and bounded lifetime drift; rotations should use a proper local manifold.

Timing tests should separate fixed offset, exposure timestamp definition, clock drift, and dropped or duplicated samples. A wrong fixed timestamp convention can mimic calibration error during rotation. Log the data contract before estimating an offset. For any future RGB branch, rolling-shutter modeling becomes a separate sensor-specific workstream with measured readout time and exposure behavior.

**Success:** a physically plausible correction that improves independent data, not merely a smaller training residual. **Stop:** calibration freedoms compensate one another without observable support or change significantly when one sequence segment is removed.

## 11. Tracking recovery, state transactions, and complete trajectories

### What has been proven

The campaign contained genuine crash defects in indexing/prior lifetime and genuine observation-contract errors. A moving crash location was not proof of a threading race. Likewise, full-sequence variability is not proof that every inconsistency is a race.

A later source audit found a rejected visual/inertial update could leave inconsistent pose/velocity/bias/prior state. Online v2 could snapshot after a bad visual relatch and export that rejected relatch as its fallback. v3 moved the snapshot to the successful IMU prediction and restored the propagated state consistently. Its focused contracts passed and all 205 logged rejected initialized updates obeyed restoration, but the full run still split and scored **42.59**. The correct invariant must be retained; the rest of recovery must be made reliable.

### The coherent repair to design

- Represent a tracking attempt as a proposed state with clear accepted/rejected ownership. Include pose, velocity, biases, marginal prior, associations, and map version. Only accepted visual information becomes the next accepted state.
- Separate an IMU prediction from a visual relatch hypothesis. On failure, revert to the valid prediction with an appropriate prior, not a half-updated combination.
- Keep coasting confidence explicit. IMU propagation can bridge a brief occlusion, but it is not an indefinitely accurate trajectory and should not create ordinary visual keyframes unsupported by observations.
- Add a recovery ladder: local-map expansion, reference/covisible re-match, broader verified pose proposal, then retrieval-based relocalization. Preserve uncertainty and use geometric acceptance at every stage.
- Distinguish global map corrections from physical motion in both state handling and logs. Record the correction transform, affected map version, reference keyframe state, and exported live pose before and after.
- If a map must reset, retain complete independent map exports and seek measured reconnection. Report selected-map and all-atlas coverage separately.

The existing 80.20 Short bridge region and v3 first-loss region are specific evidence, not interchangeable “doorway” failures. v3 first sustained loss began near native 458.64 s and reset near 463.69 s. The v2 known brief rollback jumps were near 549.74–549.79 s. A 14.44 m internal relatch discrepancy later in v2 was largely undone within the same frame; it was not a 14.44 m exported trajectory jump.

**Success:** repeated full-route connected tracking with fewer weak/coasted frames and no rejected-state leakage. **Stop:** increasing the grace period until drift is hidden, relaxing acceptance to retain a bad pose, or selecting only the largest favorable segment for an apparent accuracy win.

## 12. Short-sequence accuracy and the gap to a strong reference

Short routes should accumulate less drift, but they can still be limited by initialization, weak excitation, inconsistent calibration, incomplete coverage, bias/gravity error, landmark depth error, or one bad transition. There is no established single secret explaining the reference system's short-sequence accuracy.

The 67.19 baseline already triangulated 14/14 control points, but only 10 were within 1 m. Recovering them all was therefore not enough. The 80.20 run brought all 14 within 1 m while improving raw metric error. This makes relative geometry and stable transitions a more useful target than simply filling in missing timestamps.

### Proposed error budget

For each full Short run, report startup, stable interior, difficult transition, and final sections. Decompose:

1. Coverage and initialization latency.
2. Rigid-SE3 error versus one-global-Sim3 shape error.
3. Local relative translation/rotation error at several time or distance scales.
4. Control-point error distribution, not only an aggregate score.
5. Gravity/bias evolution and scale consistency across independently assessed route segments.
6. Per-camera/peripheral support and uncertainty.
7. Corrections at BA/calibration/recovery commits and whether they are physically smooth after accounting for gauge changes.

Then target the largest measured contribution. If most error is a few correction events, refine transaction/recovery logic before adding a dense depth network. If consistent curvature follows camera angle and repeats across routes, calibration becomes a stronger candidate. If all short geometry improves only after global optimization, inspect the local prior/window. These are decision rules, not conclusions already established.

**Success:** repeatable high accuracy across several short routes with complete coverage. **Stop:** optimizing only `sequence_1_19` until its score is high while Medium, Long, or held-out Short routes degrade.

## 13. Frontend choices and the pinhole-correspondence idea

The user explicitly allowed changing the frontend or backend. The current successful online route uses ALIKED-derived observations. Classic ORB appeared weak under blur in earlier inspection; SIFT was used for the completed offline graph. Neither observation proves a universal ranking across this dataset.

### Basic comparison

Freeze native image pairs and a small set of informative temporal gaps: neighboring frames, 1–5 s, wider-baseline revisits, blur, doorway, bridge, center, and periphery. Compare ORB, the current ALIKED path, SIFT, and a learned matcher at equal or explicitly recorded feature/compute budgets. Judge verified static matches, localization precision, angular coverage, track survival, and downstream pose/landmark support. Pairwise match counts alone are insufficient.

### Can pinhole views be used only for correspondences?

**Yes, as a proposal-generation representation.** Render one or more virtual pinhole views with known rotations and intrinsics from each native fisheye image. Extract and match there. Convert every match through its exact virtual ray and known rotation back to native F624 pixels/bearings before it becomes a SLAM measurement. Keep provenance and merge duplicates from overlapping virtual views.

Advantages: existing learned matchers and depth networks may behave better on their expected perspective images; upright views can help retrieval. Costs: resampling error, seams, duplicated observations, lost field of view, and multiple pixels representing one native measurement. Do not optimize native images with a pinhole model merely because the correspondence extractor used one. Show a virtual/native match overlay and verify peripheral coordinate round trips.

### Research directions

Use learned matches only when cheap tracking lacks support; use multi-frame track consistency instead of repeated independent pair matching; learn native-fisheye descriptors or virtual-view policies on an allowed training split; improve subpixel localization; and add blur-aware uncertainty rather than simply larger search windows. Matchers should earn their cost in whole-system accuracy and continuity.

**Success:** more correct, longer-lived, geometrically useful observations with a clear full-system gain. **Stop:** dramatic red-dot density with unchanged optimizer support, worse outliers, or unacceptable latency.

## 14. Visual diagnostics are part of each experiment

The user wants to inspect the evidence, not infer progress from a long tool transcript. Each major completed experiment must include a short summary: what changed, what was held fixed, what was tested, positive/negative/neutral outcome, score and coverage, absolute artifact paths, and what a correct result should look like.

### Required views

- Both native camera streams with red detected/unassociated features and green associations, plus an explicit definition of green for that recording. Online inliers, final-map associations, and persistent trails must not be silently conflated.
- The evaluated trajectory, landmarks, and rig using the exact saved CP Sim3 from that run. No separate GT fit for a prettier display. If raw metric geometry is also shown, use a separately labelled view.
- A growing-map view. If points are final optimized points revealed at their first retained observation, say so. It is not a recording of historical BA states.
- Gap-aware trajectory lines and explicit missing-pose intervals. Never draw a line that suggests an unsupported jump is continuously observed.
- Per-camera association funnels, support, reprojection error, coasting state, and map transitions.
- Top-view loop proposals/corrections, with paired images and before/after errors.
- Calibration fields: central/peripheral residual arrows, data coverage, per-sector counts, parameter history, projected field changes, priors/bounds, and accepted/rejected proposal reasons.
- Scale/init charts: baseline/metric support, initialization acceptance, gravity/bias/scale history, and rigid-versus-similarity diagnostics.

Existing builders include `pipeline/viz/make_lamaria_output.py`, `lamaria_global_refinement_report.py`, `lamaria_intrinsic_field_changes.py`, `lamaria_batch_calibration_report.py`, and the failure/matching report tools. Reuse them before building another dashboard.

The complete verified 80.20 evaluation Rerun is:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/online_full_evaluation.rrd
```

The current common experiment index is:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/VISUALIZATIONS.txt
```

The final offline-calibration Rerun export was stopped at the user's request and is marked incomplete. Its completed optimization/score and an incomplete renderer are distinct statuses. Do not advertise that Rerun as verified until a later authorized export and audit complete.

## 15. A multi-month research campaign, without uncontrolled run proliferation

These are staged research options, not a calendar promise or authorization to launch everything.

| Stage | Main question | Concrete work | Evidence required to advance |
|---|---|---|---|
| A: preserve and explain | Why did 80.20 improve? | Freeze result; compare recurring fixed-intrinsic VI and bounded-calibration VI; verify state/commit telemetry | Same-sequence repeatability, complete coverage, interpretable metric and score changes |
| B: continuity | Can the candidate survive normal difficult areas? | Coherent update/rollback/prior handling; stronger verified recovery; map correction propagation | Repeated full Short and Medium without regressions; no rejected-state leakage |
| C: transfer | Does the gain generalize? | Full Short/Medium/Long with a frozen configuration; held-out Short | Gains not confined to the tuned sequence; runtime and memory documented |
| D: loops | Can the route close without metric deformation? | MegaLoc retrieval, native geometry, robust metric graph correction, joint VI refinement | Verified closures on full `sequence_2_11`; no false-loop damage; full-route score gain |
| E: better information | What limits medium-range support? | Better track persistence, selective learned matching, useful older anchors, local-window design | Improved relative error and support, then full-route gain |
| F: calibration/initialization | Which physical parameters remain wrong or weak? | Device-level lens validation; staged extrinsics/timing; adaptive stereo-inertial init; optional DA3 | Independent evidence and held-out improvements, bounded identifiable parameters |
| G: backend research | Is optimization architecture the bottleneck? | Delayed/square-root marginalization, incremental robust smoothing, hierarchical batch refinement | Equivalent-factor correctness, convergence evidence, accuracy/runtime benefit |
| H: leaderboard campaign | Is the complete system competitive? | Freeze system, evaluate permitted comparable suite, prepare reproducible submission package | Comparable aggregate results and a fully auditable data/training/compute protocol |

### High-upside work worth months if earlier stages justify it

1. **Joint uncertainty-calibrated tracking and mapping.** Learn or estimate feature localization uncertainty, model correlated measurements, and feed it consistently into matching and BA. Expected benefit is better use of difficult observations; risk is learned confidence hiding systematic error.
2. **Native wide-angle multi-view tracks.** Train or adapt trackers/matchers for fisheye imagery and transitions across camera overlap, with explicit static-scene and occlusion handling. The goal is durable geometry, not just more keypoints.
3. **Calibration as a slowly learned physical model.** Separate repeatable device corrections from per-session states and use observable updates only. Test transfer across motion/lighting/temperature conditions before introducing temporal freedom.
4. **Metric learned geometry proposals.** Combine DA3/VGGT-like proposals with actual stereo and inertial factors and robust rejection. Learned geometry should improve the difficult initialization/support cases that classical measurements leave weak.
5. **A consistent hierarchical smoother.** Maintain local real-time tracking while a sparse global graph incorporates long-lived tracks and loops, then propagate corrections with the right covariance and gauge. Avoid repeated incompatible optical/VI passes.
6. **Scene structure and dynamic reasoning.** Use planes, lines, static-object masks, and semantic persistence only when they provide independently validated constraints. Strong false structure can be worse than sparse points.
7. **Adaptive compute allocation.** Spend expensive matching and refinement on moments of uncertainty or a valuable revisit; keep easy tracking cheap. Optimize accuracy per unit compute after correctness and continuity are established.
8. **Failure-aware relocalization and map reconciliation.** Preserve usable submaps with uncertainty, retrieve revisits, and merge through measured geometry. This can improve full-route coverage without pretending that missing data was tracked.

Research references identify useful mechanisms, not guaranteed state-of-the-art performance on LaMAria. Recheck current primary implementations and choose a pinned version when an experiment is actually scheduled.

## 16. Compute, implementation, and communication discipline

### Before a new experiment

Write a short experiment card: hypothesis, exact parent, one main change, expected observation if correct, comparison scope, score/coverage measures, resource budget, artifact paths, and stopping decision. A bug fix can require several coupled code changes; that is acceptable when they implement one documented invariant. Unrelated hypotheses should not be bundled invisibly.

### During implementation

- Keep `third_party/` read-only. Changes belong in `patches/`, private staging/builds, and `pipeline/`, with source/binary/runtime hashes.
- Reuse feature caches and frozen graphs when doing so preserves the comparison. Re-extract only when the frontend itself is under test.
- Test production geometry and real state transitions. Do not maintain a separate convenient triangulator that can agree with itself while SLAM remains wrong.
- Add meaningful regression tests for genuine invariants: observation ownership, rejected-state restoration, calibration validity, coordinate conversions, and factor Jacobians. Do not create an endless stream of toy tests that cannot decide the actual implementation.
- Estimate graph memory and solver behavior before the full run. Use explicit resource limits, checkpoint expensive solves, preserve attempts, and never call a killed job a completed experiment.
- A private kernel speedup is not an end-to-end speedup. A 30-attempt exit is not convergence. A successful build is not a tracking result. Report these distinctions plainly.

### At completion or a major crossroads

Update `EXPERIMENTS.md` with the outcome and exact evidence. Give the user a short table and absolute paths. State whether the result is positive, neutral, negative, incomplete, or only a contract test. Explain the next decision the evidence supports, and stop for the requested major-stage discussion instead of silently branching into several more runs.

The user has explicitly objected to many small tests returning failures without a coherent advance. The answer is not to stop testing. It is to make each test answer a necessary question, combine related repairs into a complete implementation, and promptly test the resulting full system. The project is measured by a reliable, accurate, competitive SLAM system—not by the number of experiments, patches, or generated Reruns.
