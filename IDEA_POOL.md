# Idea pool: keeping Goat-SLAM connected and accurate

Updated: **2026-10-04**.

This is a living collection of hypotheses and design discussions, not a list of
proven capabilities. **Work is stopped at the user's usage-limit request.** This
update records completed work and next questions; it authorizes no new run.
Read `docs/HANDOFF.md` and `docs/RESUME_20261004.md` before resuming.

One BabyFeatures SOS implementation and one full Long replay are complete on
`experiments/BabyFeats`: **43.729292, one retained map, 99.38% pose coverage**.
One subsequent offline fixed-camera native-graph VI refinement scored
**45.027586 with the identical 35,619 / 35,842 timestamps and 27 / 27 CPs**.
The frozen Short v2 **80.199161** and historical Long **49.881534** remain
preserved. BabyFeatures Medium and Short are **untested**. Exact evidence is in
`EXPERIMENTS.md`, `docs/BABYFEATURES_LONG_20261003.md`, and
`docs/BABYFEATURES_NATIVE_VI_20261004.md`.

## Direction and relationship to the other documents

The immediate priority is preventing lasting map fragmentation during doorways,
lighting transitions, and temporary occlusions while preserving the geometry of
the Short v2 80.199161 result. **The user's latest clarification permits passive
BabyFeature tracking throughout, but their measurements enter pose estimation
only after normal map tracking fails.** Preserve the normal estimator path.
Staying in one map must also mean
accurate, observation-supported motion; disabling resets is not proof of success.

- `GOAL.md`: the project objective and evaluation principles.
- `ROADMAP.md`: the broader research and implementation areas.
- `EXPERIMENTS.md`: measured outcomes, including negative results.
- `docs/RESUME_20261004.md`: the current stopping point, artifact identities and
  next-agent handoff; proposals below are not a queue to execute automatically.
- This file: proposed ideas, their reasoning, risks, and possible future tests.

The latest prevention-first discussion here supersedes older immediate-priority
wording about recovery in those documents. Recovery correctness remains relevant
to accepting returning map observations, but is not the whole prevention plan.

## I01. BabyFeatures: visual motion support before permanent landmarks

**Origin:** user proposal. **Status:** implemented and tested once on full Long.
**Result:** one retained map after a startup reset, 15/15 SOS returns, 99.38%
native pose coverage. Score2D 43.729292 versus native v2 34.458901; historical
Long 49.881534 remains higher. All 27 CPs recovered, only four within 1 m.
One 1.619 m final pose jump remains. This is a positive continuity result,
not a claim that accuracy or repeatability is solved. The implementation uses
at most eight frames, so the full-interval smoothing below remains an aspiration
for longer gaps. A separate final fixed-camera VI solve improved this same
trajectory to **45.027586**, without changing coverage; the largest jump became
**1.622 m**, so it did not repair the SOS discontinuity. See I11 below. Review
diagnostics before choosing the next experiment; Medium transfer is still untested.

### The idea

Use the same feature extraction pool for two roles:

| Role | What it represents | What it contributes |
|---|---|---|
| Mature feature / map landmark | A geometrically established point in the persistent map | Reprojection constraints tying current motion to that map |
| BabyFeature | A verified short-lived image track with temporary or uncertain 3D structure | Multi-view constraints on recent motion, even before permanent map membership |

These can be lifecycle states rather than two unrelated extractors or mandatory
C++ class hierarchies. A newly detected singleton is only a candidate. A track
observed in multiple frames can become useful before it is suitable for long-term
mapping. Some tracks may mature; others can constrain motion and then retire.

**BabyFeatures are good correspondences that are too short-lived for persistent
map membership.** They are not points rejected because they are geometrically
inconsistent or belong to moving objects. A briefly visible static wall corner
is useful. A confidently tracked point on a walking person can still give the
wrong camera motion.

**Latest activation decision:** track BabyFeatures passively during healthy
operation, but do not include them in SLAM pose optimization or map updates then.
Activate their visual-inertial constraints only when ordinary tracking fails.
This supersedes the earlier discussion that disabled even passive tracking.

The user's motivating example is entering a dark room and later returning to a
mapped area. There may be enough short-range optical correspondences inside the
room to help the IMU, despite few points qualifying as persistent map landmarks.
Returning observations of the original map can then constrain the intervening
motion. This is possible only to the extent that valid measurements and connected
state history survive the interval.

### A real gap in the current implementation

The frozen v2 temporal matcher carries existing landmark identities forward; it
does not convert arbitrary multi-frame red-point tracks into motion constraints:

- `baselines/online_full_v2/sources/src/Tracking.cc`,
  `SeedTemporalLandmarks` around line 4453: requires an existing, nonbad,
  same-map `MapPoint` with `Observations() > 0`.
- `Tracking.cc`, `TrackWithMotionModel` around line 4357: the initialized-IMU
  branch predicts motion before subsequent local-map visual correction.
- `baselines/online_full_v2/sources/src/Optimizer.cc`, around lines 5531 and
  5967: the corresponding visual residual loops require non-null map points.
- Existing temporary VO points in `Tracking.cc` around line 4283 belong to a
  localization-only, measured-depth path. They are not this general short-track
  mechanism in the active mapping pipeline.

Thus the proposal adds a source of visual information; it is not just a new name
for the accepted-landmark memory experiment or existing temporal seeding.

### Proposed behavior

1. **Leave healthy pose estimation unchanged.** Keep a bounded passive track
   history from the existing extraction pool. BabyFeature factors and map
   insertion are off. Normal v2 association and optimization continue as before;
   extra tracking work can affect runtime/scheduling and must be measured.
2. **Activate only after normal map tracking declares failure.** Intercept that
   failure before map reset or disconnected reinitialization. This means actual
   failure of normal map tracking, not merely a lower inlier count while tracking
   still succeeds, and not waiting for the terminal `LOST` reset branch. Save
   the last valid map/VI state and preserve a coherent prior. At activation,
   use the passively retained tracks and accepted frame history.
3. **Run a local VI window during SOS.** Combine BabyFeature observations,
   IMU preintegration, and any independently verified mature-map observations
   over recent poses, velocities, and biases. Use both cameras through their
   native Fisheye624 models and calibrated rig/IMU transforms. Repeated short
   visual constraints should limit drift rather than leaving IMU propagation
   to carry the whole interval unaided.
4. **Allow supported motion without permanent map points.** Short tracks can
   constrain relative motion while new landmarks are still being established.
   Do not require every useful track to survive a long map-point lifecycle.
5. **Remain connected to the existing map state.** Carry its metric state and
   uncertainty into the window. Do not start an unrelated arbitrary-scale
   monocular map or silently reset velocity, biases, or gravity.
6. **Continue seeking mature-map associations.** Search the original map while
   advancing through the interval. Validate returning observations jointly with
   the visual-inertial window before declaring strong map support restored.
7. **Refine the retained interval when support returns.** Optimize the bridge
   poses and inertial states using the original observations plus the returning
   map constraints. This is measured backward refinement, not endpoint snapping
   or interpolation of missing poses.
8. **Return to normal tracking and turn BabyFeature estimator factors off.** Require
   verified map support and a coherent state/prior handoff, not just one promising
   descriptor match. Transfer the accepted refined state without counting the
   SOS observations twice. Until that handoff is validated, remain in SOS.
   Passive track maintenance can continue after normal tracking resumes.
9. **Promote useful tracks selectively.** Admit a BabyFeature to the persistent
   map when depth conditioning, static consistency, view support, and uniqueness
   justify it. Long lifetime alone is not sufficient.

The intended transition is: **normal tracking (Baby tracks retained, no Baby
factors) -> declared map-tracking failure -> SOS visual-inertial tracking
(Baby factors active) -> verified normal-map tracking and bridge refinement ->
normal tracking (Baby factors off)**. Normal tracking may resume on original
landmarks or on explicitly promoted, geometrically validated new landmarks. The
latter matters on a forward route that never returns to its previous room.

### How to do this without adding a separate filter

The OpenVINS analogy is relevant: it supports both in-state SLAM landmarks and
MSCKF feature updates. Its MSCKF update triangulates a track and eliminates the
feature's linearized dependence before updating motion state. It does not make
arbitrary uncertain image matches informative automatically.

For this optimizer-based pipeline, the design preference is **an SOS-only
sliding-window VI module sharing the established map and inertial state**, with
temporary inverse-depth or point variables that can be eliminated during
solving. It does not replace the healthy normal-tracking optimizer. Structureless
visual factors are another formulation: retain motion constraints without
permanent landmark variables.

This can be a separate software module while sharing one consistent state and
measurement accounting. It does not require inserting an independent OpenVINS
filter and treating its poses as independent measurements of the same images
and IMU. Doing that naively would count correlated evidence twice.

The native camera residuals and derivatives remain authoritative. GTSAM smart
factors are a design reference, not a claim that an off-the-shelf pinhole factor
can replace the production Fisheye624 geometry without adaptation.

### Atlas and correcting the lost interval

Atlas provides useful multi-map infrastructure, but it does not automatically
consume mapless short tracks or smooth their history. In frozen v2,
`Atlas.cc::CreateNewMap` (around line 58) stores the current map and allocates a
new map; `ChangeMap` (around line 79) switches the active map. The actual
recognition and geometric merging live in `LoopClosing.cc`, including
`NewDetectCommonRegions`, `DetectCommonRegionsFromBoW`, and the merge routines.
They require supported keyframe/map-point correspondences and eligibility checks.

The first design preference is to retain the original map and its metric state
while SOS tracks the interval. Reacquisition then adds original-map constraints
to the retained bridge. If a separate submap becomes necessary, its connection,
eligible keyframes/landmarks, inertial history, and merge integration must be
designed explicitly. Atlas alone does not supply those missing constraints or
automatically repair the unrecorded motion. No Atlas adapter has been built.

### Important limits and design decisions

- **Two or three views can help without permanent landmarks.** Short static
  tracks plus the established metric VI state can constrain motion. A third
  view improves redundancy and, with useful motion, triangulation; three
  monocular views alone still have global scale ambiguity. Frame count does not
  establish metres. Existing map scale, informative IMU history, and actual
  informative stereo observations provide metric support. Two-frame optical
  constraints plus IMU can already be useful; do not require a universal
  three-frame minimum or claim IMU makes every two-frame geometry observable.
- **Low parallax is weak depth information.** Pure rotation or nearly coincident
  viewpoints cannot be treated as confident new depths. Track conditioning and
  the native camera validity domain must govern which constraints are usable.
- **Different tracks can overlap in time.** No single point must survive the
  entire room, provided the combined visual-inertial graph remains informative
  and connected. IMU edges alone give connectivity without guaranteeing accuracy.
- **Moving objects need rejection.** Many consistent tracks on one moving person
  are not many independent static constraints. Use scene consistency and spatial
  support; add semantic masks only as a separately assessed mechanism.
- **Promotion must not duplicate evidence.** Give pixel observations unique
  ownership. Replacing a temporary representation with a mature landmark must
  not re-add measurements already represented by active factors or a marginal
  prior. Decide promotion/retirement and factor ownership before implementation.
- **Do not retain a bad visual update in an IMU prior.** Accept or reject the
  joint pose/velocity/bias update coherently. Rejected candidates must not leak
  information into future state or restart confidence/grace bookkeeping.
- **Rejoining requires history.** A fixed-lag window cannot freely reoptimize
  poses already marginalized away. Retain bridge keyframes and the required
  visual/IMU factors, or support a correctly constructed delayed smoothing
  graph. Reusing archived factors alongside a prior containing those factors
  would double count them. Bound memory and label the revision horizon.
- **Long absence differs from a short doorway.** A seconds-long bridge is the
  first target; a long excursion needs keyframe compression and explicit drift
  handling. A returning pose cannot reconstruct an unconstrained past path.
- **Keep calibration stable during weak support initially.** Carry accepted
  v2 calibration into the bridge. Do not let a dark, poorly constrained window
  explain motion error by changing intrinsics/extrinsics. Coordinate any later
  map/calibration update with active factors, caches, and the marginal prior.
- **Genuine blindness remains IMU-only.** Completely black, textureless, or
  fully occluded views supply no usable visual tracks. Report growing
  uncertainty instead of promising unbounded accurate tracking.

### Further validation and visual diagnostics

The initial implementation passed 14 native numerical checks and one full Long
replay. The broader proposed cases below are not all covered by that evidence;
they are future extensions, not additional completed runs or present authorization.

First isolate the new information source: keep the extraction pool, native
geometry, and baseline matching/acceptance policies unchanged where applicable.
Design statistical gates for the new multi-view factors according to their
residual model; do not copy a single-point threshold to a different residual
dimension without justification. Avoid bundling feature-count, calibration,
initialization, and threshold changes into the first experiment.

Proposed checks should establish that BabyFeature estimator factors remain
inactive throughout successful normal tracking, activate on declared map-tracking
loss, and deactivate after a validated state handoff. Also cover useful short static
tracks, insufficient parallax, moving foregrounds, one-camera occlusion, complete
blindness, promotion without double counting, rejected-state rollback, and return
to original map landmarks.
Then replay original full prefixes through known failures, followed by complete
Short/Medium/Long runs if the first evidence is positive. No GT enters tracking.

Suggested diagnostics:

- Green: accepted mature-map observations. Amber: accepted BabyFeature
  observations. Red: currently unused/rejected detections; explain the legend.
- Per-camera counts, track age, angular/spatial distribution, residuals,
  parallax/depth conditioning, dynamic rejection, and estimator latency.
- Explicit states: map-supported, short-track-supported, and IMU-only. A colored
  point is not itself a guarantee of correct motion.
- Top view around the doorway, map identity, bridge start/end, and trajectory
  before/after the returning-map refinement. Distinguish live estimates from
  subsequently revised history.
- Success requires retained continuity **and** sound metric/relative motion,
  bounded state disagreement, useful static support, full coverage, and no
  regression of the preserved Short result. More amber dots alone is not success.

### Primary references for the concept

- OpenVINS estimator and its two feature roles:
  https://docs.openvins.com/namespaceov__msckf.html
- OpenVINS MSCKF update and feature elimination:
  https://docs.openvins.com/classov__msckf_1_1UpdaterMSCKF.html
- OpenVINS triangulation and conditioning checks:
  https://docs.openvins.com/update-featinit.html
- GTSAM structureless/smart-factor formulation:
  https://borglab.github.io/gtsam/smartfactors/
- ORB-SLAM3 multi-map recognition and merging architecture:
  https://arxiv.org/abs/2007.11898

These support the architecture discussion, not a measured benefit in Goat-SLAM.

## Other map-break prevention ideas already discussed

All entries below remain proposals unless their evidence column explicitly
describes an earlier experiment. They are not additional tests authorized now.

| ID | Idea | Expected benefit | Existing evidence / main limitation |
|---|---|---|---|
| I02 | Establish next-region landmarks before old support disappears | Preserve overlap across doorways using coverage, parallax, and track age to guide keyframes/triangulation | Existing inertial keyframe insertion already exists; assess useful geometry and mapping latency, not just more keyframes |
| I03 | Spatially balanced additional ALIKED features | Retain static background around occluders and across both lenses | Current caches bypass ordinary ORB extraction; increasing `ORBextractor.nFeatures` alone does not change cached inputs. Regenerating extraction is a separate variable |
| I04 | Uncertainty-aware candidate search | Find valid correspondences when pose predictions have become less precise | Expand bounded search effort without indiscriminately weakening final geometric acceptance. More candidates increase ambiguity and cost |
| I05 | IMU-guided optical flow / native fisheye patch tracking | Maintain adjacent-frame identities despite descriptor appearance changes | Can supply I01 tracks or improve mature associations; needs forward/backward and geometry checks. Repeated patch measurements are correlated |
| I06 | Per-camera visibility and static-scene support | Let the unobstructed camera carry useful constraints when the other sees a hand/person | Both cameras already constrain pose; proposed change is support-aware treatment, not merely enabling cam1. Avoid rejecting all difficult imagery |
| I07 | Retain useful landmark appearances through brief occlusion | Avoid immediately forgetting points that can return to view | The earlier 0.5 s accepted-landmark memory arm scored 45.874750 Medium / 42.904563 Long, with 4/6 maps: mixed versus native v2, not promoted. It required existing landmarks and did not implement I01 |
| I08 | Bounded VI bridging with coherent state/prior updates | Preserve an initialized map during temporary visual starvation | Five-second grace already exists. Longer timers alone add no information; BabyFeatures would supply an additional visual constraint source |
| I09 | Native floating-point ALIKED descriptors | Avoid possible losses from the current binary256 representation | Plausible frontend experiment, not a proven cause. Match compute and keep this separate from the initial BabyFeature design |
| I10 | Diagnose systematic periphery/calibration errors alongside occlusion | Separate geometry bias from missing static image information | Stratify held-out residuals by camera/ray angle and image condition. The good v2 score does not establish that all lens error is solved |

## I11. Separate final fixed-camera VI refinement of the connected native graph

**Origin:** follow-up discussion of the attractive but 43.73-scoring connected
Long route. **Status:** one completed offline experiment; modest positive result.

The periodic calibration path constructs a fixed-camera VI reference and a
free-intrinsics candidate, but commits only a successfully validated calibration
candidate. When calibration is rejected, its fixed-camera VI reference is also
discarded. This motivated testing a separate final geometry/state refinement.

The completed experiment reused 2,401 native keyframes, 156,557 landmarks and
1,119,060 observations from the exact BabyFeatures run. It fixed the last accepted
Fisheye624 intrinsics and the physical rig/IMU extrinsics, optimized visual and
inertial states, and transported every existing non-keyframe pose through its
original surviving reference keyframe. No new SIFT graph, GT factors, missing-pose
interpolation or SLAM replay was used. Serialized online velocities/biases were
unavailable and had to be initialized again; non-keyframe relative states were
preserved rather than individually optimized.

**Measured response:** Score2D **43.729292 → 45.027586**; CP within 1 m **4 → 5**;
horizontal RMSE **2.933179 → 2.754464 m**. Sixteen CP errors decreased and eleven
increased. All 27 CPs and exactly 35,619 exported timestamps remain. The solver
reported convergence at a cost plateau after 39 outer iterations / 15 accepted
updates; final native reprojection RMS was **0.803250 px**. The **1.622 m** pose
jump survived. These results support an offline accuracy improvement on this
run, not a claim that online tracking improved or that low pixel error guarantees
70–80 points. The older Short 67 visual/fixed-VI/calibrated-VI trials remain
negative or effectively neutral evidence in `EXPERIMENTS.md` E28–E31.

**Next ideas, not executed:** improve verified longer-range static constraints,
retain/refine a coherent SOS interval, or test a specifically identified sensor
model discrepancy. Repeating an unchanged solve is not supported as the next
high-value step. A calibration change needs independent geometric evidence and
held-out checks. Preserve this 45.03 refinement and the stronger historical Long
reference separately.

Exact report: `docs/BABYFEATURES_NATIVE_VI_20261004.md`.
Comparison visualization:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_native_vi_20261004/comparison/comparison.png
```

## I12. Diagnose the missing beginning, end drift and apparent tilt

**Origin:** the user's latest visual observations. **Status:** unresolved
interpretation; no new experiment or corrective transform has been applied.

Separate what the export proves from what the Rerun suggests:

- The Baby Long source and 45.03 refinement both omit **223 startup frames**.
  Their 40 unmatched dense-GT timestamps are all before the first estimate.
  The source input spans native 152.349–1944.399 s; exported history starts at
  163.499 s and reaches the final input. Thus missing startup is measured;
  missing exported end coverage is not the explanation for the observed end drift.
- Whole-route accuracy is still imperfect after one official CP Sim3. Source
  horizontal error reaches 10.843 m; the refined output reaches 8.590 m.
  Those global maxima do not by themselves locate or identify the drift source.
- Apparent tilt is an observation, not yet evidence of a wrong gravity vector,
  quaternion convention, camera calibration, or initialization. The raw map,
  saved evaluation rotation, viewer up-axis and actual route elevation are
  distinct things to inspect. Do not rotate the trajectory or cloud by eye and
  present the change as an estimator correction.

**Next diagnostic when work resumes:** identify the exact source/candidate
artifact being viewed; inspect export coverage, raw poses, saved official Sim3,
gravity/frame conventions and camera/rig transforms together. Plot existing
position/orientation residuals over time, with SOS and backend-update boundaries,
to distinguish a global visual tilt from changing local error and state-history
discontinuities. Keep the existing full denominators. GT is diagnostic and
evaluation evidence only, never an initialization, calibration, smoothing or
trajectory-joining input to the estimator.

The available full Rerun is the **43.729292 source**, not the later 45.027586
refinement. Its Baby panels reconstruct passive descriptor tracks; individual
solver inlier identities were not logged. Aggregate SOS counts and activation
come from actual logs. This distinction matters when interpreting apparent
support or geometry at the late failure.

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_babyfeats_long_20261003/evaluation_baby_20261004/run.rrd
```

## Wider idea inventory — retained, not the current discussion scope

The detailed hypotheses, attempted work, and scores remain in `ROADMAP.md` and
`EXPERIMENTS.md`. This index keeps the wider pool visible without duplicating or
reclassifying historical experiments as new proposals.

| Area | Ideas to retain | Detailed location |
|---|---|---|
| Initialization and metric scale | Observable multi-frame stereo-inertial startup; confidence-weighted learned depth such as DA3, metrically checked against actual sensor constraints | ROADMAP sections 3–4 |
| Medium-range/local accuracy | Informative local VI windows, persistent tracks, robust uncertainty models, lines/planes where supported | ROADMAP sections 5–6 |
| Global refinement | Optical versus VI batch refinement, convergence/weighting improvements, coherent priors and metric rig constraints | ROADMAP section 7; EXPERIMENTS E28–E31 include negative/neutral measured refinements; I11 and docs/BABYFEATURES_NATIVE_VI_20261004.md record the modest 43.73→45.03 native-graph result |
| Loop closure | MegaLoc retrieval, native-rig verification, CLoSeR-inspired streaming reconstruction/closure investigation, metric graph correction with VI refinement | ROADMAP section 8; discussion proposals, not completed integrations |
| Online calibration | Bounded observable intrinsics; later staged rig/IMU extrinsics and time-offset refinement | ROADMAP sections 9–10; v2 already adjusts native intrinsics, not those extrinsics |
| Short accuracy and motion knots | Investigate motion-derived slow/dwelling segments for extra refinement without surveyed control-point inputs or assumed zero velocity | ROADMAP section 12; deferred until continuity is reliable |
| Frontend alternatives | Pinhole virtual views for correspondence proposals with native-ray mapping, float/learned descriptors, dense/multi-frame trackers | ROADMAP section 13 |
| Diagnostics and reproducibility | Native geometry overlays, full trajectory coverage, exact evaluation-aligned Reruns, paired evidence and preserved baselines | ROADMAP sections 14–16 |

## How to extend this pool

Give each new idea an ID, origin, current status, intended mechanism, evidence
already available, likely failure modes, and the smallest decisive future
comparison. Link measured results into `EXPERIMENTS.md` when work is actually
performed. Never turn a proposal into an implementation claim or score promise.

Latest completed estimator work: **I01's one full Long replay and I11's one
offline fixed-camera native VI refinement**. The source Rerun is complete and
verified; the refinement has scored outputs and comparison diagnostics, without
a claimed candidate Rerun. No missing poses or inter-map registrations were
invented. Frozen Short 80.20 and historical Long 49.88 remain preserved. The
current action is documentation and handoff only; use `docs/RESUME_20261004.md`
and `docs/HANDOFF.md` to restart from the actual evidence and the user's latest
questions rather than automatically rerunning old experiments.
