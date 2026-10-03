# Visual recovery to local VI: confirmed source handoff

This is a read-only analysis of experiment A Medium. The frozen candidate was
not changed after replay started. Large displacements below measure disagreement
between the visual hypothesis and subsequent VI solution, not GT error.

## Observed events

| Native time (s) | Visual recovery inliers cam0/cam1 | Local VI input cam0/cam1 | Local VI retained cam0/cam1 | VI displacement from visual pose | Final decision |
|---|---:|---:|---:|---:|---|
|1120.537393837|45 / 5|54 / 5|21 / 0|7.5196 m|Commit21|
|1205.287393837|51 / 76|107 / 152|11 / 5|5.4930 m|Commit16|
|1209.287393837|29 / 57|85 / 145|8 / 12|8.5168 m|Commit20|

The input column includes new local projection/temporal proposals; those are
associations, not already verified inliers. At1205.287, the visual pose moved
5.4929 m from the IMU prediction and the next VI optimizer moved5.4930 m from
the visual pose. Those similar magnitudes suggest strong disagreement with the
propagated state; scalar distances alone do not prove vector direction or which
solution is correct. The local optimizer changed velocity by only0.000531 m/s
and gyro/accelerometer biases by zero at this event.

The complete transaction set has12 geometrically accepted relatches:5 committed,
7 rejected. Rejected transactions restore the pre-relatch prediction and preserve
the existing lost timestamp. Committed events reset the timestamp in the same
frame. In particular, t1205.287393837 logs lost_since=1205.287393837; the previous
failed relatch at1205.237393837 keeps lost_since=1205.087393837.

Log evidence (relative to project):
`experiments/lamaria_continuity_batch_20261003/recovery_direct_medium/runs/recovery_direct_medium_medium_full/run.log`
- Lines76341–76345: first table event: RECOVERY_DIAG, RELATCH_DIAG, LM_DIAG, transaction.
- Lines81980–81984: second event.
- Lines82244–82248: third event.
- Lines81878,81885,81892,81898,81977,82154,82164: seven rejected transactions.

## Exact source path and state transition

All following source paths begin:
`build/orbslam3_lamaria_recovery_direct_20261003/sources/`

1. `src/Tracking.cc:2865,2910–2912`: frame IMU preintegration is created from
   the previous frame bias; the frame receives both frame-to-frame integration
   and `mpImuPreintegratedFromLastKF`, plus the existing `mpLastKeyFrame`.
   `PredictStateIMU` at2921–2968 propagates pose, velocity and bias from that
   last KF when the map updated, otherwise from the previous frame.
2. `src/Tracking.cc:3206–3228`: RECENTLY_LOST predicts IMU state, captures the
   A transaction snapshot, then attempts the visual relatch. The helper does
   not create a new inertial state/preintegration anchor.
3. `src/Tracking.cc:5560–5597`: recovery installs a visual pose and landmark
   matches, runs native pose-only optimization, and requires at least50
   verified observations. Pose-only optimization writes only pose
   (`src/Optimizer.cc:1158`); it does not jointly reconcile velocity, bias,
   marginal prior or the inertial reference keyframe with the recovered pose.
   `RelocalizeWith` deliberately does not update `mnLastRelocFrameId`
   (`src/Tracking.cc:5458`).
4. `src/Tracking.cc:4542–4545`: local tracking seeds temporal observations,
   updates the visual local map, and adds projected matches. Updating the
   visual reference KF (`5422–5425`) is distinct from changing the inertial
   `mCurrentFrame.mpLastKeyFrame`; the existing temporal KF chain is also
   explicitly added at5407 onward.
5. `src/Tracking.cc:4583–4624`: without a recent-relocation frame ID, tracking
   does not enter the temporary pose-only branch. It selects last-frame VI
   only if a previous frame marginal prior and valid integrations exist;
   otherwise it selects last-keyframe VI if that KF is valid and in the same
   map. All12 observed successful visual relatches used `last_keyframe`.
   The selection checks pointer existence, positive finite integration dt,
   valid KF and map membership, not consistency between the recovered visual
   pose and its inherited inertial state/covariance.
6. `src/Optimizer.cc:5492–5507`: last-keyframe VI initializes current pose,
   velocity and biases from this mixed frame state. `5669–5689` fixes the
   previous KF pose, velocity and biases and connects current pose/velocity
   to it with the original keyframe-to-current preintegration. The covariance
   weighting is the inverse of that preintegration covariance
   (`src/G2oTypes.cc:492–509`); no separate relatch handoff is applied.
7. `src/Optimizer.cc:5710–5823`: the native optimizer iteratively refits pose
   and classifies observations. If fewer than30 inliers remain and
   `bRecInit=false`, the existing rescue path reclassifies observations with
   chi2<18 for mono and<24 for stereo (`5826–5851`). The default parameter is
   false (`include/Optimizer.h:67`) and the call omits an override. Thus the
   final16–21 counted observations are not necessarily the strict final
   chi2=5.991 inliers from visual recovery. The log does not count rescue
   admissions separately, so their exact contribution is unknown.
8. `src/Optimizer.cc:5855–5900`: this VI solution overwrites body pose,
   velocity and bias and constructs a fresh `ConstraintPoseImu` marginal
   prior centered on it. `LM_DIAG prior_exists=1` is measured after this step;
   it is not evidence that an old marginal prior was present before the solve.
9. `src/Tracking.cc:4770`: the stricter post-relocation50-inlier gate is not
   armed by coast relatching. At4780, RECENTLY_LOST returns success when the
   inlier count exceeds `mnRecentlyLostMinInliers`. The default is10
   (`include/Tracking.h:355`) and the actual run settings do not override it.
   This branch precedes normal IMU_STEREO's15-inlier gate. The current coast
   comment saying its bar is raised does not match this default configuration.
10. `src/Tracking.cc:3408–3427`: a locally accepted relatch commits the state,
    refreshes the lost timestamp, sets state OK and exits coasting. The A
    outer rollback executes only if local confirmation returns false.
    `include/TrackingStateRollback.h:19–21` preserves timestamp on rejection
    and updates it on acceptance. `src/Tracking.cc:3664` then copies the
    accepted current frame, including its new prior, into the previous frame
    for subsequent propagation/optimization. Rejected states are restored
    before that copy.

## Interpretation and next experiment boundary

The correspondence experiment did produce real geometric recovery hypotheses,
but it also exposed an inherited gap between visual recovery acceptance and
visual-inertial state acceptance. A confirmed transaction is currently allowed
to discard most recovery observations, move several metres, and still be
accepted through the ordinary RECENTLY_LOST threshold. The transaction rollback
correction protects explicit failures; it cannot protect a solve that this old
policy declares successful.

A next repair should make recovery a coherent state transition: preserve the
visual candidate and original propagated state; jointly reconcile pose,
velocity/bias and the temporally valid inertial anchor through a short recovery
window; create a marginal prior only from the accepted coherent solve. Require
final geometric support and residual checks for the recovered pose itself, with
an explicit recovery acceptance policy rather than falling through the ordinary
>10 threshold/rescue path. Do not simply transplant a visually retrieved KF
into the IMU chain, reuse its old preintegration for a different start state,
or discard IMU history. A large visual correction can be correct, so a hard
metre jump cap alone would not solve this.

Existing last-keyframe VI already jointly optimizes the current pose, velocity
and biases. The missing element is a dedicated recovery-window/consistency
acceptance policy, not the absence of any inertial reconciliation. Starting
that optimizer from a changed visual pose and inherited temporal state is not
inherently invalid. Also, all three 16/20/21-inlier examples exceed the normal
15-inlier gate: changing the recent-loss threshold from >10 to >=15 would not
reject them. The missing explicit recovery confirmation remains the issue.

This analysis does not establish that the visual hypothesis is correct against
GT, that calibration is wrong, or that the inherited covariance is numerically
incorrect. Those remain separate checks. No new recovery-window implementation
or experiment was started in this batch.

## Long completion confirms the default gate in an actual11-inlier commit

Long completed all35,842 inputs and official Score2D38.941196, with8 retained
maps and62.50% scored native coverage. Recovery:762 calls,46 hypothesis objects,
74 successful solver returns (repeated iterations can count the same hypothesis),
2 geometrically accepted relatches,2 local commits,0 relatch rollbacks.

At t1010.149294925, cam1 PnP recovers84 visual inliers (33 cam0 /51 cam1).
Local VI enters with39/65 associations, moves0.494549m from the visual pose,
and keeps11/0. It then commits exactly11 inliers, refreshes lost_since to
1010.149294925 and exits coast. This dynamically confirms that the >10 early
RECENTLY_LOST branch bypasses normal stereo's15 threshold. It does not prove
that either pose is correct or that a stronger threshold alone improves the
complete trajectory.

At t1766.399313924, cam1 PnP recovers54 visual inliers (18/36); local VI receives
76/46 associations, moves0.364263m from the visual hypothesis and retains0/30.
This second transaction also commits and refreshes grace.

Long log:
`experiments/lamaria_continuity_batch_20261003/recovery_direct_long/runs/recovery_direct_long_long_full/run.log`
- Lines69279–69283:84→104→11 event, cam1 hypothesis and same-frame commit.
- Lines127038–127043:54→122→30 event.

Reproducible per-sequence plots and JSON:
`experiments/lamaria_continuity_batch_20261003/recovery_direct_medium/recovery_diagnostics/`
`experiments/lamaria_continuity_batch_20261003/recovery_direct_long/recovery_diagnostics/`
Each contains summary.json, recovery_funnel.png and relatch_handoff.png. Their
source is `patches/orbslam3_lamaria_recovery_direct_20261003/analyze_recovery.py`.
The Long handoff plot explicitly shows the11-inlier event and its0.495m
estimate disagreement, not GT error.
