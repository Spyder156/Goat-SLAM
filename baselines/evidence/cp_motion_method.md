# Control-point camera-motion audit

**The user's knot/stop-and-wave observation is strongly supported on these three training sequences.** This was a bounded offline diagnostic. No SLAM source/configuration, estimator inputs, score, trajectory, or existing Rerun was changed.

## What was compared

The sparse annotation files provide timestamps where each control point was observed. Detections for the same point are grouped into visits unless separated by more than 10 seconds. Short has 14 visits; Medium has 18 visits to 17 named points (AA1812 is revisited); Long has 27 visits. CP coordinates and pixel detections are not used in motion calculations.

Calibrated gyro norms and same-map translation from the final saved atlas measure motion. Dense GT speed is shown only as a diagnostic cross-check, never as a detector input. Each visit is compared with one equal-duration non-overlapping control window. Controls are selected by temporal proximity only, processing longest visits first, excluding every annotated visit plus a 5-second margin. All 59 control windows are distinct and disjoint from annotations. Median absolute control separations are 65/84/28 seconds for Short/Medium/Long; some controls are farther away (maximum 335/523/419 seconds). Thus these are temporal, duration-matched comparisons, not randomized or environment-matched trials.

## Results

Each number below is the median across visit-level means or medians, not a pooled frame statistic.

| Sequence | Visits | CP/control gyro (rad/s) | CP/control 1-s translation chord (m/s) | CP/control 5-s net progress (m/s) |
|---|---:|---:|---:|---:|
| Short | 14 | 1.23 / 0.74 | 0.53 / 1.54 | 0.102 / 1.525 |
| Medium | 18 | 1.37 / 0.71 | 0.50 / 1.50 | 0.091 / 1.479 |
| Long | 27 | 1.45 / 0.72 | 0.50 / 1.40 | 0.093 / 1.379 |

All 59 visits have higher mean gyro activity than their control. Translation is lower in all 57 pairs with finite SLAM motion (Long has two CP visits without usable trajectory). Dense GT corroborates the translation contrast: CP versus control medians 0.50/1.58, 0.48/1.53, 0.50/1.41 m/s.

**The sensor is moving during these knots.** Its 1-second translation is about 0.5 m/s while its 5-second net progress is only about 0.1 m/s: motion repeatedly changes direction. This is consistent with waving/scanning in a small area, not a stationary IMU. Camera/body lever arms and human arm motion also matter. A blanket zero-velocity factor would be physically wrong.

## Exploratory scan cue

A first fixed rule, mean gyro >0.6 rad/s AND median 1-second speed <0.35 m/s, mostly failed (only 5/59 visits). This is recorded, not hidden. After seeing that, the follow-up used the same thresholds with **5-second net progress** instead of the 1-second speed. This follow-up is exploratory, not held-out detector validation. No threshold search was run and neither rule consumes CP labels.

| Sequence | Visits overlapping cue for >=3 samples | Cue events | Events overlapping a labelled visit | Events outside labelled visits |
|---|---:|---:|---:|---:|
| Short | 14/14 | 15 | 14 | 1 |
| Medium | 17/18 | 17 | 17 | 0 |
| Long | 24/27 | 25 | 24 | 1 |

Overall 55/59 visits overlap the cue, and 55/57 detected events overlap annotated visits. These are exploratory annotation-overlap statistics, not a demonstrated production precision/recall. The two unlabelled events could be other legitimate scans. Annotations themselves select visibility, so they do not exhaust scan activity. CP visits occupy 1,238/3,878 analysed seconds (31.9%); 1,070/1,217 positive cue seconds (87.9%) lie inside visit spans, and only 29 positive seconds lie more than 5 seconds outside any visit. Events group positive samples with gaps <=3 seconds and require >=3 positive samples.

The missed Medium visit and first missed Long visit have only 3 usable same-map speed samples. The two other missed Long visits have none. Therefore fragmentation limits this cue precisely where recovery is needed. The motion comparison uses final trajectories and centred windows, so a live detector still requires causal implementation and separate validation. Metric scale errors in tiny maps also affect speed thresholds.

## What this can and cannot support

A motion-only scan detector can schedule extra local matching, retain useful keyframes, or allocate more VI refinement to well-observed scan episodes. Select diverse views with sufficient parallax rather than simply adding every blurred frame. A scan does not identify a control point or its coordinates, prove a revisited location, or provide an absolute position constraint. Any estimator benefit remains untested. Do not use training CP coordinates/annotations as inference inputs. Do not clamp the trajectory or add hard zero velocity just because net progress is low.

## Visuals

- `net_progress_knots.png`: all three sequences; green = CP label spans, purple = motion-only cue, blue = five-second net progress. A perfect temporal association would align every purple event with a green span, without missed spans or extra events. Blanks in the blue curve indicate unavailable same-map motion.
- `cp_vs_matched_controls.png`: visit-vs-control scatter using independent gyro and 1-second translation speed.
- `short_motion_timeline.png`, `medium_motion_timeline.png`, `long_motion_timeline.png`: gyro, SLAM/GT speed, annotations, and the failed first stationary-style cue.
- `cp_visits_and_matched_controls.csv`: all 59 pairs with exact windows and metrics.
- `summary.json`, `net_progress_summary.json`, `protocol.json`: machine-readable results/method.
- `audit.py`, `net_progress.py`: reproduction scripts; read cached inputs only and write this diagnostic folder.
