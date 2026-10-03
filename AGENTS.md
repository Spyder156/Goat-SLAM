# Working on Goat-SLAM

Start with `docs/HANDOFF.md`, then `GOAL.md`, `ROADMAP.md`, and `EXPERIMENTS.md`.
Read `docs/BASELINE_REPRODUCTION.md` before rebuilding or replaying a baseline.

- Preserve the frozen Short 80.199161 v2, Short 67.188379 fixed-intrinsics, and
  Long 49.881534 history implementations. The v2 transfer fragments Medium and
  Long; newest does not mean best everywhere.
- `baselines/` and `vendor/` are preservation snapshots. Develop a new named
  patch/profile and build directory instead of overwriting a retained baseline.
  Never edit working `third_party/` checkouts directly.
- Ground truth is evaluation-only. Never use it to initialize, calibrate, match,
  or stitch independent maps. Report coverage and fragmentation with scores.
- Keep work and artifacts in this project's namespace. The original
  INSV_STITCHING project is independent and must not be modified or required.
- Preserve matching/reprojection acceptance bounds unless an experiment
  explicitly investigates them. Current priority is recovery correspondence
  quality and refinement continuity; proposed fixes are not measured results.
- Make runs uniquely named. Record inputs, configuration, source/build hashes,
  logs, scores, and diagnostic paths. Do not overwrite prior runs.
- After a major experiment, explain the change, tests, positive/negative result,
  remaining limitation, and what the user should inspect in the visualizations.
  Give plain absolute local paths. Pause for discussion at major crossroads.
- Use small, focused commits with short messages and no added assistant credits.
