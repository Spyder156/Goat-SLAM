# SLAM

Dual-fisheye visual-inertial SLAM for Insta360-class rigs.
Target: Hilti x Trimble SLAM Challenge 2026.

LaMAria full-sequence build, launch commands, validation and known limits:
`docs/LAMARIA_CONTINUITY.txt`. The LaMAria launchers default to the validated
history/continuity profile; explicit older build/config overrides remain available.

Project direction, research plan and verified results: `GOAL.md`, `ROADMAP.md`,
and `EXPERIMENTS.md`.

## Project-owned experiment storage

`experiments/` points only to this project's dedicated storage:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments
```

It is not shared with the original SLAM project's experiment directory. Do not
point it back there. The 58 LaMAria experiment directories were moved intact on
2026-10-03; no forwarding links were left in the old location. Historical
commands and provenance retain the paths used when they executed. Runtime
readers explicitly resolve those archived paths to the current location.
Relocation checks and the current visualization index are in
`maintenance/relocation_20261003/`. See `docs/ARTIFACT_STORAGE.md` for details.

## Layout

| dir | what |
|---|---|
| `third_party/` | upstream repos, pinned. **Never edit directly** — see `patches/` |
| `patches/`     | our diffs against `third_party/`, plus files we added |
| `build/`       | build trees (gitignored) |
| `pipeline/`    | our code, by stage |
| `configs/`     | per-dataset, per-estimator configs |
| `experiments/` | one folder per run, per the `docs/OUTPUT.md` contract |
| `docs/`        | OUTPUT.md (run contract), SOLUTIONS.md, experiments.md, SLAMS.md |

## pipeline/

| stage | what |
|---|---|
| `datasets/`    | .insv / rosbag -> our folder layout, EuRoC converters |
| `step0_calib/` | per-unit intrinsics + rig extrinsics + IMU metric scale, from images alone |
| `vi_ba/`       | visual-inertial bundle adjustment (Ceres): IMU preint + relpose + roll/pitch factors |
| `run/`         | estimator runners, bake-off, scoring |
| `viz/`         | run-output builder (`docs/OUTPUT.md` contract) |
| `okvis_probe/` | convention/geometry probe for the OKVIS family |

## Rules
- Every run produces a folder per `docs/OUTPUT.md`. No ad-hoc outputs.
- Upstream repos are read-only; changes live in `patches/`.
- Data lives in `Data/` at the repo root, gitignored.
