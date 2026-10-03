# Reproducing the retained LaMAria baselines

Current experimental branch handoff (2026-10-04): `docs/RESUME_20261004.md`.
BabyFeatures Long43.73 and its offline native VI45.03 refinement are separate
candidates; they do not supersede the preserved profiles documented here. The
user stopped work for their usage limit; this document is not a replay request.

This is the reproducibility record for the implementations retained for Goat-SLAM. It was checked against local build manifests, source overlays, saved run commands, configurations, and official local score artifacts on 2026-10-03. No estimator was rerun for this document.

Read `GOAL.md`, `ROADMAP.md`, and `EXPERIMENTS.md` before changing an estimator. Preserve the successful implementations as separate profiles. The latest implementation is not the best on every sequence.

## 1. Which implementation produced which result?

| Profile | Sequence | Official local Score2D | Coverage and limits |
|---|---|---:|---|
| `orbslam3_lamaria_history` | Long `sequence_3_17` | **49.881533645273585** | One retained map, zero resets, 35,772/35,842 input poses; drift remains |
| `orbslam3_lamaria_online_radial`, **fixed** config | Short `sequence_1_19` | **67.18837945428926** | One map, 18,343 estimated poses; metric scale remains wrong |
| `orbslam3_lamaria_online_full_v2` | Short `sequence_1_19` | **80.19916100650815** | One map, all 2,999 GT timestamps associated, 14/14 CP within 1 m; brief pose jumps remain |

**67.19 is a Short result, not a Long result.** The v2 method transferred to Medium at 62.764799 and to Long with verified native lens input at 34.458901, both fragmented. The later v3 rollback variant scored 42.593277 on Short. Neither replaces the frozen Short v2 or continuous Long history baseline.

These are local evaluations of individual training recordings, not submitted leaderboard results or category averages. Score2D averages control-point error scores. The evaluator applies one control-point-derived Sim3 after estimation. This does not make raw metric scale correct. Missing poses, control points, and independent map origins must remain explicit.

## 2. Source preservation is more than an upstream commit

The current vendor source is a private, modified ORB-SLAM3 fork. Its local repository HEAD was `8da1194289525c5bb03b5e019cda9510b61a451c`; that identifier alone does **not** identify all files used in the successful builds. The existing base manifests explicitly say:

> Current local fork, including existing fixes; apply only after these hashes match.

All three baseline patch packages use the same 295-file base manifest, SHA-256:

```text
2f83a7b0dd7b6b5e8cdc9695eee46effd6ff0084802800350289889892b7c6a3
```

Preserve the following layers together:

1. **Exact private vendor source**, including files modified or added outside upstream history: the ORB root CMake file, `src/`, `include/`, the rig runner, `cmake_modules/`, and the complete used DBoW2/g2o/Sophus source and licenses. A new clone of stock upstream ORB-SLAM3 is not an interchangeable base.
2. **ELSED source**, compiled directly into the library even when line features are disabled. Observed local HEAD: `1878213b2f5f06a9261d8b1838f53d48e5fd128d`. The three compiled ELSED translation units are `ELSED.cpp`, `EdgeDrawer.cpp`, and `FullSegmentInfo.cpp`.
3. **Profile patches and added files** under `patches/orbslam3_lamaria_history`, `patches/orbslam3_lamaria_online_radial`, and `patches/orbslam3_lamaria_online_full_v2`. Each package is applied directly to the common private base. Do not apply these three packages on top of one another.
4. **Frozen final source overlays and build manifests** for those profiles. They provide an independent record of what actually compiled, including added calibration and rollback headers. Preserve the full overlay, not only the list of files changed by the last patch.
5. **Binary/runtime reconstruction evidence**: the original 295 hashed base files include generated CMake metadata, old object files, and libraries. The current builder validates those files even though the three successful profiles compiled all 35 translation units and reused zero old objects.

The publication layout is:

```text
vendor/ORB_SLAM3/
vendor/ELSED/
baselines/history/{sources/,build_manifest.json}
baselines/radial_fixed/{sources/,build_manifest.json}
baselines/online_full_v2/{sources/,build_manifest.json}
baselines/runtime/vendor_build.tar.xz
baselines/runtime/history.tar.xz
baselines/runtime/radial_fixed.tar.xz
baselines/runtime/online_full_v2.tar.xz
tools/restore_baselines.py
```

`vendor_build.tar.xz` preserves the missing hashed baseline objects, generated build metadata, and dependency libraries. The per-profile runtime bundles retain each exact runner and ORB library. The `radial_fixed` preservation label maps to the original `orbslam3_lamaria_online_radial` build used with the fixed configuration.

Verify the publication bundle, then restore into a new destination:

```bash
python3 tools/restore_baselines.py --verify
python3 tools/restore_baselines.py --destination /absolute/new/Goat-SLAM-working
```

The destination must not already exist. Restoration materializes the complete working layout, including the expected `third_party/` and `build/` names. Run the production build/replay commands below from that restored working directory. The restoration utility and its bundle manifest are the authority for exact archive contents. Do not replace the preserved source with a stock submodule checkout or silently regenerate baseline hashes. Restoration/hash checks and builder dry-runs validate preservation; they do not establish an independently completed fresh source compilation or SLAM replay.

`pipeline/run/build_lamaria.py` is an audited **overlay builder**, not a standalone clean-upstream build system. It reads:

```text
third_party/ORB_SLAM3/build/CMakeFiles/ORB_SLAM3.dir/link.txt
third_party/ORB_SLAM3/build/CMakeFiles/ORB_SLAM3.dir/flags.make
third_party/ORB_SLAM3/build/CMakeFiles/mono_rig_euroc.dir/link.txt
```

It verifies the base manifest, copies source into an isolated overlay, applies one patch package, compiles in Docker, and writes a manifest pinning the image, inputs, source, runner, and library. A fresh CMake build from source may be a useful future portability path, but is not automatically the same historical build and has not been validated by this documentation audit.

Do not modify upstream/vendor files in place. New estimator changes belong in a new patch/profile or an explicit private staging build.

## 3. Exact historical build fingerprints

All three profiles used Docker image `insv/orbslam3:gdb` with actual image ID:

```text
sha256:e81cd138e05acfdfccb29585a1d8923f33e04d1ce087a461203130befa3524f1
```

The exact image is now backed up locally at:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/backups/github_initial_20261003/orbslam3-gdb.tar.gz
```

Its archive SHA-256 is `7d7aab9118349b4fa784bbfeebcc8b3c3c2eccbb45f722dd164aa2a9466d6595`.
`baselines/runtime_image.json` records the image ID and restore command. The
403,082,735-byte image archive is outside GitHub; retain it with the project
backups. Frozen per-profile settings are also copied into each profile's
`baselines/<profile>/settings.yaml`.

The local image has no registry `RepoDigest`. Its name alone is not a reproducible dependency. Preserve a Docker image archive in dedicated project artifact storage and verify its loaded image ID. A source Git checkout does not contain this runtime image. The original image's exact Dockerfile is not present in the inspected project; similarly named legacy BA Dockerfiles are not substitutes.

| Profile | Patch SHA-256 | `libORB_SLAM3.so` SHA-256 |
|---|---|---|
| history | `037b9dd52b74f000755b8ac672c47bbde9ae2ba2cb74d66a1a5009c1487291d6` | `f50d9a99a882b379dd855d06ada83f4c32715c19a1ca226d1fd56194bc41ee0f` |
| online_radial | `25b7bd0fdaafd6a92a9994ef39015a3370b2453e9a37ca37d8d3ddd7dcf8a4f5` | `49d36ab726b24eedb534097bdd5d24a10491ded89506e9298bfa16a080ed14e2` |
| online_full_v2 | `bd9f9c5eb0f664b08f322e928c4e7579b0624e84b901f8da2f2620df2e4aba2d` | `7664755942310a1513cde99aeb47d3c48c4cf17a5412196b970b6315018c3414` |

All three runner binaries have SHA-256:

```text
a0efe826e73155b3f307882cb5176ec630eca9bd2147cba04cbcd395eec93f66
```

This shared runner hash does not mean the estimators are identical: their dynamically loaded ORB libraries differ. The runner source hash is `60d7c288f10653510a2b1b6262e277abb0ed3cc96bdd080e3aed36088c386d23`.

Other required runtime files:

| File | SHA-256 |
|---|---|
| `Thirdparty/DBoW2/lib/libDBoW2.so` | `9c2082e6b7c5370606e597b20ac53010366353d14ba1b795a5bbc7ed17a77540` |
| `Thirdparty/g2o/lib/libg2o.so` | `4e3dbc9b6b041b85797727cc14aea48293ddf9750ee6e8de438a22459a83c61c` |
| `Vocabulary/ORBvoc.txt` | `f8dd027f7a6cb88129821341194d7f2c75b77b3394257ddd0d2229863d1a3570` |

The text vocabulary is 145,250,924 bytes. If distributed separately or compressed, verify the decompressed file against that hash. It is still required even though the image features are precomputed ALIKED descriptors.

The C++ builds used `-march=native`. Preserve the original binaries for exact historical comparison; another CPU/compiler can generate different binaries and may require a rebuild. A bit-identical binary alone does not guarantee a bit-identical multithreaded SLAM trajectory.

## 4. Configurations are part of the implementation

These committed configurations exactly match the saved run configurations:

| Baseline | Configuration | SHA-256 |
|---|---|---|
| Long 49.88 | `configs/orbslam3_lamaria/stereo_inertial_continuous.yaml` | `5c9a16293a8715d5c756f199ae240d7df8203a55c6deee273ccc7868f5e70786` |
| Short 67.19 | `configs/orbslam3_lamaria/online_radial_20261003/fixed.yaml` | `3d2cc2708509027af30982020299ca1d4258a1073bb21f4404f9d57a44ddf7bf` |
| Short 80.20 | `configs/orbslam3_lamaria/online_full_20261003/short.yaml` | `b75b3c555e06646bafe4a00fedcfed4ec19a958678a9a8f248db8b7b6f0f2757` |

The `online_full_20261003` directory name does not mean its configuration belongs only to the unsuccessful v1 binary. The successful v2 replay used that exact `short.yaml` with the v2 build.

The fixed/online radial experiments share a build but have different configuration modes. Do not reproduce the 67.19 fixed control using `online.yaml`.

Long history used the inherited calibration available at the time. The later Long v2 transfer replaced its lens inputs with verified factory-native parameters because the inherited separate-focal COLMAP fit violated the newer single-focal camera contract. Therefore those two Long runs are not a calibration-controlled comparison. Preserve the original Long configuration when reproducing its 49.88 result.

## 5. Required external inputs and environments

Dataset images, VRS recordings, learned feature caches, weights, IMU CSVs, Rerun recordings, and complete run logs are not ordinary source-code dependencies to silently commit. Their local locations and hashes are recorded in each run's `command.json`, `result.json`, `config/imu_manifest.json`, and score provenance.

For the two Short baselines, the input root is:

```text
/media/raghav/HardDrive1/Mecka/lamaria/bench_basalt/orbslam/current_pipeline_20261002/sequence_1_19
```

Required contents include `euroc/`, the underlying targets of both camera image symlinks, `aliked_cam0/`, `aliked_cam1/`, `supported_timestamps.txt`, `calibrated_imu.csv`, and its `.manifest.json`. Short has 18,352 native image timestamps but only 18,351 IMU-supported inputs. Do not add the unsupported last frame to the estimator or remove it from source coverage accounting.

Short calibrated IMU SHA-256:

```text
fbeb0d589d4caf13b3867e937bf225806ac6acb7546426675cad802610185b59
```

Short supported timestamp SHA-256:

```text
ebbcc1d26d215a491c711ed63c84c94236fd668957117bb639a0bec38a22bcf0
```

Long uses `euroc_native_3_17`, `aliked_kp_3_17`, and `aliked_kp_3_17_cam1` beneath `/media/raghav/HardDrive1/Mecka/lamaria/bench_basalt/orbslam`. Its calibrated IMU is an older external audit artifact:

```text
/home/raghav/workspace/MeckaAI/lamaria_audit_20261001/native_stereo/calibrated_imu/production_data.csv
```

Long calibrated IMU SHA-256 is `3f4ff94e86ce0c4e6f056facd665f50a234c59f24b668f2d5bd1acaa0f6e3803`; its full 35,842 timestamp list SHA-256 is `796b89976b02b441b34d1203196b14b2596a4b5d23e71ea4a165bc120fe0442d`. Preserve the adjacent IMU manifest when relocating this CSV. The in-repository `pipeline/datasets/rectify_aria_imu.py` implements factory/time correction, but regeneration still requires the original raw IMU and VRS calibration. Do not rectify an already corrected CSV again.

The estimator consumes precomputed 256-bit binarized ALIKED keypoint caches. Re-extracting features with a newer network/package/weight or a different binarization is a new input experiment, not an exact baseline replay. The saved cache directories are required until the complete feature-generation recipe and weight fingerprints are independently frozen.

The official scoring toolkit is external to this project at:

```text
/home/raghav/workspace/MeckaAI/third_party/lamaria_toolkit
```

Saved toolkit commit: `aa811b3281981930f30328d3cf5fad04a331e93e`. Use an unchanged checkout at that revision and pass its path explicitly with `--toolkit`. The scoring wrapper checks the toolkit's Git state; copying only Python files without its provenance is not currently equivalent. Per-sequence evaluation assets live under `/media/raghav/HardDrive1/Mecka/lamaria/sequence_1_19` or `sequence_3_17` and include native calibration plus dense and sparse GT. GT is for scoring/display only.

Observed host environment used for postprocessing:

```text
/home/raghav/miniconda3/envs/lamaria/bin/python
Python 3.11.16
numpy 2.4.6
scipy 1.17.1
opencv-python 5.0.0.93
rerun-sdk 0.33.0
pycolmap 3.13.0.dev0
projectaria-tools 2.3.0
```

The saved Short scoring provenance additionally records `pyceres 2.6` and `lamaria 0.1.0`. These are host package versions, not the C++ OpenCV version inside the ORB Docker image. A development-version package string is insufficient to recreate a private wheel; preserve its wheel/source or the existing environment separately. Torch/ALIKED extraction dependencies are unnecessary for replaying already frozen feature caches but necessary for rebuilding those inputs.

## 6. Rebuild commands on the preserved workstation layout

Run from the project checkout after restoring the vendor/build prerequisites and required runtime image. These are the existing production builder commands, not newly invented estimator math:

```bash
python3 pipeline/run/build_lamaria.py \
  --patch-dir patches/orbslam3_lamaria_history \
  --out build/orbslam3_lamaria_history --jobs 4 --dry-run

python3 pipeline/run/build_lamaria.py \
  --patch-dir patches/orbslam3_lamaria_online_radial \
  --out build/orbslam3_lamaria_online_radial --jobs 4 --dry-run

python3 pipeline/run/build_lamaria.py \
  --patch-dir patches/orbslam3_lamaria_online_full_v2 \
  --out build/orbslam3_lamaria_online_full_v2 --jobs 4 --dry-run
```

The dry-run verifies the base hashes and describes the build without compiling. Remove `--dry-run` only when a rebuild is intended. Preserve historical binaries/manifests before writing to their build output directories; a new candidate should use a distinct output path. Do not confuse a successful dry-run with a completed compiler build or a benchmark replay.

## 7. Replay the successful Short v2

Choose a genuinely new output name. The example retains the original four-CPU cap, diagnostic logging and GDB launcher; these settings can affect scheduling and should be recorded.

```bash
LAMARIA_PYTHON=/home/raghav/miniconda3/envs/lamaria/bin/python
SHORT_DATA=/media/raghav/HardDrive1/Mecka/lamaria/bench_basalt/orbslam/current_pipeline_20261002/sequence_1_19
RUN_OUTPUT=experiments/REPLAY_SHORT_V2_UNIQUE_NAME

"$LAMARIA_PYTHON" -B pipeline/run/run_lamaria.py \
  --build build/orbslam3_lamaria_online_full_v2 \
  --config configs/orbslam3_lamaria/online_full_20261003/short.yaml \
  --dataset "$SHORT_DATA/euroc" \
  --timestamps "$SHORT_DATA/supported_timestamps.txt" \
  --kp0 "$SHORT_DATA/aliked_cam0" --kp1 "$SHORT_DATA/aliked_cam1" \
  --calibrated-imu "$SHORT_DATA/calibrated_imu.csv" \
  --temporal-associations --cpus 4 --match-diagnostics --debugger --skip-viz \
  --out "$RUN_OUTPUT"
```

Append `--dry-run` for input validation without creating a run. For the 67.19 Short control, change only the selected baseline pair to:

```text
--build build/orbslam3_lamaria_online_radial
--config configs/orbslam3_lamaria/online_radial_20261003/fixed.yaml
```

Use a different new output directory for that replay. The successful online calibration has been observed once on this Short sequence; a rerun may differ because local mapping and optimizer scheduling are multithreaded. The saved 80.199161 artifact is historical evidence, not a promised deterministic score.

## 8. Replay the continuous Long history baseline

```bash
LAMARIA_PYTHON=/home/raghav/miniconda3/envs/lamaria/bin/python
LONG_INPUT=/media/raghav/HardDrive1/Mecka/lamaria/bench_basalt/orbslam

"$LAMARIA_PYTHON" -B pipeline/run/run_lamaria.py \
  --build build/orbslam3_lamaria_history \
  --config configs/orbslam3_lamaria/stereo_inertial_continuous.yaml \
  --dataset "$LONG_INPUT/euroc_native_3_17" \
  --timestamps "$LONG_INPUT/euroc_native_3_17/timestamps.txt" \
  --kp0 "$LONG_INPUT/aliked_kp_3_17" \
  --kp1 "$LONG_INPUT/aliked_kp_3_17_cam1" \
  --calibrated-imu /home/raghav/workspace/MeckaAI/lamaria_audit_20261001/native_stereo/calibrated_imu/production_data.csv \
  --temporal-associations --cpus 4 --skip-viz \
  --out experiments/REPLAY_LONG_HISTORY_UNIQUE_NAME
```

The original history command rendered afterward through the launcher's normal output flow; `--skip-viz` here defers that packaging step. It does not alter the estimator. The original history run did not request the later per-frame match diagnostics/GDB switches. Read its preserved `command.json` for the exact historical container command.

## 9. Score raw output and display the exact evaluated geometry

For the Short replay above:

```bash
"$LAMARIA_PYTHON" -B pipeline/run/score_lamaria.py \
  --run "$RUN_OUTPUT" \
  --assets /media/raghav/HardDrive1/Mecka/lamaria/sequence_1_19 \
  --sequence sequence_1_19 \
  --toolkit /home/raghav/workspace/MeckaAI/third_party/lamaria_toolkit \
  --source-timestamps "$SHORT_DATA/euroc/timestamps.txt" \
  --map-scope largest

"$LAMARIA_PYTHON" -B pipeline/viz/make_lamaria_output.py \
  --run-dir "$RUN_OUTPUT" \
  --output-dir "$RUN_OUTPUT/evaluation" \
  --dataset "$SHORT_DATA/euroc" \
  --timestamps "$SHORT_DATA/euroc/timestamps.txt" \
  --config "$RUN_OUTPUT/config/settings.yaml" \
  --gt /media/raghav/HardDrive1/Mecka/lamaria/sequence_1_19/gt_dense.txt \
  --evaluation-score "$RUN_OUTPUT/lamaria_score/scores.json"
```

For Long, use `sequence_3_17`, its corresponding assets and full timestamp list. `--map-scope largest` selects a map without consulting GT and reports other maps as omitted; it must never silently join them. A result expected to be single-map can instead use `--map-scope single`, which refuses fragmentation. The scorer refuses to overwrite existing score output.

The display command uses the saved evaluator Sim3 for trajectory, landmarks and rig display. It does not separately fit dense GT. Full native image timestamps can include images without an estimated pose; that is deliberate. Preserve and inspect `coverage.json`, the score's selection/denominator fields, and Rerun verification before claiming a complete trajectory.

## 10. Archived evidence and path portability

The complete local experiment root is:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments
```

The three original baseline directories relative to it are:

```text
lamaria_history_full_20261002
lamaria_radial_fixed_short_full_20261003/runs/lamaria_radial_fixed_short_full_20261003_short_full
lamaria_online_full_v2_short_full_20261003/runs/lamaria_online_full_v2_short_full_20261003_short_full
```

Each contains raw atlas geometry, the input/config/build manifests, complete log, coverage, and `lamaria_score/` provenance. The verified 80.20 display is `lamaria_online_full_v2_short_full_20261003/online_full_evaluation.rrd`. Reruns and datasets remain external artifacts, not Git source files.

The original project's source and experiment directory must not become dependencies again. Archived command strings intentionally retain the locations used at execution time; do not execute those strings verbatim. Current launchers resolve archived experiment paths through `pipeline/artifact_paths.py` and use the relocated storage. That helper's destination, the suite JSON, external dataset/IMU paths, the Conda Python path, and the scoring toolkit default are workstation-specific and need explicit configuration on another machine.

The copied CMake and ELF paths `/orb`, `/work`, `/src`, and `/build` are Docker mount paths. The current launchers map this project's own source/build directories there. They are not paths into the original checkout. The default build/run profile remains `history`; successful Short v2 must be selected explicitly.

The separate offline COLMAP/VI experiments require additional dependencies: copied COLMAP source/build, the local `fisheye-slam` image, and private patched Ceres for the later solvers. They are not required to replay the three online baselines above. Preserve `pipeline/vi_ba_lamaria`, `patches/ceres_lamaria_parallel_2_2`, their build manifests and experiment execution manifests if retaining those experiments. Their own source/binary/environment provenance must not be conflated with the ORB runtime image.

## 11. Reproduction acceptance criteria

- The restored private base and profile source hashes match the published manifests.
- The required runtime image and shared libraries are available; the runner loads the intended profile library.
- Config, corrected IMU, timestamp list and feature cache inputs match the intended baseline. No GT enters initialization, tracking, calibration or BA.
- A run finishes normally and exports all surviving atlas maps with distinct gauges. Count internal map splits, resets, startup omissions and coasting separately.
- Score with the unchanged official toolkit and full recording denominators. Keep the historical score and new replay result separate when nondeterminism changes the outcome.
- Render the exact saved evaluation transform, both camera streams and actual landmarks, and verify the recording.
- A clean Git checkout with restored local bundles is not the same claim as a verified fresh-machine build from public dependencies. State which one was tested.
