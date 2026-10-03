#!/usr/bin/env python3
"""Run calibrated LaMAria SLAM into one new experiments/ output folder.

Use --dry-run to validate inputs and print the plan without creating a run.
Images and ALIKED features remain untouched. The dataset fixture substitutes
only the factory-corrected IMU CSV. No ground truth reaches the estimator.
"""
import argparse
from datetime import datetime, timezone
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import relocated_path
import re
import shutil
import subprocess

from build_lamaria import PROJECT, sha256


def timestamp_list(path):
    values = [int(line.strip()) for line in Path(path).read_text().splitlines() if line.strip() and not line.lstrip().startswith("#")]
    if len(values) < 2 or any(b <= a for a, b in zip(values, values[1:])):
        raise ValueError("Camera timestamps must be a strictly increasing integer-nanosecond list")
    return values


def calibrated_manifest(csv_path, manifest_path, first, last):
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("output_sha256") != sha256(csv_path):
        raise ValueError("Calibrated IMU hash does not match its manifest")
    if manifest.get("ground_truth_used") is not False or manifest.get("imu_label") != "imu-right":
        raise ValueError("Require factory-only imu-right calibration manifest for this configuration")
    if not (manifest["first_timestamp_ns"] < first and manifest["last_timestamp_ns"] > last):
        raise ValueError("Calibrated IMU does not bracket the selected camera timestamps")
    return manifest


def docker_command(args, run_dir, camera_dirs, imu_csv, image_targets=()):
    mounts = [(args.build.resolve(), "/build", "ro"), (args.vendor.resolve(), "/orb", "ro"), (run_dir, "/run", "rw")]
    # Preserve absolute symlink targets in the small EuRoC fixture. Mount just
    # the needed directories/files, not a whole writable dataset or home tree.
    readonly = {*camera_dirs, *image_targets, args.kp0.resolve(), args.kp1.resolve()}
    if not imu_csv.is_relative_to(run_dir):
        readonly.add(imu_csv)
    mounts.extend((path, str(path), "ro") for path in sorted(readonly))
    command = ["docker", "run", "--rm", "--name", "lamaria-" + run_dir.name,
               "--network", "none", "--cpus", str(args.cpus), "--ulimit", "core=0",
               "--user", f"{os.getuid()}:{os.getgid()}"]
    for host, destination, mode in mounts:
        command += ["-v", f"{host}:{destination}:{mode}"]
    if getattr(args, "match_diagnostics", False):
        command += ["-e", "LAMARIA_MATCH_DIAGNOSTICS=1", "-e", "LAMARIA_MATCH_DIAG_EVERY=1"]
    if getattr(args, "baby_features", False):
        command += ["-e", "LAMARIA_BABY_FEATURES=1"]
    program = ["/build/bin/stereo_lamaria_euroc",
               "/orb/Vocabulary/ORBvoc.txt", "/run/config/settings.yaml", "/run/input/euroc",
               "/run/config/timestamps_ns.txt", run_dir.name]
    debugger = getattr(args, "debugger", False)
    command += ["-w", "/run", "-e", "LD_LIBRARY_PATH=/build/lib:/orb/Thirdparty/DBoW2/lib:/orb/Thirdparty/g2o/lib:/usr/local/lib",
                "-e", "KP_DIR=" + str(args.kp0.resolve()), "-e", "KP_DIR1=" + str(args.kp1.resolve()),
                "-e", "LAMARIA_TEMPORAL_ASSOCIATIONS=" + str(int(getattr(args, "temporal_associations", True))),
                "--entrypoint", "/usr/bin/gdb" if debugger else program[0], args.image]
    if debugger:
        command += ["--batch", "--return-child-result", "-ex", "set pagination off",
                    "-ex", "set print thread-events off", "-ex", "set disable-randomization off",
                    "-ex", "run", "-ex", "thread apply all bt", "--args", *program]
    else:
        command += program[1:]
    return command


def run(args):
    run_dir = args.out.resolve()
    skip_viz = getattr(args, "skip_viz", False)
    if run_dir.exists():
        raise FileExistsError(f"Run already exists; choose a new --out to preserve its log: {run_dir}")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", run_dir.name):
        raise ValueError("Run directory name must contain only letters, digits, dot, dash and underscore")
    times = timestamp_list(args.timestamps)
    config = args.config.read_text()
    if not re.search(r"^Rig\.non_overlapping:\s*0\s*(?:#.*)?$", config, re.M):
        raise ValueError("LaMAria stereo/hybrid settings must set Rig.non_overlapping: 0")
    if "Fisheye624" not in config:
        raise ValueError("This launcher requires the calibrated Fisheye624 configuration")
    manifest_path = args.build / "build_manifest.json"
    build = json.loads(manifest_path.read_text())
    for relative, key in (("lib/libORB_SLAM3.so", "library_sha256"), ("bin/stereo_lamaria_euroc", "runner_sha256")):
        if sha256(args.build / relative) != build[key]:
            raise ValueError(f"Build artifact changed since its manifest: {relative}")
    if args.image != build["image"]:
        raise ValueError("Runtime image must match the recorded build image")
    if args.vendor.resolve() != Path(build["vendor"]).resolve():
        raise ValueError("Runtime vendor path must match the build manifest")
    camera_dirs = [(args.dataset / "mav0" / f"cam{camera}" / "data").resolve(strict=True) for camera in (0, 1)]
    image_targets = set()
    for directory in camera_dirs:
        for stamp in times:
            image_path = directory / f"{stamp}.png"
            if not image_path.is_file():
                raise ValueError(f"Missing input image: {image_path}")
            # EuRoC .png entries can themselves be links to original .jpg
            # files. Their resolved parent must also be visible in Docker.
            image_targets.add(image_path.resolve(strict=True).parent)
    for directory in (args.kp0, args.kp1):
        if not directory.is_dir():
            raise ValueError(f"Missing ALIKED feature directory: {directory}")
    imu_manifest = None
    if args.calibrated_imu:
        imu_csv = args.calibrated_imu.resolve(strict=True)
        source_manifest = args.imu_manifest or args.calibrated_imu.with_suffix(args.calibrated_imu.suffix + ".manifest.json")
        imu_manifest = calibrated_manifest(imu_csv, source_manifest, times[0], times[-1])
    else:
        if not args.vrs:
            raise ValueError("Supply --vrs to rectify raw IMU, or --calibrated-imu and its factory manifest")
        imu_csv = run_dir / "input/calibrated_imu.csv"
        source_manifest = imu_csv.with_suffix(".csv.manifest.json")
    raw_imu = args.raw_imu or args.dataset / "mav0/imu0/data.csv"
    correction_command = [str(args.imu_python), "-B", str(PROJECT / "pipeline/datasets/rectify_aria_imu.py"),
                          "--vrs", str(args.vrs), "--input-csv", str(raw_imu), "--output-csv", str(imu_csv),
                          "--camera-timestamps", str(args.timestamps)] if not args.calibrated_imu else None
    command = docker_command(args, run_dir, camera_dirs, imu_csv, image_targets)
    export_command = [str(args.viz_python), "-B", str(PROJECT / "pipeline/viz/make_lamaria_output.py"),
                      "--run-dir", str(run_dir), "--dataset", str(args.dataset.resolve()),
                      "--timestamps", str(run_dir / "config/timestamps_ns.txt"),
                      "--config", str(run_dir / "config/settings.yaml")]
    if args.gt:
        export_command += ["--gt", str(args.gt.resolve())]
    coverage_command = [str(args.viz_python), "-B", str(PROJECT / "pipeline/run/evaluate_lamaria_coverage.py"),
                        "--run-dir", str(run_dir), "--out", str(run_dir / "coverage.json")]
    record = {"run": run_dir.name, "expected_frames": len(times), "first_timestamp_ns": times[0],
              "last_timestamp_ns": times[-1], "expected_duration_s": (times[-1] - times[0]) * 1e-9,
              "command": command, "imu_correction_command": correction_command,
              "export_command": export_command, "coverage_command": coverage_command, "build_manifest": build,
              "source_config_sha256": sha256(args.config), "source_timestamps_sha256": sha256(args.timestamps),
              "feature_directories": [str(args.kp0.resolve()), str(args.kp1.resolve())],
              "visualization_skipped": skip_viz,
              "match_diagnostics": getattr(args, "match_diagnostics", False),
              "baby_features": getattr(args, "baby_features", False),
              "debugger": getattr(args, "debugger", False),
              "ground_truth_used_by_estimator": False}
    if args.dry_run:
        print(json.dumps(record, indent=2))
        return 0
    image_id = subprocess.check_output(["docker", "image", "inspect", "--format", "{{.Id}}", args.image], text=True).strip()
    if image_id != build["image_id"]:
        raise ValueError("Docker image tag now identifies a different image than the build")
    # Do all input checks before claiming the run directory; existing runs are
    # never overwritten, even after a failed or interrupted replay.
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "config").mkdir()
    (run_dir / "viz").mkdir()
    fixture = run_dir / "input/euroc/mav0"
    for camera, directory in enumerate(camera_dirs):
        (fixture / f"cam{camera}").mkdir(parents=True)
        (fixture / f"cam{camera}/data").symlink_to(directory, target_is_directory=True)
    (fixture / "imu0").mkdir()
    # An internal relative link survives the run directory's /run container mount.
    link_target = os.path.relpath(imu_csv, fixture / "imu0") if imu_csv.is_relative_to(run_dir) else str(imu_csv)
    (fixture / "imu0/data.csv").symlink_to(link_target)
    shutil.copy2(args.config, run_dir / "config/settings.yaml")
    # Normalize comments/blank lines away: runner filenames are literal ns lines.
    (run_dir / "config/timestamps_ns.txt").write_text("".join(str(t) + "\n" for t in times))
    shutil.copy2(manifest_path, run_dir / "config/build_manifest.json")
    if correction_command:
        with (run_dir / "imu_calibration.log").open("w") as log:
            subprocess.run(correction_command, stdout=log, stderr=subprocess.STDOUT, check=True)
        imu_manifest = calibrated_manifest(imu_csv, source_manifest, times[0], times[-1])
    shutil.copy2(source_manifest, run_dir / "config/imu_manifest.json")
    record["imu_manifest"] = imu_manifest
    record["started"] = datetime.now(timezone.utc).isoformat()
    (run_dir / "command.json").write_text(json.dumps(record, indent=2) + "\n")
    try:
        with (run_dir / "run.log").open("w") as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                       timeout=args.timeout if args.timeout > 0 else None)
        returncode = completed.returncode
    except subprocess.TimeoutExpired:
        subprocess.run(["docker", "stop", "--time", "5", "lamaria-" + run_dir.name], check=False)
        returncode = 124
    record.update({"finished": datetime.now(timezone.utc).isoformat(), "returncode": returncode})
    (run_dir / "result.json").write_text(json.dumps(record, indent=2) + "\n")
    print(f"Estimator exited {returncode}; raw outputs: {run_dir}", flush=True)
    # Export even a partial trajectory when available, retaining the failed run
    # status. The exporter owns explicit ns conversion and shared world geometry.
    trajectory = run_dir / ("f_" + run_dir.name + ".txt")
    raw_export_returncode = 0 if trajectory.is_file() and trajectory.stat().st_size else 1
    record["raw_export_returncode"] = raw_export_returncode
    export_returncode = None if skip_viz else 1
    if not raw_export_returncode and not skip_viz:
        with (run_dir / "export.log").open("w") as log:
            exported = subprocess.run(export_command, stdout=log, stderr=subprocess.STDOUT)
        record["export_returncode"] = exported.returncode
        export_returncode = exported.returncode
        (run_dir / "result.json").write_text(json.dumps(record, indent=2) + "\n")
        if exported.returncode:
            print(f"Export failed; see {run_dir / 'export.log'}", file=sys.stderr)
    elif raw_export_returncode:
        print("No trajectory was exported; run output contract is incomplete", file=sys.stderr)
    record["export_returncode"] = export_returncode
    (run_dir / "result.json").write_text(json.dumps(record, indent=2) + "\n")
    # Account for every input frame and every retained coordinate frame, even
    # when estimation/export failed. A small largest-map RRD must not conceal
    # resets, cleared history, or independent atlas segments.
    with (run_dir / "coverage.log").open("w") as log:
        coverage = subprocess.run(coverage_command, stdout=log, stderr=subprocess.STDOUT)
    record["coverage_returncode"] = coverage.returncode
    (run_dir / "result.json").write_text(json.dumps(record, indent=2) + "\n")
    if coverage.returncode:
        print(f"Coverage accounting failed; see {run_dir / 'coverage.log'}", file=sys.stderr)
    else:
        summary = json.loads((run_dir / "coverage.json").read_text())
        print(f"Saved poses for {summary['exported_pose_fraction']:.2%} of input frames in "
              f"{summary['independent_segments_exported']} coordinate frame(s); "
              f"{summary['completed_active_map_resets']} active-map resets. "
              f"Details: {run_dir / 'coverage.json'}", flush=True)
    return returncode or raw_export_returncode or export_returncode or coverage.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=relocated_path, help="Raw EuRoC-layout LaMAria dataset")
    parser.add_argument("--timestamps", required=True, type=relocated_path, help="One integer camera timestamp in ns per line")
    parser.add_argument("--kp0", required=True, type=relocated_path)
    parser.add_argument("--kp1", required=True, type=relocated_path)
    parser.add_argument("--out", required=True, type=relocated_path, help="New experiments/<run_name> folder")
    parser.add_argument("--vrs", type=relocated_path)
    parser.add_argument("--raw-imu", type=relocated_path, help="Default: dataset/mav0/imu0/data.csv")
    parser.add_argument("--calibrated-imu", type=relocated_path, help="Reuse previously factory-corrected CSV instead of rectifying")
    parser.add_argument("--imu-manifest", type=relocated_path, help="Default: calibrated CSV path + .manifest.json")
    parser.add_argument("--imu-python", type=relocated_path, default=Path(sys.executable), help="Python with numpy and projectaria_tools")
    parser.add_argument("--viz-python", type=relocated_path, default=Path(sys.executable), help="Python with numpy/scipy/OpenCV/Rerun")
    parser.add_argument("--gt", type=relocated_path, help="Optional cam0 TUM ground truth in seconds; scoring/display only")
    parser.add_argument("--config", type=relocated_path, default=PROJECT / "configs/orbslam3_lamaria/stereo_inertial_continuous.yaml")
    parser.add_argument("--build", type=relocated_path, default=PROJECT / "build/orbslam3_lamaria_history")
    parser.add_argument("--vendor", type=relocated_path, default=PROJECT / "third_party/ORB_SLAM3")
    parser.add_argument("--image", default="insv/orbslam3:gdb")
    parser.add_argument("--cpus", type=float, default=4)
    parser.add_argument("--temporal-associations", action=argparse.BooleanOptionalAction, default=True,
                        help="Carry camera-local temporal landmark matches into geometric optimization")
    parser.add_argument("--timeout", type=float, default=0, help="Wall-clock timeout in seconds; 0 means unlimited")
    parser.add_argument("--skip-viz", action="store_true",
                        help="Diagnostic run: retain raw SLAM exports and coverage, defer Rerun rendering")
    parser.add_argument("--match-diagnostics", action="store_true",
                        help="Log local-map matching diagnostics every frame; matching thresholds remain unchanged")
    parser.add_argument("--baby-features", action="store_true",
                        help="Enable the experimental passive BabyFeature tracker and SOS-only visual-inertial constraints")
    parser.add_argument("--debugger", action="store_true",
                        help="Run under batch GDB and retain all-thread backtraces on failure; nonzero debugger exits remain failures")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.cpus <= 0 or args.timeout < 0:
        parser.error("--cpus must be positive and --timeout nonnegative")
    try:
        code = run(args)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f"Run failed: {exc}\n")
    raise SystemExit(code)


if __name__ == "__main__":
    main()
