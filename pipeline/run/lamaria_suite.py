#!/usr/bin/env python3
"""Validate cached LaMAria inputs and prepare isolated experiments; never run SLAM.

Startup and loss-context clips restart initialization. Only loss-prefix retains
the earlier image/IMU history; medium full is required for a loop-closure trial.
"""
import argparse
import contextlib
import io
import json
from pathlib import Path
import shlex
import shutil
import struct

from build_lamaria import PROJECT, sha256
from run_lamaria import calibrated_manifest, run, timestamp_list

DEFAULT_MANIFEST = PROJECT / "configs/orbslam3_lamaria/suite_20261002.json"
PROFILES = ("startup", "loss-context", "loss-prefix", "full")


def project_path(value):
    path = Path(value)
    return path.resolve() if path.is_absolute() else (PROJECT / path).resolve()


def load_manifest(path):
    manifest = json.loads(path.read_text())
    if manifest.get("schema_version") != 1:
        raise ValueError("Unsupported suite manifest version")
    return manifest


def choose_timestamps(times, profile, manifest, entry, event_name=None):
    event = None
    lower, upper = times[0], times[-1]
    note = "Full supported sequence; original timestamps and all earlier history retained."
    if profile == "startup":
        upper = min(upper, lower + int(manifest["startup_seconds"] * 1_000_000_000))
        note = "Initialization diagnostic from the original sequence start; not a full-sequence score."
    elif profile in ("loss-context", "loss-prefix"):
        events = entry["events"]
        event = next((e for e in events if e["name"] == event_name), None) if event_name else events[0]
        if event is None:
            raise ValueError(f"Unknown event {event_name}; available: {[e['name'] for e in events]}")
        upper = min(upper, event["end_ns"] + int(manifest["event_context_after_seconds"] * 1_000_000_000))
        if profile == "loss-context":
            lower = max(lower, event["start_ns"] - int(manifest["event_context_before_seconds"] * 1_000_000_000))
            note = "COLD START clip: initialization and map history differ from the full run. This cannot reproduce the prior map state or evaluate loop closure."
        else:
            note = "Stateful prefix replay from the original sequence start through the selected event. It preserves earlier inputs, but multithreaded execution may vary."
    elif profile != "full":
        raise ValueError(f"Unknown profile: {profile}")
    selected = [t for t in times if lower <= t <= upper]
    if len(selected) < 2:
        raise ValueError("Selected window has fewer than two frames")
    return selected, {"interpretation": note, "event": event,
                      "starts_at_original_sequence_start": selected[0] == times[0],
                      "full_sequence_scoring_allowed": profile == "full",
                      "loop_evaluation_allowed": profile == "full"}


def validate_entry(entry):
    """Check every image/feature path and feature length; sample binary headers."""
    paths = {key: Path(entry[key]) for key in ("config", "source_timestamps", "supported_timestamps")}
    imu = Path(entry["calibrated_imu"])
    paths["imu_manifest"] = imu.with_suffix(imu.suffix + ".manifest.json")
    paths["baseline_result"] = Path(entry["baseline"]) / "result.json"
    for key, path in paths.items():
        if sha256(path) != entry["sha256"][key]:
            raise ValueError(f"Pinned {key} changed: {path}")
    source = timestamp_list(paths["source_timestamps"])
    supported = timestamp_list(paths["supported_timestamps"])
    if len(source) != entry["baseline_source_frames"] or len(supported) != entry["baseline_supported_frames"]:
        raise ValueError("Source/supported frame counts differ from the baseline")
    indices = {stamp: index for index, stamp in enumerate(source)}
    try:
        retained = [indices[t] for t in supported]
    except KeyError as exc:
        raise ValueError("Supported timestamp is absent from the original source") from exc
    if retained != list(range(retained[0], retained[0] + len(retained))):
        raise ValueError("Supported inputs must be a contiguous source interval")
    imu_record = calibrated_manifest(imu, paths["imu_manifest"], supported[0], supported[-1])
    # Every source frame strictly inside calibrated IMU support must be retained.
    expected = [t for t in source if imu_record["first_timestamp_ns"] < t < imu_record["last_timestamp_ns"]]
    if expected != supported:
        raise ValueError("Supported timestamps do not equal the complete calibrated IMU interval")
    probes = {i * (len(supported) - 1) // 31 for i in range(32)}
    supported_set = set(supported)
    for cam in (0, 1):
        images = Path(entry["dataset"]) / f"mav0/cam{cam}/data"
        features = Path(entry[f"kp{cam}"])
        for i, stamp in enumerate(supported):
            image = images / f"{stamp}.png"
            feature = features / f"{stamp}.kp"
            if not image.is_file() or image.stat().st_size == 0:
                raise ValueError(f"Missing or empty native image: {image}")
            size = feature.stat().st_size
            # External loader format: int32 N, N*(xy float32 + response float32 + 32 bytes).
            if size < 4 or (size - 4) % 44:
                raise ValueError(f"Invalid feature file length: {feature}")
            if i in probes:
                with feature.open("rb") as handle:
                    count, = struct.unpack("<i", handle.read(4))
                if count < 0 or size != 4 + 44 * count:
                    raise ValueError(f"Feature header/length mismatch: {feature}")
    return supported, {
        "source_frames": len(source), "supported_frames": len(supported),
        "unsupported_prefix_frames": retained[0],
        "unsupported_suffix_frames": len(source) - retained[-1] - 1,
        "unsupported_timestamps_ns": [t for t in source if t not in supported_set],
        "images_and_feature_lengths_checked": 2 * len(supported),
        "feature_headers_sampled": 2 * len(probes),
        "feature_contents_rehashed": False,
        "imu_csv_sha256": imu_record["output_sha256"],
        "imu_first_timestamp_ns": imu_record["first_timestamp_ns"],
        "imu_last_timestamp_ns": imu_record["last_timestamp_ns"],
        "imu_policy": "Reuse the entire existing calibrated CSV. No trimming, timestamp rebasing, extrapolation or additional calibration.",
        "baseline_sha256": entry["sha256"],
    }


def runner_args(manifest, entry, case, run_dir, args):
    build = project_path(args.build or manifest["build"])
    build_manifest = json.loads((build / "build_manifest.json").read_text())
    return argparse.Namespace(out=run_dir, timestamps=case / "timestamps_ns.txt",
        config=case / "settings.yaml", build=build, vendor=Path(build_manifest["vendor"]),
        dataset=Path(entry["dataset"]), kp0=Path(entry["kp0"]), kp1=Path(entry["kp1"]),
        calibrated_imu=Path(entry["calibrated_imu"]), imu_manifest=None,
        vrs=None, raw_imu=None, imu_python=Path(manifest["python"]),
        viz_python=Path(manifest["python"]), gt=Path(entry["assets"]) / "gt_dense.txt",
        image=build_manifest["image"], cpus=args.cpus, timeout=0,
        dry_run=True, temporal_associations=True, skip_viz=not args.with_viz,
        match_diagnostics=getattr(args, "match_diagnostics", False),
        baby_features=getattr(args, "baby_features", False),
        debugger=getattr(args, "debugger", False))


def runner_command(manifest, opts):
    command = [manifest["python"], str(PROJECT / "pipeline/run/run_lamaria.py")]
    for name in ("dataset", "timestamps", "kp0", "kp1", "calibrated_imu", "config", "build", "vendor", "image", "gt", "out", "cpus", "imu_python", "viz_python"):
        command += ["--" + name.replace("_", "-"), str(getattr(opts, name))]
    command += ["--temporal-associations"]
    if opts.skip_viz:
        command += ["--skip-viz"]
    if getattr(opts, "match_diagnostics", False):
        command += ["--match-diagnostics"]
    if getattr(opts, "baby_features", False):
        command += ["--baby-features"]
    if getattr(opts, "debugger", False):
        command += ["--debugger"]
    return command


def shell_file(path, command, comment):
    path.write_text("#!/usr/bin/env bash\nset -eu\n# " + comment + "\nexec " + shlex.join(command) + "\n")
    path.chmod(0o755)


def prepare(manifest, entries, validated, args):
    out = args.out.resolve()
    if out.exists():
        raise FileExistsError(f"Suite output exists; select a fresh path: {out}")
    profiles = PROFILES if args.profile == "all" else (args.profile,)
    out.mkdir(parents=True)
    plans = []
    for key, entry in entries.items():
        times, validation = validated[key]
        for profile in profiles:
            selected, semantics = choose_timestamps(times, profile, manifest, entry, args.event)
            name = key + "_" + profile.replace("-", "_")
            case = out / name
            case.mkdir()
            config = Path(args.config or entry["config"]).resolve(strict=True)
            shutil.copy2(config, case / "settings.yaml")
            (case / "timestamps_ns.txt").write_text("".join(f"{t}\n" for t in selected))
            run_dir = out / "runs" / (out.name + "_" + name)
            opts = runner_args(manifest, entry, case, run_dir, args)
            with contextlib.redirect_stdout(io.StringIO()) as captured:
                code = run(opts)  # Existing runner --dry-run validation, never execution.
            if code:
                raise ValueError(f"Runner dry-run failed for {name}: {code}")
            dry_run = json.loads(captured.getvalue())
            command = runner_command(manifest, opts)
            render = dry_run["export_command"]
            shell_file(case / "run.sh", command, semantics["interpretation"])
            shell_file(case / "render.sh", render, "Explicit Rerun export after the estimator has completed.")
            scoring = None
            if profile == "full":
                scoring = [manifest["python"], manifest["score_script"], "--run", str(run_dir),
                    "--assets", entry["assets"], "--sequence", entry["sequence"],
                    "--toolkit", manifest["toolkit"], "--source-timestamps", entry["source_timestamps"],
                    "--map-scope", "largest"]
                shell_file(case / "score.sh", scoring, "Local full-sequence score; largest single map, all omitted GT poses remain failures.")
            plan = {"sequence": entry["sequence"], "profile": profile, "baseline": entry["baseline"],
                **semantics, "selected_frames": len(selected), "first_timestamp_ns": selected[0],
                "last_timestamp_ns": selected[-1], "input_span_seconds": (selected[-1] - selected[0]) / 1e9,
                "baseline_estimator_wall_seconds": entry["baseline_estimator_wall_seconds"],
                "asset_validation": validation, "suite_manifest_sha256": sha256(args.manifest),
                "config_sha256": sha256(case / "settings.yaml"), "config_override": args.config is not None,
                "source_config": str(config), "run_directory": str(run_dir), "run_command": command,
                "render_command": render, "score_command": scoring,
                "scoring_note": "Full profiles score one largest map selected without GT; never join independent map frames. Clips receive no benchmark scoring command.",
                "runner_dry_run": dry_run}
            (case / "plan.json").write_text(json.dumps(plan, indent=2) + "\n")
            plans.append({key: plan[key] for key in ("sequence", "profile", "selected_frames", "input_span_seconds", "run_directory")})
    (out / "suite.json").write_text(json.dumps({"plans": plans, "estimator_executed": False,
        "images_or_features_copied": False, "default_visualization_skipped": not args.with_viz}, indent=2) + "\n")
    print(json.dumps({"suite": str(out), "plans": plans, "estimator_executed": False}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("list", "validate", "prepare"))
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--sequence", choices=("short", "medium", "long", "all"), default="all")
    parser.add_argument("--profile", choices=(*PROFILES, "all"), default="startup")
    parser.add_argument("--event", help="Default: first listed event. Long uses a recovered coasting episode, not a reset.")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--build", type=Path)
    parser.add_argument("--config", type=Path, help="Explicit candidate config; restricted to a single selected sequence")
    parser.add_argument("--cpus", type=float, default=4)
    parser.add_argument("--with-viz", action="store_true", help="Render Rerun automatically; default defers it to render.sh")
    parser.add_argument("--match-diagnostics", action="store_true", help="Enable per-frame matcher logging without changing thresholds")
    parser.add_argument("--baby-features", action="store_true", help="Opt in to passive BabyFeature tracking with SOS-only estimator use")
    parser.add_argument("--debugger", action="store_true", help="Capture all-thread GDB backtraces if the estimator fails")
    args = parser.parse_args()
    if args.config and args.sequence == "all":
        parser.error("A config contains sequence-specific calibration; select one --sequence")
    if args.cpus <= 0 or (args.command == "prepare" and not args.out):
        parser.error("--cpus must be positive; prepare requires a fresh --out")
    try:
        manifest = load_manifest(args.manifest)
        entries = manifest["sequences"] if args.sequence == "all" else {args.sequence: manifest["sequences"][args.sequence]}
        if args.command == "list":
            print(json.dumps({"profiles": PROFILES, "sequences": entries}, indent=2))
            return
        if args.command == "prepare" and args.out.exists():
            raise FileExistsError(f"Suite output exists; choose a fresh path: {args.out}")
        validated = {key: validate_entry(entry) for key, entry in entries.items()}
        if args.command == "validate":
            print(json.dumps({key: value[1] for key, value in validated.items()}, indent=2))
        else:
            prepare(manifest, entries, validated, args)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Suite failed: {exc}\n")


if __name__ == "__main__":
    main()
