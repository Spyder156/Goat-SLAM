#!/usr/bin/env python3
"""Build the LaMAria repair in build/, leaving third_party read-only.

Requires an existing ORB-SLAM3 CMake build from the recorded baseline and the
local insv/orbslam3:gdb image. Changed translation units are rebuilt; unchanged
objects are hash-checked and reused. No vocabulary or dataset copy is made.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

PROJECT = Path(__file__).resolve().parents[2]
LIB_TARGET = "CMakeFiles/ORB_SLAM3.dir"
RUNNER_SOURCE = "Examples/Stereo-Inertial/stereo_lamaria_euroc.cc"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def patch_paths(text):
    paths = re.findall(r"^\+\+\+ b/(.+)$", text, re.M)
    for path in paths:
        if Path(path).is_absolute() or ".." in Path(path).parts:
            raise ValueError(f"Unsafe patch path: {path}")
    return paths


def base_objects(arguments):
    return [arg for arg in arguments if arg.startswith(LIB_TARGET + "/") and arg.endswith(".o")]


def source_for_object(obj):
    return str(Path(obj).relative_to(LIB_TARGET))[:-2]


def rewrite_link(arguments, replacements, output, runner=False):
    """Resolve CMake's relative paths without modifying its build tree."""
    result = []
    after_output = False
    for arg in arguments[1:]:
        if after_output:
            result.append(output)
            after_output = False
        elif arg == "-o":
            result.append(arg)
            after_output = True
        elif arg in replacements:
            result.append(replacements[arg])
        elif runner and arg.startswith("CMakeFiles/") and arg.endswith(".o"):
            result.append("/work/objects/runner.o")
        elif arg.startswith("CMakeFiles/"):
            result.append("/orb/build/" + arg)
        elif arg == "../lib/libORB_SLAM3.so":
            result.append("/work/lib/libORB_SLAM3.so")
        elif arg.startswith("../Thirdparty/"):
            result.append("/orb/" + arg[3:])
        else:
            result.append(arg)
    return result


def verify_baseline(vendor, manifest_path):
    if not manifest_path.exists():
        raise ValueError(f"Missing baseline hashes: {manifest_path}")
    manifest = json.loads(manifest_path.read_text())
    files = manifest.get("files", {})
    if not files:
        raise ValueError("Baseline manifest must contain a nonempty files mapping")
    mismatches = []
    for relative, expected in files.items():
        path = Path(relative)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError(f"Unsafe baseline path: {relative}")
        actual = vendor / path
        if not actual.is_file() or sha256(actual) != expected:
            mismatches.append(relative)
    if mismatches:
        raise ValueError("Vendor differs from the tested baseline: " + ", ".join(mismatches[:12]))
    return files


def run(args):
    vendor, package, out = args.vendor.resolve(), args.patch_dir.resolve(), args.out.resolve()
    if out == vendor or vendor in out.parents or out in vendor.parents:
        raise ValueError("Build output must be separate from the vendor tree")
    files = verify_baseline(vendor, package / "base_manifest.json")
    patch = package / "geometry.patch"
    changed = patch_paths(patch.read_text())
    link_path = vendor / "build" / LIB_TARGET / "link.txt"
    runner_link_path = vendor / "build/CMakeFiles/mono_rig_euroc.dir/link.txt"
    link = shlex.split(link_path.read_text())
    runner_link = shlex.split(runner_link_path.read_text())
    objects = base_objects(link)
    if not objects:
        raise ValueError("No reusable ORB_SLAM3 objects in baseline CMake link command")
    headers_changed = any(path.startswith("include/") for path in changed)
    newest_header = max(path.stat().st_mtime_ns for path in (vendor / "include").rglob("*") if path.is_file())
    compile_sources, reused = [], {}
    for obj in objects:
        source = source_for_object(obj)
        path = vendor / source
        if source.startswith("elsed/"):
            path = args.elsed.resolve() / source[len("elsed/"):]
        object_path = vendor / "build" / obj
        if not path.is_file() or not object_path.is_file():
            raise ValueError(f"Missing baseline source/object: {source}")
        stale = object_path.stat().st_mtime_ns < max(path.stat().st_mtime_ns, newest_header)
        if source in changed or headers_changed or stale:
            compile_sources.append(source)
        else:
            relative = "build/" + obj
            if relative not in files or (not source.startswith("elsed/") and source not in files):
                raise ValueError(f"Baseline manifest lacks reusable source/object hashes: {source}")
            reused[relative] = sha256(object_path)
    if not (package / "new_files" / RUNNER_SOURCE).is_file():
        raise ValueError(f"Missing patched runner: {RUNNER_SOURCE}")
    plan = {"vendor": str(vendor), "patch_dir": str(package), "output": str(out),
            "image": args.image, "compile_sources": compile_sources,
            "reuse_objects": len(reused), "patch_sha256": sha256(patch),
            "baseline_manifest_sha256": sha256(package / "base_manifest.json")}
    if args.dry_run:
        print(json.dumps(plan, indent=2))
        return
    out.mkdir(parents=True, exist_ok=True)
    # A failed rebuild must not leave an old success manifest authorizing a run
    # with a partially replaced library/runner pair.
    (out / "build_manifest.json").unlink(missing_ok=True)
    for directory in ("sources", "objects", "lib", "bin", "logs"):
        (out / directory).mkdir(exist_ok=True)
    # Fresh small source overlay, never a copy of Vocabulary or the dataset.
    overlay = out / "sources"
    for tree in ("src", "include"):
        if (overlay / tree).exists():
            shutil.rmtree(overlay / tree)
        shutil.copytree(vendor / tree, overlay / tree)
    for relative in changed:
        destination = overlay / relative
        if not destination.exists() and (vendor / relative).is_file():
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(vendor / relative, destination)
    subprocess.run(["patch", "--batch", "--forward", "--fuzz=0", "-p1", "-d", str(overlay), "-i", str(patch)], check=True)
    shutil.copytree(package / "new_files", overlay, dirs_exist_ok=True)
    image_id = subprocess.check_output(["docker", "image", "inspect", "--format", "{{.Id}}", args.image], text=True).strip()
    base = ["docker", "run", "--rm", "--network", "none", "--user", f"{os.getuid()}:{os.getgid()}",
            "-v", f"{vendor}:/orb:ro", "-v", f"{args.elsed.resolve()}:/elsed:ro", "-v", f"{out}:/work",
            "--entrypoint", "/usr/bin/c++", args.image]
    flag_text = (vendor / "build" / LIB_TARGET / "flags.make").read_text()
    flags = []
    # Put the patched headers first. Keep the exact baseline flags and libraries.
    flags += ["-I/work/sources", "-I/work/sources/include", "-I/work/sources/include/CameraModels"]
    for key in ("CXX_FLAGS", "CXX_DEFINES", "CXX_INCLUDES"):
        match = re.search(r"^" + key + r"\s*=\s*(.*)$", flag_text, re.M)
        if not match:
            raise ValueError(f"No {key} in baseline flags.make")
        flags += shlex.split(match.group(1))
    header_hashes = {str(p.relative_to(overlay)): sha256(p) for p in (overlay / "include").rglob("*") if p.is_file()}
    shared = json.dumps({"flags": flags, "image_id": image_id, "headers": header_hashes}, sort_keys=True)
    replacements = {}

    def compile_one(source):
        source_path = overlay / source
        container_source = "/work/sources/" + source
        if source.startswith("elsed/"):
            source_path = args.elsed.resolve() / source[len("elsed/"):]
            container_source = "/elsed/" + source[len("elsed/"):]
        obj_name = source.replace("/", "_") + ".o"
        fingerprint = hashlib.sha256((shared + sha256(source_path)).encode()).hexdigest()
        cache = out / "objects" / (obj_name + ".json")
        object_path = out / "objects" / obj_name
        valid = False
        if cache.exists() and object_path.exists():
            previous = json.loads(cache.read_text())
            valid = previous.get("fingerprint") == fingerprint and previous.get("sha256") == sha256(object_path)
        if not valid:
            with (out / "logs" / (obj_name + ".log")).open("w") as log:
                subprocess.run(base + flags + ["-c", container_source, "-o", "/work/objects/" + obj_name], stdout=log, stderr=subprocess.STDOUT, check=True)
            cache.write_text(json.dumps({"fingerprint": fingerprint, "sha256": sha256(object_path)}) + "\n")
        print(("REUSED " if valid else "COMPILED ") + source, flush=True)
        return LIB_TARGET + "/" + source + ".o", "/work/objects/" + obj_name

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        replacements.update(pool.map(compile_one, compile_sources))
    with (out / "logs/link.log").open("w") as log:
        subprocess.run(base + rewrite_link(link, replacements, "/work/lib/libORB_SLAM3.so"), stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(base + flags + ["-c", "/work/sources/" + RUNNER_SOURCE, "-o", "/work/objects/runner.o"], stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(base + rewrite_link(runner_link, {}, "/work/bin/stereo_lamaria_euroc", runner=True), stdout=log, stderr=subprocess.STDOUT, check=True)
    # The manifest pins the complete tested input and result, including reused objects.
    plan.update({"image_id": image_id, "reused_objects_sha256": reused,
                 "patched_sources_sha256": {source: sha256(overlay / source) for source in compile_sources if not source.startswith("elsed/")},
                 "headers_sha256": header_hashes,
                 "runner_source_sha256": sha256(overlay / RUNNER_SOURCE),
                 "library_sha256": sha256(out / "lib/libORB_SLAM3.so"),
                 "runner_sha256": sha256(out / "bin/stereo_lamaria_euroc")})
    (out / "build_manifest.json").write_text(json.dumps(plan, indent=2) + "\n")
    print(f"Built {out / 'bin/stereo_lamaria_euroc'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vendor", type=Path, default=PROJECT / "third_party/ORB_SLAM3")
    parser.add_argument("--elsed", type=Path, default=PROJECT / "third_party/ELSED")
    parser.add_argument("--patch-dir", type=Path, default=PROJECT / "patches/orbslam3_lamaria_history")
    parser.add_argument("--out", type=Path, default=PROJECT / "build/orbslam3_lamaria_history")
    parser.add_argument("--image", default="insv/orbslam3:gdb")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true", help="Validate hashes and print plan without writing or building")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    try:
        run(args)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f"Build failed: {exc}\n")


if __name__ == "__main__":
    main()
