#!/usr/bin/env python3
"""Verify a published snapshot or restore it into a new workspace.

This utility only reads, hashes, copies and safely unpacks files. It never runs
builds, Docker, SLAM, package installers or downloaded executables. SNAPSHOT.json
is an integrity inventory, not an independently authenticated signature.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import stat
import sys
import tarfile


class RestoreError(ValueError):
    """Invalid snapshot, unsafe path or incompatible destination."""


@dataclass(frozen=True)
class PlannedFile:
    target: str
    source: Path
    member: str | None = None
    size: int | None = None
    mode: int = 0o644


SOURCE_DIRECTORIES = ("pipeline", "configs", "patches", "docs")
SOURCE_DOCUMENTS = ("README.md", "AGENTS.md", "GOAL.md", "ROADMAP.md", "EXPERIMENTS.md",
                    "LICENSE", "LICENSE.md", "LICENSE.txt", "NOTICE")
SKIP_NAMES = {".git", "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache"}
HEX_SHA256 = re.compile(r"[0-9a-fA-F]{64}\Z")
SAFE_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")


def relative_path(value, *, directory=False):
    """Accept portable canonical relative paths, without traversal or aliases."""
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise RestoreError(f"Invalid relative path: {value!r}")
    raw = value.rstrip("/") if directory else value
    parts = raw.split("/")
    if any(p in ("", ".", "..") or ":" in p for p in parts):
        raise RestoreError(f"Unsafe relative path: {value!r}")
    path = PurePosixPath(raw)
    if path.is_absolute():
        raise RestoreError(f"Absolute path is forbidden: {value!r}")
    return path.as_posix()


def regular_source(root, relative):
    """Reject symlinks in every path component, not merely the last component."""
    relative = relative_path(relative)
    current = root
    for component in relative.split("/"):
        current /= component
        try:
            info = current.lstat()
        except FileNotFoundError as exc:
            raise RestoreError(f"Missing snapshot file: {relative}") from exc
        if stat.S_ISLNK(info.st_mode):
            raise RestoreError(f"Symlink in snapshot path: {relative}")
    if not stat.S_ISREG(info.st_mode):
        raise RestoreError(f"Expected a regular file: {relative}")
    return current


def digest(path):
    sha = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def unique_object(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise RestoreError(f"Duplicate JSON key: {key!r}")
        out[key] = value
    return out


def read_inventory(root):
    source = regular_source(root, "baselines/SNAPSHOT.json")
    try:
        doc = json.loads(source.read_text(), object_pairs_hook=unique_object)
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise RestoreError(f"Invalid snapshot JSON: {exc}") from exc
    if not isinstance(doc, dict) or type(doc.get("format")) is not int or doc["format"] != 1:
        raise RestoreError("Expected SNAPSHOT.json format 1")
    files, profiles = doc.get("files"), doc.get("profiles")
    if not isinstance(files, dict) or not files:
        raise RestoreError("Snapshot files must be a nonempty path-to-SHA256 mapping")
    if not isinstance(profiles, dict) or not profiles:
        raise RestoreError("Snapshot profiles must be a nonempty mapping")
    names = set()
    for name, profile in profiles.items():
        if not isinstance(name, str) or not SAFE_NAME.fullmatch(name):
            raise RestoreError(f"Unsafe profile name: {name!r}")
        if not isinstance(profile, dict):
            raise RestoreError(f"Invalid profile: {name}")
        target = profile.get("build_directory")
        if not isinstance(target, str) or not SAFE_NAME.fullmatch(target):
            raise RestoreError(f"Unsafe build_directory for {name}: {target!r}")
        if target in names:
            raise RestoreError(f"Duplicate build directory: {target}")
        names.add(target)
    for relative, expected in files.items():
        relative_path(relative)
        if not isinstance(expected, str) or not HEX_SHA256.fullmatch(expected):
            raise RestoreError(f"Invalid SHA256 for {relative}")
        actual = digest(regular_source(root, relative))
        if actual != expected.lower():
            raise RestoreError(f"SHA256 mismatch: {relative}\nExpected {expected}\nActual   {actual}")
    return doc


def source_files(root, subtree):
    """Enumerate only regular files; never follow source-tree links."""
    directory = root / subtree
    if directory.is_symlink() or not directory.is_dir():
        raise RestoreError(f"Missing or symlinked source directory: {subtree}")
    for parent, directories, files in os.walk(directory, followlinks=False):
        directories[:] = sorted(d for d in directories if d not in SKIP_NAMES)
        for name in directories:
            if (Path(parent) / name).is_symlink():
                raise RestoreError(f"Symlinked source directory: {Path(parent) / name}")
        for name in sorted(files):
            if name.endswith((".pyc", ".pyo")):
                continue
            path = Path(parent) / name
            relative = path.relative_to(root).as_posix()
            yield regular_source(root, relative)


def archive_members(archive):
    """Validate all members before a destination is created. No extractall()."""
    seen = {}
    try:
        with tarfile.open(archive, "r:xz") as stream:
            for member in stream:
                if member.name in (".", "./") and member.isdir():
                    continue
                name = relative_path(member.name, directory=member.isdir())
                if not (member.isfile() or member.isdir()):
                    raise RestoreError(f"Links/devices/special entries forbidden: {archive.name}:{member.name}")
                if name in seen and not (seen[name].isdir() and member.isdir()):
                    raise RestoreError(f"Duplicate archive member: {archive.name}:{name}")
                if member.size < 0:
                    raise RestoreError(f"Negative archive member size: {name}")
                seen[name] = member
    except (tarfile.TarError, EOFError) as exc:
        raise RestoreError(f"Invalid archive {archive}: {exc}") from exc
    for name in seen:
        for ancestor in PurePosixPath(name).parents:
            parent = seen.get(ancestor.as_posix())
            if parent is not None and not parent.isdir():
                raise RestoreError(f"Archive file used as directory: {archive.name}:{ancestor}")
    return [member for member in seen.values() if member.isfile()]


def create_plan(root, inventory):
    files = inventory["files"]
    plan = []

    def add_tree(source_prefix, target_prefix, *, require_hash):
        for source in source_files(root, source_prefix):
            relative = source.relative_to(root).as_posix()
            if require_hash and relative not in files:
                raise RestoreError(f"Unhashed snapshot source file: {relative}")
            target = PurePosixPath(target_prefix) / source.relative_to(root / source_prefix).as_posix()
            plan.append(PlannedFile(relative_path(target.as_posix()), source))

    def add_archive(relative, target_prefix):
        if relative not in files:
            raise RestoreError(f"Archive missing from SHA256 inventory: {relative}")
        archive = regular_source(root, relative)
        members = archive_members(archive)
        if not members:
            raise RestoreError(f"Archive has no regular files: {relative}")
        for member in members:
            target = relative_path((PurePosixPath(target_prefix) / member.name).as_posix())
            plan.append(PlannedFile(target, archive, member.name, member.size, member.mode & 0o777))

    add_tree("vendor/ORB_SLAM3", "third_party/ORB_SLAM3", require_hash=True)
    add_tree("vendor/ELSED", "third_party/ELSED", require_hash=True)
    for directory in SOURCE_DIRECTORIES:
        if (root / directory).exists() or (root / directory).is_symlink():
            add_tree(directory, directory, require_hash=False)
    for name in SOURCE_DOCUMENTS:
        if (root / name).exists() or (root / name).is_symlink():
            plan.append(PlannedFile(name, regular_source(root, name)))
    add_archive("baselines/runtime/vendor_build.tar.xz", "third_party/ORB_SLAM3")
    for name, profile in inventory["profiles"].items():
        source_prefix = f"baselines/{name}"
        target_prefix = f"build/{profile['build_directory']}"
        add_tree(f"{source_prefix}/sources", f"{target_prefix}/sources", require_hash=True)
        manifest = f"{source_prefix}/build_manifest.json"
        if manifest not in files:
            raise RestoreError(f"Unhashed build manifest: {manifest}")
        plan.append(PlannedFile(f"{target_prefix}/build_manifest.json", regular_source(root, manifest)))
        add_archive(f"baselines/runtime/{name}.tar.xz", target_prefix)
    targets = set()
    for item in plan:
        if item.target in targets:
            raise RestoreError(f"Restore sources collide at: {item.target}")
        targets.add(item.target)
    for target in targets:
        for parent in PurePosixPath(target).parents:
            if parent.as_posix() in targets:
                raise RestoreError(f"File/directory collision in restore plan: {parent}")
    return plan


def destination_path(root, value):
    raw = Path(os.path.abspath(os.path.expanduser(value)))
    if os.path.lexists(raw):
        raise RestoreError(f"Destination already exists; refusing to overwrite: {raw}")
    if not raw.parent.is_dir():
        raise RestoreError(f"Destination parent must already exist: {raw.parent}")
    dest = raw.parent.resolve() / raw.name
    if dest == root or root in dest.parents:
        raise RestoreError("Destination must be outside the published source repository")
    return dest


def restore(root, destination, inventory, plan):
    # Exclusive mkdir is the final no-overwrite check after all validation.
    destination.mkdir(mode=0o700, exist_ok=False)
    archives = {}
    try:
        for item in plan:
            target = destination / item.target
            target.parent.mkdir(parents=True, exist_ok=True)
            if item.member is not None:
                archives.setdefault(item.source, []).append(item)
                continue
            with item.source.open("rb") as src, target.open("xb") as dst:
                shutil.copyfileobj(src, dst, length=1024 * 1024)
            target.chmod(stat.S_IMODE(item.source.stat().st_mode) & 0o777)
        # Open each compressed archive once, not once per contained file.
        for archive, items in archives.items():
            required = {item.member: item for item in items}
            with tarfile.open(archive, "r:xz") as stream:
                for member in stream:
                    item = required.pop(member.name, None)
                    if item is None:
                        continue
                    target = destination / item.target
                    if not member.isfile() or member.size != item.size:
                        raise RestoreError(f"Archive changed after validation: {archive}:{member.name}")
                    src = stream.extractfile(member)
                    if src is None:
                        raise RestoreError(f"Unreadable archive file: {member.name}")
                    with src, target.open("xb") as dst:
                        shutil.copyfileobj(src, dst, length=1024 * 1024)
                    if target.stat().st_size != item.size:
                        raise RestoreError(f"Truncated restored file: {item.target}")
                    target.chmod(item.mode)
            if required:
                raise RestoreError(f"Archive changed; missing members: {archive}")
        for item in plan:
            if item.member is None:
                expected = inventory["files"].get(item.source.relative_to(root).as_posix())
                if expected is not None and digest(destination / item.target) != expected.lower():
                    raise RestoreError(f"Restored checksum mismatch: {item.target}")
    except Exception:
        print(f"Restore did not complete. Partial new directory retained for inspection: {destination}", file=sys.stderr)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--verify", action="store_true", help="Check all inventory hashes and archive/path safety without writing")
    modes.add_argument("--destination", metavar="PATH", help="Restore into a nonexistent directory outside this source checkout")
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    try:
        destination = destination_path(root, args.destination) if args.destination else None
        inventory = read_inventory(root)
        plan = create_plan(root, inventory)
        print(f"Verified {len(inventory['files'])} SHA256 entries; {len(plan)} restorable files; profiles: {', '.join(inventory['profiles'])}")
        if destination is None:
            print("Verification complete. No files written; no commands executed.")
            return 0
        restore(root, destination, inventory, plan)
        print(f"Restored source and frozen runtime snapshots to {destination}")
        print("No build, estimator, Docker image or installation was run. Dataset and environment prerequisites remain external.")
        if (destination / "pipeline/run/build_lamaria.py").is_file():
            preferred = "online_full_v2" if "online_full_v2" in inventory["profiles"] else next(iter(inventory["profiles"]))
            build_name = inventory["profiles"][preferred]["build_directory"]
            patch = destination / "patches" / build_name
            if patch.is_dir():
                command = ["python3", str(destination / "pipeline/run/build_lamaria.py"), "--patch-dir", str(patch), "--out", str(destination / "build" / build_name), "--dry-run"]
                print("Optional existing-builder validation (printed only):")
                print(shlex.join(command))
        return 0
    except (RestoreError, OSError, tarfile.TarError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
