"""Resolve archived experiment locations after the project storage migration.

Only the two exact former experiment roots are translated. Source manifests,
commands, hashes, scientific results, and unrelated input paths stay unchanged.
This is deliberately not a fallback to the original project's storage.
"""
from pathlib import Path

EXPERIMENT_ROOT = Path('/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments')
FORMER_EXPERIMENT_ROOTS = (
    Path('/media/raghav/HardDrive1/INSV_STITCHING/experiments'),
    Path('/home/raghav/workspace/INSV_STITCHING/SLAM/experiments'),
)


def relocated_path(value):
    """Translate a recorded path without reading or changing its source file."""
    path = Path(value)
    for previous in FORMER_EXPERIMENT_ROOTS:
        if path.is_relative_to(previous):
            return EXPERIMENT_ROOT / path.relative_to(previous)
    return path


def relocated_record(value):
    """Normalize only path strings/keys for comparisons with archived records."""
    if isinstance(value, str):
        return str(relocated_path(value)) if value.startswith('/') else value
    if isinstance(value, list):
        return [relocated_record(item) for item in value]
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            canonical = relocated_record(key)
            if canonical in result:
                raise ValueError('Relocation would merge distinct manifest keys')
            result[canonical] = relocated_record(item)
        return result
    return value
