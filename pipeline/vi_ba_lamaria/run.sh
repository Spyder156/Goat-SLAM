#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd "$(dirname "$0")/../.." && pwd)"
exec python3 "$task_root/pipeline/vi_ba_lamaria/launch.py" "$@"
