#!/usr/bin/env bash
# Use this project's copied build. Identical host/container paths keep COLMAP
# image references valid without depending on the original checkout.
set -euo pipefail
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BINARY="$R/build/colmap_build/src/colmap/exe/colmap"
[[ -x "$BINARY" ]] || { echo "Missing project COLMAP build: $BINARY" >&2; exit 1; }
MOUNTS=(-v "$R:$R" -v "$PWD:$PWD")
for folder in "$R/experiments" "$R/Data"; do
  if [[ -d "$folder" ]]; then
    physical="$(readlink -f "$folder")"
    MOUNTS+=(-v "$physical:$physical")
  fi
done
exec docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -e "LD_LIBRARY_PATH=$R/build/colmap_build/_deps/onnxruntime-build/lib" \
  --gpus all "${MOUNTS[@]}" -w "$PWD" \
  fisheye-slam "$BINARY" "$@"
