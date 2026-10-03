#!/usr/bin/env bash
# Fast correctness harness: run the first N seconds of a sequence instead of all
# 4951 frames. Full runs take ~15 min, this takes ~1, which is the difference
# between testing an idea and guessing at one.
#
# This checks that the estimator RUNS CORRECTLY -- no crash, lines actually
# tracked rather than churned, residuals sane. It is not a scoring harness:
# 20 s of a 165 s sequence says nothing useful about ATE.
#
#   short_test.sh <config.yaml> <outdir> [secs=20]
set -euo pipefail
if (( $# < 2 )); then
  echo "Usage: $0 <project-relative-config.yaml> <project-relative-outdir> [secs=20]" >&2
  exit 2
fi
W="$(cd "$(dirname "$0")/../.." && pwd)"
CFG="$1"; OUT="$2"; SECS="${3:-20}"
[[ -d "$W/Data/Hilti" ]] || { echo "Set up this project's Data/Hilti before running: $W/Data/Hilti" >&2; exit 1; }
[[ -f "$W/$CFG" ]] || { echo "Missing project config: $W/$CFG" >&2; exit 1; }
mkdir -p "$W/$OUT"
# Data must be selected for this copy; never fall back to the original checkout.
DATA_ROOT="$(readlink -f "$W/Data")"
EXPERIMENT_ROOT="$(readlink -f "$W/experiments")"
docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -w "/ws/$OUT" -v "$W:/ws" \
  -v "$DATA_ROOT:/ws/Data" -v "$EXPERIMENT_ROOT:/ws/experiments" \
  -v "$W/configs/orbslam3_hilti:/config" insv/orbslam3 bash -c "
  export LD_LIBRARY_PATH=/ws/third_party/ORB_SLAM3/lib:/ws/third_party/ORB_SLAM3/Thirdparty/DBoW2/lib:/ws/third_party/ORB_SLAM3/Thirdparty/g2o/lib:\$LD_LIBRARY_PATH
  stdbuf -oL -eL /ws/third_party/ORB_SLAM3/Examples/Monocular-Inertial/mono_rig_euroc \
    /ws/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /ws/$CFG \
    /ws/Data/Hilti/orb/floor_EG_2025-12-02_run_2 \
    /ws/configs/orbslam3_hilti/timestamps_run2_first${SECS}s.txt orb > log.txt 2>&1"
echo "-> $OUT/log.txt"
