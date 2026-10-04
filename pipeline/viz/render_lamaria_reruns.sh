#!/usr/bin/env bash
# Build the evaluation-aligned Rerun (saved CP Sim3, Baby panels) for scored LaMAria arms.
# Same builder and flags as the earlier Reruns: the planner's render.sh arguments plus
#   --output-dir <case>/evaluation_baby_20261004 --evaluation-score <run>/lamaria_score/scores.json --baby-overlays
# Usage:  render_reruns_20261004.sh [--dry-run] CASE [CASE ...]      (CASE = arm folder name)
#         PARALLEL=2 render_reruns_20261004.sh CASE ...               (default 1 at a time)
# Skips a case whose run.rrd already exists; never overwrites. Logs: <case>/render_20261004.log
set -u
E=/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments
PY=/home/raghav/miniconda3/envs/lamaria/bin/python
DRY=0; [ "${1:-}" = "--dry-run" ] && { DRY=1; shift; }
PARALLEL=${PARALLEL:-1}

case_dir() {   # arm name -> case folder (batch1 holds the two Baby transfer runs)
    for root in "$E/lamaria_batch3_20261004" "$E/lamaria_batch2_20261004" "$E/lamaria_batch1_20261004"; do
        [ -d "$root/$1/runs" ] && { echo "$root/$1"; return; }
    done
    return 1
}

render_one() {
    local name=$1 dir run rsh out log args
    dir=$(case_dir "$name") || { echo "[$name] no such case"; return 1; }
    run=$(ls -d "$dir"/runs/*_full | head -1)
    rsh=$(ls "$dir"/*/render.sh | grep -v /runs/ | head -1)
    out=$dir/evaluation_baby_20261004; log=$dir/render_20261004.log
    [ -f "$out/run.rrd" ] && { echo "[$name] exists: $out/run.rrd"; return 0; }
    [ -f "$run/lamaria_score/scores.json" ] || { echo "[$name] not scored, skipped"; return 1; }
    # planner arguments (--run-dir --dataset --timestamps --config --gt), taken verbatim from render.sh
    args=$(tr ' ' '\n' < "$rsh" | awk '/^--/{flag=1} flag' | tr '\n' ' ')
    for f in $(echo "$args" | tr ' ' '\n' | grep '^/'); do [ -e "$f" ] || { echo "[$name] missing input $f"; return 1; }; done
    local cmd="$PY -B /home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/pipeline/viz/make_lamaria_output.py $args --output-dir $out --evaluation-score $run/lamaria_score/scores.json --baby-overlays"
    if [ $DRY = 1 ]; then echo "[$name] OK  -> $out/run.rrd"; return 0; fi
    rm -rf "$out"; echo "[$name] start $(date +%T)"
    if $cmd > "$log" 2>&1; then
        echo "[$name] done $(date +%T) $(du -h "$out/run.rrd" | cut -f1) $(cat "$out/run.rrd.verify.log" 2>/dev/null) -> $out/run.rrd"
    else
        echo "[$name] FAILED, see $log"; return 1
    fi
}
export -f render_one case_dir; export E PY DRY
printf '%s\n' "$@" | xargs -P "$PARALLEL" -I{} bash -c 'render_one {}'
