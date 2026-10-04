#!/usr/bin/env bash
# Offline global fixed-camera VI refinement on a finished run: prepare -> solve -> export -> official score.
# Usage: run_native_vi_chain.sh <run_dir> <experiment_dir> <imu_csv> <sequence_id> <assets_dir>
set -u
RUN=$1; X=$2; IMU=$3; SEQ=$4; ASSETS=$5
ROOT=/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3; PY=/home/raghav/miniconda3/envs/lamaria/bin/python
cd $ROOT; test ! -e $X || { echo "exists: $X"; exit 1; }; mkdir -p $X
echo "prepare $(date +%T)"; $PY -B pipeline/run/prepare_lamaria_native_refinement.py --source $RUN --out $X/input || exit 1
cp $IMU $X/input/calibrated_imu.csv; [ -f $IMU.manifest.json ] && cp $IMU.manifest.json $X/input/calibrated_imu.csv.manifest.json
echo "solve $(date +%T)"; LAMARIA_BA_BINARY=$ROOT/build/colmap_lamaria_babyfeats_vi_20261004/colmap_lamaria_ba $PY -B pipeline/vi_ba_lamaria/launch.py --input_path $X/input/model --output_path $X/offline/vi_fixed/model --imu_csv $X/input/calibrated_imu.csv --settings_yaml $X/input/accepted_settings.yaml --mode vi_fixed --linear_solver sparse_schur --max_iterations 100 --max_threads 8 || exit 1
echo "export $(date +%T)"; $PY -B pipeline/run/export_lamaria_native_refinement.py --source-run $RUN --prepared $X/input --model $X/offline/vi_fixed/model --out $X/runs/vi_fixed || exit 1
SRC_TS=$($PY - <<PYEOF
import json,ast
p=json.load(open("$RUN/lamaria_score/provenance.json")); c=p.get('command'); c=ast.literal_eval(c) if isinstance(c,str) else c
print(c[c.index('--source-timestamps')+1] if c and '--source-timestamps' in c else '')
PYEOF
)
echo "score $(date +%T) source_timestamps=$SRC_TS"; $PY -B pipeline/run/score_lamaria.py --run $X/runs/vi_fixed --assets $ASSETS --sequence $SEQ --toolkit /home/raghav/workspace/MeckaAI/third_party/lamaria_toolkit ${SRC_TS:+--source-timestamps $SRC_TS} --map-scope largest || exit 1
echo "source $(grep -o '"Score2D": [0-9.]*' $RUN/lamaria_score/scores.json) -> refined $(grep -o '"Score2D": [0-9.]*' $X/runs/vi_fixed/lamaria_score/scores.json)"
echo "done $(date +%T)"
