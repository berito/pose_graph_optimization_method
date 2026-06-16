#!/usr/bin/env bash
# Generate the correlated (grouped) spoiled-data grid for one dataset, mirroring
# the SPOILED_DATA/ layout: SPOILED_DATA_CORR/<rate>/<NN>.{g2o,groups,gt_labels}.
# rate = outliers as % of true loops; 10 trials/rate, seed=run index.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GEN="$ROOT/experiments/datagen/generateCorrelatedDataset.py"

DATASET="${DATASET:-M3500}"
INLIERS="${INLIERS:-1954}"
RATES="${RATES:-10 20 30 40 50 60 70 80 90 100}"
TRIALS="${TRIALS:-10}"
GROUP_LEN="${GROUP_LEN:-5}"
MIN_SEP="${MIN_SEP:-50}"
MAX_SEP="${MAX_SEP:-200}"
DISP_TRANS="${DISP_TRANS:-5.0}"
DISP_ROT="${DISP_ROT:-30.0}"
SIGMA_T="${SIGMA_T:-0.1}"
SIGMA_R="${SIGMA_R:-0.01}"

DSDIR="$ROOT/experiments/datasets/2D/$DATASET"
IN="$DSDIR/graph.g2o"
GT="$DSDIR/GT.txt"
OUT="$DSDIR/SPOILED_DATA_CORR"
MAN="$OUT/manifest.csv"

mkdir -p "$OUT"
echo "dataset,rate,run,seed,n_outliers,inliers,group_len,min_sep,max_sep,disp_trans,disp_rot,mode,file,command" > "$MAN"

for rate in $RATES; do
  n_out=$(python3 -c "print(int(round($rate/100.0*$INLIERS)))")
  mkdir -p "$OUT/$rate"
  for ((r=0; r<TRIALS; r++)); do
    NN=$(printf "%02d" "$r")
    f="$OUT/$rate/$NN.g2o"
    cmd="python3 $GEN --in $IN --gt $GT --out $f --mode grouped --num-outliers $n_out --group-len $GROUP_LEN --min-sep $MIN_SEP --max-sep $MAX_SEP --sigma-t $SIGMA_T --sigma-r $SIGMA_R --disp-trans $DISP_TRANS --disp-rot $DISP_ROT --seed $r"
    $cmd >/dev/null
    echo "$DATASET,$rate,$NN,$r,$n_out,$INLIERS,$GROUP_LEN,$MIN_SEP,$MAX_SEP,$DISP_TRANS,$DISP_ROT,grouped,$f,\"$cmd\"" >> "$MAN"
  done
  echo "rate $rate: $n_out outliers x $TRIALS trials"
done
echo "done -> $OUT"
