#!/usr/bin/env bash
# Common ATE/RPE step for ANY method: run the evaluator on every result trajectory (.TRJ)
# that lacks a sibling .TE. Idempotent. Run INSIDE the devcontainer (needs g2o libs on path).
# Dataset (for GT) is read from the result path: results/<METHOD>/<dataset>/...
#   bash experiments/analysis/evaluate.sh [results_root]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EVAL="$REPO/build/baselines/evaluator/evaluator"
ROOT="${1:-$REPO/experiments/results}"

[ -x "$EVAL" ] || { echo "evaluator not built: $EVAL" >&2; exit 1; }

n=0; skip=0
while IFS= read -r trj; do
  te="${trj%.TRJ}.TE"
  [ -s "$te" ] && { skip=$((skip+1)); continue; }
  ds=$(realpath --relative-to="$REPO/experiments/results" "$trj" | cut -d/ -f2)
  gt="$REPO/experiments/datasets/2D/$ds/GT.txt"
  [ -f "$gt" ] || { echo "no GT for dataset '$ds' — skip $trj" >&2; continue; }
  "$EVAL" "$trj" "$gt" "$te" >/dev/null 2>&1 && n=$((n+1))
done < <(find "$ROOT" -name '*.TRJ')
echo "evaluated $n new trajectories ($skip already had .TE) -> .TE"
