#!/bin/bash
# Run a baseline comparator. Human-friendly: pass the algorithm name and run just that.
#
#   bash run_baseline.sh <METHOD> [dataset] [rate] [run]
#
#   METHOD  : PCM GNC DCS HUBER GM ADAPT MAXMIX     (or "all" for every method)
#   dataset : M3500 (default) | M3500,INTEL | all   (select by name; NO default-all)
#   rate    : 50    (default) | 10,30,50   | all    (all = 10..100)
#   run     : 00    (default) | 00,01      | all    (all = 00..09)
#
# Examples:
#   bash run_baseline.sh DCS                   # ONE run: DCS on M3500, 50% outliers, seed 00
#   bash run_baseline.sh DCS M3500 30 02       # ONE specific run
#   bash run_baseline.sh DCS M3500,INTEL all all  # DCS on TWO named datasets, all rates x seeds
#   bash run_baseline.sh PCM,DCS CSAIL all all # two methods on CSAIL only
#   bash run_baseline.sh all all all all       # the full campaign
#
# Resumable (valid .PR+.TE skipped), atomic, parallel (CAP, default 26). Run inside the devcontainer.
set -u

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
EVAL="$REPO/build/baselines/evaluator/evaluator"
CFGDIR="$REPO/experiments/configs/baselines"
TAG="${TAG:-BASE}"
DATE="${DATE:-$(date +%y%m%d)}"
CAP="${CAP:-26}"

ALL_METHODS="PCM GNC DCS HUBER GM ADAPT MAXMIX"
ALL_DATASETS="CSAIL FR079 FRH MIT INTEL M3500"
ALL_RATES="10 20 30 40 50 60 70 80 90 100"
ALL_RUNS="00 01 02 03 04 05 06 07 08 09"

declare -A INLIERS=( [CSAIL]=128 [FR079]=229 [FRH]=1505 [MIT]=20 [INTEL]=256 [M3500]=1954 )

resolve() {
  case "$1" in
    PCM)    echo "$REPO/build/baselines/robust_gtsam/gtsam_PCM_2D pos";;
    GNC)    echo "$REPO/build/baselines/robust_gtsam/gtsam_GNC_2D pos";;
    DCS)    echo "$REPO/build/baselines/robust_gtsam/gtsam_DCS_2D pos";;
    GM)     echo "$REPO/build/baselines/robust_gtsam/gtsam_GM_2D pos";;
    ADAPT)  echo "$REPO/build/baselines/robust_gtsam/gtsam_ADAPT_2D pos";;
    MAXMIX) echo "$REPO/build/baselines/robust_g2o/src/off/MAXMIX_2D cfg";;
    HUBER)  echo "$REPO/build/baselines/robust_g2o/src/off/HUBER_2D cfg";;
  esac
}

valid_pr() { [ -s "$1" ] && head -n1 "$1" | grep -qE '^[0-9.]+([eE.+-]*[0-9]*)?[[:space:]]+[0-9.]'; }

run_one() {
  local m="$1" ds="$2" rate="$3" run="$4"
  local base="$CFGDIR/${m}.yaml"
  local input="$REPO/experiments/datasets/2D/${ds}/SPOILED_DATA/${rate}/${run}.g2o"
  local gt="$REPO/experiments/datasets/2D/${ds}/GT.txt"
  local outdir="$REPO/experiments/results/${m}/${ds}/${DATE}/${TAG}/${rate}"
  local final_pr="$outdir/${run}.PR" final_te="$outdir/${run}.TE" final_trj="$outdir/${run}.TRJ"

  valid_pr "$final_pr" && [ -s "$final_te" ] && return 0
  [ -f "$base" ]  || { echo "MISS cfg $base"     >&2; return 1; }
  [ -f "$input" ] || { echo "MISS input $input"  >&2; return 1; }
  read -r bin style <<< "$(resolve "$m")"
  [ -x "$bin" ]   || { echo "MISS bin $bin"      >&2; return 1; }

  mkdir -p "$outdir"; local scratch="$outdir/.tmp_${run}"; rm -rf "$scratch"; mkdir -p "$scratch"
  local yaml="$scratch/run.yaml"; cp "$base" "$yaml"
  yq -i ".dataset=\"$input\""              "$yaml"
  yq -i ".output=\"$scratch/${run}.TRJ\""  "$yaml"
  yq -i ".canonic_inliers=${INLIERS[$ds]}" "$yaml"

  ( cd "$scratch"
    if [ "$style" = "cfg" ]; then "$bin" -cfg "$yaml"; else "$bin" "$yaml"; fi
  ) >"$scratch/log.txt" 2>&1
  local rc=$?
  [ -f "$scratch/${run}.TRJ" ] && "$EVAL" "$scratch/${run}.TRJ" "$gt" "$scratch/${run}.TE" >/dev/null 2>&1

  if [ $rc -eq 0 ] && valid_pr "$scratch/${run}.PR" && [ -s "$scratch/${run}.TE" ]; then
    mv -f "$scratch/${run}.TRJ" "$final_trj" 2>/dev/null
    mv -f "$scratch/${run}.PR" "$final_pr"; mv -f "$scratch/${run}.TE" "$final_te"
    rm -rf "$scratch"; return 0
  fi
  echo "FAIL $m $ds $rate $run rc=$rc (kept $scratch/log.txt)" >&2; return 1
}

# Internal worker (used by the parallel dispatch).
if [ "${1:-}" = "--one" ]; then shift; run_one "$@"; exit $?; fi

# ── Human-friendly positional CLI ─────────────────────────────────────────────
m_arg="${1:-}"
if [ -z "$m_arg" ] || [ "$m_arg" = "-h" ] || [ "$m_arg" = "--help" ]; then
  awk 'NR>1 && /^#/{sub(/^# ?/,"");print} NR>1 && !/^#/{exit}' "$0"; exit 0
fi
expand() { [ "$1" = "all" ] && echo "$2" || echo "${1//,/ }"; }
methods=$(expand "$m_arg"        "$ALL_METHODS")
datasets=$(expand "${2:-M3500}"  "$ALL_DATASETS")
rates=$(expand "${3:-50}"        "$ALL_RATES")
runs=$(expand "${4:-00}"         "$ALL_RUNS")

export TAG DATE REPO EVAL CFGDIR CAP
export -f run_one valid_pr resolve
self="$REPO/experiments/scripts/run_baseline.sh"

jobs_file="$(mktemp)"
for m in $methods; do for ds in $datasets; do for rate in $rates; do for run in $runs; do
  echo "$m $ds $rate $run"
done; done; done; done > "$jobs_file"
total=$(wc -l < "$jobs_file")

echo "[$(date)] baseline | methods=[$methods] | datasets=[$datasets] | rates=[$rates] | runs=[$runs] | $total run(s) | cap=$CAP"
xargs -P "$CAP" -L1 bash "$self" --one < "$jobs_file"
rm -f "$jobs_file"

# Human touch: for a single run, print the result right here.
if [ "$total" -eq 1 ]; then
  pr="$REPO/experiments/results/${methods}/${datasets}/${DATE}/${TAG}/${rates}/${runs}.PR"
  te="${pr%.PR}.TE"
  if valid_pr "$pr"; then
    echo "--- ${methods} ${datasets} rate=${rates} run=${runs} ---"
    echo "P/R       : $(head -n1 "$pr")"
    [ -s "$te" ] && echo "ATE/RPE   : $(cat "$te")"
  else
    echo "no valid result (see scratch log under that result dir)"
  fi
else
  done_n=$(find "$REPO/experiments/results" -path "*/$DATE/$TAG/*" -name '*.TE' 2>/dev/null | wc -l)
  echo "[$(date)] done — valid .TE under $DATE/$TAG: $done_n"
fi
[ "${SERVICE:-0}" = "1" ] && { echo "[$(date)] idling (docker rm -f to clean up)."; sleep infinity; }
exit 0
