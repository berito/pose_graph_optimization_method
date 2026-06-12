#!/bin/bash
# Run IPC over DATASETS x RATES x RUNS at s_factor=3. Resumable: finished runs are skipped.
# Env knobs: CAP DATASETS RATES RUNS TAG DATE.   Single run: --one <ds> <rate> <run>
set -u

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
BIN="$REPO/build/ipc/ipc_tester_2D"
TAG="${TAG:-IPC_S3}"
DATE="${DATE:-$(date +%y%m%d)}"
DATASETS="${DATASETS:-CSAIL FR079 FRH MIT INTEL M3500}"
RATES="${RATES:-10 20 30 40 50 60 70 80 90 100}"
RUNS="${RUNS:-00 01 02 03 04 05 06 07 08 09}"
CAP="${CAP:-26}"

valid_pr() { [ -s "$1" ] && head -n1 "$1" | grep -qE '^[0-9]+(\.[0-9]+)?[[:space:]]+[0-9]'; }

run_one() {
  local ds="$1" rate="$2" run="$3"
  local cfg="$REPO/ipc/cfg/2D/${ds}_params.yaml"
  local input="$REPO/experiments/datasets/2D/${ds}/SPOILED_DATA/${rate}/${run}.g2o"
  local gt="$REPO/experiments/datasets/2D/${ds}/GT.txt"
  local outdir="$REPO/experiments/results/IPC/${ds}/${DATE}/${TAG}/${rate}"
  local final_trj="$outdir/${run}.TRJ"
  local final_pr="$outdir/${run}.PR"

  valid_pr "$final_pr" && return 0
  [ -f "$cfg" ]   || { echo "MISSING cfg   $cfg"   >&2; return 1; }
  [ -f "$input" ] || { echo "MISSING input $input" >&2; return 1; }

  mkdir -p "$outdir"
  local scratch="$outdir/.tmp_${run}"
  rm -rf "$scratch"; mkdir -p "$scratch"
  local yaml="$scratch/run.yaml"
  cp "$cfg" "$yaml"
  yq -i ".dataset=\"$input\""             "$yaml"
  yq -i ".ground_truth=\"$gt\""           "$yaml"
  yq -i ".output=\"$scratch/${run}.TRJ\"" "$yaml"
  yq -i ".s_factor=3.0"                   "$yaml"
  [ -n "${FAST_TH:-}" ] && yq -i ".fast_reject_th=$FAST_TH" "$yaml"
  [ -n "${SLOW_TH:-}" ] && yq -i ".slow_reject_th=$SLOW_TH" "$yaml"

  ( cd "$scratch" && "$BIN" -c "$yaml" ) >"$scratch/log.txt" 2>&1
  local rc=$?
  if [ $rc -eq 0 ] && valid_pr "$scratch/${run}.PR"; then
    mv -f "$scratch/${run}.TRJ" "$final_trj" 2>/dev/null
    mv -f "$scratch/${run}.PR"  "$final_pr"
    rm -rf "$scratch"
    return 0
  fi
  echo "FAIL $ds $rate $run rc=$rc (kept $scratch/log.txt)" >&2
  return 1
}

if [ "${1:-}" = "--one" ]; then shift; run_one "$@"; exit $?; fi

export -f run_one valid_pr
export REPO BIN TAG DATE FAST_TH SLOW_TH
self="$REPO/experiments/scripts/run_ipc_campaign.sh"

jobs_file="$(mktemp)"
for ds in $DATASETS; do for rate in $RATES; do for run in $RUNS; do
  echo "$ds $rate $run"
done; done; done > "$jobs_file"
total=$(wc -l < "$jobs_file")

echo "[$(date)] START $total runs | cap=$CAP | s=3 | tag=$TAG | date=$DATE | $DATASETS"
xargs -P "$CAP" -L1 bash "$self" --one < "$jobs_file"
rm -f "$jobs_file"

done_n=$(find "$REPO/experiments/results/IPC" -name '*.PR' 2>/dev/null | wc -l)
echo "[$(date)] DONE — valid .PR: $done_n / $total"
if [ "$done_n" -lt "$total" ]; then
  echo "[$(date)] re-run to resume the rest."
elif [ "${SERVICE:-0}" = "1" ]; then
  echo "[$(date)] complete — idling (docker rm -f the container to clean up)."
  sleep infinity
fi
exit 0
