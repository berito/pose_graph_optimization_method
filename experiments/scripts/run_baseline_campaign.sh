#!/bin/bash
# Run baseline comparators over DATASETS x RATES x RUNS (same spoiled data as IPC). Resumable.
# Env: METHODS CAP DATASETS RATES RUNS TAG DATE.   Single run: --one <method> <ds> <rate> <run>
set -u

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
EVAL="$REPO/build/baselines/evaluator/evaluator"
CFGDIR="$REPO/experiments/configs/baselines"
TAG="${TAG:-BASE}"
DATE="${DATE:-$(date +%y%m%d)}"
METHODS="${METHODS:-PCM GNC DCS HUBER GM ADAPT MAXMIX}"
DATASETS="${DATASETS:-CSAIL FR079 FRH MIT INTEL M3500}"
RATES="${RATES:-10 20 30 40 50 60 70 80 90 100}"
RUNS="${RUNS:-00 01 02 03 04 05 06 07 08 09}"
CAP="${CAP:-26}"

declare -A INLIERS=( [CSAIL]=128 [FR079]=229 [FRH]=1505 [MIT]=20 [INTEL]=256 [M3500]=1954 )

# echo "<binary> <cli-style>"  (style: pos = positional cfg, cfg = -cfg flag)
resolve() {
  case "$1" in
    PCM)   echo "$REPO/build/baselines/robust_gtsam/gtsam_PCM_2D pos";;
    GNC)   echo "$REPO/build/baselines/robust_gtsam/gtsam_GNC_2D pos";;
    DCS)   echo "$REPO/build/baselines/robust_gtsam/gtsam_DCS_2D pos";;
    GM)    echo "$REPO/build/baselines/robust_gtsam/gtsam_GM_2D pos";;
    ADAPT) echo "$REPO/build/baselines/robust_gtsam/gtsam_ADAPT_2D pos";;
    MAXMIX) echo "$REPO/build/baselines/robust_g2o/src/off/MAXMIX_2D cfg";;
    HUBER) echo "$REPO/build/baselines/robust_g2o/src/off/HUBER_2D cfg";;
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
  read -r bin style <<< "$(resolve "$m" "$ds")"
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

if [ "${1:-}" = "--one" ]; then shift; run_one "$@"; exit $?; fi

export TAG DATE METHODS DATASETS RATES RUNS CAP
self="$REPO/experiments/scripts/run_baseline_campaign.sh"
jobs_file="$(mktemp)"
for m in $METHODS; do for ds in $DATASETS; do for rate in $RATES; do for run in $RUNS; do
  echo "$m $ds $rate $run"
done; done; done; done > "$jobs_file"
total=$(wc -l < "$jobs_file")
echo "[$(date)] START $total runs | methods=[$METHODS] | cap=$CAP | tag=$TAG | date=$DATE"
xargs -P "$CAP" -L1 bash "$self" --one < "$jobs_file"
rm -f "$jobs_file"
done_n=$(find "$REPO/experiments/results" -path "*/$DATE/$TAG/*" -name '*.TE' 2>/dev/null | wc -l)
echo "[$(date)] PASS COMPLETE — valid .TE: $done_n / $total"
[ "$done_n" -lt "$total" ] && echo "[$(date)] $((total-done_n)) missing (e.g. FR079 gtsam crashes / interrupted) — re-run to retry."
# Under the service, ALWAYS idle after a full pass (persistent failures like FR079 must not trigger a restart loop).
[ "${SERVICE:-0}" = "1" ] && { echo "[$(date)] idling (docker rm -f to clean up)."; sleep infinity; }
exit 0
