# campaign_lib.sh — the SHARED experiment system for the per-method runners
# (run_ipc.sh / run_taco.sh / run_dcipc.sh). The runners stay decoupled and are
# launched independently; this lib only provides the common mechanics so that
# saving / resume / provenance behave IDENTICALLY for every method.
#
# A runner must, before calling cl_dispatch, define:
#   vars : METHOD RES BIN CFGDIR SRC
#          DATASETS RATES RUNS LAMBDAS DATADIR_NAME SCEN_TAG DATE CAP
#          (and any method knobs: S_FACTOR USE_KBL K_BUDDIES USE_REC)
#   funcs: apply_overrides <yaml> <lambda>   # method-specific yq edits
#          signature <lambda>                # result-dir signature string
# Then: cl_dispatch "$0" "$@"

CL_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$CL_LIB_DIR/../../.." && pwd)"

# Canonical lists, only used to expand the literal keyword "all".
CL_ALL_DATASETS="CSAIL FR079 FRH MIT INTEL M3500"
CL_ALL_RATES="10 20 30 40 50 60 70 80 90 100"
CL_ALL_RUNS="00 01 02 03 04 05 06 07 08 09"

# "all" -> the full list; otherwise treat as a comma- or space-separated set of names.
cl_expand() { if [ "$1" = "all" ]; then echo "$2"; else echo "${1//,/ }"; fi; }

cl_usage() {
  local lam=""; [ "$METHOD" = "dcipc" ] && lam=" [lambdas]"
  echo "Usage: bash $(basename "$1") <datasets> <rates> <runs>$lam"
  echo "  datasets : M3500 | M3500,INTEL | all   (NO default-all; default: $DATASETS)"
  echo "  rates    : 50 | 10,30,50 | all         (default: $RATES)"
  echo "  runs     : 00 | 00,01 | all            (default: $RUNS)"
  [ "$METHOD" = "dcipc" ] && echo "  lambdas  : 0,50 | 0                     (default: $LAMBDAS)"
  echo "  e.g. bash $(basename "$1") M3500 50 00          # one experiment"
  echo "       bash $(basename "$1") M3500,INTEL all all  # two datasets, full grid"
}

valid_pr() { [ -s "$1" ] && head -n1 "$1" | grep -qE '^[0-9]+(\.[0-9]+)?[[:space:]]+[0-9]'; }

# Run exactly one (ds, rate, run, lambda). Atomic: result is committed to its final
# path only on success; a finished run is independent of all others. Resumable: an
# existing valid .PR is skipped. Provenance: git-free meta.json + config snapshot.
cl_run_one() {
  local ds="$1" rate="$2" run="$3" lambda="$4"
  local cfg="$CFGDIR/${ds}_params.yaml"
  local input="$REPO/experiments/datasets/2D/${ds}/${DATADIR_NAME}/${rate}/${run}.g2o"
  local gt="$REPO/experiments/datasets/2D/${ds}/GT.txt"
  local sig; sig="$(signature "$lambda")"
  local outdir="$REPO/experiments/results/${RES}/${ds}/${DATE}/${SCEN_TAG}/${sig}/${rate}"
  local final_pr="$outdir/${run}.PR"

  valid_pr "$final_pr" && return 0
  [ -f "$cfg" ]   || { echo "MISSING cfg   $cfg"   >&2; return 1; }
  [ -f "$input" ] || { echo "MISSING input $input" >&2; return 1; }

  mkdir -p "$outdir"
  local scratch="$outdir/.tmp_${run}"; rm -rf "$scratch"; mkdir -p "$scratch"
  local yaml="$scratch/run.yaml"; cp "$cfg" "$yaml"
  yq -i ".dataset=\"$input\""             "$yaml"
  yq -i ".ground_truth=\"$gt\""           "$yaml"
  yq -i ".output=\"$scratch/${run}.TRJ\"" "$yaml"
  apply_overrides "$yaml" "$lambda"

  local bin_sha src_sha data_sha
  bin_sha="$(sha256sum "$BIN" 2>/dev/null | cut -c1-16)"
  src_sha="$(for d in $SRC; do cat "$d"/*.cpp "$d"/*.hpp 2>/dev/null; done | sha256sum | cut -c1-16)"
  data_sha="$(sha256sum "$input" 2>/dev/null | cut -c1-16)"
  local t0; t0=$(date +%s)
  ( cd "$scratch" && "$BIN" -c "$yaml" ) >"$scratch/log.txt" 2>&1
  local rc=$?; local wall=$(( $(date +%s) - t0 ))

  if [ $rc -eq 0 ] && valid_pr "$scratch/${run}.PR"; then
    cat > "$scratch/${run}.meta.json" <<EOF
{
  "method": "$METHOD", "dataset": "$ds", "rate": $rate, "run": "$run",
  "lambda": $lambda, "s_factor": ${S_FACTOR:-0}, "use_kbl": ${USE_KBL:-0}, "k_buddies": ${K_BUDDIES:-0}, "use_recovery": ${USE_REC:-0},
  "scenario_tag": "$SCEN_TAG", "datadir": "$DATADIR_NAME",
  "data_file": "$input", "data_sha256": "$data_sha",
  "bin_sha256": "$bin_sha", "src_sha256": "$src_sha", "date": "$DATE", "wall_s": $wall
}
EOF
    mv -f "$scratch/${run}.TRJ"       "$outdir/${run}.TRJ" 2>/dev/null
    mv -f "$scratch/${run}.PR"        "$final_pr"
    mv -f "$scratch/${run}.meta.json" "$outdir/${run}.meta.json"
    cp -f "$yaml"                     "$outdir/${run}.cfg.yaml"
    rm -rf "$scratch"; return 0
  fi
  echo "FAIL $METHOD $ds $rate $run lam=$lambda rc=$rc (kept $scratch/log.txt)" >&2
  return 1
}

# Entry point. cl_dispatch "$0" "$@"
#   worker mode:  cl_dispatch <self> --one <ds> <rate> <run> <lambda>
#   launch mode:  builds the ds×rate×run×lambda matrix and runs it CAP-wide.
cl_dispatch() {
  local self="$1"; shift
  if [ "${1:-}" = "--one" ]; then shift; cl_run_one "$@"; exit $?; fi

  # Human positional CLI: <datasets> <rates> <runs> [lambdas]. Comma-lists + "all".
  # Selecting datasets by name is REQUIRED in spirit — there is no default-all.
  case "${1:-}" in -h|--help) cl_usage "$self"; exit 0;; esac
  [ -n "${1:-}" ] && DATASETS="$(cl_expand "$1" "$CL_ALL_DATASETS")"
  [ -n "${2:-}" ] && RATES="$(cl_expand "$2" "$CL_ALL_RATES")"
  [ -n "${3:-}" ] && RUNS="$(cl_expand "$3" "$CL_ALL_RUNS")"
  [ -n "${4:-}" ] && LAMBDAS="$(cl_expand "$4" "")"

  local jobs; jobs="$(mktemp)"
  local ds rate run lam
  for ds in $DATASETS; do for rate in $RATES; do for run in $RUNS; do for lam in $LAMBDAS; do
    echo "$ds $rate $run $lam"
  done; done; done; done > "$jobs"
  local total; total=$(wc -l < "$jobs")

  echo "[$(date)] START $METHOD | $total runs | cap=${CAP:-8} | scen=$SCEN_TAG ($DATADIR_NAME) | rates=$RATES | lambdas=$LAMBDAS | $DATASETS"
  xargs -P "${CAP:-8}" -L1 bash "$self" --one < "$jobs"
  rm -f "$jobs"

  local done_n; done_n=$(find "$REPO/experiments/results/${RES}" -name '*.PR' 2>/dev/null | wc -l)
  echo "[$(date)] DONE $METHOD — valid .PR under results/${RES}: $done_n"
  [ "${SERVICE:-0}" = "1" ] && { echo "[$(date)] idling (docker rm -f to clean up)."; sleep infinity; }
}
