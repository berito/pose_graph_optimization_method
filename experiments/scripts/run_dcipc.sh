#!/bin/bash
# DC-IPC ablation runner — independent. Shares only the experiment SYSTEM (campaign_lib.sh).
# The contribution: varies LAMBDAS (0 ≡ IPC). Default scenario = correlated data.
#
# Env: DATASETS RATES RUNS LAMBDAS S_FACTOR USE_KBL K_BUDDIES USE_REC DATADIR_NAME SCEN_TAG DATE CAP
# Single job: --one <ds> <rate> <run> <lambda>
set -u
source "$(dirname "$0")/lib/campaign_lib.sh"

METHOD=dcipc; RES=DC_IPC
BIN="$REPO/build/dc_ipc/dc_ipc_tester_2D"
CFGDIR="$REPO/dc_ipc/cfg/2D"
SRC="$REPO/dc_ipc/src $REPO/dc_ipc/include/dc_ipc"

S_FACTOR="${S_FACTOR:-3}"
USE_KBL="${USE_KBL:-0}"
K_BUDDIES="${K_BUDDIES:-0}"
USE_REC="${USE_REC:-0}"
DATASETS="${DATASETS:-M3500}"
RATES="${RATES:-10 20 30 40 50}"
RUNS="${RUNS:-00 01 02 03 04 05 06 07 08 09}"
LAMBDAS="${LAMBDAS:-0 50}"               # the headline ablation axis (0 ≡ IPC)
DATADIR_NAME="${DATADIR_NAME:-SPOILED_DATA_CORR}"
SCEN_TAG="${SCEN_TAG:-corr}"
DATE="${DATE:-$(date +%y%m%d)}"
CAP="${CAP:-8}"

apply_overrides() {
  local y="$1" lam="$2"
  local kbl=false; [ "$USE_KBL" = "1" ] && kbl=true
  local rec=false; [ "$USE_REC" = "1" ] && rec=true
  yq -i ".s_factor=${S_FACTOR}.0"        "$y"
  yq -i ".use_best_k_buddies=${kbl}"     "$y"
  yq -i ".k_buddies=${K_BUDDIES}"        "$y"
  yq -i ".use_recovery=${rec}"           "$y"
  yq -i ".lambda=${lam}.0"               "$y"
}
signature() { echo "lam${1}_kbl${K_BUDDIES}_rec${USE_REC}"; }

cl_dispatch "$0" "$@"
