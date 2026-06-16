#!/bin/bash
# TACO campaign runner — independent. Shares only the experiment SYSTEM (campaign_lib.sh).
# TACO 2D default s=10; kBL/recovery are TACO's live knobs.
#
# Env: DATASETS RATES RUNS S_FACTOR USE_KBL K_BUDDIES USE_REC DATADIR_NAME SCEN_TAG DATE CAP
# Single job: --one <ds> <rate> <run> <lambda(ignored)>
set -u
source "$(dirname "$0")/lib/campaign_lib.sh"

METHOD=taco; RES=TACO
BIN="$REPO/build/taco/taco_tester_2D"
CFGDIR="$REPO/taco/cfg/2D"
SRC="$REPO/taco/src"

S_FACTOR="${S_FACTOR:-10}"
USE_KBL="${USE_KBL:-1}"
K_BUDDIES="${K_BUDDIES:-2}"
USE_REC="${USE_REC:-1}"
DATASETS="${DATASETS:-M3500}"
RATES="${RATES:-10 20 30 40 50}"
RUNS="${RUNS:-00 01 02 03 04 05 06 07 08 09}"
LAMBDAS="0"
DATADIR_NAME="${DATADIR_NAME:-SPOILED_DATA}"
SCEN_TAG="${SCEN_TAG:-rand}"
DATE="${DATE:-$(date +%y%m%d)}"
CAP="${CAP:-8}"

apply_overrides() {
  local y="$1"
  local kbl=false; [ "$USE_KBL" = "1" ] && kbl=true
  local rec=false; [ "$USE_REC" = "1" ] && rec=true
  yq -i ".s_factor=${S_FACTOR}.0"        "$y"
  yq -i ".use_best_k_buddies=${kbl}"     "$y"
  yq -i ".k_buddies=${K_BUDDIES}"        "$y"
  yq -i ".use_recovery=${rec}"           "$y"
}
signature() { echo "taco_s${S_FACTOR}_k${K_BUDDIES}_rec${USE_REC}"; }

cl_dispatch "$0" "$@"
