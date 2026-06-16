#!/bin/bash
# IPC campaign runner — independent. Shares only the experiment SYSTEM (campaign_lib.sh:
# atomic save / resume / provenance). Replication: DATADIR_NAME=SPOILED_DATA (random).
# G1 on correlated: DATADIR_NAME=SPOILED_DATA_CORR SCEN_TAG=corr.
#
# Env: DATASETS RATES RUNS S_FACTOR DATADIR_NAME SCEN_TAG DATE CAP
# Single job: --one <ds> <rate> <run> <lambda(ignored)>
set -u
source "$(dirname "$0")/lib/campaign_lib.sh"

METHOD=ipc; RES=IPC
BIN="$REPO/build/ipc/ipc_tester_2D"
CFGDIR="$REPO/ipc/cfg/2D"
SRC="$REPO/ipc/src"

S_FACTOR="${S_FACTOR:-3}"
DATASETS="${DATASETS:-M3500}"
RATES="${RATES:-10 20 30 40 50}"
RUNS="${RUNS:-00 01 02 03 04 05 06 07 08 09}"
LAMBDAS="0"                              # IPC has no lambda; single value drives the matrix
DATADIR_NAME="${DATADIR_NAME:-SPOILED_DATA}"
SCEN_TAG="${SCEN_TAG:-rand}"
DATE="${DATE:-$(date +%y%m%d)}"
CAP="${CAP:-8}"

apply_overrides() { yq -i ".s_factor=${S_FACTOR}.0" "$1"; }
signature() { echo "ipc_s${S_FACTOR}"; }

cl_dispatch "$0" "$@"
