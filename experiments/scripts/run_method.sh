#!/usr/bin/env bash
# run_method.sh — run ONE robust-PGO method on ONE config, normalising the
# per-tool CLI differences. Part of the S0.2 experiment scaffold (see TASKS.md).
#
# Usage:
#   run_method.sh <binary_name> <config.yaml> [out_dir]
#
# Examples:
#   run_method.sh ipc_tester_2D  cfg/2D/M3500_params.yaml
#   run_method.sh IN_SC_2D       experiments/configs/M3500_sc.yaml
#   run_method.sh gtsam_PCM_2D   experiments/configs/M3500_pcm.yaml  experiments/results/M3500
#
# The three tool families take their config DIFFERENTLY (learned from the
# baseline README + IPC README):
#   - IPC          (ipc_tester_*) : -c   <cfg>
#   - g2o baselines (IN_* / off)  : -cfg <cfg>
#   - GTSAM tier   (gtsam_*)      : <cfg>   (positional, no flag)
#
# Each binary writes <output>.txt (trajectory) + <output>.PR (precision recall time),
# where <output> is the `output:` key inside the config.
#
# Binaries are LOCATED by search under build/ so this survives the superbuild's
# exact output layout (build/ipc/…, build/baselines/robust_g2o/src/incr/…, etc.).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"   # configs use repo-root-relative dataset paths

BIN_NAME="${1:?usage: run_method.sh <binary_name> <config.yaml> [out_dir]}"
CFG="${2:?missing config.yaml}"
OUT_DIR="${3:-}"

[ -f "$CFG" ] || { echo "ERROR: config not found: $CFG" >&2; exit 1; }

# locate the binary inside build/
BIN="$(find build -type f -name "$BIN_NAME" -perm -u+x 2>/dev/null | head -n1 || true)"
[ -n "$BIN" ] || { echo "ERROR: binary '$BIN_NAME' not found under build/ — build first (see S0.1)." >&2; exit 1; }

# pick the config-passing convention from the binary name
case "$BIN_NAME" in
  ipc_tester_*)        FLAG="-c" ;;
  gtsam_*)             FLAG="" ;;        # positional
  *)                   FLAG="-cfg" ;;    # g2o baselines (IN_*, offline)
esac

echo ">> $BIN_NAME  ($BIN)"
echo ">> cfg: $CFG"
if [ -n "$FLAG" ]; then
  "$BIN" "$FLAG" "$CFG"
else
  "$BIN" "$CFG"
fi

# collect outputs next to results if an out_dir was given
if [ -n "$OUT_DIR" ]; then
  mkdir -p "$OUT_DIR"
  OUT_BASE="$(grep -E '^\s*output\s*:' "$CFG" | head -n1 | sed -E 's/.*:\s*"?([^"]+)"?.*/\1/')"
  for ext in "" .PR .txt; do
    [ -f "${OUT_BASE}${ext}" ] && mv -f "${OUT_BASE}${ext}" "$OUT_DIR/" || true
  done
  echo ">> outputs moved to $OUT_DIR/"
fi
