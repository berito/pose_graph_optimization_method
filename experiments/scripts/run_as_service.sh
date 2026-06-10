#!/bin/bash
# Run the IPC campaign in a Docker container.
# Env: RESTART(no|unless-stopped) NAME CAP DATASETS RATES RUNS TAG DATE
set -euo pipefail

NAME="${NAME:-ipc_campaign_s1}"
RESTART="${RESTART:-unless-stopped}"
REPO="/home/ai-server-01/code_base/robust_pgo"
SCRIPT="${SCRIPT:-experiments/scripts/run_ipc_campaign.sh}"

ENV_ARGS=()
for v in CAP DATASETS RATES RUNS TAG DATE; do
  [ -n "${!v:-}" ] && ENV_ARGS+=("-e" "$v=${!v}")
done

IMAGE="${IMAGE:-$(docker images --format '{{.Repository}}:{{.Tag}}' | grep -m1 '^vsc-robust_pgo' || true)}"
[ -n "$IMAGE" ] || { echo "ERROR: vsc-robust_pgo image not found." >&2; exit 1; }

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "ERROR: '$NAME' already running (docker logs -f $NAME / docker stop $NAME)." >&2; exit 1
fi
docker rm -f "$NAME" >/dev/null 2>&1 || true

docker run -d --name "$NAME" \
  --restart "$RESTART" \
  --user "$(id -u):$(id -g)" \
  -e SERVICE=1 \
  "${ENV_ARGS[@]}" \
  -v "$REPO:/workspaces/robust_pgo" \
  -w /workspaces/robust_pgo \
  "$IMAGE" \
  bash "$SCRIPT"

echo "started '$NAME' (image=$IMAGE restart=$RESTART) — docker logs -f $NAME"
