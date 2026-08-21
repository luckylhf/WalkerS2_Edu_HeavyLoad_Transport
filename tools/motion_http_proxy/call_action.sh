#!/usr/bin/env bash
set -euo pipefail

BASE_URL="${BASE_URL:-http://192.168.11.2:12300}"

if [[ "${1:-}" != "--execute" || $# -ne 1 ]]; then
  cat >&2 <<'EOF'
危险：此脚本会请求 s2/move_biped_home，机器人可能立即运动或恢复站立。
默认未发送任何 HTTP 请求。清空运动区、确认姿态和急停手段后，显式执行：
  ./call_action.sh --execute
EOF
  exit 2
fi

headers=(-H "Content-Type: application/json")
if [[ -n "${MOTION_PROXY_API_KEY:-}" ]]; then
  headers+=(-H "X-API-Key: ${MOTION_PROXY_API_KEY}")
fi

echo "DANGER: sending s2/move_biped_home to ${BASE_URL}" >&2
curl --fail --show-error \
  "${headers[@]}" \
  --data-binary '{"action":"s2/move_biped_home"}' \
  "${BASE_URL}/v1/call_action"
echo
