#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BASE_URL="${BASE_URL:-http://192.168.11.2:12300}"
MOTION_JSON_FILE=""

if [[ "${1:-}" != "--execute" || $# -ne 2 ]]; then
  cat >&2 <<'EOF'
危险：此脚本会把关节目标发送到真机，可能立即引发机器人运动。
默认未发送任何 HTTP 请求。清空运动区、逐项复核 JSON、确认急停手段后，显式执行：
  ./call_motion.sh --execute /path/to/reviewed-motion.json
必须显式提供已审核文件；仓库占位示例不能作为真机目标。
EOF
  exit 2
fi

MOTION_JSON_FILE="$2"
if [[ ! -f "${MOTION_JSON_FILE}" ]]; then
  echo "motion json file not found: ${MOTION_JSON_FILE}" >&2
  exit 1
fi
if [[ "$(readlink -f -- "${MOTION_JSON_FILE}")" == \
      "$(readlink -f -- "${SCRIPT_DIR}/motion.example.json")" ]]; then
  echo "refusing to execute the bundled placeholder motion.example.json" >&2
  exit 2
fi

headers=(-H "Content-Type: application/json")
if [[ -n "${MOTION_PROXY_API_KEY:-}" ]]; then
  headers+=(-H "X-API-Key: ${MOTION_PROXY_API_KEY}")
fi

echo "DANGER: sending motion from ${MOTION_JSON_FILE} to ${BASE_URL}" >&2
curl --fail --show-error \
  "${headers[@]}" \
  --data-binary "@${MOTION_JSON_FILE}" \
  "${BASE_URL}/v1/motions"
echo
