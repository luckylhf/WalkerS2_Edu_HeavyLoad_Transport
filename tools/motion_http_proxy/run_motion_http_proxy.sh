#!/usr/bin/env bash
# The generated colcon setup scripts may reference unset variables such as
# COLCON_TRACE, so nounset must stay disabled while sourcing them.
set -eo pipefail

WALKER_SETUP="${WALKER_SETUP:-/opt/walker/setup.bash}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROXY_BIN="${PROXY_BIN:-${SCRIPT_DIR}/motion_http_proxy}"

# Keep compatibility with the CMake layout used during local development.
if [[ ! -x "${PROXY_BIN}" && -x "${SCRIPT_DIR}/motion_http_proxy" ]]; then
  PROXY_BIN="${SCRIPT_DIR}/motion_http_proxy"
fi

if [[ ! -f "${WALKER_SETUP}" ]]; then
  echo "missing Walker environment: ${WALKER_SETUP}" >&2
  exit 1
fi

if [[ ! -x "${PROXY_BIN}" ]]; then
  echo "proxy binary is not executable: ${PROXY_BIN}" >&2
  exit 1
fi

source "${WALKER_SETUP}"

set -u

if ! command -v rosa >/dev/null 2>&1; then
  echo "rosa was not found after sourcing ${WALKER_SETUP}" >&2
  exit 1
fi

# Container-side default. Docker should publish the host side as 127.0.0.1:12300.
export HTTP_PROXY_HOST="${HTTP_PROXY_HOST:-0.0.0.0}"
export HTTP_PROXY_PORT="${HTTP_PROXY_PORT:-12300}"

exec "${PROXY_BIN}"
