#!/bin/bash
# 在真机录制当前静态场景，并下载到本地 robot_data/tf_analysis_current。
# 下载后即可运行 ./tools/run_offline_sim.sh 做离线仿真。
#
# 用法:
#   ./tools/download_current_bag.sh [录制秒数]
set -euo pipefail

DURATION="${1:-15}"
REMOTE="s2_3_ros"
REMOTE_SCRIPT="/home/ubt/demo3/tools/record_current_bag_remote.sh"
REMOTE_DIR="/tmp/tf_analysis_current"
LOCAL_DIR="robot_data/tf_analysis_current"
DB="tf_analysis_current_0.db3"
METADATA="metadata.yaml"

echo "[download] recording on ${REMOTE} for ${DURATION}s ..."
ssh -o BatchMode=yes "${REMOTE}" \
  "bash ${REMOTE_SCRIPT} ${REMOTE_DIR} ${DURATION}"

echo "[download] copying to ${LOCAL_DIR} ..."
mkdir -p "${LOCAL_DIR}"
scp -o BatchMode=yes "${REMOTE}:${REMOTE_DIR}/${DB}" "${LOCAL_DIR}/${DB}"
scp -o BatchMode=yes "${REMOTE}:${REMOTE_DIR}/${METADATA}" "${LOCAL_DIR}/${METADATA}" || true

echo "[download] local files:"
ls -la "${LOCAL_DIR}"
echo "[download] done. Run ./tools/run_offline_sim.sh to simulate."
