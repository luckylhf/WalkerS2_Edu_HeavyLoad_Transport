#!/bin/bash
# 真机端录制一帧静态场景 rosbag，用于下载到本地做离线仿真。
#
# 用法:
#   ./tools/record_current_bag_remote.sh [输出目录] [录制秒数]
# 默认:
#   输出目录: /tmp/tf_analysis_current
#   录制秒数: 15
#
# 录制话题与 tools/extract_current_bag.py 保持一致:
#   /tf_static /tf /sensor/camera/stereo/pointcloud/raw /mc/joint_states
set -eo pipefail

OUT="${1:-/tmp/tf_analysis_current}"
DURATION="${2:-15}"

source /opt/ros/humble/setup.bash

TOPICS=(
  /tf_static
  /tf
  /sensor/camera/stereo/pointcloud/raw
  /mc/joint_states
)

PARENT="$(dirname "${OUT}")"
mkdir -p "${PARENT}"
rm -rf "${OUT}"

echo "[record] topics: ${TOPICS[*]}"
echo "[record] output: ${OUT}  duration: ${DURATION}s"

# SIGINT 让 ros2 bag record 正常收尾写入 db3；超时是预期行为。
timeout --signal=INT --kill-after=5s "${DURATION}s" \
  ros2 bag record -o "${OUT}" "${TOPICS[@]}" \
  || echo "[record] stopped by timeout (expected)"

echo "[record] saved files:"
find "${OUT}" -maxdepth 2 -type f | sort
