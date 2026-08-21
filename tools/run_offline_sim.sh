#!/bin/bash
# Walker S2 双臂抱箱离线真机点云启动脚本
#
# 用法:
#   ./tools/run_offline_sim.sh       用当前真机 rosbag 提取的一帧点云，启用 IK
#   ./tools/run_offline_sim.sh false 同上，但不启动 IK

IK="${1:-true}"
ROS_PYTHON="/usr/bin/python3"
IK_PYTHON=~/miniconda3/envs/ik/bin/python3

if [ ! -x "$ROS_PYTHON" ] || [ ! -x "$IK_PYTHON" ]; then
  echo "Missing required Python runtime: ROS=$ROS_PYTHON, IK=$IK_PYTHON" >&2
  exit 1
fi

# Keep ROS nodes on the ROS Humble Python ABI (3.10).  IK 脚本使用可移植的
# `#!/usr/bin/env python3` shebang，运行前通过 conda activate ik 让 python3
# 指向提供 Pinocchio/CasADi 的 ik 环境；ros2/colcon 本身仍固定用 /usr/bin/python3。
export PATH="/usr/bin:$PATH"
export CONDA_PREFIX=~/miniconda3/envs/ik
export IK_PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages"
# 从静止真机 rosbag 中提取中间一帧；只读取一次，运行时持续重放这同一帧。
"$ROS_PYTHON" tools/extract_current_bag.py

# 旧版 ament_python 的 --symlink-install 缓存可能将应为软链接的位置
# 留成普通目录，随后会报“existing path cannot be removed”。只清理受影响
# 的本包构建/安装缓存，保留其余工作区产物。
if [ -d build/box_grasp_demo/ament_cmake_python/box_grasp_demo/box_grasp_demo ] || \
   [ -f build/box_grasp_demo/CMakeCache.txt ] && \
   grep -q "$HOME/miniconda3" build/box_grasp_demo/CMakeCache.txt; then
  rm -rf build/box_grasp_demo install/box_grasp_demo
fi

colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE="$ROS_PYTHON"
source install/setup.bash
# 激活 ik 环境，使 IK 脚本的 `#!/usr/bin/env python3` 解析到 Pinocchio/CasADi 解释器。
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ik

# ---- 生成 / 复用 IK .so 求解器，通过 so_path 传入 launch ----
SO_LAUNCH_ARG=""
if [ "${IK}" = "true" ]; then
  SO_CACHE_DIR="$PWD/.casadi_cache"
  SO_PATH="$SO_CACHE_DIR/ik_solver.so"
  URDF_PATH="$PWD/install/walker_s2_description/share/walker_s2_description/urdf/s2_v1/s2_v1.urdf"
  PKG_DIR="$PWD/install/walker_s2_description/share"
  GEN_SO="$PWD/src/box_grasp_demo/src/box_grasp_demo/arm_ik/gen_so.py"
  WALKER_IK_PY="$PWD/src/box_grasp_demo/src/box_grasp_demo/arm_ik/walker_ik.py"

  if [ ! -f "$SO_PATH" ] || \
     [ "$SO_PATH" -ot "$GEN_SO" ] || \
     [ "$SO_PATH" -ot "$WALKER_IK_PY" ] || \
     [ "$SO_PATH" -ot "$URDF_PATH" ]; then
    echo "=== 生成 IK .so 求解器 ==="
    mkdir -p "$SO_CACHE_DIR"
    LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH" \
    PYTHONPATH="$PWD/src/box_grasp_demo/src:$PYTHONPATH" \
      python3 "$GEN_SO" \
        --urdf "$URDF_PATH" \
        --pkg-dir "$PKG_DIR" \
        --pkg-dir "$PKG_DIR/walker_s2_description" \
        --output ik_solver.so
  fi
  SO_LAUNCH_ARG="so_path:=${SO_PATH}"
fi

ros2 launch box_grasp_demo box_grasp_recorded_frame.launch.py \
  frame_data_dir:="$PWD/robot_data/current_bag" ik:=${IK} rviz:=true \
  ${SO_LAUNCH_ARG}
