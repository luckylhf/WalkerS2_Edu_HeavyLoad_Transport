#!/bin/bash
set -e

IK_ENV="ik"
CONDA_BASE="${HOME}/miniconda3"
NAMESPACE="/demo3"
CAMERA_TOPIC="/sensor/camera/stereo/pointcloud/raw"

echo "=== 编译 ==="
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

echo ""
echo "=== 检测 IK 环境: ${IK_ENV} ==="
source "${CONDA_BASE}/etc/profile.d/conda.sh"
conda activate "${IK_ENV}"

# 获取 conda Python site-packages 路径
PY_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
IK_PYTHONPATH="${CONDA_PREFIX}/lib/python${PY_VER}/site-packages"

# 验证 pinocchio 可用
python3 -c "import pinocchio; print('pinocchio', pinocchio.__version__)"
python3 -c "import pinocchio.casadi; print('pinocchio.casadi OK')"

echo ""
echo "============================================"
echo "  Walker S2 感知 + IK"
echo "  命名空间: ${NAMESPACE}"
echo "  IK PYTHONPATH: ${IK_PYTHONPATH}"
echo "============================================"
echo "  查看检测: ros2 topic echo ${NAMESPACE}/box_grasp_demo/status --once"
echo "  查看 IK:   ros2 topic echo ${NAMESPACE}/joint_states --once"
echo "  切换模式: ros2 topic pub ${NAMESPACE}/pose_command std_msgs/String \"data: 'grasp'\" -1"
echo "============================================"
echo ""

# 将 IK_PYTHONPATH 导出到环境变量，launch 文件会读取它
export IK_PYTHONPATH

# ---- 生成 / 复用 IK .so 求解器，通过 so_path 传入 launch ----
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

ros2 launch box_grasp_demo box_grasp_demo.launch.py \
    ns:=${NAMESPACE} ik:=true rviz:=false \
    input_topic:=${CAMERA_TOPIC} \
    so_path:=${SO_PATH}
