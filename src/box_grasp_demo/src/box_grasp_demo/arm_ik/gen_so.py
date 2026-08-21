#!/usr/bin/env python3
"""将 Walker S2 IK 求解器编译为 .so 动态库。

用法:
    # 从 ROS2 环境
    python gen_so.py --urdf /path/to/walker_s2.urdf \\
                     --pkg-dir /path/to/ros_ws/src

    # 独立使用（需要 arm_ik 在 PYTHONPATH 中）
    PYTHONPATH=.:$PYTHONPATH python gen_so.py --urdf /path/to/walker_s2.urdf
"""

import argparse
import os
import sys

# 支持独立执行和包内导入
try:
    from box_grasp_demo.arm_ik.walker_ik import WalkerArmIK
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
    from walker_ik import WalkerArmIK


def main():
    parser = argparse.ArgumentParser(description="编译 Walker S2 IK 求解器为 .so")
    parser.add_argument("--urdf", required=True, help="Walker S2 URDF 文件路径")
    parser.add_argument("--mesh", default=None, help="Mesh 搜索路径（默认与 URDF 同目录）")
    parser.add_argument("--pkg-dir", action="append", default=[],
                        help="用于解析 package:// 前缀的搜索路径（可多次指定）")
    parser.add_argument("--output", default="ik_solver.so", help="输出 .so 路径")
    parser.add_argument("--conda-prefix", default=None,
                        help="Conda 环境路径（默认使用 $CONDA_PREFIX）")
    args = parser.parse_args()

    cache_dir = os.path.join(os.getcwd(), ".casadi_cache")
    os.makedirs(cache_dir, exist_ok=True)

    so_path = os.path.join(cache_dir, args.output)

    ik = WalkerArmIK(
        urdf_path=args.urdf,
        mesh_path=args.mesh,
        package_dirs=args.pkg_dir if args.pkg_dir else None,
        jit=False,
    )
    ik.export_so(output_so=so_path, conda_prefix=args.conda_prefix)


if __name__ == "__main__":
    main()
