#!/usr/bin/env python3
"""Walker S2 IK 求解器独立测试脚本。

用法:
    # 从 ROS2 环境
    python test_ik.py --urdf /path/to/walker_s2.urdf \\
                      --pkg-dir /path/to/ros_ws/src

    # 独立使用（需要 arm_ik 在 PYTHONPATH 中）
    PYTHONPATH=.:$PYTHONPATH python test_ik.py --urdf /path/to/walker_s2.urdf

    # 使用预编译 .so
    python test_ik.py --urdf /path/to/walker_s2.urdf --so ik_solver.so
"""

import argparse
import os
import sys

import numpy as np

try:
    from box_grasp_demo.arm_ik.walker_ik import WalkerArmIK
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
    from walker_ik import WalkerArmIK


def main():
    parser = argparse.ArgumentParser(description="Walker S2 IK 求解器测试")
    parser.add_argument("--urdf", required=True, help="Walker S2 URDF 文件路径")
    parser.add_argument("--mesh", default=None, help="Mesh 搜索路径")
    parser.add_argument("--pkg-dir", action="append", default=[],
                        help="用于解析 package:// 前缀的搜索路径（可多次指定）")
    parser.add_argument("--so", default=None, help="预编译 .so 文件路径")
    parser.add_argument("--jit", action="store_true", help="启用 CasADi JIT")
    args = parser.parse_args()

    # 初始化
    so_path = None
    if args.so:
        so_path = args.so
        if not os.path.exists(so_path):
            so_path = os.path.join(os.getcwd(), ".casadi_cache", args.so)
        if not os.path.exists(so_path):
            print(f"[WARN] {args.so} 不存在，回退到 Python 求解器")
            so_path = None

    print(f"[INFO] URDF: {args.urdf}")
    print(f"[INFO] SO:   {so_path}")
    print(f"[INFO] JIT:  {args.jit}")

    ik = WalkerArmIK(
        urdf_path=args.urdf,
        mesh_path=args.mesh,
        package_dirs=args.pkg_dir if args.pkg_dir else None,
        jit=args.jit,
        so_path=so_path,
    )

    # 用 pregrasp 姿态的 FK 生成可达目标
    q_pregrasp = np.array([
        0.0, 0.0,  np.pi/2, -np.pi/2, -np.pi/2, 0.0, 0.0,   # 左手 7 DOF
        0.0, 0.0, -np.pi/2,  np.pi/2,  np.pi/2, 0.0, 0.0,   # 右手 7 DOF
    ])
    L_target, R_target = ik.fk(q_pregrasp)
    print(f"\n目标（由 pregrasp FK 生成）:")
    print(f"  左手: {np.array2string(L_target[:3, 3], precision=5)}")
    print(f"  右手: {np.array2string(R_target[:3, 3], precision=5)}")

    print("\n=== IK 求解 ===\n")
    sol_q = ik.solve_ik(L_target, R_target, q_init=np.zeros(14), max_iters=3)

    sol_L = ik.get_arm_q(sol_q, side="L")
    sol_R = ik.get_arm_q(sol_q, side="R")

    print("\n----------------- 右手 -------------------")
    print(f"关节: {np.array2string(sol_R, precision=5, suppress_small=True)}")
    R_fk = ik.reduced_robot.framePlacement(sol_q, ik.R_hand_id).translation
    print(f"FK:   {np.array2string(R_fk, precision=5, suppress_small=True)}")
    print(f"目标: {np.array2string(R_target[:3, 3], precision=5, suppress_small=True)}")

    print("\n----------------- 左手 -------------------")
    print(f"关节: {np.array2string(sol_L, precision=5, suppress_small=True)}")
    L_fk = ik.reduced_robot.framePlacement(sol_q, ik.L_hand_id).translation
    print(f"FK:   {np.array2string(L_fk, precision=5, suppress_small=True)}")
    print(f"目标: {np.array2string(L_target[:3, 3], precision=5, suppress_small=True)}")

    # 精度汇总
    err_R = float(np.linalg.norm(R_fk - R_target[:3, 3]))
    err_L = float(np.linalg.norm(L_fk - L_target[:3, 3]))
    print(f"\n============================================")
    print(f"精度汇总: 右手 {err_R*1000:.3f}mm, 左手 {err_L*1000:.3f}mm")
    print(f"============================================")


if __name__ == "__main__":
    main()
