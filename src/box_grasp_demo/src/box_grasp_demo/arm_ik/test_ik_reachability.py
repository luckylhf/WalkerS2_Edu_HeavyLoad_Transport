#!/usr/bin/env python3
"""测试 Walker S2 双臂 IK 在不同桌面高度下的可解算范围。

目的：
    找到“桌面在 base_link 系的高度（table_z_in_base）”在什么区间内，
    pregrasp / grasp / aftergrasp / place 四组双臂 IK 都能收敛到阈值以内，
    从而给出“桌子不能太高也不能太矮”的上下界。

背景公式（与 box_grasp_node_ros2.py 一致）：
    table_z_in_base = table_height - base_link_height
    crouch           = (table_height - desired_table_z) - base_link_height

    IK 只关心最终手腕目标在 base_link 系的位置，因此本测试直接扫描
    table_z_in_base；对给定站立 base_link 高度，可换算成绝对桌面高度。

用法（在 conda ``ik`` 环境中）：
    python test_ik_reachability.py \
        --urdf ~/work/s2_demo3/src/walker_s2_description/urdf/s2_v1/s2_v1.urdf \
        --pkg-dir ~/work/s2_demo3/src \
        --z-min -0.20 --z-max 0.40 --z-step 0.02
"""

import argparse
import math
import os
import sys

import numpy as np

try:
    from box_grasp_demo.arm_ik.walker_ik import WalkerArmIK
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
    from walker_ik import WalkerArmIK


# 与 config/box_grasp_ros2.yaml 保持一致。
PREGrasp_Q = np.array([
    0.0, -0.785, -1.57, -1.57, 1.57, 0.0, 0.0,   # L 臂 7 关节
    0.0, -0.785, 1.57, -1.57, -1.57, 0.0, 0.0,   # R 臂 7 关节
])

LEFT_WRIST_TO_GRASP_RPY = np.array([-1.570790312, -0.000007873, 1.570783077])
LEFT_WRIST_TO_GRASP_T = np.array([-0.068178153, 0.093897912, -0.021682127])
RIGHT_WRIST_TO_GRASP_RPY = np.array([1.570783077, 0.000009148, -1.570790312])
RIGHT_WRIST_TO_GRASP_T = np.array([-0.068178153, 0.093897912, 0.021682127])

# 当前 box_model=small_box 的完整参数（small_box 覆盖 base_box 的字段）。
BOX_MODEL = {
    "box_length": 0.39,
    "box_width": 0.30,
    "box_height": 0.12,
    "tool_contact_below_top": 0.035,
    "side_clearance": -0.02,
    "pregrasp_distance": 0.08,
    "grasp_long_edge": False,
}

# 抓取桌箱子中心（当前 YAML 注释中的目标位置 X≈0.48, Y=0）。
GRASP_BOX_XY = np.array([0.48, 0.0])
# 放置位姿来自 place_x / place_y / place_yaw_deg。
PLACE_BOX_XY = np.array([0.40, 0.0])
PLACE_YAW_DEG = -90.0
PLACE_CLEARANCE = 0.005

POS_THRESHOLD = 1e-3
ORI_THRESHOLD = 1e-3
MAX_ITERS = 3


def rpy_matrix(rpy):
    """ROS/URDF 固定轴 XYZ RPY 旋转矩阵（与 box_grasp_node_ros2.py 一致）。"""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def tool_rotation(inward):
    """从抓取 inward 向量构造 tool 坐标系旋转（与主程序 _tool_rotation 一致）。"""
    inward = np.asarray(inward, dtype=np.float64)
    tool_y = np.array([0.0, 0.0, 1.0])
    tool_z = inward - tool_y * np.dot(tool_y, inward)
    tool_z_norm = np.linalg.norm(tool_z)
    tool_z = tool_z / tool_z_norm if tool_z_norm > 1e-9 else np.array([0.0, 1.0, 0.0])
    tool_x = np.cross(tool_y, tool_z)
    tool_x_norm = np.linalg.norm(tool_x)
    tool_x = tool_x / tool_x_norm if tool_x_norm > 1e-9 else np.array([1.0, 0.0, 0.0])
    return np.column_stack((tool_x, tool_y, tool_z))


def box_rotation_from_yaw(yaw_deg):
    """按主程序 publish_place_targets 的约定构造箱体旋转。

    与 detector 对“长边朝向机器人”箱子的输出一致：
    column0 = 长轴 u，column1 = 短轴 v（朝 +X），column2 = 竖轴 n。
    """
    yaw_rot = rpy_matrix([0.0, 0.0, math.radians(yaw_deg)])
    return np.column_stack((yaw_rot[:, 0], yaw_rot[:, 1], np.array([0.0, 0.0, 1.0])))


def grasp_centers(center, rotation):
    """复刻 detector_open3d.BoxDetector.grasp_centers 的抓取几何。"""
    u, v, n = rotation.T
    contact_height = BOX_MODEL["box_height"] / 2.0 - BOX_MODEL["tool_contact_below_top"]
    half_length = BOX_MODEL["box_length"] / 2.0 + BOX_MODEL["side_clearance"]

    plus = center + u * half_length + n * contact_height
    minus = center - u * half_length + n * contact_height
    if u[1] >= 0:
        left_grasp, right_grasp = plus, minus
        left_inward, right_inward = -u, u
    else:
        left_grasp, right_grasp = minus, plus
        left_inward, right_inward = u, -u

    left_pre = left_grasp - left_inward * BOX_MODEL["pregrasp_distance"]
    right_pre = right_grasp - right_inward * BOX_MODEL["pregrasp_distance"]
    return {
        "left": (left_grasp, left_inward, left_pre),
        "right": (right_grasp, right_inward, right_pre),
    }


def wrist_to_grasp_matrix(rpy, t):
    mat = np.eye(4)
    mat[:3, :3] = rpy_matrix(rpy)
    mat[:3, 3] = t
    return mat


def compute_wrist_targets(center, rotation):
    """返回 (T_l, T_r, T_l_pre, T_r_pre) 四组 4x4 手腕目标。

    与主程序 cloud_callback 中 wrist_tf / wrist_pre_tf 计算一致。
    """
    grasps = grasp_centers(center, rotation)
    wg_inv = {
        "left": np.linalg.inv(wrist_to_grasp_matrix(
            LEFT_WRIST_TO_GRASP_RPY, LEFT_WRIST_TO_GRASP_T)),
        "right": np.linalg.inv(wrist_to_grasp_matrix(
            RIGHT_WRIST_TO_GRASP_RPY, RIGHT_WRIST_TO_GRASP_T)),
    }

    out = {}
    for side, key in (("left", "left"), ("right", "right")):
        grasp, inward, pregrasp = grasps[key]
        rot = tool_rotation(inward)
        grasp_tf = np.eye(4)
        grasp_tf[:3, :3], grasp_tf[:3, 3] = rot, grasp
        pregrasp_tf = grasp_tf.copy()
        pregrasp_tf[:3, 3] = pregrasp
        aftergrasp_tf = grasp_tf.copy()
        aftergrasp_tf[:3, 3] = grasp + np.array([0.0, 0.0, 1.0]) * 0.10
        out[side] = {
            "grasp": grasp_tf @ wg_inv[key],
            "pregrasp": pregrasp_tf @ wg_inv[key],
            "aftergrasp": aftergrasp_tf @ wg_inv[key],
        }
    return out


def fk_error(ik, sol_q, T_l, T_r):
    """返回 (max_pos_error_m, max_rot_error_rad)；未收敛/NaN 返回 (inf, inf)。"""
    if sol_q is None or np.any(np.isnan(sol_q)):
        return float("inf"), float("inf")
    placement_L = ik.reduced_robot.framePlacement(sol_q, ik.L_hand_id)
    placement_R = ik.reduced_robot.framePlacement(sol_q, ik.R_hand_id)
    dist_L = float(np.linalg.norm(placement_L.translation - T_l[:3, 3]))
    dist_R = float(np.linalg.norm(placement_R.translation - T_r[:3, 3]))
    rot_err_L = float(np.linalg.norm(
        _log3(placement_L.rotation.T @ T_l[:3, :3])))
    rot_err_R = float(np.linalg.norm(
        _log3(placement_R.rotation.T @ T_r[:3, :3])))
    return max(dist_L, dist_R), max(rot_err_L, rot_err_R)


def _log3(rot):
    import pinocchio as pin
    return pin.log3(rot)


def solve_one(ik, T_l, T_r):
    """镜像 arm_ik_pinocchio._solve_mode 的收敛判定与一次重试。"""
    sol_q = ik.solve_ik(
        T_l, T_r,
        q_init=PREGrasp_Q,
        max_iters=MAX_ITERS,
        pos_threshold=POS_THRESHOLD,
        ori_threshold=ORI_THRESHOLD,
        verbose=False,
    )
    pos_err, rot_err = fk_error(ik, sol_q, T_l, T_r)
    if pos_err < POS_THRESHOLD and rot_err < ORI_THRESHOLD:
        return True, pos_err, rot_err

    sol_q = ik.solve_ik(
        T_l, T_r,
        q_init=PREGrasp_Q,
        max_iters=MAX_ITERS + 3,
        pos_threshold=POS_THRESHOLD,
        ori_threshold=ORI_THRESHOLD,
        verbose=False,
    )
    pos_err, rot_err = fk_error(ik, sol_q, T_l, T_r)
    return (pos_err < POS_THRESHOLD and rot_err < ORI_THRESHOLD), pos_err, rot_err


def build_ik(args):
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
    return WalkerArmIK(
        urdf_path=args.urdf,
        mesh_path=args.mesh,
        package_dirs=args.pkg_dir if args.pkg_dir else None,
        jit=args.jit,
        so_path=so_path,
    )


def main():
    parser = argparse.ArgumentParser(description="Walker S2 双臂 IK 桌面高度可达性测试")
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--mesh", default=None)
    parser.add_argument("--pkg-dir", action="append", default=[])
    parser.add_argument("--so", default=None)
    parser.add_argument("--jit", action="store_true")
    parser.add_argument("--z-min", type=float, default=-0.20)
    parser.add_argument("--z-max", type=float, default=0.40)
    parser.add_argument("--z-step", type=float, default=0.02)
    args = parser.parse_args()

    ik = build_ik(args)

    z_values = np.arange(args.z_min, args.z_max + 1e-9, args.z_step)

    print("\n抓取/放置 桌面在 base_link 系高度扫描：")
    print(f"  grasp box_xy={GRASP_BOX_XY.tolist()}, "
          f"place box_xy={PLACE_BOX_XY.tolist()}, place_yaw={PLACE_YAW_DEG}°")
    print(f"  收敛阈值: pos<{POS_THRESHOLD*1000:.1f}mm, ori<{ORI_THRESHOLD:.3f}rad, "
          f"max_iters={MAX_ITERS}")
    print("-" * 110)
    print(f"{'table_z_in_base':>15} | "
          f"{'grasp':^8} | {'grasp_err':>18} | "
          f"{'place':^8} | {'place_err':>18}")
    print("-" * 110)

    valid_rows = []
    for z in z_values:
        grasp_center = np.array([GRASP_BOX_XY[0], GRASP_BOX_XY[1],
                                 z + BOX_MODEL["box_height"] / 2.0])
        place_center = np.array([PLACE_BOX_XY[0], PLACE_BOX_XY[1],
                                 z + PLACE_CLEARANCE + BOX_MODEL["box_height"] / 2.0])

        # 抓取：yaw=-90（长边朝向机器人）
        g_targets = compute_wrist_targets(grasp_center, box_rotation_from_yaw(-90.0))
        g_ok, g_pos, g_rot = solve_one(ik, g_targets["left"]["grasp"],
                                       g_targets["right"]["grasp"])

        # 放置：place_yaw_deg=-90
        p_targets = compute_wrist_targets(place_center, box_rotation_from_yaw(PLACE_YAW_DEG))
        p_ok, p_pos, p_rot = solve_one(ik, p_targets["left"]["grasp"],
                                       p_targets["right"]["grasp"])

        both_ok = g_ok and p_ok
        if both_ok:
            valid_rows.append(z)

        print(f"{z:>15.3f} | "
              f"{'OK' if g_ok else 'FAIL':^8} | "
              f"{g_pos*1000:>7.1f}mm/{g_rot:>6.4f} | "
              f"{'OK' if p_ok else 'FAIL':^8} | "
              f"{p_pos*1000:>7.1f}mm/{p_rot:>6.4f}")

    print("-" * 110)
    if valid_rows:
        lo, hi = min(valid_rows), max(valid_rows)
        print(f"\n=== 可解算范围（grasp 与 place 同时收敛） ===")
        print(f"  table_z_in_base ∈ [{lo:.3f}, {hi:.3f}] m")
        print(f"\n  → 建议使用范围（留 2~3 cm 余量）: [{lo + 0.03:.3f}, {hi - 0.03:.3f}] m")
        print(f"\n换算成绝对桌面高度（table_height = base_link_height + table_z_in_base）：")
        for base_h in (0.815, 0.866, 0.90):
            print(f"  base_link 离地 {base_h:.3f}m 时，桌面绝对高度应 ∈ "
                  f"[{base_h + lo:.3f}, {base_h + hi:.3f}] m")
    else:
        print("\n未找到任何同时收敛的高度，请检查 URDF/参数/阈值。")


if __name__ == "__main__":
    main()
