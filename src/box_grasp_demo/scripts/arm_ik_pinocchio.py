#!/usr/bin/env python3
"""Walker S2 双臂 IK 节点 — 基于 Pinocchio + CasADi + IPOPT。

替代 arm_ik_ros2.py，使用联合双臂优化代替独立求解。
订阅 wrist 目标位姿，求解 14 个手臂关节并发布 /joint_states。

特性：
- 双臂联合优化，位置 + 方向误差同时最小化
- 热启动迭代，2-3 次即收敛至 <0.1 mm
- 可选 .so 预编译加速（< 1 ms/次）
"""

import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage

from box_grasp_demo_msgs.srv import ComputePlaceIK

# 导入新的 IK 求解器
from box_grasp_demo.arm_ik import WalkerArmIK
from box_grasp_demo.arm_ik.walker_ik import (
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
)


def pose_matrix(msg: PoseStamped) -> np.ndarray:
    """PoseStamped → 4x4 齐次矩阵。"""
    q = msg.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    out = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w), msg.pose.position.x],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w), msg.pose.position.y],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y), msg.pose.position.z],
        [0, 0, 0, 1]], dtype=float)
    return out


class ArmIKPinocchio(Node):
    """基于 Pinocchio + CasADi 的双臂 IK ROS2 节点。"""

    def __init__(self):
        super().__init__("arm_ik")

        # ---- 参数 ----
        self.declare_parameter("urdf_file", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("target_topic", "/sim_box_grasp/box_grasp_demo")
        self.declare_parameter("pose_command_topic", "/sim_box_grasp/pose_command")
        self.declare_parameter("initial_mode", "pregrasp")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("joint_state_topic", "/joint_states")  # 关节状态发布话题
        self.declare_parameter("publish_arm_tf", False)  # 是否发布手臂 TF（离线 RViz 模型移动用）
        self.declare_parameter("arm_tf_topic", "/tf")    # 手臂 TF 发布话题
        self.declare_parameter("so_path", "")          # 预编译 .so 路径（可选）
        self.declare_parameter("jit", False)            # 启用 CasADi JIT
        self.declare_parameter("max_iters", 5)          # 最大迭代次数
        self.declare_parameter("pos_threshold", 1e-4)   # 位置收敛阈值 (m)
        self.declare_parameter("ori_threshold", 1e-4)   # 方向收敛阈值 (rad)
        self.declare_parameter("pregrasp_q", [
            0.0, -math.pi/4, -math.pi/2, -math.pi/2,  math.pi/2, 0.0, 0.0,   # L
            0.0, -math.pi/4,  math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0,   # R
        ])  # 默认抓取姿态（14 个手臂关节角，rad）

        urdf_path = str(self.get_parameter("urdf_file").value)
        if not urdf_path:
            raise RuntimeError("arm_ik requires urdf_file")

        so_path = str(self.get_parameter("so_path").value) or None
        jit = bool(self.get_parameter("jit").value)

        # ---- 解析 package_dirs（用于 URDF 中 package:// 路径） ----
        package_dirs = [os.path.dirname(urdf_path)]  # URDF 所在目录
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("walker_s2_description")
            package_dirs.append(os.path.dirname(pkg_share))  # share 的父目录
            package_dirs.append(pkg_share)                    # share 目录本身
        except Exception:
            pass
        # 也加入源码目录
        src_candidate = os.path.join(
            os.path.dirname(urdf_path), "..", "..", "..")
        if os.path.isdir(src_candidate):
            package_dirs.append(os.path.abspath(src_candidate))

        # ---- 构建 IK 求解器 ----
        self.get_logger().info(f"加载 URDF: {urdf_path}")
        self.get_logger().info(f"JIT: {jit}, SO: {so_path}")

        self.ik = WalkerArmIK(
            urdf_path=urdf_path,
            package_dirs=package_dirs,
            jit=jit,
            so_path=so_path,
        )
        self.get_logger().info(
            f"IK 求解器就绪，简化模型 {self.ik.reduced_robot.model.nq} 个关节"
        )

        # ---- 腰部后仰补偿 ----
        # 简化模型把 waist 锁定在 0 位；真机腰部后仰时，把 IK 目标从真机
        # 世界变换到模型世界（等效按真机腰角求解）。这里缓存 q=0 时
        # torso_link 相对 base_link 的位姿（模型直立基准）。
        # 补偿基准缺失时 IK 解在真机上会整体偏移 ~10cm 导致抓空，
        # 不静默降级：报致命错误并终止程序。
        self._torso0 = None
        try:
            frame_id = self.ik.reduced_robot.model.getFrameId("torso_link")
            placement = self.ik.reduced_robot.framePlacement(
                np.zeros(self.ik.reduced_robot.model.nq), frame_id)
            self._torso0 = np.eye(4)
            self._torso0[:3, :3] = placement.rotation
            self._torso0[:3, 3] = placement.translation
        except Exception as exc:
            self.get_logger().fatal(
                f"无法获取 torso_link 直立基准位姿，后仰补偿不可用，终止程序: {exc}")
            raise SystemExit(1) from exc

        # 真机 waist 关节读数，用于交叉验证 TF 中 torso 是否真实反映后仰。
        self._waist_pitch = None
        self.create_subscription(
            JointState, "/mc/joint_states", self._mc_joint_state_cb, 10)

        # ---- 关节名称映射 ----
        self._arm_joint_names = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

        # 非手臂关节（固定为零）
        urdf_text = open(urdf_path, "r", encoding="utf-8").read()
        urdf_root = ET.fromstring(urdf_text)
        arm_set = set(self._arm_joint_names)
        self._non_arm_names = [
            j.get("name") for j in urdf_root.findall("joint")
            if j.get("type") != "fixed" and j.get("name") not in arm_set
        ]

        # 手臂运动链的 parent/child frame，用于离线 RViz 直接发布 TF，
        # 让模型手臂跟随 IK 解算姿态（不经过 robot_state_publisher）。
        self._arm_tf_pairs = []
        for j in urdf_root.findall("joint"):
            if j.get("name") in arm_set and j.get("type") != "fixed":
                self._arm_tf_pairs.append((
                    j.find("parent").get("link"),
                    j.find("child").get("link"),
                ))

        self.get_logger().info(
            f"手臂关节: {len(self._arm_joint_names)}, "
            f"非手臂关节: {len(self._non_arm_names)}, "
            f"手臂 TF 链: {len(self._arm_tf_pairs)}"
        )

        # 默认抓取姿态（双臂向前伸，用于抓取开始时作为 IK 初始种子）。
        # 关节顺序 [shoulder_pitch, shoulder_roll, shoulder_yaw,
        #           elbow_roll, elbow_yaw, wrist_pitch, wrist_roll] x L/R。
        # 从 YAML 参数 pregrasp_q 读取，便于按真机当前姿态调整（如肩 roll ≈ -45°）。
        pregrasp_q = [float(v) for v in self.get_parameter("pregrasp_q").value]
        if len(pregrasp_q) != 14:
            raise RuntimeError(
                f"pregrasp_q 必须为 14 个值（L 臂 7 + R 臂 7），实际 {len(pregrasp_q)}")
        self._pregrasp_q = np.array(pregrasp_q)
        # 零位：双手自然下垂（与 RViz 默认关节全零一致）。
        self._zero_q = np.zeros(14)

        # ---- 状态 ----
        self._targets = {"L": {"pregrasp": None, "grasp": None, "aftergrasp": None,
                               "retract": None, "place": None},
                         "R": {"pregrasp": None, "grasp": None, "aftergrasp": None,
                               "retract": None, "place": None}}
        self._mode = str(self.get_parameter("initial_mode").value)
        # 缓存最近一次 place 解；任一 place 目标更新时立即失效。
        self._place_solution = None

        # 零位为双臂自然下垂；抓取/放置类模式（pregrasp/grasp/aftergrasp/
        # retract/place）从默认抓取姿态出发，让 IK 从更接近目标的构型开始求解。
        if self._mode == "zero":
            self._q = self._zero_q.copy()
        else:
            self._q = self._pregrasp_q.copy()

        # ---- ROS 接口 ----
        topic = str(self.get_parameter("target_topic").value).rstrip("/")
        self.create_subscription(
            PoseStamped, topic + "/left_wrist_pregrasp_pose",
            lambda m: self._set_target("L", "pregrasp", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/right_wrist_pregrasp_pose",
            lambda m: self._set_target("R", "pregrasp", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/left_wrist_grasp_pose",
            lambda m: self._set_target("L", "grasp", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/right_wrist_grasp_pose",
            lambda m: self._set_target("R", "grasp", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/left_wrist_aftergrasp_pose",
            lambda m: self._set_target("L", "aftergrasp", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/right_wrist_aftergrasp_pose",
            lambda m: self._set_target("R", "aftergrasp", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/left_wrist_retract_pose",
            lambda m: self._set_target("L", "retract", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/right_wrist_retract_pose",
            lambda m: self._set_target("R", "retract", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/left_wrist_place_pose",
            lambda m: self._set_target("L", "place", m), 1)
        self.create_subscription(
            PoseStamped, topic + "/right_wrist_place_pose",
            lambda m: self._set_target("R", "place", m), 1)
        self.create_subscription(
            String, str(self.get_parameter("pose_command_topic").value),
            self._set_mode, 1)
        # 检测节点在箱子稳定后发布 Empty，触发打印 5 组 14 关节 IK 数据
        # （pregrasp / grasp / aftergrasp / retract / place）。
        self.create_subscription(
            Empty, topic + "/ik_print", self._print_all_ik, 1)
        self._compute_place_ik_srv = self.create_service(
            ComputePlaceIK, "~/compute_place_ik", self._compute_place_ik)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_state_topic").value), 10)
        self._ik_result_pub = self.create_publisher(String, topic + "/ik_result", 10)

        # 离线 RViz 模型移动：直接把手臂 TF 发布到 /tf，绕过
        # robot_state_publisher 的 /joint_states 订阅。
        publish_arm_tf = str(self.get_parameter("publish_arm_tf").value).lower()
        self._enable_arm_tf = publish_arm_tf in ("true", "1", "on", "yes")
        if self._enable_arm_tf:
            self._arm_tf_pub = self.create_publisher(
                TFMessage, str(self.get_parameter("arm_tf_topic").value), 10)
            self.get_logger().info(
                f"手臂 TF 发布已启用: "
                f"{str(self.get_parameter('arm_tf_topic').value)}")

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self._timer = self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(f"初始模式: {self._mode}")

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _set_target(self, side: str, mode: str, msg: PoseStamped) -> None:
        self._targets[side][mode] = msg
        if mode == "place":
            self._place_solution = None

    def _mc_joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == "waist_pitch_joint":
                self._waist_pitch = float(pos)

    def _compute_place_ik(
            self,
            request: ComputePlaceIK.Request,
            response: ComputePlaceIK.Response) -> ComputePlaceIK.Response:
        """根据当前左右 place 目标重新求解 14 关节 IK。"""
        del request
        # 调用服务即代表要求重新计算；即使本次失败也不保留旧解。
        self._place_solution = None
        missing = [
            side for side in ("L", "R")
            if self._targets[side].get("place") is None
        ]
        if missing:
            response.success = False
            response.message = (
                "缺少 place 手腕目标: " + ", ".join(missing))
            response.joint_positions = []
            return response

        try:
            # 服务必须基于当前 place 目标重新计算，不能复用抓取/打印缓存。
            sol_q = self._solve_mode("place", q_init=self._pregrasp_q)
            if sol_q is None:
                response.success = False
                response.message = "当前 place 目标 IK 求解失败或未收敛"
                response.joint_positions = []
                return response

            joint_positions = np.asarray(sol_q, dtype=float).reshape(-1)
            if joint_positions.size != 14:
                response.success = False
                response.message = (
                    "place IK 关节数量错误: "
                    f"期望 14，实际 {joint_positions.size}")
                response.joint_positions = []
                return response
            if not np.all(np.isfinite(joint_positions)):
                response.success = False
                response.message = "place IK 结果包含非有限值"
                response.joint_positions = []
                return response

            self._place_solution = joint_positions.copy()
            response.success = True
            response.message = "place IK 重新计算成功"
            response.joint_positions = joint_positions.tolist()
        except Exception as exc:
            self.get_logger().error(f"place IK 服务求解异常: {exc}")
            response.success = False
            response.message = f"place IK 求解异常: {exc}"
            response.joint_positions = []
        return response

    def _set_mode(self, msg: String) -> None:
        if msg.data in ("zero", "pregrasp", "grasp", "aftergrasp",
                        "retract", "place"):
            self._mode = msg.data
            # 抓取开始时先回到默认抓取姿态，再以此为种子进行 IK；
            # 切回零位则直接落到双臂自然下垂构型。
            if msg.data == "zero":
                self._q = self._zero_q.copy()
            else:
                self._q = self._pregrasp_q.copy()
            self.get_logger().info(f"切换模式: {self._mode}")

    def _to_base(self, msg: PoseStamped) -> np.ndarray:
        """将 PoseStamped 变换到 base_link 坐标系，返回 4x4 矩阵。"""
        target = pose_matrix(msg)
        base_frame = str(self.get_parameter("base_frame").value)
        if msg.header.frame_id != base_frame:
            try:
                tf = self._tf_buffer.lookup_transform(
                    base_frame, msg.header.frame_id,
                    rclpy.time.Time(), timeout=Duration(seconds=0.1))
                t = tf.transform.translation
                q = tf.transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                rot = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
                tfm = np.eye(4)
                tfm[:3, :3] = rot
                tfm[:3, 3] = [t.x, t.y, t.z]
                target = tfm @ target
            except Exception as exc:
                self.get_logger().warning(
                    f"TF 查询失败 {base_frame}←{msg.header.frame_id}: {exc}",
                    throttle_duration_sec=2.0)
        return target

    def _update(self) -> None:
        """定时器回调：求解 IK 并发布关节状态。

        IK 失败时保持当前位置不移动，并打印详细失败原因。
        """
        if self._mode == "zero":
            self._q = self._zero_q.copy()
        else:
            target_l = self._targets["L"].get(self._mode)
            target_r = self._targets["R"].get(self._mode)
            if target_l is None or target_r is None:
                self._publish_joint_state()
                return

            try:
                T_l = self._to_base(target_l)
                T_r = self._to_base(target_r)
                # 腰部后仰补偿（与 _solve_mode 一致）。
                adj = self._torso_adjustment()
                T_l = adj @ T_l
                T_r = adj @ T_r

                max_iters = int(self.get_parameter("max_iters").value)
                pos_th = float(self.get_parameter("pos_threshold").value)
                ori_th = float(self.get_parameter("ori_threshold").value)

                # 使用上次解作为热启动种子
                sol_q = self.ik.solve_ik(
                    T_l, T_r,
                    q_init=self._q,
                    max_iters=max_iters,
                    pos_threshold=pos_th,
                    ori_threshold=ori_th,
                    verbose=False,
                )

                # NaN 保护
                if np.any(np.isnan(sol_q)):
                    self.get_logger().warning("IK 结果含 NaN，尝试 pregrasp 种子…")
                    sol_q = self.ik.solve_ik(
                        T_l, T_r,
                        q_init=self._pregrasp_q,
                        max_iters=max_iters + 3,
                        pos_threshold=pos_th,
                        ori_threshold=ori_th,
                        verbose=False,
                    )

                # ---- FK 验证：计算左右手各自的位置/方向误差 ----
                placement_L = self.ik.reduced_robot.framePlacement(
                    sol_q, self.ik.L_hand_id)
                placement_R = self.ik.reduced_robot.framePlacement(
                    sol_q, self.ik.R_hand_id)

                dist_L = float(np.linalg.norm(
                    placement_L.translation - T_l[:3, 3]))
                dist_R = float(np.linalg.norm(
                    placement_R.translation - T_r[:3, 3]))
                rot_err_L = float(np.linalg.norm(
                    pin.log3(placement_L.rotation.T @ T_l[:3, :3])))
                rot_err_R = float(np.linalg.norm(
                    pin.log3(placement_R.rotation.T @ T_r[:3, :3])))

                # ---- 判断是否达标 ----
                pos_fail_L = dist_L >= pos_th
                pos_fail_R = dist_R >= pos_th
                rot_fail_L = rot_err_L >= ori_th
                rot_fail_R = rot_err_R >= ori_th
                any_fail = pos_fail_L or pos_fail_R or rot_fail_L or rot_fail_R

                if any_fail:
                    # 构建失败原因
                    parts = []
                    if pos_fail_L:
                        parts.append(f"左手位置({dist_L*1000:.1f}mm)")
                    if pos_fail_R:
                        parts.append(f"右手位置({dist_R*1000:.1f}mm)")
                    if rot_fail_L:
                        parts.append(f"左手方向({rot_err_L:.3f}rad)")
                    if rot_fail_R:
                        parts.append(f"右手方向({rot_err_R:.3f}rad)")

                    self.get_logger().error(
                        f"IK 不可到达 — {'; '.join(parts)} | "
                        f"目标(pos_th={pos_th*1000:.0f}mm, ori_th={ori_th:.0e}rad) | "
                        f"保持当前位置"
                    )
                    # 不更新 self._q → 机器人不移动
                else:
                    # IK 成功，更新关节状态
                    self._q = sol_q
                    self.get_logger().info(
                        f"IK ✓ | dist L:{dist_L*1000:.2f}mm R:{dist_R*1000:.2f}mm | "
                        f"rot L:{rot_err_L:.4f} R:{rot_err_R:.4f}",
                        throttle_duration_sec=5.0)

            except Exception as exc:
                self.get_logger().error(
                    f"IK 求解异常: {exc} | 保持当前位置")
                # 不更新 self._q → 机器人不移动

        self._publish_joint_state()

    def _fk_error(self, sol_q, T_l, T_r):
        """计算左右手腕 FK 与目标之间的位置/方向误差。

        返回 (max_pos_error_m, max_rot_error_rad)。sol_q 为空或含 NaN 时
        返回 (inf, inf)，便于调用方直接判断未收敛。
        """
        if sol_q is None or np.any(np.isnan(sol_q)):
            return float("inf"), float("inf")

        placement_L = self.ik.reduced_robot.framePlacement(
            sol_q, self.ik.L_hand_id)
        placement_R = self.ik.reduced_robot.framePlacement(
            sol_q, self.ik.R_hand_id)

        dist_L = float(np.linalg.norm(placement_L.translation - T_l[:3, 3]))
        dist_R = float(np.linalg.norm(placement_R.translation - T_r[:3, 3]))
        rot_err_L = float(np.linalg.norm(
            pin.log3(placement_L.rotation.T @ T_l[:3, :3])))
        rot_err_R = float(np.linalg.norm(
            pin.log3(placement_R.rotation.T @ T_r[:3, :3])))
        return max(dist_L, dist_R), max(rot_err_L, rot_err_R)

    def _torso_adjustment(self) -> np.ndarray:
        """返回把真机当前 torso 位姿变换到模型直立 torso 位姿的 4x4。

        模型把 waist 锁定在 0 位求解；真机腰部后仰 θ 时，torso 随之后仰，
        IK 目标需先经此变换再送入模型，求出的手臂关节才能在真机命中原目标。
        等价变换：T_adj = T_torso(模型直立) @ inv(T_torso(真机当前))。
        补偿失效时（TF 缺失/超时/未反映真实后仰）IK 解在真机上会整体
        偏移 ~10cm 导致抓空，因此不静默回退：报致命错误并终止程序。
        """
        if self._torso0 is None:
            self.get_logger().fatal(
                "torso 直立基准未初始化，后仰补偿不可用，终止程序")
            raise SystemExit(1)
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link", "torso_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception as exc:
            self.get_logger().fatal(
                f"TF 查询失败 base_link←torso_link: {exc}；"
                "后仰补偿不可用，终止程序")
            raise SystemExit(1) from exc
        t = tf.transform.translation
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        torso_real = np.eye(4)
        torso_real[:3, :3] = rot
        torso_real[:3, 3] = [t.x, t.y, t.z]
        adj = self._torso0 @ np.linalg.inv(torso_real)
        # 用 waist_pitch_joint 读数交叉验证 TF。真机存在命令角与关节
        # 读数的固有差异（如命令 18.3° 时读数 14.1°），且腰部运动/
        # 回正过程中 TF（若为命令值）与读数间存在瞬态差异，因此数量
        # 差异只告警不终止。仅当两者方向相反且都显著非零（TF 反映的
        # 后仰方向根本错误）时才视为补偿失效、终止程序。
        if self._waist_pitch is None:
            self.get_logger().warning(
                "未收到 /mc/joint_states 的 waist_pitch_joint 读数，"
                "跳过 TF 交叉验证（后仰补偿仍按 TF 生效）",
                throttle_duration_sec=10.0)
            return adj
        tf_pitch = float(pin.log3(adj[:3, :3])[1])
        if (abs(tf_pitch) > 0.05 and abs(self._waist_pitch) > 0.05
                and tf_pitch * self._waist_pitch < 0.0):
            self.get_logger().fatal(
                f"后仰补偿失效：TF 中 torso 后仰 {math.degrees(tf_pitch):.1f}° "
                f"与 waist_pitch_joint 读数 "
                f"{math.degrees(self._waist_pitch):.1f}° 方向相反，终止程序")
            raise SystemExit(1)
        return adj

    def _solve_mode(self, mode: str, q_init: np.ndarray | None = None):
        """对 pregrasp/grasp/aftergrasp/retract/place 其中一组求解 14 关节 IK。

        求解后做 FK 误差校验，未收敛时换 pregrasp 种子重试一次；仍失败则
        返回 None，调用方不应再把该解写入 ik_data。
        """
        target_l = self._targets["L"].get(mode)
        target_r = self._targets["R"].get(mode)
        if target_l is None or target_r is None:
            self.get_logger().warning(f"缺少 {mode} 手腕目标")
            return None
        T_l = self._to_base(target_l)
        T_r = self._to_base(target_r)
        # 腰部后仰补偿：把目标变换到模型世界（waist=0）再求解。
        adj = self._torso_adjustment()
        self.get_logger().info(
            f"{mode} IK 后仰补偿生效：TF 测得 torso 后仰 "
            f"{math.degrees(float(pin.log3(adj[:3, :3])[1])):.1f}°")
        T_l = adj @ T_l
        T_r = adj @ T_r

        max_iters = int(self.get_parameter("max_iters").value)
        pos_th = float(self.get_parameter("pos_threshold").value)
        ori_th = float(self.get_parameter("ori_threshold").value)

        if q_init is None:
            q_init = self._pregrasp_q

        sol_q = self.ik.solve_ik(
            T_l, T_r,
            q_init=q_init,
            max_iters=max_iters,
            pos_threshold=pos_th,
            ori_threshold=ori_th,
            verbose=False,
        )
        pos_err, rot_err = self._fk_error(sol_q, T_l, T_r)
        if pos_err < pos_th and rot_err < ori_th:
            return sol_q

        # 第一次可能掉进错误局部最优或未收敛，换 pregrasp 种子再试一次。
        self.get_logger().warning(
            f"{mode} IK 未收敛 (pos={pos_err*1000:.1f}mm, "
            f"rot={rot_err:.4f}rad)，改用 pregrasp 种子重试")
        sol_q = self.ik.solve_ik(
            T_l, T_r,
            q_init=self._pregrasp_q,
            max_iters=max_iters + 3,
            pos_threshold=pos_th,
            ori_threshold=ori_th,
            verbose=False,
        )
        pos_err, rot_err = self._fk_error(sol_q, T_l, T_r)
        if pos_err < pos_th and rot_err < ori_th:
            return sol_q

        self.get_logger().error(
            f"{mode} IK 求解失败: pos={pos_err*1000:.1f}mm, "
            f"rot={rot_err:.4f}rad (阈值 pos={pos_th*1000:.1f}mm, "
            f"rot={ori_th:.4f}rad)")
        return None

    def _print_all_ik(self, msg: Empty) -> None:
        """打印 5 组 14 关节 IK 数据，顺序 [pregrasp, grasp, aftergrasp, retract, place]。

        每组为 [left_arm_7, right_arm_7]，共 14 个关节角（rad）。
        5 组按 pregrasp→grasp→aftergrasp→retract→place 顺序热启动串联；
        place 解仅在当前左右目标未更新时缓存复用。
        """
        groups = []
        q_seed = self._pregrasp_q
        for mode in ("pregrasp", "grasp", "aftergrasp", "retract", "place"):
            if mode == "place" and self._place_solution is not None:
                sol_q = self._place_solution
            else:
                sol_q = self._solve_mode(mode, q_init=q_seed)
                if sol_q is None:
                    err = String()
                    err.data = json.dumps({"error": f"{mode} IK failed"})
                    self._ik_result_pub.publish(err)
                    self.get_logger().error(
                        f"{mode} IK 求解失败，中止 ik_data 输出")
                    return
                if mode == "place":
                    self._place_solution = sol_q
                else:
                    q_seed = sol_q
            groups.append([round(float(v), 6) for v in sol_q])
        self.get_logger().info("14 关节 IK 数据: " + repr(groups))
        result_msg = String()
        result_msg.data = json.dumps(groups)
        self._ik_result_pub.publish(result_msg)

    def _publish_joint_state(self) -> None:
        """发布完整关节状态（非手臂关节置零）。"""
        # q 顺序：[L_arm (7), R_arm (7)]
        positions = (
            [0.0] * len(self._non_arm_names)
            + self._q.tolist()
        )
        names = self._non_arm_names + self._arm_joint_names

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self._pub.publish(msg)

        if self._enable_arm_tf:
            self._publish_arm_tf(self._q)

    def _publish_arm_tf(self, q: np.ndarray) -> None:
        """发布 14 个手臂关节的 parent→child 变换到 /tf。

        q 是简化模型的 14 维手臂配置。这里直接使用 Pinocchio 简化模型
        做 FK，然后发布手臂链的相对变换，避免与录制帧里的非手臂动态 TF
        冲突。
        """
        model = self.ik.reduced_robot.model
        data = self.ik.reduced_robot.data
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        tfm = TFMessage()
        stamp = self.get_clock().now().to_msg()
        for parent, child in self._arm_tf_pairs:
            try:
                parent_id = model.getFrameId(parent)
                child_id = model.getFrameId(child)
            except Exception:
                continue
            rel = data.oMf[parent_id].inverse() * data.oMf[child_id]
            quat = pin.Quaternion(rel.rotation)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = float(rel.translation[0])
            t.transform.translation.y = float(rel.translation[1])
            t.transform.translation.z = float(rel.translation[2])
            t.transform.rotation.x = float(quat.x)
            t.transform.rotation.y = float(quat.y)
            t.transform.rotation.z = float(quat.z)
            t.transform.rotation.w = float(quat.w)
            tfm.transforms.append(t)

        self._arm_tf_pub.publish(tfm)


def main(args=None):
    rclpy.init(args=args)
    node = ArmIKPinocchio()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
