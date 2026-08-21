#!/usr/bin/python3
"""发送 DetectBox action goal 并打印反馈与最终结果。

用法：
    trigger_detect_box.py [action_name]

action_name 默认为 /demo3/box_grasp_demo/detect_box。
"""

import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from box_grasp_demo_msgs.action import DetectBox
from box_grasp_demo_msgs.srv import ComputePlaceIK, SuggestCrouch


MOTION_BASE_URL = os.environ.get("MOTION_BASE_URL", "http://192.168.11.2:12300")
MOTION_COMPONENT_NAMES = ["left_arm", "right_arm"]
MOTION_MODE = int(os.environ.get("MOTION_MODE", "1"))
MOTION_VEL_SCALE = float(os.environ.get("MOTION_VEL_SCALE", "0.2"))
MOTION_API_KEY = os.environ.get("MOTION_PROXY_API_KEY", "")

# biped home 到位后、发送下蹲命令前的固定间隔（秒）。
# move_biped_home 是异步 action：base_link 高度回到站立位只说明腿到位，
# biped 控制器状态机从 home 动作收尾退出还需要时间，立即发下蹲命令
# 可能被忽略（表现为只站起来、不下蹲）。手动 curl 分步发送天然有间隔
# 所以能成功，这里显式补上这个间隔。
BIPED_HOME_SETTLE_SEC = float(os.environ.get("BIPED_HOME_SETTLE_SEC", "3.0"))

# /v1/motions 服务端任务串行执行：上一个任务未结束时新命令会被拒绝，
# 响应 output 里出现 "Task is running"。此时等待后重发。
MOTION_BUSY_RETRY_ATTEMPTS = int(os.environ.get("MOTION_BUSY_RETRY_ATTEMPTS", "5"))
MOTION_BUSY_RETRY_DELAY_SEC = float(os.environ.get("MOTION_BUSY_RETRY_DELAY_SEC", "2.0"))

# 当前 base_link 与目标高度差小于该值时视为已就位，跳过 home+下蹲。
CROUCH_SKIP_TOLERANCE = 0.02

# 下蹲到位后等待 biped 任务收尾的固定间隔（秒）。
# motion 服务端任务串行：TF 高度到位不等于下蹲任务结束，紧接着发手臂
# 命令会被 "Task is running" 拒绝。此间隔让下蹲任务完全结束后再进下一步。
CROUCH_SETTLE_SEC = float(os.environ.get("CROUCH_SETTLE_SEC", "2.0"))

# biped+双臂合并请求的 vel_scale（biped 下蹲 0.1 与手臂 0.2 的折中）。
COMBINED_VEL_SCALE = float(os.environ.get("COMBINED_VEL_SCALE", "0.15"))

# 搬运期间腰部后仰角（rad，waist 组件 goals 第二个参数，正值后仰）。
# 默认 0.32 rad（约 18.3°）；可用同名环境变量覆盖。
WAIST_TILT_BACK_RAD = float(os.environ.get("WAIST_TILT_BACK_RAD", "0.32"))

# ik_data 中的 5 组 14 关节数据顺序与动作名称一一对应。
MOTION_POINT_NAMES = ["预抓取位置", "抓取位置", "抬起位置", "收回位置", "放置位置"]

# 放下箱子后，手臂返回拍照位置时的 14 关节目标（rad）。
MOTION_RETURN_TO_PHOTO_JOINTS = [
    0, -0.785, -1.57, -1.57, 1.57, 0.0, 0,
    0, -0.785, 1.57, -1.57, -1.57, 0.0, 0,
]

# 流程末尾收起双臂的轨迹（从拍照位逐段回到双臂零位），含 head 2 关节。
MOTION_RETRACT_BODY = {
    "component_names": ["left_arm", "right_arm", "head"],
    "goals": [
        [
            0, -0.785, -1.57, -1.57, 1.57, 0.0, 0,
            0, -0.785, 1.57, -1.57, -1.57, 0.0, 0,
            0, -0.65,
        ],
        [
            0, -0.785, -1.57, 0, 1.57, 0.0, 0,
            0, -0.785, 1.57, 0, -1.57, 0.0, 0,
            0, -0.65,
        ],
        [
            0, 0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.0, 0,
            0, 0.0,
        ],
    ],
    "mode": 1,
    "vel_scale": 0.1,
}

# 用于判断机器人当前是否处于“手臂零位”的 14 个手臂关节。
ARM_JOINTS = [
    "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
    "L_elbow_roll_joint", "L_elbow_yaw_joint",
    "L_wrist_pitch_joint", "L_wrist_roll_joint",
    "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
    "R_elbow_roll_joint", "R_elbow_yaw_joint",
    "R_wrist_pitch_joint", "R_wrist_roll_joint",
]

# 手臂零位 -> 拍照位置的轨迹，内容对应 motion_ready.json。
# 顺序为 [left_arm(7), right_arm(7), head(2)]，模式 1，低速。
MOTION_READY_BODY = {
    "component_names": ["left_arm", "right_arm", "head"],
    "goals": [
        [
            0, 0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.0, 0,
            0, -0.65,
        ],
        [
            0, -0.785, -1.57, 0, 1.57, 0.0, 0,
            0, -0.785, 1.57, 0, -1.57, 0.0, 0,
            0, -0.65,
        ],
        [
            0, -0.785, -1.57, -1.57, 1.57, 0.0, 0,
            0, -0.785, 1.57, -1.57, -1.57, 0.0, 0,
            0, -0.65,
        ],
    ],
    "mode": 1,
    "vel_scale": 0.2,
}


class DetectBoxClient(Node):
    def __init__(self, action_name, debug=True):
        super().__init__("detect_box_client")
        self.debug = debug
        # biped home（站立基准）只需激活一次；之后的下蹲调整直接发
        # 绝对下蹲量，不再重复 home。
        self._biped_homed = False
        self._action_client = ActionClient(self, DetectBox, action_name)
        self._goal_handle = None
        self._result = None
        self._status = None
        self._joint_state = None
        self.create_subscription(
            JointState, "/mc/joint_states", self._joint_state_cb, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # suggest_crouch 服务与 action 同名路径下的 ~/suggest_crouch。
        action_base = action_name.rstrip("/").rsplit("/", 1)[0]
        self._suggest_crouch_client = self.create_client(
            SuggestCrouch, f"{action_base}/suggest_crouch")
        # IK 节点与 box_grasp_demo 同处 action_base 的上一级命名空间。
        namespace = action_base.rsplit("/", 1)[0]
        self._compute_place_ik_client = self.create_client(
            ComputePlaceIK, f"{namespace}/arm_ik/compute_place_ik")

    def _confirm(self, prompt, auto_confirm):
        """统一处理人工确认；非调试模式仅自动确认正常执行步骤。"""
        if not self.debug:
            if auto_confirm:
                print("非调试模式：自动执行。")
            else:
                print("非调试模式：异常情况不自动忽略。")
            return auto_confirm
        answer = input(prompt).strip().lower()
        return answer in ("y", "yes")

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("action server 未就绪（等待 5s）")
            return False

        goal_msg = DetectBox.Goal()
        self.get_logger().info("发送检测 goal ...")

        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error("发送 DetectBox goal 超时（10s）")
            return False

        self._goal_handle = send_future.result()
        if self._goal_handle is None:
            self.get_logger().error("goal 被拒绝")
            return False

        if not self._goal_handle.accepted:
            self.get_logger().warn("goal 未被接受")
            return False

        self.get_logger().info("goal 已接受，等待检测结果 ...")
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90.0)

        if not result_future.done():
            self.get_logger().error("等待 DetectBox action 结果超时（90s）")
            self.cancel_active_goal()
            return False

        response = result_future.result()
        if response is None:
            self.get_logger().error("未收到 action result")
            return False

        self._status = response.status
        self._result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"DetectBox action 未成功结束，status={response.status}")
            return False
        return self._result is not None and bool(self._result.success)

    def cancel_active_goal(self):
        """请求取消仍在执行的 DetectBox goal；无活动 goal 时直接成功。"""
        goal = self._goal_handle
        if goal is None:
            return True
        try:
            future = goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if not future.done():
                self.get_logger().error("取消 DetectBox goal 超时")
                return False
            response = future.result()
            return response is not None
        except Exception as exc:
            self.get_logger().error(f"取消 DetectBox goal 失败: {exc}")
            return False

    def _joint_state_cb(self, msg):
        self._joint_state = msg

    def _wait_for_joint_state(self, timeout_sec=3.0):
        """等待一条 /mc/joint_states 消息，返回消息或 None。"""
        self._joint_state = None
        start = time.monotonic()
        while self._joint_state is None:
            if time.monotonic() - start >= timeout_sec:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._joint_state

    @staticmethod
    def _arm_positions_from_joint_state(joint_state):
        """从 JointState 中按 ARM_JOINTS 顺序提取 14 个手臂关节角。"""
        if joint_state is None:
            return None
        idx = {name: i for i, name in enumerate(joint_state.name)}
        if any(name not in idx for name in ARM_JOINTS):
            return None
        return [float(joint_state.position[idx[name]]) for name in ARM_JOINTS]

    def _wait_for_arm_target(self, group, timeout_sec=20.0, tolerance=0.08):
        """等待 /mc/joint_states 中 14 个手臂关节接近目标。

        HTTP /v1/motions 返回 200 只代表运动命令被接收，不代表动作完成；
        必须轮询关节状态，等真实手臂到位后再继续下一步。
        """
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            joint_state = self._wait_for_joint_state(timeout_sec=0.25)
            current = self._arm_positions_from_joint_state(joint_state)
            if current is None:
                continue
            diffs = [abs(float(t) - c) for t, c in zip(group, current)]
            if diffs and max(diffs) <= tolerance:
                return True
        return False

    def _print_current_vs_target(self, group):
        """打印当前手臂关节角，并计算目标与当前的最大关节差。"""
        joint_state = self._wait_for_joint_state(timeout_sec=1.5)
        current = self._arm_positions_from_joint_state(joint_state)
        if current is None:
            print("无法读取当前手臂关节，跳过关节差提示。")
            return

        diffs = [abs(float(t) - c) for t, c in zip(group, current)]
        max_diff = max(diffs) if diffs else 0.0
        print(f"当前关节(rad): {[round(v, 4) for v in current]}")
        print(f"目标与当前最大关节差: {max_diff:.4f} rad")
        if max_diff < 0.01:
            print("警告：目标与当前关节几乎一致，机器人可能不运动（IK 可能未收敛）。")

    def prepare_photo_position(self):
        """确保手臂处于拍照位置，若不在则发送轨迹并等待到位。

        原先发送 HTTP 轨迹后立即返回，导致机器人还没走到拍照位置就开始
        拍照检测。这里改为：读取当前关节，判断是否已在拍照位；若不在，
        发送 motion_ready 轨迹，并轮询关节状态直到到位或超时。
        """
        joint_state = self._wait_for_joint_state()
        if joint_state is None:
            print("无法读取 /mc/joint_states，拍照位置准备失败。")
            return False

        # 拍照位置 = MOTION_READY_BODY 最后一个路点的 14 个手臂关节。
        target = MOTION_READY_BODY["goals"][-1][:14]
        current = self._arm_positions_from_joint_state(joint_state)
        if current is not None:
            diffs = [abs(float(t) - c) for t, c in zip(target, current)]
            if diffs and max(diffs) <= 0.05:
                print("手臂已在拍照位置，跳过“去拍照位置”。")
                return True

        print("\n" + "=" * 60)
        print("手臂不在拍照位置，需要先移动到拍照位置。")
        print(f"轨迹组件 : {MOTION_READY_BODY['component_names']}")
        print(f"轨迹路点 : {len(MOTION_READY_BODY['goals'])} 个")
        print("=" * 60)
        if not self._confirm("是否执行去拍照位置？[y/N] ",
                             auto_confirm=True):
            print("已取消去拍照位置，流程停止。")
            return False

        if not self._http_health():
            print("机器人 HTTP 服务健康检查失败，无法去拍照位置。")
            return False

        status, text = self._http_post_json(MOTION_READY_BODY)
        if status != 200:
            print(f"发送去拍照位置轨迹失败，HTTP {status or '无响应'}：{text}")
            return False

        print(f"已发送去拍照位置轨迹，HTTP {status}")
        if text:
            print(f"响应: {text}")

        print("等待手臂到达拍照位置 ...")
        if self._wait_for_arm_target(target, timeout_sec=40.0, tolerance=0.08):
            print("已到达拍照位置。")
            return True

        print("等待到达拍照位置超时，机器人可能仍在运动中。")
        return False

    def _request_crouch(self, mode):
        """向主程序请求建议下蹲量。

        返回 (crouch, base_link_height, target_base_link_height,
        standing_base_link_height)，失败返回 None。
        各值都来自主程序同一次计算，避免 trigger 自己再读一次 TF 造成不一致。
        """
        if not self._suggest_crouch_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("suggest_crouch service 未就绪（等待 3s）")
            return None
        req = SuggestCrouch.Request()
        req.mode = mode
        future = self._suggest_crouch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error("suggest_crouch 请求超时（10s）")
            return None
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error("suggest_crouch 请求失败")
            return None
        return (float(resp.crouch_height),
                float(resp.base_link_height),
                float(resp.target_base_link_height),
                float(resp.standing_base_link_height))

    def _get_base_link_height(self, timeout_sec=0.2):
        """读取 base_footprint→base_link 的 Z，失败返回 None。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_footprint", "base_link", rclpy.time.Time(),
                timeout=Duration(seconds=timeout_sec))
            return float(tf.transform.translation.z)
        except Exception:
            return None

    def _wait_for_base_height(self, target, tolerance=0.02, timeout_sec=20.0):
        """轮询 TF，直到 base_link 离地高度接近 target。"""
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            current = self._get_base_link_height()
            if current is not None and abs(current - target) <= tolerance:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _http_post_biped_home(self):
        """激活 biped home（下蹲量 0 的站立位）。

        下蹲指令必须先在 home 基础上才能调用：先向 /v1/call_action 发送
        {"action":"s2/move_biped_home"}，让机器人回到站立位（下蹲量 0.0），
        之后才能发送绝对下蹲命令。
        """
        body = {"action": "s2/move_biped_home"}
        return self._http_post_json(body, endpoint="/v1/call_action")

    def _http_post_biped_offset(self, crouch, tag="biped 下蹲"):
        """发送 biped 绝对下蹲量；负值下蹲，0.0 恢复站立高度。

        goals 第三个元素是绝对下蹲量（相对站立位，负值向下），不是相对
        当前高度的增量。该值直接来自主程序 suggest_crouch 服务。
        """
        body = {
            "component_names": ["biped"],
            # 四舍五入到 4 位小数：与手动 curl 的 -0.101 一致，
            # 避免 Python 浮点产生 -0.10100000000000009 之类长尾数。
            "goals": [[0.0, 0.0, round(float(crouch), 4), 0.0, 0.0, 0.0]],
            "mode": 1,
            "vel_scale": 0.1,
        }
        return self._post_motion_with_retry(body, tag)

    def prepare_crouch(self, mode):
        """拍照/抓取/放置前，按主程序建议值调整下蹲高度并等待到位。"""
        requested = self._request_crouch(mode)
        if requested is None:
            print("无法获取下蹲建议，下蹲调整失败。")
            return False

        crouch, base_link_height, target_base, standing_base = requested
        print(f"[{mode}] 建议下蹲量: {crouch:.3f} m "
              f"(当前 base_link={base_link_height:.3f}m, "
              f"目标 base_link={target_base:.3f}m)")
        if abs(crouch) < 0.005:
            print("下蹲量接近 0，无需调整身高。")
            return True

        # 当前 base_link 已在目标高度附近时无需再次调整，
        # 避免同一高度被反复发送绝对下蹲量（如拍照与抓取目标相同）。
        if abs(base_link_height - target_base) <= CROUCH_SKIP_TOLERANCE:
            print(f"当前 base_link={base_link_height:.3f}m 已接近目标 "
                  f"{target_base:.3f}m（±{CROUCH_SKIP_TOLERANCE:g}m），"
                  f"无需再次下蹲。")
            return True

        if not self._confirm("是否执行下蹲调整？[y/N] ",
                             auto_confirm=True):
            print("已取消下蹲，流程停止。")
            return False

        if not self._http_health():
            print("机器人 HTTP 服务健康检查失败，无法下蹲。")
            return False

        # biped home（站立基准）只需在流程开始激活一次；之后的身高调整
        # 都直接发送绝对下蹲量，不再重复 home。
        if not self._ensure_biped_homed(standing_base):
            return False

        status, text = self._http_post_biped_offset(crouch)
        if status != 200:
            print(f"发送下蹲命令失败，HTTP {status or '无响应'}：{text}")
            return False

        if text:
            print(f"下蹲命令响应: {text}")
        print(f"已发送下蹲命令，等待 base_link 高度达到 {target_base:.3f} m ...")
        if self._wait_for_base_height(target_base):
            print("下蹲到位。")
            print(f"等待 {CROUCH_SETTLE_SEC:g}s 让下蹲任务收尾，"
                  f"避免下一步手臂命令被 Task is running 拒绝 ...")
            time.sleep(CROUCH_SETTLE_SEC)
            return True

        # 第一次命令可能被 biped home 收尾状态吞掉（返回 200 但不执行）。
        # 等待超时说明没下蹲，此时 biped 已空闲，重发一次等效于手动 curl。
        print("下蹲未到位，biped 应已空闲，重发一次下蹲命令 ...")
        status, text = self._http_post_biped_offset(crouch)
        if status != 200:
            print(f"重发下蹲命令失败，HTTP {status or '无响应'}：{text}")
            return False

        if text:
            print(f"重发下蹲命令响应: {text}")
        print(f"已重发下蹲命令，等待 base_link 高度达到 {target_base:.3f} m ...")
        if self._wait_for_base_height(target_base):
            print("下蹲到位。")
            print(f"等待 {CROUCH_SETTLE_SEC:g}s 让下蹲任务收尾 ...")
            time.sleep(CROUCH_SETTLE_SEC)
            return True

        print("等待下蹲到位超时，机器人可能仍在运动中。")
        return False

    def prepare_navigation_height(self):
        """发送绝对下蹲量 0.0 回到站立高度，不重复激活 biped home。"""
        requested = self._request_crouch("photo")
        if requested is None:
            print("无法获取站立高度，导航身高准备失败。")
            return False

        standing_base = requested[3]
        print("\n" + "=" * 60)
        print("恢复导航站立高度（biped 绝对下蹲量 0.0）。")
        print(f"目标 base_link 站立高度: {standing_base:.3f} m")
        print("=" * 60)
        if not self._confirm("是否执行导航身高准备？[y/N] ",
                             auto_confirm=True):
            print("已取消导航身高准备。")
            return False

        if not self._http_health():
            print("机器人 HTTP 服务健康检查失败，无法准备导航身高。")
            return False

        status, text = self._http_post_biped_offset(
            0.0, tag="恢复导航高度")
        if status != 200:
            print(f"发送恢复导航高度命令失败，HTTP {status or '无响应'}：{text}")
            return False

        if text:
            print(f"恢复导航高度命令响应: {text}")
        print(f"已发送绝对下蹲量 0.0，等待 base_link 回到站立位 "
              f"{standing_base:.3f} m ...")
        if self._wait_for_base_height(standing_base):
            print("已到达导航站立高度。")
            print(f"等待 {CROUCH_SETTLE_SEC:g}s 让 biped 任务收尾 ...")
            time.sleep(CROUCH_SETTLE_SEC)
            return True

        print("等待回到导航站立高度超时，机器人可能仍在运动中。")
        return False

    def compute_current_place_ik(self):
        """请求 IK 节点基于当前 place 目标和当前 TF 重算放置关节角。"""
        client = self._compute_place_ik_client
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("compute_place_ik service 未就绪（等待 5s）")
            return None
        future = client.call_async(ComputePlaceIK.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            self.get_logger().error("compute_place_ik 请求超时（30s）")
            return None
        response = future.result()
        if response is None or not response.success:
            message = "无响应" if response is None else response.message
            self.get_logger().error(f"compute_place_ik 失败: {message}")
            return None
        joints = [float(v) for v in response.joint_positions]
        if len(joints) != 14 or any(not math.isfinite(v) for v in joints):
            self.get_logger().error("compute_place_ik 返回的关节数据无效")
            return None
        print(f"已按当前放置高度重新计算 place IK: {response.message}")
        return joints

    def _http_health(self):
        """调用 /healthz，确认机器人 HTTP 服务可用。"""
        req = urllib.request.Request(MOTION_BASE_URL + "/healthz", method="GET")
        if MOTION_API_KEY:
            req.add_header("X-API-Key", MOTION_API_KEY)
        try:
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                return resp.status == 200
        except urllib.error.URLError as exc:
            self.get_logger().warning(f"健康检查失败: {exc}")
            return False

    def _http_post_json(self, body, endpoint="/v1/motions"):
        """向动作代理发送 JSON body，返回 (status, text)。"""
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(
            MOTION_BASE_URL + endpoint, data=data, method="POST")
        req.add_header("Content-Type", "application/json")
        if MOTION_API_KEY:
            req.add_header("X-API-Key", MOTION_API_KEY)

        try:
            with urllib.request.urlopen(req, timeout=30.0) as resp:
                return resp.status, resp.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read().decode("utf-8", "replace")
        except urllib.error.URLError as exc:
            return None, str(exc)

    def _http_post_motion(self, group):
        """向 /v1/motions 发送单个 14 关节目标点。"""
        body = {
            "component_names": MOTION_COMPONENT_NAMES,
            "goals": [group],
            "mode": MOTION_MODE,
            "vel_scale": MOTION_VEL_SCALE,
        }
        return self._post_motion_with_retry(body, "手臂运动")

    def _post_motion_with_retry(self, body, tag):
        """发送 /v1/motions 命令；服务端任务忙（Task is running）时重试。

        motion 服务任务串行执行，例如 biped 下蹲任务尚未结束时，紧接着的
        手臂命令会被拒绝（响应 output 含 "Task is running"）。等待后重发，
        最多 MOTION_BUSY_RETRY_ATTEMPTS 次。
        """
        for attempt in range(1, MOTION_BUSY_RETRY_ATTEMPTS + 1):
            status, text = self._http_post_json(body)
            if status == 200 and text and "Task is running" in text:
                print(f"{tag}: 服务忙（上一任务未结束），第 {attempt}/"
                      f"{MOTION_BUSY_RETRY_ATTEMPTS} 次，等待 "
                      f"{MOTION_BUSY_RETRY_DELAY_SEC:g}s 重试 ...")
                time.sleep(MOTION_BUSY_RETRY_DELAY_SEC)
                continue
            return status, text
        return status, text

    @staticmethod
    def _parse_ik_data(ik_data):
        """解析 action 返回的 ik_data，返回 5 组 14 关节角
        [pregrasp, grasp, aftergrasp, retract, place]。"""
        try:
            groups = json.loads(ik_data)
        except ValueError as exc:
            print(f"ik_data 不是有效 JSON: {exc}")
            return None

        if not isinstance(groups, list) or len(groups) != 5:
            print("ik_data 应包含 5 组关节数据，无法执行动作")
            return None

        for i, group in enumerate(groups):
            if not isinstance(group, list) or len(group) != 14:
                print(f"第 {i + 1} 组关节数据长度应为 14，实际长度异常")
                return None
            if any(not isinstance(v, (int, float)) or not math.isfinite(float(v))
                   for v in group):
                print(f"第 {i + 1} 组关节数据含非有限数或非数值")
                return None
        return groups

    @staticmethod
    def _print_motion_header(description):
        print("\n" + "=" * 60)
        print(f"{description}，准备通过 HTTP 逐步驱动机器人")
        print(f"HTTP 地址 : {MOTION_BASE_URL}")
        print(f"组件顺序 : {MOTION_COMPONENT_NAMES}")
        print(f"mode     : {MOTION_MODE}")
        print(f"vel_scale: {MOTION_VEL_SCALE}")
        print("=" * 60)

    def _run_motion_steps(self, steps):
        """发送手臂动作序列，供抓取和放置序列共用。

        非调试模式：合并为一个 /v1/motions 请求一次发送全部路点，
        服务端按轨迹顺序连续执行，省去逐步等待的往返耗时。
        调试模式：逐步确认并逐点发送。
        """
        if not self._http_health():
            print("机器人 HTTP 服务健康检查失败，已停止，不执行任何动作。")
            return False

        if not self.debug:
            return self._run_motion_steps_batched(steps)
        return self._run_motion_steps_stepwise(steps)

    def _run_motion_steps_batched(self, steps):
        """合并为一个请求发送全部路点，等待最后一个路点到位。"""
        body = {
            "component_names": MOTION_COMPONENT_NAMES,
            "goals": [group for _, group in steps],
            "mode": MOTION_MODE,
            "vel_scale": MOTION_VEL_SCALE,
        }
        print(f"\n合并发送 {len(steps)} 个动作点: "
              f"{' → '.join(name for name, _ in steps)}")
        for i, (name, group) in enumerate(steps):
            print(f"  路点 {i + 1} {name}: {group}")
        self._print_current_vs_target(steps[-1][1])

        status, text = self._post_motion_with_retry(body, "手臂动作序列")
        if status != 200:
            print(f"发送合并动作序列失败，HTTP {status or '无响应'}：{text}")
            return False
        print(f"已发送合并动作序列，HTTP {status}")
        if text:
            print(f"响应: {text}")

        # 服务端顺序执行多段轨迹；等最后一个路点到位，超时按路点数放宽。
        timeout = 40.0 * len(steps)
        print(f"等待最后路点 '{steps[-1][0]}' 到位（最长 {timeout:g}s）...")
        if self._wait_for_arm_target(steps[-1][1], timeout_sec=timeout):
            print("动作序列全部完成。")
            return True
        print("等待动作序列完成超时，机器人可能仍在运动中。")
        print("已停止流程。")
        return False

    def _run_motion_steps_stepwise(self, steps):
        """逐步确认并逐点发送手臂目标（调试模式）。"""
        for i, (name, group) in enumerate(steps):
            print(f"\n第 {i + 1} / {len(steps)} 步：{name}")
            print(f"关节角(rad): {group}")
            self._print_current_vs_target(group)
            if not self._confirm("是否执行这一步？[y/N] ",
                                 auto_confirm=True):
                print("已取消，剩余动作不执行。")
                return False

            status, text = self._http_post_motion(group)
            if status != 200:
                print(f"发送 {name} 失败，HTTP {status or '无响应'}：{text}")
                return False

            print(f"已发送 {name}，HTTP {status}")
            if text:
                print(f"响应: {text}")

            print(f"等待 {name} 到位 ...")
            if self._wait_for_arm_target(group, timeout_sec=40.0):
                print(f"{name} 已完成。")
            else:
                print(f"等待 {name} 完成超时，机器人可能仍在运动中。")
                print("已停止流程。")
                return False

        return True

    def _combined_motion_body(self, biped_offsets, arm_groups):
        """构造 biped+双臂合并请求：每个路点 = biped 6 维 + 双臂 14 维。"""
        return {
            "component_names": ["biped", "left_arm", "right_arm"],
            "goals": [list(biped) + list(arm)
                      for biped, arm in zip(biped_offsets, arm_groups)],
            "mode": 1,
            "vel_scale": COMBINED_VEL_SCALE,
        }

    def _post_waist_pitch(self, pitch_rad, name):
        """发送 waist 组件俯仰命令（goals 第二个参数，正值后仰）。"""
        body = {
            "component_names": ["waist"],
            "goals": [[0.0, float(pitch_rad)]],
            "mode": 1,
            "vel_scale": 0.1,
        }
        status, text = self._post_motion_with_retry(body, name)
        if status != 200:
            print(f"发送{name}失败，HTTP {status or '无响应'}：{text}")
            return False
        print(f"已发送{name}（pitch={pitch_rad:.4f} rad），HTTP {status}")
        print(f"等待 {CROUCH_SETTLE_SEC:g}s 让腰部动作收尾 ...")
        time.sleep(CROUCH_SETTLE_SEC)
        return True

    def tilt_waist_back(self):
        """腰部后仰（默认 0.32 rad≈18.3°），搬运期间让重心后移。"""
        print("\n" + "=" * 60)
        print(f"腰部后仰 {WAIST_TILT_BACK_RAD:.4f} rad（重心平衡）")
        print("=" * 60)
        return self._post_waist_pitch(WAIST_TILT_BACK_RAD, "腰部后仰")

    def tilt_waist_center(self):
        """腰部回正（pitch 0），放置完成后恢复。"""
        print("\n" + "=" * 60)
        print("腰部回正（pitch 0）")
        print("=" * 60)
        return self._post_waist_pitch(0.0, "腰部回正")

    def activate_biped_home(self):
        """流程开始时执行一次 call_action home，并等待站立到位。

        后续下蹲和恢复站立均使用 /v1/motions 的绝对下蹲量，不再调用 home。
        """
        requested = self._request_crouch("photo")
        if requested is None:
            print("无法获取站立高度，home 激活失败。")
            return False
        standing_base = requested[3]
        print("\n流程开始，激活一次 biped home（call_action）...")
        status, text = self._http_post_biped_home()
        if status != 200:
            print(f"激活 biped home 失败，HTTP {status or '无响应'}：{text}")
            return False
        print(f"等待 base_link 回到站立位 {standing_base:.3f} m ...")
        if not self._wait_for_base_height(standing_base):
            print("等待回到站立位超时，机器人可能仍在运动中。")
            return False
        self._biped_homed = True
        print(f"biped home 已到位，等待 {BIPED_HOME_SETTLE_SEC:g}s "
              f"让控制器收尾 ...")
        time.sleep(BIPED_HOME_SETTLE_SEC)
        return True

    def _ensure_biped_homed(self, standing_base):
        """确保 biped home 已激活（未激活则执行 call_action 并等待站立）。"""
        if self._biped_homed:
            return True
        status, text = self._http_post_biped_home()
        if status != 200:
            print(f"激活 biped home 失败，HTTP {status or '无响应'}：{text}")
            return False
        print(f"已激活 biped home，等待 base_link 回到站立位 "
              f"{standing_base:.3f} m ...")
        if not self._wait_for_base_height(standing_base):
            print("等待回到站立位超时，机器人可能仍在运动中。")
            return False
        self._biped_homed = True
        print(f"biped home 已到位，等待 {BIPED_HOME_SETTLE_SEC:g}s "
              f"让控制器收尾 ...")
        time.sleep(BIPED_HOME_SETTLE_SEC)
        return True

    def run_pick_combined(self, ik_data):
        """非调试模式：抓取段 + 完成后站直，合并为一个 /v1/motions 请求。

        拍照前已下蹲，此步骤不再下蹲。路点 = biped 6 维 + 双臂 14 维：
          蹲(保持)+预抓取 → 蹲(保持)+抓取 → 蹲(保持)+抬起 → 蹲(保持)+收回
          → 站直(0)+保持收回
        站直使用 motions 的下蹲量 0（home 是激活站立功能的 call_action，
        仅在流程开始由 activate_biped_home 执行一次）。
        """
        groups = self._parse_ik_data(ik_data)
        if groups is None:
            return False
        requested = self._request_crouch("grasp")
        if requested is None:
            print("无法获取抓取下蹲建议，取消合并抓取序列。")
            return False
        crouch, _, _, standing_base = requested
        biped_crouch = [0.0, 0.0, round(float(crouch), 4), 0.0, 0.0, 0.0]
        biped_stand = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        print("\n" + "=" * 60)
        print("合并发送：抓取段 + 站直收尾（5 路点）")
        print(f"保持下蹲量: {crouch:.4f} m, 最后路点站直(0), "
              f"vel_scale: {COMBINED_VEL_SCALE}")
        print("=" * 60)
        for i, name in enumerate(MOTION_POINT_NAMES[:4]):
            print(f"  路点 {i + 1} {name}: {biped_crouch + groups[i]}")
        print(f"  路点 5 站直+保持收回: {biped_stand + groups[3]}")

        body = self._combined_motion_body(
            [biped_crouch, biped_crouch, biped_crouch, biped_crouch, biped_stand],
            [groups[0], groups[1], groups[2], groups[3], groups[3]])
        status, text = self._post_motion_with_retry(body, "合并抓取段+站直")
        if status != 200:
            print(f"发送合并序列失败，HTTP {status or '无响应'}：{text}")
            return False
        print(f"已发送合并序列，HTTP {status}")
        if text:
            print(f"响应: {text}")

        # 最后路点是站直+保持收回：base_link 回到站立位即序列完成。
        print(f"等待最后路点站直完成（base_link≈{standing_base:.3f}m）...")
        if not self._wait_for_base_height(standing_base):
            print("等待站直到位超时。")
            return False
        print("等待手臂收回位到位（最长 160s）...")
        if not self._wait_for_arm_target(groups[3], timeout_sec=160.0):
            print("等待抓取序列完成超时。")
            return False
        print("合并抓取序列完成（已站直）。")
        print(f"等待 {CROUCH_SETTLE_SEC:g}s 让运动任务收尾 ...")
        time.sleep(CROUCH_SETTLE_SEC)
        return True

    def run_place_combined(self):
        """非调试模式：放置段 + 完成后站直，合并为一个请求。

        前置：必须已执行 prepare_crouch("place") 把 biped 下蹲到位——
        放置 IK 需要在下蹲后的 base_link 高度下重新解算，站立状态下
        桌面在 base_link 下方（table_z 为负），IK 无法收敛。
        路点：蹲(保持)+放置 → 蹲(保持)+返回拍照 → 站直(0)+保持返回拍照。
        """
        place_group = self.compute_current_place_ik()
        if place_group is None:
            return False
        requested = self._request_crouch("place")
        if requested is None:
            print("无法获取放置下蹲建议，取消合并放置序列。")
            return False
        crouch, base_link_height, target_base, standing_base = requested
        biped_crouch = [0.0, 0.0, round(float(crouch), 4), 0.0, 0.0, 0.0]
        biped_stand = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if abs(base_link_height - target_base) > CROUCH_SKIP_TOLERANCE * 2:
            print(f"警告: 当前 base_link={base_link_height:.3f}m 未处于放置"
                  f"下蹲位 {target_base:.3f}m，放置 IK 结果可能不可用。")

        print("\n" + "=" * 60)
        print("合并发送：放置段 + 站直收尾（3 路点）")
        print(f"保持下蹲量: {crouch:.4f} m, 最后路点站直(0), "
              f"vel_scale: {COMBINED_VEL_SCALE}")
        print("=" * 60)
        print(f"  路点 1 放置位置: {biped_crouch + place_group}")
        print(f"  路点 2 返回拍照位置: {biped_crouch + MOTION_RETURN_TO_PHOTO_JOINTS}")
        print(f"  路点 3 站直+保持返回拍照: {biped_stand + MOTION_RETURN_TO_PHOTO_JOINTS}")

        body = self._combined_motion_body(
            [biped_crouch, biped_crouch, biped_stand],
            [place_group, MOTION_RETURN_TO_PHOTO_JOINTS,
             MOTION_RETURN_TO_PHOTO_JOINTS])
        status, text = self._post_motion_with_retry(body, "合并放置段+站直")
        if status != 200:
            print(f"发送合并序列失败，HTTP {status or '无响应'}：{text}")
            return False
        print(f"已发送合并序列，HTTP {status}")
        if text:
            print(f"响应: {text}")

        print("等待手臂返回拍照位置到位（最长 80s）...")
        if not self._wait_for_arm_target(MOTION_RETURN_TO_PHOTO_JOINTS,
                                         timeout_sec=80.0):
            print("等待放置序列完成超时。")
            return False
        print(f"等待最后路点站直完成（base_link≈{standing_base:.3f}m）...")
        if not self._wait_for_base_height(standing_base):
            print("等待站直到位超时。")
            return False
        print("合并放置序列完成（已站直）。")
        print(f"等待 {CROUCH_SETTLE_SEC:g}s 让运动任务收尾 ...")
        time.sleep(CROUCH_SETTLE_SEC)
        return True

    def run_pick_sequence(self, ik_data):
        """执行预抓取、抓取、抬起三组 IK 动作。"""
        groups = self._parse_ik_data(ik_data)
        if groups is None:
            return False

        self._print_motion_header("检测到箱子，开始抓取序列")
        steps = [(MOTION_POINT_NAMES[i], groups[i]) for i in range(4)]
        return self._run_motion_steps(steps)

    def run_place_sequence(self, ik_data):
        """执行放置 IK 动作，再将手臂返回拍照位置。"""
        groups = self._parse_ik_data(ik_data)
        if groups is None:
            return False

        self._print_motion_header("开始放置序列")
        steps = [
            (MOTION_POINT_NAMES[4], groups[4]),
            ("返回拍照位置", MOTION_RETURN_TO_PHOTO_JOINTS),
        ]
        return self._run_motion_steps(steps)

    def run_current_place_sequence(self):
        """重新计算当前 place IK，执行放置并返回拍照位置。"""
        place_group = self.compute_current_place_ik()
        if place_group is None:
            return False
        self._print_motion_header("开始当前高度的放置序列")
        return self._run_motion_steps([
            (MOTION_POINT_NAMES[4], place_group),
            ("返回拍照位置", MOTION_RETURN_TO_PHOTO_JOINTS),
        ])

    def retract_arms(self):
        """流程末尾收起双臂：从拍照位逐段回到双臂零位（含 head）。"""
        print("\n" + "=" * 60)
        print("流程末尾收起双臂")
        print("=" * 60)
        status, text = self._post_motion_with_retry(
            MOTION_RETRACT_BODY, "收起双臂")
        if status != 200:
            print(f"发送收起双臂轨迹失败，HTTP {status or '无响应'}：{text}")
            return False
        print(f"已发送收起双臂轨迹，HTTP {status}")
        if text:
            print(f"响应: {text}")
        # 轨迹最后一个路点 = 双臂零位，取前 14 个手臂关节等待到位。
        target = MOTION_RETRACT_BODY["goals"][-1][:14]
        print("等待双臂收起到位 ...")
        if self._wait_for_arm_target(target):
            print("双臂已收起。")
            return True
        print("等待双臂收起超时，机器人可能仍在运动中。")
        return False

    def run_motion_sequence(self, ik_data):
        """兼容原接口，依次执行完整抓取和放置序列。"""
        if not self.run_pick_sequence(ik_data):
            return False
        if not self.run_place_sequence(ik_data):
            return False

        print("\n全部动作已确认并发送完成。")
        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        center = feedback.box_center
        self.get_logger().info(
            f"[反馈] 稳定 {feedback.stable_count} 次，连续漏检 "
            f"{feedback.miss_count} 次，中心 "
            f"({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})m | "
            f"{feedback.message}")

    def print_result(self):
        if self._result is None:
            print("无结果")
            return

        print("=" * 60)
        print("DetectBox action 结果")
        print("=" * 60)
        print(f"status         : {self._status}")
        print(f"success        : {self._result.success}")
        print(f"message        : {self._result.message}")
        print(f"box_center     : {list(self._result.box_center)}")
        print(f"box_dimensions : {list(self._result.box_dimensions)}")
        print(f"grasp_crouch   : {self._result.grasp_crouch:.3f} m")
        print(f"place_crouch   : {self._result.place_crouch:.3f} m")
        print(f"ik_data        : {self._result.ik_data}")
        print("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    action_name = "/demo3/box_grasp_demo/detect_box"
    if len(sys.argv) > 1:
        action_name = sys.argv[1]

    client = DetectBoxClient(action_name, debug=True)
    try:
        if not client.prepare_photo_position():
            print("未完成拍照位置准备，流程结束。")
            return

        # 拍照/检测前先按主程序建议值调整下蹲高度，保证桌面落在 IK 可达范围。
        if not client.prepare_crouch("photo"):
            print("拍照前下蹲调整未完成，流程结束。")
            return

        if client.send_goal():
            client.print_result()
            result = client._result
            if result is not None and result.success:
                if result.ik_data:
                    client.run_motion_sequence(result.ik_data)
                else:
                    print("检测成功但未收到 IK 关节数据，无法执行抓取动作。")
            else:
                print("未检测到箱子，不执行抓取动作。")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
