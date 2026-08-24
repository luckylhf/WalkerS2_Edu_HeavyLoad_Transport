#!/usr/bin/python3
"""ROS2 Humble adapter for the Walker S2 box grasp demo."""

import json
import math
import os
import time

import numpy as np
import rclpy
import tf2_ros
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray

from box_grasp_demo_msgs.action import DetectBox
from box_grasp_demo_msgs.srv import SuggestCrouch
from box_grasp_demo.detector_open3d import BoxDetection, BoxDetector, DetectorConfig


# ---------------------------------------------------------------------------
# IK 可达性 / 硬件能力常量（由 URDF 与 biped 硬件决定，不属于可配置参数）。
# 数值来源：box_grasp_demo.arm_ik.test_ik_reachability 扫描结果。
# ---------------------------------------------------------------------------
# 桌面相对 base_link 的 Z 在 [0.07, 0.47] m 时 pregrasp/grasp/aftergrasp/place
# 都能稳定收敛。实测 0.85m 桌下蹲 5cm（table_z_in_base≈0.035）即可解算，
# 期望值取 0.035，对应 0.85m 桌下蹲约 5cm。
# 注意：0.035 低于扫描建议下界 0.07，实际抓取 IK 需真机确认可解算。
DESIRED_GRASP_TABLE_Z = 0.035
DESIRED_PLACE_TABLE_Z = 0.035
# biped 最大下蹲量（m），下蹲为负值。
MAX_CROUCH = 0.2
# 桌面绝对高度（离地）合法性范围，超出则拒绝执行。
# IK 可达的是“箱子中心在 base_link 系的高度 c ∈ [0.13, 0.53] m”
# （由 test_ik_reachability 对 0.12m 箱扫描换算：c = 桌面z + box_height/2）。
#   c = (H - base_link_height) + box_height/2
#   桌子不能太矮：最大下蹲 base_link=0.666、未来最大箱高 h_box_max 时
#       箱子中心仍 ≥ 下界 → H_min = 0.666 + 0.13 - h_box_max/2。
#   桌子不能太高：站立 base_link=0.866、未来最小箱高 h_box_min 时
#       箱子中心仍 ≤ 上界 → H_max = 0.866 + 0.53 - h_box_min/2。
# 实际场景桌面区间 [0.75, 1.10] m：
#   H_min 取 0.75（0.75 桌面 + 0.30 箱最大下蹲时 c≈0.234，仍安全）；
#   H_max 取 1.10（站立时 0.30 箱 c≈0.384，远低于上界，留真机余量）。
TABLE_HEIGHT_MIN = 0.75
TABLE_HEIGHT_MAX = 1.10
# 收回（retract）参数已移入 config/box_grasp_ros2.yaml（body_surface_x /
# retract_body_clearance / retract_rz_deg），此处不再保留代码常量。
#
# 历史语义（供理解参数）：
#   body_surface_x —— 躯干前表面在 base_link 系的 X（URDF waist_pitch_link
#     碰撞盒 0.2844/2 = 0.1422，取 0.142；实际去0.12，原因见box_grasp_ros2.yaml说明）。
#     收回目标 = 箱子近身面 X ≈ body_surface_x + retract_body_clearance，
#     再按箱体尺寸反推中心目标 X。
#     实测默认配 10.0：目标中心 X 远大于箱子当前 X，min() 钳位取当前值，
#     即收回不产生平移、仅摆正姿态（详见 cloud_callback 中 retract 计算）。
#   retract_rz_deg —— 收回后箱体绕 base_link Z 轴的转角（deg），-90 表示
#     箱体长轴转到左右（Y）方向横抱在胸前。



class _RclpySleep:
    """可被 rclpy Task 驱动的非阻塞 sleep。

    rclpy Humble 的默认单线程 executor 不会运行 asyncio 事件循环，因此
    ``await asyncio.sleep`` 会报 ``no running event loop``。这里实现一个
    原生 ``__await__``，让协程在时间到达前持续 ``yield`` 把执行权交还给
    executor，从而保证点云订阅、定时器和 IK 结果订阅能继续运行。
    """

    def __init__(self, duration):
        self._end = time.monotonic() + duration

    def __await__(self):
        while time.monotonic() < self._end:
            yield
        return None


def transform_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
        [0, 0, 0, 1],
    ], dtype=np.float64)


def rpy_matrix(rpy):
    """Return the ROS/URDF fixed-axis XYZ RPY rotation matrix."""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def load_extrinsic(path, direction):
    with open(os.path.expanduser(path), "r") as stream:
        if path.lower().endswith(".json"):
            data = json.load(stream)
        else:
            import yaml
            data = yaml.safe_load(stream)
    transform = data.get("transform", data)
    if "RT" in data and not {"translation", "rotation"}.issubset(transform):
        raise ValueError("stereo_params.json RT is left/right stereo calibration, not robot hand-eye")
    if "translation" not in transform or "rotation" not in transform:
        raise ValueError("extrinsic must contain translation and rotation")
    t, q = transform["translation"], transform["rotation"]
    x, y, z, w = float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])
    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), float(t["x"])],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), float(t["y"])],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), float(t["z"])],
        [0, 0, 0, 1],
    ], dtype=np.float64)
    if direction in ("camera_from_parent", "camera_from_target"):
        matrix = np.linalg.inv(matrix)
    elif direction not in ("parent_from_camera", "target_from_camera"):
        raise ValueError("invalid extrinsic direction")
    return matrix


def matrix_to_quaternion(matrix):
    m = matrix
    trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w], dtype=float)


def make_pose(frame, stamp, position, rotation):
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [float(v) for v in position]
    q = matrix_to_quaternion(rotation)
    msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
    msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
    return msg


class BoxGraspNode(Node):
    def __init__(self):
        # YAML 里的 box_models.* 是嵌套参数，节点没有逐条 declare，需要
        # 让 rclpy 在启动时自动声明 parameter overrides，否则
        # get_parameters_by_prefix 无法枚举到 small_box 这类型号。
        super().__init__("box_grasp_demo",
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        def declare(name, value):
            # 参数文件里的 override 已在 super().__init__ 中自动声明，
            # 这里若已存在则直接返回，避免重复声明抛异常。
            if self.has_parameter(name):
                return self.get_parameter(name).value
            return Node.declare_parameter(self, name, value).value

        declare("input_topic", "/sensor/camera/stereo/pointcloud/raw")
        declare("target_frame", "base_link")
        declare("camera_parent_frame", "head_pitch_link")
        declare("camera_extrinsic_file", "")
        declare("camera_extrinsic_direction", "parent_from_camera")
        for name, value in (("plane_distance", 0.012),
                            ("upright_box", True), ("max_box_tilt_deg", 0.0),
                            ("top_outlier_margin", 0.025),
                            ("measure_actual_height", False),
                            ("aftergrasp_lift_height", 0.05),
                            ("support_height_min", 0.80), ("support_height_max", 1.10),
                            ("min_plane_inliers", 40), ("processing_period", 0.25)):
            declare(name, value)
        declare("box_model", "base_box")
        declare("stable_count_threshold", 10)
        declare("stable_position_tolerance", 0.01)
        declare("ground_frame", "base_footprint")
        declare("table_height", 0.88)
        declare("ik_print_topic", "~/ik_print")
        declare("ik_result_topic", "~/ik_result")
        declare("max_consecutive_misses", 3)
        declare("action_timeout", 5.0)
        declare("ik_result_timeout", 3.0)
        # 固定变换：wrist_roll_link → tool.stl 工作原点 [50, 0, 20] mm。
        # 这里的平移是 T_wrist_grasp 的平移，即抓取原点在 wrist 坐标系中的位置。
        # 当前启动使用 walker_s2_description/urdf/s2_v1/s2_v1.urdf。
        declare("left_wrist_to_grasp_translation_xyz",
                               [-0.063830042, 0.093897597, 0.018317873])
        declare("left_wrist_to_grasp_rpy",
                               [-1.560270605, -0.001184104, 1.565962477])
        declare("right_wrist_to_grasp_translation_xyz",
                               [-0.028666097, 0.196032729, -0.024593477])
        declare("right_wrist_to_grasp_rpy",
                               [1.580291657, 0.004695696, 1.575647901])
        declare("min_component_points", 30)
        declare("workspace_min", [-0.10, -0.60, -0.20])
        declare("workspace_max", [1.00, 0.60, 0.80])
        declare("pointcloud_offset", [0.0, 0.0, 0.0])
        declare("placement_roi_enabled", False)
        declare("placement_roi_min", [0.0, 0.0, 0.0])
        declare("placement_roi_max", [0.0, 0.0, 0.0])
        declare("base_link_height", 0.8722)
        declare("place_table_height", 0.78)
        declare("place_x", 0.40)
        declare("place_y", 0.0)
        declare("place_yaw_deg", 0.0)
        declare("place_clearance", 0.005)
        # 收回（retract）目标参数，yaml 中可覆盖。
        declare("body_surface_x", 10.0)
        declare("retract_body_clearance", 0.0)
        declare("retract_rz_deg", -90.0)
        # GUI 模式切换命令话题（相对话题，随节点 namespace 解析）。
        declare("pose_command_topic", "pose_command")

        get = lambda n: self.get_parameter(n).value
        self.target_frame = get("target_frame")
        self.ground_frame = get("ground_frame")
        self.table_height = float(get("table_height"))
        # 放置桌与抓取桌高度相互独立，允许两台高度不同。
        self.place_table_height = float(get("place_table_height"))
        # 最近一次视觉检测实测的桌面高度（base_link 系），仅供日志记录。
        self.last_detected_table_z = None
        # support_height_min/max 在此作为相对桌面中心高度的偏移量。
        self.support_height_min_offset = float(get("support_height_min"))
        self.support_height_max_offset = float(get("support_height_max"))
        # 箱子型号：box_model 指定型号名，box_models 里 base_box 为默认，
        # 其他型号只覆盖差异字段，未覆盖字段自动继承 base_box。
        self.box_model_name = get("box_model")
        self.box_model_params = self._resolve_box_model()
        self.get_logger().info(
            f"使用箱子型号 '{self.box_model_name}'："
            f"length={self.box_model_params['box_length']:.3f}m, "
            f"width={self.box_model_params['box_width']:.3f}m, "
            f"height={self.box_model_params['box_height']:.3f}m")
        self.camera_parent_frame = get("camera_parent_frame")
        # The S2 publishes a complete camera chain in /tf_static:
        # head_pitch_link -> stereo_left_rgb_link -> ... ->
        # stereo_left_rectified_optical_frame.  This file is only a fallback
        # for deployments where that chain is genuinely unavailable.
        self.extrinsic = None
        if get("camera_extrinsic_file"):
            self.extrinsic = load_extrinsic(get("camera_extrinsic_file"), get("camera_extrinsic_direction"))
            self.get_logger().info(
                f"camera extrinsic loaded as fallback for {self.camera_parent_frame}")
        self.workspace_min = np.asarray(get("workspace_min"), dtype=float)
        self.workspace_max = np.asarray(get("workspace_max"), dtype=float)
        self.pointcloud_offset = np.asarray(get("pointcloud_offset"), dtype=float)
        # wrist_roll_link → tool.stl 工作原点 [50, 0, 20] mm（合并后的一级变换）
        self.wrist_to_grasp = {}
        for side in ("left", "right"):
            t = np.asarray(get(f"{side}_wrist_to_grasp_translation_xyz"), dtype=float)
            rpy = np.asarray(get(f"{side}_wrist_to_grasp_rpy"), dtype=float)
            if t.shape != (3,) or rpy.shape != (3,):
                raise ValueError(f"invalid {side} wrist_to_grasp transform")
            r = rpy_matrix(rpy)
            if not np.allclose(r.T @ r, np.eye(3), atol=2e-5):
                raise ValueError(f"{side} wrist_to_grasp rotation is not orthonormal")
            self.wrist_to_grasp[side] = np.eye(4)
            self.wrist_to_grasp[side][:3, :3] = r
            self.wrist_to_grasp[side][:3, 3] = t
        roi_min, roi_max = get("placement_roi_min"), get("placement_roi_max")
        enabled = bool(get("placement_roi_enabled"))
        self.placement_enabled = enabled
        self.roi_min = np.asarray(roi_min, dtype=float) if enabled else None
        self.roi_max = np.asarray(roi_max, dtype=float) if enabled else None
        self.detector = BoxDetector(DetectorConfig(
            box_length=self.box_model_params["box_length"],
            box_width=self.box_model_params["box_width"],
            box_height=self.box_model_params["box_height"],
            dimension_tolerance=self.box_model_params["dimension_tolerance"],
            plane_distance=get("plane_distance"),
            min_component_points=get("min_component_points"),
            min_plane_inliers=get("min_plane_inliers"),
            max_object_height=self.box_model_params["max_object_height"],
            side_clearance=self.box_model_params["side_clearance"],
            upright_box=get("upright_box"), max_box_tilt_deg=get("max_box_tilt_deg"),
            top_outlier_margin=get("top_outlier_margin"),
            measure_actual_height=get("measure_actual_height"),
            pregrasp_distance=self.box_model_params["pregrasp_distance"],
            tool_contact_below_top=self.box_model_params["tool_contact_below_top"],
            grasp_long_edge=bool(self.box_model_params["grasp_long_edge"]),
            support_height_min=get("support_height_min"), support_height_max=get("support_height_max")),
            logger=self.get_logger())
        qos = 1
        self.box_pub = self.create_publisher(PoseStamped, "~/box_pose", qos)
        self.left_pub = self.create_publisher(PoseStamped, "~/left_grasp_pose", qos)
        self.right_pub = self.create_publisher(PoseStamped, "~/right_grasp_pose", qos)
        self.left_pre_pub = self.create_publisher(PoseStamped, "~/left_pregrasp_pose", qos)
        self.right_pre_pub = self.create_publisher(PoseStamped, "~/right_pregrasp_pose", qos)
        self.left_wrist_pub = self.create_publisher(PoseStamped, "~/left_wrist_grasp_pose", qos)
        self.right_wrist_pub = self.create_publisher(PoseStamped, "~/right_wrist_grasp_pose", qos)
        self.left_wrist_pre_pub = self.create_publisher(PoseStamped, "~/left_wrist_pregrasp_pose", qos)
        self.right_wrist_pre_pub = self.create_publisher(PoseStamped, "~/right_wrist_pregrasp_pose", qos)
        self.left_after_pub = self.create_publisher(PoseStamped, "~/left_aftergrasp_pose", qos)
        self.right_after_pub = self.create_publisher(PoseStamped, "~/right_aftergrasp_pose", qos)
        self.left_wrist_after_pub = self.create_publisher(PoseStamped, "~/left_wrist_aftergrasp_pose", qos)
        self.right_wrist_after_pub = self.create_publisher(PoseStamped, "~/right_wrist_aftergrasp_pose", qos)
        self.left_wrist_retract_pub = self.create_publisher(PoseStamped, "~/left_wrist_retract_pose", qos)
        self.right_wrist_retract_pub = self.create_publisher(PoseStamped, "~/right_wrist_retract_pose", qos)
        self.placement_pub = self.create_publisher(PoseStamped, "~/placement_pose", qos)
        self.place_box_pub = self.create_publisher(PoseStamped, "~/place_box_pose", qos)
        self.left_wrist_place_pub = self.create_publisher(PoseStamped, "~/left_wrist_place_pose", qos)
        self.right_wrist_place_pub = self.create_publisher(PoseStamped, "~/right_wrist_place_pose", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", qos)
        # tool 坐标轴 marker 独立定时发布：挂 wrist_roll_link 上随 TF
        # 运动，切换 IK 模式时夹爪可视化实时跟随手腕。
        self._tool_marker_timer = self.create_timer(0.1, self._publish_tool_markers)
        self.status_pub = self.create_publisher(String, "~/status", qos)
        self.ik_print_pub = self.create_publisher(Empty, get("ik_print_topic"), qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_process = 0.0

        # 相机点云 QoS：BEST_EFFORT 以匹配相机发布者
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub = self.create_subscription(PointCloud2, get("input_topic"), self.cloud_callback, cloud_qos)
        self.get_logger().info(f"listening to {get('input_topic')}; output frame: {self.target_frame}")

        # 默认不检测，等待 action goal 触发。
        self.checking = False
        self.stable_count = 0
        self.last_box_center = None
        self.miss_count = 0
        self.stable_count_threshold = int(get("stable_count_threshold"))
        self.stable_position_tolerance = float(get("stable_position_tolerance"))
        self.max_consecutive_misses = int(get("max_consecutive_misses"))

        # IK 节点求解完成后回传 4 组 14 关节数据，供 action result 使用。
        self.ik_result_sub = self.create_subscription(
            String, get("ik_result_topic"), self.on_ik_result, qos)
        self.latest_ik_data = None

        # GUI 模式切换：订阅 pose_command，非零位模式时按 yaml 箱子关系
        # 把绿色 box marker 移到对应模式位姿（检测/收回/放置）。
        self._current_mode = None
        self._detected_box_pose = None
        self._retract_box_pose = None
        self._place_box_pose = None
        self.create_subscription(
            String, get("pose_command_topic"), self._on_pose_command, qos)

        # 放置位姿由配置直接计算，与点云检测无关；定时发布保证 IK 节点
        # 切到 place 模式时始终能收到左右手腕目标。
        self.place_timer = self.create_timer(1.0, self.publish_place_targets)

        # 拍照前/抓取/放置阶段的下蹲高度建议服务。
        self.suggest_crouch_srv = self.create_service(
            SuggestCrouch, "~/suggest_crouch", self._suggest_crouch_callback)

        # Action 状态（由 async execute callback 驱动，不阻塞点云/定时器回调）
        self._active_goal = None
        self._goal_start_time = None
        self._has_detected_once = False
        self._final_box_info = None
        self._final_time = None
        self._action_timeout = float(get("action_timeout"))
        self._ik_result_timeout = float(get("ik_result_timeout"))

        self._action_server = ActionServer(
            self,
            DetectBox,
            "~/detect_box",
            execute_callback=self._execute_detect,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
        )
        self.get_logger().info(
            f"默认不检测；发送 action goal 到 '~/detect_box' 开始检查，"
            f"连续 {self.stable_count_threshold} 次稳定后返回结果；"
            f"{self._action_timeout:.0f}s 内未检测到箱子则中止")

    @staticmethod
    def _tool_rotation(inward):
        """从抓取 inward 向量构造 tool 坐标系旋转（与抓取循环保持一致）。"""
        tool_y = np.array([0.0, 0.0, 1.0])
        tool_z = inward - tool_y * np.dot(tool_y, inward)
        tool_z_norm = np.linalg.norm(tool_z)
        tool_z = tool_z / tool_z_norm if tool_z_norm > 1e-9 else np.array([0.0, 1.0, 0.0])
        tool_x = np.cross(tool_y, tool_z)
        tool_x_norm = np.linalg.norm(tool_x)
        tool_x = tool_x / tool_x_norm if tool_x_norm > 1e-9 else np.array([1.0, 0.0, 0.0])
        return np.column_stack((tool_x, tool_y, tool_z))

    # box_models 未在 YAML 提供时的兜底默认值，与 base_box 保持一致。
    _DEFAULT_BOX_MODEL = {
        "box_length": 0.40,
        "box_width": 0.30,
        "box_height": 0.14,
        "dimension_tolerance": 0.08,
        "max_object_height": 0.50,
        "tool_contact_below_top": 0.06,
        "side_clearance": -0.02,
        "pregrasp_distance": 0.08,
        "grasp_long_edge": False,
    }

    def _get_box_models(self) -> dict:
        """读取 box_models 前缀下的参数并还原成 {型号: {字段: 值}}。

        rclpy 不支持直接 declare_parameter 字典值，因此 YAML 里写
        box_models.base_box.box_length 这类扁平 key，这里用
        get_parameters_by_prefix 收集后重组为嵌套 dict。
        """
        models = {}
        for name, param in self.get_parameters_by_prefix("box_models").items():
            # 兼容不同 rclpy 版本：key 可能是去掉前缀后的
            # "base_box.box_length"，也可能保留 "box_models.base_box.box_length"。
            if name.startswith("box_models."):
                name = name[len("box_models."):]
            parts = name.split(".")
            if len(parts) < 2:
                continue
            model_name, field = parts[0], ".".join(parts[1:])
            models.setdefault(model_name, {})[field] = param.value
        return models

    def _resolve_box_model(self) -> dict:
        """返回当前 box_model 的完整参数：base_box 打底，型号覆盖差异字段。"""
        models = self._get_box_models()
        base = models.get("base_box", {})
        selected = models.get(self.box_model_name, {})
        merged = dict(self._DEFAULT_BOX_MODEL)
        merged.update(base)
        merged.update(selected)
        return merged

    def _get_base_link_height(self) -> float:
        """动态读取 base_link 离地高度，失败时回退到配置参数。

        真机下蹲/站立会改变 base_footprint→base_link 的 translation.z，
        因此不能使用固定的 base_link_height。TF 查询失败时回退到配置值。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.ground_frame, self.target_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.15))
            return float(tf.transform.translation.z)
        except tf2_ros.TransformException:
            fallback = float(self.get_parameter("base_link_height").value)
            self.get_logger().warning(
                f"TF 查询失败 {self.ground_frame}←{self.target_frame}，"
                f"回退 base_link_height={fallback:.3f}",
                throttle_duration_sec=5.0)
            return fallback

    def _crouch_suggestion(self, mode: str):
        """计算建议下蹲量（绝对值，相对站立位，负值向下）。

        biped /v1/motions 的 goals 第三个元素是绝对下蹲量，即相对站立位
        base_link_height 的位移，而不是相对当前高度的增量。因此：
            target_base_link_height = table_height - desired_table_z
            crouch = target_base_link_height - standing_base_link_height
        并 clamp 到硬件能力 [-MAX_CROUCH, 0]。

        返回 (crouch, current_base_link_height, target_base_link_height)。
        current_base_link_height 仅用于日志/诊断，不参与 crouch 计算。
        """
        if mode == "place":
            table_height = self.place_table_height
            desired_table_z = DESIRED_PLACE_TABLE_Z
        else:
            table_height = self.table_height
            desired_table_z = DESIRED_GRASP_TABLE_Z

        standing_base = float(self.get_parameter("base_link_height").value)
        current_base = self._get_base_link_height()
        target_base = table_height - desired_table_z
        crouch = target_base - standing_base
        # 下蹲量是负值；clamp 到硬件能力 [-MAX_CROUCH, 0]，过矮桌子只能
        # 下蹲到极限，过矮到连极限都够不到会在 _validate_table_heights 拦下。
        crouch = float(min(max(crouch, -MAX_CROUCH), 0.0))
        return crouch, current_base, target_base

    def _validate_table_heights(self):
        """校验抓取桌/放置桌绝对高度是否在可解算范围内。

        太高或太矮时返回 (False, 原因)，调用方应中止 action、不执行后续
        检测/抓取/放置。合法范围来自 arm_ik/test_ik_reachability.py 的
        可达性扫描。
        """
        problems = []
        for label, height in (("抓取桌 table_height", self.table_height),
                              ("放置桌 place_table_height", self.place_table_height)):
            if not (TABLE_HEIGHT_MIN <= height <= TABLE_HEIGHT_MAX):
                problems.append(
                    f"{label}={height:.3f}m 超出允许范围 "
                    f"[{TABLE_HEIGHT_MIN:.3f}, {TABLE_HEIGHT_MAX:.3f}]m")
        if problems:
            return False, "；".join(problems)
        return True, (
            f"抓取桌={self.table_height:.3f}m, 放置桌={self.place_table_height:.3f}m "
            f"均在允许范围 [{TABLE_HEIGHT_MIN:.3f}, {TABLE_HEIGHT_MAX:.3f}]m")

    def _suggest_crouch_callback(self, request, response):
        crouch, base_link_height, target_base_link_height = self._crouch_suggestion(
            request.mode)
        standing_base_link_height = float(
            self.get_parameter("base_link_height").value)
        response.success = True
        response.crouch_height = float(crouch)
        response.base_link_height = float(base_link_height)
        response.target_base_link_height = float(target_base_link_height)
        response.standing_base_link_height = float(standing_base_link_height)
        response.message = (
            f"mode={request.mode}, base_link_height={base_link_height:.3f}m, "
            f"target_base_link_height={target_base_link_height:.3f}m, "
            f"crouch={crouch:.3f}m")
        self.get_logger().info(f"建议下蹲: {response.message}")
        return response

    def _update_support_height(self):
        """从 ground_frame→base_link 的 TF 动态计算桌面在 base_link 系的 Z。

        下蹲/站立会改变 base_link 离地高度，写死的 support_height_min/max
        会因此失效。这里读取 base_footprint→base_link 的 translation.z 作为
        base_link 离地高度，再用 table_height（桌面离地绝对高度）
        反推出桌面在 base_link 系的高度范围。
        """
        base_link_height = self._get_base_link_height()
        table_z = (self.table_height - base_link_height
                   + float(self.pointcloud_offset[2]))
        self.detector.config.support_height_min = table_z + self.support_height_min_offset
        self.detector.config.support_height_max = table_z + self.support_height_max_offset
        self.get_logger().info(
            f"桌面高度: base_link离地={base_link_height:.3f}m, "
            f"桌面在base_link系Z={table_z:.3f}m, "
            f"support范围=[{self.detector.config.support_height_min:.3f}, "
            f"{self.detector.config.support_height_max:.3f}]",
            throttle_duration_sec=2.0)

    def publish_place_targets(self):
        """按配置计算并发布 IK place 模式的箱体位姿与左右手腕目标。

        place_table_height 为放置桌离地高度；base_link 离地高度由 TF 动态
        读取，因此转换到 base_link 系时需减去 base_link_height。箱体中心
        X/Y 和 yaw 由 place_x / place_y / place_yaw_deg 指定，抓取几何沿用
        当前 box_model 里的 side_clearance / tool_contact_below_top 等参数。
        """
        try:
            box_length = float(self.box_model_params["box_length"])
            box_width = float(self.box_model_params["box_width"])
            box_height = float(self.box_model_params["box_height"])
            place_clearance = float(self.get_parameter("place_clearance").value)
            place_x = float(self.get_parameter("place_x").value)
            place_y = float(self.get_parameter("place_y").value)
            yaw = math.radians(float(self.get_parameter("place_yaw_deg").value))

            # 放置桌与抓取桌高度独立：使用 place_table_height 推算
            # 放置桌面在 base_link 系的 Z，再加上离桌安全间隙。
            base_link_height = self._get_base_link_height()
            place_z = (self.place_table_height - base_link_height
                       + place_clearance)

            center = np.array([place_x, place_y, place_z + box_height / 2.0])
            yaw_rot = rpy_matrix([0.0, 0.0, yaw])
            rotation = np.column_stack(
                (yaw_rot[:, 0], yaw_rot[:, 1], np.array([0.0, 0.0, 1.0])))
            support_plane = np.array([0.0, 0.0, 1.0, -place_z])
            detection = BoxDetection(
                center=center,
                rotation=rotation,
                dimensions=np.array([box_length, box_width, box_height], dtype=float),
                support_plane=support_plane,
                score=0.0,
                point_count=0,
            )
            grasps = self.detector.grasp_centers(detection)
            stamp = self.get_clock().now().to_msg()
            self.place_box_pub.publish(make_pose(self.target_frame, stamp, center, rotation))
            # 保存放置位姿（place 模式下箱子的位置，yaml 箱体尺寸）。
            self._place_box_pose = (
                center.copy(),
                rotation.copy(),
                np.array([box_length, box_width, box_height], dtype=float),
            )

            for side in ("left", "right"):
                grasp, inward, _ = grasps[side]
                tool_rot = self._tool_rotation(inward)
                grasp_tf = np.eye(4)
                grasp_tf[:3, :3] = tool_rot
                grasp_tf[:3, 3] = grasp
                wrist_tf = grasp_tf @ np.linalg.inv(self.wrist_to_grasp[side])
                pub = (self.left_wrist_place_pub if side == "left"
                       else self.right_wrist_place_pub)
                pub.publish(make_pose(self.target_frame, stamp,
                                      wrist_tf[:3, 3], wrist_tf[:3, :3]))
        except Exception as exc:
            self.get_logger().warning(
                f"publish_place_targets: {exc}", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Action 服务：检测触发、过程反馈、结果/超时
    # ------------------------------------------------------------------
    def _handle_goal(self, goal_request):
        if self._active_goal is not None and self._active_goal.is_active:
            self.get_logger().warning("已有检测 goal 在执行，拒绝新 goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        self.get_logger().info("收到取消请求，停止检测")
        self.checking = False
        self._final_box_info = None
        return CancelResponse.ACCEPT

    def _publish_action_feedback(self, stable_count, miss_count, center, message):
        goal = self._active_goal
        if goal is None or not goal.is_active:
            return
        fb = DetectBox.Feedback()
        fb.stable_count = int(stable_count)
        fb.miss_count = int(miss_count)
        fb.box_center = [0.0, 0.0, 0.0] if center is None else [float(v) for v in center]
        fb.message = message
        goal.publish_feedback(fb)

    def on_ik_result(self, msg):
        self.latest_ik_data = msg.data
        self.get_logger().info("收到 IK 结果: " + msg.data)

    async def _execute_detect(self, goal_handle):
        """Action 执行体：等待检测稳定后返回结果，或超时中止。

        使用 _RclpySleep 主动让出执行权，使点云订阅、place 定时器、IK
        结果订阅等回调能继续在同一个单线程 executor 内运行。
        """
        self._active_goal = goal_handle
        self._goal_start_time = time.monotonic()
        self._has_detected_once = False
        self._final_box_info = None
        self._final_time = None
        self.latest_ik_data = None

        # 桌面太高或太矮时直接拒绝执行，避免盲目下蹲/抓取/放置。
        valid, reason = self._validate_table_heights()
        if not valid:
            self.get_logger().error(f"桌面高度校验失败，拒绝执行: {reason}")
            result = DetectBox.Result()
            result.success = False
            result.message = f"桌面高度超出允许范围，不执行: {reason}"
            goal_handle.abort()
            self._active_goal = None
            self.checking = False
            return result
        self.get_logger().info(f"桌面高度校验通过: {reason}")

        self.checking = True
        self.stable_count = 0
        self.last_box_center = None
        self.miss_count = 0
        self.get_logger().info("action goal 已接受，开始检测箱子")
        self._publish_action_feedback(0, 0, None, "开始检测箱子")

        while True:
            if not goal_handle.is_active:
                self._active_goal = None
                self.checking = False
                return DetectBox.Result()

            now = time.monotonic()

            # 已稳定，等待 IK 结果后成功返回
            if self._final_box_info is not None:
                if self.latest_ik_data is not None:
                    return self._build_success_result(goal_handle)
                if now - self._final_time >= self._ik_result_timeout:
                    self.get_logger().warning(
                        "等待 IK 结果超时，返回检测结果（不含 IK 关节数据）")
                    return self._build_success_result(goal_handle)
                await _RclpySleep(0.05)
                continue

            # 尚未检测到箱子，超过 action_timeout 则中止
            if not self._has_detected_once:
                elapsed = now - self._goal_start_time
                if elapsed >= self._action_timeout:
                    self.get_logger().warning(
                        f"{self._action_timeout:.0f}s 内未检测到箱子，中止 goal")
                    result = DetectBox.Result()
                    result.success = False
                    result.message = f"{self._action_timeout:.0f}s 内未检测到箱子"
                    goal_handle.abort()
                    self._active_goal = None
                    self.checking = False
                    return result

            await _RclpySleep(0.05)

    def _build_success_result(self, goal_handle):
        """组装成功结果并结束 goal。"""
        info = self._final_box_info
        box = info["detection"]
        result = DetectBox.Result()
        result.success = True
        result.box_center = [float(v) for v in box.center]
        result.box_dimensions = [float(v) for v in box.dimensions]
        result.ik_data = self.latest_ik_data or ""
        # 抓取/放置阶段建议下蹲量（绝对值，相对站立位），供 trigger 在后续
        # 动作阶段按需调整身高。
        result.grasp_crouch, _, _ = self._crouch_suggestion("grasp")
        result.place_crouch, _, _ = self._crouch_suggestion("place")
        result.message = (
            f"检测完成：箱子中心 ({box.center[0]:.3f}, {box.center[1]:.3f}, "
            f"{box.center[2]:.3f})m，尺寸 ({box.dimensions[0]:.3f}, "
            f"{box.dimensions[1]:.3f}, {box.dimensions[2]:.3f})m"
        )
        if not result.ik_data:
            result.message += "；未收到 IK 关节数据"
        goal_handle.succeed()
        self._active_goal = None
        self._final_box_info = None
        self.checking = False
        return result

    def cloud_callback(self, msg):
        if not self.checking:
            return
        now = time.monotonic()
        if now - self.last_process < float(self.get_parameter("processing_period").value):
            return
        self.last_process = now
        try:
            raw_points = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            # ROS 2 Humble returns a structured NumPy array here.  Older
            # sensor_msgs_py versions returned an iterable of tuples, so keep
            # the fallback for both message-reader behaviours.
            raw_dtype = getattr(raw_points, "dtype", None)
            if getattr(raw_dtype, "names", None):
                points = np.column_stack((raw_points["x"], raw_points["y"],
                                           raw_points["z"])).astype(float)
            else:
                points = np.asarray(list(raw_points), dtype=float)
            if len(points) == 0:
                self.publish_status("no_points")
                return
            if msg.header.frame_id != self.target_frame:
                # Normal S2 path.  Always honor PointCloud2.header.frame_id:
                # the cloud is in stereo_left_rectified_optical_frame, not in
                # the hand-eye file's generic "camera" frame.  TF2 combines
                # the device-published static camera mount and dynamic head
                # pose at this exact cloud stamp.
                try:
                    cloud_time = rclpy.time.Time.from_msg(msg.header.stamp)
                    tf = self.tf_buffer.lookup_transform(
                        self.target_frame, msg.header.frame_id, cloud_time,
                        timeout=Duration(seconds=0.15))
                    points = (transform_matrix(tf) @
                              np.c_[points, np.ones(len(points))].T).T[:, :3]
                except tf2_ros.TransformException as direct_tf_error:
                    # Explicit fallback for systems that do not publish a
                    # point-cloud-frame TF chain.  Do not use this path on
                    # the S2 while the device /tf_static chain is present.
                    if self.extrinsic is None:
                        raise direct_tf_error
                    self.get_logger().warning(
                        "point-cloud TF unavailable; falling back to camera_extrinsic_file: "
                        f"{direct_tf_error}", throttle_duration_sec=5.0)
                    points = (self.extrinsic @
                              np.c_[points, np.ones(len(points))].T).T[:, :3]
                    tf = self.tf_buffer.lookup_transform(
                        self.target_frame, self.camera_parent_frame,
                        rclpy.time.Time(), timeout=Duration(seconds=0.15))
                    points = (transform_matrix(tf) @
                              np.c_[points, np.ones(len(points))].T).T[:, :3]
            points_pre_ws = len(points)
            points = points[np.all((points >= self.workspace_min) & (points <= self.workspace_max), axis=1)]
            if np.any(self.pointcloud_offset != 0.0):
                # 工作空间过滤后再整体平移，避免把箱子/桌面点移出 workspace 导致漏检。
                points = points + self.pointcloud_offset
            self._update_support_height()
            detection = self.detector.detect(points)
            if detection is None:
                # 容忍偶发漏检：只有连续漏检达到阈值才清零稳定计数，
                # 否则保持当前计数，等下一帧继续。
                self.miss_count += 1
                if self.miss_count >= self.max_consecutive_misses:
                    self.stable_count = 0
                    self.last_box_center = None
                self.publish_status("box_not_found")
                self._publish_action_feedback(
                    self.stable_count, self.miss_count, None,
                    f"未检测到箱子 | 漏检 {self.miss_count}/{self.max_consecutive_misses}")
                self.get_logger().warning(
                    f"未检测到箱子 | 漏检 {self.miss_count}/{self.max_consecutive_misses} | "
                    f"过滤前:{points_pre_ws}点 → WS[{self.workspace_min}~{self.workspace_max}] → {len(points)}点 "
                    f"X[{points[:,0].min():.2f},{points[:,0].max():.2f}] "
                    f"Y[{points[:,1].min():.2f},{points[:,1].max():.2f}] "
                    f"Z[{points[:,2].min():.2f},{points[:,2].max():.2f}]"
                    if len(points) > 0 else
                    f"未检测到箱子 | 漏检 {self.miss_count}/{self.max_consecutive_misses} | "
                    f"过滤前:{points_pre_ws}点 → WS后:0点 (WS太窄!)",
                    throttle_duration_sec=5.0)
                return
            stamp = msg.header.stamp

            # ---- 检测成功日志 ----
            self.miss_count = 0
            c = detection.center
            d = detection.dimensions
            # 记录视觉实测桌面高度，供放置阶段复用，保证抓取/放置同一基准。
            self.last_detected_table_z = float(c[2] - d[2] / 2.0)
            self.get_logger().info(
                f"✓ 箱子: 中心({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})m  "
                f"尺寸({d[0]:.3f}, {d[1]:.3f}, {d[2]:.3f})m  "
                f"点数={detection.point_count}  "
                f"桌面Z={self.last_detected_table_z:.3f}m")
            self.box_pub.publish(make_pose(self.target_frame, stamp, detection.center, detection.rotation))
            grasps = self.detector.grasp_centers(detection)
            # 保存检测位姿（pregrasp/grasp/aftergrasp 模式下箱子的位置）。
            self._detected_box_pose = (
                np.asarray(detection.center, dtype=float),
                np.asarray(detection.rotation, dtype=float),
                np.asarray(detection.dimensions, dtype=float),
            )

            # 连续稳定计数：只有连续检测到箱子且中心位置未移动才累加。
            center = np.asarray(detection.center, dtype=float)
            if self.last_box_center is None:
                movement = 0.0
                self.stable_count = 1
            else:
                movement = float(np.linalg.norm(center - self.last_box_center))
                if movement <= self.stable_position_tolerance:
                    self.stable_count += 1
                else:
                    self.stable_count = 1
            self.last_box_center = center
            self._has_detected_once = True
            self.get_logger().info(
                f"  稳定计数: {self.stable_count}/{self.stable_count_threshold} "
                f"(本次移动 {movement:.4f} m)")
            self._publish_action_feedback(
                self.stable_count, self.miss_count, center,
                f"检测到箱子，稳定计数 {self.stable_count}/{self.stable_count_threshold}，"
                f"中心 ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})m")

            frame_debug = {}
            aftergrasp_lift = float(self.get_parameter("aftergrasp_lift_height").value)

            # ---- 收回（retract）目标：箱子摆正并靠近身体中线 ----
            # 收回后箱体绕 Z 轴旋转 retract_rz_deg（默认 -90°：长轴转到
            # 左右方向横抱），rx=ry=0。近身面到中心的距离按旋转后朝向
            # 身体（-X）方向的半宽计算：rz=0 时为长边 box_length/2，
            # rz=±90° 时为短边 box_width/2。若箱子已经比目标更靠近身体，
            # min() 钳位不再朝外推出，仍会摆正姿态并把中心 Y 归零。
            body_surface_x = float(self.get_parameter("body_surface_x").value)
            retract_body_clearance = float(
                self.get_parameter("retract_body_clearance").value)
            retract_rz = math.radians(
                float(self.get_parameter("retract_rz_deg").value))
            box_length = float(self.box_model_params["box_length"])
            box_width = float(self.box_model_params["box_width"])
            box_height = float(self.box_model_params["box_height"])
            half_extent_x = max(
                abs(math.cos(retract_rz)) * box_length / 2.0,
                abs(math.sin(retract_rz)) * box_width / 2.0)
            target_near_face_x = body_surface_x + retract_body_clearance
            target_center_x = target_near_face_x + half_extent_x
            retract_center_x = min(target_center_x, float(center[0]))
            retract_box_center = np.array([
                retract_center_x,
                0.0,
                float(center[2]) + float(detection.normal[2]) * aftergrasp_lift,
            ])
            retract_box_tf = np.eye(4)
            retract_box_tf[:3, :3] = rpy_matrix([0.0, 0.0, retract_rz])
            retract_box_tf[:3, 3] = retract_box_center
            # 保存模式箱位姿（yaml 箱体尺寸），供 GUI 模式切换时更新
            # 绿色 box marker 的位置。
            self._retract_box_pose = (
                retract_box_center.copy(),
                retract_box_tf[:3, :3].copy(),
                np.array([box_length, box_width, box_height], dtype=float),
            )
            self.get_logger().info(
                f"  收回目标: 近身面X={target_near_face_x:.3f}m, "
                f"箱体中心=({retract_box_center[0]:.3f}, 0.000, "
                f"{retract_box_center[2]:.3f})m, "
                f"收回距离X={float(center[0]) - retract_center_x:.3f}m, "
                f"箱体姿态rx=0 ry=0 rz={math.degrees(retract_rz):.0f}°")

            for side, pub, pre_pub, after_pub in (
                    ("left", self.left_pub, self.left_pre_pub, self.left_after_pub),
                    ("right", self.right_pub, self.right_pre_pub, self.right_after_pub)):
                grasp, inward, pregrasp = grasps[side]
                # box 抓取点坐标系（抓取位姿 tool_rot），必须与 yaml 中
                # wrist_to_grasp 的 tool 坐标系约定一致：
                #   绿 +Y = 向上（base_link +Z）
                #   蓝 +Z = 向箱子里面（inward，由 detector 给出）
                #   红 +X = Y×Z（右手系；左手朝前 +X、右手朝后 -X）
                tool_y = np.array([0.0, 0.0, 1.0])
                tool_z = inward - tool_y * np.dot(tool_y, inward)
                tool_z_norm = np.linalg.norm(tool_z)
                tool_z = tool_z / tool_z_norm if tool_z_norm > 1e-9 else np.array([0.0, 1.0, 0.0])
                tool_x = np.cross(tool_y, tool_z)
                tool_x_norm = np.linalg.norm(tool_x)
                tool_x = tool_x / tool_x_norm if tool_x_norm > 1e-9 else np.array([1.0, 0.0, 0.0])
                tool_rot = np.column_stack((tool_x, tool_y, tool_z))
                pub.publish(make_pose(self.target_frame, stamp, grasp, tool_rot))
                pre_pub.publish(make_pose(self.target_frame, stamp, pregrasp, tool_rot))
                aftergrasp = grasp + detection.normal * aftergrasp_lift
                after_pub.publish(make_pose(self.target_frame, stamp, aftergrasp, tool_rot))

                # 手腕位姿 = 抓取位姿 @ inv(wrist_to_grasp)
                # T_world_wrist = T_world_grasp @ inv(T_wrist_grasp)
                grasp_tf = np.eye(4)
                grasp_tf[:3, :3], grasp_tf[:3, 3] = tool_rot, grasp
                pregrasp_tf = grasp_tf.copy()
                pregrasp_tf[:3, 3] = pregrasp
                aftergrasp_tf = grasp_tf.copy()
                aftergrasp_tf[:3, 3] = aftergrasp
                # 保持双手相对箱体的夹持关系：先求抬起状态下的
                # T_box_tool，再应用到 rx=ry=rz=0 的收回箱体目标。
                aftergrasp_box_tf = np.eye(4)
                aftergrasp_box_tf[:3, :3] = detection.rotation
                aftergrasp_box_tf[:3, 3] = (
                    center + detection.normal * aftergrasp_lift)
                box_to_tool = np.linalg.inv(aftergrasp_box_tf) @ aftergrasp_tf
                retract_tf = retract_box_tf @ box_to_tool
                wg_inv = np.linalg.inv(self.wrist_to_grasp[side])
                wrist_tf = grasp_tf @ wg_inv
                wrist_pre_tf = pregrasp_tf @ wg_inv
                wrist_after_tf = aftergrasp_tf @ wg_inv
                wrist_retract_tf = retract_tf @ wg_inv
                wrist_pub = self.left_wrist_pub if side == "left" else self.right_wrist_pub
                wrist_pre_pub = (self.left_wrist_pre_pub if side == "left"
                                 else self.right_wrist_pre_pub)
                wrist_after_pub = (self.left_wrist_after_pub if side == "left"
                                   else self.right_wrist_after_pub)
                wrist_retract_pub = (self.left_wrist_retract_pub if side == "left"
                                     else self.right_wrist_retract_pub)
                wrist_pub.publish(make_pose(self.target_frame, stamp,
                                            wrist_tf[:3, 3], wrist_tf[:3, :3]))
                wrist_pre_pub.publish(make_pose(self.target_frame, stamp,
                                                wrist_pre_tf[:3, 3], wrist_pre_tf[:3, :3]))
                wrist_after_pub.publish(make_pose(self.target_frame, stamp,
                                                  wrist_after_tf[:3, 3], wrist_after_tf[:3, :3]))
                wrist_retract_pub.publish(make_pose(self.target_frame, stamp,
                                                    wrist_retract_tf[:3, 3],
                                                    wrist_retract_tf[:3, :3]))
                frame_debug[side] = {
                    "grasp": grasp_tf,
                    "aftergrasp": aftergrasp_tf,
                    "wrist_to_grasp": self.wrist_to_grasp[side],
                    "wrist": wrist_tf,
                    "wrist_pre": wrist_pre_tf,
                    "wrist_after": wrist_after_tf,
                    "wrist_retract": wrist_retract_tf,
                }

            # ---- 手腕目标日志 ----
            wrist_positions = {}
            for side, data in frame_debug.items():
                wrist_positions[side] = data["wrist"][:3, 3]
            self.get_logger().info(
                f"  左手腕: ({wrist_positions['left'][0]:.3f}, "
                f"{wrist_positions['left'][1]:.3f}, "
                f"{wrist_positions['left'][2]:.3f})m  "
                f"右手腕: ({wrist_positions['right'][0]:.3f}, "
                f"{wrist_positions['right'][1]:.3f}, "
                f"{wrist_positions['right'][2]:.3f})m",
                throttle_duration_sec=2.0)

            # Only an explicit destination ROI can identify a placement
            # surface. Otherwise the source table would be mislabeled as the
            # destination and the orange marker would appear beside the box.
            placement = None
            if self.placement_enabled and self.roi_min is not None and self.roi_max is not None:
                placement = self.detector.detect_support_surface(
                    points, self.roi_min, self.roi_max)
            placement_data = None
            if placement is not None:
                p = placement.center + placement.support_plane[:3] * detection.dimensions[2] / 2.0
                self.placement_pub.publish(make_pose(self.target_frame, stamp, p, placement.rotation))
                placement_data = {"center_m": [round(float(v), 4) for v in p],
                                  "points": placement.point_count}
            self.marker_pub.publish(self.make_markers(
                detection, grasps, stamp, placement, frame_debug))
            self.status_pub.publish(String(data=json.dumps({
                "state": "target_ready", "frame": self.target_frame,
                "box_center_m": [round(float(v), 4) for v in detection.center],
                "dimensions_m": [round(float(v), 4) for v in detection.dimensions],
                "wrist_to_grasp": {
                    side: {
                        "translation_m": [round(float(v), 5) for v in data["wrist_to_grasp"][:3, 3]],
                    } for side, data in frame_debug.items()},
                "placement": placement_data})))

            if self.stable_count >= self.stable_count_threshold:
                # 保存最终检测结果，并触发 IK 节点求解 4 组关节数据；
                # action 执行体收到 ik_result 后返回成功结果。
                self._final_box_info = {"detection": detection, "grasps": grasps,
                                        "frame_debug": frame_debug}
                self._final_time = time.monotonic()
                self.latest_ik_data = None
                self.ik_print_pub.publish(Empty())
                self.checking = False
                self.stable_count = 0
                self.last_box_center = None
                self.get_logger().info("已达稳定次数，等待 IK 结果并返回 action 结果")
                # 识别完成时打印一次放置高度，便于核对是否压桌，避免每秒重复打印。
                _base = self._get_base_link_height()
                _clearance = float(self.get_parameter("place_clearance").value)
                _place_z = self.place_table_height - _base + _clearance
                self.get_logger().info(
                    f"放置高度: base_link离地={_base:.3f}m, "
                    f"place_table_height={self.place_table_height:.3f}m, "
                    f"place_clearance={_clearance:.3f}m, place_z={_place_z:.3f}m")
        except Exception as exc:
            self.get_logger().warning(f"box_grasp_demo: {exc}")

    @staticmethod
    def hand_rotation(inward, up):
        x = inward / max(np.linalg.norm(inward), 1e-9)
        z = up / max(np.linalg.norm(up), 1e-9)
        y = np.cross(z, x)
        y /= max(np.linalg.norm(y), 1e-9)
        return np.column_stack((x, y, z))

    def _box_pose_for_mode(self, mode):
        """返回当前模式对应的箱体位姿 (center, rotation, dimensions)。

        pregrasp/grasp/aftergrasp 用检测位姿；retract 用收回位姿
        （yaml 箱体尺寸 + retract_rz_deg 转角）；place 用放置位姿。
        尚无对应位姿时返回 None。
        """
        if mode == "retract":
            return self._retract_box_pose
        if mode == "place":
            return self._place_box_pose
        if mode in ("pregrasp", "grasp", "aftergrasp"):
            return self._detected_box_pose
        return None

    def _on_pose_command(self, msg):
        """GUI 模式切换：非零位模式把绿色 box marker 移到对应箱体位姿。"""
        mode = msg.data
        self._current_mode = mode
        if mode == "zero":
            self.get_logger().info("模式 zero: 箱子 marker 保持不动")
            return
        pose = self._box_pose_for_mode(mode)
        if pose is None:
            self.get_logger().info(
                f"模式 {mode}: 尚无箱体位姿（等待检测/配置），跳过 marker 更新")
            return
        center, rotation, dimensions = pose
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = Time()
        marker.ns, marker.id = "box", 0
        marker.type, marker.action = Marker.CUBE, Marker.ADD
        marker.pose = make_pose(self.target_frame, Time(), center, rotation).pose
        marker.scale.x, marker.scale.y, marker.scale.z = [float(v) for v in dimensions]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.8, 0.2, 0.35
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.get_logger().info(
            f"模式 {mode}: 箱子 marker → ({center[0]:.3f}, {center[1]:.3f}, "
            f"{center[2]:.3f})m")

    def _publish_tool_markers(self):
        """定时发布 tool 坐标轴 marker（球 + RGB 三轴），挂 wrist_roll_link 上。

        marker 位姿取固定安装变换 wrist_to_grasp（wrist 系），RViz 会按
        最新 TF 重投影，因此切换 IK 模式时 tool 坐标系跟随手腕运动。
        """
        markers = MarkerArray()
        # stamp 置零：RViz 用最新 TF 重投影，避免 marker 时间戳比 TF 树
        # 最新数据还新导致 "No transform to fixed frame" 报错。
        stamp = Time()
        for side_id, side in enumerate(("left", "right")):
            wrist_link = ("L_wrist_roll_link" if side == "left"
                          else "R_wrist_roll_link")
            wrist_to_grasp = self.wrist_to_grasp[side]
            origin = wrist_to_grasp[:3, 3]
            q = matrix_to_quaternion(wrist_to_grasp[:3, :3])

            sphere = Marker()
            sphere.header.frame_id, sphere.header.stamp = wrist_link, stamp
            sphere.ns, sphere.id = "tool", 100 + side_id * 20
            sphere.type, sphere.action = Marker.SPHERE, Marker.ADD
            sphere.pose.position.x = float(origin[0])
            sphere.pose.position.y = float(origin[1])
            sphere.pose.position.z = float(origin[2])
            sphere.pose.orientation.x = float(q[0])
            sphere.pose.orientation.y = float(q[1])
            sphere.pose.orientation.z = float(q[2])
            sphere.pose.orientation.w = float(q[3])
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.035
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.85, 0.0, 1.0
            markers.markers.append(sphere)

            axis_colors = ((1.0, 0.1, 0.1), (0.1, 1.0, 0.1), (0.1, 0.4, 1.0))
            for axis, axis_color in enumerate(axis_colors):
                arrow = Marker()
                arrow.header.frame_id, arrow.header.stamp = wrist_link, stamp
                arrow.ns, arrow.id = "tool", 100 + side_id * 20 + 1 + axis
                arrow.type, arrow.action = Marker.ARROW, Marker.ADD
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.018, 0.025
                end = origin + wrist_to_grasp[:3, axis] * 0.12
                arrow.points = [Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2])),
                                Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = *axis_color, 1.0
                markers.markers.append(arrow)
        self.marker_pub.publish(markers)

    def make_markers(self, detection, grasps, stamp, placement=None, frame_debug=None):
        markers = MarkerArray()
        box = Marker()
        box.header.frame_id, box.header.stamp = self.target_frame, stamp
        box.ns, box.id, box.type, box.action = "box", 0, Marker.CUBE, Marker.ADD
        # GUI 切换过非零位模式时，box marker 用该模式的箱体位姿
        # （收回/放置），否则保持检测位姿。
        mode_pose = self._box_pose_for_mode(self._current_mode)
        if mode_pose is not None:
            box.pose = make_pose(self.target_frame, stamp,
                                 mode_pose[0], mode_pose[1]).pose
            box.scale.x, box.scale.y, box.scale.z = [float(v) for v in mode_pose[2]]
        else:
            box.pose = make_pose(self.target_frame, stamp,
                                 detection.center, detection.rotation).pose
            box.scale.x, box.scale.y, box.scale.z = [float(v) for v in detection.dimensions]
        box.color.r, box.color.g, box.color.b, box.color.a = 0.1, 0.8, 0.2, 0.35
        markers.markers.append(box)
        for marker_id, side in ((1, "left"), (2, "right")):
            grasp, _, pregrasp = grasps[side]
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.target_frame, stamp
            marker.ns, marker.id, marker.type, marker.action = "grasp", marker_id, Marker.ARROW, Marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.04, 0.04
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.3, 1.0, 0.9
            marker.points = [Point(x=float(pregrasp[0]), y=float(pregrasp[1]), z=float(pregrasp[2])),
                             Point(x=float(grasp[0]), y=float(grasp[1]), z=float(grasp[2]))]
            markers.markers.append(marker)
        # tool 坐标轴已改由 _publish_tool_markers 定时挂 wrist_roll_link
        # 发布（随 TF 运动）；这里只清理旧版绝对坐标 marker 的残留 id。
        if frame_debug:
            for side_id in range(2):
                for del_ns, del_id in (
                        ("tool", 104 + side_id * 20),
                        ("wrist_to_tool", 120 + side_id * 20),
                        ("wrist_to_tool", 121 + side_id * 20),
                        ("wrist_target", 110 + side_id * 20),
                        ("wrist_target", 111 + side_id * 20),
                        ("wrist_target", 112 + side_id * 20),
                        ("wrist_target", 113 + side_id * 20)):
                    del_m = Marker()
                    del_m.header.frame_id, del_m.header.stamp = self.target_frame, stamp
                    del_m.ns, del_m.id, del_m.action = del_ns, del_id, Marker.DELETE
                    markers.markers.append(del_m)
        if placement is not None:
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.target_frame, stamp
            marker.ns, marker.id, marker.type, marker.action = "placement", 0, Marker.CUBE, Marker.ADD
            p = placement.center + placement.support_plane[:3] * detection.dimensions[2] / 2.0
            marker.pose = make_pose(self.target_frame, stamp, p, placement.rotation).pose
            marker.scale.x = max(float(placement.dimensions[0]), 0.05)
            marker.scale.y = max(float(placement.dimensions[1]), 0.05)
            marker.scale.z = float(detection.dimensions[2])
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.6, 0.1, 0.35
            markers.markers.append(marker)
        else:
            # Clear a stale placement marker if ROI detection is disabled or
            # the destination surface temporarily disappears.
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.target_frame, stamp
            marker.ns, marker.id, marker.action = "placement", 0, Marker.DELETE
            markers.markers.append(marker)
        return markers

    def publish_status(self, state):
        self.status_pub.publish(String(data=json.dumps({"state": state})))


def main(args=None):
    rclpy.init(args=args)
    node = BoxGraspNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
