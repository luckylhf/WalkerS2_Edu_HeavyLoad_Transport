#!/usr/bin/python3
"""Republish one extracted real S2 cloud with its matching TF and joint state.

The extracted frame comes from a static rosbag scene.  It is intentionally
republished at a low rate so late-starting RViz, detector, and IK nodes all
receive exactly the same recorded scene rather than a synthetic box cloud.
"""

import pickle
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Float64, Header
from tf2_msgs.msg import TFMessage


class RecordedFramePublisher(Node):
    def __init__(self):
        super().__init__("recorded_frame_publisher")
        self.declare_parameter("frame_data_dir", "")
        self.declare_parameter("cloud_topic", "/sensor/camera/stereo/pointcloud/raw")
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("exclude_arm_tf", False)  # 真机录制帧不发布手臂动态 TF，改由 IK 发布
        self.declare_parameter("enable_sim_crouch", False)
        self.declare_parameter(
            "sim_crouch_command_topic", "/demo3/sim/base_link_height_cmd")
        self.declare_parameter(
            "sim_base_height_topic", "/demo3/sim/base_link_height")
        self.declare_parameter("sim_crouch_speed", 0.05)
        data_dir = Path(str(self.get_parameter("frame_data_dir").value)).expanduser()
        if not data_dir.is_dir():
            raise RuntimeError(f"frame_data_dir does not exist: {data_dir}")

        with (data_dir / "cloud.pkl").open("rb") as stream:
            self.cloud = pickle.load(stream)
        with (data_dir / "tf_static.pkl").open("rb") as stream:
            self.tf_static = pickle.load(stream)
        with (data_dir / "tf_dynamic.pkl").open("rb") as stream:
            self.tf_dynamic = pickle.load(stream)
        with (data_dir / "joint_state.pkl").open("rb") as stream:
            self.joint_state = pickle.load(stream)

        # RViz subscribes as RELIABLE by default.  A RELIABLE publisher also
        # remains compatible with the detector's BEST_EFFORT subscription.
        cloud_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=5)
        static_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("cloud_topic").value), cloud_qos)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.joint_pub = self.create_publisher(JointState, "/mc/joint_states", 10)

        # 离线回放中的“下蹲”不调用真机 HTTP。控制端发布目标 base_link
        # 绝对离地高度，本节点平滑修改 base_footprint→base_link，并将固定录制
        # 点云按相反方向平移，使桌面/箱体相对机身的几何随下蹲真实变化。
        self._sim_crouch_enabled = bool(
            self.get_parameter("enable_sim_crouch").value)
        if self._sim_crouch_enabled:
            self._recorded_base_height = self._find_recorded_base_height()
            self._sim_base_height = self._recorded_base_height
            self._sim_target_base_height = self._recorded_base_height
            self._sim_crouch_speed = max(
                float(self.get_parameter("sim_crouch_speed").value), 0.001)
            self._last_sim_update = time.monotonic()
            self._base_from_cloud_rotation = (
                self._find_base_from_cloud_rotation())
            command_topic = str(
                self.get_parameter("sim_crouch_command_topic").value)
            height_topic = str(
                self.get_parameter("sim_base_height_topic").value)
            self.create_subscription(
                Float64, command_topic, self._on_sim_height_command, 10)
            self.sim_height_pub = self.create_publisher(
                Float64, height_topic, 10)
            self.get_logger().info(
                f"仿真下蹲已启用: command={command_topic}, "
                f"state={height_topic}, 初始高度={self._recorded_base_height:.3f}m, "
                f"速度={self._sim_crouch_speed:.3f}m/s")

        # 若启用，则从录制帧的动态 TF 中剔除手臂运动链，
        # 避免与 arm_ik 发布到 /tf 的手臂姿态产生冲突。
        self._exclude_arm_tf = bool(self.get_parameter("exclude_arm_tf").value)
        self._arm_tf_children = {
            "L_shoulder_pitch_link", "L_shoulder_roll_link", "L_shoulder_yaw_link",
            "L_elbow_roll_link", "L_elbow_yaw_link", "L_wrist_pitch_link", "L_wrist_roll_link",
            "R_shoulder_pitch_link", "R_shoulder_roll_link", "R_shoulder_yaw_link",
            "R_elbow_roll_link", "R_elbow_yaw_link", "R_wrist_pitch_link", "R_wrist_roll_link",
        }

        self._publish_static()
        rate = max(float(self.get_parameter("publish_rate").value), 0.1)
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self._publish()
        self.get_logger().info(
            f"replaying one recorded cloud: {len(self.cloud['points'])} points, "
            f"frame={self.cloud['stamp']['frame_id']}")

    def _find_recorded_base_height(self):
        for item in self.tf_dynamic:
            if (item["parent"] == "base_footprint"
                    and item["child"] == "base_link"):
                return float(item["xyz"][2])
        raise RuntimeError(
            "recorded frame has no base_footprint -> base_link transform")

    @staticmethod
    def _transform_matrix(item):
        """Return parent_from_child as a 4x4 matrix."""
        x, y, z, w = (float(v) for v in item["xyzw"])
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
             2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
             2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w),
             1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)
        matrix[:3, 3] = np.asarray(item["xyz"], dtype=np.float64)
        return matrix

    def _find_base_from_cloud_rotation(self):
        """Resolve the recorded rotation base_link_from_cloud_frame."""
        cloud_frame = str(self.cloud["stamp"]["frame_id"])
        transforms = list(self.tf_static) + list(self.tf_dynamic)
        graph = {}
        for item in transforms:
            parent, child = item["parent"], item["child"]
            parent_from_child = self._transform_matrix(item)
            graph.setdefault(child, []).append((parent, parent_from_child))
            graph.setdefault(parent, []).append(
                (child, np.linalg.inv(parent_from_child)))

        queue = [(cloud_frame, np.eye(4, dtype=np.float64))]
        visited = {cloud_frame}
        while queue:
            frame, frame_from_cloud = queue.pop(0)
            if frame == "base_link":
                return frame_from_cloud[:3, :3]
            for neighbour, neighbour_from_frame in graph.get(frame, []):
                if neighbour in visited:
                    continue
                visited.add(neighbour)
                queue.append(
                    (neighbour, neighbour_from_frame @ frame_from_cloud))
        raise RuntimeError(
            f"recorded TF cannot resolve base_link <- {cloud_frame}")

    def _on_sim_height_command(self, msg):
        target = float(msg.data)
        if not np.isfinite(target) or not 0.4 <= target <= 1.2:
            self.get_logger().warning(
                f"忽略非法仿真 base_link 高度: {target}")
            return
        changed = abs(target - self._sim_target_base_height) > 1e-6
        self._sim_target_base_height = target
        if changed:
            self.get_logger().info(f"仿真下蹲目标高度: {target:.3f}m")

    def _update_sim_height(self):
        now = time.monotonic()
        dt = max(now - self._last_sim_update, 0.0)
        self._last_sim_update = now
        error = self._sim_target_base_height - self._sim_base_height
        step = self._sim_crouch_speed * dt
        if abs(error) <= step:
            self._sim_base_height = self._sim_target_base_height
        elif error > 0.0:
            self._sim_base_height += step
        else:
            self._sim_base_height -= step

    @staticmethod
    def _stamp_now(node):
        return node.get_clock().now().to_msg()

    @staticmethod
    def _transform(item, stamp):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = item["parent"]
        msg.child_frame_id = item["child"]
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = item["xyz"]
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = item["xyzw"]
        return msg

    def _publish_static(self):
        # Static TF convention uses a zero stamp.
        zero = Time()
        self.static_pub.publish(TFMessage(
            transforms=[self._transform(item, zero) for item in self.tf_static]))

    def _publish(self):
        if self._sim_crouch_enabled:
            self._update_sim_height()
        stamp = self._stamp_now(self)
        transforms = [self._transform(item, stamp) for item in self.tf_dynamic]
        if self._sim_crouch_enabled:
            for transform in transforms:
                if (transform.header.frame_id == "base_footprint"
                        and transform.child_frame_id == "base_link"):
                    transform.transform.translation.z = self._sim_base_height
                    break
        if self._exclude_arm_tf:
            transforms = [t for t in transforms
                          if t.child_frame_id not in self._arm_tf_children]
        self.tf_pub.publish(TFMessage(transforms=transforms))

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(self.joint_state["name"])
        joint.position = list(self.joint_state["position"])
        joint.velocity = list(self.joint_state.get("velocity", []))
        joint.effort = list(self.joint_state.get("effort", []))
        self.joint_pub.publish(joint)

        header = Header(stamp=stamp, frame_id=self.cloud["stamp"]["frame_id"])
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = np.asarray(self.cloud["points"], dtype=np.float32)
        if self._sim_crouch_enabled:
            # 下蹲后固定桌面在 base_link 系中应升高 recorded_h-current_h。
            # 点仍以相机坐标发布，因此先把 base_link +Z 位移旋转回相机系。
            vertical_shift = self._recorded_base_height - self._sim_base_height
            cloud_shift = (
                self._base_from_cloud_rotation.T
                @ np.array([0.0, 0.0, vertical_shift], dtype=np.float64)
            ).astype(np.float32)
            points = points + cloud_shift
        msg = PointCloud2(header=header, height=1, width=len(points), fields=fields,
                          is_bigendian=False, point_step=12, row_step=12 * len(points),
                          is_dense=True, data=points.tobytes())
        self.cloud_pub.publish(msg)
        if self._sim_crouch_enabled:
            self.sim_height_pub.publish(Float64(data=self._sim_base_height))


def main(args=None):
    rclpy.init(args=args)
    node = RecordedFramePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
