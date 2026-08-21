#!/usr/bin/python3
"""头部关节 TF 覆盖 — 读取 /mc/joint_states 真实角度，发布动态 /tf。

系统 robot_state_publisher 发布 fixed 头部 TF（不反映实际角度）。
此节点发布 waists→head_yaw→head_pitch 的正确变换，TF2 自动覆盖固定值。
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


def rpy_to_quat(rpy):
    """ROS URDF rpy (fixed-axis XYZ) → quaternion [x,y,z,w]."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def rpy_to_mat(rpy):
    """ROS URDF rpy (fixed-axis XYZ) → 3x3 rotation matrix."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return R


def rotate_z(angle):
    """Rotation matrix around Z by angle (rad)."""
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def rotate_y(angle):
    """Rotation matrix around Y by angle (rad)."""
    c, s = math.cos(angle), math.sin(angle)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def mat_mul(A, B):
    """3x3 matrix multiplication."""
    return [
        [A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
         A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
         A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2]],
        [A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
         A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
         A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2]],
        [A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
         A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
         A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2]],
    ]


def mat_to_quat(R):
    """3x3 rotation matrix → quaternion [x,y,z,w]."""
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


class HeadTFBridge(Node):
    """发布动态头部 TF，覆盖 robot_state_publisher 的固定值。

    URDF 定义:
      waist_pitch_link → head_yaw_link  (origin: [0.0031,0,0.4841] rpy=[0,0.0065,0])
      head_yaw_link → head_pitch_link  (origin: [0,0,0.025] rpy=[0,0,0])

    关节:
      head_yaw_joint   → Z 轴旋转 (yaw)
      head_pitch_joint → Y 轴旋转 (pitch)
    """

    def __init__(self):
        super().__init__("head_tf_bridge")

        # URDF origin 参数
        self.waist_to_yaw_xyz = [0.0031354, 0.0, 0.48409]
        self.waist_to_yaw_rpy = [0.0, 0.0064767, 0.0]

        self.yaw_to_pitch_xyz = [0.0, 0.0, 0.025]
        self.yaw_to_pitch_rpy = [0.0, 0.0, 0.0]

        # 缓存最新关节值
        self._pitch = 0.0
        self._yaw = 0.0
        self._has_data = False

        self._pub = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(
            JointState, "/mc/joint_states", self._cb, 10)
        # 50Hz 高频发布以覆盖系统固件 TF
        self._timer = self.create_timer(0.02, self._publish_tf)
        self.get_logger().info("头部 TF 桥接就绪 (50Hz)")

    def _cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == "head_pitch_joint":
                self._pitch = float(pos)
            elif name == "head_yaw_joint":
                self._yaw = float(pos)
        self._has_data = True

    def _publish_tf(self):
        if not self._has_data:
            return
        pitch = self._pitch
        yaw = self._yaw
        now = self.get_clock().now().to_msg()
        tfs = []

        # ---- waist_pitch_link → head_yaw_link (yaw joint, Z axis) ----
        R_origin = rpy_to_mat(self.waist_to_yaw_rpy)
        R_yaw = rotate_z(yaw or 0.0)
        R = mat_mul(R_origin, R_yaw)
        qx, qy, qz, qw = mat_to_quat(R)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "waist_pitch_link"
        t.child_frame_id = "head_yaw_link"
        t.transform.translation.x = self.waist_to_yaw_xyz[0]
        t.transform.translation.y = self.waist_to_yaw_xyz[1]
        t.transform.translation.z = self.waist_to_yaw_xyz[2]
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        tfs.append(t)

        # ---- head_yaw_link → head_pitch_link (pitch joint, Y axis) ----
        R_origin2 = rpy_to_mat(self.yaw_to_pitch_rpy)
        R_pitch = rotate_y(pitch)
        R2 = mat_mul(R_origin2, R_pitch)
        qx2, qy2, qz2, qw2 = mat_to_quat(R2)

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "head_yaw_link"
        t2.child_frame_id = "head_pitch_link"
        t2.transform.translation.x = self.yaw_to_pitch_xyz[0]
        t2.transform.translation.y = self.yaw_to_pitch_xyz[1]
        t2.transform.translation.z = self.yaw_to_pitch_xyz[2]
        t2.transform.rotation.x = qx2
        t2.transform.rotation.y = qy2
        t2.transform.rotation.z = qz2
        t2.transform.rotation.w = qw2
        tfs.append(t2)

        self._pub.publish(TFMessage(transforms=tfs))


def main(args=None):
    rclpy.init(args=args)
    node = HeadTFBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
