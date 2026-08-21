#!/usr/bin/python3
"""关节状态桥接 — 从 /mc/joint_states 转换关节名并发布到 /joint_states。

解决两个问题：
1. 踝关节串并转换: driver_inside/outside → pitch/roll
2. 头部关节缺少: head_pitch/yaw_joint 不在 URDF 用名中

暂定转换公式（并联机构 → 串联 URDF）:
  ankle_pitch = (driver_inside + driver_outside) / 2
  ankle_roll  = (driver_inside - driver_outside) / 2

注意：踝关节串并关系尚未用真机脚底/IMU 标定验证。它只用于
本地 RobotModel 显示，不能作为 base_link 离地高度或 ground/map
变换的依据。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")

        # 需要转换的踝关节
        self._ankle_pairs = [
            ("L_ankle_driver_inside_joint", "L_ankle_driver_outside_joint",
             "L_ankle_pitch_joint", "L_ankle_roll_joint"),
            ("R_ankle_driver_inside_joint", "R_ankle_driver_outside_joint",
             "R_ankle_pitch_joint", "R_ankle_roll_joint"),
        ]

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            JointState, "/mc/joint_states", self._cb, 10)
        self.get_logger().info("关节状态桥接就绪 — /mc/joint_states → /joint_states")

    def _cb(self, msg: JointState):
        # 建立索引
        idx = {n: i for i, n in enumerate(msg.name)}

        # 提取需要转换的踝关节值
        mc_values = {}
        for inside_name, outside_name, pitch_name, roll_name in self._ankle_pairs:
            if inside_name in idx and outside_name in idx:
                mc_values[inside_name] = float(msg.position[idx[inside_name]])
                mc_values[outside_name] = float(msg.position[idx[outside_name]])

        if not mc_values:
            return  # 等待包含踝关节的消息

        # 构建新的 JointState
        out = JointState()
        out.header = msg.header
        out.name = []
        out.position = []

        for name, pos in zip(msg.name, msg.position):
            p = float(pos)

            # 检查是否是踝关节 driver（用 MC 名字跳过，后面用 URDF 名添加）
            is_ankle_driver = False
            for inside_name, outside_name, pitch_name, roll_name in self._ankle_pairs:
                if name == inside_name:
                    is_ankle_driver = True
                    inside_val = p
                    # 等到遇到对应的 outside 再一起处理
                    continue
                elif name == outside_name:
                    is_ankle_driver = True
                    outside_val = p
                    # 串并转换: pitch = (in+out)/2, roll = (in-out)/2
                    in_val = mc_values.get(inside_name, 0.0)
                    out_val = p
                    pitch = (in_val + out_val) / 2.0
                    roll = (in_val - out_val) / 2.0
                    out.name.append(pitch_name)
                    out.position.append(pitch)
                    out.name.append(roll_name)
                    out.position.append(roll)
                    continue

            if not is_ankle_driver:
                out.name.append(name)
                out.position.append(p)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
