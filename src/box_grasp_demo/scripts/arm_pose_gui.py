#!/usr/bin/python3
"""轻量 GUI：切换双臂目标模式（预抓 / 抓取 / 抓取后 / 零位）。

发布 std_msgs/String 到 arm_ik_pinocchio 的 pose_command_topic，
对应 _set_mode 中的 "pregrasp" / "grasp" / "aftergrasp" / "retract" /
"place" / "zero"。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ArmPoseGui(Node):
    def __init__(self):
        super().__init__("arm_pose_gui")
        self.declare_parameter("pose_command_topic", "/demo3/pose_command")
        topic = str(self.get_parameter("pose_command_topic").value)
        self._pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"GUI 命令话题: {topic}")

        import tkinter as tk

        self._tk = tk
        self._root = tk.Tk()
        self._root.title("Arm IK Pose Control")
        self._root.geometry("680x180")

        self._status = tk.StringVar(value="当前模式：等待选择")
        tk.Label(self._root, textvariable=self._status,
                 font=("sans", 14)).pack(pady=12)

        row = tk.Frame(self._root)
        row.pack(pady=8)
        for label, mode in (("预抓取", "pregrasp"),
                            ("抓取", "grasp"),
                            ("抓取后", "aftergrasp"),
                            ("收回", "retract"),
                            ("放置", "place"),
                            ("零位", "zero")):
            btn = tk.Button(row, text=label, width=10, height=2,
                            command=lambda m=mode, l=label: self._send(m, l))
            btn.pack(side=tk.LEFT, padx=6)

        self._closing = False
        # Tkinter 与 ROS2 在同一线程内交替执行
        self._root.after(10, self._spin_once)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _send(self, mode: str, label: str) -> None:
        msg = String()
        msg.data = mode
        self._pub.publish(msg)
        self._status.set(f"当前模式：{label}")
        self.get_logger().info(f"切换模式: {mode}")

    def _spin_once(self) -> None:
        if self._closing:
            return
        rclpy.spin_once(self, timeout_sec=0.01)
        self._root.after(10, self._spin_once)

    def _on_close(self) -> None:
        self._closing = True
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ArmPoseGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
