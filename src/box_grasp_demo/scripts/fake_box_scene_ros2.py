#!/usr/bin/env python3
"""Publish a deterministic table and box point cloud for RViz testing.

This node deliberately publishes the same PointCloud2 type as the stereo
camera.  The regular box_grasp_node_ros2 then performs detection and grasp
pose calculation, so this is an end-to-end perception test without a robot.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import String


def rgb_float(red, green, blue):
    """Encode RGB bytes in the float32 field used by RViz PointCloud2."""
    value = (int(red) << 16) | (int(green) << 8) | int(blue)
    return np.array([value], dtype=np.uint32).view(np.float32)[0].item()


class FakeBoxScene(Node):
    def __init__(self):
        super().__init__("fake_box_scene")
        self.declare_parameter("output_topic", "/sim/pointcloud")
        self.declare_parameter("frame_id", "sim_world")
        self.declare_parameter("publish_rate", 2.0)
        # The robot soles are on sim_world/z=0.  The box is on a raised table.
        self.declare_parameter("table_z", 0.90)
        # Robot forward is +X.  Put the box farther forward and rotate its
        # long axis onto Y, so the long side faces the robot.
        self.declare_parameter("box_center", [0.65, 0.00, 1.00])
        self.declare_parameter("box_length", 0.40)
        self.declare_parameter("box_width", 0.30)
        self.declare_parameter("box_height", 0.20)
        self.declare_parameter("box_yaw_deg", 90.0)
        # Keep the baseline RViz scene perfectly stable.  Set this to a
        # non-zero value explicitly when testing sensor noise.
        self.declare_parameter("noise_std", 0.0)
        self.declare_parameter("gui", False)
        self.declare_parameter("random_seed", 21)
        self.declare_parameter("pose_command_topic", "/sim_box_grasp/pose_command")

        self.topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.pub = self.create_publisher(PointCloud2, self.topic, 1)
        self.pose_command_pub = self.create_publisher(
            String, str(self.get_parameter("pose_command_topic").value), 1)
        rate = max(float(self.get_parameter("publish_rate").value), 0.1)
        self.timer = self.create_timer(1.0 / rate, self.publish_scene)
        self.rng = np.random.default_rng(int(self.get_parameter("random_seed").value))
        self._lock = threading.Lock()
        self._box = self._read_box_parameters()
        self.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                       PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                       PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                       PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)]
        self.cloud_data = self.build_scene()
        self.publish_scene()
        if bool(self.get_parameter("gui").value):
            self.start_gui()
        self.get_logger().info(
            f"publishing fake table and box to {self.topic} in frame {self.frame_id}")

    def _read_box_parameters(self):
        return {
            "center": np.asarray(self.get_parameter("box_center").value, dtype=float),
            "length": float(self.get_parameter("box_length").value),
            "width": float(self.get_parameter("box_width").value),
            "height": float(self.get_parameter("box_height").value),
            "yaw_deg": float(self.get_parameter("box_yaw_deg").value),
        }

    def build_scene(self):
        table_z = float(self.get_parameter("table_z").value)
        with self._lock:
            box = {key: value.copy() if isinstance(value, np.ndarray) else value
                   for key, value in self._box.items()}
        center = box["center"]
        length, width, height = box["length"], box["width"], box["height"]
        yaw = math.radians(box["yaw_deg"])
        noise_std = max(float(self.get_parameter("noise_std").value), 0.0)

        # Dense support plane.  Its height is intentionally inside the demo's
        # configured support_height_min/max range.
        grid = np.linspace(-0.45, 1.30, 55)
        tx, ty = np.meshgrid(grid, np.linspace(-0.90, 0.90, 55))
        table = np.column_stack((tx.ravel(), ty.ravel(),
                                 np.full(tx.size, table_z)))

        # Sample the visible top and four side faces in box-local coordinates.
        local_x = np.linspace(-length / 2.0, length / 2.0, 25)
        local_y = np.linspace(-width / 2.0, width / 2.0, 19)
        local_z = np.linspace(0.0, height, 13)
        top_x, top_y = np.meshgrid(local_x, local_y)
        top = np.column_stack((top_x.ravel(), top_y.ravel(),
                               np.full(top_x.size, height)))
        side_x, side_z = np.meshgrid(local_x, local_z)
        front = np.column_stack((side_x.ravel(), np.full(side_x.size, -width / 2.0),
                                 side_z.ravel()))
        back = np.column_stack((side_x.ravel(), np.full(side_x.size, width / 2.0),
                                side_z.ravel()))
        side_y, side_z = np.meshgrid(local_y, local_z)
        left = np.column_stack((np.full(side_y.size, -length / 2.0), side_y.ravel(),
                                side_z.ravel()))
        right = np.column_stack((np.full(side_y.size, length / 2.0), side_y.ravel(),
                                 side_z.ravel()))
        box_local = np.vstack((top, front, back, left, right))
        rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                             [math.sin(yaw), math.cos(yaw), 0.0],
                             [0.0, 0.0, 1.0]])
        # box_center is the actual geometric centre; the default places the
        # bottom face on the raised table.
        box = box_local @ rotation.T + np.array(
            [center[0], center[1], center[2] - height / 2.0])

        if noise_std:
            table = table + self.rng.normal(0.0, noise_std, table.shape)
            box = box + self.rng.normal(0.0, noise_std, box.shape)
        points = np.vstack((table, box))
        table_rgb = rgb_float(135, 135, 135)
        box_rgb = rgb_float(210, 90, 50)
        colors = np.full(len(points), table_rgb, dtype=np.float32)
        colors[len(table):] = box_rgb
        return [(float(p[0]), float(p[1]), float(p[2]), float(c))
                for p, c in zip(points, colors)]

    def randomize_box(self):
        """在 Walker S2 双臂可达空间内随机生成箱子。

        工作空间基于 Pinocchio IK 验证：
        - X: 0.28--0.35 m (base_link 前方舒适区域)
        - Y: -0.10--0.08 m (双手重叠工作区)
        - 桌面高度: 0.87--0.93 m (越高越容易够到)
        - 箱子始终放置在桌面上，偏航角使长边朝向机器人。
        """
        length = float(self._box["length"])
        width = float(self._box["width"])
        height = float(self._box["height"])

        # ---- 随机桌面高度（影响抓取 Z） ----
        table_z = float(self.rng.uniform(0.87, 0.93))
        self.set_parameters([rclpy.parameter.Parameter("table_z", value=table_z)])

        # ---- 箱子中心 XY（保守可达区域，通过率 > 95%） ----
        x_low, x_high = 0.28, 0.33
        y_low, y_high = -0.10, 0.04
        x = float(self.rng.uniform(x_low, x_high))
        y = float(self.rng.uniform(y_low, y_high))
        # 箱子底面在桌面上，中心 Z = table_z + height/2
        z = table_z + height / 2.0

        # ---- 偏航角：长边大致朝向机器人 ----
        yaw = float(self.rng.uniform(82.0, 98.0))

        with self._lock:
            self._box["center"] = np.array([x, y, z], dtype=float)
            self._box["yaw_deg"] = yaw
        self.cloud_data = self.build_scene()

    def set_box_from_gui(self, x, y, yaw, length, width, height):
        with self._lock:
            self._box = {"center": np.array([float(x), float(y), 1.00]),
                         "length": max(float(length), 0.08),
                         "width": max(float(width), 0.08),
                         "height": max(float(height), 0.05),
                         "yaw_deg": float(yaw)}
        self.cloud_data = self.build_scene()

    def publish_pose_command(self, command):
        self.pose_command_pub.publish(String(data=str(command)))

    def start_gui(self):
        """Run a small Tk control panel in a daemon thread."""
        def run():
            try:
                import tkinter as tk
                root = tk.Tk()
                root.title("Walker S2 随机箱体 / 预抓取测试")
                root.resizable(False, False)
                fields = {}
                initial = {"x": self._box["center"][0], "y": self._box["center"][1],
                           "yaw": self._box["yaw_deg"], "length": self._box["length"],
                           "width": self._box["width"], "height": self._box["height"]}
                for row, (name, label) in enumerate((
                        ("x", "中心 X (m)"), ("y", "中心 Y (m)"),
                        ("yaw", "偏航角 (deg)"), ("length", "长度 (m)"),
                        ("width", "宽度 (m)"), ("height", "高度 (m)"))):
                    tk.Label(root, text=label, width=14).grid(row=row, column=0, padx=6, pady=3)
                    value = tk.StringVar(value=f"{initial[name]:.3f}")
                    fields[name] = value
                    tk.Entry(root, textvariable=value, width=12).grid(row=row, column=1, padx=6, pady=3)

                status = tk.StringVar(value="等待发布")
                def apply():
                    try:
                        self.set_box_from_gui(*(float(fields[name].get()) for name in
                                               ("x", "y", "yaw", "length", "width", "height")))
                        status.set("已更新箱体")
                    except ValueError:
                        status.set("参数格式错误")
                def randomize():
                    self.randomize_box()
                    with self._lock:
                        b = self._box.copy()
                    fields["x"].set(f"{b['center'][0]:.3f}")
                    fields["y"].set(f"{b['center'][1]:.3f}")
                    fields["yaw"].set(f"{b['yaw_deg']:.2f}")
                    status.set("已随机生成箱体")
                def command(name):
                    self.publish_pose_command(name)
                    status.set({"zero": "已切换回零姿态",
                                "pregrasp": "已激活预抓取姿态",
                                "grasp": "已激活抓取姿态"}.get(name, name))
                tk.Button(root, text="应用参数", command=apply).grid(row=6, column=0, padx=6, pady=8)
                tk.Button(root, text="随机箱子", command=randomize).grid(row=6, column=1, padx=6, pady=8)
                tk.Button(root, text="回零姿态", command=lambda: command("zero")).grid(row=8, column=0, padx=6, pady=4)
                tk.Button(root, text="预抓取姿态", command=lambda: command("pregrasp")).grid(row=8, column=1, padx=6, pady=4)
                tk.Button(root, text="抓取姿态", command=lambda: command("grasp")).grid(row=9, column=0, columnspan=2, padx=6, pady=4)
                tk.Label(root, textvariable=status, foreground="navy").grid(row=10, column=0, columnspan=2, pady=3)
                root.mainloop()
            except Exception as exc:
                self.get_logger().warning(f"box GUI unavailable: {exc}")
        threading.Thread(target=run, name="box_scene_gui", daemon=True).start()

    def publish_scene(self):

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        with self._lock:
            data = list(self.cloud_data)
        self.pub.publish(point_cloud2.create_cloud(header, self.fields, data))


def main(args=None):
    rclpy.init(args=args)
    node = FakeBoxScene()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
