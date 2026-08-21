#!/usr/bin/env python3
"""Check the dual-arm poses published by box_grasp_node_ros2."""

import json
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


def pose_matrix(pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def position(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)


class GraspValidation(Node):
    def __init__(self):
        super().__init__("grasp_validation")
        self.declare_parameter("box_length", 0.40)
        self.declare_parameter("box_width", 0.30)
        self.declare_parameter("box_height", 0.20)
        self.declare_parameter("side_clearance", 0.018)
        self.declare_parameter("pregrasp_distance", 0.12)
        self.declare_parameter("tool_contact_below_top", 0.10)
        self.declare_parameter("grasp_long_edge", False)
        self.declare_parameter("tolerance", 0.008)
        self.declare_parameter("base_topic", "/box_grasp_demo")
        self.box = self.left = self.right = None
        self.left_pre = self.right_pre = None
        base = str(self.get_parameter("base_topic").value).rstrip("/") + "/"
        self.result_pub = self.create_publisher(String, base + "validation", 1)
        self.create_subscription(PoseStamped, base + "box_pose", self.set_box, 1)
        self.create_subscription(PoseStamped, base + "left_grasp_pose", self.set_left, 1)
        self.create_subscription(PoseStamped, base + "right_grasp_pose", self.set_right, 1)
        self.create_subscription(PoseStamped, base + "left_pregrasp_pose", self.set_left_pre, 1)
        self.create_subscription(PoseStamped, base + "right_pregrasp_pose", self.set_right_pre, 1)
        self.timer = self.create_timer(0.5, self.validate)
        self.last_result = None

    def set_box(self, msg):
        self.box = msg

    def set_left(self, msg):
        self.left = msg

    def set_right(self, msg):
        self.right = msg

    def set_left_pre(self, msg):
        self.left_pre = msg

    def set_right_pre(self, msg):
        self.right_pre = msg

    def validate(self):
        if any(msg is None for msg in (self.box, self.left, self.right,
                                       self.left_pre, self.right_pre)):
            return
        center = position(self.box.pose)
        box_rotation = pose_matrix(self.box.pose)
        long_axis = box_rotation[:, 0]
        short_axis = box_rotation[:, 1]
        up = box_rotation[:, 2]
        half_length = float(self.get_parameter("box_length").value) / 2.0
        clearance = float(self.get_parameter("side_clearance").value)
        pre_distance = float(self.get_parameter("pregrasp_distance").value)
        box_height = float(self.get_parameter("box_height").value)
        contact_below_top = float(self.get_parameter("tool_contact_below_top").value)
        tolerance = float(self.get_parameter("tolerance").value)
        contact_height = box_height / 2.0 - contact_below_top
        grasp_long_edge = bool(self.get_parameter("grasp_long_edge").value)
        if grasp_long_edge:
            half_width = float(self.get_parameter("box_width").value) / 2.0 + clearance
            expected_left = center + short_axis * half_width + up * contact_height
            expected_right = center - short_axis * half_width + up * contact_height
        else:
            expected_left = center + long_axis * (half_length + clearance) + up * contact_height
            expected_right = center - long_axis * (half_length + clearance) + up * contact_height
        errors = {
            "left_position_m": float(np.linalg.norm(position(self.left.pose) - expected_left)),
            "right_position_m": float(np.linalg.norm(position(self.right.pose) - expected_right)),
            "left_pregrasp_distance_m": abs(float(np.linalg.norm(
                position(self.left_pre.pose) - position(self.left.pose))) - pre_distance),
            "right_pregrasp_distance_m": abs(float(np.linalg.norm(
                position(self.right_pre.pose) - position(self.right.pose))) - pre_distance),
            "left_right_separation_m": float(np.linalg.norm(
                position(self.left.pose) - position(self.right.pose))),
        }
        left_x = pose_matrix(self.left.pose)[:, 0]
        right_x = pose_matrix(self.right.pose)[:, 0]
        left_y = pose_matrix(self.left.pose)[:, 1]
        right_y = pose_matrix(self.right.pose)[:, 1]
        left_z = pose_matrix(self.left.pose)[:, 2]
        right_z = pose_matrix(self.right.pose)[:, 2]

        # box 抓取点坐标系：绿 +Y = 向上，蓝 +Z = 向箱子里面，红 +X = Y×Z。
        world_up = np.array([0.0, 0.0, 1.0])
        thigh_axis = long_axis if not grasp_long_edge else short_axis
        left_inward = -thigh_axis
        right_inward = thigh_axis

        def expected_tool_axes(inward):
            z = inward - world_up * np.dot(world_up, inward)
            z_norm = np.linalg.norm(z)
            z = z / z_norm if z_norm > 1e-9 else np.array([1.0, 0.0, 0.0])
            x = np.cross(world_up, z)
            x_norm = np.linalg.norm(x)
            x = x / x_norm if x_norm > 1e-9 else np.array([0.0, 1.0, 0.0])
            return x, world_up, z

        exp_left_x, exp_left_y, exp_left_z = expected_tool_axes(left_inward)
        exp_right_x, exp_right_y, exp_right_z = expected_tool_axes(right_inward)
        errors["left_tool_y_axis_error"] = float(np.linalg.norm(left_y - exp_left_y))
        errors["right_tool_y_axis_error"] = float(np.linalg.norm(right_y - exp_right_y))
        errors["left_tool_z_axis_error"] = float(np.linalg.norm(left_z - exp_left_z))
        errors["right_tool_z_axis_error"] = float(np.linalg.norm(right_z - exp_right_z))
        errors["left_tool_x_axis_error"] = float(np.linalg.norm(left_x - exp_left_x))
        errors["right_tool_x_axis_error"] = float(np.linalg.norm(right_x - exp_right_x))
        passed = all(value <= tolerance for key, value in errors.items()
                     if key != "left_right_separation_m")
        expected_separation = float(np.linalg.norm(expected_left - expected_right))
        passed = passed and abs(errors["left_right_separation_m"] - expected_separation) <= tolerance
        result = {"state": "PASS" if passed else "FAIL",
                  "frame": self.box.header.frame_id,
                  "errors": {key: round(value, 6) for key, value in errors.items()},
                  "expected_separation_m": round(expected_separation, 6)}
        encoded = json.dumps(result, separators=(",", ":"))
        self.result_pub.publish(String(data=encoded))
        if encoded != self.last_result:
            self.get_logger().info("dual-arm grasp validation: " + encoded)
            self.last_result = encoded


def main(args=None):
    rclpy.init(args=args)
    node = GraspValidation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
