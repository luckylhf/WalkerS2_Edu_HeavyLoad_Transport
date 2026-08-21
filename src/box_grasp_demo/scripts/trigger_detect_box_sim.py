#!/usr/bin/python3
"""离线 RViz 仿真专用触发器：模拟下蹲后执行 DetectBox。

本程序只使用 ROS 话题、服务和 Action，不包含任何真机 HTTP、导航或运动接口。
需要与 box_grasp_recorded_frame.launch.py 一起使用。
"""

import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64

from box_grasp_demo_msgs.action import DetectBox
from box_grasp_demo_msgs.srv import SuggestCrouch


DEFAULT_ACTION_NAME = "/demo3/box_grasp_demo/detect_box"
DEFAULT_HEIGHT_COMMAND_TOPIC = "/demo3/sim/base_link_height_cmd"
DEFAULT_HEIGHT_STATE_TOPIC = "/demo3/sim/base_link_height"


class SimDetectBoxClient(Node):
    def __init__(self, action_name, command_topic, state_topic):
        super().__init__("detect_box_sim_client")
        self._action_client = ActionClient(self, DetectBox, action_name)
        action_base = action_name.rstrip("/").rsplit("/", 1)[0]
        self._suggest_client = self.create_client(
            SuggestCrouch, f"{action_base}/suggest_crouch")
        self._height_command_pub = self.create_publisher(
            Float64, command_topic, 10)
        self._command_topic = command_topic
        self._state_topic = state_topic
        self._height = None
        self._height_subscription = self.create_subscription(
            Float64, state_topic, self._height_callback, 10)
        self._result = None
        self._status = None

    def _height_callback(self, msg):
        self._height = float(msg.data)

    def wait_for_sim_crouch(self, timeout_sec=5.0):
        """等待回放节点的命令订阅和第一条高度状态，避免启动竞态。"""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self._height is not None
                    and self._height_command_pub.get_subscription_count() > 0):
                return True
        self.get_logger().error(
            "仿真下蹲接口未就绪："
            f"未收到 {self._state_topic} 或 {self._command_topic} 无订阅者；"
            "请确认 ./tools/run_offline_sim.sh 已完成启动并保持运行")
        return False

    def request_crouch(self, mode):
        if not self._suggest_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("suggest_crouch service 未就绪（等待 5s）")
            return None
        request = SuggestCrouch.Request()
        request.mode = mode
        future = self._suggest_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error("suggest_crouch 请求超时（10s）")
            return None
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error("suggest_crouch 请求失败")
            return None
        return response

    def move_to_height(self, target, timeout_sec=10.0, tolerance=0.005):
        """通过仿真 ROS 话题设置高度，并等待状态反馈到位。"""
        deadline = time.monotonic() + timeout_sec
        command = Float64(data=float(target))
        last_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_publish >= 0.25:
                self._height_command_pub.publish(command)
                last_publish = now
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._height is not None and abs(self._height - target) <= tolerance:
                return True
        return False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        center = feedback.box_center
        self.get_logger().info(
            f"[反馈] 稳定 {feedback.stable_count} 次，连续漏检 "
            f"{feedback.miss_count} 次，中心 "
            f"({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})m | "
            f"{feedback.message}")

    def detect(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("DetectBox action server 未就绪（等待 5s）")
            return False
        self.get_logger().info("发送仿真 DetectBox goal ...")
        send_future = self._action_client.send_goal_async(
            DetectBox.Goal(), feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error("发送 DetectBox goal 超时（10s）")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("DetectBox goal 被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90.0)
        if not result_future.done():
            self.get_logger().error("等待 DetectBox 结果超时（90s）")
            goal_handle.cancel_goal_async()
            return False
        response = result_future.result()
        if response is None:
            self.get_logger().error("未收到 DetectBox action result")
            return False
        self._status = response.status
        self._result = response.result
        return (response.status == GoalStatus.STATUS_SUCCEEDED
                and self._result is not None and self._result.success)

    def print_result(self):
        result = self._result
        if result is None:
            print("无结果")
            return
        print("=" * 60)
        print("DetectBox 仿真结果")
        print("=" * 60)
        print(f"status         : {self._status}")
        print(f"success        : {result.success}")
        print(f"message        : {result.message}")
        print(f"box_center     : {list(result.box_center)}")
        print(f"box_dimensions : {list(result.box_dimensions)}")
        print(f"grasp_crouch   : {result.grasp_crouch:.3f} m（仅建议值）")
        print(f"place_crouch   : {result.place_crouch:.3f} m（仅建议值）")
        print(f"ik_data        : {result.ik_data}")
        print("=" * 60)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="离线 RViz 仿真：模拟下蹲后触发箱体检测与 IK")
    parser.add_argument("--action-name", default=DEFAULT_ACTION_NAME)
    parser.add_argument(
        "--crouch-mode", choices=("photo", "grasp", "place"),
        default="photo", help="请求的下蹲建议模式（默认: photo）")
    parser.add_argument(
        "--skip-crouch", action="store_true", help="保持当前仿真高度，直接检测")
    parser.add_argument(
        "--yes", action="store_true", help="不询问，自动执行仿真下蹲")
    parser.add_argument(
        "--height-command-topic", default=DEFAULT_HEIGHT_COMMAND_TOPIC)
    parser.add_argument(
        "--height-state-topic", default=DEFAULT_HEIGHT_STATE_TOPIC)
    parser.add_argument(
        "--crouch-timeout", type=float, default=10.0,
        help="等待仿真高度到位的超时秒数（默认: 10）")
    return parser.parse_args(argv)


def main(args=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    cli_args = parse_args(remove_ros_args(args=sys.argv)[1:])
    rclpy.init(args=args)
    client = SimDetectBoxClient(
        cli_args.action_name,
        cli_args.height_command_topic,
        cli_args.height_state_topic,
    )
    try:
        if not cli_args.skip_crouch:
            if not client.wait_for_sim_crouch():
                print("仿真高度控制未就绪，流程结束。")
                return
            suggestion = client.request_crouch(cli_args.crouch_mode)
            if suggestion is None:
                print("无法获取下蹲建议，仿真流程结束。")
                return
            print(
                f"[{cli_args.crouch_mode}] 仿真建议下蹲量: "
                f"{suggestion.crouch_height:.3f} m "
                f"(当前 base_link={suggestion.base_link_height:.3f}m, "
                f"目标 base_link={suggestion.target_base_link_height:.3f}m)")
            if not cli_args.yes:
                answer = input("是否执行仿真下蹲？[Y/n] ").strip().lower()
                if answer not in ("", "y", "yes"):
                    print("已取消仿真下蹲，流程停止。")
                    return
            print(
                f"正在回到仿真站立高度 "
                f"{suggestion.standing_base_link_height:.3f}m ...")
            if not client.move_to_height(
                    suggestion.standing_base_link_height,
                    timeout_sec=cli_args.crouch_timeout):
                current = "未知" if client._height is None else f"{client._height:.3f}m"
                print(f"仿真站立高度未到位（当前高度: {current}），流程停止。")
                return
            print("正在模拟下蹲 ...")
            if not client.move_to_height(
                    suggestion.target_base_link_height,
                    timeout_sec=cli_args.crouch_timeout):
                current = "未知" if client._height is None else f"{client._height:.3f}m"
                print(f"仿真下蹲未到位（当前高度: {current}），流程停止。")
                return
            print(f"仿真下蹲到位: base_link={client._height:.3f}m")

        if client.detect():
            client.print_result()
            print("检测与 IK 已完成；请在姿态 GUI 中切换预抓取/抓取/抓取后/放置。")
        else:
            client.print_result()
            print("仿真检测失败。")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
