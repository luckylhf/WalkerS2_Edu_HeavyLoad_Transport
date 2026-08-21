#!/usr/bin/python3
"""桌面高度标定脚本（抓取桌/放置桌通用）。

把箱子放在要标定的桌面上，运行本脚本：
    1. 用户填写卷尺实测的桌面离地高度；
    2. 触发一次 DetectBox 检测，从结果反推视觉实测桌面 Z（base_link 系）
       = 箱子中心 Z - 箱高/2；
    3. 读取当前 base_link 离地高度（TF）；
    4. 按公式推算系统桌面高度 = base_link离地 + 视觉桌面Z + 余量，
       与卷尺值对比并给出建议填写值（table_height 或 place_table_height）。

用法：
    ros2 run box_grasp_demo calibrate_table_height.py
"""

import argparse
import time

import rclpy


# 头部低头目标（rad），与拍照位 head [0, -0.65] 一致。
HEAD_PITCH_TARGET = -0.65
HEAD_PITCH_TOLERANCE = 0.1


def ensure_head_down(client):
    """检查头部是否已低头；未低头则主动发送低头命令并等待到位。"""
    joint_state = client._wait_for_joint_state(timeout_sec=3.0)
    pitch = None
    if joint_state is not None:
        for name, pos in zip(joint_state.name, joint_state.position):
            if name == "head_pitch_joint":
                pitch = float(pos)
                break
    if pitch is None:
        print("无法读取 head_pitch_joint，跳过头部检查。")
        return True

    print(f"当前头部 pitch={pitch:.3f} rad "
          f"（低头目标 {HEAD_PITCH_TARGET}）")
    if pitch <= HEAD_PITCH_TARGET + HEAD_PITCH_TOLERANCE:
        print("头部已低头，无需调整。")
        return True

    print("头部未低头，主动发送低头命令 ...")
    body = {
        "component_names": ["head"],
        "goals": [[0.0, HEAD_PITCH_TARGET]],
        "mode": 1,
        "vel_scale": 0.2,
    }
    status, text = client._post_motion_with_retry(body, "头部低头")
    if status != 200:
        print(f"发送低头命令失败，HTTP {status or '无响应'}：{text}")
        return False
    print(f"已发送低头命令，HTTP {status}")

    start = time.monotonic()
    while time.monotonic() - start < 15.0:
        js = client._wait_for_joint_state(timeout_sec=0.25)
        if js is not None:
            for name, pos in zip(js.name, js.position):
                if name == "head_pitch_joint":
                    if float(pos) <= HEAD_PITCH_TARGET + HEAD_PITCH_TOLERANCE:
                        print("头部低头到位。")
                        return True
                    break
    print("等待头部低头超时，继续流程。")
    return True


def main():
    parser = argparse.ArgumentParser(description="桌面高度标定（抓取/放置通用）")
    parser.add_argument(
        "--action-name",
        default="/demo3/box_grasp_demo/detect_box",
        help="DetectBox action 名称（默认: /demo3/box_grasp_demo/detect_box）",
    )
    parser.add_argument(
        "--clearance",
        type=float,
        default=0.005,
        help="安全间隙余量 (m)，默认 0.005",
    )
    args = parser.parse_args()

    rclpy.init()

    # 与脚本同目录安装，ros2 run 时可通过 PYTHONPATH 导入。
    from trigger_detect_box import DetectBoxClient

    client = DetectBoxClient(args.action_name, debug=False)
    try:
        print("=" * 60)
        print("桌面高度标定（抓取桌/放置桌通用）")
        print("=" * 60)

        # 检查头部姿态：未低头时主动低头，保证相机能看到桌面。
        if not ensure_head_down(client):
            print("头部低头失败，流程结束。")
            return

        tape = input("请输入卷尺实测的桌面离地高度 (m): ").strip()
        try:
            tape_height = float(tape)
        except ValueError:
            print("输入不是有效数字，流程结束。")
            return

        input("请把箱子放在桌面中心，按回车开始检测 ...")

        if not client.send_goal():
            print("检测失败，流程结束。")
            return

        result = getattr(client, "_result", None)
        if result is None or not result.success:
            message = getattr(result, "message", "") if result else ""
            print(f"未检测到箱子（success=False）{message}，流程结束。")
            return

        # 箱子中心 Z - 箱高/2 = 视觉实测桌面 Z（base_link 系）。
        center_z = float(result.box_center[2])
        box_h = float(result.box_dimensions[2])
        table_z = center_z - box_h / 2.0

        base = client._get_base_link_height()
        if base is None:
            print("无法读取 base_link 离地高度（TF 查询失败），流程结束。")
            return

        recommended = base + table_z + args.clearance

        print("\n" + "=" * 60)
        print("标定结果")
        print("=" * 60)
        print(f"卷尺实测桌面高度:           {tape_height:.3f} m")
        print(f"视觉实测桌面Z(base_link系): {table_z:.3f} m")
        print(f"base_link 离地高度:         {base:.3f} m")
        print(f"推算公式: {base:.3f} + {table_z:.3f} + {args.clearance:.3f}")
        print(f"推算桌面高度(建议填写值):   {recommended:.3f} m")
        print(f"与卷尺差值:                 {recommended - tape_height:+.3f} m")
        print("=" * 60)
        print("请将推算值填写到 config/box_grasp_ros2.yaml：")
        print(f"  抓取桌 → table_height:      {recommended:.3f}")
        print(f"  放置桌 → place_table_height: {recommended:.3f}")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
