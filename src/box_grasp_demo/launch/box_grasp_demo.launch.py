"""
真机 launch：点云检测 + IK 求解（不驱动机器人）
所有话题带 /demo1/ 前缀，与真机控制器完全隔离。
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = FindPackageShare("box_grasp_demo")
    robot_share = FindPackageShare("walker_s2_description")
    config = PathJoinSubstitution([package_share, "config", "box_grasp_ros2.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "config", "box_grasp_demo.rviz"])
    # Use the original S2 v1 model that matches the robot-published TF.
    urdf_path = PathJoinSubstitution([robot_share, "urdf", "s2_v1", "s2_v1.urdf"])

    ns = LaunchConfiguration("ns")

    # 从环境变量读取 IK Python 路径（run.sh 中设置）
    # 追加到现有 PYTHONPATH，保留 ROS2 路径
    ik_pythonpath = os.environ.get("IK_PYTHONPATH", "")
    if ik_pythonpath:
        current_pythonpath = os.environ.get("PYTHONPATH", "")
        ik_pythonpath = f"{ik_pythonpath}:{current_pythonpath}" if current_pythonpath else ik_pythonpath

    # IK 环境里的原生库（casadi/pinocchio）依赖 conda 自带的 libstdc++，
    # 通过 LD_LIBRARY_PATH 让 IK 节点优先加载 conda 的 lib 目录。
    ik_ld_library_path = os.environ.get("IK_LD_LIBRARY_PATH", "")
    if not ik_ld_library_path:
        conda_prefix = os.environ.get("CONDA_PREFIX", "")
        if conda_prefix:
            ik_ld_library_path = os.path.join(conda_prefix, "lib")
    if ik_ld_library_path:
        current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
        ik_ld_library_path = (
            f"{ik_ld_library_path}:{current_ld_library_path}"
            if current_ld_library_path
            else ik_ld_library_path
        )

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="/demo3",
                              description="命名空间前缀，隔离真机控制话题"),
        DeclareLaunchArgument("input_topic",
                              default_value="/sensor/camera/stereo/pointcloud/raw",
                              description="相机点云输入话题（不添加 ns 前缀）"),
        DeclareLaunchArgument("camera_extrinsic_file", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("ik", default_value="true",
                              description="启用 IK 求解"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="启动预抓/抓取模式切换 GUI"),
        DeclareLaunchArgument("publish_arm_tf", default_value="false",
                              description="IK 结果直接发布手臂 TF 到 /tf，驱动离线 RViz 模型"),
        DeclareLaunchArgument("so_path", default_value="",
                              description="预编译 IK .so 路径（为空则使用 Python/CasADi 实时构建）"),

        # ---- 关节状态桥接（/mc → /joint_states，含踝关节串并转换） ----
        Node(
            package="box_grasp_demo",
            executable="joint_state_bridge.py",
            name="joint_state_bridge",
            output="screen",
        ),

        # ---- 箱体检测 + 抓取位姿 ----
        Node(
            package="box_grasp_demo",
            executable="box_grasp_node_ros2.py",
            name="box_grasp_demo",
            namespace=ns,
            output="screen",
            # 检测参数全部由 YAML 配置（config/box_grasp_ros2.yaml，顶层
            # key 为 /** 通配符）提供，此处只传 launch 特有的运行时参数。
            parameters=[config, {
                "input_topic": LaunchConfiguration("input_topic"),
                "camera_extrinsic_file": LaunchConfiguration("camera_extrinsic_file"),
            }],
        ),

        # ---- Pinocchio + CasADi 双臂 IK ----
        Node(
            package="box_grasp_demo",
            executable="arm_ik_pinocchio.py",
            name="arm_ik",
            namespace=ns,
            output="screen",
            condition=IfCondition(LaunchConfiguration("ik")),
            parameters=[config, {
                "urdf_file": urdf_path,
                "base_frame": "base_link",
                "target_topic": PathJoinSubstitution([ns, "box_grasp_demo"]),
                "pose_command_topic": PathJoinSubstitution([ns, "pose_command"]),
                "initial_mode": "pregrasp",
                "publish_rate": 10.0,
                "joint_state_topic": PathJoinSubstitution([ns, "joint_states"]),
                "publish_arm_tf": LaunchConfiguration("publish_arm_tf"),
                "arm_tf_topic": "/tf",
                "so_path": LaunchConfiguration("so_path"),
                "jit": False,
                "max_iters": 3,
                "pos_threshold": 1e-3,
                "ori_threshold": 1e-3,
            }],
            additional_env={
                **({"PYTHONPATH": ik_pythonpath} if ik_pythonpath else {}),
                **({"LD_LIBRARY_PATH": ik_ld_library_path} if ik_ld_library_path else {}),
            },
        ),

        # ---- 预抓/抓取模式切换 GUI（可选） ----
        Node(
            package="box_grasp_demo",
            executable="arm_pose_gui.py",
            name="arm_pose_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
            parameters=[{
                "pose_command_topic": PathJoinSubstitution([ns, "pose_command"]),
            }],
        ),

        # ---- RViz（可选） ----
        Node(
            package="rviz2",
            executable="rviz2",
            name="box_grasp_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
