"""Offline demo using one recorded real S2 point-cloud frame, not a fake scene."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = FindPackageShare("box_grasp_demo")
    rviz_config = PathJoinSubstitution(
        [package_share, "config", "box_grasp_demo.rviz"])
    robot_urdf = (Path(get_package_share_directory("walker_s2_description")) /
                  "urdf" / "s2_v1" / "s2_v1.urdf").read_text()
    return LaunchDescription([
        DeclareLaunchArgument("frame_data_dir", description="Directory created by extract_current_bag.py"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("ik", default_value="true"),
        DeclareLaunchArgument("ns", default_value="/demo3"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="启动预抓/抓取模式切换 GUI"),
        DeclareLaunchArgument("sim_crouch_speed", default_value="0.05",
                              description="离线仿真 base_link 高度变化速度 (m/s)"),
        DeclareLaunchArgument("so_path", default_value="",
                              description="预编译 IK .so 路径（为空则使用 Python/CasADi 实时构建）"),
        Node(
            package="box_grasp_demo",
            executable="recorded_frame_publisher.py",
            name="recorded_frame_publisher",
            output="screen",
            parameters=[{
                "frame_data_dir": LaunchConfiguration("frame_data_dir"),
                "cloud_topic": "/sensor/camera/stereo/pointcloud/raw",
                "publish_rate": 2.0,
                # 手臂动态 TF 改由 IK 节点发布到 /tf，避免 RViz 模型手臂闪回录制姿态
                "exclude_arm_tf": True,
                "enable_sim_crouch": True,
                "sim_crouch_command_topic": "/demo3/sim/base_link_height_cmd",
                "sim_base_height_topic": "/demo3/sim/base_link_height",
                "sim_crouch_speed": LaunchConfiguration("sim_crouch_speed"),
            }],
        ),
        # Supplies /robot_description for RViz and consumes recorded joints.
        # Its generated TF is kept off /tf: the recorded frame publisher is
        # authoritative for the real robot pose.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_urdf}],
            remappings=[("/tf", "/recorded_model_tf")],
        ),
        # GroupAction 的 scoped=True 防止传给内部通用 launch 的 rviz=false
        # 覆盖本 launch 的同名 rviz 参数，否则下方专用 RViz 节点也会被禁用。
        GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [package_share, "launch", "box_grasp_demo.launch.py"])),
                launch_arguments={
                    "input_topic": "/sensor/camera/stereo/pointcloud/raw",
                    # 离线回放由下方 RViz 节点以 base_footprint 为固定坐标系启动，
                    # 这样仿真高度变化表现为机器人相对地面下降。
                    "rviz": "false",
                    "ik": LaunchConfiguration("ik"),
                    "ns": LaunchConfiguration("ns"),
                    "gui": LaunchConfiguration("gui"),
                    "publish_arm_tf": "true",
                    "so_path": LaunchConfiguration("so_path"),
                }.items(),
            ),
        ], scoped=True),
        Node(
            package="rviz2",
            executable="rviz2",
            name="box_grasp_rviz",
            arguments=["-d", rviz_config, "-f", "base_footprint"],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
