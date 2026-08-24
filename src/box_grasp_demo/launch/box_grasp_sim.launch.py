"""仿真 launch：假点云 + 检测 + IK + RViz，支持命名空间切换。"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_robot_description():
    path = Path(get_package_share_directory("walker_s2_description")) / "urdf/s2_v1/s2_v1.urdf"
    return path.read_text()


def generate_launch_description():
    package_share = FindPackageShare("box_grasp_demo")
    robot_share = FindPackageShare("walker_s2_description")
    config = PathJoinSubstitution([package_share, "config", "box_grasp_ros2.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "config", "box_grasp_sim.rviz"])
    robot_description = load_robot_description()
    urdf_path = PathJoinSubstitution([robot_share, "urdf/s2_v1", "s2_v1.urdf"])

    ns = LaunchConfiguration("ns")

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="/demo3"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("ik", default_value="false"),
        DeclareLaunchArgument("robot_base_z", default_value="0.904"),

        # ---- TF: sim_world → base_link ----
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_world_to_base",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0",
                "--z", LaunchConfiguration("robot_base_z"),
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "sim_world", "--child-frame-id", "base_link",
            ],
        ),

        # ---- Joint state publisher (IK → false) ----
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            arguments=[urdf_path],
            condition=UnlessCondition(LaunchConfiguration("ik")),
        ),

        # ---- Robot state publisher ----
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # ---- 假点云 ----
        Node(
            package="box_grasp_demo",
            executable="fake_box_scene_ros2.py",
            name="fake_box_scene",
            namespace=ns,
            output="screen",
            parameters=[{
                "output_topic": PathJoinSubstitution([ns, "pointcloud"]),
                "frame_id": "sim_world",
                "table_z": 0.90,
                "box_center": [0.30, 0.00, 1.00],
                "box_yaw_deg": 90.0,
                "noise_std": 0.0,
                "pose_command_topic": PathJoinSubstitution([ns, "pose_command"]),
                "gui": LaunchConfiguration("gui"),
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
                "joint_state_topic": "/joint_states",
                "so_path": "",
                "jit": False,
                "max_iters": 5,
                "pos_threshold": 1e-4,
                "ori_threshold": 1e-4,
            }],
        ),

        # ---- 箱体检测 ----
        Node(
            package="box_grasp_demo",
            executable="box_grasp_node_ros2.py",
            name="box_grasp_demo",
            namespace=ns,
            output="screen",
            parameters=[config, {
                "input_topic": PathJoinSubstitution([ns, "pointcloud"]),
                "target_frame": "sim_world",
                "camera_extrinsic_file": "",
                "support_height_min": 0.80,
                "support_height_max": 1.10,
                "placement_roi_enabled": False,
            }],
        ),

        # ---- 抓取验证 ----
        Node(
            package="box_grasp_demo",
            executable="grasp_validation_ros2.py",
            name="grasp_validation",
            namespace=ns,
            output="screen",
            parameters=[config, {
                "base_topic": PathJoinSubstitution([ns, "box_grasp_demo"]),
                "box_height": 0.20,
                "tool_contact_below_top": 0.05,
            }],
        ),

        # ---- RViz ----
        Node(
            package="rviz2",
            executable="rviz2",
            name="box_grasp_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
