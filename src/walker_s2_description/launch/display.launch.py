from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = FindPackageShare('walker_s2_description')
    
    urdf_path = PathJoinSubstitution([
        pkg_path,
        'urdf',
        's2_v1/s2_v1.urdf'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_path,
            description='URDF file path'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                pkg_path,
                'config',
                'rviz',
                'view_robot.rviz'
            ]),
            description='RViz config file path'
        ),

        # URDF验证节点
        ExecuteProcess(
            cmd=['check_urdf', LaunchConfiguration('urdf_file')],
            output='screen',
            name='urdf_validation'
        ),

        # 机器人状态发布节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', LaunchConfiguration('urdf_file')]),
                    value_type=str
                ),
                'publish_frequency': 50.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=[
                ('/joint_states', 'joint_states'),
                ('/robot_description', 'robot_description')
            ]
        ),

        # 关节状态发布节点（使用GUI版本）
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rate': 50
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
