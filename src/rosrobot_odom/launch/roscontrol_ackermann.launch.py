# ackermann_control.launch.py
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    LogInfo,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    controller_yaml_default = PathJoinSubstitution([
        FindPackageShare('rosrobot_odom'),
        'config',
        'ackermann_steering.yaml',
    ])
    controller_yaml = LaunchConfiguration('controller_yaml', default=controller_yaml_default)

    urdf_path = PathJoinSubstitution([
        FindPackageShare('rosrobot_description'),
        'urdf',
        'rosrobot.urdf',
    ])

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':  ParameterValue(
            Command(['cat ', urdf_path]),value_type=str
        ),
            'use_sim_time': use_sim_time,
        }],
    )

    # 桥接硬件接口已在 URDF 中声明，无需额外硬件驱动节点
    # 外部驱动应订阅 /hardware/rear_wheel_cmd 和 /hardware/front_steering_cmd，
    # 并发布 /hardware/joint_feedback。

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            controller_yaml,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    joint_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ackermann_steering_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    delayed_spawners = TimerAction(
        period=2.0,
        actions=[joint_broadcaster_spawner, ackermann_spawner],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('controller_yaml', default_value=controller_yaml_default),
        robot_state_pub_node,
        controller_manager_node,
        delayed_spawners,
        LogInfo(msg='阿克曼桥接硬件接口已启动，外部驱动请连接话题。'),
    ])