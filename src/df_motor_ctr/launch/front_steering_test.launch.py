import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch 描述。"""
    config_path_dir = os.path.join(
        get_package_share_directory('df_motor_ctr'),
        'config',
        'front_steering.yaml',
    )
    bridge_config = os.path.join(
        get_package_share_directory('df_joy_bridge'),
        'config',
        'bridge_config.yaml',
    )
    sbus_config = os.path.join(
        get_package_share_directory('df_joy_sbus'),
        'config',
        'joy_config.yaml',
    )
    config_REAR_path_dir = os.path.join(
        get_package_share_directory('df_motor_ctr'),
        'config',
        'motor_control.yaml',
    )
    return LaunchDescription([

        Node(
            package='df_joy_sbus',
            executable='sbus_joy',
            name='sbus_joy',    # 手柄节点
            output='screen',
            parameters=[sbus_config],
        ),
        Node(
            package='df_joy_bridge',
            executable='joy_bridge',
            name='joy_bridge',  # 手柄 桥接
            output='screen',
            parameters=[bridge_config],
        ),
        Node(
            package='df_motor_ctr',
            executable='motor_ctr',
            name='df_motor_ctr',  # 后轮 驱动
            output='screen',
            parameters=[config_REAR_path_dir],
        ),
        Node(
            package='df_motor_ctr',
            executable='front_steering_test',
            name='df_front_steering',  # 前轮转向
            output='screen',
            parameters=[config_path_dir],
        )
    ])

