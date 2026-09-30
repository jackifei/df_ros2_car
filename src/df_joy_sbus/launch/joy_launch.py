import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成 SBUS 手柄驱动节点的 launch 描述。"""
    config = os.path.join(
        get_package_share_directory('df_joy_sbus'),
        'config',
        'joy_config.yaml',
    )
    return LaunchDescription([
        Node(
            package='df_joy_sbus',
            executable='sbus_joy',
            name='sbus_joy',
            output='screen',
            parameters=[config],
        ),
    ])
