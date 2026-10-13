import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch 描述。"""
    config_path_dir = os.path.join(
        get_package_share_directory('df_motor_ctr'),
        'config',
        'joy_config.yaml',
    )
    return LaunchDescription([
        Node(
            package='df_motor_ctr',
            executable='wheel_dir',
            name='sbus_joy',
            output='screen',
            parameters=[config_path_dir],
        )
    ])

