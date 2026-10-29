import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch 描述。"""
    config_path_dir = os.path.join(
        get_package_share_directory('df_motor_ctr'),
        'config',
        'motor_control.yaml',
    )
    return LaunchDescription([
        Node(
            package='df_motor_ctr',
            executable='motor_ctr',
            name='df_motor_ctr',  # 节点名称
            output='screen',
            parameters=[config_path_dir],
        )
    ])

