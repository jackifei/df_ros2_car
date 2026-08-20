from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 配置文件路径（假设 config 目录在包的根目录下）
    config_file = os.path.join(
        get_package_share_directory('your_package_name'),  # 替换为你的包名
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                # 如果需要重映射话题，可以在这里添加
                # ('/odometry/filtered', '/filtered_odom')
            ]
        )
    ])