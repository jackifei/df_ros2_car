from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Rviz2 配置文件路径（使用 lidar_pkg 包中的 config/slam.rviz）
    rviz_config_file = os.path.join(
        get_package_share_directory('rosrobot_slam_map'),
        'config',
        'slam.rviz'
    )
    params_file = os.path.join(
        get_package_share_directory('rosrobot_slam_map'),
        'config',
        'slam_params.yaml')

    return LaunchDescription([
        # SLAM Toolbox 节点
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file]
        ),

        # Rviz2 可视化窗口
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen',
        # ),
    ])