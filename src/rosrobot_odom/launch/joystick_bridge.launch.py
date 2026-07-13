#!/usr/bin/env python3
"""
Joystick Bridge 启动文件
=========================
启动 joystick_bridge_node，加载车辆参数。

Usage:
  ros2 launch joystick_bridge joystick_bridge.launch.py
  ros2 launch joystick_bridge joystick_bridge.launch.py wheelbase:=0.35 track_width:=0.22
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('joystick_bridge')

    # ---- 参数声明 ----
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase', default_value='0.30',
        description='Wheelbase L — front-rear axle distance (m)'
    )
    track_width_arg = DeclareLaunchArgument(
        'track_width', default_value='0.20',
        description='Track width W — rear left-right wheel distance (m)'
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='50.0',
        description='Odometry publish rate (Hz)'
    )

    # ---- Joystick Bridge 节点 ----
    bridge_node = Node(
        package='joystick_bridge',
        executable='joystick_bridge_node',
        name='joystick_bridge_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'vehicle_params.yaml'),
            {
                'wheelbase': LaunchConfiguration('wheelbase'),
                'track_width': LaunchConfiguration('track_width'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
    )

    return LaunchDescription([
        wheelbase_arg,
        track_width_arg,
        publish_rate_arg,
        bridge_node,
    ])
