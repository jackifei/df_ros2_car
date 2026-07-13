#!/usr/bin/env python3
"""
EKF Localization 启动文件
==========================
启动 robot_localization 的 ekf_node，融合轮式里程计与IMU数据。

融合输入 (话题由外部节点提供):
  /odom       (轮式里程计) — vx + vyaw
  /imu/data   (6轴IMU)     — yaw + vyaw + ax + ay   [外部已有驱动]

输出:
  /odometry/filtered  (融合里程计)
  /tf                 (odom → base_link)

Usage:
  ros2 launch robot_localization_config ekf_localization.launch.py
  ros2 launch robot_localization_config ekf_localization.launch.py frequency:=100.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_localization_config')

    # ---- 参数声明 ----
    frequency_arg = DeclareLaunchArgument(
        'frequency', default_value='50.0',
        description='EKF update frequency (Hz) — must match IMU rate (external driver)'
    )
    two_d_mode_arg = DeclareLaunchArgument(
        'two_d_mode', default_value='true',
        description='2D mode for indoor planar navigation'
    )

    # ---- EKF 节点 ----
    # 话题使用默认名称，无需 remap:
    #   /odom      ← joystick_bridge 发布
    #   /imu/data  ← 外部 IMU 驱动发布
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf_params.yaml'),
            {
                'frequency': LaunchConfiguration('frequency'),
                'two_d_mode': LaunchConfiguration('two_d_mode'),
            }
        ],
    )

    return LaunchDescription([
        frequency_arg,
        two_d_mode_arg,
        ekf_node,
    ])
