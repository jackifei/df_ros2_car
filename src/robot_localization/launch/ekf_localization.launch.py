#!/usr/bin/env python3
"""EKF 定位启动文件

启动内容：
  1. base_link -> imu_link 的静态 TF
     （robot_localization 需要把 IMU 数据变换到 base_link）
  2. robot_localization 的 ekf_node

注意：
  - 阿克曼控制器需要已经在运行，并发布
    /ackermann_steering_controller/odometry。
  - IMU 需要已经在运行，并发布 /imu/data，frame_id 为 imu_link。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 本配置包名（注意：不是提供 ekf_node 可执行文件的 robot_localization 包）
    pkg_share = get_package_share_directory('robot_localization_config')
    config_file = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    frequency = LaunchConfiguration('frequency', default='50.0')

    # ---------------------------------------------------------------------
    # base_link -> imu_link 静态 TF
    # 参数顺序：x y z yaw pitch roll parent_frame child_frame
    # 若 IMU 实际安装位置/朝向有偏移，请修改前 6 个数值。
    # ---------------------------------------------------------------------
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    # ---------------------------------------------------------------------
    # EKF 节点
    # ---------------------------------------------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'frequency': ParameterValue(frequency, value_type=float),
            },
        ],
        remappings=[],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) time instead of wall time',
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='50.0',
            description='EKF update frequency (Hz)',
        ),
        imu_tf,
        ekf_node,
    ])
