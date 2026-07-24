#!/usr/bin/env python3
"""
全系统启动文件 (Bringup)
=========================
一键启动阿克曼小车所有节点。

启动的 ROS2 节点:
  1. joystick_bridge        — 手柄→阿克曼差速+里程计, 50Hz
  2. chassis_driver         — 底盘电机驱动 (可选，默认关闭)
  3. ekf_localization       — EKF融合定位, 50Hz

外部依赖 (用户已有，需提前启动):
  - IMU 驱动              — 发布 /imu/data @50Hz, frame_id="imu_link"
  - 手柄驱动              — 发布 /cmd_vel (已实现)

启动顺序:
  先启动 joystick_bridge (提供 /odom)，再启动 EKF (订阅 /odom + /imu/data)。

Usage:
  ros2 launch bringup.launch.py
  ros2 launch bringup.launch.py wheelbase:=0.35 track_width:=0.22
  ros2 launch bringup.launch.py use_ekf:=false      # 不使用EKF融合
  ros2 launch bringup.launch.py use_chassis:=true   # 启用电机驱动
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # =========================================================================
    # 参数声明
    # =========================================================================
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase', default_value='0.30',
        description='Wheelbase L (m)'
    )
    track_width_arg = DeclareLaunchArgument(
        'track_width', default_value='0.20',
        description='Track width W (m)'
    )
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate', default_value='50.0',
        description='System base frequency — Odom + EKF (Hz)'
    )
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Enable EKF fusion via robot_localization_config'
    )
    use_chassis_arg = DeclareLaunchArgument(
        'use_chassis', default_value='false',
        description='Enable chassis driver node (only if motors connected)'
    )

    # =========================================================================
    # 启动动作
    # =========================================================================

    # 1. Joystick Bridge — 手柄→差速→里程计 (50Hz)
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('joystick_bridge'),
                'launch',
                'joystick_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'wheelbase': LaunchConfiguration('wheelbase'),
            'track_width': LaunchConfiguration('track_width'),
            'publish_rate': LaunchConfiguration('sample_rate'),
        }.items(),
    )

    # 2. Chassis Driver — 底盘电机驱动（默认关闭）
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_interface'),
                'launch',
                'chassis_driver.launch.py'
            ])
        ]),
    )

    # 3. EKF Localization — 传感器融合 (50Hz)
    #    需要 /odom (由 joystick_bridge 发布) 和 /imu/data (由外部 IMU 驱动发布)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_localization_config'),
                'launch',
                'ekf_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'frequency': LaunchConfiguration('sample_rate'),
        }.items(),
    )

    # =========================================================================
    # 时序编排
    # =========================================================================

    ld = LaunchDescription()

    # 参数
    ld.add_action(wheelbase_arg)
    ld.add_action(track_width_arg)
    ld.add_action(sample_rate_arg)
    ld.add_action(use_ekf_arg)
    ld.add_action(use_chassis_arg)

    # 启动日志
    ld.add_action(LogInfo(
        msg=['Bringup starting @ ', LaunchConfiguration('sample_rate'),
             ' Hz | IMU: external driver (must be running)']
    ))

    # T=0s: Joystick Bridge
    ld.add_action(bridge_launch)

    # T=1s: Chassis Driver (延迟启动，等待 /wheel_speeds 话题就绪)
    ld.add_action(TimerAction(
        period=1.0,
        actions=[chassis_launch],
    ))

    # T=2s: EKF (最后启动，确保 /odom 和 /imu/data 话题已就绪)
    ld.add_action(TimerAction(
        period=2.0,
        actions=[ekf_launch],
    ))

    # 完成日志
    ld.add_action(TimerAction(
        period=3.0,
        actions=[LogInfo(
            msg=['Bringup complete! Bridge(50Hz) + EKF(50Hz) | '
                 'Ensure external IMU driver is publishing /imu/data']
        )],
    ))

    return ld
