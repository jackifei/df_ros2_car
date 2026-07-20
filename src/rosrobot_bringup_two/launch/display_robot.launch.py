#!/usr/bin/env python3
"""
display_robot.launch.py — RViz2 小车模型显示 + 同步后驱键盘控制 + 里程计

控制方案：同步后驱 + 前轮转向（Ackermann 风格）
  - 左右后轮同速驱动（忽略轮速差）
  - 转弯仅靠前轮转向角
  - /cmd_vel linear.x → 后轮转速
  - /cmd_vel angular.z → 前轮转向角

数据流:
  键盘 → /cmd_vel → cmd_vel_to_joints_sync → /joint_states
       → robot_state_publisher → TF (base_link → 各连杆)
       → ackermann_odometry → /odom + odom→base_link TF
       → RViz2 显示（小车模型 + 里程计轨迹）

用法:
  ros2 launch rosrobot_bringup_two display_robot.launch.py
  然后在另一终端:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  按键: i=前进  k=停止  j=左转  l=右转  ,=后退
"""
#  1231313213
import os

from launch import LaunchDescription
from launch.actions import (
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_urdf(context) -> str:
    """从 rosrobot_description 包读取 URDF 文件，展开 $(find ...) 宏。"""
    pkg_share = FindPackageShare('rosrobot_description').perform(context)
    urdf_path = os.path.join(pkg_share, 'urdf', 'rosrobot.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 展开 $(find rosrobot_description) → 实际安装路径
    content = content.replace('$(find rosrobot_description)', pkg_share)

    print(f'[INFO] URDF 已加载+展开: {urdf_path} ({len(content)} 字符)')
    return content


def generate_launch_description():

    def launch_setup(context):
        robot_desc = load_urdf(context)

        nodes = []

        # ============================================================
        # 1. robot_state_publisher —— URDF + /joint_states → TF
        #    发布 base_link → 各连杆 的 TF 关系
        # ============================================================
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': False,
            }],
        ))

        # ============================================================
        # 2. robot_description_publisher —— URDF → /robot_description
        #    以 Transient Local QoS 发布，供 RViz RobotModel 显示 STL 模型
        # 使用机器人关机姿态发布节点，对其进行发布
        # ============================================================
        nodes.append(Node(
            package='rosrobot_bringup_two',
            executable='publish_robot_description.py',
            name='robot_description_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
            }],
        ))

        # ============================================================
        # 3. cmd_vel_to_joints_sync —— /cmd_vel → /joint_states
        #    同步后驱方案核心：
        #      linear.x  → 左右后轮同速 (lh_joint, rh_joint)
        #      angular.z → 前轮转向角度 (lq_joint, rq_joint)
        #    两通道完全独立：油门与方向盘互不干涉
        # ============================================================
        nodes.append(Node(
            package='rosrobot_bringup_two',
            executable='cmd_vel_to_joints_sync.py',
            name='cmd_vel_to_joints_sync',
            output='screen',
            parameters=[{
                'wheel_radius': 0.062,
                'max_steering_angle': 0.7,
                'steering_scale': 1.0,
                'publish_rate': 50.0,
            }],
        ))

        # ============================================================
        # 4. ackermann_odometry —— /joint_states → /odom + odom→base_link TF
        #    自行车模型 (Bicycle Model):
        #      v = ω_wheel × wheel_radius
        #      ω = v × tan(δ) / wheelbase
        #    从 lh_joint 转速 + lq_joint 转向角推算位姿变化
        # ============================================================
        nodes.append(Node(
            package='rosrobot_bringup_two',
            executable='ackermann_odometry.py',
            name='ackermann_odometry',
            output='screen',
            parameters=[{
                'wheel_radius': 0.062,
                'wheelbase': 0.306,
            }],
        ))

        # ============================================================
        # 5. RViz2 —— 可视化
        #    使用 odom_display.rviz 配置：
        #      Fixed Frame: odom（里程计坐标系固定，小车在其中移动）
        #      显示: Grid + TF + RobotModel + Odometry（绿色轨迹线）
        # ============================================================
        rviz_config = PathJoinSubstitution([
            FindPackageShare('rosrobot_bringup_two'),
            'config',
            'odom_display.rviz',
        ]).perform(context)

        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': False,
            }],
        ))
        # ============================================================
        # 6. lidar 启动雷达扫描 —— 可视化
        #    获取scan数据：
        # ============================================================
        lidar_config_path = os.path.join(
        get_package_share_directory('lidar_pkg'),
        'config',
        'lidar_params.yaml'
    )
        nodes.append(Node(
            package='lidar_pkg',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[lidar_config_path]
            ))
        # ============================================================
        # 7. IMU 启动IMU —— 可视化
        #    获取/imu/data    /imu/pose   /imu/rpy数据：
        # ============================================================
        pkg_share = get_package_share_directory('dm_imu')
        params_file = os.path.join(pkg_share, 'config', 'params.yaml')

        nodes.append(Node(
            package='dm_imu',
            executable='dm_imu_node',
            name='dm_imu',
            output='screen',
            parameters=[params_file]
            ))
        # ============================================================
        # 8. 启动USB相机 —— 可视化
        #    获取/image_raw数据：
        # ============================================================
        # pkg_share = get_package_share_directory('dm_imu')
        # params_file = os.path.join(pkg_share, 'config', 'params.yaml')

        nodes.append(Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            # parameters=[params_file]
            ))



        return nodes

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
        LogInfo(
            msg='========================================================\n'
                '  同步后驱 + 前轮转向 控制方案\n'
                '  后轮同速驱动 | 前轮独立转向 | 自行车模型里程计\n'
                '  --------------------------------------------------\n'
                '  请在另一终端中运行键盘控制:\n'
                '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
                '  按键: i=前进  k=停止  j=左转  l=右转  ,=后退\n'
                '========================================================',
        ),
    ])
