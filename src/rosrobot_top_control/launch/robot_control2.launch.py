# system_full.launch.py
import os
from launch import LaunchDescription
from launch.actions import (
    LogInfo,
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    def joy_launch_setup(context):
        nodes = []

        # ============================================================
        # 1. 启动 ros2_control 框架（阿克曼控制器、关节广播器等）
        #    该文件内部会启动 robot_state_publisher、controller_manager 等
        # ============================================================
        print('[INFO] 加载 ros2_control 阿克曼控制框架')
        ackermann_launch_path = os.path.join(
            FindPackageShare('rosrobot_odom').perform(context),
            'launch',
            'rosrobot_ackermann.launch.py'
        )
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ackermann_launch_path),
            launch_arguments={
                'use_sim_time': 'false',
            }.items()
        ))

        # ============================================================
        # 2. robot_description_publisher —— URDF → /robot_description
        #    （保留，供 Rviz 等使用）
        # ============================================================
        print('[INFO] 启动 robot_description_publisher')
        robot_desc = load_urdf(context)
        nodes.append(Node(
            package='rosrobot_bringup_two',
            executable='publish_robot_description.py',
            name='robot_description_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ))

        # ----- 以下节点已被 ros2_control 替代，已移除 -----
        # - cmd_vel_to_joints_sync.py
        # - joystick_bridge_node (电子差速器)
        # - df_motor_ctr/motor_ctr (后轮驱动)
        # - df_motor_ctr/wheel_dir (前轮转向)
        # - rosrobot_odom/rosrobot_odom (里程计)
        # 现在所有运动控制与里程计均由 ackermann_steering_controller 统一处理

        # ============================================================
        # 3. 手柄控制及 twist 选择器（保持不变）
        # ============================================================
        existing_launch_path = os.path.join(
            FindPackageShare('rosrobot_top_control').perform(context),
            'launch', 'robot_twist_mux.launch.py'
        )
        print('[INFO] 启动手柄及 twist mux')
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(existing_launch_path)
        ))

        # ============================================================
        # 4. 启动 IMU（保持不变）
        # ============================================================
        print('[INFO] 启动 IMU')
        pkg_share = get_package_share_directory('dm_imu')
        config_path_imu = os.path.join(pkg_share, 'config', 'params.yaml')
        nodes.append(Node(
            package='dm_imu',
            executable='dm_imu_node',
            name='dm_imu',
            output='screen',
            parameters=[config_path_imu]
        ))

        # ============================================================
        # 5. 启动激光雷达（保持不变）
        # ============================================================
        lidar_config_path = os.path.join(
            get_package_share_directory('lidar_pkg'),
            'config', 'lidar_params.yaml'
        )
        nodes.append(Node(
            package='lidar_pkg',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[lidar_config_path]
        ))

        # ============================================================
        # 6. 雷达 Z 轴微调静态变换（保持不变）
        # ============================================================
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_z_rotation',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '-0.08',
                '--frame-id', 'lidar_Link',
                '--child-frame-id', 'lidar_Link_sub'
            ]
        ))

        # ============================================================
        # 7. RViz2（保持不变）
        # ============================================================
        print('[INFO] 启动 RViz2')
        rviz_config = PathJoinSubstitution([
            FindPackageShare('rosrobot_top_control'),
            'config', 'odom_display.rviz',
        ]).perform(context)
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}]
        ))

        # ============================================================
        # 8. USB相机（若需要，取消注释）
        # ============================================================
        # ... 相机节点代码 ...

        return nodes

    return LaunchDescription([
        OpaqueFunction(function=joy_launch_setup),
        LogInfo(msg='========================================================\n'
                    '  系统启动（ros2_control 阿克曼驱动版）\n'
                    '  🎮 DengFei  2026   文视科技  \n'
                    '========================================================'),
    ])