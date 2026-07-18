import os

from launch import LaunchDescription                    # launch文件的描述类
from launch.actions import (
    LogInfo,
    OpaqueFunction,
)
from launch_ros.actions import Node                     # 节点启动的描述类
from launch import LaunchDescription                    # launch文件的描述类
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription        # 声明launch文件内使用的Argument类
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


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
        robot_desc = load_urdf(context)
        nodes = []
        # ============================================================
        # 1. robot_state_publisher —— URDF + /joint_states → TF
        #    发布 base_link → 各连杆 的 TF 关系 robot_state_publisher是ros2自带节点包
        # ============================================================
        print(f"启动加载机器人关节节点发布")
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
        print(f"启动发布机器人描述节点")
        nodes.append(Node(
            package='rosrobot_bringup_two',
            executable='publish_robot_description.py',
            name='robot_description_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
            }],
        ))

        print(f"启动Rviz2可视化")
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
        return nodes

    return LaunchDescription([
        OpaqueFunction(function=joy_launch_setup),
        LogInfo(
            msg='========================================================\n'
                '  系统启动 \n'
                '  🎮 DengFei  2026   文视科技  \n'
                '========================================================',
        ),
    ])
