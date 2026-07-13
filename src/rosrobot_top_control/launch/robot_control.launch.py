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


def generate_launch_description():

    def joy_launch_setup(context):
        nodes = []

        # 1. 获取并包含已有的 launch 文件
        # FindPackageShare 会自动定位指定包的安装路径
        existing_launch_path = os.path.join(
            FindPackageShare(package='rosrobot_top_control').find('rosrobot_top_control'),
            'launch', 
            'robot_twist_mux.launch.py'
        )
        print(f"{existing_launch_path}")
        include_twist_mux = IncludeLaunchDescription(PythonLaunchDescriptionSource(existing_launch_path))
        # ============================================================
        # 1. launch 启动   启动twist选择器，并且启动手柄节点，
        # ============================================================
        nodes.append(include_twist_mux)

        # ============================================================
        # 2. 启动阿克曼电子差速器，用来计算后轮差速及前轮转向
        # ============================================================
        nodes.append(Node(
            package='rosrobot_odom',
            executable='joystick_bridge_node',
            output='screen',
        ))

        # # ============================================================
        # # 2. 启动电机驱动及转向 计算节点
        # # ============================================================
        # nodes.append(Node(
        #     package='df_motor_ctr',
        #     executable='motor_ctr',
        #     output='screen',
        # ))

        # # ============================================================
        # # 3. 启动电机启动节点，启动转向节点
        # # ============================================================
        # nodes.append(Node(
        #             package='df_motor_ctr',
        #             executable='wheel_dir',
        #             output='screen',
        #         ))


        return nodes

    return LaunchDescription([
        OpaqueFunction(function=joy_launch_setup),
        LogInfo(
            msg='========================================================\n'
                '  系统启动\n'
                '========================================================',
        ),
    ])
