import os

from launch import LaunchDescription                    # launch文件的描述类
from launch.actions import (
    LogInfo,
    OpaqueFunction,
)
from launch_ros.actions import Node                     # 节点启动的描述类
from launch import LaunchDescription                    # launch文件的描述类
from launch.actions import DeclareLaunchArgument        # 声明launch文件内使用的Argument类
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法


def generate_launch_description():
    
    
    def joy_launch_setup(context):
        nodes = []

        # ============================================================
        # 1. 启动手柄控制节点
        # ============================================================
        nodes.append(Node(
            package='joy',
            executable='joy_node',
            # name='robot_state_publisher',
            output='screen',
        ))
        print("启动手柄控制节点")
        # ============================================================
        # 2. 启动手柄节点数据转换成twist
        # ============================================================
        pkg_name = 'rosrobot_twist_mux'
        config_path = os.path.join(
            get_package_share_directory(pkg_name),
            'config',
            'joy2twist.yaml'
        )
        nodes.append(Node(
            package='rosrobot_twist_mux',
            executable='joy2twist',
            # name='robot_state_publisher',
            output='screen',
            parameters=[config_path]
        ))
        print("启动手柄数据转换节点")
        
        # ============================================================
        # 3. 启动 nav和joy 控制分配器
        # ============================================================
        nodes.append(Node(
            package='rosrobot_twist_mux',
            executable='rosrobot_twist_mux',
            output='screen',
        ))
        print("启动控制分配器")
        

        return nodes

    return LaunchDescription([
        OpaqueFunction(function=joy_launch_setup),
        LogInfo(
            msg='========================================================\n'
                '  手柄控制，通过手柄控制小车前进和后退\n'
                '  手柄控制优先，无导航及控制时，小车停车\n'
                '========================================================',
        ),
    ])
