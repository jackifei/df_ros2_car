"""
nav_with_map.launch.py
启动 Nav2 地图服务器 + map_loader 节点，加载指定地图。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---------- 参数 ----------

    # 地图文件路径（默认为包内 maps/map.yaml）
    pkg_dir = get_package_share_directory('rosrobot_navigation')
    default_map_yaml = os.path.join(pkg_dir, 'maps', 'map_edited.yaml')

    config_dir = os.path.join(pkg_dir, 'config')
    nav_params = os.path.join(config_dir, 'nav2_params.yaml')


    # ---------- Launch 参数 ----------
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Full path to map yaml file'
    )

    # ---------- Nav2 Map Server ----------
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav_params,
            {'yaml_filename': LaunchConfiguration('map_yaml')}
        ]
    )

    # ---------- 生命周期管理器 (用于激活 map_server) ----------
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            nav_params,
            {
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )



    return LaunchDescription([
        map_yaml_arg,
        map_server_node,
        lifecycle_node,
    ])
