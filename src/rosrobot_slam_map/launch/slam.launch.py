import os
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo,ExecuteProcess
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    # Rviz2 配置文件路径（使用 lidar_pkg 包中的 config/slam.rviz）
    rviz_config_file = os.path.join(
        get_package_share_directory('rosrobot_slam_map'),
        'config',
        'slam.rviz'
    )
    params_file = os.path.join(
        get_package_share_directory('rosrobot_slam_map'),
        'config',
        'slam_params.yaml')

    # slam_toolbox 生命周期节点
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_file],
        # 确保节点启动后不会因为缺少参数而报错
        # arguments=['--ros-args', '--log-level', 'info']
    )

    # 延时 2 秒后自动配置并激活生命周期节点
    # transition id: 1 = configure, 3 = activate
    auto_activate = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='Configuring slam_toolbox...'),
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/slam_toolbox/change_state',
                     'lifecycle_msgs/srv/ChangeState',
                     '{transition: {id: 1}}'],
                output='screen'
            ),
            LogInfo(msg='Waiting 1 second before activating...'),
            ExecuteProcess(
                cmd=['sleep', '2'],  # 确保 configure 完成
                output='screen'
            ),
            LogInfo(msg='Activating slam_toolbox...'),
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/slam_toolbox/change_state',
                     'lifecycle_msgs/srv/ChangeState',
                     '{transition: {id: 3}}'],
                output='screen'
            ),
        ]
    )


    return LaunchDescription([
            slam_toolbox_node,
            auto_activate,
        ])

        # Rviz2 可视化窗口
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen',
        # ),
