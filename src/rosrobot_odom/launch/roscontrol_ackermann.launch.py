from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 找到配置文件路径（假设 config 文件夹在你的包内）
    pkg_name = 'your_package_name'  # 替换为你的包名
    config_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'ackermann_steering.yaml'
    ])

    # 启动 controller_manager 并加载控制器
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '-c', '/controller_manager'],
        parameters=[config_path],
        output='screen',
    )

    # 启动 joint_state_broadcaster（发布关节状态）
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # 可选：启动 robot_state_publisher（需要提供 URDF）
    # 如果你的机器人描述已经加载，可以省略
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # parameters=[{'robot_description': ...}]  # 从参数服务器获取
    )

    # 可选：启动 joint_state_publisher_gui 手动调节关节（测试用）
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    # )

    return LaunchDescription([
        controller_spawner,
        joint_state_broadcaster_spawner,
        robot_state_publisher,
        # joint_state_publisher_gui,
    ])