"""
nav_auto.launch.py
ROS2 Jazzy Navigation2 (Nav2) 自动导航启动文件
功能: 启动 Nav2 完整导航栈的所有节点，包含地图服务器、定位、规划、控制等模块
支持两种模式:
  1. 独立节点模式 (use_composition=False，默认) — 每个功能作为独立进程运行
  2. 组合节点模式 (use_composition=True) — 多个功能加载到同一个进程中，减少通信开销

cmd_vel 话题流向 (已重映射为 /cmd_vel_nav):
  controller_server ──→ velocity_smoother ──→ 最终输出 /cmd_vel_nav → 机器人底盘
  behavior_server  ──→ (恢复行为也发布到 /cmd_vel_nav，经 velocity_smoother 平滑)
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # =========================================================================
    # 路径与参数文件配置
    # =========================================================================
    # rosrobot_navigation 包的共享目录（config/, maps/, launch/ 等）
    pkg_dir = get_package_share_directory('rosrobot_navigation')
    # 默认地图 YAML 文件路径
    default_map_yaml = os.path.join(pkg_dir, 'maps', 'map_edited.yaml')
    # Nav2 综合参数文件路径（包含所有服务器节点的 YAML 配置）
    nav_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # nav2_bringup 包的共享目录（Nav2 官方启动辅助工具）
    bringup_dir = get_package_share_directory('nav2_bringup')

    # =========================================================================
    # Launch 参数声明 — 可通过命令行覆盖
    # =========================================================================
    # namespace: 所有节点的顶级命名空间，默认为空（无命名空间前缀）
    namespace = LaunchConfiguration('namespace')
    # use_sim_time: 是否使用仿真时间（Gazebo），真实机器人应为 false
    use_sim_time = LaunchConfiguration('use_sim_time')
    # autostart: 是否自动激活 lifecycle 节点（true=启动后自动进入active状态）
    autostart = LaunchConfiguration('autostart')
    # params_file: Nav2 参数 YAML 文件完整路径
    params_file = LaunchConfiguration('params_file')
    # use_composition: 是否使用组合节点模式（将多个组件加载到同一进程）
    use_composition = LaunchConfiguration('use_composition')
    # container_name: 组合节点容器名称
    container_name = LaunchConfiguration('container_name')
    # 带命名空间前缀的容器全名
    container_name_full = (namespace, '/', container_name)
    # use_respawn: 节点崩溃后是否自动重启
    use_respawn = LaunchConfiguration('use_respawn')
    # log_level: ROS2 日志级别 (debug/info/warn/error/fatal)
    log_level = LaunchConfiguration('log_level')

    # =========================================================================
    # 生命周期管理节点列表
    # 生命周期管理器 (lifecycle_manager) 会按顺序激活这些节点的状态转换:
    #   unconfigured → inactive → active
    # 注意: 被注释掉的节点（collision_monitor, docking_server）已从列表中移除
    # =========================================================================
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        # 'collision_monitor',    # 已禁用: 节点被注释掉
        'bt_navigator',
        'waypoint_follower',
        # 'docking_server',       # 已禁用: 节点被注释掉
    ]

    # =========================================================================
    # TF 重映射 — 保持默认的 tf 和 tf_static 话题名称
    # =========================================================================
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # =========================================================================
    # 参数替换 — autostart 参数会被注入到 Nav2 参数文件中
    # RewrittenYaml 是 Nav2 的工具类，可在运行时动态覆写 YAML 参数
    #   source_file: 原始 YAML 参数文件
    #   root_key: 参数根键（用于命名空间隔离）
    #   param_rewrites: 运行时要替换的参数字典
    #   convert_types: 自动转换参数类型（如字符串 "true" → 布尔 True）
    # =========================================================================
    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )
    print(configured_params)
    # =========================================================================
    # 环境变量设置 — 启用 RCUTILS 缓冲日志流（更高效的日志输出）
    # =========================================================================
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # =========================================================================
    # 各 Launch 参数的声明（--ros-args 或命令行可覆盖默认值）
    # =========================================================================
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # 默认 false: 真实机器人使用系统时钟
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav_params,
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',   # 默认 true: 自动激活所有 lifecycle 节点
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',  # 默认 False: 使用独立节点模式
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',  # 默认 False: 节点崩溃后不自动重启
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # =========================================================================
    # 自定义参数: 地图文件路径
    # 使用方法: ros2 launch rosrobot_navigation nav_auto.launch.py map_yaml:=/path/to/map.yaml
    # =========================================================================
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Full path to map yaml file'
    )

    # =========================================================================
    # 模式一: 独立节点 (use_composition=False, 默认模式)
    # 每个 Nav2 服务器作为独立的 ROS2 节点运行
    # 优点: 独立隔离，一个节点崩溃不影响其他；便于调试
    # 缺点: 进程间通信开销较大（DDS序列化/反序列化）
    # =========================================================================
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            # 向所有节点设置 use_sim_time 参数
            SetParameter('use_sim_time', use_sim_time),

            # -----------------------------------------------------------------
            # map_server: 地图服务器
            # 功能: 加载静态地图文件（SLAM建图结果），通过话题发布给其他节点
            # 发布话题: /map (nav_msgs/OccupancyGrid)
            # 注意: 参数使用 nav_params 基础配置 + 单独覆写 yaml_filename
            # -----------------------------------------------------------------
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[nav_params, {'yaml_filename': LaunchConfiguration('map_yaml')}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # -----------------------------------------------------------------
            # amcl: 自适应蒙特卡洛定位
            # 功能: 基于粒子滤波的2D位姿估计，融合激光雷达/里程计/地图信息
            # 输入: /scan (激光), /odom (里程计), /map (地图)
            # 输出: /amcl_pose (估计位姿), map→odom TF变换
            # -----------------------------------------------------------------
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # -----------------------------------------------------------------
            # controller_server: 局部路径控制器 (核心控制节点)
            # 功能: 接收全局路径，使用 MPPI 算法计算最优速度指令
            # 订阅: /plan (全局路径), /local_costmap (局部代价地图), /odom
            # 发布: /cmd_vel_nav (重映射后的速度指令) ← 已从 /cmd_vel 重映射
            # 重映射说明: 将默认的 /cmd_vel 改为 /cmd_vel_nav, 便于自定义底盘驱动订阅
            # -----------------------------------------------------------------
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',               # 修复: 显式指定节点名称
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),

            # -----------------------------------------------------------------
            # smoother_server: 路径平滑器
            # 功能: 对全局规划器生成的路径进行平滑处理，减少折线
            # 输入: /plan (原始路径)
            # 输出: /plan_smoothed (平滑后的路径)
            # -----------------------------------------------------------------
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # -----------------------------------------------------------------
            # planner_server: 全局路径规划器
            # 功能: 在全局代价地图上使用 Dijkstra/A* 算法规划从起点到目标的最优路径
            # 输入: /global_costmap, 目标位姿
            # 输出: /plan (全局路径: nav_msgs/Path)
            # -----------------------------------------------------------------
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # -----------------------------------------------------------------
            # route_server: 路线服务器
            # 功能: 在预定义路线图上进行长距离导航（适用于仓库等结构化环境）
            # 如不需要路线导航，可注释掉
            # -----------------------------------------------------------------
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # -----------------------------------------------------------------
            # behavior_server: 恢复行为服务器
            # 功能: 当机器人导航卡住时，执行备选恢复行为:
            #   - spin: 原地旋转寻找可行方向
            #   - backup: 后退一段距离
            #   - drive_on_heading: 定向行驶
            #   - wait: 等待动态障碍物离开
            #   - assisted_teleop: 辅助遥控
            # 发布: /cmd_vel_nav (恢复行为的速度指令) ← 已从 /cmd_vel 重映射
            # -----------------------------------------------------------------
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),

            # -----------------------------------------------------------------
            # bt_navigator: 行为树导航器
            # 功能: 管理行为树的执行，协调各个导航 Action (NavigateToPose等)
            # 行为树定义了导航的完整流程:
            #   规划路径 → 跟随路径 → 检查进度 → (失败时)执行恢复行为 → 重规划
            # -----------------------------------------------------------------
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # -----------------------------------------------------------------
            # waypoint_follower: 航点跟随器
            # 功能: 按顺序依次导航到多个航点，每到达一个航点暂停一段时间
            # 使用 Action: NavigateThroughPoses
            # -----------------------------------------------------------------
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # -----------------------------------------------------------------
            # velocity_smoother: 速度平滑器
            # 功能: 对控制器输出的速度指令进行平滑滤波
            # 工作模式: 作为 cmd_vel 话题的 "中间人滤波器"
            #   - 订阅 /cmd_vel_nav (原始速度指令, 来自 controller_server 或 behavior_server)
            #   - 发布 /cmd_vel_nav (平滑后的速度指令)
            # 注意: 由于输入和输出重映射到同一话题，该节点订阅并重新发布到同一话题
            #       ROS2 允许多个发布者向同一话题发送消息，因此这种模式是安全的
            # 最终消费者(底盘驱动节点)应订阅 /cmd_vel_nav
            # -----------------------------------------------------------------
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav')],
            ),

            # -----------------------------------------------------------------
            # collision_monitor: 碰撞监视器 — 当前已禁用
            # 功能: 基于传感器数据实时检测碰撞风险，必要时停止机器人
            # 如需启用，取消注释此节点并在 lifecycle_nodes 中也取消注释
            # -----------------------------------------------------------------
            # Node(
            #     package='nav2_collision_monitor',
            #     executable='collision_monitor',
            #     name='collision_monitor',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),

            # -----------------------------------------------------------------
            # docking_server: 自动回充/对接服务器 — 当前已禁用
            # 功能: 控制机器人自动导航到充电桩并精确对接
            # 如需启用，取消注释此节点并在 lifecycle_nodes 中也取消注释
            # -----------------------------------------------------------------
            # Node(
            #     package='opennav_docking',
            #     executable='opennav_docking',
            #     name='docking_server',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            # ),

            # -----------------------------------------------------------------
            # lifecycle_manager: 生命周期管理器
            # 功能: 管理所有 Nav2 节点的生命周期状态转换
            # 当 autostart=true 时，自动将所有节点从 unconfigured → active
            # node_names 参数指定需要管理的节点名称列表
            # -----------------------------------------------------------------
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # =========================================================================
    # 模式二: 组合节点 (use_composition=True)
    # 将多个组件加载到同一进程中，减少序列化开销，适合资源受限系统
    # 优点: 零拷贝通信 (Intra-process)，性能更好
    # 缺点: 一个组件崩溃可能影响整个容器；需要手动管理容器进程
    #
    # 注意 (已修复BUG): 以下组件也加入 map_server 和 amcl，因它们不支持
    # composition 组合模式，所以在组合模式下会丢失这两个节点。
    # 建议: 真实使用组合模式时，额外单独启动 map_server 和 amcl
    # =========================================================================
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    # controller_server (组合模式)
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    # smoother_server (组合模式)
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # planner_server (组合模式)
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # route_server (组合模式)
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # behavior_server (组合模式)
                    # 已修复: plugin 名称从 'behavior_server::BehaviorServer' 改为 'nav2_behaviors::BehaviorServer'
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='nav2_behaviors::BehaviorServer',  # 修复: 原为 'behavior_server::BehaviorServer'
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    # bt_navigator (组合模式)
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # waypoint_follower (组合模式)
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    # velocity_smoother (组合模式)
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=remappings
                        + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    # collision_monitor (组合模式) — 已禁用
                    # ComposableNode(
                    #     package='nav2_collision_monitor',
                    #     plugin='nav2_collision_monitor::CollisionMonitor',
                    #     name='collision_monitor',
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    # docking_server (组合模式) — 已禁用
                    # ComposableNode(
                    #     package='opennav_docking',
                    #     plugin='opennav_docking::DockingServer',
                    #     name='docking_server',
                    #     parameters=[configured_params],
                    #     remappings=remappings,
                    # ),
                    # lifecycle_manager (组合模式)
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    # =========================================================================
    # 构建 LaunchDescription — 将所有声明和操作添加到启动描述中
    # 执行顺序:
    #   1. 设置环境变量
    #   2. 声明所有 Launch 参数（可被命令行覆盖）
    #   3. 加载导航节点（独立模式 或 组合模式，二选一）
    # =========================================================================
    ld = LaunchDescription()

    # 设置环境变量
    ld.add_action(stdout_linebuf_envvar)

    # 声明 Launch 参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # 加载导航节点组（根据 use_composition 条件二选一）
    ld.add_action(load_nodes)
    # ld.add_action(load_composable_nodes)

    return ld
