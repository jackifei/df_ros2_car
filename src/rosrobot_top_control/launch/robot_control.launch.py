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
        # 1. launch 启动   启动twist选择器，并且启动手柄节点，
        # ============================================================
        existing_launch_path = os.path.join(
            FindPackageShare(package='rosrobot_top_control').find('rosrobot_top_control'),
            'launch', 
            'robot_twist_mux.launch.py'
        )
        print(f"启动 手柄launch 文件")
        include_twist_mux = IncludeLaunchDescription(PythonLaunchDescriptionSource(existing_launch_path))
        
        nodes.append(include_twist_mux)
        # ============================================================
        # 2. 启动IMU  ACM0
        # ============================================================
        print(f"启动IMU")
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
        # 3. 启动阿克曼电子差速器，用来计算后轮差速及前轮转向
        # ============================================================
        print(f"启动电子差速器")
        nodes.append(Node(
            package='rosrobot_odom',
            executable='joystick_bridge_node',
            output='screen'
        ))

        # ============================================================
        # 4. 启动电机驱动计算节点
        # ============================================================
        print(f"启动后轮电机驱动")
        config_path_motor = os.path.join(
            get_package_share_directory('df_motor_ctr'),
            'config',
            'motor_control.yaml'
        )
        nodes.append(Node(
            package='df_motor_ctr',
            executable='motor_ctr',
            output='screen',
            parameters=[config_path_motor]
        ))
        # ============================================================
        # 5. 启动前轮转向驱动
        # ============================================================
        print(f"启动前轮转向驱动")
        config_path_dir = os.path.join(
            get_package_share_directory('df_motor_ctr'),
            'config',
            'wheel_dir_pwm.yaml'
        )
        nodes.append(Node(
            package='df_motor_ctr',
            executable='wheel_dir',
            output='screen',
            parameters=[config_path_dir]
        ))

        # ============================================================
        # 6. 启动里程计节点
        # ============================================================
        print(f"启动里程计计算")
        config_path_odom = os.path.join(
            get_package_share_directory('rosrobot_odom'),
            'config',
            'wheel_odom_fusion_node.yaml'
        )
        nodes.append(Node(
            package='rosrobot_odom',
            executable='rosrobot_odom',
            output='screen',
            parameters=[config_path_odom]
        ))
        # ============================================================
        # 7. lidar 启动雷达扫描 —— 可视化
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
        # 8. 添加Z轴180度旋转的静态坐标变换
        # ============================================================
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_z_rotation',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '3.14159265',  # 绕Z轴旋转180度
                '--frame-id', 'lidar_Link',    # 要旋转的link名称id
                '--child-frame-id', 'lidar_Link_sub'   # 变换后的id，然后雷达的发布节点需要绑定此sub节点
                ]
            ))

        # ============================================================
        # 8. RViz2 —— 可视化
        #    使用 odom_display.rviz 配置：
        #      Fixed Frame: odom（里程计坐标系固定，小车在其中移动）
        #      显示: Grid + TF + RobotModel + Odometry（绿色轨迹线）
        # ============================================================
        print(f"启动Rviz2可视化")
        rviz_config = PathJoinSubstitution([
            FindPackageShare('rosrobot_top_control'),
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
        # 10. 启动USB相机 —— 可视化
        #    获取/image_raw数据：
        # ============================================================
        # pkg_share = get_package_share_directory('dm_imu')
        # params_file = os.path.join(pkg_share, 'config', 'params.yaml')
        # print(f"启动USB相机")
        # nodes.append(Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam_node',
        #     output='screen',
        #     parameters=[
        #         {'image_width': 1280,
        #         'image_height': 720,
        #         'pixel_format': 'mjpeg',    # 使用 MJPEG 可获得 30fps
        #         'framerate': 30.0,
        #         'camera_frame_id': 'camera_link',
        #         'io_method': 'mmap',}]
        #     ))
       

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
