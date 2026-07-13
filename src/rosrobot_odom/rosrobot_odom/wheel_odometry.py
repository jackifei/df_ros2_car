#!/usr/bin/env python3
"""
轮式里程计模块（阿克曼底盘 + IMU 角速度）
===========================================
基于后轮编码器线速度和 IMU 角速度进行航位推算。

运动学模型:
  线速度 v   = (v_left + v_right) / 2  （来自后轮编码器）
  角速度 ω   = 来自 IMU 的 z 轴角速度

  位置增量:
    dx = v · cos(θ) · dt
    dy = v · sin(θ) · dt
    dθ = ω · dt

发布:
  /odom  (nav_msgs/Odometry)
  /tf    (odom → base_link)
"""
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class WheelOdometry(Node):
    """
    基于后轮线速度和 IMU 角速度的轮式里程计

    Usage:
        odom = WheelOdometry(node, wheelbase=0.36, track_width=0.39)
        # 每个周期调用:
        odom.update_with_imu(v=0.5, omega_imu=0.1, current_time)
    """

    def __init__(self, node, wheelbase: float, track_width: float,
                 publish_rate: float = 50.0):
        """
        Args:
            node:               ROS2 Node 实例
            wheelbase:          轴距 L (m) – 仅用于日志，不参与运动学计算
            track_width:        后轮轮距 W (m) – 仅用于日志和旧接口
            publish_rate:       发布频率 (Hz)，用于设置名义 dt（实际使用动态 dt）
        """
        self.node = node
        self.L = float(wheelbase)
        self.W = float(track_width)
        

        # ---- 位姿状态 ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ---- 上一次更新时间戳 ----
        self.last_timestamp = None

        # ---- 里程计发布 ----
        self.odom_pub = node.create_publisher(Odometry, '/odom', 10)
        # ================================================================
        #  发布 /path（纯轨迹线，供 RViz Path 显示使用）
        # ================================================================
        self.path_pub = self.create_publisher(
            Path,
            '/path',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )
        self.path_poses = []  # 累积路径点

        # ---- TF 广播器，坐标系广播器 ----
        self.tf_broadcaster = TransformBroadcaster(node)

        # ---- 坐标系名称 ----
        self.odom_frame = 'odom'
        self.base_link_frame = 'base_link'

        # ---- 协方差矩阵 ----
        # nav_msgs/Odometry 协方差矩阵按行优先排列，顺序为 [x,y,z,roll,pitch,yaw]
        # 索引 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
        self._pose_cov = [0.0] * 36
        self._pose_cov[0] = 0.01   # x
        self._pose_cov[7] = 0.01   # y
        self._pose_cov[35] = 0.05  # yaw (IMU 可能有漂移，适当增大)

        self._twist_cov = [0.0] * 36
        self._twist_cov[0] = 0.01  # vx
        self._twist_cov[35] = 0.03 # vyaw (IMU 角速度噪声)
        
        self._publish_count = 0

        node.get_logger().info(
            f'WheelOdometry (IMU-based): L={self.L:.3f}m, W={self.W:.3f}m, '
        )

    # =========================================================================
    # 推荐方法：使用后轮线速度 + IMU 角速度
    # =========================================================================

    def update_with_imu(self, v: float, omega_imu: float, timestamp) -> None:
        """
        使用后轮线速度和 IMU 角速度更新里程计（推荐用于无转向角传感器的阿克曼底盘）

        Args:
            v:           后轴中点线速度 (m/s)，来自编码器
            omega_imu:   IMU 测量的 z 轴角速度 (rad/s)，正值表示逆时针
            timestamp:   ROS2 Time 对象
        """
        if not (math.isfinite(v) and math.isfinite(omega_imu)):
            self.node.get_logger().warning(
                f"Invalid input: v={v}, omega_imu={omega_imu}. Skipping this frame."
            )
            return
        # 动态时间步长
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return
        dt = (timestamp - self.last_timestamp).nanoseconds / 1e9
        self.last_timestamp = timestamp

        if dt <= 1e-9:
            return
        

        # 位姿积分
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        self.x += v * cos_theta * dt
        self.y += v * sin_theta * dt
        self.theta += omega_imu * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 发布
        self._publish_odometry(v, omega_imu, timestamp)
        self._publish_tf(timestamp)

    def update_from_twist(self, linear_vel: float, angular_vel: float, timestamp) -> None:
        """
        直接从 Twist 的线速度/角速度更新
        """
        self._integrate_and_publish(linear_vel, angular_vel, timestamp)

    # =========================================================================
    # 内部积分与发布
    # =========================================================================

    def _integrate_and_publish(self, v: float, omega: float, timestamp) -> None:
        """统一积分和发布逻辑"""
        if not (math.isfinite(v) and math.isfinite(omega)):
            self.node.get_logger().warning(
                f"Invalid input in _integrate_and_publish: v={v}, omega={omega}"
            )
            return
        
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return
        
        dt = (timestamp - self.last_timestamp).nanoseconds / 1e9
        self.last_timestamp = timestamp
        if dt <= 1e-9:
            return

        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        self.x += v * cos_theta * dt
        self.y += v * sin_theta * dt
        self.theta += omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self._publish_odometry(v, omega, timestamp)
        self._publish_tf(timestamp)

    # =========================================================================
    # 消息发布
    # =========================================================================

    def _publish_odometry(self, v: float, omega: float, timestamp) -> None:
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_link_frame

        half_theta = self.theta / 2.0
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half_theta), w=math.cos(half_theta)
        )
        msg.pose.covariance = self._pose_cov

        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = omega
        msg.twist.covariance = self._twist_cov

        self.odom_pub.publish(msg)

    def _publish_tf(self, timestamp) -> None:
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        half_theta = self.theta / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_theta)
        t.transform.rotation.w = math.cos(half_theta)

        self.tf_broadcaster.sendTransform(t)
        # ----------------------------------------------------------------
        #  发布 /path（纯轨迹线，每 10 个点发布一次减少消息量）
        # ----------------------------------------------------------------
        now = self.get_clock().now()
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = t.transform.rotation
        self.path_poses.append(pose)

        # 限制路径长度，保留最近 5000 个点
        if len(self.path_poses) > 5000:
            self.path_poses = self.path_poses[-5000:]

        self._publish_count += 1
        # 每 10 个 odom 回调发布一次 path（50Hz → 5Hz path 更新）
        if self._publish_count % 10 == 0:
            path_msg = Path()
            path_msg.header.stamp = now.to_msg()
            path_msg.header.frame_id = 'odom'
            path_msg.poses = self.path_poses
            self.path_pub.publish(path_msg)

    # =========================================================================
    # 工具方法
    # =========================================================================

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.last_timestamp = None
        self.node.get_logger().info(f'Odometry reset to ({x:.3f}, {y:.3f}, {theta:.3f})')

    @property
    def pose(self) -> tuple:
        return (self.x, self.y, self.theta)

    @property
    def summary(self) -> str:
        return (f'Odom: x={self.x:.3f}m, y={self.y:.3f}m, '
                f'θ={math.degrees(self.theta):.1f}°')