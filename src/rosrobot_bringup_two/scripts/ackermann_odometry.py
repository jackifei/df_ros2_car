#!/usr/bin/env python3
"""
ackermann_odometry.py — 自行车模型里程计

作用:
  从 /joint_states 读取后轮转速和前轮转向角，用自行车运动学模型
  （Bicycle Model）推算小车在平面上的位姿变化，发布:

    - /odom (nav_msgs/Odometry) — 里程计消息
    - odom → base_link TF       — 坐标变换

为什么需要:
  同步后驱方案不使用 diff_drive_controller（它自带里程计），
  而 SLAM (slam_toolbox) 需要 odom→base_link 的 TF 来推算
  激光雷达帧之间的运动。本节点用自行车模型手动完成这一任务。

自行车运动学模型 (Bicycle Model):

  已知:
    v = ω_wheel * wheel_radius        ← 后轮线速度 (m/s)
    δ = steering_angle                ← 前轮转向角 (rad)
    L = wheelbase                     ← 轴距 (m)

  则:
    ω = v * tan(δ) / L                ← 车身旋转角速度 (rad/s)

  位姿积分 (Δt 内):
    Δθ = ω * Δt
    Δx = v * cos(θ) * Δt
    Δy = v * sin(θ) * Δt

  其中 θ 取 Δt 前后的平均值，以提高精度:
    θ_mid = θ + Δθ / 2

参数（从 URDF 关节原点推算）:
  wheel_radius: 0.05 m  (车轮半径)
  wheelbase:    0.306 m (前轴 x=0.167 - 后轴 x=-0.139 ≈ 0.306)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster


class AckermannOdometry(Node):
    """
    自行车模型里程计节点。

    订阅: /joint_states (使用 lh_joint 位置/速度, lq_joint 位置)
    发布: /odom (Odometry)
          odom → base_link (TF)
    """

    def __init__(self):
        super().__init__('ackermann_odometry')

        # ================================================================
        #  物理参数
        # ================================================================
        self.wheel_radius = self.declare_parameter(
            'wheel_radius', 0.062
        ).value
        self.wheelbase = self.declare_parameter(
            'wheelbase', 0.306
        ).value

        # ================================================================
        #  关节名（与 URDF 一致）
        # ================================================================
        self.wheel_joint  = 'lh_joint'    # 左后轮（也可用 rh_joint，同速）
        self.steer_joint  = 'lq_joint'    # 左前轮转向（也可用 rq_joint，同角）

        # ================================================================
        #  状态变量 — 累计位姿
        # ================================================================
        self.x     = 0.0   # odom 系下 X 坐标
        self.y     = 0.0   # odom 系下 Y 坐标
        self.theta = 0.0   # odom 系下朝向角 (yaw)

        # 上一次关节读数（用于计算速度）
        self.last_wheel_pos    = None
        self.last_steer_angle  = 0.0
        self.last_time         = None

        # 诊断计数器（前 5 次回调打印日志）
        self._cb_count = 0
        self._publish_count = 0

        # ================================================================
        #  订阅 /joint_states（使用默认 RELIABLE QoS，与 cmd_vel_to_joints_sync 匹配）
        # ================================================================
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # ================================================================
        #  发布 /odom（BEST_EFFORT QoS — 与 RViz2 Odometry 显示兼容）
        # ================================================================
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )

        # ================================================================
        #  TF 广播器 — 发布 odom → base_link
        # ================================================================
        self.tf_broadcaster = TransformBroadcaster(self)

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

        self.get_logger().info(
            'ackermann_odometry 已启动\n'
            f'  wheel_radius={self.wheel_radius}m\n'
            f'  wheelbase={self.wheelbase}m\n'
            f'  订阅: /joint_states → {self.wheel_joint} + {self.steer_joint}\n'
            f'  发布: /odom + /path + odom→base_link TF'
        )

    def joint_callback(self, msg: JointState):
        """
        接收 /joint_states，提取后轮位置和转向角，积分更新位姿。

        参数:
          msg: JointState 消息，包含所有 4 个关节的状态
        """
        # ----------------------------------------------------------------
        #  从消息中查找目标关节的索引
        # ----------------------------------------------------------------
        self._cb_count += 1
        try:
            wheel_idx = msg.name.index(self.wheel_joint)
            steer_idx = msg.name.index(self.steer_joint)
        except ValueError:
            if self._cb_count <= 3:
                self.get_logger().warn(
                    f'joint_states 中未找到 {self.wheel_joint} 或 {self.steer_joint}'
                    f'  实际关节: {list(msg.name)}'
                )
            return

        wheel_pos   = msg.position[wheel_idx]
        steer_angle = msg.position[steer_idx]
        now         = self.get_clock().now()

        # ----------------------------------------------------------------
        #  首次回调：初始化参考点，不发布
        # ----------------------------------------------------------------
        if self.last_time is None:
            self.last_wheel_pos   = wheel_pos
            self.last_steer_angle = steer_angle
            self.last_time        = now
            return

        # ----------------------------------------------------------------
        #  计算 Δt
        # ----------------------------------------------------------------
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.2:
            # 异常 dt（跳变/暂停后恢复），重置参考避免积分爆炸
            self.last_wheel_pos   = wheel_pos
            self.last_steer_angle = steer_angle
            self.last_time        = now
            return

        # ----------------------------------------------------------------
        #  计算后轮线速度 v
        #  Δθ_wheel = wheel_pos - last_wheel_pos
        #  v = Δθ_wheel / Δt * r
        #
        #  注意: lh_joint 是 continuous 类型，可能出现 2π 跳变
        # ----------------------------------------------------------------
        delta_wheel = wheel_pos - self.last_wheel_pos
        # 处理 continuous 关节的 2π 环绕
        delta_wheel = math.atan2(
            math.sin(delta_wheel), math.cos(delta_wheel)
        )
        wheel_angular_vel = delta_wheel / dt
        linear_vel = wheel_angular_vel * self.wheel_radius

        # ----------------------------------------------------------------
        #  计算车身角速度 ω
        #  ω = v * tan(δ) / L
        #  当 |δ| 接近 π/2 时 tan 趋于无穷，限幅保护
        # ----------------------------------------------------------------
        steer_angle_clamped = max(
            -1.5, min(1.5, steer_angle)
        )
        tan_steer = math.tan(steer_angle_clamped)
        angular_vel = linear_vel * tan_steer / self.wheelbase

        # ----------------------------------------------------------------
        #  位姿积分（中点法，提高精度）
        #  θ_mid = θ + Δθ/2
        #  Δx = v * cos(θ_mid) * Δt
        #  Δy = v * sin(θ_mid) * Δt
        # ----------------------------------------------------------------
        delta_theta = angular_vel * dt
        theta_mid = self.theta + delta_theta / 2.0

        self.x     += linear_vel * math.cos(theta_mid) * dt
        self.y     += linear_vel * math.sin(theta_mid) * dt
        self.theta += delta_theta

        # 归一化 θ 到 [-π, π]
        self.theta = math.atan2(
            math.sin(self.theta), math.cos(self.theta)
        )

        # ----------------------------------------------------------------
        #  更新参考点
        # ----------------------------------------------------------------
        self.last_wheel_pos   = wheel_pos
        self.last_steer_angle = steer_angle
        self.last_time        = now

        # ----------------------------------------------------------------
        #  发布 odom → base_link TF
        # ----------------------------------------------------------------
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_link'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        # yaw → quaternion
        half_yaw = self.theta / 2.0
        tf_msg.transform.rotation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(half_yaw),
            w=math.cos(half_yaw),
        )
        self.tf_broadcaster.sendTransform(tf_msg)

        # ----------------------------------------------------------------
        #  发布 /path（纯轨迹线，每 10 个点发布一次减少消息量）
        # ----------------------------------------------------------------
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = tf_msg.transform.rotation
        self.path_poses.append(pose)

        # 限制路径长度，保留最近 5000 个点
        if len(self.path_poses) > 5000:
            self.path_poses = self.path_poses[-5000:]

        # 每 10 个 odom 回调发布一次 path（50Hz → 5Hz path 更新）
        if self._publish_count % 10 == 0:
            path_msg = Path()
            path_msg.header.stamp = now.to_msg()
            path_msg.header.frame_id = 'odom'
            path_msg.poses = self.path_poses
            self.path_pub.publish(path_msg)

        # ----------------------------------------------------------------
        #  发布 /odom 里程计消息
        # ----------------------------------------------------------------
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id  = 'base_link'

        # 位姿
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = tf_msg.transform.rotation

        # 位姿协方差（6×6 矩阵，仅对角线有值，表示不确定性）
        odom_msg.pose.covariance[0]  = 0.01
        odom_msg.pose.covariance[7]  = 0.01
        odom_msg.pose.covariance[14] = 1e6
        odom_msg.pose.covariance[21] = 1e6
        odom_msg.pose.covariance[28] = 1e6
        odom_msg.pose.covariance[35] = 0.001

        # 速度
        odom_msg.twist.twist.linear.x  = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        # 速度协方差
        odom_msg.twist.covariance[0]  = 0.01
        odom_msg.twist.covariance[7]  = 0.01
        odom_msg.twist.covariance[14] = 1e6
        odom_msg.twist.covariance[21] = 1e6
        odom_msg.twist.covariance[28] = 1e6
        odom_msg.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom_msg)

        # 诊断: 前 10 次发布打印日志，方便排查数据流问题
        self._publish_count += 1
        if self._publish_count <= 10:
            self.get_logger().info(
                f'[odom #{self._publish_count}] '
                f'x={self.x:.4f} y={self.y:.4f} θ={self.theta:.4f} '
                f'v={linear_vel:.3f} ω={angular_vel:.3f} '
                f'δ={steer_angle:.3f} dt={dt:.4f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
