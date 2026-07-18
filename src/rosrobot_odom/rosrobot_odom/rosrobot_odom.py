#!/usr/bin/env python3
"""
wheel_odom_fusion_node.py — 增强型轮式里程计节点（阿克曼底盘，无IMU版本）
订阅：
  - /encoder_speed     (std_msgs/Float64)  后轴中心线速度 (m/s)
  - /df_dir_rt         (std_msgs/Float64)  前轮实时转向角 (rad)
发布：
  - /odom              (nav_msgs/Odometry)
  - /path              (nav_msgs/Path)
  - TF: odom → base_link
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist


# 导入无IMU里程计算法类（已改为非Node版本）
from rosrobot_odom.wheel_odom_noimu import WheelOdomPure


class WheelOdomFusionNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_fusion_node')

        # ---- 声明参数 ----
        self.declare_parameter('wheelbase', 0.36)
        self.declare_parameter('track_width', 0.390)
        self.declare_parameter('wheel_radius', 0.062)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('enable_lowpass', True)
        self.declare_parameter('lowpass_cutoff_hz', 15.0)
        self.declare_parameter('publish_rate', 50.0)

        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_speed = self.get_parameter('max_speed').value
        enable_lowpass = self.get_parameter('enable_lowpass').value
        cutoff_hz = self.get_parameter('lowpass_cutoff_hz').value
        publish_rate = self.get_parameter('publish_rate').value

        # ---- 创建里程计算法实例（传入 self 作为 node） ----
        self.odom = WheelOdomPure(
            wheelbase=wheelbase,
            track_width=track_width,
            wheel_radius=wheel_radius,
            max_speed=max_speed,
            enable_lowpass=enable_lowpass,
            lowpass_cutoff_hz=cutoff_hz
        )

        # ---- 传感器数据缓存 ----
        self.latest_v = 0.0               # 线速度 (m/s)
        self.latest_steering = 0.0        # 前轮转向角 (rad)

        # ---- 订阅话题 ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 编码器线速度（话题 /encoder_speed，单位 m/s）
        self.sub_speed = self.create_subscription(
            Twist, '/cmd_vel_rt', self.speed_callback, qos
        )
        # 前轮转向角（话题 /df_dir_rt）
        self.sub_steer = self.create_subscription(
            Float64, '/df_dir_rt', self.steer_callback, qos
        )

        # ---- 定时器驱动里程计更新 ----
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('WheelOdomFusionNode 启动成功')

    def speed_callback(self, msg: Twist):
        """编码器线速度回调"""
        self.latest_v = msg.linear.x
        # self.latest_steering = msg._angular.z
        # self.get_logger().info(f'{self.latest_steering}---{self.latest_v}')

    def steer_callback(self, msg: Float64):
        """前轮转向角回调"""
        self.latest_steering = (msg.data / 180) * math.pi

    def timer_callback(self):
        """定时触发里程计更新"""
        now = self.get_clock().now()
        # 调用 update，传入线速度、转向角、时间戳
        self.odom.update(
            v=self.latest_v,
            steering_angle=self.latest_steering,
            timestamp=now
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




