#!/usr/bin/env python3
"""
手柄桥接 + 里程计主节点 (50Hz)
================================
订阅手柄Twist → 阿克曼电子差速 → L/R轮速 + 里程计

订阅:
  /cmd_vel (geometry_msgs/Twist)  — 手柄控制指令

发布:
  /odom              (nav_msgs/Odometry)  @50Hz
  /wheel_speeds/left  (Float64)           — 左后轮目标速度
  /wheel_speeds/right (Float64)           — 右后轮目标速度
  /tf                 (odom → base_link)

架构:
  Twist回调 → 阿克曼反算转向角 → 电子差速 → 轮速 + 里程计
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

from .ackermann_differential import AckermannDifferential
from .wheel_odometry import WheelOdometry


class JoystickBridgeNode(Node):
    """
    手柄桥接节点

    工作流程:
      1. 订阅 /cmd_vel (Twist)
      2. 由 v 和 ω 反算等效前轮转向角 δ
      3. 阿克曼电子差速计算 L/R 后轮速度
      4. 发布 /wheel_speeds + 更新里程计 → 发布 /odom + /tf
    """

    def __init__(self):
        super().__init__('joystick_bridge_node')

        # ---- 参数声明 ----
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('track_width', 0.20)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_link_frame_id', 'base_link')

        # ---- 读取参数 ----
        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ---- 核心模块 ----  
        # wheelbase:   轴距 L — 前后轴间距 (m)
        # track_width: 后轮轮距 W — 左右后轮间距 (m)
        self.ackermann = AckermannDifferential(0.36, 0.39)
        # self.ackermann = AckermannDifferential(wheelbase, track_width)
        # self.odometry = WheelOdometry(self, track_width, self.publish_rate)

        # ---- 最新Twist缓存 ----
        self._latest_twist = None  # (v, omega)
        self._twist_received = False

        # ---- 订阅者 ----
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._twist_callback,
            10
        )

        # ---- 创建轮速发布者 ----
        # 后轮速度（使用两个 Float64 话题，兼容性好，无需编译自定义消息）
        self.left_right_wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_control/leftright', 10)
        # self.right_wheel_pub = self.create_publisher(Float64, '/wheel_speeds/right', 10)
        # ---- 创建转向角度发布者 ----
        self.left_right_dir_pub = self.create_publisher(Float64, '/wheel_control/dir', 10)
        # ---- 50Hz 主循环定时器 ----
        timer_period = 1.0 / self.publish_rate  # 0.02s
        self.timer = self.create_timer(timer_period, self._timer_callback)

        # ---- 看门狗：手柄超时检测 ----
        self._watchdog_timeout = 0.5  # 500ms 无手柄数据则停车
        self._last_twist_time = self.get_clock().now()

        self.get_logger().info(
            f'JoystickBridge started @ {self.publish_rate:.0f}Hz | '
            f'{self.ackermann.summary}'
        )

    # =========================================================================
    # Twist 回调
    # =========================================================================
    def _twist_callback(self, msg: Twist):
        """
        接收手柄控制指令
        并且更新缓存状态
        手柄发布:
          linear.x  → 期望线速度 (m/s)
          angular.z → 期望角速度 (rad/s)
        """
        v = msg.linear.x
        omega = msg.angular.z

        # 限幅
        # v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
        # omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))
        
        # 更新最新缓存
        self._latest_twist = (v, omega)
        self._twist_received = True
        self._last_twist_time = self.get_clock().now()
 
    # =========================================================================
    # 50Hz 主循环 发布
    # =========================================================================
    def _timer_callback(self):
        """50Hz固定频率主循环 (每20ms触发)"""
        now = self.get_clock().now()

        # 看门狗检查：超时则停车
        elapsed = (now - self._last_twist_time).nanoseconds / 1e9
        if elapsed > self._watchdog_timeout:
            self._publish_wheel_speeds(0.0, 0.0)
            return

        if not self._twist_received or self._latest_twist is None:
            return

        v, omega = self._latest_twist

        # 1. 阿克曼电子差速计算
        v_left, v_right, steering_angle = self.ackermann.compute_from_twist(v, omega)

        # 2. 发布后轮速度,发布前轮转向角度
        self._publish_wheel_speeds(v_left, v_right)
        self._publish_wheel_dir(steering_angle)


        # 3. 更新里程计
        # self.odometry.update(v_left, v_right, now)

    # =========================================================================
    # 后轮速度发布, 前轮转向发布
    # =========================================================================

    def _publish_wheel_speeds(self, v_left: Float64, v_right: Float64):
        """发布左右后轮目标速度"""
        msg_data = Float64MultiArray()
        msg_data.data = [v_left,v_right]
        self.left_right_wheel_pub.publish(msg_data)

    def _publish_wheel_dir(self, dir: Float64):
        """发布左右后轮目标速度"""
        msg_data_dir = Float64()
        msg_data_dir.data = dir
        self.left_right_dir_pub.publish(msg_data_dir)

    # =========================================================================
    # 状态查询
    # =========================================================================

    @property
    def is_active(self) -> bool:
        """是否正在接收手柄数据"""
        elapsed = (self.get_clock().now() - self._last_twist_time).nanoseconds / 1e9
        return self._twist_received and elapsed <= self._watchdog_timeout


def main(args=None):
    rclpy.init(args=args)
    node = JoystickBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
