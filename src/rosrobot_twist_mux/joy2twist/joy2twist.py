#!/usr/bin/env python3
"""
joy2twist.py — 导航和手柄控制 多路复用分配器

订阅话题：/joy (sensor_msgs/Joy)
发布话题：/cmd_vel_joy (geometry_msgs/Twist)

逻辑：
手柄元件	映射到 Twist 字段	说明
左摇杆上下	linear.x	前进/后退
左摇杆左右	linear.y（全向）或 angular.z（差速转向）	侧移或原地旋转
右摇杆左右	angular.z	旋转速度（常用于差速底盘）
右摇杆上下	很少用，可映射到 linear.z（无人机升降）	垂直运动
左扳机 / 右扳机	线性缩放因子或 linear.x反向	倒车或加减速
D-pad 上下	linear.x固定步进值	定速巡航
D-pad 左右	angular.z固定步进值	点动旋转
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

class joy2twist(Node):
    """
    手柄joy转换标准twist
    """

    def __init__(self):
        super().__init__('joy2twist')

        # ---- 声明参数 ----
        self.declare_parameter('axis_linear_x', 1)      # 左摇杆上下
        self.declare_parameter('axis_angular_z', 0)     # 右摇杆左右
        self.declare_parameter('max_linear_speed', 0.01) # m/s
        self.declare_parameter('max_angular_speed', 1.0)# rad/s
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)

        # ---- 读取参数 ----
        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_angular_z = self.get_parameter('axis_angular_z').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.invert_linear = self.get_parameter('invert_linear').value
        self.invert_angular = self.get_parameter('invert_angular').value

        # ---- 订阅/发布 ----
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)

        self.get_logger().info(
            f'Joy→Twist 节点已启动:\n'
            f'  linear_x 轴={self.axis_linear_x}, angular_z 轴={self.axis_angular_z}\n'
            f'  最大线速度={self.max_linear} m/s, 最大角速度={self.max_angular} rad/s\n'
            f'  死区={self.deadzone}, 反转线速度={self.invert_linear}, 反转角速度={self.invert_angular}'
        )

    def apply_deadzone(self, value: float) -> float:
        """死区处理：绝对值小于死区则置零"""
        return 0.0 if abs(value) < self.deadzone else value

    def joy_callback(self, msg: Joy):
        """将Joy消息转为Twist并发布"""
        # 检查轴数组长度是否足够
        twist = Twist()
        # 线速度 (前后)
        raw_linear = msg.axes[self.axis_linear_x]
        linear = self.apply_deadzone(raw_linear)
        if self.invert_linear:
            linear = -linear
        twist.linear.x = linear * self.max_linear
        # 角速度 (旋转)
        raw_angular = msg.axes[self.axis_angular_z]
        angular = self.apply_deadzone(raw_angular)
        if self.invert_angular:
            angular = -angular
        twist.angular.z = angular * self.max_angular
        # 发布
        self.pub.publish(twist)

        # 调试日志（可选）
        # self.get_logger().debug(f'发布 Twist: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}')
  


def main(args=None):
    rclpy.init(args=args)
    node = joy2twist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
