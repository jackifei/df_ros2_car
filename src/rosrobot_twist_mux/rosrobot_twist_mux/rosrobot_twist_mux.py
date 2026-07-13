#!/usr/bin/env python3
"""
rosrobot_twist_mux.py — 导航和手柄控制 多路复用分配器

第一路：订阅 /cmd_vel_joy (Twist)：
第二路：订阅 /cmd_vel_nav (Twist)：

逻辑：
- 默认模式：手动（手柄控制）
- 如果手柄的 START 按钮按下，切换到自动导航模式
- 在自动导航模式下，若超过0.5秒未收到导航指令，自动切回手柄控制
- 在任何时候，只要手柄有非零速度输入（即人为介入），立即切回手柄控制
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

class CmdVel_Mux_Sync(Node):
    """
    多路复用分配器
    """

    def __init__(self):
        super().__init__('rosrobot_twist_mux')
        
        # ===== 订阅遥控器节点 /joy ===== 
        self.cmd_vel_joysub = self.create_subscription(Twist, '/cmd_vel_joy', self.joysub_listener_callback, 10)
        # ===== 订阅遥控器节点 /joy ===== 
        self.cmd_vel_navsub = self.create_subscription(Twist, '/cmd_vel_nav', self.navsub_listener_callback, 10)
        # ===== 订阅遥控器节点 /joy ===== 
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_sub_callback, 10)

        # ===== 发布 /joint_states =====
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # ===== 定时器 =====
        self.publish_rate = 50  # 发布频率
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd_vel)

        self.last_joy_twist = Twist()    # 最近一次的手柄
        self.last_nav_twist = Twist()    # 最近一次的导航

        self.cmd_vel_pub_ = Twist()
        # 时间戳
        self.last_joy_time = self.get_clock().now()
        self.last_nav_time = self.get_clock().now()

        self.manual_mode = False                     # True=导航控制，False=手动控制
        self.get_logger().info('nav/joy 选择器已启动，初始模式：手动(手柄)')
        

    def joysub_listener_callback(self, msg:Twist):
        """接收 手柄控制twist"""
        self.last_joy_twist = msg
        self.last_joy_time = self.get_clock().now()
        # self.get_logger().info(f'接收手柄消息{self.last_joy_time}')

    def navsub_listener_callback(self, msg:Twist):
        """接收 导航控制twist"""
        self.last_nav_twist = msg
        self.last_nav_time = self.get_clock().now()
        # self.get_logger().info(f'接收导航消息{self.last_nav_time}')

    def joy_sub_callback(self, msg:Joy):
        """接收手柄按键/摇杆原始数据，检测START按钮"""
        # msg.buttons[7]  start按钮
        but_start_index = 7
        if msg.buttons[but_start_index] == 1 and self.manual_mode == False:
            self.last_nav_time = self.get_clock().now()
            self.manual_mode = True
            self.get_logger().warn(f'START按钮切换模式 -> 自动导航')
                


    def publish_cmd_vel(self):
        """定时回调：根据当前模式及超时/介入情况，发布最终cmd_vel"""
        now = self.get_clock().now()
        dt_joy = (now - self.last_joy_time).nanoseconds / 1e9
        dt_nav = (now - self.last_nav_time).nanoseconds / 1e9

        # --- 强制接管条件 ---
        # 1. 手柄有非零速度输入（人为介入）
        joy_nonzero = (abs(self.last_joy_twist.linear.x) > 0.001 or
                       abs(self.last_joy_twist.angular.z) > 0.001)
        if joy_nonzero and self.manual_mode:
            self.manual_mode = False
            self.get_logger().warn(f'人工介入，已切换到手动模式')

        # 2. 导航超时（0.5秒未收到新指令）
        if self.manual_mode and dt_nav > 3.0:
            self.manual_mode = False
            self.get_logger().warn('导航超时(>3.0s)，自动切回手柄控制')

        # --- 选择输出 ---
        if self.manual_mode:
            output = self.last_nav_twist 
        else:
            output = self.last_joy_twist

        # 安全保护：若无任何有效输入且超时，发布零速
        if dt_joy > 5.0 and dt_nav > 5.0:
            output = Twist()

        self.cmd_vel_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVel_Mux_Sync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
