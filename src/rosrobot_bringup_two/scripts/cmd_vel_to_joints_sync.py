#!/usr/bin/env python3
"""
cmd_vel_to_joints_sync.py — 同步后驱 + 前轮转向 控制

订阅 /cmd_vel (Twist)，转换为关节状态指令：

  控制方案（类似汽车：油门与方向盘各自独立）：
    - linear.x  → lh_joint / rh_joint 同步转速（后轮油门，直行/倒车）
    - angular.z → lq_joint / rq_joint 转向角度（前轮方向盘，左转/右转）
    - 两通道完全独立：可同时输入前进+转向 → 驱动中转弯

  与差速方案的区别：
    - 左右后轮始终接收相同速度指令（无差速项）
    - 转弯只靠前轮转向角，不干涉后轮转速
    - 后轮驱动与转向互不耦合，各自独立受控

发布到 /joint_states，供 robot_state_publisher → TF → RViz2 使用。
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

class CmdVelToJointsSync(Node):
    """
    同步后驱控制器：将 /cmd_vel 映射为前轮转向 + 后轮同步转速。

    机器人关节（来自 rosrobot URDF）：
      - lh_joint / rh_joint : 后轮连续转动关节（驱动轮，同速）
      - lq_joint / rq_joint : 前轮回转关节（转向，±0.7 rad 限位）
    """

    def __init__(self):
        super().__init__('cmd_vel_to_joints_sync')

        # ===== 可调参数 =====
        self.wheel_radius = self.declare_parameter(
            'wheel_radius', 0.61).value              # 车轮半径 (m)
        self.max_steering_angle = self.declare_parameter(
            'max_steering_angle', 0.7).value         # 最大转向角 (rad)
        self.steering_scale = self.declare_parameter(
            'steering_scale', 1.0).value             # angular.z → 转向角比例
        self.publish_rate = self.declare_parameter(
            'publish_rate', 50.0).value              # 发布频率 (Hz)

        # ===== 关节名（与 URDF 一致）=====
        self.steer_left_joint  = 'lq_joint'
        self.steer_right_joint = 'rq_joint'
        self.wheel_left_joint  = 'lh_joint'
        self.wheel_right_joint = 'rh_joint'

        # ===== 当前关节位置（积分用）=====
        self.steer_left_pos  = 0.0
        self.steer_right_pos = 0.0
        self.wheel_left_pos  = 0.0
        self.wheel_right_pos = 0.0

        # 时间戳
        self.last_time = self.get_clock().now()

        # ===== 订阅遥控器节点 /joy ===== 
        # Cself.joint_motor_sub = self.create_subscription(JointState, 'df_motor_status', self.joy_listener_callback, 10)
        # ===== 订阅遥控器节点 /joy ===== 
        # self.joint_dir_sub = self.create_subscription(JointState, 'df_dir_status', self.joint_dir_listener_callback, 10)

        # ===== 订阅 /cmd_vel =====
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ===== 发布 /joint_states =====
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ===== 定时器 =====
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)

        self.current_twist = Twist()
        self.current_joy = JointState()
        self.current_joint_dir = JointState()

       
        self.get_logger().info(
            f'cmd_vel_to_joints_sync 已启动 (同步后驱 + 前轮转向)\n'
            f'  wheel_radius={self.wheel_radius}m\n'
            f'  max_steering={self.max_steering_angle}rad\n'
            f'  控制策略: 油门(linear.x)→后轮驱动 | 方向盘(angular.z)→前轮转向\n'
            f'  两通道独立: 前进+转向可同时生效，驱动中转弯'
        )

    def joint_dir_listener_callback(self,msg:JointState):
        """接收 转向角度"""
        self.current_joint_dir = msg
        # self.get_logger().info(f'转向角度 {self.current_joint_dir.position[0]}')

    def joy_listener_callback(self,msg:JointState):
        """接收 前进后退"""
        self.current_joy = msg
        # self.get_logger().info(f'L速度 {self.current_joy.velocity[0]} R速度 {self.current_joy.velocity[1]}')

    def cmd_vel_callback(self, msg: Twist):
        """接收 /cmd_vel 指令"""
        self.current_twist = msg

    def publish_joint_states(self):
        """按定时器频率积分并发布关节状态。"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        dt = min(dt, 0.1)  # 上限防止跳变
       
        # self.get_logger().info(f'L速度 {self.current_joy.velocity[0]} R速度 {self.current_joy.velocity[1]}')
        # self.get_logger().info(f'L速度 {self.current_joy.velocity[0]} R速度 {self.current_joy.velocity[1]}转向角度 {self.current_joint_dir.position[0]}')
        
        
        # linear_x  = self.current_joy.axes[1] / 2     # 前进速度 (m/s)
        # angular_z = self.current_joy.angular.z     # 转向角速度 (rad/s)
        # if self.current_joy.velocity[0] == 0:
        #     linear_x  = 0     # 前进速度 (m/s)
        # if self.current_joy.velocity[0] > 0:
        #     linear_x  = 0.1     # 前进速度 (m/s)
        # if self.current_joy.velocity[0] < 0:
        #     linear_x  = -0.1     # 前进速度 (m/s)

        # if self.current_joint_dir.position[0] > 92:
        #     angular_z = 0.5
        # elif self.current_joint_dir.position[0] < 88:
        #     angular_z = -0.5
        # else:
        #     angular_z = 0.0
        linear_x  = 0.0  
        angular_z = 0.0

        # angular_z = self.current_twist.angular.z     # 转向角速度 (rad/s)

        # self.get_logger().info(f'速度 {linear_x} m/s 角速度 {angular_z}rad/s')

        # ============================================================
        #  转向角度：angular.z → 目标转向角（纯前轮转向）
        # ============================================================
        target_steer_angle = angular_z * self.steering_scale
        target_steer_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, target_steer_angle),
        )

        # 平滑过渡到目标角（避免突变）
        steer_speed = 1.0  # rad/s 转向响应速度
        steer_delta = steer_speed * dt
        steer_error_left  = target_steer_angle - self.steer_left_pos
        steer_error_right = target_steer_angle - self.steer_right_pos

        self.steer_left_pos  += max(-steer_delta, min(steer_delta, steer_error_left))
        self.steer_right_pos += max(-steer_delta, min(steer_delta, steer_error_right))

        # ============================================================
        #  后轮转速：linear.x → 两轮同速（同步后驱）
        #  核心简化：左右后轮速度完全相同，不叠加差速项
        #  转向与驱动独立：angular.z 控制前轮转向，linear.x 控制后轮转速
        #  两者互不干涉，可同时非零 → 驱动中转弯（类似汽车）
        # ============================================================
        wheel_angular_vel = linear_x / self.wheel_radius

        self.wheel_left_pos  += wheel_angular_vel * dt
        self.wheel_right_pos += wheel_angular_vel * dt

        # ============================================================
        #  发布 JointState
        # ============================================================
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [
            self.steer_left_joint,
            self.steer_right_joint,
            self.wheel_left_joint,
            self.wheel_right_joint,
        ]
        msg.position = [
            self.steer_left_pos,
            self.steer_right_pos,
            self.wheel_left_pos,
            self.wheel_right_pos,
        ]
        msg.velocity = [
            steer_speed if abs(steer_error_left)  > 0.001 else 0.0,
            steer_speed if abs(steer_error_right) > 0.001 else 0.0,
            wheel_angular_vel,
            wheel_angular_vel,  # ← 与左轮相同（同步驱动）
        ]

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJointsSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
