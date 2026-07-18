#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import serial.tools.list_ports
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
import math


class JoyToServoNode(Node):
    def __init__(self):
        super().__init__('joy_to_servo_node')

        # ---- 声明可配置参数 ----
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.5)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value

        # ---- 打开串口 ----
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.get_logger().info(f'✅ 串口 {port} 已打开，波特率 {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 无法打开串口 {port}: {e}')
            raise SystemExit(1)

        # ---- 订阅 joy 话题 ----
        self.subscription = self.create_subscription(
            Float64,
            '/wheel_control/dir',
            self.dir_callback,
            10
        )

        # 创建前轮转向角度的位置发布话题
        self.pub = self.create_publisher(Float64, '/df_dir_rt', 10)
        self.motor_status_data = Float64()
        self.motor_status_data.data = 0.0
        
        self.get_logger().info('🎮 等待手柄数据...')

    def dir_callback(self, msg: Float64):
        """收到手柄数据时自动调用"""
        # ---------- 用户需在此处实现角度计算 ----------
        angle =  (msg.data / math.pi) * 180
        # self.get_logger().info(f'🎮 {angle}...')
        # ----------------------------------------------
        if angle is None:
            return  # 返回 None 则不发送
        # 限幅 0~180
        # angle = max(1.0, min(180.0, angle)) # 取消限制幅度，通过前序话题进行限制
        # 格式化发送：保留一位小数 + 换行符
        cmd = f"{angle:.1f}\n"
        try:
            self.ser.write(cmd.encode())
            # self.get_logger().info(f'📤 发送: {cmd.strip()}')
            self.motor_status_data.data = angle
            self.pub.publish(self.motor_status_data) 
        except serial.SerialTimeoutException:
            self.get_logger().info('⏳ 串口写入超时')
        except Exception as e:
            self.get_logger().info(f'❌ 串口写入错误: {e}')

    

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('🔌 串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyToServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
