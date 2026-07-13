#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import serial.tools.list_ports
from sensor_msgs.msg import JointState


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
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # 创建前轮转向角度的位置发布话题
        self.pub = self.create_publisher(JointState, 'df_dir_status', 10)
        self.motor_status_data = JointState()
        self.motor_status_data.position = [0.0]
        
        self.get_logger().info('🎮 等待手柄数据...')

    def joy_callback(self, msg: Joy):
        """收到手柄数据时自动调用"""
        # ---------- 用户需在此处实现角度计算 ----------
        angle = self.calculate_angle(msg)
        # self.get_logger().info(f'🎮 {angle}...')
        # ----------------------------------------------
        if angle is None:
            return  # 返回 None 则不发送
        # 限幅 0~180
        angle = max(1.0, min(180.0, angle))
        # 格式化发送：保留一位小数 + 换行符
        cmd = f"{angle:.1f}\n"
        try:
            self.ser.write(cmd.encode())
            # self.get_logger().info(f'📤 发送: {cmd.strip()}')
            self.motor_status_data.position = [angle]
            self.pub.publish(self.motor_status_data) 
        except serial.SerialTimeoutException:
            self.get_logger().info('⏳ 串口写入超时')
        except Exception as e:
            self.get_logger().info(f'❌ 串口写入错误: {e}')

    def calculate_angle(self, msg: Joy) -> float:
        """
        【用户自定义函数】根据 joy 消息计算舵机角度
        返回值: 0.0 ~ 180.0 的浮点数，或 None（不发送）
        """
        # ========== 请在此处编写您的映射逻辑 ==========
        # 默认示例：使用 axes[0]（左右摇杆 -1~1）映射到 0~180°
        # 死区设置
        if msg.axes[0] < -0.9:
             axis_val = -0.9
        elif msg.axes[0] > 0.9:
             axis_val = 0.9
        else:
            axis_val = msg.axes[0]          # -1 ~ 1
        angle = (axis_val + 1.0) * 90.0 # 映射到 0 ~ 180
        return angle
        # ==============================================

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
