#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
df_motor_ctr 后轮驱动节点（适配 无刷电机驱动器 MSSD-60EHB_2D）
========================================================================

本节点负责阿克曼结构小车的后轮（左、右两个无刷电机）驱动：
  * 接收阿克曼控制器下发的左/右后轮速度指令，换算成电机 RPM 后通过
    RS485(MODBUS-RTU) 写入驱动器；
  * 周期读取左右电机的实时转速（RPM）与换向次数（当作编码器/位置反馈），
    转换成角速度/角度后以 JointState 发布给里程计 / 阿克曼控制器；
  * 提供急停、复位、读编码器值等控制接口。

驱动对象：MSSD-60EHB_2D（双电机一体驱动器，一个从站同时控制左、右电机）。

线程模型：所有 Modbus 读写都只在通信线程（_comm_loop）内执行。ROS2 回调、
定时器以及对外公开的 quick_stop()/reset()/read_*() 方法只读写状态标志或
缓存数据，不直接操作串口；需要写电机时由它们置标志，通信线程统一执行。

====================================================================
协议要点（详见《MSSD-60EHB_2D RS485 通讯协议及寄存器说明书 V2.0》）
====================================================================
1. 物理层 / 帧格式
   * MODBUS-RTU，支持功能码 0x03(读保持寄存器)、0x06(写单个寄存器)、
     0x10(写多个寄存器)。
   * 出厂默认：波特率 9600bps、8 数据位、无校验 + 1 停止位、从站地址 1。
     （若在初始调试时改过波特率/地址，请在 config/motor_control.yaml 中
       同步修改，保持一致。）
   * 一个 16 位寄存器 = 1 个字；32 位有符号值 = 高半字寄存器 + 低半字寄存器，
     高半字在前。示例：+1000 -> 高=0x0000 低=0x03E8；-500 -> 高=0xFFFF 低=0xFE0C。

2. 本节点默认工作在“独立速度闭环”模式（系统模式寄存器 0x007B = 1）。
   该模式下左右电机各自独立进行速度闭环，需要设置后轮左右轮速度时使用该模式。
   其它模式（同源开环/闭环、差速闭环）由使用者在初始调试时按需固定，
   本节点不做修改。

3. 本节点用到的寄存器（寄存器地址均为十进制，括号内为十六进制）：
   电机控制（写）：
     56(0x38)  停止寄存器1（右电机） 0=正常停止 1=紧急停止 2=自由停止  [0x06]
     57(0x39)  设定目标启动寄存器1 高半字（右电机速度/目标）          [0x06/0x10]
     58(0x3A)  设定目标启动寄存器1 低半字（右电机速度/目标）          [0x06/0x10]
     60(0x3C)  停止寄存器2（左电机）                                 [0x06]
     61(0x3D)  设定目标启动寄存器2 高半字（左电机速度/目标）          [0x06/0x10]
     62(0x3E)  设定目标启动寄存器2 低半字（左电机速度/目标）          [0x06/0x10]
     64(0x40)  参数恢复出厂命令（写入 1 恢复出厂）                    [0x06]
     65(0x41)  测试控制（1=左电机相序学习 2=右电机相序学习）          [0x06]

   实时状态（读，0x03）：
      9(0x09)  右电机实时转速 高半字（RPM）
     10(0x0A)  右电机实时转速 低半字（RPM）
     11(0x0B)  右电机错误状态
     16(0x10)  右电机换向次数 高半字（编码器/位置，32 位）
     17(0x11)  右电机换向次数 低半字
     20(0x14)  左电机实时转速 高半字（RPM）
     21(0x15)  左电机实时转速 低半字（RPM）
     22(0x16)  左电机错误状态
     27(0x1B)  左电机换向次数 高半字（编码器/位置，32 位）
     28(0x1C)  左电机换向次数 低半字

   换向次数说明：电机每 60 度电气角度换向一次，换算成机械角度为
   60 / 极对数（度）。即：机械角度(度) = 换向次数 * 60 / pole_pairs。

4. 速度映射与限速
   * 线速度 -> 轮子转速：RPM = v * 60 / (pi * D)，v 单位 m/s，D 为轮径(m)。
   * 本车为低速车，默认最高线速度 0.5 m/s（可在配置中修改）。

5. 本节点不再像旧版那样“按两台从机(地址 1/2)分别驱动”，因为 MSSD-60EHB_2D
   是一台双电机驱动器，左右电机共用同一个从站地址（默认 1）。
"""

import math
import threading
import time

import rclpy
from pymodbus.client import ModbusSerialClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from df_joy_bridge.msg import JoyState


# 寄存器地址常量（十进制）。所有地址均来自 MSSD-60EHB_2D 协议 V2.0。

# ---- 实时状态（读，0x03）----
REG_READ_BASE = 0x0009          # 本节点一次连续读取的起始寄存器(9)
REG_READ_COUNT = 20             # 连续读取 9~28 共 20 个寄存器
# 在连续读取结果中的索引（相对于 REG_READ_BASE）：
IDX_R_SPEED_H = 0               # 9   右电机实时转速 高半字
IDX_R_SPEED_L = 1               # 10  右电机实时转速 低半字
IDX_R_ERROR = 2                 # 11  右电机错误状态
IDX_R_POS_H = 7                 # 16  右电机换向次数 高半字
IDX_R_POS_L = 8                 # 17  右电机换向次数 低半字
IDX_L_SPEED_H = 11              # 20  左电机实时转速 高半字
IDX_L_SPEED_L = 12              # 21  左电机实时转速 低半字
IDX_L_ERROR = 13                # 22  左电机错误状态
IDX_L_POS_H = 18                # 27  左电机换向次数 高半字
IDX_L_POS_L = 19                # 28  左电机换向次数 低半字

# ---- 电机控制（写）----
REG_STOP_1 = 0x0038             # 56  停止寄存器1（右电机）
REG_TARGET_1_H = 0x0039         # 57  设定目标启动寄存器1 高半字（右电机）
REG_TARGET_1_L = 0x003A         # 58  设定目标启动寄存器1 低半字（右电机）
REG_STOP_2 = 0x003C             # 60  停止寄存器2（左电机）
REG_TARGET_2_H = 0x003D         # 61  设定目标启动寄存器2 高半字（左电机）
REG_TARGET_2_L = 0x003E         # 62  设定目标启动寄存器2 低半字（左电机）
REG_FACTORY_RESET = 0x0040      # 64  参数恢复出厂命令
REG_TEST_CTRL = 0x0041          # 65  测试控制（相序学习）

# ---- 停止寄存器取值 ----
STOP_NORMAL = 0                 # 正常停止（恢复运行/复位用）
STOP_EMERGENCY = 1              # 紧急停止
STOP_FREE = 2                   # 自由停止

# ---- 错误状态码（寄存器 11/22）说明，用于日志显示 ----
ERROR_TEXT = {
    0: "无错误",
    1: "堵转停机",
    2: "相序未学习",
    3: "相位异常",
    4: "相线过流",
    5: "母线过流",
    6: "电压过高",
    7: "电压过低",
    8: "相位学习异常",
    9: "过载异常",
    10: "485通信中断",
    11: "CAN通讯中断",
    12: "母线电压过高",
    13: "温度过高",
    14: "无感启动失败",
    15: "相电流异常",
    16: "上电大电流异常",
    17: "同步超差异常",
    18: "硬件异常",
}


class channel_MBRTU(Node):
    """MSSD-60EHB_2D 后轮双电机驱动节点（MODBUS-RTU 通信线程）。"""
    # 初始化
    def __init__(self):
        super().__init__("df_motor_control")

        # ---------- 声明可配置参数（默认值见 config/motor_control.yaml）----------
        # 串口参数：必须与驱动器的 485 参数保持一致。
        self.declare_parameter("port", "/dev/ttyUSB3")
        self.declare_parameter("baudrate", 9600)          # 驱动器出厂默认 9600
        self.declare_parameter("parity", "N")             # N=无校验, E=偶校验, O=奇校验
        self.declare_parameter("stopbits", 1)
        self.declare_parameter("bytesize", 8)
        self.declare_parameter("timeout", 0.2)            # 单次 Modbus 应答超时(秒)
        self.declare_parameter("slave_id", 1)             # 驱动器从站地址(出厂默认 1)

        # 车辆/轮子参数
        self.declare_parameter("wheel_track", 0.39)       # 后轮轮距(m)
        self.declare_parameter("wheel_diameter_m", 0.125) # 后轮直径(m)
        self.declare_parameter("max_linear_velocity", 0.5)  # 最高线速度(m/s)，低速车限速

        # 速度指令单位：'rad_s' = 轮子角速度(rad/s)，'m_s' = 轮子线速度(m/s)
        self.declare_parameter("input_unit", "rad_s")

        # 编码器换算：电机极对数（定子南北磁极对数 = 磁极数的一半）。
        # 换向次数 -> 机械角度：angle_deg = count * 60 / pole_pairs
        self.declare_parameter("pole_pairs", 4)

        # 电机旋转方向校正：若某侧电机实际转向与指令相反，改为 -1.0。
        self.declare_parameter("left_dir_sign", 1.0)
        self.declare_parameter("right_dir_sign", 1.0)

        # 通信循环/反馈发布频率(Hz)
        self.declare_parameter("loop_rate", 20.0)

        # 读取参数
        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.parity = str(self.get_parameter("parity").value)
        self.stopbits = int(self.get_parameter("stopbits").value)
        self.bytesize = int(self.get_parameter("bytesize").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.slave_id = int(self.get_parameter("slave_id").value)

        self.wheel_track = float(self.get_parameter("wheel_track").value)
        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.input_unit = str(self.get_parameter("input_unit").value).lower()
        self.pole_pairs = int(self.get_parameter("pole_pairs").value)
        self.left_dir_sign = float(self.get_parameter("left_dir_sign").value)
        self.right_dir_sign = float(self.get_parameter("right_dir_sign").value)
        self.loop_rate = float(self.get_parameter("loop_rate").value)

        # 由最高线速度推导最高轮速(RPM)，用于限幅。
        self.max_rpm = abs(self.linear_velocity_to_rpm(self.max_linear_velocity))
        if self.wheel_diameter_m <= 0.0:
            raise ValueError("wheel_diameter_m 必须大于 0")
        if self.pole_pairs <= 0:
            raise ValueError("pole_pairs 必须大于 0")

        # # ---------- 订阅：后轮速度指令 ----------
        # # 消息格式 Float64MultiArray，data = [右轮, 左轮]，单位由 input_unit 决定。
        # self.sub_rear = self.create_subscription(
        #     Float64MultiArray,
        #     "/hardware/rear_wheel_cmd",
        #     self.rear_wheel_callback,
        #     10,
        # )
        #
        # # ---------- 订阅：前轮转向反馈（JointState）----------
        # # 用于把前轮转向角合并进后轮发布的 JointState 中，方便里程计/控制器使用。
        # self.sub_steer = self.create_subscription(
        #     JointState,
        #     "/hardware/steer_feedback",
        #     self.steer_feedback_callback,
        #     10,
        # )

        # ---------- 订阅：急停 / 复位（Bool，True 触发一次）----------
        # 远端可通过这两个话题触发急停/复位，也可以直接调用方法。
        # self.sub_estop = self.create_subscription(
        #     Bool, "~/emergency_stop", self.emergency_stop_callback, 10)
        # self.sub_reset = self.create_subscription(
        #     Bool, "~/reset", self.reset_callback, 10)

        # ---------- 发布：合并后的关节反馈 ----------
        # name 顺序：[左后, 左前, 右后, 右前]，与 front_steering 节点保持一致。
        # self.joint_feedback_pub = self.create_publisher(
        #     JointState, "/hardware/joint_feedback", 10)


        # 手柄没有急停，轻推左摇杆，即可从NAV导航中抢车，摇杆默认回中速度为0，即可停车
        self._calib_sub = self.create_subscription(
            JoyState, '/joy_state', self._joy_state_callback, 10)
        # ---------- 状态变量 ----------
        self.steer_angle = 0.0                # 前轮转向角(rad)，来自 steer_feedback

        # 目标速度（RPM，已含方向校正与限幅）
        self.target_l_rpm = 0
        self.target_r_rpm = 0
        self._speed_dirty = False             # 速度指令变化标志（变化时才写，降低总线负载）
        self._last_write_l_rpm = None
        self._last_write_r_rpm = None

        # 命令/状态标志（只在回调或公开方法里“置位”，真正的 Modbus 读写由通信线程统一执行）
        self._estop_requested = False        # 请求急停一次
        self._reset_requested = False        # 请求复位一次
        self._startup_requested = False      # 请求上电初始化（正常停止 + 速度清零）
        self._shutdown_requested = False     # 请求最终停车并退出线程

        # 实时反馈（由通信线程更新，_publish_feedback 定时读取）
        self._lock = threading.Lock()
        self.l_rpm = 0.0
        self.r_rpm = 0.0
        self.l_error = 0
        self.r_error = 0
        self.l_pos_counts = 0                 # 左轮换向次数原始值
        self.r_pos_counts = 0                 # 右轮换向次数原始值
        self.l_pos_rad = 0.0                  # 左轮累计机械角度(rad)
        self.r_pos_rad = 0.0                  # 右轮累计机械角度(rad)
        self._last_l_counts = None
        self._last_r_counts = None
        self._last_logged_l_error = 0         # 用于只在错误状态变化时打印日志
        self._last_logged_r_error = 0

        # ---------- 串口 / 线程 ----------
        self.ser = None
        self._stop_event = threading.Event()
        self._comm_thread = threading.Thread(target=self._comm_loop, daemon=True)
        self._thread_done = threading.Event()  # 通信线程退出完成信号

        # 定时发布反馈（在 ROS2 执行器线程内发布，避免跨线程直接 publish）
        # self._feedback_timer = self.create_timer(
        #     1.0 / max(self.loop_rate, 1.0), self._publish_feedback)

        self.get_logger().info("MSSD-60EHB_2D 后轮驱动节点初始化")
        self._open_port()


    # 速度换算工具
    def linear_velocity_to_rpm(self, v: float) -> float:
        """
        线速度(m/s) -> 轮子转速(RPM)。
        公式：RPM = v * 60 / (pi * D)，D 为轮径(m)。
        """
        radius_m = self.wheel_diameter_m / 2.0
        return v * 30.0 / (math.pi * radius_m)

    @staticmethod
    def rad_per_sec_to_rpm(omega_rad_s: float) -> float:
        """角速度(rad/s) -> 转速(RPM)。RPM = omega * 60 / (2*pi)。"""
        return omega_rad_s * 60.0 / (2.0 * math.pi)

    @staticmethod
    def rpm_to_rad_per_sec(rpm: float) -> float:
        """转速(RPM) -> 角速度(rad/s)。omega = rpm * 2*pi / 60。"""
        return rpm * 2.0 * math.pi / 60.0

    def rpm_to_linear_velocity(self, rpm: float) -> float:
        """轮子转速(RPM) -> 线速度(m/s)。"""
        radius_m = self.wheel_diameter_m / 2.0
        return rpm * math.pi * radius_m / 30.0


    # 32 位有符号整数 <-> 两个 16 位寄存器（高半字在前）

    @staticmethod
    def _s32_to_words(value: int):
        """把 -2^31 ~ 2^31-1 的有符号整数拆成 [高半字, 低半字]。"""
        value = int(value)
        value &= 0xFFFFFFFF          # 转成无符号 32 位表示（补码）
        high = (value >> 16) & 0xFFFF
        low = value & 0xFFFF
        return [high, low]

    @staticmethod
    def _words_to_s32(high: int, low: int) -> int:
        """把 [高半字, 低半字] 合成有符号 32 位整数。"""
        raw = ((int(high) & 0xFFFF) << 16) | (int(low) & 0xFFFF)
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw

    # 串口管理

    def _open_port(self) -> bool:
        """打开 RS485 串口并启动通信线程。"""
        if self.ser is not None:
            self.get_logger().info("串口已打开，跳过重复打开")
            return False
        try:
            self.ser = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                parity=self.parity,
                stopbits=self.stopbits,
                bytesize=self.bytesize,
                timeout=self.timeout,
            )
            if not self.ser.connect():
                self.get_logger().error(f"串口 {self.port} 连接失败")
                self.ser = None
                return False

            self.get_logger().info(f"串口 {self.port} 打开成功，"
                                   f"波特率 {self.baudrate}，从站地址 {self.slave_id}")
            # 上电初始化（正常停止 + 速度清零）也统一由通信线程执行，
            # 这里只置标志，不直接做任何 Modbus 读写。
            self._startup_requested = True
            self._comm_thread.start()
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"打开串口异常: {exc}")
            self.ser = None
            return False

    def close_port(self):
        """停止通信线程并关闭串口。"""
        self._stop_event.set()
        # 先等通信线程退出（线程内可能还有未完成的 Modbus 读写），再关闭串口。
        if self._comm_thread.is_alive():
            self._comm_thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            self.ser = None
        self.get_logger().info("串口已关闭")


    # 底层 Modbus 写操作

    def _write_speed(self, left_rpm: int, right_rpm: int) -> bool:
        """
        写入左右电机速度目标（RPM）。
        使用 0x10 写多寄存器：每个电机速度占“高半字 + 低半字”共 2 个寄存器。
        正值=正转(前进)，负值=反转(后退)，0=正常停止。
        注意：本方法只能由通信线程调用（所有 Modbus 读写都集中在线程内）。
        """
        if self.ser is None:
            return False
        ok = True
        try:
            # 右电机：起始地址 0x0039，写 [高半字, 低半字]
            resp_r = self.ser.write_registers(
                address=REG_TARGET_1_H,
                values=self._s32_to_words(right_rpm),
                device_id=self.slave_id,
            )
            if resp_r is None or resp_r.isError():
                ok = False

            # 左电机：起始地址 0x003D，写 [高半字, 低半字]
            resp_l = self.ser.write_registers(
                address=REG_TARGET_2_H,
                values=self._s32_to_words(left_rpm),
                device_id=self.slave_id,
            )
            if resp_l is None or resp_l.isError():
                ok = False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"写入速度失败: {exc}")
            return False
        return ok

    def _write_stop(self, left_stop: int, right_stop: int) -> bool:
        """
        写入左右电机停止寄存器（0x06 写单寄存器）。
        left_stop / right_stop 取值：0=正常停止 1=紧急停止 2=自由停止。
        注意：停止寄存器1=右电机，停止寄存器2=左电机。
        注意：本方法只能由通信线程调用（所有 Modbus 读写都集中在线程内）。
        """
        if self.ser is None:
            return False
        ok = True
        try:
            resp_r = self.ser.write_register(
                address=REG_STOP_1, value=int(right_stop), device_id=self.slave_id)
            if resp_r is None or resp_r.isError():
                ok = False
            resp_l = self.ser.write_register(
                address=REG_STOP_2, value=int(left_stop), device_id=self.slave_id)
            if resp_l is None or resp_l.isError():
                ok = False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"写入停止寄存器失败: {exc}")
            return False
        return ok


    # 公开控制接口

    def quick_stop(self):
        """
        请求急停。只置状态标志，实际通讯（写停止寄存器）由通信线程统一执行。
        """
        self._estop_requested = True

    def reset(self):
        """
        请求复位。只置状态标志，实际通讯（正常停止 + 速度清零）由通信线程统一执行。
        用于解除急停、清除停止状态并回到可运行状态。
        """
        self._reset_requested = True

    def read_encoder_counts(self):
        """
        读取左右轮编码器值（换向次数，32 位有符号计数器）。
        返回 dict：{"left_counts":..., "right_counts":...,
                    "left_rad":..., "right_rad":...}
        """
        with self._lock:
            return {
                "left_counts": self.l_pos_counts,
                "right_counts": self.r_pos_counts,
                "left_rad": self.l_pos_rad,
                "right_rad": self.r_pos_rad,
            }

    def read_wheel_rpm(self):
        """读取左右轮实时转速(RPM)。返回 dict：{"left_rpm":..., "right_rpm":...}"""
        with self._lock:
            return {"left_rpm": self.l_rpm, "right_rpm": self.r_rpm}


    # ROS2 回调
    def _joy_state_callback(self, msg: JoyState):
        """遥控触发的各个通道的数值回调"""
        raw_channels = [
            msg.ch01, msg.ch02, msg.ch03, msg.ch04, msg.ch05,
            msg.ch06, msg.ch07, msg.ch08, msg.ch09, msg.ch10,
        ]
        switch06 = msg.switch06
        switch07 = msg.switch07
        self._current_target_pulses = raw_channels[0]
        # 值改变发送一次标志位
        # if self._pos_last_ != self._current_target_pulses and self._pos_change_trig == False:
        #     self._pos_change_trig = True
        #     self._pos_last_ = self._current_target_pulses
        #     self.get_logger().info(f'目标位置={raw_channels[0]}')
        # if not switch06 :
        #     self._button_preess_status = False
        # if switch06 and not self._button_preess_status:
        #     # 开始标定
        #     # self._start_calibration()
        #     # 航向修正标定（按钮触发 -> 保持航向 -> 自动保存归中补偿）
        #     # self._finalize_calibration()
        #     # 先不进行标定
        #     self.get_logger().info(f'6 标定开关打开={switch06}')
        #     self._button_preess_status = True

    def rear_wheel_callback(self, msg: Float64MultiArray):
        """
        接收后轮速度指令。
        msg.data = [右轮, 左轮]；单位由参数 input_unit 决定：
          * 'rad_s'：轮子角速度(rad/s) —— 与阿克曼控制器常规输出一致（默认）
          * 'm_s'  ：轮子线速度(m/s)
        内部统一换算成 RPM，并做方向校正与最高线速度限幅。
        """
        if len(msg.data) < 2:
            self.get_logger().warn("rear_wheel_cmd 数据长度不足 2，忽略")
            return

        right_in = float(msg.data[0])
        left_in = float(msg.data[1])

        # 单位换算 -> 轮子转速(RPM)
        if self.input_unit == "m_s":
            right_rpm = self.linear_velocity_to_rpm(right_in)
            left_rpm = self.linear_velocity_to_rpm(left_in)
        else:  # 默认 'rad_s'
            right_rpm = self.rad_per_sec_to_rpm(right_in)
            left_rpm = self.rad_per_sec_to_rpm(left_in)

        # 方向校正（电机实际安装方向可能相反，通过配置 ±1 校正）
        right_rpm *= self.right_dir_sign
        left_rpm *= self.left_dir_sign

        # 最高线速度限幅（低速车，默认 0.5 m/s）
        right_rpm = max(-self.max_rpm, min(self.max_rpm, right_rpm))
        left_rpm = max(-self.max_rpm, min(self.max_rpm, left_rpm))

        # 取整（寄存器只能写整数 RPM），并仅在变化时置写入标志
        right_rpm = int(round(right_rpm))
        left_rpm = int(round(left_rpm))

        with self._lock:
            changed = (right_rpm != self.target_r_rpm
                       or left_rpm != self.target_l_rpm)
            self.target_r_rpm = right_rpm
            self.target_l_rpm = left_rpm
            if changed:
                self._speed_dirty = True

    def steer_feedback_callback(self, msg: JointState):
        """接收前轮转向反馈，记录前轮转向角(rad)。"""
        try:
            # front_steering 发布的 JointState: position=[左后, 左前, 右后, 右前]
            # 前轮转向角在 position[1]（左前）与 position[3]（右前）。
            if msg.position:
                self.steer_angle = float(msg.position[1])
        except Exception:  # noqa: BLE001
            pass

    def emergency_stop_callback(self, msg: Bool):
        """急停话题回调：收到 True 时请求一次急停。"""
        if msg.data:
            self._estop_requested = True

    def reset_callback(self, msg: Bool):
        """复位话题回调：收到 True 时请求一次复位。"""
        if msg.data:
            self._reset_requested = True

    def _publish_feedback(self):
        """定时发布合并后的关节反馈（供里程计/阿克曼控制器使用）。"""
        with self._lock:
            # 反馈转速 -> 角速度(rad/s)，并应用方向校正使其与物理方向一致。
            v_l = self.rpm_to_rad_per_sec(self.l_rpm) * self.left_dir_sign
            v_r = self.rpm_to_rad_per_sec(self.r_rpm) * self.right_dir_sign
            pos_l = self.l_pos_rad
            pos_r = self.r_pos_rad
            steer = self.steer_angle

        joint = JointState()
        joint.header.stamp = self.get_clock().now().to_msg()
        joint.name = ["lh_joint", "lq_joint", "rh_joint", "rq_joint"]
        # position: [左后角度, 左前转向角, 右后角度, 右前转向角]
        joint.position = [pos_l, steer, pos_r, steer]
        # velocity: [左后轮角速度, 左前轮0, 右后轮角速度, 右前轮0]
        joint.velocity = [v_l, 0.0, v_r, 0.0]
        self.joint_feedback_pub.publish(joint)

    # 通信线程
    def _comm_loop(self):
        """串口通信主循环：本线程是唯一进行 Modbus 读写的线程。"""
        interval = 1.0 / max(self.loop_rate, 1.0)
        log_counter = 0
        while not self._stop_event.is_set():
            # 退出前处理最终停车
            if self._shutdown_requested:
                break
            try:
                self._process_commands()
                self._read_feedback()
                # 状态日志更新部分，后期运行可删除
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"通信循环异常: {exc}")

            self._stop_event.wait(interval)

        # 线程退出前的最后一次停车（仍在本线程内完成，避免外部直接操作串口）。
        if self._shutdown_requested:
            try:
                self._write_speed(0, 0)
                self._write_stop(STOP_NORMAL, STOP_NORMAL)
            except Exception:  # noqa: BLE001
                pass
        self._thread_done.set()

    def _process_commands(self):
        """按优先级处理命令标志，所有 Modbus 写入都在本方法（通信线程）内完成。"""
        # 1) 急停优先
        if self._estop_requested:
            self._estop_requested = False
            self._do_quick_stop()
            return
        # 2) 复位（复位会同时清零速度）
        if self._reset_requested:
            self._reset_requested = False
            self._do_reset()
            return
        # 3) 上电初始化（正常停止 + 速度清零）
        if self._startup_requested:
            self._startup_requested = False
            self._write_stop(STOP_NORMAL, STOP_NORMAL)
            self._write_speed(0, 0)
            self.get_logger().info("上电初始化：停止电机，目标速度清零")
            return
        # 4) 速度指令变化时写入（降低总线负载）

        if self._speed_dirty:
            if self._write_speed(self.target_l_rpm, self.target_r_rpm):
                self._last_write_l_rpm = self.target_l_rpm
                self._last_write_r_rpm = self.target_r_rpm
                self._speed_dirty = False

    def _do_quick_stop(self):
        """急停实际写入（仅通信线程调用）。"""
        ok = self._write_stop(STOP_EMERGENCY, STOP_EMERGENCY)
        if ok:
            self.get_logger().warn("已执行急停（左右电机紧急停止）")

    def _do_reset(self):
        """复位实际写入（仅通信线程调用）。"""
        ok_stop = self._write_stop(STOP_NORMAL, STOP_NORMAL)
        ok_speed = self._write_speed(0, 0)
        if ok_stop and ok_speed:
            with self._lock:
                self.target_l_rpm = 0
                self.target_r_rpm = 0
                self._last_write_l_rpm = 0
                self._last_write_r_rpm = 0
                self._speed_dirty = False
            self.get_logger().info("已复位：恢复正常停止，目标速度清零")

    def _read_feedback(self):
        """
        一次连续读取 0x0009~0x001C 共 20 个寄存器，
        解析左右电机的实时转速、错误状态、换向次数(编码器)。
        """
        if self.ser is None:
            return
        try:
            resp = self.ser.read_holding_registers(
                address=REG_READ_BASE, count=REG_READ_COUNT,
                device_id=self.slave_id)
            if resp is None or resp.isError():
                return
            regs = resp.registers
            if len(regs) < REG_READ_COUNT:
                return

            # 右电机
            r_rpm = self._words_to_s32(regs[IDX_R_SPEED_H], regs[IDX_R_SPEED_L])
            r_error = int(regs[IDX_R_ERROR])
            r_counts = self._words_to_s32(regs[IDX_R_POS_H], regs[IDX_R_POS_L])
            # self.get_logger().INFO(f"右电机: {r_rpm}{r_error}{r_counts}")
            # 左电机
            l_rpm = self._words_to_s32(regs[IDX_L_SPEED_H], regs[IDX_L_SPEED_L])
            l_error = int(regs[IDX_L_ERROR])
            l_counts = self._words_to_s32(regs[IDX_L_POS_H], regs[IDX_L_POS_L])
            self.get_logger().INFO(f"左电机: {l_rpm}{l_error}{l_counts}--右电机: {r_rpm}{r_error}{r_counts}")

            with self._lock:
                self.r_rpm = float(r_rpm)
                self.l_rpm = float(l_rpm)
                self.r_error = r_error
                self.l_error = l_error

                # 累计机械角度（换向次数 -> 弧度），并做 32 位溢出解卷绕。
                self.r_pos_counts = r_counts
                self.l_pos_counts = l_counts
                if self._last_r_counts is not None:
                    self.r_pos_rad += self._commutation_delta_to_rad(
                        r_counts - self._last_r_counts)
                if self._last_l_counts is not None:
                    self.l_pos_rad += self._commutation_delta_to_rad(
                        l_counts - self._last_l_counts)
                self._last_r_counts = r_counts
                self._last_l_counts = l_counts

                # 错误状态变化时打印日志，便于排查堵转/过流等问题（避免每周期刷屏）。
                if r_error != 0 and r_error != self._last_logged_r_error:
                    self.get_logger().warn(
                        f"右电机错误: {ERROR_TEXT.get(r_error, r_error)}")
                    self._last_logged_r_error = r_error
                if l_error != 0 and l_error != self._last_logged_l_error:
                    self.get_logger().warn(
                        f"左电机错误: {ERROR_TEXT.get(l_error, l_error)}")
                    self._last_logged_l_error = l_error
                if r_error == 0:
                    self._last_logged_r_error = 0
                if l_error == 0:
                    self._last_logged_l_error = 0
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"读取反馈失败: {exc}")

    def _commutation_delta_to_rad(self, delta_counts: int) -> float:
        """
        换向次数增量 -> 机械角度增量(rad)。
        每 1 次换向 = 60 度电气角 = 60/pole_pairs 度机械角。
        """
        return delta_counts * (math.pi / 3.0) / self.pole_pairs

    # =====================================================================
    # 节点销毁
    # =====================================================================
    def destroy_node(self):
        """重写节点销毁方法，确保线程与串口被正确释放。"""
        self.get_logger().info("正在停止电机控制线程...")
        # 通过标志请求通信线程做最后一次停车（所有通讯都在线程内完成）。
        self._shutdown_requested = True
        # 等待通信线程完成最终停车并退出（最多约 1 秒）。
        if self._comm_thread.is_alive():
            self._thread_done.wait(timeout=1.0)
        self.close_port()
        super().destroy_node()


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = channel_MBRTU()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
