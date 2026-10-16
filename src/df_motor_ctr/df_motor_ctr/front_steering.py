#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
df_motor_ctr 前轮转向驱动节点

驱动对象：ZDT 闭环步进电机（Emm 固件），MODBUS RTU 协议。
寄存器地址已按《ZDT闭环步进电机MODBUS协议使用说明V1.0.1》确认。

主要功能：
1. MODBUS RTU 驱动前轮转向步进电机；
2. 上电使能 -> 无限位碰撞回零 -> 归中定位（补偿参数 center_pos）；
3. 订阅 /hardware/front_steering_cmd（弧度）并换算为目标脉冲位置；
4. IMU 航向保持 PID（移植自 wheel_dir_pwm.py）；
5. 航向修正标定：遥控按钮触发 -> 锁定当前航向 -> PID 保持 ->
   稳定后把当前修正量折算进 center_pos 并自动保存，掉电不丢。

前轮转向的机械零点位于左转向极限，因此回零完成后必须再按 center_pos
运动到"车轮回正"位置。center_pos 即本节点的归中补偿参数。·
"""

import math
import threading
import time
from pathlib import Path

import rclpy
from pymodbus.client import ModbusSerialClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import Bool, Float64, Float64MultiArray


# ---- ZDT(Emm) MODBUS 寄存器地址 ----
REG_ENABLE = 0xF3        # 电机使能（2 寄存器）
REG_POS_PARAMS = 0xF1    # 位置模式参数：方向/加速度、速度、运动模式/同步标志
REG_POS_TARGET = 0xFC    # 快速位置模式目标位置（有符号 int32 脉冲数，高字在前）
REG_HOME_TRIGGER = 0x9A  # 触发回零（回零模式 << 8 | 同步标志）
REG_HOME_STATUS = 0x3B   # 回零状态标志
REG_STATUS = 0x3A        # 电机状态标志
REG_POSITION = 0x36      # 实时位置（符号 + 16bit，65536 = 360°）
REG_STOP = 0xFE          # 立即停止
REG_CLEAR_ERROR = 0x0E   # 解除堵转/过热/过流保护

# 回零状态标志位 (0x3B)
HOME_ORG_SF = 0x04  # 正在回零
HOME_ORG_CF = 0x08  # 回零失败

# 电机状态标志位 (0x3A)
STATUS_ENS = 0x01   # 使能
STATUS_PRF = 0x02   # 位置到达
STATUS_CGI = 0x04   # 堵转
STATUS_CGP = 0x08   # 堵转保护

# 1、上电或者重起端口后进行如下顺序操作
#  使能                                             订阅阿克曼控制器发布的前轮转向角
#  启动寻零                                         将前轮转向角转换成角度，并且写入到前轮转向线程中
#  设置定位参数
#
#  进入线程
#
#  循环读取位置角度
#  将角度通过前轮转向映射表计算出前轮转向角度
#  赋值给属性值


class FrontSteeringNode(Node):
    """前轮转向驱动 + 航向保持 + 归中补偿节点。"""





    def __init__(self):
        super().__init__('df_front_steering')

        # ==================== 串口参数 ====================
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('slave_id', 1)

        # ==================== 位置 / 归中补偿参数 ====================
        # center_pos: 车轮回正时对应的电机绝对位置（脉冲数）。零点=左转向极限。
        self.declare_parameter('center_pos', 1600)
        # 每度转向角对应的电机脉冲数。默认 16 细分：3200 脉冲/圈 => 3200/360。
        # 若转向机构有减速比，按实车标定后修改。
        self.declare_parameter('pulses_per_degree', 3200.0 / 360.0)
        self.declare_parameter('steering_angle_limit_deg', 30.0)
        self.declare_parameter('steering_sign', 1.0)  # 转向方向反了改成 -1.0
        self.declare_parameter('pos_speed_rpm', 100)    # 定位速度
        self.declare_parameter('pos_accel', 150)    # 定位加速度
        self.declare_parameter('home_mode', 2)  # 2 = 无限位碰撞回零··

        # ====================· 航向修正标定参数 ====================
        self.declare_parameter('calibration_button', 6)  # /joy 按钮索引··
        self.declare_parameter('calibration_settle_time', 2.0)
        self.declare_parameter(
            'calibration_file',
            '~/.df_motor_ctr/front_steering_center.yaml',
        )

        # ==================== IMU 与航向 PID 参数 ====================
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('heading_control_enabled', True)# 航向保持总开关
        self.declare_parameter('straight_steering_threshold', 0.03)# 转向角在 [-0.1, 0.1]
        self.declare_parameter('heading_settle_time', 1.0)# 转弯后重新直行时，等待航向稳定再锁定参考航向的时间(s)
        self.declare_parameter('imu_timeout', 0.5)# 超过该时间没有收到 IMU 消息则判定 IMU 故障(s)
        self.declare_parameter('yaw_jump_max_deg', 8.0)# 单帧 yaw 跳变超过该角度则视为异常数据(°)
        self.declare_parameter('imu_invalid_frames_before_fault', 10)# 连续异常帧数达到该值后判定 IMU 故障
        self.declare_parameter('imu_recover_time', 1.0)# IMU 连续正常多长时间后才允许恢复 PID(s)
        self.declare_parameter('max_yaw_error_deg', 10.0)# 直行偏航误差超过该值时关闭 PID，使用原始转向(°)
        self.declare_parameter('heading_kp', 0.8)# PID 比例系数：1°偏航 -> 0.6°转向修正
        self.declare_parameter('heading_ki', 0.001)# PID 积分系数··，通常先设 0，有稳态误差再调
        self.declare_parameter('heading_kd', 0.0)# PID 微分系数，通常先设 0
        self.declare_parameter('heading_correction_sign', -1.0)# 修正方向；方向反了改成 +1.0
        self.declare_parameter('heading_correction_limit_deg', 8.0)# 单次航向修正最大角度(°)
        self.declare_parameter('heading_integral_limit_deg', 5.0)# 积分项限幅，防止积分饱和````
        self.declare_parameter('yaw_median_window', 9)
        self.declare_parameter('yaw_lowpass_alpha', 0.2)

        # ---- 读取串口参数 ----
        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._timeout = self.get_parameter('timeout').value
        self._slave_id = int(self.get_parameter('slave_id').value)

        # ---- 读取位置参数 ----
        self.center_pos = int(self.get_parameter('center_pos').value)
        self._pulses_per_degree = float(self.get_parameter('pulses_per_degree').value)
        self._angle_limit_deg = float(self.get_parameter('steering_angle_limit_deg').value)
        self._steering_sign = float(self.get_parameter('steering_sign').value)
        self._pos_speed_rpm = int(self.get_parameter('pos_speed_rpm').value)
        self._pos_accel = int(self.get_parameter('pos_accel').value)
        self._home_mode = int(self.get_parameter('home_mode').value)

        # ---- 读取航向修正参数 ----
        self._calibration_button = int(self.get_parameter('calibration_button').value)
        self._calibration_settle_time = float(
            self.get_parameter('calibration_settle_time').value)
        self._calibration_file = self.get_parameter('calibration_file').value
        # 若之前标定过，则用保存的归中值覆盖默认值
        self.center_pos = self._load_center_pos(self.center_pos)

        # ---- 读取 IMU / PID 参数 ----
        self._imu_topic = self.get_parameter('imu_topic').value
        self._heading_control_enabled = bool(
            self.get_parameter('heading_control_enabled').value)
        self._straight_steering_threshold = float(
            self.get_parameter('straight_steering_threshold').value)
        self._heading_settle_time = float(
            self.get_parameter('heading_settle_time').value)
        self._imu_timeout = float(self.get_parameter('imu_timeout').value)
        self._yaw_jump_max_deg = float(
            self.get_parameter('yaw_jump_max_deg').value)
        self._imu_invalid_frames_before_fault = int(
            self.get_parameter('imu_invalid_frames_before_fault').value)
        self._imu_recover_time = float(self.get_parameter('imu_recover_time').value)
        self._max_yaw_error_deg = float(self.get_parameter('max_yaw_error_deg').value)
        self._heading_kp = float(self.get_parameter('heading_kp').value)
        self._heading_ki = float(self.get_parameter('heading_ki').value)
        self._heading_kd = float(self.get_parameter('heading_kd').value)
        self._heading_correction_sign = float(
            self.get_parameter('heading_correction_sign').value)
        self._heading_correction_limit_deg = float(
            self.get_parameter('heading_correction_limit_deg').value)
        self._heading_integral_limit_deg = float(
            self.get_parameter('heading_integral_limit_deg').value)
        self._yaw_median_window = int(self.get_parameter('yaw_median_window').value)
        self._yaw_lowpass_alpha = float(self.get_parameter('yaw_lowpass_alpha').value)

        # ---- 运行状态 ----
        self.ser = None
        self.state = 'close'
        self.running = False
        self.ready = False  # 回零 + 归中完成，允许响应转向指令
        self.home_state = 'idle'  # idle / homing / ready / failed
        self._saw_homing_active = False
        self._home_trigger_time = None
        self._home_retry_count = 0
        self._max_home_retry = 3

        # 转向命令状态
        self._cmd_angle_rad = 0.0
        self._last_heading_correction_deg = 0.0
        self._current_target_pulses = None
        self._last_sent_pulses = None

        # ---- 单线程串口访问的请求标志（只有通讯线程会读写串口） ----
        self._req_enable = None      # None=无请求, True=使能, False=断使能
        self._req_pos_params = False  # 请求设置定位参数
        self._req_home = False
        self._req_stop = False
        self._req_clear_error = False
        self._pos_dirty = False      # 是否有新目标位置待写入

        # 实时前轮转向角（通讯线程读取并发布）
        self._real_steering_deg = 0.0
        self._real_steering_rad = 0.0
        self._real_pos_valid = False

        # ---- IMU / 航向 PID 状态 ----
        self._imu_received = False
        self._current_yaw = None
        self._heading_reference = None
        self._heading_control_active = False
        self._straight_start_time = None
        self._last_imu_time = None
        self._imu_fault = False
        self._imu_invalid_count = 0
        self._imu_recover_start_time = None
        self._last_control_time = None
        self._last_yaw_error = 0.0
        self._yaw_integral = 0.0
        self._yaw_median_queue = []
        self._yaw_lowpass_prev = None

        # ---- 航向修正标定状态 ----
        self._calibrating = False
        self._calib_start_time = None
        self._calib_button_prev = False
        self._calib_bool_prev = False

        # ---- 串口线程 ----
        self._stop_event = threading.Event()
        self._comm_thread = threading.Thread(target=self._loop_start, daemon=True)

        # 订阅IMU话题
        self._imu_sub = self.create_subscription(
            Imu, self._imu_topic, self._imu_callback, 10)

        # =====================话题=============================================
        # ---- 订阅，订阅的阿克曼控制器发下的前轮转向角度话题 50HZ ----
        self._steer_sub = self.create_subscription(
            Float64MultiArray,
            '/hardware/front_steering_cmd',
            self._dir_callback,
            10,
        )
        # ---- 发布关节状态到阿克曼控制器反馈节点 通过阿克曼控制器下发的转向角度话题转发 50HZ----
        self.steer_feedback_pub = self.create_publisher(
            JointState, '/hardware/steer_feedback', 10)

        # =====================手柄节点=============================================
        # 订阅 df_joy_bridge中的手柄触发节点，事件驱动类型，只有切换的时候才进行发送，负载更小····
        self._calib_sub = self.create_subscription(
            Bool, '/heading_calibration', self._calibration_bool_callback, 10)

        # # ---- 标定完成检测定时器（20Hz） ----
        # self._calib_timer = self.create_timer(0.05, self._calibration_tick)

        # ---- 打开串口并启动 ----
        self.get_logger().info(f'前轮转向节点初始化，归中补偿 center_pos={self.center_pos}')
        self.close_port()
        self.open_port()

    # ==================================================================
    # 串口打开 / 关闭··
    # ==================================================================
    def open_port(self):
        """打开串口：使能 -> 配置位置模式 -> 触发回零 -> 启动线程。"""
        if self.state != 'close':
            self.get_logger().info('转向串口已打开')
            return False
        try:
            self.ser = ModbusSerialClient(
                port=self._port,
                baudrate=self._baudrate,
                parity='N',
                stopbits=1,
                bytesize=8,
            )
            self.ser.connect()
            time.sleep(0.1)
            self.state = 'open'
            self.get_logger().info(f'前轮转向串口 {self._port} 打开成功')

            # 所有串口写入都通过请求标志交给通讯线程，保证串口单线程访问
            self._req_enable = True
            self._req_pos_params = True
            self._req_home = True
            self.get_logger().info('启动无限位碰撞回零...')
            self._comm_thread.start()
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'前轮转向串口打开失败: {exc}')
            return False

    # =====================端口操作=============================================
    def close_port(self):
        """关闭串口。"""
        if self.state == 'close':
            return
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        finally:
            self.ser = None
            self.state = 'close'
    def stop(self):
        """停止通讯线程。"""
        self.running = False
        self.state = 'close'
    """错误处理"""
    def _quick_stop(self):
        """立即停止（0xFE: 0x98 << 8 | 同步标志）。"""
        try:
            self.ser.write_register(
                address=REG_STOP, value=0x9800, device_id=self._slave_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'急停失败: {exc}')
    def _clear_error(self):
        """解除堵转/过热/过流保护（0x0E = 1）。"""
        try:
            self.ser.write_register(
                address=REG_CLEAR_ERROR, value=1, device_id=self._slave_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'清除保护失败: {exc}')

    # ==================== 底层modbus指令 =====================================
    def _read_real_position(self):
        """读取实时位置（0x36），返回带符号的角度（度）。失败返回 None。"""
        regs = self._read_regs(REG_POSITION, 3)
        if regs is None or len(regs) < 3:
            return None
        sign, high, low = regs[0], regs[1], regs[2]
        raw = ((high << 8) | low) & 0xFFFF
        angle = raw * 360.0 / 65536.0
        return -angle if sign else angle
    def _read_regs(self, address, count):
        """读取保持寄存器，失败返回 None。"""
        if self.ser is None:
            return None
        try:
            resp = self.ser.read_holding_registers(
                address=address, count=count, device_id=self._slave_id)
            if resp is None or resp.isError():
                return None
            return list(resp.registers)
        except Exception:  # noqa: BLE001
            return None

    # ==================== 初始化指令 =====================================
    def _enable_motor(self, flag):
        """使能/失能电机（0xF3: 0xAB 开头 + 状态 + 同步标志）。"""
        try:
            self.ser.write_registers(
                address=REG_ENABLE,
                values=[(0xAB << 8) | (1 if flag else 0), 0x0000],
                device_id=self._slave_id,
            )
            if flag:
                self.get_logger().info(f'使能电机')
            else:
                self.get_logger().info(f'断使能电机')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'写入使能失败: {exc}')
    def _set_pos_params(self):
        """设定位置模式的速度/加速度/运动模式/同步标志（0xF1）。"""
        # reg1 = 方向(0) + 加速度；reg2 = 速度；reg3 = 运动模式(1=绝对位置) + 同步(0)
        try:
            self.ser.write_registers(
                address=REG_POS_PARAMS,
                values=[self._pos_accel & 0xFF,
                        self._pos_speed_rpm & 0xFFFF,
                        (0x01 << 8) | 0x00],
                device_id=self._slave_id,
            )
            self.get_logger().info(f'设置速度/加速度/运动模式')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'设定位置模式参数失败: {exc}')
    def _trigger_home(self):
        """触发回零。mode 2 = 无限位碰撞回零。"""
        try:
            self.ser.write_register(
                address=REG_HOME_TRIGGER,
                value=(self._home_mode << 8) | 0x00,
                device_id=self._slave_id,
            )
            self.home_state = 'homing'
            self._saw_homing_active = False
            self._home_trigger_time = time.time()
            self.ready = False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'触发回零失败: {exc}')
            self.home_state = 'failed'

    # ==================== 定位指令 =====================================
    @staticmethod
    def _int32_to_words(value):
        """有符号 int32 转两个 MODBUS 寄存器，高字在前。"""
        value = int(value)
        if value < 0:
            value += 1 << 32
        value &= 0xFFFFFFFF
        return [(value >> 16) & 0xFFFF, value & 0xFFFF]
    """启动定位"""
    def _set_position(self, pulses):
        """写入目标绝对位置（0xFC，有符号 int32 脉冲数）。"""
        pulses = int(pulses)
        if pulses == self._last_sent_pulses:
            return
        try:
            self.ser.write_registers(
                address=REG_POS_TARGET,
                values=self._int32_to_words(pulses),
                device_id=self._slave_id,
            )
            self._last_sent_pulses = pulses
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'写入目标位置失败: {exc}')

    def _handle_homing_state(self):
        """回零状态机。"""
        # 如果不是在寻零过程中，就return··
        if self.home_state != 'homing':
            return
        # 回零状态判断
        home_status = self._read_regs(REG_HOME_STATUS, 1)
        if home_status is None:
            return
        org_state = home_status[0] & (HOME_ORG_SF | HOME_ORG_CF)

        # 如果电机正在回零中
        if self.home_state == 'homing':
            if org_state == HOME_ORG_SF:
                self.get_logger().warn(
                    f'回零成功')
                self._saw_homing_active = True
                return
            if org_state == HOME_ORG_CF:
                self.get_logger().error('回零失败')
                if self._home_retry_count < self._max_home_retry:
                    self._home_retry_count += 1
                    self.get_logger().warn(
                        f'回零失败，第 {self._home_retry_count} 次重试...')
                    time.sleep(0.5)
                    self._trigger_home()
                else:
                    self.home_state = 'failed'
                return

            # org_state == 0x00：可能是"未开始"或"已完成"。
            # 需看到过"正在回零"，或距触发超过 0.5s，才判定完成。
            if self._saw_homing_active:
                self.get_logger().info('回零完成，执行归中定位')
                self._set_position(self.center_pos)
                self.home_state = 'ready'
                self.ready = True # 回零归中完成，允许后续定位指令
                self._last_sent_pulses = None # 最后一次的脉冲数
                self.get_logger().info(f'前轮转向就绪，归中位置={self.center_pos}')
                self._saw_homing_active = False # 只写入一次

    def _process_write_requests(self):
        """处理一次性写请求与位置目标写入（仅通讯线程调用）。"""
        # 优先级：急停 > 清错 > 使能 > 定位参数 > 回零
        if self._req_stop:
            self._req_stop = False
            self._quick_stop()
        if self._req_clear_error:
            self._req_clear_error = False
            self._clear_error()
        # 使能
        if self._req_enable is not None:
            flag = self._req_enable
            self._req_enable = None
            self._enable_motor(flag)
        if self._req_pos_params:
            self._req_pos_params = False
            self._set_pos_params()
        if self._req_home:
            self._req_home = False
            self._trigger_home()

        # 位置目标（电平量）：ready 且有新目标时写入最新值
        # self.ready = True  # 回零归中完成，允许后续定位指令
        # self._pos_dirty = True  # 是否有新目标写入
        if self._pos_dirty and self.ready and self._current_target_pulses is not None:
            self._pos_dirty = False
            self._set_position(self._current_target_pulses)

    def _update_real_steering_angle(self):
        """读取实时位置（0x36）并换算为前轮转向角（度/弧度）。"""
        motor_deg = self._read_real_position()
        if motor_deg is None:
            return
        # 电机轴角度 -> 电机脉冲数（默认 16 细分：3200 脉冲/圈）
        motor_pulses = motor_deg * 3200.0 / 360.0
        steering_pulses = motor_pulses - self.center_pos
        steering_deg = steering_pulses / self._pulses_per_degree
        # 前轮转向角，前轮转向弧度
        self._real_steering_deg = steering_deg
        self._real_steering_rad = math.radians(steering_deg)
        # self._real_pos_valid = True

    # ==================================================================
    # 通讯线程：回零状态机 + 状态读取
    # ==================================================================
    def _loop_start(self):
        self.running = True
        log_counter = 0
        while not self._stop_event.is_set():
            if self._stop_event.wait(timeout=0.02):
                break
            if self.ser is None:
                continue
            try:
                # 1. 处理一次性写请求和位置目标写入
                self._process_write_requests()
                # 2. 回零状态机
                self._handle_homing_state()
                # 3. 每个周期读取实时位置，更新前轮转向角
                self._update_real_steering_angle()
                # # 4. 发布实测前轮转向角反馈
                # self._publish_real_feedback()
                log_counter += 1
                if log_counter % 50 == 0:
                    status = self._read_regs(REG_STATUS, 1)
                    if status is not None:
                        status_val = status[0]
                        self.get_logger().info(
                            f'实时转向角={self._real_steering_deg:.2f}°, '
                            f'使能={bool(status_val & STATUS_ENS)}, '
                            f'到位={bool(status_val & STATUS_PRF)}, '
                            f'堵转={bool(status_val & STATUS_CGI)}')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f'通讯循环异常: {exc}')
        # 退出前断开使能（串口仍由本线程独占）
        if self.ser is not None:
            try:
                self._enable_motor(False)
            except Exception:  # noqa: BLE001
                pass
        self.close_port()
        self.get_logger().info('前轮转向通讯线程已停止')


    # ==================================================================
    # 转向指令回调
    # ==================================================================
    def _dir_callback(self, msg: Float64MultiArray):
        """50HZ 接收 /hardware/front_steering_cmd（rad），计算目标位置并下发。"""
        if not msg.data:
            return
        base_rad = (msg.data[0] + msg.data[1]) / 2.0
        self._cmd_angle_rad = base_rad

        # 航向保持 PID 修正（度）
        heading_correction_deg = self._compute_heading_correction(base_rad)

        base_deg = math.degrees(base_rad)
        total_deg = self._steering_sign * (base_deg + heading_correction_deg)
        total_deg = max(-self._angle_limit_deg,
                        min(self._angle_limit_deg, total_deg))

        target_pulses = self.center_pos + int(round(total_deg * self._pulses_per_degree))
        target_pulses = max(0, target_pulses)

        # 只更新目标位置并置脏标志，真正的串口写入由通讯线程统一执行
        self._current_target_pulses = target_pulses
        self._pos_dirty = True
        """发布实测前轮转向角"""
        self._publish_feedback(self._real_steering_rad, self._real_steering_deg)
    def _publish_feedback(self, steer_rad, steer_deg):
        """发布关节反馈与实时转向角。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['lh_joint', 'lq_joint', 'rh_joint', 'rq_joint']
        msg.position = [0.0, steer_rad, 0.0, steer_rad]
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        self.steer_feedback_pub.publish(msg)

    # ===================IMU 与航向保持 PID（移植自 wheel_dir_pwm.py）===============================================
    """IMU回调"""
    def _imu_callback(self, msg: Imu):
        now = self.get_clock().now()
        yaw = self._yaw_from_quaternion(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)
        if yaw is None:
            self._handle_invalid_imu(now)
            return

        if self._current_yaw is not None:
            delta = self._normalize_angle(yaw - self._current_yaw)
            if abs(math.degrees(delta)) > self._yaw_jump_max_deg:
                self._handle_invalid_imu(now)
                return

        yaw_filtered = self._filter_yaw(yaw)
        self._current_yaw = yaw_filtered
        self._accept_valid_imu(now, yaw_filtered)

    @staticmethod
    def _yaw_from_quaternion(x, y, z, w):
        values = (x, y, z, w)
        if not all(math.isfinite(v) for v in values):
            return None
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-6:
            return None
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return math.atan2(2.0 * (w * z + x * y),
                          1.0 - 2.0 * (y * y + z * z))
    @staticmethod
    def _normalize_angle(angle):
        angle = math.fmod(angle + math.pi, 2.0 * math.pi)
        if angle < 0:
            angle += 2.0 * math.pi
        return angle - math.pi
    """IMU滤波器"""
    def _filter_yaw(self, yaw_raw):
        """中值滤波 + 一阶低通滤波。"""
        self._yaw_median_queue.append(yaw_raw)
        window = self._yaw_median_window
        if window > 1:
            if len(self._yaw_median_queue) > window:
                self._yaw_median_queue = self._yaw_median_queue[-window:]
            if len(self._yaw_median_queue) < window:
                median_yaw = yaw_raw
            else:
                base = self._yaw_median_queue[0]
                diffs = sorted(
                    self._normalize_angle(a - base)
                    for a in self._yaw_median_queue)
                median_yaw = self._normalize_angle(base + diffs[len(diffs) // 2])
        else:
            median_yaw = yaw_raw

        if self._yaw_lowpass_prev is None:
            self._yaw_lowpass_prev = median_yaw
            return median_yaw
        delta = self._normalize_angle(median_yaw - self._yaw_lowpass_prev)
        self._yaw_lowpass_prev = self._normalize_angle(
            self._yaw_lowpass_prev + self._yaw_lowpass_alpha * delta)
        return self._yaw_lowpass_prev
    def _handle_invalid_imu(self, now):
        self._last_imu_time = now
        self._imu_invalid_count += 1
        self._imu_recover_start_time = None
        if self._imu_invalid_count >= self._imu_invalid_frames_before_fault:
            self._mark_imu_fault()
    def _accept_valid_imu(self, now, yaw):
        self._current_yaw = yaw
        self._imu_received = True
        self._last_imu_time = now
        self._imu_invalid_count = 0
        if not self._imu_fault:
            self._imu_recover_start_time = None
            return
        if self._imu_recover_start_time is None:
            self._imu_recover_start_time = now
        elapsed = (now - self._imu_recover_start_time).nanoseconds / 1e9
        if elapsed >= self._imu_recover_time:
            self._imu_fault = False
            self._imu_recover_start_time = None
            self.get_logger().info('IMU 已恢复，允许重新启用航向保持')
    def _mark_imu_fault(self):
        if self._imu_fault:
            return
        self._imu_fault = True
        self._imu_recover_start_time = None
        self._disable_heading_control()
        self.get_logger().warn('检测到 IMU 异常，关闭航向 PID')

    # ===================IMU 与航向保持 PID（移植自 wheel_dir_pwm.py）===============================================
    def _disable_heading_control(self):
        self._heading_control_active = False
        self._heading_reference = None
        self._straight_start_time = None
        self._reset_heading_pid()

    def _reset_heading_pid(self):
        self._last_control_time = None
        self._last_yaw_error = 0.0
        self._yaw_integral = 0.0

    # ===================航向保持计算=======================
    def _compute_heading_correction(self, steering_rad):
        """计算直线航向保持修正量（度）。标定模式下强制保持锁定航向。"""
        now = self.get_clock().now()

        if not self._heading_control_enabled:
            self._disable_heading_control()
            self._last_heading_correction_deg = 0.0
            return 0.0
        if self._imu_fault:
            self._disable_heading_control()
            self._last_heading_correction_deg = 0.0
            return 0.0
        if not self._imu_received or self._current_yaw is None:
            self._disable_heading_control()
            self._last_heading_correction_deg = 0.0
            return 0.0
        if self._last_imu_time is not None:
            elapsed = (now - self._last_imu_time).nanoseconds / 1e9
            if elapsed > self._imu_timeout:
                self._mark_imu_fault()
                self._last_heading_correction_deg = 0.0
                return 0.0

        is_straight = abs(steering_rad) <= self._straight_steering_threshold
        if self._calibrating:
            is_straight = True
            if not self._heading_control_active:
                self._heading_reference = self._current_yaw
                self._heading_control_active = True
                self._straight_start_time = None
                self._reset_heading_pid()
                self._last_control_time = now
                self._last_heading_correction_deg = 0.0
                return 0.0

        if not is_straight:
            self._disable_heading_control()
            self._last_heading_correction_deg = 0.0
            return 0.0

        if not self._heading_control_active:
            if self._straight_start_time is None:
                self._straight_start_time = now
            settle = (now - self._straight_start_time).nanoseconds / 1e9
            if settle < self._heading_settle_time:
                self._last_heading_correction_deg = 0.0
                return 0.0
            self._heading_reference = self._current_yaw
            self._heading_control_active = True
            self._straight_start_time = None
            self._reset_heading_pid()
            self._last_control_time = now
            self._last_heading_correction_deg = 0.0
            return 0.0

        if self._last_control_time is None:
            self._last_control_time = now
            self._last_heading_correction_deg = 0.0
            return 0.0

        dt = (now - self._last_control_time).nanoseconds / 1e9
        if dt <= 0.0:
            self._last_heading_correction_deg = 0.0
            return 0.0
        if dt > 0.3:
            dt = 0.3

        yaw_error_rad = self._normalize_angle(
            self._current_yaw - self._heading_reference)
        yaw_error_deg = math.degrees(yaw_error_rad)
        if abs(yaw_error_deg) > self._max_yaw_error_deg:
            self._mark_imu_fault()
            self._last_heading_correction_deg = 0.0
            return 0.0

        self._yaw_integral += yaw_error_deg * dt
        int_limit = self._heading_integral_limit_deg
        self._yaw_integral = max(-int_limit, min(int_limit, self._yaw_integral))
        derivative = (yaw_error_deg - self._last_yaw_error) / dt
        self._last_yaw_error = yaw_error_deg

        pid = (self._heading_kp * yaw_error_deg
               + self._heading_ki * self._yaw_integral
               + self._heading_kd * derivative)
        correction = self._heading_correction_sign * pid
        limit = self._heading_correction_limit_deg
        correction = max(-limit, min(limit, correction))

        self._last_control_time = now
        self._last_heading_correction_deg = correction
        return correction

    # ==================遥控触发 保存航向修正量 ================================================
    def _calibration_bool_callback(self, msg: Bool):
        pressed = bool(msg.data)
        if pressed and not self._calib_bool_prev:
            # 开始标定
            # self._start_calibration()
            # 航向修正标定（按钮触发 -> 保持航向 -> 自动保存归中补偿）
            self._finalize_calibration()
        self._calib_bool_prev = pressed

    def _finalize_calibration(self):
        if self._imu_fault or not self._imu_received:
            self.get_logger().warn('航向修正失败：IMU 不可用，未保存补偿')
            self._calibrating = False
            self._calib_start_time = None
            return

        # 把当前 PID 修正量折算成脉冲偏移，累加到 center_pos
        offset = int(round(
            self._steering_sign * self._last_heading_correction_deg
            * self._pulses_per_degree))
        new_center = max(0, self.center_pos + offset)

        self.center_pos = new_center
        self.set_parameters([
            Parameter('center_pos', Parameter.Type.INTEGER, new_center)])
        self._persist_center_pos()

        # 归中补偿已更新，重新锁定航向并复位 PID
        self._calibrating = False
        self._calib_start_time = None
        self._heading_reference = self._current_yaw
        self._reset_heading_pid()
        self._last_control_time = self.get_clock().now()
        self.get_logger().info(
            f'航向修正完成：center_pos {self.center_pos - offset} -> '
            f'{self.center_pos}（偏移 {offset}）')

    # ===================读取和保存补偿位置===============================================
    """读取补偿位置"""
    def _load_center_pos(self, default):
        path = Path(self._calibration_file).expanduser()
        if not path.exists():
            return default
        try:
            for line in path.read_text(encoding='utf-8').splitlines():
                line = line.strip()
                if line.startswith('center_pos:'):
                    return int(line.split(':', 1)[1].strip())
        except Exception:  # noqa: BLE001
            pass
        return default

    """保存补偿位置"""
    def _persist_center_pos(self):
        path = Path(self._calibration_file).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f'center_pos: {int(self.center_pos)}\n',
                            encoding='utf-8')
            self.get_logger().info(f'归中补偿已保存到 {path}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'保存归中补偿失败: {exc}')

    # ==================================================================
    # 节点销毁
    # ==================================================================
    def destroy_node(self):
        self.get_logger().info('正在停止前轮转向节点...')
        self._stop_event.set()
        if self._comm_thread.is_alive():
            self._comm_thread.join(timeout=2.0)
        self.close_port()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrontSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
