#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

import serial
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class JoyToServoNode(Node):
	def __init__(self):
		super().__init__('joy_to_servo_node')

		# ---- 声明可配置参数 ----
		self.declare_parameter('port', '/dev/ttyUSB0')
		self.declare_parameter('baudrate', 115200)
		self.declare_parameter('timeout', 0.5)

		port = self.get_parameter('port').value
		baudrate = self.get_parameter('baudrate').value
		timeout = self.get_parameter('timeout').value

		# ---- 声明 IMU 航向 PID 相关参数 ----
		# 这些参数可通过 launch/yaml 或命令行覆盖，方便实车调试时调整。
		self.declare_parameter('imu_topic', '/imu/data_raw')  # 订阅哪个 IMU 话题
		self.declare_parameter('heading_control_enabled', True)  # 航向保持总开关
		self.declare_parameter('straight_steering_threshold', 0.1)  # 转向角在 [-0.1, 0.1] rad 内判定为直行并启用 PID
		self.declare_parameter('heading_settle_time', 0.2)  # 转弯后重新直行时，等待航向稳定再锁定参考航向的时间(s)
		self.declare_parameter('imu_timeout', 0.5)  # 超过该时间没有收到 IMU 消息则判定 IMU 故障(s)
		self.declare_parameter('yaw_jump_max_deg', 30.0)  # 单帧 yaw 跳变超过该角度则视为异常数据(°)
		self.declare_parameter('imu_invalid_frames_before_fault', 10)  # 连续异常帧数达到该值后判定 IMU 故障
		self.declare_parameter('imu_recover_time', 1.0)  # IMU 连续正常多长时间后才允许恢复 PID(s)
		self.declare_parameter('max_yaw_error_deg', 15.0)  # 直行偏航误差超过该值时关闭 PID，使用原始转向(°)
		self.declare_parameter('heading_kp', 0.6)  # PID 比例系数：1°偏航 -> 0.6°转向修正
		self.declare_parameter('heading_ki', 0.0)  # PID 积分系数，通常先设 0，有稳态误差再调
		self.declare_parameter('heading_kd', 0.0)  # PID 微分系数，通常先设 0
		self.declare_parameter('heading_correction_sign', -1.0)  # 修正方向；方向反了改成 +1.0
		self.declare_parameter('heading_correction_limit_deg', 8.0)  # 单次航向修正最大角度(°)
		self.declare_parameter('heading_integral_limit_deg', 5.0)  # 积分项限幅，防止积分饱和

		self._imu_topic = self.get_parameter('imu_topic').value
		self._heading_control_enabled = self.get_parameter('heading_control_enabled').value
		self._straight_steering_threshold = self.get_parameter('straight_steering_threshold').value
		self._heading_settle_time = self.get_parameter('heading_settle_time').value
		self._imu_timeout = self.get_parameter('imu_timeout').value
		self._yaw_jump_max_deg = self.get_parameter('yaw_jump_max_deg').value
		self._imu_invalid_frames_before_fault = self.get_parameter('imu_invalid_frames_before_fault').value
		self._imu_recover_time = self.get_parameter('imu_recover_time').value
		self._max_yaw_error_deg = self.get_parameter('max_yaw_error_deg').value
		self._heading_kp = self.get_parameter('heading_kp').value
		self._heading_ki = self.get_parameter('heading_ki').value
		self._heading_kd = self.get_parameter('heading_kd').value
		self._heading_correction_sign = self.get_parameter('heading_correction_sign').value
		self._heading_correction_limit_deg = self.get_parameter('heading_correction_limit_deg').value
		self._heading_integral_limit_deg = self.get_parameter('heading_integral_limit_deg').value

		# ---- 打开串口 ----
		try:
			self.ser = serial.Serial(port, baudrate, timeout=timeout)
			self.get_logger().info(f'✅ 串口 {port} 已打开，波特率 {baudrate}')
		except serial.SerialException as e:
			self.get_logger().error(f'❌ 无法打开串口 {port}: {e}')
			raise SystemExit(1)

		# ---- 订阅前轮转向指令话题 ----
		# 注意：上游 joystick_bridge_node 发布 /wheel_control/dir 的消息类型是 Float64，
		# 单位为弧度，而不是 Float64MultiArray。
		self.subscription = self.create_subscription(
			Float64,
			'/wheel_control/dir',
			self.dir_callback,
			10
		)

		# ---- 订阅 IMU 话题 ----
		# 通过订阅 IMU 话题，利用其航向角对前轮转向进行 PID 控制。
		# 当小车直线行驶（转向指令接近 0）时，把当前航向锁为参考方向；
		# 之后用 IMU 四元数实时计算相对偏航，PID 输出前轮修正量，抑制直线偏航。
		self.imu_subscription = self.create_subscription(
			Imu,
			self._imu_topic,
			self.imu_callback,
			10
		)

		# 创建前轮转向角度的位置发布话题
		self.pub = self.create_publisher(Float64, '/df_dir_rt', 10)
		self.motor_status_data = Float64()
		self.motor_status_data.data = 0.0

		# ---- IMU / 航向 PID 运行状态 ----
		self._imu_received = False  # 是否已经收到第一帧 IMU 数据
		self._current_yaw = None  # 当前有效 IMU yaw，单位 rad
		self._heading_reference = None  # 直行开始时锁定的参考航向，单位 rad
		self._heading_control_active = False  # 当前是否正在执行直线航向保持
		self._straight_start_time = None  # 转弯后重新进入直行的稳定计时起点
		self._last_imu_time = None  # 最后一次收到 IMU 消息的 ROS 时间
		self._imu_fault = False  # IMU 是否处于故障状态
		self._imu_invalid_count = 0  # 连续异常 IMU 帧计数
		self._imu_recover_start_time = None  # IMU 故障后开始恢复计时的 ROS 时间
		self._last_control_time = None  # 上一次 PID 计算时间
		self._last_yaw_error = 0.0  # 上一次偏航误差，用于微分项
		self._yaw_integral = 0.0  # 偏航误差积分，单位 °·s

		self.get_logger().info('🎮 等待手柄数据...')

	def dir_callback(self, msg: Float64):
		"""收到前轮转向指令时自动调用"""
		# 上游发布的是弧度，按原有协议转换为发送给舵机的角度值。
		angle = (((msg.data[0] + msg.data[1]) / 2) / math.pi) * 180

		# 只在转向角位于 ±0.1 rad 内（直行）时使用 IMU 航向 PID 修正；
		# 转弯时 _compute_heading_correction 会返回 0，不使用 PID 数据。
		# heading_correction = self._compute_heading_correction(msg.data)
		# self.get_logger().info(f"{heading_correction}")
		# 最终发送角度 = 原始转向角 + 航向保持修正量
		# angle = base_angle + heading_correction

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

	def imu_callback(self, msg: Imu):
		"""
        IMU 回调：从四元数实时计算 yaw，供直线航向 PID 使用。

        只保存“最新 yaw”，不在这里直接写串口；
        真正的控制输出仍然由 dir_callback 按转向指令频率统一发送。
        """

		pass
		return
		now = self.get_clock().now()
		yaw = self._yaw_from_quaternion(
			msg.orientation.x,
			msg.orientation.y,
			msg.orientation.z,
			msg.orientation.w,
		)

		# 四元数无效（全 0、NaN 或模长过小）时按异常帧处理。
		if yaw is None:
			self._handle_invalid_imu(now)
			return

		# 与上一帧有效 yaw 比较，单帧跳变过大说明 IMU 数据异常，不更新当前 yaw。
		if self._current_yaw is not None:
			yaw_delta = self._normalize_angle(yaw - self._current_yaw)
			if abs(math.degrees(yaw_delta)) > self._yaw_jump_max_deg:
				self._handle_invalid_imu(now)
				return

		# 数据有效：更新当前 yaw，并推进故障恢复判断。
		self._accept_valid_imu(now, yaw)

	@staticmethod
	def _yaw_from_quaternion(x: float, y: float, z: float, w: float):
		"""
        把 ROS 标准四元数转换为偏航角 yaw。
        返回 None 表示四元数不可用，调用方应忽略本帧数据。
        """
		values = (x, y, z, w)
		if not all(math.isfinite(v) for v in values):
			return None

		norm = math.sqrt(x * x + y * y + z * z + w * w)
		if norm < 1e-6:
			return None

		# 归一化后再计算，避免传感器四元数未严格归一化带来的误差。
		x, y, z, w = x / norm, y / norm, z / norm, w / norm

		# 标准四元数 -> yaw 公式（绕 Z 轴旋转角）。
		yaw = math.atan2(
			2.0 * (w * z + x * y),
			1.0 - 2.0 * (y * y + z * z),
		)
		return yaw

	@staticmethod
	def _normalize_angle(angle: float) -> float:
		"""把任意角度归一化到 [-pi, pi] 区间，便于计算最小偏航误差。"""
		angle = math.fmod(angle + math.pi, 2.0 * math.pi)
		if angle < 0.0:
			angle += 2.0 * math.pi
		return angle - math.pi

	def _disable_heading_control(self):
		"""
        关闭航向保持，并清空参考航向、直行计时和 PID 历史状态。

        此时 dir_callback 会继续发送原始手柄转向角，只是不再叠加 IMU 修正。
        """
		self._heading_control_active = False
		self._heading_reference = None
		self._straight_start_time = None
		self._reset_heading_pid()

	def _mark_imu_fault(self):
		"""进入 IMU 故障状态。故障期间航向 PID 完全不输出，只保留原始转向。"""
		if self._imu_fault:
			return

		self._imu_fault = True
		self._imu_recover_start_time = None
		self._disable_heading_control()
		self.get_logger().warn('检测到 IMU 异常，已关闭航向 PID，仅使用原始转向指令')

	def _handle_invalid_imu(self, now):
		"""
        处理一帧异常 IMU 数据。

        单帧异常不会立刻判定为故障，但会累计连续异常帧数；
        达到阈值后才进入 IMU 故障状态，避免偶发毛刺导致 PID 频繁开关。
        """
		self._last_imu_time = now
		self._imu_invalid_count += 1
		self._imu_recover_start_time = None

		if self._imu_invalid_count >= self._imu_invalid_frames_before_fault:
			self._mark_imu_fault()

	def _accept_valid_imu(self, now, yaw: float):
		"""
        处理一帧有效 IMU 数据，并判断是否满足故障恢复条件。

        IMU 故障后不能立刻重新启用 PID，需要连续正常一段时间，
        确认数据稳定后再允许航向保持重新锁定参考航向。
        """
		self._current_yaw = yaw
		self._imu_received = True
		self._last_imu_time = now
		self._imu_invalid_count = 0

		if not self._imu_fault:
			self._imu_recover_start_time = None
			return

		if self._imu_recover_start_time is None:
			self._imu_recover_start_time = now

		recover_elapsed = (now - self._imu_recover_start_time).nanoseconds / 1e9
		if recover_elapsed >= self._imu_recover_time:
			self._imu_fault = False
			self._imu_recover_start_time = None
			self.get_logger().info('IMU 数据已恢复正常，允许重新启用航向 PID')

	def _reset_heading_pid(self):
		"""清除 PID 历史状态。非直行、退出直行或首次进入直行时都应调用。"""
		self._last_control_time = None
		self._last_yaw_error = 0.0
		self._yaw_integral = 0.0

	def _compute_heading_correction(self, steering_rad: float) -> float:
		"""
        计算直线行驶时的前轮航向修正量。

        参数:
            steering_rad: 当前前轮转向指令，单位 rad。

        返回:
            航向修正量，单位 °。非直行或 IMU 不可用时返回 0.0。
        """
		now = self.get_clock().now()

		# 总开关关闭：不做任何修正，只保留原始转向。
		if not self._heading_control_enabled:
			self._disable_heading_control()
			return 0.0

		# IMU 故障：关闭航向 PID，只保留原始转向。
		if self._imu_fault:
			self._disable_heading_control()
			return 0.0

		# 还没有收到过有效 IMU 数据：不能使用航向 PID。
		if not self._imu_received or self._current_yaw is None:
			self._disable_heading_control()
			return 0.0

		# IMU 数据超时：连续没有新数据，判定为故障。
		if self._last_imu_time is not None:
			imu_elapsed = (now - self._last_imu_time).nanoseconds / 1e9
			if imu_elapsed > self._imu_timeout:
				self._mark_imu_fault()
				return 0.0

		# 直线判断：转向角在 [-straight_steering_threshold, +straight_steering_threshold]
		# 内时才启用 PID，默认阈值 0.1 rad；超出该范围视为转弯。
		is_straight = abs(steering_rad) <= self._straight_steering_threshold

		# 转弯时：不启用 PID，也不使用之前积累的 PID 数据。
		if not is_straight:
			self._disable_heading_control()
			return 0.0

		# 转弯后第一次回到直行：先等待一小段时间让 yaw 稳定，
		# 再锁定当前 yaw 作为新的参考航向，避免把转弯末段的瞬时偏航锁进去。
		if not self._heading_control_active:
			if self._straight_start_time is None:
				self._straight_start_time = now

			settle_elapsed = (now - self._straight_start_time).nanoseconds / 1e9
			if settle_elapsed < self._heading_settle_time:
				return 0.0

			# 稳定时间已到：锁定参考航向并正式启动 PID。
			self._heading_reference = self._current_yaw
			self._heading_control_active = True
			self._straight_start_time = None
			self._reset_heading_pid()
			self._last_control_time = now
			return 0.0

		if self._last_control_time is None:
			self._last_control_time = now
			return 0.0

		# 计算距离上一次 PID 的时间间隔；过久没有数据时限制最大值，防止微分跳变。
		dt = (now - self._last_control_time).nanoseconds / 1e9
		if dt <= 0.0:
			return 0.0
		if dt > 0.5:
			dt = 0.5

		# 相对偏航误差，统一到 [-180°, 180°]。
		yaw_error_rad = self._normalize_angle(self._current_yaw - self._heading_reference)
		yaw_error_deg = math.degrees(yaw_error_rad)

		# 偏航误差过大：直接关闭 PID，使用原始手柄转向角。
		if abs(yaw_error_deg) > self._max_yaw_error_deg:
			self._mark_imu_fault()
			return 0.0

		# 积分项，并做抗饱和限幅。
		self._yaw_integral += yaw_error_deg * dt
		int_limit = self._heading_integral_limit_deg
		self._yaw_integral = max(-int_limit, min(int_limit, self._yaw_integral))

		# 微分项：误差变化率，用于抑制振荡；kd=0 时该项为 0。
		derivative = (yaw_error_deg - self._last_yaw_error) / dt
		self._last_yaw_error = yaw_error_deg

		# 标准位置式 PID 输出，单位 °。
		pid_output = (
				self._heading_kp * yaw_error_deg
				+ self._heading_ki * self._yaw_integral
				+ self._heading_kd * derivative
		)

		# 负反馈：航向偏左（yaw 增大）时，需要输出右转修正（转向角减小）。
		# 如果实车方向相反，把参数 heading_correction_sign 改成 +1.0。
		correction = self._heading_correction_sign * pid_output

		# 限制单次修正量，避免异常时舵机突然大角度动作。
		limit = self._heading_correction_limit_deg
		correction = max(-limit, min(limit, correction))

		self._last_control_time = now

		# 调试时可打开下面这行日志，观察 yaw / 误差 / 修正量。
		# self.get_logger().info(
		#     f'yaw={math.degrees(self._current_yaw):.2f}° '
		#     f'error={yaw_error_deg:.2f}° correction={correction:.2f}°'
		# )
		return correction

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
