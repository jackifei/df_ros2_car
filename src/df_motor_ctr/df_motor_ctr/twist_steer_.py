from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class AckermannStepperConverter:
	"""
	单步进电机联动前轮转向的阿克曼转换器。

	单位约定：
	  wheelbase: m
	  linear_x: m/s
	  angular_z: rad/s
	  max_steer_angle: rad
	  motor_to_wheel_ratio: 电机转角 / 前轮等效转角
	  steps_per_rev: 电机每转脉冲数 = (360/步距角) * 细分
	  zero_step: 前轮直行 delta=0 时对应的绝对步数
	  steer_sign: +1 或 -1，用于机械/电机方向标定
	"""

	wheelbase: float
	max_steer_angle: float = math.radians(35.0)
	min_speed: float = 0.02
	w_eps: float = 1e-6
	motor_to_wheel_ratio: float = 1.0
	steps_per_rev: int = 3200
	zero_step: int = 0
	steer_sign: float = 1.0
	current_delta: float = 0.0

	def cmd_vel_to_delta(self, linear_x: float, angular_z: float) -> float:
		"""
		根据 cmd_vel 计算等效前轮转角 delta，单位 rad。

		阿克曼自行车模型：
			delta = atan(L * w / v)

		低速保护：
		  |v| < min_speed 且 w != 0 时，不能靠公式计算，保持当前转角；
		  |v| < min_speed 且 w ≈ 0 时，目标转角回正到 0。
		"""
		v = float(linear_x)
		w = float(angular_z)

		if not math.isfinite(v) or not math.isfinite(w):
			delta_cmd = self.current_delta
		elif abs(v) < self.min_speed:
			if abs(w) < self.w_eps:
				delta_cmd = 0.0
			else:
				# 阿克曼底盘不能原地自转：保持当前转角，忽略角速度
				delta_cmd = self.current_delta
		else:
			delta_cmd = math.atan(self.wheelbase * w / v)
			delta_cmd = max(
				-self.max_steer_angle,
				min(self.max_steer_angle, delta_cmd)
			)

		# 统一再做一次机械限幅
		delta_cmd = max(
			-self.max_steer_angle,
			min(self.max_steer_angle, delta_cmd)
		)

		self.current_delta = delta_cmd
		return delta_cmd

	def delta_to_target_step(self, delta: float) -> int:
		"""
		将等效前轮转角 delta 转为步进电机绝对目标步数。

		delta_motor = motor_to_wheel_ratio * delta
		delta_step  = steer_sign * delta_motor / (2*pi) * steps_per_rev
		target_step = zero_step + round(delta_step)
		"""
		delta_motor = self.motor_to_wheel_ratio * delta
		delta_step = (
			self.steer_sign
			* delta_motor
			/ (2.0 * math.pi)
			* self.steps_per_rev
		)
		return self.zero_step + int(round(delta_step))

	def cmd_vel_to_stepper(
		self, linear_x: float, angular_z: float
	) -> Tuple[float, int]:
		"""
		输入 cmd_vel 的 linear.x 和 angular.z，
		返回 (等效前轮转角 rad, 步进电机绝对目标步数)。
		"""
		delta_cmd = self.cmd_vel_to_delta(linear_x, angular_z)
		target_step = self.delta_to_target_step(delta_cmd)
		return delta_cmd, target_step

	def reset(self, zero_step: int | None = None) -> None:
		"""重置当前转角和可选零位。"""
		self.current_delta = 0.0
		if zero_step is not None:
			self.zero_step = zero_step


if __name__ == "__main__":
	# conv = AckermannStepperConverter()


	conv = AckermannStepperConverter(
		wheelbase=1.0,
		max_steer_angle=math.radians(60.0),
		min_speed=0.02,
		motor_to_wheel_ratio=2.0,
		steps_per_rev=3200,
		zero_step=0,
		steer_sign=1.0,
	)

	# v = 0.3 m/s, w = 0.3 rad/s
	delta, target_step = conv.cmd_vel_to_stepper(0.3, 0.3)

	print("delta deg:", math.degrees(delta))  # 45.0
	print("target_step:", target_step)        # 800