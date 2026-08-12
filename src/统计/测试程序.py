import  math

def linear_velocity_to_rpm(v,wheel_diameter_m) -> float:
	"""
	将线速度（m/s）转换为轮子转速（rpm）（单位：米）

	参数:
		v (float): 线速度，单位 m/s
		wheel_diameter_m (float): 轮子直径，单位 米，默认 0.125 m（125 mm）

	返回:
		float: 转速，单位 rpm

	公式:
		n = v * 30 / (π * r) ，其中 r = wheel_diameter_m / 2
	"""
	if wheel_diameter_m <= 0:
		raise ValueError("轮子直径必须大于 0")

	radius_m = wheel_diameter_m / 2.0
	rpm = v * 30.0 / (math.pi * radius_m)
	return rpm


def rad_per_sec_to_rpm(omega_rad_s: float) -> float:
	"""
	将角速度（rad/s）转换为电机转速（RPM，转每分钟）

	参数:
		omega_rad_s (float): 角速度，单位 rad/s

	返回:
		float: 转速，单位 RPM

	公式:
		RPM = omega * 60 / (2 * π)
	"""
	rpm = omega_rad_s * 60.0 / (2.0 * math.pi)
	return rpm


result = linear_velocity_to_rpm(0.3,0.125)
result2 = rad_per_sec_to_rpm(4.5)
print(result)
print(result2)

# 应该是弧度转成轮子的速度


# 单位换算问题