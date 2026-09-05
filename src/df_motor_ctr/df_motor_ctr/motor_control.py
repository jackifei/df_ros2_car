import rclpy
import time
import threading
from pymodbus.client import ModbusSerialClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.node import Node             # ROS2 节点类
import math

# # 创建电机状态字典
# motor_state_data = {
# 	"left_motor_speed": 0.0,  # 左轮速度
# 	"right_motor_speed": 0.0, # 右轮速度
# 	"steering_speed": 0.1,   # 转向速度
# 	"current_steering_angle": 15.5   # 转向角度
# }
# # 2. 创建 String 消息并赋值
# motor_state_msg = String()
# motor_state_msg.data = json.dumps(motor_state_data)
# self.pub.publish(motor_state_msg)             # 发布电机状态消息

class motor_status:
	Oac_TF = False      # 掉电标志  7
	Cgp_TF = False      # 堵转保护标志  3
	Cgi_TF = False      # 堵转标志  2
	Ens_TF = False      # 使能状态  0

class channel_MBRTU(Node):
	"""MB线程控制器 """

	def __init__(self):
		super().__init__("df_motor_status")

		# ---- 声明可配置参数 ----
		self.dir_data = 0.0
		self.declare_parameter('port', '/dev/ttyUSB2')
		self.declare_parameter('wheel_track', 0.39)
		self.declare_parameter('wheel_diameter_m', 0.125)
		self.port = self.get_parameter('port').value
		self.wheel_track = self.get_parameter('wheel_track').value
		self.wheel_diameter_m = self.get_parameter('wheel_diameter_m').value
		# self.wheel_diameter_m = 0.125

		# 此处需要修改，上一个话题的发布是50HZ，对于点击驱动来说，相应不了这么高的频率
		# 由于发布频率的问题，需要修改为转速当变化时才进行写入，并且为int类型，下位机采用modbus协议，只能是int
		# 订阅后轮速度节点，用来数据计算
		self.sub = self.create_subscription(Float64MultiArray,
		                                    '/hardware/rear_wheel_cmd',
		                                    self.listener_callback,
		                                    10)
		# 订阅前轮转向指令
		# ---- 订阅 front_steering_cmd 前轮转向 话题 ----
		self.subscription = self.create_subscription(Float64MultiArray,
		                                             '/hardware/front_steering_cmd',
		                                             self.dir_callback,
		                                             10
		                                             )
		# 订阅前轮转向试试角度
		self.sub_dir = self.create_subscription(Float64,
		                                        '/df_dir_rt',
		                                        self.dir_listener_callback_, 10)

		self.angle = 0.0

		# 创建电机实时状态的发布对象
		# 此窗台用来反馈给阿克曼控制器
		self.cmd_vel_rt_pub = self.create_publisher(JointState,
		                                            '/hardware/joint_feedback',
		                                            10)

		self.motor_status_data = JointState()
		# self.motor_status_data.velocity = [0.0,0.0]

		self.get_logger().info(f'485电机控制初始化') 
		# 此端口只能采用同步，modbusTCP支持异步
		self.write_speed_dir = False  	# 写入速度和方向标志
		self.write_quick_stop = False 	# 写入急停标志
		self.write_clear_err = False  	# 写入清楚错误标志
		self.write_enable = True       # 写入使能标志


		self.l_speed = 0.0
		self.r_speed = 0.0
		self.write_l_speed = 0.0 # 左轮写入速度
		self.write_r_speed = 0.0 # 右轮写入速度
		self.last_l_speed_ = 0  # 值改变寄存器
		self.last_r_speed_ = 0  # 值改变寄存器

		self.l_dir = 0      # 左轮子前后旋转方向
		self.r_dir = 0      # 右轮子前后旋转方向
		
		# self.port = port
		self.slave1 = 1
		self.slave2 = 2

		self.v_l_rt = 0
		self.v_r_rt = 0

		# self.rt_l_speed_rad = 0.0
		# self.rt_l_speed_rad = 0.0

		# self.wheel_diameter_m = 0.125   # 后轮驱动轮的直径
		#self.wheel_track = 0.39   # 后轮轮距

		self._stop_event = threading.Event()
		self._lock = threading.Lock()
		self.commn_thread =  threading.Thread(target=self.loop_start_, daemon=True)
		# self.comm_thread.start()
		# 创建实例对象
		# self.logger = logging.getLogger(__name__)
		self.result_info2 = None  # 结果信息
		self.result_info = None    # 结果信息
		self.power_status_info = None    # 结果信息
		self.moto_status_l = motor_status()
		self.moto_status_r = motor_status()
		
		# self.timeout = timeout
		self.ser = None
		self.state = "close"
		self.running = False  # 线程运行控制标志
		self.close_port()
		# 实例对象
		self.open_port()

	def dir_callback(self, msg: Float64MultiArray):
		"""收到手柄数据时自动调用"""
		# ---------- 用户需在此处实现角度计算 ----------
		# self.get_logger().info(f'转向角度{msg.data[0]}  {msg.data[1]}')
		self.angle = (msg.data[0] + msg.data[1]) / 2

	def pub_joint_status(self):

		self.motor_status_data.header.stamp = self.get_clock().now().to_msg()

		self.motor_status_data.name = ['lh_joint', 'lq_joint', 'rh_joint', 'rq_joint']
		# --- 位置 (position) 单位：弧度 (rad) ---
		# 后轮：位置通常不重要，里程计只用速度，但为了完整性设为 0.0
		# 前轮：转向角，正值表示左转（根据 REP-103）
		self.motor_status_data.position = [0.0, self.angle, 0.0, self.angle]
		# --- 速度 (velocity) 单位：弧度/秒 (rad/s) ---
		# 后轮：分别赋值左右轮的实际转速
		# 前轮：转向速度通常不用于里程计，设为 0.0

		self.motor_status_data.velocity = [self.v_l_rt , 0.0, self.v_r_rt, 0.0]  # 转向速度设为0

		self.cmd_vel_rt_pub.publish(self.motor_status_data)

	def linear_velocity_to_rpm(self,v) -> float:
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
		if self.wheel_diameter_m <= 0:
			raise ValueError("轮子直径必须大于 0")

		radius_m = self.wheel_diameter_m / 2.0
		rpm = v * 30.0 / (math.pi * radius_m)
		return rpm

	@staticmethod
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
	@staticmethod
	def rpm_to_rad_per_sec(rpm: float) -> float:
		"""
		将电机转速（RPM，转每分钟）转换为角速度（rad/s）

		参数:
			rpm (float): 电机转速，单位 RPM

		返回:
			float: 角速度，单位 rad/s

		公式:
			omega = rpm * 2 * π / 60
		"""
		omega_rad_s = rpm * 2.0 * math.pi / 60.0
		return omega_rad_s

	def rpm_to_linear_velocity(self,n) -> float:
		"""
		将轮子转速（rpm）转换为线速度（m/s）（单位：米）

		参数:
			n (float): 转速，单位 rpm
			wheel_diameter_m (float): 轮子直径，单位 米，默认 0.125 m（125 mm）

		返回:
			float: 线速度，单位 m/s

		公式:
			v = n * π * r / 30 ，其中 r = wheel_diameter_m / 2
		"""
		if self.wheel_diameter_m <= 0:
			raise ValueError("轮子直径必须大于 0")

		radius_m = self.wheel_diameter_m / 2.0
		v = n * math.pi * radius_m / 30.0
		return v
	def listener_callback_(self, msg:Float64MultiArray):
		"""
		接收阿克曼控制器的硬件驱动信号,此方法应该用来去控制电机进行旋转
		"""
		# self.get_logger().info(f'{msg.data[0]}  {msg.data[1]}')
		pass

	def dir_listener_callback_(self, msg:Float64):
		"""前轮实时转向角"""
		# 接收到的是角度，需要将角度转换成弧度
		self.dir_data = (msg.data / 180 ) * math.pi

		# self.get_logger().info(f'{self.dir_data}')


	def listener_callback(self, msg:Float64MultiArray):
		"""
		接受手柄消息，响应手柄按键，将参数写入到电机驱动中
		"""
		# self.get_logger().info(f'{msg.data[0]}  {msg.data[1]}')
		# self.l_speed = self.linear_velocity_to_rpm(msg.data[0])
		
		# self.r_speed = self.linear_velocity_to_rpm(msg.data[1])
		# 速度反馈使用下发指令
		# self.v_l_rt = msg.data[1]
		# self.v_r_rt = msg.data[0]

		self.l_speed = self.rad_per_sec_to_rpm(msg.data[1])
		self.r_speed = self.rad_per_sec_to_rpm(msg.data[0])
		# self.get_logger().info(f'---L  {self.l_speed}  ----R  {self.r_speed}')
		self.pub_joint_status()   # 发布关节速度
		self.write_l_speed = int(abs(self.l_speed))
		self.write_r_speed = int(abs(self.r_speed))

		if self.l_speed >= 0 and self.r_speed >=0 :
			if self.last_l_speed_ != self.write_l_speed or self.last_r_speed_ != self.write_r_speed:
				self.l_dir = 1      # 左轮子前后旋转方向
				self.r_dir = 0      # 右轮子前后旋转方向

				self.last_l_speed_ = self.write_l_speed
				self.last_r_speed_ = self.write_r_speed
				self.write_speed_dir = True
				# self.get_logger().info(f'写入速度:方向 + ')
		if self.l_speed < 0 and self.r_speed < 0:
			if self.last_l_speed_ != self.write_l_speed or self.last_r_speed_ != self.write_r_speed:
				self.l_dir = 0      # 左轮子前后旋转方向
				self.r_dir = 1      # 右轮子前后旋转方向

				self.last_l_speed_ = self.write_l_speed
				self.last_r_speed_ = self.write_r_speed
				self.write_speed_dir = True
				# self.get_logger().info(f'写入速度 :方向 - ')


	def pull_wheel(self,v_l,v_r,l_dir,r_dir):
		"""前进，正转  00/01 分别代表CW/CCW"""
		self.ser.write_registers(
			address=0xF6,  # 寄存器起始地址（0x00F3）
			values=[(l_dir * 256) + 100, v_l, 0],
			device_id=self.slave2  # 从机地址
		)
		self.ser.write_registers(
			address=0xF6,  # 寄存器起始地址（0x00F3）
			values=[(r_dir * 256) + 100, v_r, 0],
			device_id=self.slave1  # 从机地址
		)

	"""打开端口"""
	def open_port(self):
		"""打开串口"""
		if self.state != "close":
			self.get_logger().info(f'端口已关闭') 
			return False
		try:
			self.ser = ModbusSerialClient(port=self.port, baudrate=115200,
											 parity="N",
											 stopbits=1,
											 bytesize=8) # 最后一项必须关闭，否则报错strict=False
			self.ser.connect()
			# 添加串口就绪等待
			time.sleep(0.1)
			self.state = "open"
			self.get_logger().info(f'✈️ 1端口打开成功')
			# 启动线程
			# self.start()
			self.get_logger().info(f'✈️ 2电机进入待机状态')
			self.get_logger().info(f'✈️ 3初始状态使能')
			self.enable_motor(flag=True)
			self.get_logger().info(f'✈️ 4使能完成')
			self.get_logger().info(f'✈️ 5设置初始速度0,初始方向')
			self.pull_wheel(v_l=0,v_r = 0,l_dir=0,r_dir=0)
			time.sleep(0.1)
			self.get_logger().info(f'✈️ 6启动电机控制线程')
			self.commn_thread.start()
			return True
		except Exception as e:
			self.get_logger().info(f"打开控制错误，错误信息⬇️")
			self.get_logger().info(f"error: {e}") 
			return False
		


	"""关闭端口"""
	def close_port(self):
		"""关闭串口"""
		if self.state == "close":
			self.get_logger().info(f"端口关闭")
			return

		try:
			if self.ser:
				self.ser.close()
				self.stop()
			self.get_logger().info(f"Port {self.port} closed")
		except Exception as e:
			self.get_logger().info(f"Error while closing port: {e}")
		finally:
			self.stop()
			self.ser = None
			self.state = "close"

	"""停止通讯"""
	def stop(self):
		"""停止线程运行"""
		self.running = False
		self.state = "close"

	"""线程循环读取"""
	def loop_start_(self):
		"""线程主运行函数"""
		self.running = True
		while not self._stop_event.is_set():
			try:
				if self._stop_event.wait(timeout=0.05):
					self.get_logger().info(f'收到停止信号，立即跳出循环')  
					break
				try:
					# self.get_logger().info(f'电机循环')
					result_pos_l,result_rpm_l, moto_status_l_l,moto_status_r_r = self.read_slave_data(slave=self.slave1)
					result_pos_r,result_rpm_r, moto_status_l_r,moto_status_r_r = self.read_slave_data(slave=self.slave2)
					# self.get_logger().info(f'循环读取中{result_pos_l}{result_pos_r}')
					# 通过转速rpm计算线速度，并且添加到twist中，进行发布
					self.v_l_rt =  round(self.rpm_to_rad_per_sec(result_rpm_l),1)
					self.v_r_rt  =  round(self.rpm_to_rad_per_sec(result_rpm_r) * -1,1)

					# self.v_l_rt =  round(self.rpm_to_rad_per_sec(self.write_l_speed),1)
					# self.v_r_rt  =  round(self.rpm_to_rad_per_sec(self.write_r_speed) * -1,1)

					# self.get_logger().info(f'速度L{v_l} 速度R{v_r} ') 
					# 线速度
					# self.motor_status_data.linear.x  = (v_l + v_r) / 2.0
					# 角速度
					# self.motor_status_data.linear.z  = (v_l - v_r) / self.wheel_track

					# 1. 快速拷贝状态（持锁时间极短）
					if self.write_speed_dir :
						self.pull_wheel(v_l=self.write_l_speed,v_r = self.write_r_speed,l_dir=self.l_dir,r_dir=self.r_dir)
						# self.get_logger().info(f'L速度{self.write_l_speed} L速度{self.write_r_speed}')
						self.write_speed_dir = False
					# 写入使能或者急停
					if self.write_quick_stop :
						self.get_logger().info(f'电机-急停')
						self.quick_stop()
						self.write_quick_stop = False
					# 清除错误
					if self.write_clear_err :
						self.get_logger().info(f'电机-清除错误')
						self.clear_error()
						self.write_clear_err = False
					# 写入使能
					if self.write_enable :
						self.get_logger().info(f'电机-写入使能')
						self.enable_motor(flag=True)
						self.write_enable = False       # 写入使能标志

				except:
					pass
			except Exception as e:
				self.get_logger().info(f'链接已断开')
			# time.sleep(0.05)
	
		self.close_port()
		print(f"端口 线程 已停止")

	def read_slave_data(self, slave):
		"""
		实时转速
		实时角度
		使能状态
		"""
		# ------------------实时位置角度
		pos_status = self.ser.read_holding_registers(address=54, count=3, device_id=slave)
		pos_registers = pos_status.registers
		result_pos = self.convert_motor_position(pos_registers)
		# print(f"{slave}实时位置角度 {result_pos}")
		# ------------------实时转速
		rpm_status = self.ser.read_holding_registers(address=53, count=2, device_id=slave)
		rpm_registers = rpm_status.registers
		if rpm_registers[0] == 0:
			result_rpm = rpm_registers[1]
		else:
			result_rpm = -rpm_registers[1]
		#  print(f"{slave}实时速度 {result_rpm}")
		#  # ------------------使能状态
		motor_status = self.ser.read_holding_registers(address=58, count=1, device_id=slave)
		motor_registers = motor_status.registers
		result_status = self.word_to_bits(motor_registers[0])
		if slave == 1:
			if result_status[0] == 1:
				self.moto_status_l.Ens_TF = True
			else:
				self.moto_status_l.Ens_TF = False
			if result_status[2] == 1:
				self.moto_status_l.Cgi_TF = True
			else:
				self.moto_status_l.Cgi_TF = False
			if result_status[3] == 1:
				self.moto_status_l.Cgp_TF = True
			else:
				self.moto_status_l.Cgp_TF = False
			if result_status[7] == 1:
				self.moto_status_l.Oac_TF = True
			else:
				self.moto_status_l.Oac_TF = False
		else:
			if result_status[0] == 1:
				self.moto_status_r.Ens_TF = True
			else:
				self.moto_status_r.Ens_TF = False
			if result_status[2] == 1:
				self.moto_status_r.Cgi_TF = True
			else:
				self.moto_status_r.Cgi_TF = False
			if result_status[3] == 1:
				self.moto_status_r.Cgp_TF = True
			else:
				self.moto_status_r.Cgp_TF = False
			if result_status[7] == 1:
				self.moto_status_r.Oac_TF = True
			else:
				self.moto_status_r.Oac_TF = False
		return result_pos,result_rpm, self.moto_status_l,self.moto_status_r
		# print(self.moto_status.Ens_TF)

	@staticmethod
	def word_to_bits(value):
		"""
		将 0-65535 的整数转换为 16 位的二进制列表 (高位在前)
		"""
		# 1. format(value, '016b') 将数字转为 16 位二进制字符串 (如 '0000000000000001')
		# 2. list(...) 将字符串拆分为字符列表
		# 3. [int(x) for x in ...] 将字符 '0'/'1' 转为整数 0/1
		return [int(x) for x in format(value, '016b')]

	@staticmethod
	def convert_motor_position(data_list):
		"""
		将电机返回的原始列表数据转换为角度值
		:param data_list: 包含 [符号, 高字节, 低字节] 的列表
		:return: 浮点型角度值
		"""
		if len(data_list) < 3:
			return None

		# 1. 获取符号 (0=正, 1=负)
		sign_byte = data_list[0]
		multiplier = -1 if sign_byte == 1 else 1

		# 2. 获取位置数值 (高位 * 256 + 低位)
		high_byte = data_list[1]
		low_byte = data_list[2]
		raw_position = (high_byte << 8) | low_byte  # 等同于 high_byte * 256 + low_byte

		# 3. 应用公式: (位置 * 360) / 65536
		angle = (raw_position * 360.0) / 65536.0

		# 4. 加上符号并返回
		final_angle = angle * multiplier
		return round(final_angle, 1)  # 保留一位小数、

	@staticmethod
	def int32_to_two_words(value):
		"""
		将32位整数拆分为两个16位的字
		value: 32位有符号整数
		返回: (low_word, high_word)
		"""
		# 确保值在32位有符号整数范围内
		if value < -2 ** 31 or value > 2 ** 31 - 1:
			raise ValueError("值超出32位有符号整数范围")

		# 转换为无符号32位表示
		if value < 0:
			value = (1 << 32) + value

		# 提取低16位和高16位
		low_word = value & 0xFFFF
		high_word = (value >> 16) & 0xFFFF
		result_data = [int(low_word), int(high_word)]
		return result_data

	def enable_motor(self,flag):
		"""
		控制电机使能/不使能
		:param client: ModbusRTU客户端实例
		:param slave_addr: 从机地址（电机地址）
		:param enable_state: 使能状态（0x00=不使能，0x01=使能）
		:return: 是否成功（True/False）
		"""
		try:
			# ModbusSerialClient.write_register()
			if flag == True:
				self.ser.write_registers(
					address=0xF3,  # 寄存器起始地址（0x00F3）
					values=[(171 * 256) + 1,0],
					device_id=self.slave1  # 从机地址
				)
				self.ser.write_registers(
					address=0xF3,  # 寄存器起始地址（0x00F3）
					values=[(171 * 256) + 1, 0],
					device_id=self.slave2  # 从机地址
				)
			else:
				self.ser.write_registers(
					address=0xF3,  # 寄存器起始地址（0x00F3）
					values=[(171 * 256) + 0, 0],
					device_id=self.slave1  # 从机地址
				)
				self.ser.write_registers(
					address=0xF3,  # 寄存器起始地址（0x00F3）
					values=[(171 * 256) + 0, 0],
					device_id=self.slave2  # 从机地址
				)
		except Exception as e:
			self.get_logger().info(f"写入使能错误")
			self.get_logger().info(f"error: {e}") 
			return False 

	

	"""清楚错误"""
	def clear_error(self):
		"""清除堵转/过热/过流保护"""
		self.ser.write_register(
			address=14,  # 寄存器起始地址（0x00F3）
			value=1,
			device_id=self.slave1  # 从机地址
		)
		self.ser.write_register(
			address=14,  # 寄存器起始地址（0x00F3）
			value=1,
			device_id=self.slave2  # 从机地址
		)

	"""急停"""
	def quick_stop(self):
		"""急停"""
		# ModbusSerialClient.write
		self.ser.write_register(address=0xFE, value=38912, device_id=self.slave1)
		self.ser.write_register(address=0xFE, value=38912, device_id=self.slave2)


	def destroy_node(self):
		"""重写节点销毁方法，确保线程被正确停止"""
		self.get_logger().info("正在停止电机控制线程...")
		self.enable_motor(False)
		self._stop_event.set()      # 发送停止信号
		self.close_port()
		super().destroy_node()

def main(args=None):
	# 创建串口线程 (替换为实际串口名)
	rclpy.init(args=args)                                   # ROS2 Python接口初始化
	ser_thread = channel_MBRTU()              # 创建ROS2节点对象并进行初始化
	try:
		rclpy.spin(ser_thread)
	except KeyboardInterrupt:
		pass
	finally:
		ser_thread.destroy_node()
		rclpy.shutdown()

