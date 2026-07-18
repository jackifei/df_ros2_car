#!/usr/bin/env python3
"""
纯轮式里程计 — 阿克曼底盘（编码器 + 转向角）
===============================================

仅依赖后轮编码器和前轮转向角传感器进行航位推算，
不使用 IMU，适合低成本或 IMU 失效时的降级运行。

  ┌──────────────┐  ┌──────────────┐
  │ 后轮编码器    │  │ 前轮转向角    │
  │ → 线速度 v   │  │ → δ          │
  └──────┬───────┘  └──────┬───────┘
         │                 │
         ▼                 ▼
  ┌─────────────────────────────────────┐
  │  自行车模型: ω = v·tan(δ)/L         │
  │  + 一阶低通滤波（可选）             │
  └─────────────────────────────────────┘
         │
         ▼
  ┌──────────────────┐
  │  中点法积分       │
  │  θ_mid = θ+Δθ/2  │
  │  Δx = v·cos(θ_mid)·Δt               │
  │  Δy = v·sin(θ_mid)·Δt               │
  └──────────────────┘
         │
         ▼
  ┌──────────────────┐
  │  /odom + TF       │
  └──────────────────┘

特点：
  - 无 IMU 依赖，纯编码器 + 转向角
  - 中点法积分（二阶精度）
  - 零速检测（静止锁定位姿）
  - 动态协方差
  - 异常保护（dt 跳变、tan 极值、限幅）

适用条件：
  - 阿克曼转向底盘（前轮转向，后轮驱动）
  - 具备后轮编码器、前轮转向角传感器
  - ROS2 Jazzy
"""
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class WheelOdomPure(Node):
    """
    纯编码器 + 转向角里程计，无 IMU 融合。
    设计原则：
      - 角速度完全由自行车模型计算：ω = v·tan(δ)/L
      - 可选一阶低通滤波平滑角速度（默认开启）
      - 积分法：中点法（二阶精度）
      - 协方差：动态调整，反映真实不确定度
      - 鲁棒性：多层异常保护
    """

    # =========================================================================
    # 限幅常数
    # =========================================================================
    MAX_STEER = math.pi / 6.0      # 30°，前轮物理限位
    MAX_OMEGA = 0.5                # rad/s，角速度上限
    MIN_SPEED_FOR_STEER = 0.02     # m/s，低于此速度转向模型不可靠
    MAX_DT = 0.2                   # s，超过视为数据中断

    # 零速检测阈值
    STILL_V_THRESHOLD = 0.005      # m/s
    STILL_W_THRESHOLD = 0.01       # rad/s
    STILL_FRAME_COUNT = 10         # 连续静止帧数

    def __init__(self, wheelbase: float, track_width: float,
                 wheel_radius: float = 0.062,
                 max_speed: float = 0.3,
                 enable_lowpass: bool = True,
                 lowpass_cutoff_hz: float = 20.0,
                 publish_rate: float = 50.0):
        """
        Args:
            node:               ROS2 Node 实例（用于创建发布器和日志）
            wheelbase:          轴距 L (m) — 用于 ω=v·tan(δ)/L
            track_width:        后轮轮距 W (m) — 用于日志和诊断
            wheel_radius:       车轮半径 (m) — 用于日志
            max_speed:          小车最大线速度 (m/s)
                                用于：速度限幅上限、协方差归一化
            enable_lowpass:     是否对自行车模型角速度进行一阶低通滤波
                                （减少转向角噪声导致的抖动）
            lowpass_cutoff_hz:  低通滤波器截止频率 (Hz)
            publish_rate:       名义发布频率 (Hz)，仅用于参考
        """
        # ---- 参数合法性校验 ----
        super().__init__('wheel_odom_pure')
        if wheelbase <= 0:
            raise ValueError(f'wheelbase 必须 > 0，当前值: {wheelbase}')
        if max_speed <= 0:
            raise ValueError(f'max_speed 必须 > 0，当前值: {max_speed}')

        self.L = float(wheelbase)
        self.W = float(track_width)
        self.r = float(wheel_radius)
        self.max_speed = float(max_speed)
        self.enable_lowpass = enable_lowpass
        self.lowpass_cutoff_hz = lowpass_cutoff_hz

        # 速度限幅取 max_speed 的 1.5 倍（留余量给下坡/滑移等超速场景）
        self._speed_limit = self.max_speed * 1.0
        #  位姿状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        #  时间追踪
        self.last_timestamp = None
        self.filtered_omega = 0.0       # 低通滤波后的角速度
        #  零速检测
        self._still_counter = 0

        # =====================================================================
        #  发布器
        # =====================================================================
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.path_pub = self.create_publisher(
            Path, '/path',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )
        self.path_poses = []

        # =====================================================================
        #  TF 广播器
        # =====================================================================
        self.tf_broadcaster = TransformBroadcaster(self)

        # =====================================================================
        #  坐标系名称
        # =====================================================================
        self.odom_frame = 'odom'
        self.base_link_frame = 'base_link'

        # =====================================================================
        #  协方差矩阵（初始值）
        # =====================================================================
        self._pose_cov = [0.0] * 36
        self._pose_cov[0] = 0.01
        self._pose_cov[7] = 0.01
        self._pose_cov[35] = 0.02

        self._twist_cov = [0.0] * 36
        self._twist_cov[0] = 0.01
        self._twist_cov[35] = 0.01

        # =====================================================================
        #  计数器
        # =====================================================================
        self._publish_count = 0
        self._diag_count = 0
        self._warn_count = 0

        self.get_logger().info(
            f'WheelOdomPure 已初始化 (不使用IMU):\n'
            f'  L={self.L:.3f}m  W={self.W:.3f}m  r={self.r:.3f}m\n'
            f'  max_speed={self.max_speed:.2f}m/s  speed_limit={self._speed_limit:.2f}m/s\n'
            f'  低通滤波={"启用" if self.enable_lowpass else "禁用"} '
            f'(cutoff={self.lowpass_cutoff_hz}Hz)\n'
            f'  传感器源: 编码器v + 转向角δ'
        )

    # =========================================================================
    #  公开 API — 主更新入口
    # =========================================================================

    def update(self, v: float, steering_angle: float,
               timestamp=None) -> None:
        """
        仅使用编码器线速度和前轮转向角，积分更新位姿并发布 /odom 和 TF。

        调用频率建议 >= 50Hz。

        Args:
            v:              后轴中点线速度 (m/s)，前进为正
            steering_angle: 前轮转向角 (rad)，左转为正
            timestamp:      rclpy.time.Time 对象
        """
        # ------------------------------------------------------------------
        # Step 1: 输入有效性检查
        # ------------------------------------------------------------------
        if not (math.isfinite(v) and math.isfinite(steering_angle)):
            self._warn_throttled(f"无效输入: v={v}, δ={steering_angle}")
            return

        # ------------------------------------------------------------------
        # Step 2: 时间步长计算 & 异常保护
        # ------------------------------------------------------------------
        if self.last_timestamp is None:
            if timestamp is not None:
                self.last_timestamp = timestamp
            return
            

        dt = (timestamp - self.last_timestamp).nanoseconds / 1e9

        if dt <= 1e-9:
            return

        if dt > self.MAX_DT:
            self.get_logger().warn(
                f'数据中断: dt={dt:.3f}s > {self.MAX_DT}s, 重置积分参考'
            )
            self.last_timestamp = timestamp
            return

        self.last_timestamp = timestamp

        # ------------------------------------------------------------------
        # Step 3: 限幅保护
        # ------------------------------------------------------------------
        v = self._clamp(v, -self._speed_limit, self._speed_limit)
        steering_angle = self._clamp(steering_angle,
                                     -self.MAX_STEER, self.MAX_STEER)

        # ------------------------------------------------------------------
        # Step 4: 零速检测 — 静止时锁定位姿，不积分
        # ------------------------------------------------------------------
        # 先计算转向角对应的角速度（用于零速判断）
        omega_steer = self._bicycle_omega(v, steering_angle)

        is_still = (abs(v) < self.STILL_V_THRESHOLD and
                    abs(omega_steer) < self.STILL_W_THRESHOLD)

        if is_still:
            self._still_counter += 1
        else:
            self._still_counter = 0

        if self._still_counter >= self.STILL_FRAME_COUNT:
            # 确认静止：发布当前位置，不积分
            self._publish_odometry(0.0, 0.0, timestamp)
            self._publish_tf(timestamp)
            return

        # ------------------------------------------------------------------
        # Step 5: 角速度获取（自行车模型 + 可选低通滤波）
        # ------------------------------------------------------------------
        omega = self._get_omega(omega_steer, dt)

        # ------------------------------------------------------------------
        # Step 6: 中点法积分（二阶精度）
        # ------------------------------------------------------------------
        delta_theta = omega * dt
        theta_mid = self.theta + delta_theta / 2.0

        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += delta_theta

        # 归一化 θ 到 [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ------------------------------------------------------------------
        # Step 7: 动态协方差
        # ------------------------------------------------------------------
        self._update_covariance(v, abs(omega))

        # ------------------------------------------------------------------
        # Step 8: 发布
        # ------------------------------------------------------------------
        self._publish_odometry(v, omega, timestamp)
        self._publish_tf(timestamp)

        # ------------------------------------------------------------------
        # Step 9: 诊断日志
        # ------------------------------------------------------------------
        self._diag_count += 1
        if self._diag_count <= 10 or self._diag_count % 200 == 0:
            self.get_logger().info(
                f'[odom #{self._diag_count}] '
                f'x={self.x:.4f} y={self.y:.4f} θ={math.degrees(self.theta):.1f}° '
                f'v={v:.3f} ω={omega:.4f} '
                f'(steer_model={omega_steer:.4f}) '
                f'dt={dt:.4f}'
            )

    # =========================================================================
    #  自行车模型：转向角 → 角速度
    # =========================================================================

    def _bicycle_omega(self, v: float, steering_angle: float) -> float:
        """
        ω = v · tan(δ) / L

        保护逻辑：
          - |v| < MIN_SPEED_FOR_STEER → 返回 0（低速时 tan(δ) 信噪比差）
          - tan 极值保护
          - 结果限幅
        """
        if abs(v) < self.MIN_SPEED_FOR_STEER:
            return 0.0

        tan_steer = math.tan(steering_angle)
        tan_steer = self._clamp(tan_steer, -50.0, 50.0)

        omega = v * tan_steer / self.L
        return self._clamp(omega, -self.MAX_OMEGA, self.MAX_OMEGA)

    # =========================================================================
    #  角速度处理（可选低通滤波）
    # =========================================================================

    def _get_omega(self, omega_raw: float, dt: float) -> float:
        """
        如果启用低通滤波，对自行车模型角速度进行一阶低通滤波；
        否则直接返回原始值。
        """
        if not self.enable_lowpass:
            return omega_raw

        if self._diag_count == 0:
            self.filtered_omega = omega_raw
        else:
            # 一阶低通滤波：beta = min(1, dt * 2*pi*fc)
            beta = min(1.0, dt * 2.0 * math.pi * self.lowpass_cutoff_hz)
            self.filtered_omega += beta * (omega_raw - self.filtered_omega)

        return self.filtered_omega

    # =========================================================================
    #  动态协方差
    # =========================================================================

    def _update_covariance(self, v: float, omega_abs: float) -> None:
        """
        根据当前运动状态动态调整协方差。

        速度越大 → 滑移风险↑ → xy 不确定度↑
        角速度越大 → 侧滑风险↑ → yaw 不确定度↑
        """
        speed = abs(v)
        speed_ratio = speed / self.max_speed if self.max_speed > 0 else 0.0

        self._pose_cov[0] = 0.002 + 0.01 * speed_ratio       # x
        self._pose_cov[7] = 0.002 + 0.01 * speed_ratio       # y
        self._pose_cov[35] = 0.002 + 0.02 * omega_abs         # yaw

        self._twist_cov[0] = 0.002 + 0.005 * speed_ratio      # vx
        self._twist_cov[35] = 0.002 + 0.01 * omega_abs        # vyaw

    # =========================================================================
    #  消息发布
    # =========================================================================

    def _publish_odometry(self, v: float, omega: float, timestamp) -> None:
        """发布 nav_msgs/Odometry"""
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_link_frame

        half_theta = self.theta / 2.0
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(half_theta),
            w=math.cos(half_theta),
        )
        msg.pose.covariance = self._pose_cov

        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = omega
        msg.twist.covariance = self._twist_cov

        self.odom_pub.publish(msg)

    def _publish_tf(self, timestamp) -> None:
        """发布 odom → base_link 变换和 /path 轨迹"""
        # ---- TF ----
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        half_theta = self.theta / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_theta)
        t.transform.rotation.w = math.cos(half_theta)

        self.tf_broadcaster.sendTransform(t)

        # ---- /path 轨迹 ----
        pose = PoseStamped()
        pose.header.stamp = timestamp.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = t.transform.rotation
        self.path_poses.append(pose)

        # 保留最近 5000 个点
        if len(self.path_poses) > 5000:
            self.path_poses = self.path_poses[-5000:]

        # 降频发布（如 50Hz odom → 5Hz path）
        self._publish_count += 1
        if self._publish_count % 10 == 0:
            path_msg = Path()
            path_msg.header.stamp = timestamp.to_msg()
            path_msg.header.frame_id = 'odom'
            path_msg.poses = self.path_poses
            self.path_pub.publish(path_msg)

    # =========================================================================
    #  工具方法
    # =========================================================================

    @staticmethod
    def _clamp(val: float, lo: float, hi: float) -> float:
        """值限幅"""
        return max(lo, min(hi, val))

    def _warn_throttled(self, msg: str) -> None:
        """限流警告（前 5 次全量打印，之后每 500 次一次）"""
        self._warn_count += 1
        if self._warn_count <= 5 or self._warn_count % 500 == 0:
            self.get_logger().warn(msg)

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """重置里程计位姿"""
        self.x = x
        self.y = y
        self.theta = theta
        self.last_timestamp = None
        self.filtered_omega = 0.0
        self._still_counter = 0
        self.get_logger().info(
            f'里程计已重置 → ({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°)'
        )

    @property
    def pose(self) -> tuple:
        """返回当前位姿 (x, y, theta)"""
        return (self.x, self.y, self.theta)

    @property
    def summary(self) -> str:
        """返回可读的位姿摘要"""
        return (f'Odom: x={self.x:.3f}m, y={self.y:.3f}m, '
                f'θ={math.degrees(self.theta):.1f}°')

