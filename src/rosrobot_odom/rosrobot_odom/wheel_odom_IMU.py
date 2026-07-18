#!/usr/bin/env python3
"""
增强型轮式里程计 — 阿克曼底盘多传感器融合
=============================================

融合四种传感器数据进行稳定航位推算：

  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐
  │ 后轮编码器    │  │ 前轮转向角    │  │ 6轴 IMU             │
  │ → 线速度 v   │  │ → δ          │  │ → ω_z, yaw          │
  └──────┬───────┘  └──────┬───────┘  └─────────┬───────────┘
         │                 │                     │
         ▼                 ▼                     ▼
  ┌──────────────────────────────────────────────────────────┐
  │              互补滤波器 (Complementary Filter)            │
  │                                                          │
  │  ω_fused = α·ω_imu + (1-α)·ω_steer                       │
  │  其中 ω_steer = v·tan(δ)/L  (自行车模型)                 │
  │                                                          │
  │  IMU yaw → 缓慢修正积分航向漂移                           │
  │  零速检测 → 静止时锁定位姿不积分                          │
  └──────────────────────────────────────────────────────────┘
         │
         ▼
  ┌──────────────────┐
  │  中点法积分       │
  │  θ_mid = θ+Δθ/2  │
  │  Δx = v·cos(θ_mid)·Δt                                   │
  │  Δy = v·sin(θ_mid)·Δt                                   │
  └──────────────────┘
         │
         ▼
  ┌──────────────────┐
  │  /odom + TF       │
  └──────────────────┘


与 wheel_odometry.py 相比的改进：
  1. 角速度融合 — IMU陀螺仪(高频低延迟) + 转向角模型(无漂移) → 互补滤波
  2. 航向角约束 — IMU yaw 作为绝对参考，缓慢修正积分漂移
  3. 中点法积分 — 比前向欧拉法高一阶精度，转弯轨迹更准
  4. 动态互补系数 — 低速信任转向模型，高速信任IMU陀螺仪
  5. 零速检测 — 静止时锁定位姿，防止编码器噪声漂移
  6. 异常保护 — dt跳变、传感器超限、tan极值保护

适用条件：
  - 阿克曼转向底盘（前轮转向，后轮驱动）
  - 具备后轮编码器、前轮转向角传感器、6轴IMU
  - ROS2 Jazzy
"""
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class WheelOdomFusion:
    """
    融合编码器 + 转向角 + IMU 的增强型轮式里程计。

    设计原则：
      - 角速度：IMU陀螺仪主导高频响应 + 转向角模型修正漂移
      - 航向角：积分角速度为主 + IMU yaw 缓慢拉回绝对参考
      - 积分法：中点法（二阶精度）
      - 协方差：动态调整，反映真实不确定度
      - 鲁棒性：多层异常保护，单传感器故障可降级运行

    Usage:
        odom = WheelOdomFusion(node, wheelbase=0.306, track_width=0.39,
                               max_speed=0.3)
        # 由 IMU 回调或定时器驱动，每周期调用:
        odom.update(
            v=0.15,             # 编码器线速度 (m/s)
            steering_angle=0.1, # 前轮转向角 (rad)
            omega_imu=0.05,     # IMU z轴角速度 (rad/s)
            yaw_imu=1.57,       # IMU 航向角 (rad)，可选
            timestamp=now       # rclpy.time.Time
        )
    """

    # =========================================================================
    # 限幅常数
    # =========================================================================
    MAX_STEER = math.pi / 3.0      # 60°，前轮物理限位
    MAX_OMEGA = 5.0                # rad/s，角速度上限
    MIN_SPEED_FOR_STEER = 0.02     # m/s，低于此速度转向模型不可靠
    MAX_DT = 0.2                   # s，超过视为数据中断

    # 零速检测阈值
    STILL_V_THRESHOLD = 0.005      # m/s
    STILL_W_THRESHOLD = 0.01       # rad/s
    STILL_FRAME_COUNT = 10         # 连续静止帧数

    # IMU 陀螺仪噪声死区（rad/s），低于此值的角速度视为传感器底噪
    GYRO_NOISE_FLOOR = 0.005       # rad/s ≈ 0.3°/s

    # IMU yaw 初始化参数
    YAW_INIT_SAMPLES = 30          # 初始化时收集的样本数

    def __init__(self, node, wheelbase: float, track_width: float,
                 wheel_radius: float = 0.062,
                 max_speed: float = 0.3,
                 alpha_gyro: float = 0.92,
                 yaw_correction_gain: float = 0.005,
                 publish_rate: float = 50.0):
        """
        Args:
            node:               ROS2 Node 实例（用于创建发布器和日志）
            wheelbase:          轴距 L (m) — 用于 ω=v·tan(δ)/L
            track_width:        后轮轮距 W (m) — 用于日志和诊断
            wheel_radius:       车轮半径 (m) — 用于日志
            max_speed:          小车最大线速度 (m/s)
                                用于：速度限幅上限、互补滤波速度归一化
            alpha_gyro:         互补滤波系数 [0, 1]
                                1.0 = 完全信任 IMU 陀螺仪
                                0.0 = 完全信任转向角模型
                                推荐值: 0.90 ~ 0.95
            yaw_correction_gain: IMU 航向角修正增益 [0, 1]
                                越大修正越快，但不平滑
                                推荐值: 0.001 ~ 0.01
            publish_rate:       名义发布频率 (Hz)，仅用于参考
        """
        # ---- 参数合法性校验 ----
        if wheelbase <= 0:
            raise ValueError(f'wheelbase 必须 > 0，当前值: {wheelbase}')
        if max_speed <= 0:
            raise ValueError(f'max_speed 必须 > 0，当前值: {max_speed}')

        self.node = node
        self.L = float(wheelbase)
        self.W = float(track_width)
        self.r = float(wheel_radius)
        self.max_speed = float(max_speed)
        self.alpha_gyro = float(alpha_gyro)
        self.yaw_gain = float(yaw_correction_gain)

        # 速度限幅取 max_speed 的 1.5 倍（留余量给下坡/滑移等超速场景）
        self._speed_limit = self.max_speed * 1.5

        # =====================================================================
        #  位姿状态
        # =====================================================================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # =====================================================================
        #  时间追踪
        # =====================================================================
        self.last_timestamp = None

        # =====================================================================
        #  滤波器状态
        # =====================================================================
        self.filtered_omega = 0.0       # 融合角速度（低通输出）

        # =====================================================================
        #  IMU 航向角校正状态
        #  记录的是 theta - yaw_imu 的差值序列，即使小车在运动中初始化也正确
        # =====================================================================
        self._yaw_ready = False
        self._yaw_offset = 0.0          # mean(theta - yaw_imu)
        self._yaw_offset_samples = []   # 初始化阶段收集的 (theta - yaw_imu)

        # =====================================================================
        #  零速检测
        # =====================================================================
        self._still_counter = 0

        # =====================================================================
        #  发布器
        # =====================================================================
        self.odom_pub = node.create_publisher(Odometry, '/odom', 10)

        self.path_pub = node.create_publisher(
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
        self.tf_broadcaster = TransformBroadcaster(node)

        # =====================================================================
        #  坐标系名称
        # =====================================================================
        self.odom_frame = 'odom'
        self.base_link_frame = 'base_link'

        # =====================================================================
        #  协方差矩阵（初始值）
        #  索引: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
        # =====================================================================
        self._pose_cov = [0.0] * 36
        self._pose_cov[0] = 0.01
        self._pose_cov[7] = 0.01
        self._pose_cov[35] = 0.02   # 融合后 yaw 不确定度低于纯 IMU

        self._twist_cov = [0.0] * 36
        self._twist_cov[0] = 0.01
        self._twist_cov[35] = 0.01  # 融合后 vyaw 更可信

        # =====================================================================
        #  计数器
        # =====================================================================
        self._publish_count = 0
        self._diag_count = 0
        self._warn_count = 0

        node.get_logger().info(
            f'WheelOdomFusion 已初始化:\n'
            f'  L={self.L:.3f}m  W={self.W:.3f}m  r={self.r:.3f}m\n'
            f'  max_speed={self.max_speed:.2f}m/s  speed_limit={self._speed_limit:.2f}m/s\n'
            f'  α_gyro={self.alpha_gyro}  yaw_gain={self.yaw_gain}\n'
            f'  融合源: 编码器v + 转向角δ + IMU陀螺仪ω_z + IMU航向角yaw'
        )

    # =========================================================================
    #  公开 API — 主更新入口
    # =========================================================================

    def update(self, v: float, steering_angle: float,
               omega_imu: float, yaw_imu: float = None,
               timestamp=None) -> None:
        """
        融合多传感器数据，积分更新位姿并发布 /odom 和 TF。

        调用频率建议 >= 50Hz（与 IMU 发布频率一致）。

        Args:
            v:              后轴中点线速度 (m/s)，前进为正
            steering_angle: 前轮转向角 (rad)，左转为正
            omega_imu:      IMU 陀螺仪 z 轴角速度 (rad/s)，逆时针为正
            yaw_imu:        IMU 航向角 (rad)，可选。
                            如果提供，将用于缓慢修正积分漂移。
                            首次调用后需要约 0.5~1s 初始化（30帧）。
            timestamp:      rclpy.time.Time 对象
        """
        # ------------------------------------------------------------------
        # Step 1: 输入有效性检查（四个传感器全部校验）
        # ------------------------------------------------------------------
        if not (math.isfinite(v) and math.isfinite(steering_angle) and
                math.isfinite(omega_imu)):
            self._warn_throttled(
                f"无效输入: v={v}, δ={steering_angle}, ω_imu={omega_imu}"
            )
            return

        if yaw_imu is not None and not math.isfinite(yaw_imu):
            self._warn_throttled(f"无效 IMU yaw: {yaw_imu}")
            yaw_imu = None

        # ------------------------------------------------------------------
        # Step 2: 时间步长计算 & 异常保护
        # ------------------------------------------------------------------
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return

        dt = (timestamp - self.last_timestamp).nanoseconds / 1e9

        if dt <= 1e-9:
            return

        if dt > self.MAX_DT:
            self.node.get_logger().warn(
                f'数据中断: dt={dt:.3f}s > {self.MAX_DT}s, 重置积分参考'
            )
            self.last_timestamp = timestamp
            return

        self.last_timestamp = timestamp

        # ------------------------------------------------------------------
        # Step 3: 限幅保护
        # ------------------------------------------------------------------
        v = self._clamp(v, -self._speed_limit, self._speed_limit)
        omega_imu = self._clamp(omega_imu, -self.MAX_OMEGA, self.MAX_OMEGA)
        steering_angle = self._clamp(steering_angle,
                                     -self.MAX_STEER, self.MAX_STEER)

        # ------------------------------------------------------------------
        # Step 4: 零速检测 — 静止时锁定位姿，不积分
        # ------------------------------------------------------------------
        is_still = (abs(v) < self.STILL_V_THRESHOLD and
                    abs(omega_imu) < self.STILL_W_THRESHOLD)

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
        # Step 5: IMU 航向角偏移初始化（必须在积分之前）
        #         记录 theta - yaw_imu 差值，运动中也正确
        # ------------------------------------------------------------------
        if yaw_imu is not None and not self._yaw_ready:
            self._init_yaw_offset(yaw_imu)

        # ------------------------------------------------------------------
        # Step 6: 转向角 → 角速度（自行车模型）
        # ------------------------------------------------------------------
        omega_steer = self._bicycle_omega(v, steering_angle)

        # ------------------------------------------------------------------
        # Step 7: 互补滤波 — 融合 IMU 陀螺仪 + 转向角模型
        # ------------------------------------------------------------------
        omega_fused = self._complementary_filter(omega_imu, omega_steer, v, dt)

        # ------------------------------------------------------------------
        # Step 8: IMU 航向角漂移修正
        # ------------------------------------------------------------------
        if yaw_imu is not None and self._yaw_ready:
            self._apply_yaw_correction(yaw_imu, dt)

        # ------------------------------------------------------------------
        # Step 9: 中点法积分（二阶精度）
        #
        #   θ_mid = θ + Δθ/2
        #   Δx = v · cos(θ_mid) · Δt
        #   Δy = v · sin(θ_mid) · Δt
        #   θ_new = θ + Δθ
        # ------------------------------------------------------------------
        delta_theta = omega_fused * dt
        theta_mid = self.theta + delta_theta / 2.0

        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += delta_theta

        # 归一化 θ 到 [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ------------------------------------------------------------------
        # Step 10: 动态协方差
        # ------------------------------------------------------------------
        self._update_covariance(v, abs(omega_fused))

        # ------------------------------------------------------------------
        # Step 11: 发布
        # ------------------------------------------------------------------
        self._publish_odometry(v, omega_fused, timestamp)
        self._publish_tf(timestamp)

        # ------------------------------------------------------------------
        # Step 12: 诊断日志
        # ------------------------------------------------------------------
        self._diag_count += 1
        if self._diag_count <= 10 or self._diag_count % 200 == 0:
            self.node.get_logger().info(
                f'[odom #{self._diag_count}] '
                f'x={self.x:.4f} y={self.y:.4f} θ={math.degrees(self.theta):.1f}° '
                f'v={v:.3f} ω_f={omega_fused:.4f} '
                f'(gyro={omega_imu:.4f} steer={omega_steer:.4f}) '
                f'yaw_ok={self._yaw_ready} dt={dt:.4f}'
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

        tan_steer = math.tan(steering_angle)  # 已在 update 中限幅
        tan_steer = self._clamp(tan_steer, -50.0, 50.0)

        omega = v * tan_steer / self.L
        return self._clamp(omega, -self.MAX_OMEGA, self.MAX_OMEGA)

    # =========================================================================
    #  互补滤波器
    # =========================================================================

    def _complementary_filter(self, omega_imu: float, omega_steer: float,
                               v: float, dt: float) -> float:
        """
        ω_fused = α_dyn · ω_imu + (1 - α_dyn) · ω_steer

        动态 α 策略：
          - 转向模型不可用(ω_steer≈0)：α→1.0，完全信任 IMU
            但 IMU 陀螺仪值若低于噪声死区则强制置零，防止直行时噪声积分
          - 低速：α 较低 → 偏向转向模型（无漂移）
          - 高速：α 接近 alpha_gyro → IMU 主导（响应快）

        速度归一化基准 = max_speed（而非硬编码），适配不同车速的小车。

        融合后经一阶低通滤波平滑（截止频率 ≈ 20Hz）。
        """
        if abs(omega_steer) < 1e-6:
            # 直行或极低速：转向模型无输出，完全信任 IMU
            # 但若 IMU 值低于噪声死区则置零，防止静止/直行时陀螺仪底噪漂移航向
            if abs(omega_imu) < self.GYRO_NOISE_FLOOR:
                raw_omega = 0.0
            else:
                raw_omega = omega_imu
        else:
            # 动态权重：速度越高越信任 IMU
            # speed_factor: 0(静止) → 1(≥max_speed)
            speed_factor = min(1.0, abs(v) / self.max_speed)
            # dynamic_alpha: alpha_gyro×0.3(低速) → alpha_gyro(高速)
            dynamic_alpha = self.alpha_gyro * (0.3 + 0.7 * speed_factor)
            raw_omega = (dynamic_alpha * omega_imu +
                         (1.0 - dynamic_alpha) * omega_steer)

        # 一阶低通滤波 (τ ≈ 0.05s, fc ≈ 20Hz)
        if self._diag_count == 0:
            self.filtered_omega = raw_omega
        else:
            beta = min(1.0, dt * 20.0)
            self.filtered_omega += beta * (raw_omega - self.filtered_omega)

        return self.filtered_omega

    # =========================================================================
    #  IMU 航向角偏移初始化
    # =========================================================================

    def _init_yaw_offset(self, yaw_imu: float) -> None:
        """
        收集 theta - yaw_imu 差值样本，用均值作为 offset。

        关键设计：
          记录的是每一帧的 (theta - yaw_imu) 差值，而不是 yaw_imu 绝对值。
          这样即使小车在运动中初始化，差值序列也是稳定的（两者同步变化），
          均值等于真实的初始偏移。
        """
        delta = self.theta - yaw_imu
        self._yaw_offset_samples.append(delta)

        if len(self._yaw_offset_samples) >= self.YAW_INIT_SAMPLES:
            self._yaw_offset = (
                sum(self._yaw_offset_samples) / len(self._yaw_offset_samples)
            )
            self._yaw_ready = True
            self._yaw_offset_samples.clear()  # 释放内存

            self.node.get_logger().info(
                f'IMU 航向角已初始化 ({self.YAW_INIT_SAMPLES} 样本): '
                f'yaw_offset={math.degrees(self._yaw_offset):.1f}°'
            )

    # =========================================================================
    #  IMU 航向角修正
    # =========================================================================

    def _apply_yaw_correction(self, yaw_imu: float, dt: float) -> None:
        """
        用 IMU 绝对航向角缓慢修正积分漂移。

        目标航向 = yaw_imu + offset（offset 在初始化阶段算出）

        增益调度（根据误差大小动态调整）：
          - 误差 < 2°  → ×1  基础增益，平滑跟随
          - 误差 2°~5° → ×2  加速修正
          - 误差 > 5°  → ×4  快速拉回（可能发生滑移/碰撞）
        """
        yaw_target = yaw_imu + self._yaw_offset

        # 角度误差 → 归一化到 [-π, π]
        error = math.atan2(
            math.sin(yaw_target - self.theta),
            math.cos(yaw_target - self.theta)
        )

        # 增益调度
        abs_err_deg = abs(math.degrees(error))
        gain = self.yaw_gain
        if abs_err_deg > 2.0:
            gain *= 2.0
        if abs_err_deg > 5.0:
            gain *= 2.0   # 总 ×4

        # 修正 theta（缓慢拉回）
        correction = error * gain * dt
        self.theta += correction

        # 修正后立即归一化，防止连续大修正导致越界
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 同时给 filtered_omega 加小偏置，让后续帧的角速度积分也向修正方向靠拢。
        # 系数 0.1 使偏置约为直接修正的 1/10，够平滑又不会过度累积。
        self.filtered_omega += error * gain * 0.1

    # =========================================================================
    #  动态协方差
    # =========================================================================

    def _update_covariance(self, v: float, omega_abs: float) -> None:
        """
        根据当前运动状态动态调整协方差。

        速度越大 → 滑移风险↑ → xy 不确定度↑
        角速度越大 → 侧滑风险↑ → yaw 不确定度↑

        协方差基准值针对低速小车 (≤0.3 m/s) 做了校准。
        """
        speed = abs(v)

        # 以 max_speed 为归一化基准
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
        # 使用与 odom/TF 一致的时间戳，避免时间不同步
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
            self.node.get_logger().warn(msg)

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """重置里程计位姿（也会重新初始化 IMU yaw offset）"""
        self.x = x
        self.y = y
        self.theta = theta
        self.last_timestamp = None
        self.filtered_omega = 0.0
        self._yaw_ready = False
        self._yaw_offset = 0.0
        self._yaw_offset_samples.clear()
        self._still_counter = 0
        self.node.get_logger().info(
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


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()