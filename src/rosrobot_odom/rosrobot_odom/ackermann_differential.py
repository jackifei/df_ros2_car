#!/usr/bin/env python3
"""
阿克曼电子差速模块
====================
根据车辆线速度和前轮转向角计算左右后轮速度差。

阿克曼转向几何:
  车辆转弯时四个轮子绕同一个瞬心旋转。
  内轮（靠近转向中心）速度慢，外轮速度快。

参数:
  L (wheelbase):    前后轴轴距 (m)  0.36
  W (track_width):  后轮轮距 (m)    0.39

公式:
  R = L / tan(δ)                          转弯半径
  v_left  = v × (R - W/2) / R             内轮速度
  v_right = v × (R + W/2) / R             外轮速度
  δ = atan(L × ω / v)                     从Twist反算等效转向角
"""

import math


class AckermannDifferential:
    """
    阿克曼电子差速计算器

    Usage:
        ackermann = AckermannDifferential(wheelbase=0.30, track_width=0.20)
        v_left, v_right = ackermann.compute_wheel_speeds(
            linear_vel=0.5,
            steering_angle=0.3
        )
    """

    def __init__(self, wheelbase: float, track_width: float,max_steering_angle: float = math.pi / 6):
        """
        Args:
            wheelbase:   轴距 L — 前后轴间距 (m)
            track_width: 后轮轮距 W — 左右后轮间距 (m)
        """
        if wheelbase <= 0 or track_width <= 0:
            raise ValueError(
                f'wheelbase ({wheelbase}) and track_width ({track_width}) must be positive'
            )
        self.L = float(wheelbase)
        self.W = float(track_width)

        self.max_steering = float(max_steering_angle)


    def compute_wheel_speeds(self, linear_vel: float, steering_angle: float) -> tuple:
        """
        根据线速度和前轮转向角计算左右后轮速度

        Args:
            linear_vel:     车辆质心线速度 (m/s)，前进为正
            steering_angle: 前轮转向角 (rad)，左转为正

        Returns:
            (v_left, v_right): 左后轮速度, 右后轮速度 (m/s)
        """

        # 钳位转向角到 [-max_steering, +max_steering] 限制到正负45度，0.785弧度
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))


        # 直行：两轮速度相等
        if abs(steering_angle) < 1e-6:
            return linear_vel, linear_vel

        # 转弯半径 R = L / tan(|δ|)
        R = self.L / math.tan(abs(steering_angle))

        # 确保转弯半径不小于半轮距（避免负速度或除零）
        half_track = self.W / 2.0
        if abs(R) < half_track:
            R = math.copysign(half_track, R)

        # 左右轮速度R = self.L / math.tan(abs(steering_angle))
        v_left  = linear_vel * (R - half_track) / R
        v_right = linear_vel * (R + half_track) / R

        if steering_angle > 0:          # 左转：左内右外
            return v_left, v_right
        elif steering_angle < 0:        # 右转：右内左外
            return v_right, v_left
        else:                           # 直行（前面已过滤，保留防御性代码）
            return v_left, v_right
        


    def steering_angle_from_twist(self, linear_vel: float, angular_vel: float) -> float:
        """
        从线速度和角速度反算等效前轮转向角
        self.max_steering = math.radians(30)  # ≈ 0.5236 rad
        用于手柄输入场景：手柄发布 Twist（v, ω），
        需要反算出对应的前轮转向角 δ。

        推导: ω = v / R = v × tan(δ) / L  →  δ = atan(L × ω / v)

        Args:
            linear_vel:  线速度 v (m/s)
            angular_vel: 角速度 ω (rad/s)

        Returns:
            steering_angle: 等效前轮转向角 δ (rad)
        """
        if abs(linear_vel) > 0.01:
            # 正常行驶: δ = atan(L × ω / v)
            raw = self.L * angular_vel / linear_vel
            angle = math.atan(raw)
            # 将转向角限制在 ±30度（±max_steering）范围内
            return max(-self.max_steering, min(self.max_steering, angle))
        else:
            # 将角速度映射到转向角，gain 可根据实际调试确定
            # 例如：当 ω = 1.0 rad/s 时，希望转向角达到满偏 30度
            gain = self.max_steering / 1.0  # 1.0 rad/s 对应满偏
            angle = angular_vel * gain
            return max(-self.max_steering, min(self.max_steering, angle))
       

    def compute_from_twist(self, linear_vel: float, angular_vel: float) -> tuple:
        """
        便捷方法：直接从 Twist 的 v, ω 计算后轮速度

        Args:
            linear_vel:  线速度 (m/s)
            angular_vel: 角速度 (rad/s)

        Returns:
            (v_left, v_right, steering_angle)
        """
        steering_angle = self.steering_angle_from_twist(linear_vel, angular_vel)
        v_left, v_right = self.compute_wheel_speeds(linear_vel, steering_angle)
        return v_left, v_right, steering_angle

    @property
    def summary(self) -> str:
        """返回模块参数摘要"""
        return (
            f'AckermannDifferential(L={self.L:.3f}m, W={self.W:.3f}m) | '
            f'Min turn radius: {self.L:.3f}m'
        )
