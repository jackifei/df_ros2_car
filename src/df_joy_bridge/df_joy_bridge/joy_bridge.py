#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
df_joy_bridge 桥接/处理节点。

功能：
  1. 订阅 /joy_value（df_joy_sbus/msg/JoyValue，原始 SBUS 通道值）；
  2. 把前 10 个通道按各自配置线性映射到微秒（默认 1000~2000）；
  3. 把固定开关通道 ch06/ch07 按阈值转换为开关量 0/1；
  4. 发布 /joy_state（df_joy_bridge/msg/JoyState）。

每个通道的映射区间和开关阈值都通过 config/bridge_config.yaml 配置，
开关通道固定为 6/7（ch06/ch07 的映射值仍然照常发布）。
本文件中所有「修改点」注释都是预留的修改位置，按需调整。
"""

import rclpy
from rclpy.node import Node
from df_joy_sbus.msg import JoyValue
from df_joy_bridge.msg import JoyState

# 通道号固定为前 10 个通道，与 JoyState.msg 中的 ch01~ch10 保持一致
CHANNEL_KEYS = [
    'ch01', 'ch02', 'ch03', 'ch04', 'ch05',
    'ch06', 'ch07', 'ch08', 'ch09', 'ch10',
]

# 开关通道固定为 6、7（从 1 开始的通道号）
SWITCH_CHANNELS_1 = 5
SWITCH_CHANNELS_2 = 6


class JoyBridgeNode(Node):
    """订阅原始手柄数据，映射后发布状态消息的节点。"""

    def __init__(self):
        super().__init__('joy_bridge')

        # ---- 声明参数（默认值会被 yaml 覆盖）----
        self.declare_parameter('input_topic', 'joy_value')
        self.declare_parameter('output_topic', 'joy_state')
        self.declare_parameter('switch_threshold', 1500.0)
        # 每个通道单独映射：channel_mapping.<通道号>.{raw_min,raw_max,out_min,out_max}
        for key in CHANNEL_KEYS:
            self.declare_parameter(f'channel_mapping.{key}.raw_min', 192)
            self.declare_parameter(f'channel_mapping.{key}.raw_max', 1792)
            self.declare_parameter(f'channel_mapping.{key}.out_min', 1000.0)
            self.declare_parameter(f'channel_mapping.{key}.out_max', 2000.0)

        # ---- 读取参数 ----
        input_topic = self.get_parameter('input_topic').value  # 订阅的手柄话题
        output_topic = self.get_parameter('output_topic').value # 发布的话题名称

        self._switch_threshold = float(
            self.get_parameter('switch_threshold').value)

        self._channel_maps = {}
        for key in CHANNEL_KEYS:
            self._channel_maps[key] = {
                'raw_min': float(
                    self.get_parameter(f'channel_mapping.{key}.raw_min').value),
                'raw_max': float(
                    self.get_parameter(f'channel_mapping.{key}.raw_max').value),
                'out_min': float(
                    self.get_parameter(f'channel_mapping.{key}.out_min').value),
                'out_max': float(
                    self.get_parameter(f'channel_mapping.{key}.out_max').value),
            }

        # 发布话题
        self._publisher = self.create_publisher(JoyState, output_topic, 10)
        # 订阅话题
        self._subscription = self.create_subscription(JoyValue, input_topic, self._on_joy, 10)

        self.get_logger().info(
            f'手柄桥接话题程序准备: {input_topic} -> {output_topic}')
        self.get_logger().info(
            f'开关通道固定为 6/7，判定阈值 {self._switch_threshold} us')

    # 修改点：映射规则（每个通道独立映射）
    def _map_channel(self, key, raw):
        """把指定通道的原始值按该通道自己的区间线性映射并限幅。"""
        m = self._channel_maps[key]
        span = m['raw_max'] - m['raw_min']
        if span == 0.0:
            return m['out_min']
        us = m['out_min'] + (raw - m['raw_min']) * \
            (m['out_max'] - m['out_min']) / span
        lo, hi = sorted((m['out_min'], m['out_max']))
        us = max(lo, min(hi, us))
        return us

    # 修改点：开关判定规则
    def _switch_state(self, us_value):
        """把微秒值转换为 0/1，默认 >= 阈值判为 1。"""
        return 1 if us_value >= self._switch_threshold else 0

    def _on_joy(self, msg):
        """收到原始数据后的处理回调。"""
        # 1. 取出前 10 个通道原始值
        # 修改点：若增减通道，需同步修改 JoyState.msg 和下方赋值
        raw_channels = [
            msg.ch01, msg.ch02, msg.ch03, msg.ch04, msg.ch05,
            msg.ch06, msg.ch07, msg.ch08, msg.ch09, msg.ch10,
        ]

        # 2. 每个通道按各自配置线性映射到微秒（ch06/ch07 也照常映射并发布）
        us_values = [
            self._map_channel(key, v)
            for key, v in zip(CHANNEL_KEYS, raw_channels)
        ]

        # 3. 组装输出消息
        out = JoyState()
        out.header = msg.header
        out.ch01 = us_values[0]
        out.ch02 = us_values[1]
        out.ch03 = us_values[2]
        out.ch04 = us_values[3]
        out.ch05 = us_values[4]
        out.ch06 = us_values[5]
        out.ch07 = us_values[6]
        out.ch08 = us_values[7]
        out.ch09 = us_values[8]
        out.ch10 = us_values[9]

        # 4. 开关量：开关通道固定为 6/7，按 switch_threshold 转换
        out.switch06 = self._switch_state(us_values[SWITCH_CHANNELS_1])
        out.switch07 = self._switch_state(us_values[SWITCH_CHANNELS_2])

        self._publisher.publish(out)


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    node = JoyBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
