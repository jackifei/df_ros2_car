#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
df_joy_bridge 桥接/处理节点。

功能：
  1. 订阅 /joy_value（df_joy_sbus/msg/JoyValue，原始 SBUS 通道值）；
  2. 把前 10 个通道线性映射到微秒（默认 1000~2000）；
  3. 把 ch06/ch07 按阈值转换为开关量 0/1；
  4. 发布 /joy_state（df_joy_bridge/msg/JoyState）。

映射区间、开关通道和阈值都通过 config/bridge_config.yaml 配置。
本文件中所有「修改点」注释都是预留的修改位置，按需调整。
"""

import rclpy
from rclpy.node import Node
from df_joy_sbus.msg import JoyValue
from df_joy_bridge.msg import JoyState


class JoyBridgeNode(Node):
    """订阅原始手柄数据，映射后发布状态消息的节点。"""

    def __init__(self):
        super().__init__('joy_bridge')

        # ---- 声明参数（默认值会被 yaml 覆盖）----
        self.declare_parameter('input_topic', 'joy_value')
        self.declare_parameter('output_topic', 'joy_state')
        self.declare_parameter('raw_min', 192)
        self.declare_parameter('raw_max', 1792)
        self.declare_parameter('us_min', 1000)
        self.declare_parameter('us_max', 2000)
        self.declare_parameter('switch_channels', [6, 7])
        self.declare_parameter('switch_threshold', 1500)

        # ---- 读取参数 ----
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._raw_min = float(self.get_parameter('raw_min').value)
        self._raw_max = float(self.get_parameter('raw_max').value)
        self._us_min = float(self.get_parameter('us_min').value)
        self._us_max = float(self.get_parameter('us_max').value)
        self._switch_channels = list(
            self.get_parameter('switch_channels').value)
        self._switch_threshold = float(
            self.get_parameter('switch_threshold').value)

        self._publisher = self.create_publisher(JoyState, output_topic, 10)
        self._subscription = self.create_subscription(
            JoyValue, input_topic, self._on_joy, 10)

        self.get_logger().info(
            f'Bridge ready: {input_topic} -> {output_topic}')

    # 修改点：映射规则
    def _map_to_us(self, raw):
        """把原始值线性映射到微秒，并限幅到 [us_min, us_max]。"""
        span = self._raw_max - self._raw_min
        if span == 0.0:
            return int(round(self._us_min))
        us = self._us_min + (raw - self._raw_min) * \
            (self._us_max - self._us_min) / span
        us = max(self._us_min, min(self._us_max, us))
        return int(round(us))

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

        # 2. 线性映射到微秒
        us_values = [self._map_to_us(v) for v in raw_channels]

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

        # 4. 开关量：按 yaml 里的 switch_channels / switch_threshold 转换
        # 修改点：如需更多开关通道，扩展 JoyState.msg 并在此追加
        out.switch06 = self._switch_state(us_values[5]) \
            if 6 in self._switch_channels else 0
        out.switch07 = self._switch_state(us_values[6]) \
            if 7 in self._switch_channels else 0

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
