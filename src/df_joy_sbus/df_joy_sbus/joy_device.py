#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SBUS 手柄驱动节点：读取串口 SBUS 原始通道值并发布到 joy_value 话题。

串口参数固定为 SBUS 协议要求：100000 波特、8 数据位、偶校验、2 停止位。
每帧 25 字节，包含 16 个模拟通道（每个 11 bit，范围 0~2047）和 2 个数字
通道（CH17/CH18，位于标志字节的 bit0/bit1）。

发布内容（话题名可配置，默认 joy_value，消息类型 df_joy_sbus/msg/JoyValue）：
    - ch01 ~ ch16 : 16 个模拟通道原始值（uint16，0~2047，不做归一化）
    - ch17 / ch18 : 两个数字通道（uint8，0/1）
"""

import rclpy
from rclpy.node import Node
from df_joy_sbus.msg import JoyValue
import serial


SBUS_BAUDRATE = 100000
SBUS_BYTES_PER_FRAME = 25
SBUS_START_BYTE = 0x0F
SBUS_END_BYTE = 0x00
SBUS_CHANNEL_COUNT = 16


def parse_sbus_frame(data):
    """解析一帧 25 字节 SBUS 数据。

    返回 ``(channels, ch17, ch18, frame_lost, failsafe)``。
    格式非法时抛出 ValueError。
    """
    if len(data) != SBUS_BYTES_PER_FRAME:
        raise ValueError('frame length mismatch')
    if data[0] != SBUS_START_BYTE:
        raise ValueError('bad start byte')
    if data[-1] != SBUS_END_BYTE:
        raise ValueError('bad end byte')

    channels = []
    for i in range(SBUS_CHANNEL_COUNT):
        bit_offset = i * 11
        byte_index = 1 + bit_offset // 8
        bit_index = bit_offset % 8
        raw = (
            data[byte_index]
            | (data[byte_index + 1] << 8)
            | (data[byte_index + 2] << 16)
        )
        channels.append((raw >> bit_index) & 0x07FF)

    flags = data[23]
    ch17 = (flags >> 0) & 0x01
    ch18 = (flags >> 1) & 0x01
    frame_lost = (flags >> 2) & 0x01
    failsafe = (flags >> 3) & 0x01
    return channels, ch17, ch18, frame_lost, failsafe


class SbusJoyNode(Node):
    """从 SBUS 串口读取手柄原始通道值并发布 JoyValue 的节点。"""

    def __init__(self):
        super().__init__('sbus_joy')

        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baudrate', SBUS_BAUDRATE)
        self.declare_parameter('frame_id', 'joy')
        self.declare_parameter('poll_hz', 200.0)
        self.declare_parameter('topic', 'joy_value')

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self._frame_id = self.get_parameter('frame_id').value
        poll_hz = self.get_parameter('poll_hz').value
        topic = self.get_parameter('topic').value

        self._publisher = self.create_publisher(JoyValue, topic, 10)

        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            timeout=0,
        )
        self._buffer = bytearray()
        self._last_signal_warn = 0.0

        self._timer = self.create_timer(1.0 / poll_hz, self._read_callback)
        self.get_logger().info(
            f'Opened SBUS on {port} @ {baudrate} (8E2), '
            f'publishing to /{topic}'
        )

    def _read_callback(self):
        """定时器回调：非阻塞读取串口并解析完整帧。"""
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                self._buffer.extend(self._serial.read(waiting))
            self._drain_frames()
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial error: {exc}')
            self.destroy_timer(self._timer)

    def _drain_frames(self):
        """从缓冲区中取出所有完整帧并发布。"""
        while len(self._buffer) >= SBUS_BYTES_PER_FRAME:
            start = self._buffer.find(SBUS_START_BYTE)
            if start < 0:
                self._buffer.clear()
                return
            if start > 0:
                del self._buffer[:start]
                continue

            frame = bytes(self._buffer[:SBUS_BYTES_PER_FRAME])
            if frame[-1] != SBUS_END_BYTE:
                del self._buffer[0]
                continue
            del self._buffer[:SBUS_BYTES_PER_FRAME]

            try:
                channels, ch17, ch18, frame_lost, failsafe = \
                    parse_sbus_frame(frame)
            except ValueError:
                continue
            self._publish(channels, ch17, ch18)
            self._warn_signal_issue(frame_lost, failsafe)

    def _publish(self, channels, ch17, ch18):
        """构造并发布 JoyValue 消息（原始通道值，不归一化）。"""
        msg = JoyValue()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        for index, value in enumerate(channels, start=1):
            setattr(msg, 'ch{:02d}'.format(index), int(value))
        msg.ch17 = int(ch17)
        msg.ch18 = int(ch18)
        self._publisher.publish(msg)

    def _warn_signal_issue(self, frame_lost, failsafe):
        """丢帧或失控保护时按 2 秒节流输出一次警告。"""
        if not frame_lost and not failsafe:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_signal_warn < 2.0:
            return
        self._last_signal_warn = now
        self.get_logger().warn(
            f'Signal issue detected: frame_lost={frame_lost}, '
            f'failsafe={failsafe}'
        )


def main(args=None):
    """ROS2 节点主入口。"""
    rclpy.init(args=args)
    node = SbusJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
