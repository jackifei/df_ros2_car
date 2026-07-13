#!/usr/bin/env python3
"""
publish_robot_description.py
读取 robot_description 参数，以 Transient Local QoS 发布到 /robot_description 话题，
供 RViz2 RobotModel 显示加载并渲染 URDF 中的 STL 网格模型。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    """发布 robot_description 到 /robot_description 话题（Transient Local，仅发一次）"""

    def __init__(self):
        super().__init__('robot_description_publisher')

        # 声明参数并读取
        self.declare_parameter('robot_description', '')
        robot_desc = self.get_parameter('robot_description').value

        if not robot_desc:
            self.get_logger().error(
                'robot_description 参数为空！RViz2 将无法显示 STL 模型。'
            )
            return

        self.get_logger().info(
            f'robot_description 参数长度: {len(robot_desc)} 字符'
        )

        # 检查是否仍残留旧包名
        if 'rosrobottt' in robot_desc:
            self.get_logger().warn(
                'URDF 中仍存在 "rosrobottt" 引用，mesh 文件路径可能错误！'
            )
        else:
            self.get_logger().info('包名检查通过 (无 rosrobottt 残留)')

        # 打印 mesh 路径供调试
        import re
        meshes = re.findall(r'filename="([^"]*)"', robot_desc)
        if meshes:
            self.get_logger().info(f'URDF 中引用的 mesh 文件 ({len(meshes)} 个):')
            for m in meshes:
                self.get_logger().info(f'  {m}')
        else:
            self.get_logger().warn('URDF 中未找到任何 mesh 引用！')

        # Transient Local QoS —— 后来的订阅者也能收到
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.create_publisher(String, '/robot_description', qos)

        msg = String()
        msg.data = robot_desc
        self.publisher.publish(msg)
        self.get_logger().info('已发布到 /robot_description 话题')


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
