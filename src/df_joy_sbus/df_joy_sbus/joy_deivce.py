
"""
@作者: 邓飞
@说明: 订阅joy手柄话题
"""

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
# from sensor_msgs.msg import Image       # 图像消息类型
from sensor_msgs.msg import Joy

# from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import numpy as np                      # Python数值计算库
# import pymodbus


"""
创建一个订阅者节点
数组	定义	    说明	    数组	定义	说明
摇杆0	左摇杆左右	左正右负	按钮0	A	
摇杆1	左摇杆前后	左正右负	按钮1	B	
摇杆2	L2	        禁用	    按钮2	X	
摇杆3	右摇杆左右	左正右负	按钮3	Y	
摇杆4	右摇杆前后	前正后负	按钮4	L1	
摇杆5	R2	        禁用	    按钮5	R1	
摇杆6	左左右	    左正右负	按钮6	select	
摇杆7	左前后	    前正后负	 按钮7	start	
                                按钮8	mode	
                                按钮9	左摇杆按下	
                                按钮10	右摇杆按下	

"""
class JoySubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Joy, 'joy', self.listener_callback, 10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）


    def object_detect(self, msg:Joy):
        # print(result)
        # 左摇杆 前进  +
        msg.axes[1]
        # 左摇杆 后退 -
        msg.axes[1]

        # 左摇杆 左转  +
        msg.axes[0]
        # 左摇杆 右转  -
        msg.axes[0]
        
        self.get_logger().info(f'摇杆:{msg.axes}  按钮:{msg.buttons}')         # 输出日志信息，提示已进入回调函数

    def listener_callback(self, msg:Joy):
        # self.get_logger().info('Receiving video frame')         # 输出日志信息，提示已进入回调函数
        # image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # 将ROS的图像消息转化成OpenCV图像
        self.object_detect(msg)                               # 苹果检测


def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = JoySubscriber("topic_joy_sub")              # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口