# 首先启动手柄
ros2 run joy joy_node

# 启动手柄转换器
ros2 run rosrobot_twist_mux joy2twist

# 启动第三个终端，用来监控话题内容
ros2 topic echo /cmd_vel_joy



