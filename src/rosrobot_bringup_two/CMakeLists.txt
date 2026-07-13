cmake_minimum_required(VERSION 3.8)
project(rosrobot_bringup_two)

find_package(ament_cmake REQUIRED)

# 安装目录
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 Python 脚本
install(
  PROGRAMS
    scripts/cmd_vel_to_joints_sync.py
    scripts/publish_robot_description.py
    scripts/ackermann_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
