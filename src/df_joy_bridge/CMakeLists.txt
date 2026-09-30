cmake_minimum_required(VERSION 3.8)
project(df_joy_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JoyState.msg"
  DEPENDENCIES std_msgs
)

# 安装 Python 桥接脚本为可执行文件
install(PROGRAMS
  df_joy_bridge/joy_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joy_bridge
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
