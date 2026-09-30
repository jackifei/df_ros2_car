cmake_minimum_required(VERSION 3.8)
project(df_joy_sbus)

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
  "msg/JoyValue.msg"
  DEPENDENCIES std_msgs
)

# 将驱动脚本安装为两个可执行名（sbus_joy / get_topic_joy），
# 避免与 rosidl_generate_interfaces 内部的 ament_python_install_package 冲突。
install(PROGRAMS
  df_joy_sbus/joy_device.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sbus_joy
)
install(PROGRAMS
  df_joy_sbus/joy_device.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME get_topic_joy
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
