cmake_minimum_required(VERSION 3.8)
project(ackermann_hardware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ackermann_hardware_bridge.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 使用 ament_target_dependencies 自动处理链接和包含目录
ament_target_dependencies(${PROJECT_NAME}
  "hardware_interface"
  "pluginlib"
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
  "sensor_msgs"
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装插件描述文件
install(FILES plugins/ackermann_hardware_bridge.xml
   DESTINATION share/${PROJECT_NAME}
 )
pluginlib_export_plugin_description_file(hardware_interface plugins/ackermann_hardware_bridge.xml)
# 导出 CMake 目标
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hardware_interface)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()