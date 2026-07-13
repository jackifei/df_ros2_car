cmake_minimum_required(VERSION 3.8)
project(rosrobot_description)

# 使用 ament_cmake 构建系统
find_package(ament_cmake REQUIRED)

# 如果有其他依赖（例如 rclcpp, std_msgs 等），在这里添加
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# 声明可执行文件或库（示例，根据实际需求调整）
# add_executable(rosrobot_node src/main.cpp)
# target_link_libraries(rosrobot_node ${rclcpp_LIBRARIES})
# ament_target_dependencies(rosrobot_node rclcpp std_msgs)

# 安装目录（config, launch, meshes, urdf）
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

# 如果是纯配置文件/启动文件包，可以省略上述 add_executable 部分

# 必须调用此宏来生成 package.xml 中的依赖信息
ament_package()