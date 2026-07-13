
#  1: RViz 显示 安装插件
sudo apt install ros-jazzy-rviz-imu-plugin
# 你前面是 jazzy，那就是 ros-jazzy-rviz-imu-plugin

sudo apt install ros-${ROS_DISTRO}-imu-filter-madgwick
# 安装滤波器
🚨 坑一：IMU 消息里没有 orientation（最常见）

很多原始 IMU 驱动只发 linear_acceleration+ angular_velocity，orientation那一坨全是 0 或 NaN——rviz 的 Imu 插件主要靠 orientation 画姿态，没它就画不出（或者画个静止的）。

解决：跑一个姿态滤波节点补 orientation：
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args -p fix_covariance:=true -p publish_tf:=false \
  --remap /imu/data_raw:=/imu raw话题 -r /imu/data:=/imu/data_filtered

  rviz 里订阅 /imu/data_filtered就有姿态了。