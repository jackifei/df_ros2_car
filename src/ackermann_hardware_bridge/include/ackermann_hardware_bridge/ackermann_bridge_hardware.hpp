#ifndef ACKERMANN_HARDWARE_BRIDGE__ACKERMANN_BRIDGE_HARDWARE_HPP_
#define ACKERMANN_HARDWARE_BRIDGE__ACKERMANN_BRIDGE_HARDWARE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ackermann_hardware_bridge
{

struct JointData
{
  std::string name;
  bool is_steering = false;
  bool is_drive = false;

  std::unique_ptr<double> state_position{new double(0.0)};
  std::unique_ptr<double> state_velocity{new double(0.0)};
  std::unique_ptr<double> command_velocity{nullptr};
  std::unique_ptr<double> command_position{nullptr};
};

class AckermannBridgeHardware : public hardware_interface::SystemInterface
{
public:
  // 生命周期回调（替代旧的 configure / start / stop）
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 导出接口（不变）
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 读写（新增 time 和 period 参数）
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ROS2 节点与通信成员
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr front_steering_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

  // 后台 spin 线程
  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> spinning_{false};

  // 关节数据
  std::vector<JointData> joints_;
  std::vector<std::string> feedback_joint_names_;

  // 反馈缓存与互斥锁
  sensor_msgs::msg::JointState last_feedback_;
  bool feedback_received_ = false;
  std::mutex feedback_mutex_;

  // 话题名称
  std::string rear_wheel_cmd_topic_ = "/hardware/rear_wheel_cmd";
  std::string front_steering_cmd_topic_ = "/hardware/front_steering_cmd";
  std::string joint_feedback_topic_ = "/hardware/joint_feedback";
};

}  // namespace ackermann_hardware_bridge

#endif  // ACKERMANN_HARDWARE_BRIDGE__ACKERMANN_BRIDGE_HARDWARE_HPP_