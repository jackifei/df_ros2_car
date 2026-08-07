#ifndef ACKERMANN_HARDWARE_BRIDGE__ACKERMANN_BRIDGE_HARDWARE_HPP_
#define ACKERMANN_HARDWARE_BRIDGE__ACKERMANN_BRIDGE_HARDWARE_HPP_

#include <atomic>
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

// 命令缓冲区（预分配，无动态内存）
struct CommandBuffer
{
  std::vector<double> rear_velocities;
  std::vector<double> front_positions;
};

// 无锁双缓冲包装器
struct CommandDoubleBuffer
{
  CommandBuffer front;                // 前台缓冲区（实时线程写入）
  CommandBuffer back;                 // 后台缓冲区（发布线程读取）
  std::atomic<CommandBuffer*> active{nullptr}; // 指向当前可读的缓冲区
};

// 反馈缓冲区（预分配，无动态内存）
struct FeedbackBuffer
{
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
};

class AckermannBridgeHardware : public hardware_interface::SystemInterface
{
public:
  // 生命周期回调
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 导出接口
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 读写
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 后台线程相关
  void publish_loop();
  void publish_callback();
  void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 关节数据
  std::vector<JointData> joints_;

  // 话题名称
  std::string rear_wheel_cmd_topic_ = "/hardware/rear_wheel_cmd";
  std::string front_steering_cmd_topic_ = "/hardware/front_steering_cmd";
  std::string joint_feedback_topic_ = "/hardware/joint_feedback";

  // ROS 2 节点与通信对象
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr front_steering_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

  // 预分配的消息对象
  std_msgs::msg::Float64MultiArray rear_wheel_msg_;
  std_msgs::msg::Float64MultiArray front_steering_msg_;

  // ★★★ 无锁双缓冲命令缓冲区 ★★★
  CommandDoubleBuffer cmd_double_buffer_;

  // 反馈缓冲区（仍使用锁，但临界区极小）
  FeedbackBuffer fb_buffer_;
  std::mutex fb_mutex_;
  bool fb_received_ = false;

  // 后台线程控制
  std::atomic<bool> running_{false};
  std::thread pub_thread_;
};

}  // namespace ackermann_hardware_bridge

#endif  // ACKERMANN_HARDWARE_BRIDGE__ACKERMANN_BRIDGE_HARDWARE_HPP_