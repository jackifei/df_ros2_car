#include "ackermann_hardware_bridge/ackermann_hardware_bridge.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ackermann_hardware_bridge
{

// ---------- 生命周期回调 ----------

hardware_interface::CallbackReturn AckermannBridgeHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 提取参数覆盖
  for (const auto & param : info.hardware_parameters)
  {
    if (param.first == "rear_wheel_cmd_topic")
      rear_wheel_cmd_topic_ = param.second;
    else if (param.first == "front_steering_cmd_topic")
      front_steering_cmd_topic_ = param.second;
    else if (param.first == "joint_feedback_topic")
      joint_feedback_topic_ = param.second;
  }

  // 构建关节数据结构
  joints_.clear();

  for (const auto & joint_info : info.joints)
  {
    JointData jd;
    jd.name = joint_info.name;

    for (const auto & cmd_if : joint_info.command_interfaces)
    {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION)
        jd.is_steering = true;
      else if (cmd_if.name == hardware_interface::HW_IF_VELOCITY)
        jd.is_drive = true;
    }

    if (jd.is_drive)
      jd.command_velocity = std::make_unique<double>(0.0);
    if (jd.is_steering)
      jd.command_position = std::make_unique<double>(0.0);

    joints_.push_back(std::move(jd));
  }

  RCLCPP_INFO(rclcpp::get_logger("AckermannBridgeHardware"),
              "on_init: configured with %zu joints", joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannBridgeHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 可选的配置步骤，这里直接成功
  RCLCPP_INFO(rclcpp::get_logger("AckermannBridgeHardware"), "on_configure");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannBridgeHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 创建节点、发布者、订阅者
  node_ = std::make_shared<rclcpp::Node>("ackermann_hardware_bridge_node");

  // ★★★ 将发布者的 QoS 深度改为 1，减少内部缓冲，避免 publish 阻塞 ★★★
  rear_wheel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    rear_wheel_cmd_topic_, rclcpp::QoS(1));
  front_steering_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    front_steering_cmd_topic_, rclcpp::QoS(1));

  feedback_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_feedback_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AckermannBridgeHardware::feedback_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Bridge activated, publishing to '%s' and '%s'",
              rear_wheel_cmd_topic_.c_str(), front_steering_cmd_topic_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannBridgeHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rear_wheel_pub_.reset();
  front_steering_pub_.reset();
  feedback_sub_.reset();
  node_.reset();

  RCLCPP_INFO(rclcpp::get_logger("AckermannBridgeHardware"), "Bridge deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------- 接口导出 ----------

std::vector<hardware_interface::StateInterface>
AckermannBridgeHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & jd : joints_)
  {
    state_interfaces.emplace_back(
      jd.name, hardware_interface::HW_IF_POSITION, jd.state_position.get());
    state_interfaces.emplace_back(
      jd.name, hardware_interface::HW_IF_VELOCITY, jd.state_velocity.get());
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AckermannBridgeHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & jd : joints_)
  {
    if (jd.is_drive)
    {
      command_interfaces.emplace_back(
        jd.name, hardware_interface::HW_IF_VELOCITY, jd.command_velocity.get());
    }
    else if (jd.is_steering)
    {
      command_interfaces.emplace_back(
        jd.name, hardware_interface::HW_IF_POSITION, jd.command_position.get());
    }
  }
  return command_interfaces;
}

// ---------- 读写 ----------

hardware_interface::return_type AckermannBridgeHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!node_)
    return hardware_interface::return_type::OK;

  // 在加锁前先处理所有待处理的 ROS 2 回调（包括 feedback_callback）
  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!feedback_received_)
    return hardware_interface::return_type::OK;

  const auto & fb_names = last_feedback_.name;
  const auto & fb_pos   = last_feedback_.position;
  const auto & fb_vel   = last_feedback_.velocity;

  for (size_t i = 0; i < joints_.size(); ++i)
  {
    auto it = std::find(fb_names.begin(), fb_names.end(), joints_[i].name);
    if (it != fb_names.end())
    {
      size_t idx = std::distance(fb_names.begin(), it);
      if (idx < fb_pos.size())
        *joints_[i].state_position = fb_pos[idx];
      if (idx < fb_vel.size())
        *joints_[i].state_velocity = fb_vel[idx];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannBridgeHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!node_ || !rear_wheel_pub_ || !front_steering_pub_)
    return hardware_interface::return_type::OK;

  std::vector<double> rear_velocities;
  std::vector<double> front_positions;

  for (const auto & jd : joints_)
  {
    if (jd.is_drive && jd.command_velocity)
      rear_velocities.push_back(*jd.command_velocity);
    else if (jd.is_steering && jd.command_position)
      front_positions.push_back(*jd.command_position);
  }

  if (!rear_velocities.empty())
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = rear_velocities;
    rear_wheel_pub_->publish(msg);
  }

  if (!front_positions.empty())
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = front_positions;
    front_steering_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

void AckermannBridgeHardware::feedback_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  last_feedback_ = *msg;
  feedback_received_ = true;
}

}  // namespace ackermann_hardware_bridge

PLUGINLIB_EXPORT_CLASS(
  ackermann_hardware_bridge::AckermannBridgeHardware,
  hardware_interface::SystemInterface)