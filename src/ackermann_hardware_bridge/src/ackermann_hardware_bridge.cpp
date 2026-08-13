// ackermann_hardware_bridge.cpp
#include "ackermann_hardware_bridge/ackermann_hardware_bridge.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

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

    // 始终分配状态指针（无论是否为驱动/转向）
    jd.state_position = std::make_unique<double>(0.0);
    jd.state_velocity = std::make_unique<double>(0.0);

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
  RCLCPP_INFO(rclcpp::get_logger("AckermannBridgeHardware"), "on_configure");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannBridgeHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    cmd_double_buffer_.front.rear_velocities.reserve(joints_.size());
    cmd_double_buffer_.front.front_positions.reserve(joints_.size());
    cmd_double_buffer_.back.rear_velocities.reserve(joints_.size());
    cmd_double_buffer_.back.front_positions.reserve(joints_.size());
    cmd_double_buffer_.active.store(&cmd_double_buffer_.back);
  // ★★★ 初始化所有关节的状态值为 0.0，防止 NaN ★★★
  for (auto & joint : joints_)
  {
    if (joint.state_position)  *joint.state_position  = 0.0;
    if (joint.state_velocity)  *joint.state_velocity  = 0.0;
    if (joint.command_velocity) *joint.command_velocity = 0.0;
    if (joint.command_position) *joint.command_position = 0.0;
  }

  // 创建节点（注意：节点将在后台线程中使用）
  node_ = std::make_shared<rclcpp::Node>("ackermann_hardware_bridge_node");

  // ★★★ 预分配消息对象，运行时零分配 ★★★
  rear_wheel_msg_ = std_msgs::msg::Float64MultiArray();
  front_steering_msg_ = std_msgs::msg::Float64MultiArray();

  // 创建发布者（QoS深度保持1）
  rear_wheel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    rear_wheel_cmd_topic_, rclcpp::QoS(1));
  front_steering_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    front_steering_cmd_topic_, rclcpp::QoS(1));

  // 创建订阅者（使用 SensorDataQoS 保证低延迟）
  feedback_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_feedback_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AckermannBridgeHardware::feedback_callback, this, std::placeholders::_1));

  // ★★★ 初始化命令缓冲区（预分配容量）★★★
//  cmd_buffer_.rear_velocities.clear();
//  cmd_buffer_.front_positions.clear();
//  cmd_buffer_.rear_velocities.reserve(joints_.size());
//  cmd_buffer_.front_positions.reserve(joints_.size());

  // ★★★ 初始化反馈缓冲区 ★★★
  fb_buffer_.positions.resize(joints_.size(), 0.0);
  fb_buffer_.velocities.resize(joints_.size(), 0.0);
  fb_buffer_.names.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    fb_buffer_.names[i] = joints_[i].name;

  // ★★★ 启动后台发布线程（独立于实时循环）★★★
  running_.store(true);
  pub_thread_ = std::thread(&AckermannBridgeHardware::publish_loop, this);

  RCLCPP_INFO(node_->get_logger(), "Bridge activated, background thread started.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannBridgeHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停止后台线程
  running_.store(false);
  if (pub_thread_.joinable())
    pub_thread_.join();

  // 清理 ROS 资源
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
    if (jd.is_steering)
    {
      command_interfaces.emplace_back(
        jd.name, hardware_interface::HW_IF_POSITION, jd.command_position.get());
    }
  }
  return command_interfaces;
}

// ---------- 实时读写（仅操作共享内存，无阻塞）----------

hardware_interface::return_type AckermannBridgeHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 尝试获取反馈锁（非阻塞方式？此处使用普通锁，但临界区极小）
  std::lock_guard<std::mutex> lock(fb_mutex_);
  if (!fb_received_)
    return hardware_interface::return_type::OK;

  // 将反馈缓冲区内容拷贝到关节状态（预分配，无动态内存）
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    *joints_[i].state_position = fb_buffer_.positions[i];
    *joints_[i].state_velocity = fb_buffer_.velocities[i];
    // ★ 添加日志 ★
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "read: joint[%zu] pos=%.3f vel=%.3f",
        i, *joints_[i].state_position, *joints_[i].state_velocity);
  }
  fb_received_ = false;  // 标记已消费

  return hardware_interface::return_type::OK;

  // 开环模式：直接将命令值作为状态反馈（无需等待外部反馈）
//  for (size_t i = 0; i < joints_.size(); ++i)
//  {
//    if (joints_[i].is_drive && joints_[i].command_velocity)
//      *joints_[i].state_velocity = *joints_[i].command_velocity;
//    if (joints_[i].is_steering && joints_[i].command_position)
//      *joints_[i].state_position = *joints_[i].command_position;
//  }
//  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannBridgeHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 找到当前空闲的缓冲区（不是 active 的那个）
  CommandBuffer* idle = (cmd_double_buffer_.active.load() == &cmd_double_buffer_.front)
                            ? &cmd_double_buffer_.back
                            : &cmd_double_buffer_.front;
  idle->rear_velocities.clear();
  idle->front_positions.clear();

  for (const auto & jd : joints_)
  {
    if (jd.is_drive && jd.command_velocity)
      idle->rear_velocities.push_back(*jd.command_velocity);
    else if (jd.is_steering && jd.command_position)
      idle->front_positions.push_back(*jd.command_position);
  }

  // 原子交换：让 active 指向刚写完的缓冲区
  cmd_double_buffer_.active.store(idle);
//  RCLCPP_INFO(rclcpp::get_logger("AckermannBridgeHardware"),
//            "write: stored %zu rear, %zu front commands",
//            idle->rear_velocities.size(), idle->front_positions.size());
  return hardware_interface::return_type::OK;
}

// ---------- 后台线程：负责所有 DDS 操作 ----------

void AckermannBridgeHardware::publish_loop()
{
  // 创建执行器并添加节点，以便处理订阅回调和定时器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

//    RCLCPP_INFO(node_->get_logger(), "Background thread alive");


  // 创建发布定时器（例如 10ms 周期，可根据实际控制周期调整）

  auto timer = node_->create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() { this->publish_callback(); });

  // 循环执行 spin，直到收到停止信号
  while (rclcpp::ok() && running_.load())
  {
    executor.spin_once(std::chrono::milliseconds(1));
  }
}

void AckermannBridgeHardware::publish_callback()
{

  // 原子读取当前可读缓冲区
  CommandBuffer* buf = cmd_double_buffer_.active.load();

//  RCLCPP_INFO(rclcpp::get_logger("AckermannBridgeHardware"),
//            "publish_callback: reading %zu rear, %zu front commands",
//            buf->rear_velocities.size(), buf->front_positions.size());

  if (buf->rear_velocities.empty() && buf->front_positions.empty())
    return;

  // 填充预分配消息（浅拷贝 vector）
  rear_wheel_msg_.data = buf->rear_velocities;
  front_steering_msg_.data = buf->front_positions;

  if (!buf->rear_velocities.empty())
    rear_wheel_pub_->publish(rear_wheel_msg_);
  if (!buf->front_positions.empty())
    front_steering_pub_->publish(front_steering_msg_);
}

void AckermannBridgeHardware::feedback_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 订阅回调在后台线程中执行，更新反馈缓冲区
  std::lock_guard<std::mutex> lock(fb_mutex_);

  // 将收到的反馈按关节顺序填入 fb_buffer_
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), joints_[i].name);
    if (it != msg->name.end())
    {
      size_t idx = std::distance(msg->name.begin(), it);
      fb_buffer_.positions[i] = (idx < msg->position.size()) ? msg->position[idx] : 0.0;
      fb_buffer_.velocities[i] = (idx < msg->velocity.size()) ? msg->velocity[idx] : 0.0;
    }
    else
    {
      fb_buffer_.positions[i] = 0.0;
      fb_buffer_.velocities[i] = 0.0;
    }
  }
  fb_received_ = true;
}

}  // namespace ackermann_hardware_bridge

PLUGINLIB_EXPORT_CLASS(
  ackermann_hardware_bridge::AckermannBridgeHardware,
  hardware_interface::SystemInterface)