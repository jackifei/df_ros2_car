# Gazebo + LiDAR SLAM 建图 — 完整架构规划

> **控制方案：同步后驱 + 前轮转向（汽车式独立控制）**
> - 后轮：同速驱动（油门），左右轮始终相同转速
> - 前轮：纯转向（方向盘），±0.7 rad 限位
> - 两通道独立：`linear.x` 管油门，`angular.z` 管方向盘，互不干涉
>
> 基于 `rosrobot_description` + `rosrobot_bringup_two` 扩展。ROS2 Jazzy 环境。

---

## 1. 最终目录结构

```
vscode_create_/
├── rosrobot_description/              ← 现有：URDF + 网格模型
│   ├── urdf/
│   │   └── rosrobot.urdf              ← 【需改】添加 <gazebo> + <ros2_control> 标签
│   ├── meshes/
│   ├── config/
│   │   ├── joint_names_rosrobottt.yaml ← 现有
│   │   └── ros2_control.yaml           ← 【新增】同步后驱控制器配置 ★ 与差速方案不同
│   ├── launch/
│   ├── CMakeLists.txt
│   └── package.xml                     ← 【需改】加 gazebo / ros2_control 依赖
│
├── rosrobot_bringup_two/              ← 现有：同步后驱显示 + 控制（RViz）
│   ├── launch/
│   │   ├── display_robot.launch.py     ← 现有
│   │   ├── gazebo_bringup.launch.py    ← 【新增】启动 Gazebo 物理仿真
│   │   └── slam_bringup.launch.py      ← 【新增】SLAM 建图 + RViz 可视化
│   ├── config/
│   │   ├── rosrobot.rviz               ← 现有
│   │   └── slam_mapping.rviz           ← 【新增】SLAM 可视化 RViz 布局
│   ├── scripts/
│   │   ├── cmd_vel_to_joints_sync.py    ← 现有（RViz 显示模式用，Gazebo 中不需要）
│   │   ├── publish_robot_description.py ← 现有
│   │   ├── cmd_vel_gazebo_bridge.py    ← 【新增】cmd_vel → 多控制器命令分发
│   │   └── ackermann_odometry.py       ← 【新增】自行车模型里程计 ★ 同步后驱必需
│   ├── CMakeLists.txt                  ← 【需改】加新脚本安装
│   └── package.xml                     ← 【需改】加新依赖
│
├── rosrobot_gazebo/                   ← 【新建】Gazebo 仿真专用包
│   ├── launch/
│   │   └── spawn_robot.launch.py       ← 启动 Gazebo + 生成小车 + 加载控制器
│   ├── worlds/
│   │   └── room.world                   ← 仿真环境（含围墙供雷达探测）
│   ├── CMakeLists.txt
│   └── package.xml
│
└── GAZEBO_SLAM_PLAN.md                ← 本文件
```

---

## 2. 各模块职责 & 需新增的节点/配置

### 2.1 `rosrobot_description/` — URDF 加装 Gazebo 传感器 & ros2_control

| 改动项 | 说明 |
|--------|------|
| `urdf/rosrobot.urdf` | 添加 `<gazebo>` LiDAR 传感器插件；添加 `<ros2_control>` 硬件接口标签 |
| `config/ros2_control.yaml` | ★ **与差速方案完全不同**：3 个控制器 — `joint_state_broadcaster` + `rear_wheel_controller` (velocity) + `steering_controller` (position) |
| `package.xml` | 添加 `<exec_depend>gazebo_ros2_control</exec_depend>` `<exec_depend>ros2_controllers</exec_depend>` `<exec_depend>gazebo_ros_pkgs</exec_depend>` |

#### URDF 添加的标签（示意）

```xml
<!-- ===== ros2_control 硬件接口（同步后驱方案）===== -->
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <!-- 后轮：velocity 指令接口 -->
  <joint name="lh_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <joint name="rh_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <!-- 前轮转向：position 指令接口 -->
  <joint name="lq_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="rq_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>

<!-- ===== LiDAR 传感器 ===== -->
<gazebo reference="lidar_Link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>15</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>12.0</max>
        <resolution>0.02</resolution>
      </range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_Link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### `ros2_control.yaml`（★ 同步后驱专用）

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # 1) 关节状态广播（只读，反馈 Gazebo 物理状态）
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # 2) 后轮同步速度控制器
    rear_wheel_controller:
      type: velocity_controllers/JointGroupVelocityController

    # 3) 前轮位置控制器
    steering_controller:
      type: position_controllers/JointGroupPositionController

# ----- 后轮速度控制器参数 -----
rear_wheel_controller:
  ros__parameters:
    joints:
      - lh_joint
      - rh_joint
    command_interfaces:
      - velocity
    state_interfaces:
      - position
      - velocity

# ----- 前轮位置控制器参数 -----
steering_controller:
  ros__parameters:
    joints:
      - lq_joint
      - rq_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

> **与差速方案的关键区别：** 差速方案用 1 个 `diff_drive_controller` 管全部；同步后驱用 3 个独立控制器，各自管理自己的关节。`rear_wheel_controller` 接收 `Float64MultiArray [v, v]`（两轮同值），`steering_controller` 接收 `[angle, angle]`。

### 2.2 `rosrobot_gazebo/` — 物理仿真包（新建）

| 文件 | 类型 | 职责 |
|------|------|------|
| `spawn_robot.launch.py` | launch | 启动 Gazebo → 加载 room.world → 生成小车 → 加载 ros2_control 控制器 |
| `worlds/room.world` | SDF | 仿真环境：封闭房间，墙壁可被激光雷达探测 |

### 2.3 `rosrobot_bringup_two/` — 综合启动（扩展现有包）

| 文件 | 类型 | 职责 |
|------|------|------|
| `scripts/cmd_vel_gazebo_bridge.py` | 【新增】 | `/cmd_vel` Twist → 分发到 `/rear_wheel_controller/commands` (velocity) + `/steering_controller/commands` (position) |
| `scripts/ackermann_odometry.py` | 【新增】 | 读取 `/joint_states`，自行车模型解算 → `/odom` (Odometry) + `odom → base_link` TF |
| `gazebo_bringup.launch.py` | 【新增】 | 组合：Gazebo → spawn 小车 → 3 个控制器 → bridge → odometry → robot_state_publisher → RViz |
| `slam_bringup.launch.py` | 【新增】 | `gazebo_bringup` 全部 + `slam_toolbox` 建图 → RViz（Map + LaserScan） |
| `config/slam_mapping.rviz` | 【新增】 | RViz 布局：Map + LaserScan + TF + RobotModel + 2D Pose Estimate |

### 2.4 依赖的 ROS2 标准节点 / 控制器（无需自己写代码）

| 节点/控制器 | 来源包 | 订阅 | 发布 |
|------------|--------|------|------|
| `joint_state_broadcaster` | `ros2_controllers` | Gazebo 关节状态 | `/joint_states` |
| `rear_wheel_controller` | `ros2_controllers` | `/rear_wheel_controller/commands` | 后轮速度 → Gazebo 物理 |
| `steering_controller` | `ros2_controllers` | `/steering_controller/commands` | 前轮转向角 → Gazebo 物理 |
| `robot_state_publisher` | 已有 | `/joint_states` + URDF | `/tf`（不含 odom→base_link） |
| `gazebo_ros_ray_sensor` | `gazebo_ros_pkgs` | Gazebo 射线 | `/scan` (LaserScan) |
| `slam_toolbox` | `slam_toolbox` | `/scan` + `/tf` | `/map` + `map→odom` TF |

### 2.5 需自己写的节点（★ 同步后驱特有）

| 节点 | 包 | 订阅 | 发布 | 说明 |
|------|----|------|------|------|
| `cmd_vel_gazebo_bridge` | `rosrobot_bringup_two` | `/cmd_vel` (Twist) | `→ /rear_wheel_controller/commands` (Float64MultiArray)<br>`→ /steering_controller/commands` (Float64MultiArray) | ★ 替代 `diff_drive_controller` 的内置 `cmd_vel` 接口 |
| `ackermann_odometry` | `rosrobot_bringup_two` | `/joint_states` (JointState) | `/odom` (Odometry)<br>`→ odom→base_link` TF | ★ `diff_drive_controller` 自带里程计；同步后驱需手写 |

---

## 3. 数据流（完整 SLAM 建图链路）

```
                  键盘 / 手柄
                      │
                      │  /cmd_vel (geometry_msgs/Twist)
                      ▼
           ┌──────────────────────────┐
           │  cmd_vel_gazebo_bridge   │  ← 【新增】Twist → 控制器命令分发
           └─────┬──────────┬─────────┘
                 │          │
     Float64MultiArray    Float64MultiArray
     [v, v] 后轮同速      [angle, angle] 前轮同角
                 │          │
                 ▼          ▼
   ┌──────────────────┐  ┌──────────────────┐
   │ rear_wheel_ctrl  │  │ steering_ctrl    │
   │ velocity_control │  │ position_control │
   │ joints:[lh,rh]   │  │ joints:[lq,rq]   │
   └────────┬─────────┘  └────────┬─────────┘
            │ 后轮速度指令         │ 前轮位置指令
            └──────────┬──────────┘
                       ▼
           ┌─────────────────────┐
           │   Gazebo 物理引擎    │
           └──┬────────┬─────────┘
              │        │
     ┌────────┘        └─────────┐
     ▼                           ▼
┌──────────────┐          ┌────────────────┐
│joint_state   │          │  LiDAR 射线     │
│broadcaster   │          │  传感器插件      │
└──────┬───────┘          └───────┬────────┘
       │ /joint_states            │ /scan
       ▼                          │
┌──────────────┐                  │
│robot_state   │                  │
│publisher     │                  │
└──────┬───────┘                  │
       │ /tf (joint→links)       │
       │                          │
┌──────┴───────┐                  │
│ackermann     │                  │
│odometry      │  ← 【新增】       │
└──────┬───────┘                  │
       │ /odom                    │
       │ odom→base_link TF        │
       │                          │
       └──────────┬───────────────┘
                  │
                  ▼
         ┌───────────────┐
         │  slam_toolbox  │ ← online_async 在线建图
         └───────┬───────┘
                 │ /map (OccupancyGrid)
                 │ map → odom TF
                 │
                 ▼
         ┌───────────────┐
         │     RViz2      │
         │  · RobotModel  │ ← 小车 3D 模型
         │  · Map         │ ← 二维占据栅格地图
         │  · LaserScan   │ ← 雷达点云可视化
         │  · TF          │ ← 完整坐标树
         └────────────────┘
```

---

## 4. TF 树（建图模式）

```
               ┌──────────┐
               │   map    │ ← slam_toolbox 发布（地图原点）
               └────┬─────┘
                    │ map → odom
                    ▼
               ┌──────────┐
               │   odom   │ ← ackermann_odometry 发布（里程原点）★ 差速方案此处是 diff_drive_controller
               └────┬─────┘
                    │ odom → base_link
                    ▼
               ┌──────────┐
               │ base_link│ ← 车身
               └─┬──┬──┬──┘
          ┌──────┘  │  └──────┐
          ▼         ▼         ▼
   ┌──────────┐ ┌──────────┐ ┌──────────┐
   │lidar_Link│ │ lh_Link  │ │ rh_Link  │
   │  (fixed) │ │(continuous)│(continuous)│
   └──────────┘ └──────────┘ └──────────┘
          │
    ┌─────┴─────┐
    ▼           ▼
┌────────┐ ┌────────┐
│lq_Link │ │rq_Link │
│(revol) │ │(revol) │
└────────┘ └────────┘
```

---

## 5. 里程计模型（自行车运动学）

```
    前轮转向角 δ (lq_joint / rq_joint)
         │
    L ←──┘──────→ 转弯半径 R = L / tan(δ)
    │             角速度 ω = v / R = v * tan(δ) / L
    │
   后轮转速 → 线速度 v = ω_wheel * r
   (lh_joint + rh_joint)

位置更新 (Δt 内):
  Δx = v * cos(θ) * Δt
  Δy = v * sin(θ) * Δt
  Δθ = v * tan(δ) / L * Δt
```

参数（从 URDF 关节原点推算）：
- 轴距 L ≈ 0.306 m（前轴 `lq_joint` x=0.167 到后轴 `lh_joint` x=-0.139）
- 车轮半径 r = 0.05 m

---

## 6. `/cmd_vel` → 控制器命令映射

`cmd_vel_gazebo_bridge.py` 的转换逻辑：

```
输入: /cmd_vel (Twist)
  linear.x  = v (m/s)     ← 油门
  angular.z = ω (rad/s)   ← 方向盘

输出:
  → /rear_wheel_controller/commands
    Float64MultiArray [ω_wheel, ω_wheel]
    ω_wheel = v / wheel_radius       (两元素同值)

  → /steering_controller/commands
    Float64MultiArray [δ, δ]
    δ = clamp(ω * steering_scale, -max_steer, +max_steer)   (两元素同值)
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `wheel_radius` | 0.05 m | 后轮半径 |
| `max_steering_angle` | 0.7 rad | 前轮转向限位 |
| `steering_scale` | 1.0 | angular.z → 转向角比例 |
| `publish_rate` | 50 Hz | 发布频率 |

---

## 7. 实施顺序

| 阶段 | 任务 | 关键文件 | 验证方式 |
|------|------|----------|----------|
| **①** | URDF 添加 `<ros2_control>` 硬件接口 + `<gazebo>` LiDAR 传感器 | `rosrobot.urdf` | `check_urdf` 通过 |
| **②** | 创建 `ros2_control.yaml`（3 控制器） | `ros2_control.yaml` | 参数无语法错误 |
| **③** | 新建 `rosrobot_gazebo` 包 + `spawn_robot.launch.py` + `room.world` | 整个包 | Gazebo 启动 → 小车可见 |
| **④** | 编写 `cmd_vel_gazebo_bridge.py` | 一个脚本 | `/cmd_vel` → 控制器命令可观测 |
| **⑤** | 编写 `ackermann_odometry.py` | 一个脚本 | `/odom` 话题有数据，`odom→base_link` TF 存在 |
| **⑥** | 创建 `gazebo_bringup.launch.py` | launch 文件 | 键盘控制 → 小车在 Gazebo 中行驶转弯 |
| **⑦** | 创建 `slam_bringup.launch.py` + `slam_mapping.rviz` | launch + rviz | RViz 中 Map 随小车移动逐步构建 |
| **⑧** | 调优雷达参数 + SLAM 参数 | `rosrobot.urdf` + slam 参数 | 建图质量满意 |

---

## 8. 两种运行模式对比

| | 显示模式（已有） | Gazebo 仿真模式（待建） |
|------|-----------|----------------|
| Launch 文件 | `display_robot.launch.py` | `gazebo_bringup.launch.py` |
| `/cmd_vel` → 关节 | `cmd_vel_to_joints_sync.py` 直接发布 `/joint_states` | `cmd_vel_gazebo_bridge.py` → 控制器 → Gazebo → `joint_state_broadcaster` → `/joint_states` |
| 里程计 | 无（小车原地） | `ackermann_odometry.py` → `/odom` |
| 时间源 | 系统时钟 | `use_sim_time:=true` (Gazebo 时钟) |

---

## 9. 关键依赖汇总

```xml
<!-- rosrobot_description/package.xml 需添加 -->
<exec_depend>gazebo_ros2_control</exec_depend>
<exec_depend>ros2_controllers</exec_depend>
<exec_depend>gazebo_ros_pkgs</exec_depend>

<!-- rosrobot_bringup_two/package.xml 需添加 -->
<exec_depend>slam_toolbox</exec_depend>
<exec_depend>nav_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>ros2_control</exec_depend>
<exec_depend>ros2_controllers</exec_depend>

<!-- rosrobot_gazebo/package.xml -->
<exec_depend>rosrobot_description</exec_depend>
<exec_depend>gazebo_ros</exec_depend>
<exec_depend>ros2_control</exec_depend>
<exec_depend>ros2_controllers</exec_depend>
```

---

## 10. 注意事项

1. **`cmd_vel_to_joints_sync.py` 在 Gazebo 中不需要** — 仿真中由 `joint_state_broadcaster` 反馈关节状态。`cmd_vel_gazebo_bridge.py` 负责把 `/cmd_vel` 分发给 2 个控制器。
2. **里程计是同步后驱方案的手写部分** — `diff_drive_controller` 自带里程计，同步后驱需要 `ackermann_odometry.py` 用自行车模型手动解算，这是 SLAM 必需的。
3. **后轮 `velocity_controllers/JointGroupVelocityController`** — 接收 `Float64MultiArray`，数组长度 = joints 数量，每元素对应一个 joint。两个后轮始终发相同值。
4. **前轮 `position_controllers/JointGroupPositionController`** — 接收目标角度值，Gazebo 内部 PID 驱动关节到目标位置。两个前轮始终发相同角度。
5. **`use_sim_time`** — 所有仿真相关节点必须设置 `use_sim_time:=true`，包括 `ackermann_odometry`。
6. **slam_toolbox `online_async` 模式** — 推荐用于实时建图，延迟低，适合遥操作建图场景。
7. **雷达 `update_rate`** — 建议 10~20 Hz，过高会加重 SLAM 计算负担。
8. **轴距参数** — 需从 URDF 关节原点精确计算：`L = |front_axle_x - rear_axle_x|`，目前推算约 0.306 m。
