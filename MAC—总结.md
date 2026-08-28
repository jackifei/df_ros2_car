# df_ros2_car 项目总结（roscontrol 分支）

> 基于 ROS2 Jazzy 的阿克曼（Ackermann）结构室内小车完整工程
> 开发者：dengfei ｜ 秦皇岛文视科技有限公司
> 当前分支：`roscontrol` ｜ HEAD：`4076e00`（修改ekf，2026-08-20）
> 总结日期：2026-08-20

---

## 1. 项目概述

本项目是一套**阿克曼式底盘小车**的完整 ROS2 工程，覆盖遥控驾驶、电机驱动、IMU、激光雷达、SLAM 建图、Nav2 自主导航、RViz 可视化等完整链路。

**本分支与 main 分支的核心区别：底盘控制改用 ros2_control 官方框架。**

| 维度 | main 分支 | roscontrol 分支（当前） |
|------|-----------|------------------------|
| 底盘控制 | 自研代码实现阿克曼电子差速 + 转向 PID（joystick_bridge、joystick_steer_pid、straight_line_pid） | **ros2_control 官方 `ackermann_steering_controller`** + 自研硬件接口桥 `ackermann_hardware_bridge` |
| 程序结构 | 业务逻辑全部手写，耦合度高 | 复用成熟控制器模块，结构简化，方便使用现有模块开发 |

**底盘方案（关键特征）**：`同步后驱 + 前轮转向`，类似汽车的"油门 + 方向盘"独立控制：

- 后轮：左右两个独立电机（Modbus RTU 485 通信），同速驱动（无轮速差）；
- 前轮：单片机/舵机（USB 串口）控制转向角（URDF 限位 ±0.7 rad，控制器配置限位 ±0.52 rad ≈ 30°）；
- 转弯完全靠前轮转角：`linear.x` 管油门、`angular.z` 管方向盘，两通道独立。

## 2. 软硬件环境

| 类别 | 内容 |
|------|------|
| 操作系统 | Ubuntu 24.04（目标机） |
| ROS2 | Jazzy Jalisco |
| 构建工具 | colcon（Python 包用 setup.py / C++ 包用 ament_cmake） |
| 底盘 | 阿克曼结构：后轮双电机（485 总线）+ 前轮转向舵机（串口文本协议） |
| IMU | 6 轴（`dm_imu`，/dev/ttyACM0，921600 波特，50Hz） |
| 雷达 | 低成本扫地机雷达（`lidar_pkg`，/dev/ttyACM1，360°，15Hz，frame_id `lidar_Link_sub`） |
| 遥控 | 手柄（joy 驱动 + 自研 joy2twist + twist_mux） |
| 仿真（历史） | Gazebo Harmonic（规划文档保留，未在当前主链路） |
| 其他 | USB 摄像头（代码中已注释，rosrobot_opencv 有示例） |

## 3. 本分支关键变化（vs main）

1. **新增 `ackermann_hardware_bridge` C++ 包**：ros2_control 硬件接口插件（`SystemInterface`），在 ros2_control 框架与外部硬件驱动节点之间建立话题桥接。
2. **URDF 增加 `<ros2_control>` 声明**：硬件插件改为 `ackermann_hardware_bridge::AckermannBridgeHardware`，后轮 velocity 接口、前轮 position 接口。
3. **新增实车 ros2_control 启动链路**：`roscontrol_ackermann.launch.py`（rosrobot_odom）与 `robot_control2.launch.py`（rosrobot_top_control 全量启动），拉起 controller_manager + joint_state_broadcaster + ackermann_steering_controller。
4. **新增 `ackermann_steering.yaml`**：Jazzy 版 `ackermann_steering_controller` 完整参数（底盘机械参数、关节名称、限速限角）。
5. **最新提交（4076e00）修改 EKF**：`ekf_params.yaml` 由约 500 行精简为约 50 行，`ekf_localization.launch.py` 同步重写。
6. **新增 `server_bridge.launch.py`**：rosbridge_websocket（端口 9090）+ rosapi 服务器桥接，用于远程/网页端连接。
7. **`slam_params.yaml` 修改完善**：slam_toolbox 建图参数调优。
8. **移除自研 PID 文件**：`joystick_steer_pid.py`、`straight_line_pid.py` 从代码树删除（被 ackermann_steering_controller 取代）。
9. **顶层 README 更新**：明确"本版本使用 ros control 中的阿克曼控制器进行小车底盘驱动"。

## 4. 目录结构总览

```
df_ros2_car/                          # 工作空间根目录
├── README.md                         # 启动流程说明（ros2_control 版）
├── MAC—总结.md                       # 本文件
└── src/                              # ROS2 工作空间源码
    ├── ackermann_hardware_bridge/    # ★ 新增：ros2_control 硬件接口桥（C++）
    ├── rosrobot_top_control/         # ★ 总控启动包（含 robot_control2 全量启动）
    ├── rosrobot_odom/                # ★ 实车 ros2_control 启动 + 阿克曼配置 + 里程计
    ├── rosrobot_twist_mux/           # ★ 手柄/导航双路速度选择器 + joy2twist
    ├── df_motor_ctr/                 # ★ 电机驱动（485 Modbus + 转向串口）
    ├── IMU-ros2/                     # IMU 驱动（dm_imu）
    ├── rosrobot_lidar_pkg/           # 雷达驱动（第三方开源，C++）
    ├── rosrobot_slam_map/            # SLAM 建图启动 + 参数（slam_toolbox）
    ├── rosrobot_navigation/          # Nav2 导航参数/启动
    ├── rosrobot_description/         # URDF 模型 + ros2_control 硬件声明
    ├── robot_localization_config/    # EKF 多传感器融合配置
    ├── rosrobot_bringup_two/         # 显示/仿真时代 bringup + 关节同步脚本
    ├── learning_tf/                  # turtlesim TF 教学示例
    ├── rosrobot_opencv/              # 摄像头话题发布/订阅示例
    ├── teleop_twist_joy/             # 第三方手柄遥操作包（vendored）
    ├── GAZEBO_SLAM_PLAN.md           # Gazebo+SLAM 仿真规划文档
    ├── 变更记录.md / 控制说明.md      # 开发变更与控制说明
    └── 统计/                         # 早期 joystick_bridge 架构方案（历史文档）
```

## 5. ros2_control 控制链路（本分支核心）

```
手柄 /joy ─→ joy2twist ─→ /cmd_vel_joy ──┐
                                          ├─→ twist_mux ─→ /cmd_vel
Nav2 ─────────────────→ /cmd_vel_nav ────┘        │
                                                  ▼
                        ┌──── ackermann_steering_controller（ros2_control 官方）────┐
                        │   /cmd_vel → 阿克曼运动学解算 → 后轮速度 + 前轮转角指令     │
                        └───────────────┬──────────────────────────────────────────┘
                                        ▼
                        ackermann_hardware_bridge（SystemInterface 硬件插件）
                        write():  /hardware/rear_wheel_cmd      （后轮速度）
                                  /hardware/front_steering_cmd （前轮转角）
                        read():   /hardware/joint_feedback     （关节反馈）
                                        │
                        ┌───────────────┴────────────────┐
                        ▼                                ▼
              df_motor_ctr / motor_ctr         df_motor_ctr / wheel_dir
              （485 Modbus 驱动后轮双电机）      （串口协议驱动前轮舵机）
                        │
                        ▼
        /encoder_speed + /df_dir_rt ─→ 轮式里程计 /odom ─→ EKF（+ /imu/data）
                                                          ─→ /odometry/filtered + /tf
```

控制器输出的里程计 TF 通过 remap 发布：`/ackermann_steering_controller/tf_odometry → /tf`（odom → base_link）。

### 硬件接口桥工作原理

`ackermann_hardware_bridge`（`src/ackermann_hardware_bridge/`）：

- 继承 `hardware_interface::SystemInterface`，实现 `on_init/on_configure/on_activate/on_deactivate/read/write`；
- 在 `on_init()` 中根据关节的 `command_interface` 类型自动分类：`velocity` → 驱动关节（后轮）、`position` → 转向关节（前轮）；
- `write()` 向外部驱动发布后轮速度/前轮转角命令；`read()` 用 `spin_some` 处理 `/hardware/joint_feedback` 反馈写回 StateInterface；
- 话题名通过 URDF `<param>` 可配置（`rear_wheel_cmd_topic` / `front_steering_cmd_topic` / `joint_feedback_topic`）；
- 已通过两轮代码审查并修复 10 个问题（依赖缺失、空指针保护、死代码清理等），详见包内 `PACKAGE_ANALYSIS.md`。

## 6. 功能包详解

| 包 | 类型 | 功能与关键文件 |
|----|------|----------------|
| **ackermann_hardware_bridge** | C++ | ★ 新增。ros2_control 硬件接口插件，话题桥接 ros2_control ↔ 外部驱动（/hardware/rear_wheel_cmd、/hardware/front_steering_cmd、/hardware/joint_feedback） |
| **rosrobot_top_control** | Python | 总控启动。`robot_control.launch.py`（旧链路）、`robot_control2.launch.py`（★ ros2_control 全量启动：controller_manager + ackermann 控制器 + twist_mux + IMU + 电机 + 转向 + 雷达 + RViz）、`server_bridge.launch.py`（rosbridge 9090 + rosapi） |
| **rosrobot_odom** | Python | `roscontrol_ackermann.launch.py`（★ 实车 ros2_control 启动）、`config/ackermann_steering.yaml`（★ 阿克曼控制器参数）；`joystick_bridge_node.py`（旧版手柄桥接，robot_control2 中已注释）、`rosrobot_odom.py` + `wheel_odom_noimu.py`（自行车模型里程计，订阅 /cmd_vel_rt + /df_dir_rt → /odom + TF + /path）、`wheel_odom_IMU.py`（IMU 融合版，未接入） |
| **rosrobot_twist_mux** | Python | `joy2twist.py`：/joy → /cmd_vel_joy（轴1线速度、轴3角速度，带死区与阿克曼约束）；`rosrobot_twist_mux.py`：/cmd_vel_joy 与 /cmd_vel_nav 二选一，手柄 START 切导航、手柄输入夺回、导航超时自动切回 |
| **df_motor_ctr** | Python | `motor_control.py`：Modbus RTU 驱动后轮双电机，订阅 /wheel_control/leftright，发布实测 /cmd_vel_rt；`wheel_dir_pwm.py`：订阅 /wheel_control/dir，弧度→度串口驱动前轮舵机，发布 /df_dir_rt |
| **IMU-ros2** | Python | `dm_imu` 驱动：/dev/ttyACM0 @921600，50Hz 发布 /imu/data（四元数）、/imu/rpy、/imu/pose，带 CRC 串口模块 |
| **rosrobot_lidar_pkg** | C++ | 第三方雷达驱动，/dev/ttyACM1，frame_id `lidar_Link_sub`，发布 /scan |
| **rosrobot_slam_map** | Python | slam_toolbox 在线建图（`sync_slam_toolbox_node`），`slam_params.yaml`：odom→base_link→scan，分辨率 0.02，开启闭环检测 |
| **rosrobot_navigation** | Python/YAML | Nav2 全套：`nav_auto.launch.py`（map_server、amcl、controller_server(MPPI)、smoother、planner、route、behavior、bt_navigator、waypoint_follower、velocity_smoother → /cmd_vel_nav）；`nav_control.launch.py`（地图加载 + 静态 TF map→odom）；`nav2_params.yaml` 含 17 模块中文注释 |
| **rosrobot_description** | URDF | 小车模型 + `<ros2_control>` 硬件声明（AckermannBridgeHardware）；关节：lh_joint/rh_joint（后轮 continuous，velocity 接口）、lq_joint/rq_joint（前轮 revolute，position 接口，±0.7 rad）；`config/ros2_control.yaml` 为 Gazebo 控制器配置（历史） |
| **robot_localization_config** | YAML | EKF 融合：/odom + /imu/data → /odometry/filtered + /tf；15 状态、50Hz、2D 模式（最新提交精简重写） |
| **rosrobot_bringup_two** | Python/CMake | 显示/仿真时代 bringup：`cmd_vel_to_joints_sync.py`、`publish_robot_description.py`、`ackermann_odometry.py` |
| **learning_tf / rosrobot_opencv / teleop_twist_joy** | Python/C++ | TF 教学、摄像头示例、第三方手柄遥操作（vendored） |

## 7. 关键配置说明

### ackermann_steering_controller（实车，`rosrobot_odom/config/ackermann_steering.yaml`）

| 参数 | 值 | 说明 |
|------|-----|------|
| `update_rate` | 50 Hz | controller_manager 刷新率 |
| `wheelbase` | 0.36 m | 轴距（前转向轴 → 后驱动轴） |
| `traction_track_width` | 0.39 m | 后轮轮距 |
| `steering_track_width` | 0.39 m | 前轮轮距 |
| `traction_wheels_radius` | 0.0625 m | 后轮半径 |
| `traction_joints_names` | [lh_joint, rh_joint] | 驱动关节（右、左顺序） |
| `steering_joints_names` | [lq_joint, rq_joint] | 转向关节 |
| `linear_velocity_max` | 0.3 m/s | 最大线速度 |
| `angular_velocity_max` | 0.2 rad/s | 最大角速度 |
| `steering_angle_max` | 0.52 rad | 最大转向角（≈30°） |
| `cmd_vel_timeout` | 0.5 s | 指令超时自动停机 |
| `open_loop` | true | 开环估算里程计 |
| `publish_rate` | 50 Hz | 里程计发布频率 |

### EKF 融合（`robot_localization_config/config/ekf_params.yaml`）

- 频率 50Hz、`two_d_mode: true`、`sensor_timeout: 0.06s`；
- 坐标系：`map_frame=map`、`odom_frame=odom`、`base_link_frame=base_link`、`world_frame=odom`；
- 输入：`odom0=/odom`（轮式里程计）、`imu0=/imu/data`（6 轴 IMU）；
- 输出：`/odometry/filtered` + odom → base_link TF；
- 最新提交精简了过程噪声/初始协方差配置。

## 8. 启动方式

```bash
# 1) 整机启动（ros2_control 版，★ 本分支推荐）
ros2 launch rosrobot_top_control robot_control2.launch.py

# 2) 旧版整机启动（自研差速链路，不含 controller_manager）
ros2 launch rosrobot_top_control robot_control.launch.py

# 3) 单独启动 ros2_control（controller_manager + ackermann 控制器）
ros2 launch rosrobot_odom roscontrol_ackermann.launch.py

# 4) SLAM 建图 + 保存地图
ros2 launch rosrobot_slam_map slam.launch.py
ros2 run nav2_map_server map_saver_cli -f map

# 5) 导航（先加载地图并重定位，再自动导航）
ros2 launch rosrobot_navigation nav_control.launch.py
ros2 run rosrobot_navigation rosrobot_nav
ros2 launch rosrobot_navigation nav_auto.launch.py

# 6) 单独启动 IMU / 雷达
ros2 run dm_imu dm_imu_node
ros2 launch lidar_pkg lidar.launch.py

# 7) 服务器桥接（远程 Web 访问）
ros2 launch rosrobot_top_control server_bridge.launch.py
```

**手柄说明**：启动后按手柄 `START` 开启控制；手柄 5 分钟无输入会自动睡眠，需重启。

## 9. 已知注意事项与待办

- **端口权限**：系统重新插拔 USB 后，需确认串口端口号与实际设备对应（IMU ttyACM0、雷达 ttyACM1、转向串口），并检查 udev 权限。
- **EKF launch 占位符**：`ekf_localization.launch.py` 中仍使用占位包名 `your_package_name`，直接运行会找不到配置路径，需要改为实际包名 `robot_localization`。
- **EKF 参数注释不一致**：`ekf_params.yaml` 中部分注释（如"融合线速度 ✓"）与开关值（false）不一致，调参时以实际值为准。
- **双链路并存**：`robot_control.launch.py`（自研）与 `robot_control2.launch.py`（ros2_control）并存，两套链路不可同时启动，避免重复控制与话题冲突。
- **历史文档**：`统计/方案.md`、`统计/完成.md` 是早期 joystick_bridge 架构的方案与交付文档，与本分支实际结构不一致，仅作参考。
- **配置对比参考**：`rosrobot_odom/config/ackermann_steering.yaml` 中附有官方文档链接（steering_controllers_library userdoc），修改参数时建议对照官方说明。

## 10. 版本信息

| 项目 | 内容 |
|------|------|
| 分支 | roscontrol |
| HEAD | `4076e00` 修改ekf（2026-08-20 18:12:52） |
| 分支主要提交 | 桥接接口 TF 修复 → slam 参数完善 → 服务器桥接 → EKF 精简 |
| 对比 main | main 分支为自研阿克曼/PID 控制（`4b8771c` 增加转向pid） |

> 文档版本：v1.0（roscontrol 分支）
> 生成时间：2026-08-20
