# nav_auto.launch.py — ROS2 Jazzy Nav2 自动导航启动文件说明

## 概述

本文件启动 ROS2 Jazzy Navigation2 (Nav2) 完整导航栈，包含地图加载、定位、路径规划、运动控制等全部功能模块。

## 支持两种运行模式

| 模式 | 参数 | 说明 |
|------|------|------|
| **独立节点** (默认) | `use_composition:=False` | 每个功能作为独立进程运行，稳定、便于调试 |
| **组合节点** | `use_composition:=True` | 多个组件加载到同一进程，零拷贝通信，性能更好 |

```bash
# 默认独立节点模式
ros2 launch rosrobot_navigation nav_auto.launch.py

# 组合节点模式
ros2 launch rosrobot_navigation nav_auto.launch.py use_composition:=True
```

---

## cmd_vel 话题流向

速度指令话题已重映射为 `/cmd_vel_nav`，完整流向：

```
controller_server ──→ cmd_vel_nav ──┐
                                     ├──→ velocity_smoother ──→ cmd_vel_nav ──→ 底盘驱动节点
behavior_server  ──→ cmd_vel_nav ───┘     (平滑滤波, 重新发布)
```

> **关键设计**: `velocity_smoother` 采用"中间人滤波器"模式 — 订阅 `cmd_vel_nav` 接收原始指令，平滑后重新发布回同一话题。ROS2 允许多个发布者写同一话题，此设计安全可靠。

### 涉及修改的节点

| 节点 | remapping | 原因 |
|------|-----------|------|
| `controller_server` | `cmd_vel` → `cmd_vel_nav` | 控制指令的输出话题 |
| `behavior_server` | `cmd_vel` → `cmd_vel_nav` | 恢复行为的输出话题 |
| `velocity_smoother` | `cmd_vel` → `cmd_vel_nav` | 订阅 + 重新发布（滤波器） |

**底盘驱动节点只需订阅 `/cmd_vel_nav` 即可。**

---

## 各节点功能一览

| 节点名称 | ROS2 包 | 功能描述 |
|----------|---------|----------|
| `map_server` | `nav2_map_server` | 加载静态地图文件 (yaml+pgm)，发布 `/map` 话题 (OccupancyGrid) |
| `amcl` | `nav2_amcl` | 自适应蒙特卡洛定位，基于粒子滤波融合激光/里程计/地图信息，发布 map→odom TF |
| `controller_server` | `nav2_controller` | MPPI 控制器，计算并发布最优速度指令到 `/cmd_vel_nav` |
| `smoother_server` | `nav2_smoother` | 路径平滑器，对全局规划路径进行平滑处理，减少折线 |
| `planner_server` | `nav2_planner` | 全局路径规划器，使用 Dijkstra/A* 在全局代价地图上规划最优路径 |
| `route_server` | `nav2_route` | 路线服务器，在预定义路线图上进行长距离导航（仓库等结构化环境） |
| `behavior_server` | `nav2_behaviors` | 恢复行为服务器：spin(旋转)、backup(后退)、drive_on_heading、wait、assisted_teleop |
| `bt_navigator` | `nav2_bt_navigator` | 行为树引擎，编排整个导航流程（规划→跟踪→检查→失败恢复→重规划） |
| `waypoint_follower` | `nav2_waypoint_follower` | 航点跟随器，按顺序导航到多个航点 |
| `velocity_smoother` | `nav2_velocity_smoother` | 速度平滑器，对控制指令进行低通滤波，限制加速度，提升运动平稳性 |
| `lifecycle_manager` | `nav2_lifecycle_manager` | 生命周期管理器，统一管理所有节点的状态转换 (unconfigured→active) |

### 已禁用的节点

| 节点 | 原因 |
|------|------|
| `collision_monitor` | 碰撞监视器，当前未启用。如需启用，取消注释该节点并加入 `lifecycle_nodes` 列表 |
| `docking_server` | 自动回充对接，当前未启用。需配合充电桩视觉/红外检测使用 |

---

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `""` | 所有节点的顶级命名空间 |
| `use_sim_time` | `false` | 是否使用仿真时间 (Gazebo)，真实机器人应为 false |
| `params_file` | `config/nav2_params.yaml` | Nav2 综合参数文件完整路径 |
| `autostart` | `true` | 是否自动激活 lifecycle 节点 |
| `use_composition` | `False` | 是否使用组合节点模式 |
| `container_name` | `nav2_container` | 组合节点容器名称 |
| `use_respawn` | `False` | 节点崩溃后是否自动重启 |
| `log_level` | `info` | ROS2 日志级别 (debug/info/warn/error/fatal) |
| `map_yaml` | `maps/map_edited.yaml` | 地图 YAML 文件完整路径 |

### 使用示例

```bash
# 使用自定义地图启动
ros2 launch rosrobot_navigation nav_auto.launch.py map_yaml:=/home/user/my_maps/floor1.yaml

# 使用仿真时间 + 调试日志
ros2 launch rosrobot_navigation nav_auto.launch.py use_sim_time:=true log_level:=debug

# 配置命名空间
ros2 launch rosrobot_navigation nav_auto.launch.py namespace:=robot1
```

---

## 已修复的 Bug 记录

| # | 问题 | 严重程度 | 修复内容 |
|---|------|----------|----------|
| 1 | `controller_server` 独立节点未显式指定 `name` 参数，与其他节点不一致 | 低 | 添加 `name='controller_server'` |
| 2 | 组合模式下 `behavior_server` 的 plugin 名称错误：`behavior_server::BehaviorServer` | **高** | 修正为 `nav2_behaviors::BehaviorServer`，否则组合模式无法加载 |
| 3 | 组合模式缺少 `map_server` 和 `amcl` 组件（这两个包不支持 composition） | 中 | 添加注释说明，提示需额外启动 |
| 4 | 文件 docstring 写为 `nav_with_map.launch.py`，与实际文件名不符 | 低 | 修正为 `nav_auto.launch.py` |
| 5 | `lifecycle_manager` 参数格式不统一（独立模式用两个字典，组合模式用一个） | 低 | 统一为单个参数字典格式 |
