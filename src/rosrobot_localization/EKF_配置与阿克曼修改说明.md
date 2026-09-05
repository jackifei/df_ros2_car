# EKF 数据融合配置与阿克曼修改说明

本文件汇总 `robot_localization` EKF 融合阿克曼里程计与 IMU 的完整配置、启动方式，
以及需要由用户在阿克曼控制器配置中自行修改的地方。

适用环境：ROS 2 Jazzy

---

## 1. 目标

用 `robot_localization` 的 `ekf_node` 融合：

- 阿克曼控制器里程计：`/ackermann_steering_controller/odometry`
- IMU：`/imu/data`

输出：

- `/odometry/filtered`
- `odom -> base_link` 的 TF

---

## 2. EKF 相关文件

| 文件 | 说明 |
| --- | --- |
| `src/robot_localization/config/ekf_params.yaml` | EKF 参数 |
| `src/robot_localization/launch/ekf_localization.launch.py` | EKF 单独启动文件 |
| `src/robot_localization/setup.py` | 配置包元数据（包名已统一） |
| `src/robot_localization/package.xml` | 配置包元数据 |

配置包名统一为：`robot_localization_config`。

> 注意：真正提供 `ekf_node` 可执行文件的仍是 apt 安装的 `robot_localization` 包；
> 本目录的 `robot_localization_config` 只是存放 YAML 与 launch 的配置包。

---

## 3. EKF 参数关键点

文件：`src/robot_localization/config/ekf_params.yaml`

### 3.1 基本参数

```yaml
frequency: 50.0
sensor_timeout: 0.1
two_d_mode: true
publish_tf: true

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
```

### 3.2 阿克曼里程计输入

```yaml
odom0: /ackermann_steering_controller/odometry
odom0_config:
  - true    # x
  - true    # y
  - false   # z
  - false   # roll
  - false   # pitch
  - true    # yaw
  - true    # vx
  - false   # vy
  - false   # vz
  - false   # vroll
  - false   # vpitch
  - true    # vyaw
  - false   # ax
  - false   # ay
  - false   # az
odom0_differential: false
odom0_relative: false
```

即融合阿克曼里程计的 `x / y / yaw / vx / vyaw`。

### 3.3 IMU 输入

```yaml
imu0: /imu/data
imu0_config:
  - false   # x
  - false   # y
  - false   # z
  - false   # roll
  - false   # pitch
  - true    # yaw
  - false   # vx
  - false   # vy
  - false   # vz
  - false   # vroll
  - false   # vpitch
  - false   # vyaw
  - false   # ax
  - false   # ay
  - false   # az
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true
```

**为什么 IMU 只融合 yaw：**

当前 IMU 驱动 `src/IMU-ros2/dm_imu/node.py` 只发布 `orientation`，
没有发布 `angular_velocity` 和 `linear_acceleration`，且相关协方差为 `-1`。
因此目前只能用 IMU 的姿态 yaw，不能融合 `vyaw / ax / ay`。

如果以后在 IMU 驱动中补上陀螺仪角速度和有效协方差，可以再把 `vyaw` 打开。

### 3.4 9 轴 IMU 可开启的额外参数

你的 IMU 是 9 轴（加速度计 + 陀螺仪 + 磁力计），理论上可以提供：

- `orientation`：roll / pitch / yaw，其中 yaw 由磁力计辅助，基本不随时间漂移。
- `angular_velocity`：陀螺仪角速度。
- `linear_acceleration`：加速度计线加速度。

因此在 EKF 中可以额外开启：

#### 建议开启

```yaml
imu0_config:
  - false   # x
  - false   # y
  - false   # z
  - false   # roll
  - false   # pitch
  - true    # yaw    9 轴磁力计航向，已开启
  - false   # vx
  - false   # vy
  - false   # vz
  - false   # vroll
  - false   # vpitch
  - true    # vyaw   陀螺仪 z 轴角速度
  - false   # ax
  - false   # ay
  - false   # az
imu0_relative: false
imu0_remove_gravitational_acceleration: true
```

#### 可选开启（进阶）

如果驱动正确发布了线加速度，且底盘振动不大，可以再开启：

```yaml
imu0_config:
  # ...
  - true    # ax
  - true    # ay
  - false   # az
```

说明：

- `two_d_mode: true` 时，`roll / pitch / vroll / vpitch` 不参与更新，无需开启。
- 9 轴绝对航向应保持 `imu0_relative: false`。
- 开启 `ax / ay` 前必须保证 `imu0_remove_gravitational_acceleration: true`。
- 如果底盘振动较大，`ax / ay` 容易引入噪声，建议先只开 `vyaw`。

#### 前置条件：IMU 驱动必须补齐数据

当前 `src/IMU-ros2/dm_imu/node.py` 只发布 `orientation`，没有发布
`angular_velocity` 和 `linear_acceleration`，并且把它们的协方差设成了 `-1`。

要开启 `vyaw / ax / ay`，需要先在 IMU 驱动中补充：

- `imu.angular_velocity.x/y/z`（单位：rad/s）
- `imu.linear_acceleration.x/y/z`（单位：m/s²）
- 把 `angular_velocity_covariance` 和 `linear_acceleration_covariance`
  的对角线改成有效正数（例如 `0.01` 或 `0.05`），不能是 `-1`。

### 3.5 协方差

- `process_noise_covariance`：15×15，共 225 个元素。
- `initial_estimate_covariance`：15×15，共 225 个元素，初始状态取很小的 `1e-9`。

> 原来的协方差只有 9×9 = 81 个元素，会导致 robot_localization 解析失败，已修正。

---

## 4. 启动方式

### 4.1 主启动文件

主启动文件：`src/rosrobot_top_control/launch/robot_control2.launch.py`

已按该文件原有格式加入 EKF 启动，位置在阿克曼控制器 spawner 之后：

1. `base_link -> imu_link` 静态 TF
2. `robot_localization` 的 `ekf_node`

```python
# 5.1 base_link -> imu_link 静态 TF
nodes.append(Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_imu_link',
    output='screen',
    arguments=[
        '--x', '0',
        '--y', '0',
        '--z', '0',
        '--roll', '0',
        '--pitch', '0',
        '--yaw', '0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'imu_link',
    ],
))

# 5.2 EKF 节点
config_path_ekf = os.path.join(
    get_package_share_directory('rosrobot_localization'),
    'config',
    'ekf_params.yaml'
)
nodes.append(Node(
    package='rosrobot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[
        config_path_ekf,
        {'use_sim_time': False},
    ],
))
```

> `base_link -> imu_link` 当前为零偏移。若 IMU 实际安装位置或朝向有偏移，
> 修改 `--x/--y/--z/--roll/--pitch/--yaw`。

### 4.2 单独启动 EKF

也可以单独启动 EKF：

```bash
ros2 launch rosrobot_localization ekf_localization.launch.py
```

---

## 5. 需要用户在阿克曼配置中自行修改的地方

文件：`src/rosrobot_odom/config/ackermann_steering.yaml`

> 本次没有直接修改该文件，以下内容由用户自己修改。

### 5.1 关节顺序改为“右关节在前、左关节在后”

```yaml
traction_joints_names: ['rh_joint','lh_joint']
traction_joints_state_names: ['rh_joint','lh_joint']

steering_joints_names: ['rq_joint','lq_joint']
steering_joints_state_names: ['rq_joint','lq_joint']
```

Jazzy 官方要求两关节时顺序为：右关节、左关节。

### 5.2 关闭阿克曼的 TF 发布

```yaml
enable_odom_tf: false
```

原因：EKF 也发布 `odom -> base_link`，两者会冲突。

### 5.3 机械参数核对

| 参数 | 当前值 | URDF 反算参考值 | 说明 |
| --- | --- | --- | --- |
| `wheelbase` | 0.36 | 约 0.306 | 前轮 x=0.167，后轮 x=-0.139 |
| `traction_track_width` | 0.39 | 约 0.385 | 后轮 y=±0.1925 |
| `steering_track_width` | 0.39 | 约 0.28 | 前轮 y=±0.14 |
| `traction_wheels_radius` | 0.0625 | 需实测 | 建议用卡尺量实际轮胎半径 |

### 5.4 open_loop 与 position_feedback

```yaml
open_loop: true
position_feedback: false
```

- 没有真实编码器/舵机反馈时，保持 `open_loop: true`。
- 有真实反馈并发布 `/hardware/joint_feedback` 时，再改 `open_loop: false`，
  并让 `position_feedback` 匹配反馈类型。

### 5.5 无效/过时参数

Jazzy 的 `ackermann_steering_controller` 中不存在的参数：

- `cmd_vel_timeout` → 应改为 `reference_timeout`
- `use_stamped_vel`
- `linear_velocity_max`
- `angular_velocity_max`
- `steering_angle_max`
- `publish_rate`

Jazzy 中阿克曼控制器订阅的是：

```text
/ackermann_steering_controller/reference  (TwistStamped)
```

不是 `/cmd_vel`（Twist）。

### 5.6 重复的 open_loop

配置顶部有注释掉的 `# open_loop: true`，底部又有生效的 `open_loop: true`，
整理时保留一个即可。

---

## 6. 启动与验证命令

```bash
# 1. 阿克曼控制器（发布 /ackermann_steering_controller/odometry）
ros2 launch rosrobot_odom roscontrol_ackermann.launch.py

# 2. 主启动（已包含 EKF）
ros2 launch rosrobot_top_control robot_control2.launch.py
```

验证：

```bash
ros2 topic hz /ackermann_steering_controller/odometry
ros2 topic echo /ackermann_steering_controller/odometry --once
ros2 topic echo /odometry/filtered --once

ros2 run tf2_tools view_frames
```

TF 树应满足：

- `odom -> base_link` 只有一个来源（EKF）。
- 存在 `base_link -> imu_link`。

---

## 7. 重要提醒

1. `joystick_bridge_node` 目前实际不发布 `/odom`（里程计发布代码被注释）。
   EKF 的里程计来源应为阿克曼控制器。
2. 主启动文件 `robot_control2.launch.py` 中已有
   `('/ackermann_steering_controller/tf_odometry', '/tf')` 重映射，
   因此务必把阿克曼 `enable_odom_tf` 改为 `false`，避免 TF 冲突。
3. IMU 需要单独启动，并发布 `/imu/data`，frame_id 为 `imu_link`。
