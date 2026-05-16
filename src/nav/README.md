# nav — 纯定位 + Nav2 自主导航

Cartographer 纯定位与 Nav2 路径规划的组合包，适用于差分驱动底盘（非完整约束），无轮式里程计。

## 依赖

- `carto`（Lua 配置文件）
- `cartographer_ros`
- Nav2 全家桶：`nav2_planner`、`nav2_controller`、`nav2_mppi_controller`、`nav2_navfn_planner`、`nav2_behaviors`、`nav2_bt_navigator`、`nav2_smoother`、`nav2_velocity_smoother`、`nav2_lifecycle_manager`、`nav2_costmap_2d`、`nav2_map_server`
- `dm_imu`、`ldlidar_stl_ros2`

## 安装

```bash
cd ~/roscar
colcon build --packages-up-to nav
source install/setup.bash
```

## 启动

### 1. 纯定位

仅运行 Cartographer 纯定位，加载已有地图，不进行路径规划：

```bash
ros2 launch nav localization.launch.py
```

指定地图文件：

```bash
ros2 launch nav localization.launch.py load_state_filename:=/path/to/map.pbstream
```

### 2. 路径规划

仅运行 Nav2 路径规划栈（需要 Cartographer 已在发布 `/map`）：

```bash
ros2 launch nav navigation.launch.py
```

### 3. 完整自主导航

纯定位 + Nav2 路径规划一键启动：

```bash
ros2 launch nav bringup.launch.py
```

## 数据流

```
                         ┌──────────────────────────┐
                         │  Cartographer             │
IMU ────────► /imu/data ─┤  (pure localization)     │──► TF: map → odom
LiDAR ──────► /scan     ─┤                           │──► /map (occupancy grid)
                         └──────────────────────────┘
                                     │
                                     ▼
                         ┌──────────────────────────┐
                         │  Nav2                     │
/map ────────────────────┤  global_costmap           │
                         │  planner_server (NavFn A*) │
                         │  smoother_server           │
                         │  controller_server (MPPI)  │──► cmd_vel_nav
                         │  behavior_server           │
                         │  bt_navigator              │
                         └──────────────────────────┘
                                     │
                                     ▼
                         ┌──────────────────────────┐
                         │  velocity_smoother        │──► cmd_vel
                         └──────────────────────────┘
                                     │
                                     ▼
                         ┌──────────────────────────┐
                         │  stm32_bridge             │──► STM32 (UART)
                         └──────────────────────────┘
```

## cmd_vel 管道

```
controller_server → cmd_vel_nav → velocity_smoother → cmd_vel → stm32_bridge
behavior_server   → cmd_vel_nav ↗
```

- **controller_server** 和 **behavior_server** 发布到 `cmd_vel_nav`
- **velocity_smoother** 订阅 `cmd_vel_nav`，平滑后输出到 `cmd_vel`
- **stm32_bridge** 订阅 `cmd_vel`，转换为 STM32 11 字节帧

## TF 树

```
map → odom → base_link → base_laser  (Z=0.18m)
                       → imu_link    (identity)
```

| 变换 | 发布者 | 类型 |
|------|--------|------|
| `map → odom` | Cartographer | 动态（位姿图优化） |
| `odom → base_link` | Cartographer | 动态（局部 SLAM） |
| `base_link → base_laser` | nav 包 + ld06 驱动 | 静态（Z=0.18m） |
| `base_link → imu_link` | nav 包 | 静态（identity，满足 Cartographer IMU 同位检查） |

> Cartographer 要求 IMU 与 `tracking_frame` 同位（`translation.norm() < 1e-5`）。`base_link → imu_link` 设为 identity，IMU 发布帧设为 `imu_link`，满足此约束。

## 配置说明 (`config/nav2_params.yaml`)

### 规划器 — NavFn A\*

| 参数 | 值 | 说明 |
|------|-----|------|
| `planner_plugins` | `["GridBased"]` | 基于网格的规划器 |
| `plugin` | `nav2_navfn_planner::NavfnPlanner` | 使用 NavFn |
| `tolerance` | `0.5` | 目标点容差 (m) |
| `use_astar` | `true` | 启用 A\* 搜索 |
| `allow_unknown` | `true` | 允许穿越未知区域 |

### 控制器 — MPPI (DiffDrive)

| 参数 | 值 | 说明 |
|------|-----|------|
| `motion_model` | `"DiffDrive"` | 差分驱动模型 |
| `vx_max` / `vx_min` | `0.5` / `-0.35` | 线速度范围 (m/s) |
| `vy_max` | `0.0` | 禁止横向速度 |
| `wz_max` | `1.9` | 最大角速度 (rad/s) |
| `consider_footprint` | `false` | **必须为 false**（代价图用 robot_radius） |
| `time_steps` | `20` | MPPI 时间步数 |
| `batch_size` | `1000` | 采样批大小 |
| `temperature` | `0.1` | 采样温度 |

#### Critic 权重（CostCritic 必须主导）

| Critic | 权重 | 用途 |
|--------|------|------|
| **CostCritic** | **20.0** | 避障（最高优先级） |
| PathAlignCritic | 8.0 | 对齐全局路径 |
| PathFollowCritic | 5.0 | 路径前进进度 |
| GoalCritic | 5.0 | 接近目标 |
| PreferForwardCritic | 5.0 | 优先前进 |
| ConstraintCritic | 4.0 | 速度/加速度约束 |
| GoalAngleCritic | 3.0 | 朝向目标方向 |
| PathAngleCritic | 2.0 | 匹配路径朝向 |

### 速度平滑器

| 参数 | 值 | 说明 |
|------|-----|------|
| `feedback` | `"OPEN_LOOP"` | 开环模式（Cartographer 不发布 `/odom`） |
| `max_velocity` | `[0.5, 0.0, 2.0]` | X/Y/Yaw 最大速度 |
| `min_velocity` | `[-0.5, 0.0, -2.0]` | X/Y/Yaw 最小速度 |
| `input_topic` | `cmd_vel_nav` | 从控制器接收 |
| `output_topic` | `cmd_vel` | 输出给底盘驱动 |

### 全局代价图

| 参数 | 值 | 说明 |
|------|-----|------|
| `global_frame` | `map` | 全局坐标系 |
| `rolling_window` | `false` | 固定地图，不滚动 |
| `robot_radius` | `0.3` | 机器人半径 (m) |
| `plugins` | `static + obstacle + inflation` | 三层代价图 |
| `obstacle_max_range` | `6.0` | 障碍物最大探测距离 (m) |
| `raytrace_max_range` | `7.0` | 射线追踪最大距离 (m) |

### 局部代价图

| 参数 | 值 | 说明 |
|------|-----|------|
| `global_frame` | `odom` | 里程计坐标系 |
| `rolling_window` | `true` | 以机器人为中心滚动 |
| `width × height` | `4 × 4` | 窗口大小 (m) |
| `robot_radius` | `0.3` | 机器人半径 (m) |
| `plugins` | `obstacle + inflation` | 两层代价图 |
| `obstacle_max_range` | `4.0` | 障碍物最大探测距离 (m) |
| `raytrace_max_range` | `5.0` | 射线追踪最大距离 (m) |

### 行为服务器（恢复行为）

| 行为 | 说明 |
|------|------|
| `spin` | 原地旋转 |
| `backup` | 后退 |
| `wait` | 等待代价图清除 |

## 文件结构

```
nav/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── nav2_params.yaml       # Nav2 完整配置
└── launch/
    ├── localization.launch.py  # Cartographer 纯定位
    ├── navigation.launch.py    # Nav2 路径规划
    └── bringup.launch.py       # 定位 + 导航组合
```

## 关键注意事项

- **`consider_footprint: false`** 绝不能改为 `true`——代价图使用 `robot_radius` 而非 `footprint` 多边形，改为 `true` 会导致 controller_server 配置失败，lifecycle_manager 中止所有节点启动
- **lifecycle_manager 参数必须用直接字典传入**，不能从 YAML 加载——节点名 `lifecycle_manager_navigation` 与构造名 `lifecycle_manager` 不匹配，YAML 键无法匹配导致参数静默丢失
- **速度平滑器使用 OPEN_LOOP 模式**——Cartographer 只发布 TF 变换，不发布 `/odom` 话题，不支持闭环反馈
- **vy_max = 0.0**——底盘为非完整约束差分驱动，不能产生横向速度
- **全局代价图 `subscribe_to_updates: true`**——订阅 `/map` 话题更新，Cartographer 动态更新地图时代价图自动跟随
