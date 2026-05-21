# roscar

ROS 2 Jazzy 机器人项目，集成 DM IMU 姿态传感器和 LDROBOT 激光雷达（LD06），用于差分驱动小车。

**当前状态**：传感器驱动就绪，Cartographer LiDAR + IMU 建图已实现（无轮式里程计）。roscar_nav 定位+Nav2路径规划已实现。
**IMU 连接方式**：DM IMU 通过 RS-485 → MAX485 TTL 模块 → GPIO UART4（`/dev/ttyAMA4`，GPIO 12/13，pins 32/33）。绕过 RP1 USB 控制器，彻底解决树莓派5 USB 周期性重置问题（详见 [doc/rpi5-usb-autosuspend-bug.md](doc/rpi5-usb-autosuspend-bug.md)）。

## 项目结构

```
roscar/
├── src/
│   ├── dm_imu/              # Python (ament_python) — DM IMU 串口驱动节点
│   ├── ldlidar_stl_ros2/     # C++ (ament_cmake) — LDROBOT 激光雷达驱动 (LD06/LD19/STL-27L)
│   ├── carto/                # C++ (ament_cmake) — Cartographer 建图 launch + config
│   ├── roscar_nav/           # Python (ament_python) — Cartographer 定位 + Nav2 路径规划
│   ├── chasis/               # Python (ament_python) — C30D 底盘串口控制 (STM32 USART3)
│   ├── core/                 # Python (ament_python) — 总控节点，系统编排调度
│   ├── doc/                  # 通信协议文档 (ros_protocol.md)
│   └── map/                  # 存放 map 文件（pgm + yaml + pbstream）
```

## 构建方式

```bash
cd /home/yilong/roscar
colcon build --symlink-install
source install/setup.bash  # 或 sic
```

- dm_imu: `--symlink-install` 允许 Python 文件修改后无需重新 build
- ldlidar_stl_ros2: C++ 包，修改后需重新 `colcon build`
- carto: C++ 包，只包含 launch/config 文件和脚本，修改后需 `colcon build`
- roscar_nav: Python 包，`--symlink-install` 允许修改后无需重新 build
- chasis: Python 包，`--symlink-install` 允许修改后无需重新 build
- core: Python 包，`--symlink-install` 允许修改后无需重新 build

## dm_imu — Python 串口节点

Python ROS 2 包，通过 `pyserial` 读取 DM IMU 串口数据，发布：
- `sensor_msgs/Imu` — `/imu/data`
- `geometry_msgs/Vector3Stamped` — `/imu/rpy`（默认度）
- `geometry_msgs/PoseStamped` — `/imu/pose`

**连接方式**：IMU 通过 RS-485 → MAX485 TTL 转换模块 → GPIO UART4（`/dev/ttyAMA4`）。USB 断开时后台读线程会自动重连（等待设备重连最多 60 秒）。

关键文件：
- `dm_imu/node.py:27` — `DmImuNode`，200Hz 定时轮询
- `dm_imu/modules/dm_serial.py:34` — `DM_Serial`，后台读线程 + 主线程取最新帧，含 USB 断开自动恢复
- `dm_imu/modules/dm_crc.py:40` — `dm_crc16`，CCITT 表驱动 CRC16（初值 0xFFFF）
- `config/params.yaml` — 默认参数
- `launch/dm_imu.launch.py` — 启动文件，默认 port `/dev/ttyAMA4`
- `launch/dm_imu_rviz.launch.py` — 启动 IMU + RViz 可视化

串口帧格式（19 字节）：`0x55 0xAA | ? | RID | 3×float32(LE) | CRC16(LE) | 0x0A`
- RID 0x01 = 加速度，0x02 = 陀螺仪，0x03 = RPY（度）
- CRC 默认包含帧头 0x55,0xAA；失败自动尝试不含帧头

默认参数：
- `port`: `/dev/ttyAMA4`, `baudrate`: 921600
- `publish_imu_data: true`, `publish_rpy: true`, `publish_pose: true`
- `frame_id: imu_link`, `publish_rpy_in_degree: true`, `qos_reliable: true`

## ldlidar_stl_ros2 — C++ 激光雷达驱动

支持 LDROBOT LD06 / LD19 / STL-27L。通过树莓派 GPIO UART（`/dev/ttyAMA0`）连接 LD06 获取点云，发布 `sensor_msgs/LaserScan`。

关键文件：
- `src/demo.cpp` — 节点入口，参数声明、雷达初始化、10Hz 主循环
- `ldlidar_driver/` — 厂商 SDK：串口通讯、数据解析(LiPKG)、滤波(TOFBF)

LD06 参数：`product_name: LDLiDAR_LD06`, `topic_name: scan`, `frame_id: base_laser`, `port_name: /dev/ttyAMA0`（树莓派 GPIO UART）, `port_baudrate: 230400`

注意：`src/demo.cpp:161` 第一个 scan 帧会被跳过（用于计算 scan_time 差值）。

Pi 5 GPIO UART 启用：`enable_uart=1` 加入 `/boot/firmware/config.txt`，重启后 `/dev/ttyAMA0` 即 GPIO14(TXD)/GPIO15(RXD)。

IMU 使用 UART4（`dtoverlay=uart4-pi5` 加入 config.txt），设备 `/dev/ttyAMA4`，引脚 GPIO12(TXD4, pin32) / GPIO13(RXD4, pin33)。

## carto — Cartographer 建图

基于 Google Cartographer 的 LiDAR + IMU 2D 建图方案，无轮式里程计。

关键文件：
- `launch/create_map.launch.py` — 一键建图（IMU + LiDAR + Cartographer）
- `launch/carto_2d.launch.py` — 仅 Cartographer（需手动启动传感器）
- `launch/save_map.launch.py` — 一键保存地图（pgm + yaml + pbstream）
- `config/cartographer_2d.lua` — 参数配置

### 运行

```bash
# 一键启动
ros2 launch carto create_map.launch.py

# 指定 IMU 端口
ros2 launch carto create_map.launch.py imu_port:=/dev/ttyAMA4
```

### TF 树

```
map → odom → base_link → base_laser     (Z=0.18)
                       → imu_link        (identity)
```
- `map → odom`：Cartographer 发布（`provide_odom_frame: true`）
- `odom → base_link`：Cartographer 局部 SLAM 发布
- `base_link → base_laser`：ld06.launch.py 中 static_transform_publisher（Z=0.18）
- `base_link → imu_link`：create_map.launch.py 中 static_transform_publisher（identity）

### 配置要点

[`cartographer_2d.lua`](src/carto/config/cartographer_2d.lua)：
```lua
tracking_frame = "imu_link"   -- IMU 提供重力对齐
published_frame = "base_link"
provide_odom_frame = true     -- 由 Cartographer 发布 odom→base_link
use_odometry = false          -- 无轮式里程计
use_imu_data = true           -- IMU 提供旋转先验

-- 针对快速运动调优（相比默认值）
motion_filter.max_distance_meters = 0.1    -- 更密集插入 scan（默认 0.2）
motion_filter.max_angle_radians = 0.5°     -- 更密集插入 scan（默认 1°）
real_time_correlative_scan_matcher.linear_search_window = 0.3   -- 扩大搜索（默认 0.1）
real_time_correlative_scan_matcher.angular_search_window = 30°  -- 扩大搜索（默认 20°）
ceres_scan_matcher.occupied_space_weight = 20.  -- 更信任激光数据（默认 1）
ceres_scan_matcher.translation_weight = 5.      -- 降低先验权重（默认 10）
ceres_scan_matcher.rotation_weight = 20.        -- 降低先验权重（默认 40）
adaptive_voxel_filter.min_num_points = 100      -- 保留更多特征点（默认 200）
```

### 保存地图

建图满意后建议保存两种格式：

```bash
# 一键保存（推荐）
ros2 launch carto save_map.launch.py map_name:=my_map
# 生成 my_map.pgm + my_map.yaml + my_map.pbstream

# 或手动分别保存
ros2 run nav2_map_server map_saver_cli -f /home/yilong/roscar/src/map/my_map
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/yilong/roscar/src/map/my_map.pbstream'}"

# 从 pbstream 离线导出栅格地图
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/yilong/roscar/src/map/my_map.pbstream \
  -map_filestem /home/yilong/roscar/src/map/my_map
```

### 调试经验

- **TF "imu_link does not exist"**：需要 `base_link → imu_link` 的 static_transform_publisher。`carto_2d.launch.py` 有但 `create_map.launch.py` 如果漏了就会报这个 warning。
- **`ordered_multi_queue` waiting for IMU**：Cartographer 在等 IMU 数据但没收到。检查 IMU 端口是否正确（`/dev/ttyAMA4`）、`publish_imu_data` 是否为 true、以及 `ros2 topic hz /imu/data` 确认 IMU 是否在发数据。
- **配置参数导致 SIGABRT**：Cartographer Lua 配置中，某些参数值（如 `occupied_space_weight` 过高、`ceres_solver_options` 字段覆盖、`use_imu_based` 开启等）可能导致崩溃。遇到 SIGABRT 时先回退配置，逐参数排查。
- **IMU 可以临时关掉**：设置 `use_imu_data = false` 用纯激光跑通，确认 SLAM 本身没问题后再加回 IMU 调试。

## 运行命令

```bash
# 串口权限
sudo chmod 777 /dev/ttyAMA0  # LD06 GPIO UART
sudo chmod 777 /dev/ttyAMA4  # IMU GPIO UART4
sudo chmod 777 /dev/ttyACM0  # C30D 底盘 STM32 USART3

# 一键建图
ros2 launch carto create_map.launch.py

# 或分步启动
ros2 launch dm_imu dm_imu.launch.py
ros2 launch ldlidar_stl_ros2 ld06.launch.py
ros2 launch carto carto_2d.launch.py

# 底盘控制
ros2 launch chasis c30d_ctrl.launch.py
# 键盘遥控
ros2 launch chasis c30d_keyctrl.launch.py

# 保存地图
ros2 launch carto save_map.launch.py map_name:=my_map
```

## 项目约定

- 话题优先级：`/imu/rpy` > `/imu/data` > `/imu/pose`
- RPY 角度默认以"度"发布
- QoS 默认 Reliable（便于 RViz 直接显示）
- 传感器帧：`imu_link`（IMU）、`base_laser`（LiDAR）
- 新增节点遵循 dm_imu 的架构模式（后台读线程 + 主线程发布，Lock 保护共享数据）
- Cartographer 无轮式里程计，依赖 IMU 旋转 + 扫描匹配做位姿估计
- IMU 端口默认 `/dev/ttyAMA4`（GPIO UART4，RS-485 → MAX485 TTL）
- LD06 端口默认 `/dev/ttyAMA0`（GPIO UART）
- C30D 底盘端口默认 `/dev/ttyACM0`（STM32 USART3）
- `sic` = source install/setup.bash（定义在 `~/.bashrc`）

## chasis — C30D 底盘串口控制

通过串口向 STM32 底盘控制板 (USART3) 下发速度指令，协议详见 [src/doc/ros_protocol.md](src/doc/ros_protocol.md)。

两个节点：
- **c30d_ctrl** — 底盘驱动节点，订阅 `/cmd_vel` 并通过串口下发 11 字节控制帧
- **key_ctrl** — 键盘遥操作节点，监听 WASD + Q/E 键发布 `/cmd_vel`

关键文件：
- `chasis/c30d_ctrl.py:45` — `C30dCtrlNode`，50Hz 定时发送，订阅 `/cmd_vel`
- `chasis/key_ctrl.py:23` — `KeyCtrlNode`，后台键盘线程 + 定时发布，含松键自动停车
- `launch/c30d_ctrl.launch.py` — 仅启动底盘控制
- `launch/c30d_keyctrl.launch.py` — 底盘控制 + 键盘遥操作（需要 xterm）
- `config/c30d_params.yaml` — 默认参数

### 通信参数

- 波特率：115200 8N1，无硬件流控
- 下行帧：11 字节（帧头 `0x7B`/帧尾 `0x7D`，XOR 校验）
- 速度编码：mm/s，大端 int16，3 轴 (vx/vy/vz)

### 运行

```bash
# 仅底盘控制（接收 Nav2 的 /cmd_vel）
ros2 launch chasis c30d_ctrl.launch.py

# 底盘控制 + 键盘遥控
ros2 launch chasis c30d_keyctrl.launch.py

# 指定串口
ros2 launch chasis c30d_ctrl.launch.py port:=/dev/ttyUSB0
```

### 键盘操控

| 键 | 动作 |
|----|------|
| W/S | 前进/后退 |
| A/D | 左移/右移 |
| Q/E | 逆时针/顺时针旋转 |
| 其他键 | 停车 |
| 松键超时 | 0.3 秒自动停车 |

## core — 总控节点，系统编排调度

一键启动多子系统组合。

关键文件：
- `launch/explore_manu.launch.py` — 键盘操控 + 实时建图（carto + chasis）

### 运行

```bash
# 键盘操控 + 实时建图
ros2 launch core explore_manu.launch.py

# 指定串口
ros2 launch core explore_manu.launch.py imu_port:=/dev/ttyUSB0 chasis_port:=/dev/ttyACM0
```

## roscar_nav — Cartographer 定位 + Nav2 路径规划

以静态地图为参考完成定位（Cartographer 纯定位模式）以及路径规划（Nav2）。

关键文件：
- `launch/navigate.launch.py` — 一键启动：定位 + Nav2（IMU + LiDAR + Cartographer 定位 + Nav2 Plannner/Controller/BT）
- `launch/localize.launch.py` — 仅定位（IMU + LiDAR + Cartographer 定位 + pose_to_odom）
- `config/carto_localize.lua` — Cartographer 纯定位 Lua 参数
- `config/nav2_params.yaml` — Nav2 全部参数（planner/controller/costmap/BT/behavior）
- `roscar_nav/pose_to_odom.py` — TF 变换 → `/odom` Odometry 话题（Nav2 Controller 需要）。参数：`publish_rate`(50Hz)、`odom_frame`、`base_frame`
- `rviz/nav2_view.rviz` — RViz 预置配置（含 Map/TF/LaserScan/Plan/Costmap/GoalTool）

### 架构

```
DM_IMU → /imu/data ──┐
LD06   → /scan     ──┤
                     ├── Cartographer 纯定位 ── TF: map → odom → base_link
Map Server (pgm)  ──┤                           └── /odom (pose_to_odom)
                     └── Nav2 Planner + Controller ── /cmd_vel ── c30d_ctrl → STM32
```

### 运行

```bash
# 一键启动定位 + 路径规划（headless，默认 use_rviz=false）
ros2 launch roscar_nav navigate.launch.py

# 带 RViz
ros2 launch roscar_nav navigate.launch.py use_rviz:=true

# 仅定位
ros2 launch roscar_nav localize.launch.py

# 指定初始位姿
ros2 launch roscar_nav navigate.launch.py \
    initial_x:=1.0 initial_y:=0.5 initial_yaw_deg:=90.0

# 在 RViz 中：先用 2D Pose Estimate 标初始位姿，再用 Nav2 Goal 点目标
```

### RViz 使用

```bash
source /home/yilong/roscar/install/setup.bash
rviz2 -d /home/yilong/roscar/install/roscar_nav/share/roscar_nav/rviz/nav2_view.rviz
```
必须加载预置配置文件，裸 `rviz2` 没有预置 Display 看不到 `/map`。

### 配置要点

[`carto_localize.lua`](src/roscar_nav/config/carto_localize.lua) vs 建图配置的区别：
- `pure_localization_trimmer = { max_submaps_to_keep = 6 }` — 限制活跃 submap 数量
- `optimize_every_n_nodes = 20` — 定期优化但不如建图频繁
- `submaps.num_range_data = 160` — 更少创建 submap（建图为 90）
- 其余扫描匹配参数与建图一致

`localize.launch.py` 使用 `OpaqueFunction` 仅在 initial_x/y/yaw_deg 非零时才传 `-initial_trajectory_pose` 给 Cartographer，避免每次都重新初始化。`map_dir` 默认 `/home/yilong/roscar/src/map`。

[`nav2_params.yaml`](src/roscar_nav/config/nav2_params.yaml)：
- 全局规划器：Smac Hybrid-A* (Reeds-Shepp)
- 局部控制器：Regulated Pure Pursuit
- 最大线速度：0.1 m/s，轮距：0.19m
- 无需 AMCL（Cartographer 提供 `map→odom→base_link`）
- `bt_navigator` 不设 `plugin_lib_names`（用 Nav2 默认列表避免重复注册）

### Nav2 调试

- **bt_navigator "ID already registered"**：不要手动指定 `plugin_lib_names`。Jazzy 的 bt_navigator 自动加载核心 BT 插件，手动指定会导致重复。
- **behavior_server 插件名**：用 `nav2_behaviors::Spin` 而非 `nav2_spin_behavior::Spin`。
- **Cartographer `std::length_error`**：`max_submaps_to_keep` 过小（如 3）可能触发。增大到 6 可解决。

## 依赖

- ROS 2 Jazzy
- cartographer_ros (ros-jazzy-cartographer-ros)
- nav2_map_server (ros-jazzy-nav2-map-server)
- nav2_planner, nav2_controller, nav2_bt_navigator, nav2_behaviors, nav2_lifecycle_manager
- pyserial (dm_imu)
- sensor_msgs, geometry_msgs, nav_msgs (dm_imu + roscar_nav)
- rclcpp, sensor_msgs, tf2_ros (ldlidar_stl_ros2)
- pyserial, geometry_msgs (chasis)
