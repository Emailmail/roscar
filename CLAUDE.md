# roscar

ROS 2 Jazzy 机器人项目，集成 DM IMU 姿态传感器和 LDROBOT 激光雷达（LD06），用于差分驱动小车。

**当前状态**：传感器驱动就绪，Cartographer LiDAR + IMU 建图已实现（无轮式里程计）。

## 项目结构

```
roscar/
├── src/
│   ├── dm_imu/              # Python (ament_python) — DM IMU 串口驱动节点
│   ├── ldlidar_stl_ros2/     # C++ (ament_cmake) — LDROBOT 激光雷达驱动 (LD06/LD19/STL-27L)
│   ├── carto/                # C++ (ament_cmake) — Cartographer launch + config
│   └── map/                  # 存放 map 文件（pgm + yaml + pbstream）
```

## 构建方式

```bash
cd /home/relog/roscar
colcon build --symlink-install
source install/setup.bash
```

- dm_imu: `--symlink-install` 允许 Python 文件修改后无需重新 build
- ldlidar_stl_ros2: C++ 包，修改后需重新 `colcon build`
- carto: C++ 包，只包含 launch/config 文件和脚本，修改后需 `colcon build`

## dm_imu — Python 串口节点

Python ROS 2 包，通过 `pyserial` 读取 DM IMU 串口数据，发布：
- `sensor_msgs/Imu` — `/imu/data`
- `geometry_msgs/Vector3Stamped` — `/imu/rpy`（默认度）
- `geometry_msgs/PoseStamped` — `/imu/pose`

关键文件：
- `dm_imu/node.py:27` — `DmImuNode`，200Hz 定时轮询
- `dm_imu/modules/dm_serial.py:34` — `DM_Serial`，后台读线程 + 主线程取最新帧
- `dm_imu/modules/dm_crc.py:40` — `dm_crc16`，CCITT 表驱动 CRC16（初值 0xFFFF）
- `config/params.yaml` — 默认参数
- `launch/dm_imu.launch.py` — 启动文件，默认 port `/dev/ttyACM0`

串口帧格式（19 字节）：`0x55 0xAA | ? | RID | 3×float32(LE) | CRC16(LE) | 0x0A`
- RID 0x01 = 加速度，0x02 = 陀螺仪，0x03 = RPY（度）
- CRC 默认包含帧头 0x55,0xAA；失败自动尝试不含帧头

默认参数：
- `port`: `/dev/ttyACM0`, `baudrate`: 921600
- `publish_imu_data: true`, `publish_rpy: true`, `publish_pose: true`
- `frame_id: imu_link`, `publish_rpy_in_degree: true`, `qos_reliable: true`

## ldlidar_stl_ros2 — C++ 激光雷达驱动

支持 LDROBOT LD06 / LD19 / STL-27L。通过串口获取点云，发布 `sensor_msgs/LaserScan`。

关键文件：
- `src/demo.cpp` — 节点入口，参数声明、雷达初始化、10Hz 主循环
- `ldlidar_driver/` — 厂商 SDK：串口通讯、数据解析(LiPKG)、滤波(TOFBF)

LD06 参数：`product_name: LDLiDAR_LD06`, `topic_name: scan`, `frame_id: base_laser`, `port_baudrate: 230400`

注意：`src/demo.cpp:161` 第一个 scan 帧会被跳过（用于计算 scan_time 差值）。

## carto — Cartographer 建图

基于 Google Cartographer 的 LiDAR + IMU 2D 建图方案，无轮式里程计。

关键文件：
- `launch/create_map.launch.py` — 一键启动（IMU + LiDAR + Cartographer）
- `launch/carto_2d.launch.py` — 仅 Cartographer（需手动启动传感器）
- `config/cartographer_2d.lua` — 参数配置

### 运行

```bash
# 一键启动
ros2 launch carto create_map.launch.py

# 指定 IMU 端口
ros2 launch carto create_map.launch.py imu_port:=/dev/ttyACM0
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

建图满意后，建议保存两种格式：

```bash
# 1. 栅格地图 — 给导航/AMCL 用
ros2 run nav2_map_server map_saver_cli -f /home/relog/roscar/src/map/my_map
# 生成 my_map.pgm + my_map.yaml

# 2. pbstream — Cartographer 完整 SLAM 状态（子图、位姿图、IMU 标定）
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/relog/roscar/src/map/my_map.pbstream'}"
# 可用来继续建图，或离线转为栅格地图

# 从 pbstream 离线导出栅格地图
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/relog/roscar/src/map/my_map.pbstream \
  -map_filestem /home/relog/roscar/src/map/my_map
```

### 调试经验

- **TF "imu_link does not exist"**：需要 `base_link → imu_link` 的 static_transform_publisher。`carto_2d.launch.py` 有但 `create_map.launch.py` 如果漏了就会报这个 warning。
- **`ordered_multi_queue` waiting for IMU**：Cartographer 在等 IMU 数据但没收到。检查 IMU 端口是否正确（`/dev/ttyACM0` vs `/dev/ttyACM1`）、`publish_imu_data` 是否为 true、以及 `ros2 topic hz /imu/data` 确认 IMU 是否在发数据。
- **配置参数导致 SIGABRT**：Cartographer Lua 配置中，某些参数值（如 `occupied_space_weight` 过高、`ceres_solver_options` 字段覆盖、`use_imu_based` 开启等）可能导致崩溃。遇到 SIGABRT 时先回退配置，逐参数排查。
- **IMU 可以临时关掉**：设置 `use_imu_data = false` 用纯激光跑通，确认 SLAM 本身没问题后再加回 IMU 调试。

## 运行命令

```bash
# 串口权限
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0

# 一键建图
ros2 launch carto create_map.launch.py

# 或分步启动
ros2 launch dm_imu dm_imu.launch.py
ros2 launch ldlidar_stl_ros2 ld06.launch.py
ros2 launch carto carto_2d.launch.py

# 保存地图
ros2 run nav2_map_server map_saver_cli -f /home/relog/roscar/map/my_map
```

## 项目约定

- 话题优先级：`/imu/rpy` > `/imu/data` > `/imu/pose`
- RPY 角度默认以"度"发布
- QoS 默认 Reliable（便于 RViz 直接显示）
- 传感器帧：`imu_link`（IMU）、`base_laser`（LiDAR）
- 新增节点遵循 dm_imu 的架构模式（后台读线程 + 主线程发布，Lock 保护共享数据）
- Cartographer 无轮式里程计，依赖 IMU 旋转 + 扫描匹配做位姿估计
- IMU 端口默认 `/dev/ttyACM0`

## 依赖

- ROS 2 Jazzy
- cartographer_ros (ros-jazzy-cartographer-ros)
- nav2_map_server (ros-jazzy-nav2-map-server)
- pyserial (dm_imu)
- sensor_msgs, geometry_msgs (dm_imu)
- rclcpp, sensor_msgs, tf2_ros (ldlidar_stl_ros2)
