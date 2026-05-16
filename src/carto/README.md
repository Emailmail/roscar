# carto — Cartographer SLAM 建图

基于 Google Cartographer 的 2D SLAM 建图包，适配 LD06 激光雷达 + DM 系列 IMU，无轮式里程计。

## 依赖

- `cartographer_ros`
- `dm_imu` (IMU 驱动)
- `ldlidar_stl_ros2` (LD06 激光雷达驱动)
- `tf2_ros`

## 安装

```bash
cd ~/roscar
colcon build --packages-up-to carto
source install/setup.bash
```

## 启动

### 1. SLAM 建图

```bash
ros2 launch carto create_map.launch.py
```

启动后 Cartographer 进入建图模式，发布话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 实时占据栅格地图（分辨率 0.05m） |
| `/submap_list` | `cartographer_ros_msgs/SubmapList` | 子图列表 |

同时订阅 `/imu/data`（IMU）和 `/scan`（激光雷达）进行 SLAM 优化。

### 2. 保存地图

建图完成后，在**另一个终端**执行：

```bash
ros2 launch carto save_map.launch.py
```

这会将当前地图保存为 `.pbstream` 文件到 `src/map/map.pbstream`。

**工作流程**：
1. 调用 `/finish_trajectory` 服务，结束当前轨迹
2. 等待 2 秒，确保轨迹完成
3. 调用 `/write_state` 服务，将状态写入 `.pbstream` 文件

## 配置

### SLAM 参数 (`config/mapping.lua`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tracking_frame` | `base_link` | 跟踪坐标系 |
| `map_frame` | `map` | 地图坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `provide_odom_frame` | `true` | Cartographer 发布 map→odom 变换 |
| `use_odometry` | `false` | 不使用轮式里程计 |
| `num_laser_scans` | `1` | 单线激光雷达 |
| `TRAJECTORY_BUILDER_2D.min_range` | `0.1` | 激光最小距离 (m) |
| `TRAJECTORY_BUILDER_2D.max_range` | `12.0` | 激光最大距离 (m) |
| `TRAJECTORY_BUILDER_2D.voxel_filter_size` | `0.025` | 体素滤波尺寸 (m) |
| `TRAJECTORY_BUILDER_2D.submaps.num_range_data` | `90` | 每子图激光帧数 |
| `pose_publish_period_sec` | `0.005` | 位姿发布周期 (200 Hz) |

### 纯定位参数 (`config/localization.lua`)

继承 `mapping.lua` 全部参数，并覆盖：

| 参数 | 值 | 说明 |
|------|-----|------|
| `pure_localization_trimmer.max_submaps_to_keep` | `3` | 最多保留 3 个子图 |
| `POSE_GRAPH.optimize_every_n_nodes` | `20` | 每 20 个节点优化一次位姿图 |

## TF 树

```
map → odom → base_link → base_laser  (Z=0.18m)
                       → imu_link    (identity)
```

Cartographer 追踪 `base_link`（机器人本体）。IMU 必须与追踪帧同位（Cartographer 硬性检查），`base_link → imu_link` 设为 identity 满足此要求。

| 变换 | 发布者 | 类型 |
|------|--------|------|
| `map → odom` | Cartographer | 动态（位姿图优化） |
| `odom → base_link` | Cartographer | 动态（局部 SLAM） |
| `base_link → base_laser` | carto 包 + ld06 驱动 | 静态（Z=0.18m） |
| `base_link → imu_link` | carto 包 | 静态（identity，满足 IMU 同位检查） |

> `base_link → base_laser` 由本包和 ld06 驱动双重发布，确保 TF 链不依赖单一来源。

## 话题重映射

| Cartographer 输入 | 实际话题 | 说明 |
|--------------------|----------|------|
| `imu` | `imu/data` | DM IMU 发布 |

## 文件结构

```
carto/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── mapping.lua          # SLAM 建图配置
│   └── localization.lua     # 纯定位配置
└── launch/
    ├── create_map.launch.py  # 启动建图
    └── save_map.launch.py    # 保存地图
```

## 地图文件

保存的地图文件位于 `src/map/` 目录：

- `map.pbstream`：Cartographer 序列化状态（用于纯定位加载）

`.pbstream` 文件可通过 `cartographer_assets_writer` 转换为 Nav2 兼容的 `.pgm`/`.yaml` 格式。
