# roscar

ROS 2 Jazzy 机器人项目，集成 DM IMU 姿态传感器和 LDROBOT 激光雷达（LD06），用于差分驱动小车。

**当前状态**：传感器驱动就绪，SLAM/导航待重新构建（使用 LiDAR + IMU，无轮式里程计）。

## 项目结构

```
roscar/
├── src/
│   ├── dm_imu/              # Python (ament_python) — DM IMU 串口驱动节点
│   ├── ldlidar_stl_ros2/     # C++ (ament_cmake) — LDROBOT 激光雷达驱动 (LD06/LD19/STL-27L)
│   └── map/                  # 存放 map 文件的空目录
```

`slam/`、`nav/`、`carto/` 已删除，待重新构建。

## 构建方式

```bash
cd /home/relog/roscar
colcon build --symlink-install
source install/setup.bash
```

- dm_imu: `--symlink-install` 允许 Python 文件修改后无需重新 build
- ldlidar_stl_ros2: C++ 包，修改后需重新 `colcon build`

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
- `launch/dm_imu.launch.py` — 启动文件

串口帧格式（19 字节）：`0x55 0xAA | ? | RID | 3×float32(LE) | CRC16(LE) | 0x0A`
- RID 0x01 = 加速度，0x02 = 陀螺仪，0x03 = RPY（度）
- CRC 默认包含帧头 0x55,0xAA；失败自动尝试不含帧头

默认参数：
- `port`: `/dev/ttyACM0`, `baudrate`: 921600
- `publish_imu_data: false`, `publish_rpy: true`, `publish_pose: false`
- `frame_id: imu_link`, `publish_rpy_in_degree: true`, `qos_reliable: true`

## ldlidar_stl_ros2 — C++ 激光雷达驱动

支持 LDROBOT LD06 / LD19 / STL-27L。通过串口获取点云，发布 `sensor_msgs/LaserScan`。

关键文件：
- `src/demo.cpp` — 节点入口，参数声明、雷达初始化、10Hz 主循环
- `ldlidar_driver/` — 厂商 SDK：串口通讯、数据解析(LiPKG)、滤波(TOFBF)

LD06 参数：`product_name: LDLiDAR_LD06`, `topic_name: scan`, `frame_id: base_laser`, `port_baudrate: 230400`

注意：`src/demo.cpp:161` 第一个 scan 帧会被跳过（用于计算 scan_time 差值）。

## 运行命令

```bash
# IMU
ros2 launch dm_imu dm_imu.launch.py port:=/dev/ttyACM0

# 激光雷达（LD06）
ros2 launch ldlidar_stl_ros2 ld06.launch.py

# 串口权限
sudo chmod 777 /dev/ttyUSB0   # 或对应的 tty 设备
```

## SLAM 方案（待实现）

目标：仅用 LiDAR + IMU 建图，无轮式里程计。

TF 树设计：
```
map → odom → base_link → base_laser     (Z=0.18)
                       → imu_link        (identity 或机械偏移)
```
- `map → odom`：SLAM Toolbox 发布（扫描匹配修正）
- `odom → base_link`：rf2o_laser_odometry 或类似激光里程计发布（帧间运动估计）
- `base_link → base_laser` / `base_link → imu_link`：静态变换

关键注意事项（来自之前的调试经验）：
- SLAM Toolbox (Jazzy) 是 **lifecycle 节点**，必须用 `LifecycleNode` + `TRANSITION_CONFIGURE` → `TRANSITION_ACTIVATE`
- 参数必须包含 `mode: mapping`
- 所有数值参数必须写正确类型（YAML 不加引号的 `3` 是 int，`3.0` 是 double）
- 无轮式里程计时，不能只给静态 identity `odom → base_link`——SLAM Toolbox 会因运动模型为零而卡住。需要激光里程计提供实际运动估计
- IMU 可以关掉先用纯激光跑通，再加回来调试 (`use_imu_data: false`)
- 调试用 `debug_logging: true` 配合官方默认参数模板

## 项目约定

- 话题优先级：`/imu/rpy` > `/imu/data` > `/imu/pose`
- RPY 角度默认以"度"发布
- QoS 默认 Reliable（便于 RViz 直接显示）
- 传感器帧：`imu_link`（IMU）、`base_laser`（LiDAR）
- 新增节点遵循 dm_imu 的架构模式（后台读线程 + 主线程发布，Lock 保护共享数据）

## 依赖

- ROS 2 Jazzy
- pyserial (dm_imu)
- sensor_msgs, geometry_msgs (dm_imu)
- rclcpp, sensor_msgs (ldlidar_stl_ros2)
- slam_toolbox (ros-jazzy-slam-toolbox)
- rf2o_laser_odometry（待安装，从源码构建）
