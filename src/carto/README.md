# carto

基于 Cartographer 的 LiDAR + IMU 2D 建图，无轮式里程计。

## 使用

```bash
# 一键建图
ros2 launch carto create_map.launch.py

# 保存地图（pgm + yaml + pbstream）
ros2 launch carto save_map.launch.py map_name:=my_map
```

## 配置

[`config/cartographer_2d.lua`](config/cartographer_2d.lua) — 针对无里程计快速运动调优。
