# 树莓派5 USB CDC ACM 设备间歇性断开问题

## 现象

在树莓派5上使用 DM IMU（USB CDC ACM 设备，VID:PID = `ffff:ffff`）时，IMU 运行一段时间（几十秒到几分钟）后完全停止发送数据：

- `/imu/rpy` 话题不再发布新消息
- 关闭 ROS 节点后重启失败，必须重新插拔 USB 才能恢复
- **同一 IMU 在 x86 PC（Ubuntu）上完全正常**

## 根因

**IMU 在树莓派5上会间歇性 USB 断开重连**，内核日志记录：

```
00:38:41  usb 4-1: new full-speed USB device number 3  ← 首次连接
00:55:11  usb 4-1: USB disconnect, device number 3     ← 断开！
00:55:15  usb 4-1: new full-speed USB device number 4  ← 重连
01:03:53  usb 4-1: USB disconnect, device number 4     ← 又断开！
01:03:55  usb 4-1: new full-speed USB device number 5  ← 又重连
```

断开原因可能是：
- 树莓派5 USB 接口供电不足或不稳定
- RP1 芯片的 xHCI 控制器与某些 USB 设备存在兼容性问题
- USB 线缆质量/信号完整性

当 IMU 断开时，dm_imu 节点持有的 `/dev/ttyACM0` 文件描述符变为陈旧（指向已不存在的设备），`select()` 永远等不到数据，表现为"卡死"。重连后新设备创建了新的 `/dev/ttyACM0`，但节点仍在使用旧 fd，因此即使手动重启节点也可能失败（旧进程未正确关闭端口）。

## 解决方案

### 软件层面（已实施）

修改 `dm_serial.py` 的 `_reader_loop`，增加 USB 断开检测和自动恢复：

1. **I/O 异常捕获**：`SerialException`/`OSError` 触发时，关闭旧端口并等待设备重连
2. **无数据超时检测**：连续 2 秒无数据视为可能断开，触发重连流程
3. **自动重连**：等待 `/dev/ttyACM0` 消失后重新出现，自动打开并恢复读取
4. **错误日志**：`node.py` 每 2 秒检查 `last_error()` 并通过 ROS logger 输出

### 硬件层面（建议排查）

如果软件恢复后断开频率仍然很高，建议：

1. **使用带外部供电的 USB Hub**（最可能有效的方案）
2. **更换短一点、质量好的 USB 线缆**
3. **检查树莓派5 供电**（是否使用官方电源适配器，输出是否足够）
4. **将 IMU 插到不同的 USB 口试试**（树莓派5 有 4 个 USB 口）

### 内核参数（备选，非必要）

`usbcore.autosuspend=-1` 已确认**不能**解决此问题（断开是设备主动行为，非主机 autosuspend 导致）。

## 相关修改

- [dm_serial.py](../src/dm_imu/dm_imu/modules/dm_serial.py) — `_reader_loop`, `_wait_reconnect`, `_close_serial`
- [node.py](../src/dm_imu/dm_imu/node.py) — `_on_timer_stats`, `destroy_node`
