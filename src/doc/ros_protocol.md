# WHEELTEC STM32 与 ROS 上位机通信协议

## 1. 概述

本文档描述 WHEELTEC 多模态机器人控制固件 (V5.0, STM32F407) 与 ROS 上位机之间的串口通信协议。通信为双向：STM32 周期性地向 ROS 主机上报机器人状态，ROS 主机向 STM32 下发速度控制指令。

## 2. 通信配置

### 2.1 串口端口与引脚

STM32 有三路串口可用于与 ROS 主机通信：

| 端口     | 默认角色     | TX 引脚 | RX 引脚 | 时钟总线 |
|----------|--------------|---------|---------|----------|
| USART1   | 备用 ROS 端口 | PA9     | PA10    | APB2     |
| USART3   | **默认 ROS 端口** | PD8     | PD9     | APB1     |
| UART5    | 备用 ROS 端口 | PC12    | PD2     | APB1     |

> USART2 (PD5/PD6, 9600bps) 专用于蓝牙 APP 遥控，不参与 ROS 通信。

### 2.2 串口帧格式

所有端口统一使用以下帧格式：

| 参数       | 值                       |
|------------|--------------------------|
| 波特率     | **115200** bps           |
| 数据位     | 8                        |
| 停止位     | 1                        |
| 校验位     | 无 (None)                |
| 硬件流控   | 无 (None)                |
| GPIO 模式  | 推挽复用 (AF_PP)          |
| GPIO 上拉  | 内部上拉 (PUPD_UP)        |
| GPIO 速率  | 50 MHz                   |

> USART3 代码中有 230400 bps 的注释备用配置（`BALANCE/system.c:145`），默认使用 115200。

### 2.3 中断优先级

| 端口   | 抢占优先级 | 子优先级 |
|--------|-----------|---------|
| USART1 | 1         | 0       |
| USART3 | 2         | 0       |
| UART5  | 2         | 0       |

### 2.4 控制输入优先级

系统支持 6 个并发控制源，通过标志位选通。**默认控制源为 USART3**（所有标志位均为 0）。当某个端口收到有效控制帧时，该端口将自身的标志位置 1，并将其他端口标志位清零，从而独占控制权。

| 控制源     | 标志位          | 优先级说明     |
|------------|-----------------|---------------|
| USART3     | 默认 (全 0)      | 默认 ROS 端口  |
| USART1     | `Usart1_ON_Flag` | 抢占式切换     |
| UART5      | `Usart5_ON_Flag` | 抢占式切换     |
| PS2 手柄   | `PS2_ON_Flag`    | 物理遥控器     |
| 蓝牙 APP   | `APP_ON_Flag`    | 手机 APP       |
| CAN 总线   | `CAN_ON_Flag`    | 工业总线       |

## 3. 上行数据包 (STM32 -> ROS 主机)

### 3.1 数据包格式

STM32 以 **20 Hz** 频率向 ROS 主机上报 24 字节的状态数据包。

| 字节偏移 | 字段名          | 类型     | 字节序 | 说明                                     |
|----------|----------------|----------|--------|------------------------------------------|
| 0        | Frame Header   | `uint8`  | -      | 帧头，固定值 `0x7B`                      |
| 1        | Flag_Stop      | `uint8`  | -      | 软件失效标志，0=正常，非0=停机/故障       |
| 2-3      | X_speed        | `int16`  | 大端   | X 轴线速度，单位 **mm/s**                 |
| 4-5      | Y_speed        | `int16`  | 大端   | Y 轴线速度，单位 **mm/s**                 |
| 6-7      | Z_speed        | `int16`  | 大端   | Z 轴角速度，单位 **mm/s**（ROS 端需转换为 rad/s） |
| 8-9      | Accel_X        | `int16`  | 大端   | IMU 加速度 X 轴（已转换至 ROS 坐标系）     |
| 10-11    | Accel_Y        | `int16`  | 大端   | IMU 加速度 Y 轴（已转换至 ROS 坐标系）     |
| 12-13    | Accel_Z        | `int16`  | 大端   | IMU 加速度 Z 轴                           |
| 14-15    | Gyro_X         | `int16`  | 大端   | IMU 陀螺仪 X 轴（已转换至 ROS 坐标系）     |
| 16-17    | Gyro_Y         | `int16`  | 大端   | IMU 陀螺仪 Y 轴（已转换至 ROS 坐标系）     |
| 18-19    | Gyro_Z         | `int16`  | 大端   | IMU 陀螺仪 Z 轴（Flag_Stop=1 时强制为 0）  |
| 20-21    | Power_Voltage  | `int16`  | 大端   | 电池电压 × 1000（如 12.5V → 12500）       |
| 22       | Checksum       | `uint8`  | -      | 字节 0~21 的 XOR 校验和                   |
| 23       | Frame Tail     | `uint8`  | -      | 帧尾，固定值 `0x7D`                       |

**总长度：24 字节**

### 3.2 速度解算说明

X/Y/Z 速度值由各电机编码器速度通过运动学正解得出，具体算法因车型而异（麦克纳姆轮、全向轮、阿克曼、差速、四轮、坦克）。

### 3.3 IMU 坐标系变换 (IMU → ROS)

STM32 发送的 IMU 数据已经过坐标系转换，适配 ROS 标准坐标系：

| ROS 轴 | 数据来源   |
|--------|-----------|
| Accel / Gyro X | `imu.y` 轴 |
| Accel / Gyro Y | `-imu.x` 轴（取反） |
| Accel / Gyro Z | `imu.z` 轴（保持） |

> 注意：当 `Flag_Stop = 1` 时，Gyro_Z 被强制置零以抑制静态漂移噪声。

### 3.4 电池电压解码

```
实际电压 (V) = 接收值 / 1000
```

例如：收到 `12500` → 电压为 `12.5V`。

### 3.5 自动充电上行数据包（可选）

当硬件支持自动充电且 `Get_Charging_HardWare` 标志位置位时，在 24 字节主数据包之后立即发送 8 字节的充电状态数据包。帧头和帧尾与主数据包不同。

| 字节偏移 | 字段              | 类型     | 说明                        |
|----------|-------------------|----------|-----------------------------|
| 0        | Frame Header      | `uint8`  | 帧头，固定值 `0x7C`          |
| 1-2      | Charging_Current  | `int16`  | 当前充电电流                 |
| 3        | RED               | `uint8`  | 红外传感器状态               |
| 4        | Charging          | `uint8`  | 是否正在充电 (0/1)            |
| 5        | Allow_Recharge    | `uint8`  | 是否允许自动充电              |
| 6        | Checksum          | `uint8`  | 字节 0~5 的 XOR 校验和       |
| 7        | Frame Tail        | `uint8`  | 帧尾，固定值 `0x7F`          |

## 4. 下行数据包 (ROS 主机 -> STM32)

### 4.1 数据包格式

ROS 主机向 STM32 下发 **11 字节**的控制指令。

| 字节偏移 | 字段          | 类型     | 字节序 | 说明                                      |
|----------|---------------|----------|--------|-------------------------------------------|
| 0        | Frame Header  | `uint8`  | -      | 帧头，固定值 `0x7B`                       |
| 1        | Command       | `uint8`  | -      | 命令字节，控制模式选择（详见 4.2）           |
| 2        | (保留)        | `uint8`  | -      | 保留字段，参与校验但未被使用                |
| 3-4      | X_speed_raw   | `int8`×2 | 大端   | X 目标速度，编码格式详见 4.3               |
| 5-6      | Y_speed_raw   | `int8`×2 | 大端   | Y 目标速度                                 |
| 7-8      | Z_speed_raw   | `int8`×2 | 大端   | Z 目标角速度（阿克曼车型会转向前轮转角）      |
| 9        | Checksum      | `uint8`  | -      | 字节 0~8 的 XOR 校验和                     |
| 10       | Frame Tail    | `uint8`  | -      | 帧尾，固定值 `0x7D`                       |

**总长度：11 字节**

### 4.2 命令字节 (Byte 1)

| 命令值 | 模式            | 行为说明                                                   |
|--------|----------------|------------------------------------------------------------|
| `0x00` | 正常控制        | 关闭自动充电，清零所有其他控制源标志，USART3 接管控制权。速度写入 `Move_X/Y/Z` |
| `0x01` | 自动充电-导航   | 开启自动充电 (`Allow_Recharge=1`)。若红外传感器未触发 (`RED_STATE==0`)，设置 `nav_walk=1` 导航至充电桩。速度写入 `Recharge_UP_Move_X/Y/Z` |
| `0x02` | 自动充电-常规   | 开启自动充电。速度写入 `Recharge_UP_Move_X/Y/Z`             |
| `0x03` | 红外对接        | 红外对接模式。速度写入 `Red_Docker_X/Y/Z`                   |
| `0xFF` | IP 配置帧       | 字节 3~6 编码目标 IP 地址。需**连续 50 次**收到相同的 IP 帧才确认写入（防误触发） |

### 4.3 速度编码格式

每轴速度使用 **2 字节**，编码规则：

```
raw_value = (High << 8) | Low     // 拼成 int16，单位为 mm/s
speed_m_s = raw_value / 1000      // 转换为 m/s (float)
```

**示例：**

| 期望速度   | 高字节 `rxbuf[3/5/7]` | 低字节 `rxbuf[4/6/8]` |
|------------|----------------------|----------------------|
| +1.0 m/s   | `0x03`               | `0xE8` (1000)        |
| +0.1 m/s   | `0x00`               | `0x64` (100)         |
| 0 m/s      | `0x00`               | `0x00`               |
| -0.2 m/s   | `0xFF`               | `0x38` (-200)        |
| -1.0 m/s   | `0xFC`               | `0x18` (-1000)       |

> 计算公式：`transition = (High << 8) | Low; speed = transition/1000 + (transition%1000)*0.001;`

### 4.4 阿克曼车型特殊处理

当车型为阿克曼 (`Car_Mode == Akm_Car`) 时，Z 轴速度会被 `Vz_to_Akm_Angle()` 函数转换为前轮转向角而非直接作为角速度。该函数同时考虑最小转弯半径约束 (`MINI_AKM_MIN_TURN_RADIUS`)。

### 4.5 IP 配置帧 (命令 `0xFF`)

当命令字节为 `0xFF` 时，字节 3~6 编码目标 IP：

```
IP[0] = rxbuf[3]
IP[1] = rxbuf[4]
IP[2] = rxbuf[5]
IP[3] = rxbuf[6]
```

需要**连续 50 帧** (`IP_CONFIRM_COUNT = 50`) 收到相同的 IP 值才会确认并保存。如果连续帧的 IP 值发生变化，计数器重置为 1 并跟踪新 IP。

## 5. 校验和计算

校验和采用**逐字节 XOR（异或）**计算：

```
checksum = byte[0] ^ byte[1] ^ ... ^ byte[N-1]
```

| 数据包类型   | 校验范围    | 校验结果存放位置 |
|-------------|------------|----------------|
| 主 TX 包    | 字节 0 ~ 21 | 字节 22        |
| RX 包        | 字节 0 ~ 8  | 字节 9         |
| 自动充电 TX 包 | 字节 0 ~ 5  | 字节 6         |

### 伪代码

```python
def xor_checksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result
```

## 6. 数据包解析流程

### 6.1 接收状态机

```
空闲 → 收到 0x7B (帧头) → 逐字节填充缓冲 → 满 11 字节 →
  → 检查 rxbuf[10] == 0x7D (帧尾)
    → 检查 checksum == rxbuf[9]
      → 解析命令字节 → 提取速度 → 重置 command_lost_count = 0
```

- 若第一个字节不是 `0x7B`，计数器保持为 0，不进入解析流程。
- 若帧尾或校验和不匹配，数据包被静默丢弃。

### 6.2 启动保护

上电后 10 秒内 (`Time_count < CONTROL_DELAY = 1000`)，所有串口接收数据均被忽略，防止启动过程中的干扰信号引发误动作。

### 6.3 命令丢失检测

每次成功解析有效数据包时，`command_lost_count` 被重置为 0。ROS 上位机可通过持续发送控制帧来保持连接活跃，若 STM32 在一定时间内未收到有效帧，可由控制循环判定超时。

## 7. 常量速查表

| 常量名                   | 十六进制 | 十进制 | 说明                |
|--------------------------|----------|--------|---------------------|
| `FRAME_HEADER`           | `0x7B`   | 123    | 主数据包帧头         |
| `FRAME_TAIL`             | `0x7D`   | 125    | 主数据包帧尾         |
| `AutoCharge_HEADER`      | `0x7C`   | 124    | 充电数据包帧头       |
| `AutoCharge_TAIL`        | `0x7F`   | 127    | 充电数据包帧尾       |
| `SEND_DATA_SIZE`         | -        | 24     | 主 TX 数据包字节数   |
| `RECEIVE_DATA_SIZE`      | -        | 11     | RX 数据包字节数      |
| `AutoCharge_DATA_SIZE`   | -        | 8      | 充电 TX 数据包字节数 |
| `CONTROL_DELAY`          | -        | 1000   | 启动后禁止控制的滴答数 |
| `IP_CONFIRM_COUNT`       | -        | 50     | IP 确认所需的连续帧数 |

## 8. 数据包帧边界识别

由于帧头 `0x7B` 和帧尾 `0x7D` 分别为 `{` 和 `}` 的 ASCII 码值，在调试时可通过串口助手以文本模式直观地观察帧边界。

上行数据包示例（十六进制）：
```
7B 00 03 E8 00 00 00 00 FF FF FF FF FF FF FF FF FF FF FF FF 30 D4 00 7D
│    │       │       │       │               │               │    │  │
│    │       │       │       │               │               │    │  └── 帧尾
│    │       │       │       │               │               │    └── 校验和
│    │       │       │       │               │               └── 电池电压×1000
│    │       │       │       │               └── 陀螺仪 X/Y/Z
│    │       │       │       └── 加速度 X/Y/Z
│    │       │       └── Z 角速度
│    │       └── Y 线速度
│    └── X 线速度 (0x03E8 = 1000 mm/s)
└── 帧头 0x7B
```

下行数据包示例（十六进制）—— 设置 X 轴速度 0.5 m/s：
```
7B 00 00 01 F4 00 00 00 00 8F 7D
│  │  │  │     │     │     │  │
│  │  │  │     │     │     │  └── 帧尾
│  │  │  │     │     │     └── 校验和 (0x7B^0x00^0x00^0x01^0xF4^0x00^0x00^0x00^0x00)
│  │  │  │     │     └── Z 速度 = 0
│  │  │  │     └── Y 速度 = 0
│  │  │  └── X 速度 = 0x01F4 = 500 mm/s = 0.5 m/s
│  │  └── 保留
│  └── 命令 = 0x00 (正常控制)
└── 帧头
```

## 9. ROS 端对接建议

### 9.1 上位机初始化步骤

1. 打开串口设备（如 `/dev/ttyUSB0`，对应 STM32 的 USART3）。
2. 配置串口参数：115200 bps, 8N1, 无硬件流控。
3. 启动独立线程循环读取 24 字节上行数据包，按第 3 节格式解析。
4. 启动独立线程以 **20~50 Hz** 周期发送 11 字节下行控制指令。
5. 上电后等待至少 **10 秒**，待 STM32 启动保护期结束后再发送控制指令。

### 9.2 上行数据解析伪代码

```python
import struct

def parse_upload_packet(data: bytes):
    """解析 STM32 上报的 24 字节数据包"""
    if len(data) != 24:
        return None
    if data[0] != 0x7B or data[23] != 0x7D:
        return None
    # 校验和
    if xor_checksum(data[:22]) != data[22]:
        return None

    return {
        "flag_stop": data[1],
        "x_speed_mm_s": int.from_bytes(data[2:4], 'big', signed=True),
        "y_speed_mm_s": int.from_bytes(data[4:6], 'big', signed=True),
        "z_speed_mm_s": int.from_bytes(data[6:8], 'big', signed=True),
        "accel_x": int.from_bytes(data[8:10], 'big', signed=True),
        "accel_y": int.from_bytes(data[10:12], 'big', signed=True),
        "accel_z": int.from_bytes(data[12:14], 'big', signed=True),
        "gyro_x": int.from_bytes(data[14:16], 'big', signed=True),
        "gyro_y": int.from_bytes(data[16:18], 'big', signed=True),
        "gyro_z": int.from_bytes(data[18:20], 'big', signed=True),
        "voltage": int.from_bytes(data[20:22], 'big', signed=True) / 1000.0,
    }
```

### 9.3 下行控制命令伪代码

```python
def build_control_packet(vx_ms: float, vy_ms: float, vz_ms: float, cmd: int = 0x00):
    """构建 11 字节下行控制数据包"""
    vx_raw = int(vx_ms * 1000)   # m/s -> mm/s
    vy_raw = int(vy_ms * 1000)
    vz_raw = int(vz_ms * 1000)

    buf = bytearray(11)
    buf[0] = 0x7B               # 帧头
    buf[1] = cmd                 # 命令
    buf[2] = 0x00                # 保留
    buf[3] = (vx_raw >> 8) & 0xFF
    buf[4] = vx_raw & 0xFF
    buf[5] = (vy_raw >> 8) & 0xFF
    buf[6] = vy_raw & 0xFF
    buf[7] = (vz_raw >> 8) & 0xFF
    buf[8] = vz_raw & 0xFF
    buf[9] = xor_checksum(buf[:9])  # 校验和
    buf[10] = 0x7D                  # 帧尾
    return bytes(buf)
```

### 9.4 注意事项

1. **字节序**：所有多字节整数均为大端序 (Big-Endian)。
2. **IMU 数值**：发送的是原始 ADC 值，ROS 端需根据 IMU 型号 (MPU6050 或 ICM20948) 自行换算为物理单位（m/s^2 和 rad/s）。
3. **阿克曼车型**：Z 轴下发的是前轮转向角而非角速度，需要通过 `Vz_to_Akm_Angle()` 转换。
4. **连续发送**：ROS 端应以稳定频率持续发送控制帧。若超过一定时间未收到有效帧，STM32 将视为连接丢失。
5. **自动充电**：充电数据包（8 字节）紧接在主 TX 包之后发送，帧头为 `0x7C` 而非 `0x7B`。接收端应检查下一个字节来判断是否为充电包。
