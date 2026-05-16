# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# From workspace root
colcon build

# Build a single package and its deps
colcon build --packages-up-to <pkg>
```

No tests exist; lint is disabled in CMakeLists.txt. After building, source `install/setup.bash` to make packages discoverable.

## Git

- `.gitignore` must ignore: `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`, `*.egg-info/`, `.vscode/`, `.idea/`, `.claude/`
- `ldlidar_stl_ros2` is a **git submodule** — clone with `git clone --recurse-submodules`
- **Always push to a remote** — `rm -rf` + no remote + no backup = permanent data loss. extundelete is useless on a mounted ext4 partition. Recovery lesson from 2026-05-16.

## Architecture

This is a **ROS 2 Jazzy** robot project. Same sensor suite (LDRobot LD06 LiDAR + DM-Series IMU) fused through Google Cartographer. Three operating modes:

**SLAM mapping** — `src/carto/launch/create_map.launch.py`:
Sensors → Cartographer builds a new map → saves to `src/map/` as `.pbstream` + `.pgm`/`.yaml`

**Pure localization** — `src/nav/launch/localization.launch.py`:
Sensors → Cartographer loads pre-built `.pbstream` → scan-to-submap matching → publishes `map → odom → base_link` TF tree + occupancy grid on `/map`

**Autonomous navigation** — `src/nav/launch/bringup.launch.py`:
Localization + Nav2 path planning (planner + controller + BT + recovery). Uses NavFn A* planner and MPPI DiffDrive controller.

## Packages

- **`carto`** (ament_cmake) — Cartographer SLAM Lua configs + launch files. No C++ source. Installs `launch/` and `config/`.
- **`nav`** (ament_cmake) — Localization + Nav2 path planning. No C++ source. Installs `launch/` and `config/`.
  - `localization.launch.py` — Cartographer pure localization
  - `navigation.launch.py` — Nav2 path planning stack
  - `bringup.launch.py` — Combines both for full autonomous nav
  - `nav2_params.yaml` — Full Nav2 configuration
- **`dm_imu`** (ament_python) — DM-series IMU driver (`/dev/ttyACM0`, 921600 baud). Publishes `/imu/data`, `/imu/rpy`, `/imu/pose`. CRC-16 frame validation.
- **`ldlidar_stl_ros2`** (ament_cmake) — LD06 LiDAR driver. Publishes `/scan`. Standalone clone.
- **`stm32_bridge`** (ament_python) — STM32 serial bridge + keyboard teleop. Subscribes `/cmd_vel` and forwards 11-byte binary frames to STM32 over UART (Type-C USB). Follows `dm_imu` pattern (pyserial, entry_points, config YAML).
- **`src/map/`** — Static map artifacts. Not a package.

## TF tree

```
map → odom → base_link → base_laser  (Z=0.18m for LD06)
                       → imu_link    (identity)
```

`map → odom` from Cartographer (or AMCL). Static transforms in sensor launch files.

## Key conventions

- ROS 2 Jazzy, C++14, Python 3.12
- Frame IDs: `base_link` (robot origin), `base_laser`, `imu_link`
- Cartographer configs in `src/carto/config/` (`.lua`), sensor params in each package's `config/` (`.yaml`)
- Cartographer remaps: `imu` → `imu/data`, `echoes` → `scan`
- Hardcoded path: `MAP_DIR = '/home/relog/roscar/src/map'` in localization.launch.py
- Keyboard input: **must open `/dev/tty` directly** (`os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)`), never use `sys.stdin` (ROS launch gives a pipe).
- Serial: IMU on `/dev/ttyACM0`, LiDAR on `/dev/ttyUSB0`, STM32 on `/dev/ttySTM32` (actual: `/dev/ttyACM1` or `/dev/ttyUSB1` via Type-C)

---

## Nav2 path planning (detailed reference)

### Data flow

```
occupancy_grid_node → /map
                           ↓
                    global_costmap (static_layer + obstacle_layer + inflation_layer)
                           ↓
bt_navigator → planner_server (NavFn A*) → smoother_server → controller_server (MPPI DiffDrive)
                                                                       ↓
                                                               local_costmap (obstacle_layer + inflation_layer)
                                                                       ↓
                                             cmd_vel_nav → velocity_smoother (OPEN_LOOP) → cmd_vel → stm32_bridge → STM32
```

### cmd_vel pipeline (critical)

```
controller_server → cmd_vel_nav → velocity_smoother → cmd_vel → stm32_bridge → STM32
```

controller_server, behavior_server, and velocity_smoother all need `cmd_vel` remapped:
```python
controller_server: remappings + [('cmd_vel', 'cmd_vel_nav')]
behavior_server:   remappings + [('cmd_vel', 'cmd_vel_nav')]
velocity_smoother: remappings + [('cmd_vel', 'cmd_vel_nav')]
```

`velocity_smoother` outputs the final `cmd_vel` (geometry_msgs/Twist). `stm32_bridge` subscribes to `cmd_vel` with best-effort QoS (keep_last depth=1), converts to 11-byte STM32 frames, sends over UART.

### Lifecycle management (critical)

- Node name: `lifecycle_manager_navigation`
- `node_names` and `autostart` **MUST be passed as direct flat parameter dicts** in the launch file, NOT from YAML:
  ```python
  lifecycle_manager = Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_navigation',
      parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
      output='screen',
  )
  ```
- **Why**: The lifecycle_manager constructor hardcodes the node name as "lifecycle_manager", but the launch names it "lifecycle_manager_navigation". The YAML key `lifecycle_manager` won't match → parameters silently not loaded → nodes never activate.
- **Why this matters**: If ANY node fails to configure (FATAL error), the lifecycle manager **aborts the entire bringup** — ALL nodes stay unconfigured or inactive forever. No path planning, no error messages explaining why.

### Costmap rules

- `/map` topic source: `cartographer_occupancy_grid_node` (NOT `nav2_map_server`). These two **conflict** on `/map`. Only launch one.
- Global costmap: `static_layer + obstacle_layer + inflation_layer`, frame `map`, not rolling
- Local costmap: `obstacle_layer + inflation_layer`, frame `odom`, rolling 4m×4m
- `robot_radius: 0.3` for both. No `footprint` polygon.
- `track_unknown_space: true`, `always_send_full_costmap: True`
- Global obstacle_layer: `obstacle_max_range: 6.0`, `raytrace_max_range: 7.0`
- Local obstacle_layer: `obstacle_max_range: 4.0`, `raytrace_max_range: 5.0`

### Planner

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

Path smoothing: `nav2_smoother::SimpleSmoother`

### MPPI controller + critic weights

```yaml
motion_model: "DiffDrive"
vx_max: 0.5, vx_min: -0.35, vy_max: 0.0, wz_max: 1.9
```

Critic weights — CostCritic MUST dominate path-following critics:

| Critic | Weight | Purpose |
|--------|--------|---------|
| CostCritic | **20.0** | Obstacle avoidance (highest) |
| PathAlignCritic | 8.0 | Align trajectory with global path |
| PathFollowCritic | 5.0 | Progress along path |
| GoalCritic | 5.0 | Approach goal |
| PreferForwardCritic | 5.0 | Prefer forward motion |
| ConstraintCritic | 4.0 | Velocity/accel constraints |
| GoalAngleCritic | 3.0 | Face goal direction |
| PathAngleCritic | 2.0 | Match path orientation |

**Previous failure**: CostCritic=8.0 and PathAlignCritic=14.0 (sum of path-following=19.0 vs CostCritic=8.0) caused the robot to drive straight through obstacles because following the path was weighted more heavily than avoiding obstacles.

### consider_footprint (CRITICAL — caused full system failure)

**MUST stay `false`**. Setting to `true` causes:

1. MPPI CostCritic requires a polygon `footprint` parameter in the costmap
2. Our costmap only has `robot_radius: 0.3`, no `footprint` polygon
3. controller_server FATAL error during `on_configure`: *"Considering footprint in collision checking but no robot footprint provided in the costmap"*
4. lifecycle_manager sees controller_server configure failure → **aborts entire bringup**
5. ALL Nav2 nodes (planner, controller, BT, behavior, smoother, velocity_smoother) stay in unconfigured/inactive state
6. **No path planning, no action servers, no error visible to user** — completely silent failure

With `consider_footprint: false`, CostCritic uses costmap values (already inflated from `robot_radius`) — works correctly.

### Behavior server

```yaml
behavior_plugins: ["spin", "backup", "wait"]   # recovery behaviors
local_frame: odom
global_frame: map
robot_base_frame: base_link
```

### Velocity smoother

```yaml
feedback: "OPEN_LOOP"   # no odom topic feedback (Cartographer only publishes TF, not /odom topic)
max_velocity: [0.5, 0.0, 2.0]
min_velocity: [-0.5, 0.0, -2.0]
```

### Nav2 package.xml dependencies

```xml
<depend>nav2_amcl</depend>
<depend>nav2_behaviors</depend>
<depend>nav2_bt_navigator</depend>
<depend>nav2_controller</depend>
<depend>nav2_costmap_2d</depend>
<depend>nav2_lifecycle_manager</depend>
<depend>nav2_map_server</depend>
<depend>nav2_mppi_controller</depend>
<depend>nav2_msgs</depend>
<depend>nav2_navfn_planner</depend>
<depend>nav2_planner</depend>
<depend>nav2_smoother</depend>
<depend>nav2_velocity_smoother</depend>
```

---

## Costmap parameter loading gotcha

Costmap params are defined as separate top-level YAML keys (`global_costmap`, `local_costmap`) in the default nav2_bringup style. ROS 2's `parameters=[yaml_file]` distributes params by node name — these keys match no launched node. The costmap sub-nodes (created internally by planner_server / controller_server) inherit params through `NodeOptions` copy from their parent. This works in nav2_bringup but be aware: `ros2 param get /planner_server global_costmap.track_unknown_space` may show "Parameter not set" even though the costmap IS using it via defaults. Verify with `ros2 param get /planner_server global_costmap.plugins` to confirm loading.

---

## STM32 serial protocol (stm32_bridge)

### Frame format

11 bytes fixed length, UART3 from ROS → STM32:

```
Byte 0:    0x7B  (frame header)
Byte 1:    Mode (0x00=normal velocity control)
Byte 2:    Reserved (0x00, but included in checksum)
Byte 3-4:  X speed  — signed int16 big-endian, mm/s (linear.x × 1000)
Byte 5-6:  Y speed  — signed int16 big-endian, mm/s (linear.y × 1000)
Byte 7-8:  Z speed  — signed int16 big-endian, mm/s (angular.z × 1000)
Byte 9:    XOR checksum of bytes 0-8
Byte 10:   0x7D  (frame tail)
```

Mode values:
- `0x00` — Normal velocity control (disables auto recharge)
- `0x01` / `0x02` — Auto recharge navigation
- `0x03` — Infrared docking speed set
- `0xFF` — IP frame (bytes 3-6 = IP octets)

Speed encoding example: X=0.5 m/s → 500 mm/s → 0x01F4 → high=0x01, low=0xF4. Negative values use signed two's complement. Z axis is yaw angular velocity (rad/s) with the same ×1000 encoding.

### stm32_bridge node

Package `stm32_bridge` (ament_python). Follows `dm_imu` structure.

**Parameters:**

| Param | Default | Description |
|-------|---------|-------------|
| `port` | `/dev/ttySTM32` | STM32 serial device |
| `baud` | `115200` | Baud rate (must match STM32 firmware) |
| `mode` | `0` | Control mode (see above) |
| `tx_rate_hz` | `20.0` | Send frequency |
| `timeout_ms` | `500` | Watchdog: auto-send zero if no cmd_vel |

**Key behaviors:**
- Subscribes to `cmd_vel` with best-effort QoS (keep_last depth=1) — avoids queuing stale commands
- Timer at `tx_rate_hz` pulls latest cmd_vel and sends frame
- Watchdog: if no cmd_vel message for `timeout_ms`, sends zero-velocity frame (safety brake)
- `destroy_node()` sends zero-velocity before closing serial

**Launch:**
```bash
ros2 launch stm32_bridge stm32_bridge.launch.py port:=/dev/ttyACM1
```

### keyboard_teleop node

Also in `stm32_bridge` package. Simple press-to-go / release-to-stop — no state machine, no tap-vs-hold detection.

| Key | Action | cmd_vel output |
|-----|--------|---------------|
| W | Forward | linear.x = +speed |
| S | Backward | linear.x = -speed |
| A | Left (strafe) | linear.y = +speed |
| D | Right (strafe) | linear.y = -speed |
| Q | Rotate CCW | angular.z = +speed |
| E | Rotate CW | angular.z = -speed |
| Space | Emergency stop | All zeros |

**Parameters:** `linear_speed` (default 0.5 m/s), `angular_speed` (default 1.0 rad/s), `idle_timeout` (0.3s auto-stop on key release).

**Implementation rules (CRITICAL — learned from failures):**

1. **Always open `/dev/tty` directly, NEVER use `sys.stdin`**. ROS 2 launch connects stdin to a pipe, not a TTY → `termios.tcgetattr(sys.stdin)` fails with `Inappropriate ioctl for device`. Use `os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)` instead.

2. **Do NOT use terminal prefix** (`xterm -e`, `gnome-terminal --`, etc.) in the launch file. If the terminal emulator isn't installed (xterm) or doesn't forward stdin properly, the node breaks silently. Run both keyboard and bridge nodes in the same terminal — raw mode naturally captures key events.

3. **Key repeat is handled by the kernel, not the terminal**. In raw mode (`tty.setraw`), holding a key still produces repeated characters. The `idle_timeout` detects release (no more characters arrive). No need for explicit press/release detection.

4. **Logic is: received key → SET velocity (not add) → publish; no key for idle_timeout → SET velocity to 0**. Each key event sets `_vx/_vy/_vz` to a fixed value (e.g., `lx * self.lin_speed`), never accumulates.

**Combined launch** (keyboard_teleop + stm32_bridge):
```bash
ros2 launch stm32_bridge keyboard_control.launch.py port:=/dev/ttyACM1
```

### STM32 firmware debugging (critical)

**ROS side sends absolute speed targets, NOT deltas.** If the robot accelerates when holding W (speed accumulates), the bug is in the STM32 firmware — it's doing `current_speed += received_speed` instead of `current_speed = received_speed`.

Quick verification: run `ros2 topic echo /cmd_vel` while pressing W. If `linear.x` stays at `0.5` (constant), the ROS side is correct and the STM32 firmware needs fixing.

### Differential drive chassis (non-holonomic)

Our chassis is a balancing-style differential drive — **cannot produce Y-axis (lateral) velocity**. Nav2 config must reflect this:

| Param | Value | Why |
|-------|-------|-----|
| `motion_model` | `"DiffDrive"` | MPPI uses correct kinematics internally |
| `vy_max` | `0.0` | Prevent MPPI from outputting lateral velocity |
| `vy_std` | `0.0` | Don't waste computation sampling Y-axis noise |
| velocity_smoother max/min Y | `0.0` | Prevent smoother from adding lateral component |

### Debugging tips

- **`ros2 topic echo /cmd_vel`** — first step when robot movement seems wrong. Confirm the ROS side publishes expected values before suspecting the firmware.
- **`ros2 param get /planner_server global_costmap.plugins`** — verify costmap params are actually loaded (direct `ros2 param get` on individual keys may show "not set" — see Costmap parameter loading gotcha above).

---

## Simulation (planned, not implemented)

- ROS 2 Jazzy has `nav2-minimal-tb3-sim` + `ros-gz-sim` (Gazebo Harmonic)
- Standard launch: `ros2 launch nav2_bringup tb3_simulation_launch.py` (Gazebo + TB3 + AMCL + Nav2 + Rviz)
- TB3 frame differences vs our robot:
  - LiDAR frame: `base_scan` vs our `base_laser`
  - IMU topic: `/imu` vs our `/imu/data`
  - Has odometry topic `/odom` (simulation) vs our Cartographer-only TF
  - LiDAR range: ~3.5m vs our LD06 12m
- To test our own Cartographer + Nav2 stack in sim:
  - Cartographer config needs `use_odometry = true` and topic remaps
  - Need SLAM mode first (no pre-built sim map)
  - Later save pbstream and switch to pure localization
- Two approaches:
  - A: Use default nav2_bringup simulation (quick, tests Nav2 flow)
  - B: Custom sim launch with our Cartographer + our Nav2 params (tests full stack)
