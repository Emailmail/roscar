import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def xor_checksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def build_control_packet(vx_m_s: float, vy_m_s: float, vz_rad_s: float, cmd: int = 0x00) -> bytes:
    """构建 11 字节下行控制数据包。

    vx_m_s, vy_m_s: 线速度 (m/s)
    vz_rad_s:       绕 Z 轴角速度 (rad/s)
    cmd:            命令字节，0x00 = 正常控制
    """
    vx_raw = int(vx_m_s * 1000)
    vy_raw = int(vy_m_s * 1000)
    vz_raw = int(vz_rad_s * 1000)

    # clamp to int16 range
    def clamp_i16(v):
        return max(-32768, min(32767, v))

    vx_raw = clamp_i16(vx_raw)
    vy_raw = clamp_i16(vy_raw)
    vz_raw = clamp_i16(vz_raw)

    buf = bytearray(11)
    buf[0] = 0x7B
    buf[1] = cmd
    buf[2] = 0x00  # reserved
    struct.pack_into('>hhh', buf, 3, vx_raw, vy_raw, vz_raw)
    buf[9] = xor_checksum(buf[:9])
    buf[10] = 0x7D
    return bytes(buf)


class C30dCtrlNode(Node):
    def __init__(self):
        super().__init__('c30d_ctrl')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('frame_id', 'base_link')

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._publish_hz = self.get_parameter('publish_hz').value

        # latest cmd_vel, thread-safe
        self._lock = threading.Lock()
        self._latest_vx = 0.0
        self._latest_vy = 0.0
        self._latest_vz = 0.0
        self._cmd_received = False

        # serial port
        import serial
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
            )
            self.get_logger().info(f'Opened serial {self._port} @ {self._baudrate}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to open serial: {e}')
            raise

        # subscriber
        self._sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, 10
        )

        # timer — send control frame at fixed rate
        period = 1.0 / self._publish_hz
        self._timer = self.create_timer(period, self._on_timer_send)

        # stats
        self._send_count = 0
        self._timer_stat = self.create_timer(5.0, self._on_timer_stats)

        self.get_logger().info(
            f'C30D controller ready — sending at {self._publish_hz:.0f} Hz to {self._port}'
        )

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._latest_vx = msg.linear.x
            self._latest_vy = msg.linear.y
            self._latest_vz = msg.angular.z
            self._cmd_received = True

    def _on_timer_send(self):
        with self._lock:
            vx, vy, vz = self._latest_vx, self._latest_vy, self._latest_vz

        packet = build_control_packet(vx, vy, vz)
        try:
            self._ser.write(packet)
            self._send_count += 1
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def _on_timer_stats(self):
        self.get_logger().info(
            f'Sent {self._send_count} packets, '
            f'latest cmd_vel: vx={self._latest_vx:.3f}, vy={self._latest_vy:.3f}, vz={self._latest_vz:.3f}'
        )

    def destroy_node(self):
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = C30dCtrlNode()
    except Exception:
        rclpy.shutdown()
        import sys
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
