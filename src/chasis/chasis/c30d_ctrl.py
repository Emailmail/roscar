import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros


def xor_checksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def build_control_packet(vx_m_s: float, vy_m_s: float, vz_rad_s: float, cmd: int = 0x00) -> bytes:
    """构建 11 字节下行控制数据包（三轮全向轮）。

    vx_m_s, vy_m_s: 线速度 (m/s)，vx=前进，vy=左移
    vz_rad_s:       绕 Z 轴角速度 (rad/s)，逆时针为正
    cmd:            命令字节，0x00 = 正常控制

    STM32 固件接收 vx/vy/vz 后内部完成三轮逆运动学解算。
    """
    vx_raw = int(vx_m_s * 1000)
    vy_raw = int(vy_m_s * 1000)
    vz_raw = int(vz_rad_s * 1000)

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

        # latest cmd_vel + encoder speeds, thread-safe
        self._lock = threading.Lock()
        self._latest_vx = 0.0
        self._latest_vy = 0.0
        self._latest_vz = 0.0
        self._cmd_received = False

        # STM32 encoder-reported speeds (from uplink)
        self._enc_vx = 0.0
        self._enc_vy = 0.0
        self._enc_vz = 0.0

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

        # odometry publisher — pose from Cartographer TF, twist from STM32 encoders
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._odom_timer = self.create_timer(0.05, self._publish_odom)

        # background thread: read STM32 uplink telemetry
        self._running = True
        self._uplink_thread = threading.Thread(target=self._read_uplink, daemon=True)
        self._uplink_thread.start()

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

    def _read_uplink(self):
        """Background thread: read 24-byte uplink packets from STM32.

        Protocol (ref src/doc/ros_to_c30d.md Section 3):
          24 bytes: 0x7B | Flag | X_speed(2) | Y_speed(2) | Z_speed(2) |
                    Accel(6) | Gyro(6) | Voltage(2) | Checksum | 0x7D
          Speeds are int16 big-endian, units: X/Y in mm/s, Z in mm/s (convert to rad/s).
        """
        buf = bytearray()
        while self._running:
            try:
                if self._ser.in_waiting > 0:
                    chunk = self._ser.read(self._ser.in_waiting)
                    buf.extend(chunk)
                    while len(buf) >= 24:
                        # scan for valid frame header
                        if buf[0] != 0x7B:
                            buf.pop(0)
                            continue
                        if buf[23] != 0x7D:
                            buf.pop(0)
                            continue
                        if xor_checksum(buf[:22]) != buf[22]:
                            buf.pop(0)
                            continue
                        # parse speeds: int16 big-endian, mm/s → m/s (or rad/s for Z)
                        vx = int.from_bytes(buf[2:4], 'big', signed=True) / 1000.0
                        vy = int.from_bytes(buf[4:6], 'big', signed=True) / 1000.0
                        vz = int.from_bytes(buf[6:8], 'big', signed=True) / 1000.0
                        with self._lock:
                            self._enc_vx = vx
                            self._enc_vy = vy
                            self._enc_vz = vz
                        buf = buf[24:]
                else:
                    time.sleep(0.002)
            except Exception:
                time.sleep(0.01)

    def _publish_odom(self):
        """Publish Odometry: pose from Cartographer TF, twist from STM32 encoders."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        with self._lock:
            enc_vx = self._enc_vx
            enc_vy = self._enc_vy
            enc_vz = self._enc_vz

        now = self.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = transform.transform.translation.x
        odom.pose.pose.position.y = transform.transform.translation.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x = enc_vx
        odom.twist.twist.linear.y = enc_vy
        odom.twist.twist.angular.z = enc_vz
        self._odom_pub.publish(odom)

    def _on_timer_stats(self):
        self.get_logger().info(
            f'Sent {self._send_count} packets, '
            f'cmd_vel: vx={self._latest_vx:.3f} vy={self._latest_vy:.3f} vz={self._latest_vz:.3f} | '
            f'enc: vx={self._enc_vx:.3f} vy={self._enc_vy:.3f} vz={self._enc_vz:.3f}'
        )

    def destroy_node(self):
        self._running = False
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
