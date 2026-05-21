import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


_KEY_MAP = {
    'w': (1.0, 0.0, 0.0),       # 前进
    's': (-1.0, 0.0, 0.0),      # 后退
    'a': (0.0, 1.0, 0.0),       # 左移
    'd': (0.0, -1.0, 0.0),      # 右移
    'q': (0.0, 0.0, 1.0),       # 逆时针旋转
    'e': (0.0, 0.0, -1.0),      # 顺时针旋转
}


class KeyCtrlNode(Node):
    """键盘遥操作节点（三轮全向轮）— 读取 WASD + Q/E 并发布 /cmd_vel。

    W/S: 前进/后退 (vx)   A/D: 左移/右移 (vy)   Q/E: 旋转 (vz)
    STM32 内部完成三轮逆运动学解算，将 vx/vy/vz 映射到各轮转速。
    """

    def __init__(self):
        super().__init__('key_ctrl')

        self.declare_parameter('speed', 0.3)       # 线速度 m/s
        self.declare_parameter('turn', 1.0)         # 角速度 rad/s
        self.declare_parameter('cmd_hz', 20.0)      # 发布频率
        self.declare_parameter('stale_timeout', 0.3) # 松键后多久停车 (秒)

        self._speed = self.get_parameter('speed').value
        self._turn = self.get_parameter('turn').value
        self._cmd_hz = self.get_parameter('cmd_hz').value
        self._stale_timeout = self.get_parameter('stale_timeout').value

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._lock = threading.Lock()
        self._vx, self._vy, self._vz = 0.0, 0.0, 0.0
        self._last_key_time = self.get_clock().now()

        # keyboard thread
        self._running = threading.Event()
        self._running.set()
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        # publish timer
        period = 1.0 / self._cmd_hz
        self._timer = self.create_timer(period, self._on_timer_publish)

        self.get_logger().info(
            'Keyboard control ready — '
            'W/S: fwd/back  A/D: left/right  Q/E: rotate CCW/CW  Other: stop'
        )

    def _key_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    break
                ch = ch.lower()
                if ch == '\x03':  # Ctrl-C
                    raise KeyboardInterrupt
                with self._lock:
                    if ch in _KEY_MAP:
                        sx, sy, sz = _KEY_MAP[ch]
                        self._vx = sx * self._speed
                        self._vy = sy * self._speed
                        self._vz = sz * self._turn
                    else:
                        self._vx = self._vy = self._vz = 0.0
                    self._last_key_time = self.get_clock().now()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _on_timer_publish(self):
        with self._lock:
            vx, vy, vz = self._vx, self._vy, self._vz
            elapsed = (self.get_clock().now() - self._last_key_time).nanoseconds * 1e-9

        # stale check — auto-stop after timeout
        if elapsed > self._stale_timeout:
            vx = vy = vz = 0.0

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = vz
        self._pub.publish(msg)

    def destroy_node(self):
        self._running.clear()
        try:
            super().destroy_node()
        except Exception:
            pass


def main():
    rclpy.init()
    node = KeyCtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
