import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf2_ros


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.publish_rate = self.get_parameter('publish_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.last_pose = None
        self.last_time = None
        self.latest_odom = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'PoseToOdom started: {self.odom_frame}->{self.base_frame} '
            f'at {self.publish_rate}Hz')

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return

        now = self.get_clock().now()
        t = transform.transform.translation
        r = transform.transform.rotation

        linear_x = 0.0
        angular_z = 0.0

        if self.last_pose is not None and self.last_time is not None:
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt > 1e-6:
                dx = t.x - self.last_pose[0]
                dy = t.y - self.last_pose[1]

                curr_yaw = self._yaw_from_quaternion(r)
                prev_yaw = self.last_pose[2]

                linear_x = math.sqrt(dx * dx + dy * dy) / dt
                angular_z = self._angle_diff(curr_yaw, prev_yaw) / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = t.x
        odom.pose.pose.position.y = t.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = r
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z

        self.odom_pub.publish(odom)

        self.last_pose = (t.x, t.y, self._yaw_from_quaternion(r))
        self.last_time = now

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        return d


def main():
    rclpy.init()
    node = PoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
