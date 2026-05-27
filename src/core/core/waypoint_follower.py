import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import TransformListener, Buffer


def make_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation = make_quaternion(yaw)
    return pose


# Waypoints for the path
# O: origin, facing +x
# A: (1.5, 0), facing +y
# B: (1.5, 8.5), facing +x
# C: (4.5, 8.5), facing -y
# D: (4.5, -0.5), facing +y
WAYPOINTS_FORWARD = [
    ("A", make_pose(1.5, 0.0, math.pi / 2)),
    ("B", make_pose(1.5, 8.5, 0.0)),
    ("C", make_pose(4.5, 8.5, -math.pi / 2)),
    ("D", make_pose(4.5, -0.5, math.pi / 2)),
]

WAYPOINTS_RETURN = [
    ("C", make_pose(4.5, 8.5, -math.pi / 2)),
    ("B", make_pose(1.5, 8.5, 0.0)),
    ("A", make_pose(1.5, 0.0, math.pi / 2)),
    ("O", make_pose(0.0, 0.0, 0.0)),
]

GOAL_TOLERANCE_XY = 0.15
GOAL_TOLERANCE_YAW_RAD = 0.3


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info("WaypointFollower ready — waiting for navigate_to_pose action server")

    def _wait_for_action(self, timeout_sec: float = 60.0) -> bool:
        if self._action_client.server_is_ready():
            return True
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        elapsed = 0.0
        while rclpy.ok() and elapsed < timeout_sec:
            if self._action_client.server_is_ready():
                return True
            time.sleep(0.5)
            elapsed += 0.5
        return self._action_client.server_is_ready()

    def _send_goal(self, name: str, pose: PoseStamped) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending goal '{name}': x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"Goal '{name}' was rejected by action server")
            return False

        self.get_logger().info(f"Goal '{name}' accepted — navigating...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f"Goal '{name}' failed (no result)")
            return False

        status = result.status
        if status == 0:
            self.get_logger().info(f"Goal '{name}' reached successfully")
            return True
        else:
            self.get_logger().warn(f"Goal '{name}' ended with status {status}")
            return False

    def run(self):
        self.get_logger().info("Waiting 25s for Nav2 initialization...")
        time.sleep(25.0)

        if not self._wait_for_action():
            self.get_logger().fatal("navigate_to_pose action server never appeared, aborting")
            return

        self.get_logger().info("=== Starting forward path: A → B → C → D ===")
        for name, pose in WAYPOINTS_FORWARD:
            if not rclpy.ok():
                return
            if not self._send_goal(name, pose):
                self.get_logger().error(f"Aborting path at '{name}'")
                return

        self.get_logger().info("=== Forward path complete! Returning: C → B → A → O ===")
        for name, pose in WAYPOINTS_RETURN:
            if not rclpy.ok():
                return
            if not self._send_goal(name, pose):
                self.get_logger().error(f"Aborting path at '{name}'")
                return

        self.get_logger().info("=== All waypoints complete! ===")


def main():
    rclpy.init()
    node = WaypointFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
