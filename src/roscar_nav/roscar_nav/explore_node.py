import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion

# ANSI escapes for message text
C_RST = "\033[0m"
C_YEL = "\033[33m"
C_GRN = "\033[32m"
C_CYN = "\033[36m"


class ExploreNode(Node):
    """Frontier-based autonomous exploration.

    Subscribes to Cartographer /map and /odom (from pose_to_odom),
    publishes /goal_pose for bt_navigator to consume.
    """

    def __init__(self):
        super().__init__("explore_node")

        self.declare_parameter("explore_rate", 2.0)
        self.declare_parameter("min_frontier_size", 10)
        self.declare_parameter("goal_reached_dist", 0.5)
        self.declare_parameter("goal_timeout", 30.0)
        self.declare_parameter("start_delay", 5.0)
        self.declare_parameter("min_known_ratio", 0.01)
        self.declare_parameter("blind_dist", 2.0)
        self.declare_parameter("max_blind_attempts", 4)

        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self._latest_map = None
        self._robot_pose = None  # (x, y)
        self._robot_yaw = 0.0
        self._current_goal = None  # (x, y)
        self._goal_is_blind = False
        self._goal_start_time = None

        self._start_time = self.get_clock().now()
        self._started = False
        self._completed = False
        self._blind_attempts = 0

        rate = self.get_parameter("explore_rate").value
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info(
            f"{C_CYN}Explore node ready, will start after "
            f"{self.get_parameter('start_delay').value:.0f}s delay{C_RST}"
        )

    # ── callbacks ───────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _odom_cb(self, msg: Odometry):
        self._robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        q = msg.pose.pose.orientation
        self._robot_yaw = self._yaw_from_quat(q)

    # ── main loop ───────────────────────────────────────────────────

    def _timer_cb(self):
        if not self._started:
            now = self.get_clock().now()
            delay_ns = self.get_parameter("start_delay").value * 1e9
            if (now - self._start_time).nanoseconds < delay_ns:
                return
            self._started = True
            self.get_logger().info(f"{C_CYN}Exploration started{C_RST}")

        if self._latest_map is None or self._robot_pose is None:
            return

        if self._completed:
            return

        # Check current goal status
        if self._current_goal is not None:
            elapsed_s = (
                self.get_clock().now() - self._goal_start_time
            ).nanoseconds / 1e9
            dist = self._dist(self._robot_pose, self._current_goal)

            if dist < self.get_parameter("goal_reached_dist").value:
                if self._goal_is_blind:
                    self.get_logger().info(f"{C_CYN}Blind goal reached, checking frontiers...{C_RST}")
                else:
                    self.get_logger().info(f"{C_GRN}Goal reached{C_RST}")
                self._current_goal = None
                self._goal_is_blind = False
            elif elapsed_s > self.get_parameter("goal_timeout").value:
                self.get_logger().info(f"{C_YEL}Goal timeout, picking next target{C_RST}")
                self._current_goal = None
                self._goal_is_blind = False
            else:
                return  # still working

        # Find next goal: frontier first, blind if map too small
        frontiers = self._find_frontiers(self._latest_map)
        if frontiers:
            self._blind_attempts = 0
            best = self._select_best(frontiers, self._robot_pose)
            self._publish_goal(best[0], best[1], best[2], blind=False)
            return

        # No frontiers — check if map has enough content
        known_ratio = self._known_ratio(self._latest_map)
        if known_ratio < self.get_parameter("min_known_ratio").value:
            self.get_logger().info(
                f"{C_YEL}Map too sparse ({known_ratio:.3%} known), waiting...{C_RST}"
            )
            return

        # Map has content but no frontiers — try blind goal to expand
        max_blind = self.get_parameter("max_blind_attempts").value
        if self._blind_attempts < max_blind:
            self._blind_attempts += 1
            self._send_blind_goal()
        else:
            self.get_logger().info(
                f"{C_GRN}No frontiers after {max_blind} blind attempts "
                f"— exploration complete ({known_ratio:.1%} known){C_RST}"
            )
            self._completed = True

    # ── blind goal ─────────────────────────────────────────────────

    def _send_blind_goal(self):
        dist = self.get_parameter("blind_dist").value
        # rotate direction for each attempt: 0°, 120°, -120°, 60°, ...
        offsets = [0.0, math.pi * 2 / 3, -math.pi * 2 / 3, math.pi / 3]
        idx = (self._blind_attempts - 1) % len(offsets)
        angle = self._robot_yaw + offsets[idx]

        gx = self._robot_pose[0] + dist * math.cos(angle)
        gy = self._robot_pose[1] + dist * math.sin(angle)
        self._publish_goal(gx, gy, 0, blind=True)
        self.get_logger().info(
            f"{C_CYN}Blind goal #{self._blind_attempts}: ({gx:.2f}, {gy:.2f}) "
            f"@{math.degrees(offsets[idx]):.0f}° offset{C_RST}"
        )

    # ── goal publish helper ─────────────────────────────────────────

    def _publish_goal(self, x, y, size, blind=False):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)

        self._current_goal = (x, y)
        self._goal_is_blind = blind
        self._goal_start_time = self.get_clock().now()
        if not blind:
            self.get_logger().info(
                f"{C_CYN}New goal: ({x:.2f}, {y:.2f})  size={size}{C_RST}"
            )

    # ── frontier detection ──────────────────────────────────────────

    def _find_frontiers(self, map_msg: OccupancyGrid):
        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        data = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        free = data == 0
        unknown = data == -1
        frontier_mask = np.zeros((h, w), dtype=bool)

        # free cells with at least one unknown neighbour (4-connectivity)
        frontier_mask[1:, :] |= free[:-1, :] & unknown[1:, :]
        frontier_mask[:-1, :] |= free[1:, :] & unknown[:-1, :]
        frontier_mask[:, 1:] |= free[:, :-1] & unknown[:, 1:]
        frontier_mask[:, :-1] |= free[:, 1:] & unknown[:, :-1]

        clusters = self._cluster_bfs(frontier_mask)

        min_size = self.get_parameter("min_frontier_size").value
        frontiers = []
        for cells in clusters:
            if len(cells) < min_size:
                continue
            cy = sum(c[0] for c in cells) / len(cells)
            cx = sum(c[1] for c in cells) / len(cells)
            frontiers.append((cx * res + ox, cy * res + oy, len(cells)))

        return frontiers

    @staticmethod
    def _known_ratio(map_msg: OccupancyGrid):
        data = np.array(map_msg.data, dtype=np.int8)
        known = (data == 0) | (data == 100)
        return int(np.sum(known)) / len(data) if len(data) > 0 else 0.0

    def _cluster_bfs(self, mask: np.ndarray):
        h, w = mask.shape
        visited = np.zeros((h, w), dtype=bool)
        clusters = []

        for i in range(h):
            for j in range(w):
                if not mask[i, j] or visited[i, j]:
                    continue
                cluster = []
                queue = [(i, j)]
                visited[i, j] = True
                while queue:
                    ci, cj = queue.pop(0)
                    cluster.append((ci, cj))
                    for di in (-1, 0, 1):
                        for dj in (-1, 0, 1):
                            if di == 0 and dj == 0:
                                continue
                            ni, nj = ci + di, cj + dj
                            if 0 <= ni < h and 0 <= nj < w:
                                if mask[ni, nj] and not visited[ni, nj]:
                                    visited[ni, nj] = True
                                    queue.append((ni, nj))
                clusters.append(cluster)

        return clusters

    # ── scoring ─────────────────────────────────────────────────────

    def _select_best(self, frontiers, robot_pose):
        best = None
        best_score = -float("inf")
        rx, ry = robot_pose
        for fx, fy, size in frontiers:
            dist = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)
            score = size - dist * 0.5
            if score > best_score:
                best_score = score
                best = (fx, fy, size)
        return best

    @staticmethod
    def _yaw_from_quat(q: Quaternion):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _dist(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def main():
    rclpy.init()
    node = ExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
