#!/usr/bin/env python3
"""
Autonomous Terrain-Aware Explorer
------------------------------------
Full closed loop: detect frontiers -> rank by terrain cost -> plan A*
path to best frontier -> drive the robot along that path automatically,
with no manual control.

Debugging note: an early version reported "reached frontier goal"
almost instantly with no real movement. Root cause: WAYPOINT_TOLERANCE
was larger than the spacing between raw A* grid waypoints, so early
waypoints were satisfied without genuine travel. Fixed by downsampling
the path to coarser waypoint spacing and tightening the tolerance,
verified with explicit distance-to-target logging showing real,
continuously decreasing distance as the robot moved.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import heapq


TERRAIN_COST_ZONES = [
    (2.0, 0.0, 1.5, 5.0),
    (-2.0, 1.5, 1.3, 4.0),
    (0.0, 3.0, 2.5, 3.0),
    (3.0, -2.0, 1.8, 2.0),
]
COST_WEIGHT = 0.5
WAYPOINT_TOLERANCE = 0.15   # meters - tightened from 0.3 to avoid skipping waypoints
LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.6


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.current_path = []
        self.waypoint_index = 0

        # Drive loop runs independently at 10Hz, using whatever path is current
        self.timer = self.create_timer(0.1, self.drive_loop)

        self.get_logger().info('Autonomous explorer started.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        frontier_cells = self.find_frontier_cells(grid)
        if not frontier_cells:
            return

        clusters = self.cluster_frontiers(frontier_cells)
        scored = []
        for cluster in clusters:
            avg_row = sum(c[0] for c in cluster) / len(cluster)
            avg_col = sum(c[1] for c in cluster) / len(cluster)
            wx = origin_x + (avg_col * resolution)
            wy = origin_y + (avg_row * resolution)
            info_gain = len(cluster)
            cost = self.compute_terrain_cost(wx, wy)
            score = info_gain - (COST_WEIGHT * cost * info_gain)
            scored.append({'row': int(avg_row), 'col': int(avg_col),
                            'x': wx, 'y': wy, 'score': score})

        scored.sort(key=lambda f: f['score'], reverse=True)
        best = scored[0]
        self.publish_markers(scored)

        robot_col = int((self.robot_x - origin_x) / resolution)
        robot_row = int((self.robot_y - origin_y) / resolution)
        start = (robot_row, robot_col)
        goal = (best['row'], best['col'])

        path_cells = self.astar(grid, start, goal)
        if path_cells is None:
            return

        # Downsample so consecutive waypoints are spaced further apart
        # than WAYPOINT_TOLERANCE, otherwise several waypoints near the
        # start get satisfied instantly without real movement.
        all_waypoints = [
            (origin_x + c * resolution, origin_y + r * resolution)
            for r, c in path_cells
        ]
        step = max(1, int(0.3 / resolution))
        downsampled = all_waypoints[::step]
        if downsampled[-1] != all_waypoints[-1]:
            downsampled.append(all_waypoints[-1])

        self.current_path = downsampled
        self.waypoint_index = 0

        self.get_logger().info(
            f'New path: {len(self.current_path)} waypoints, '
            f'first waypoint at {self.current_path[0]}, robot at ({self.robot_x:.2f}, {self.robot_y:.2f})')

        self.publish_path(path_cells, origin_x, origin_y, resolution)

    def drive_loop(self):
        """Runs at 10Hz. Steers toward the current waypoint, advancing
        through the path as each waypoint is reached."""
        if not self.current_path or self.waypoint_index >= len(self.current_path):
            self.stop_robot()
            return

        target_x, target_y = self.current_path[self.waypoint_index]
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.hypot(dx, dy)

        if distance < WAYPOINT_TOLERANCE:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.current_path):
                self.get_logger().info('Reached frontier goal!')
                self.stop_robot()
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.robot_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        cmd = Twist()
        if abs(angle_diff) > 0.3:
            cmd.angular.z = ANGULAR_SPEED if angle_diff > 0 else -ANGULAR_SPEED
        else:
            cmd.linear.x = LINEAR_SPEED
            cmd.angular.z = angle_diff * 1.5

        self.get_logger().info(
            f'Driving: dist={distance:.2f}m, linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def compute_terrain_cost(self, x, y):
        total = 0.0
        for zx, zy, radius, cost in TERRAIN_COST_ZONES:
            d = math.hypot(x - zx, y - zy)
            if d < radius:
                total += cost * (1.0 - d / radius)
        return total

    def find_frontier_cells(self, grid):
        height, width = grid.shape
        cells = []
        for r in range(1, height - 1):
            for c in range(1, width - 1):
                if grid[r, c] != 0:
                    continue
                neigh = [grid[r-1, c], grid[r+1, c], grid[r, c-1], grid[r, c+1]]
                if -1 in neigh:
                    cells.append((r, c))
        return cells

    def cluster_frontiers(self, frontier_cells, max_distance=3):
        cells = set(frontier_cells)
        visited = set()
        clusters = []
        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = []
            queue = [cell]
            visited.add(cell)
            while queue:
                cur = queue.pop()
                cluster.append(cur)
                r, c = cur
                for dr in range(-max_distance, max_distance + 1):
                    for dc in range(-max_distance, max_distance + 1):
                        n = (r + dr, c + dc)
                        if n in cells and n not in visited:
                            visited.add(n)
                            queue.append(n)
            if len(cluster) >= 3:
                clusters.append(cluster)
        return clusters

    def astar(self, grid, start, goal):
        height, width = grid.shape

        def passable(cell):
            r, c = cell
            if not (0 <= r < height and 0 <= c < width):
                return False
            return grid[r, c] == 0

        def neighbors(cell):
            r, c = cell
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nxt = (r+dr, c+dc)
                if passable(nxt):
                    yield nxt

        def heuristic(a, b):
            return math.hypot(a[0]-b[0], a[1]-b[1])

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal or heuristic(current, goal) < 2:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for nxt in neighbors(current):
                step = math.hypot(nxt[0]-current[0], nxt[1]-current[1])
                tentative = g_score[current] + step
                if nxt not in g_score or tentative < g_score[nxt]:
                    g_score[nxt] = tentative
                    priority = tentative + heuristic(nxt, goal)
                    heapq.heappush(open_set, (priority, nxt))
                    came_from[nxt] = current

        return None

    def publish_path(self, path_cells, origin_x, origin_y, resolution):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for row, col in path_cells:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = origin_x + col * resolution
            pose.pose.position.y = origin_y + row * resolution
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def publish_markers(self, scored):
        marker_array = MarkerArray()
        for i, f in enumerate(scored):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = f['x']
            marker.pose.position.y = f['y']
            marker.pose.position.z = 0.2
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35
            if i == 0:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
