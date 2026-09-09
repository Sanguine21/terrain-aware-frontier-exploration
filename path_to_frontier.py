#!/usr/bin/env python3
"""
Path Planner to Best Frontier
-------------------------------
Combines frontier detection (with terrain-cost ranking) with A* path
planning on the live occupancy grid, computing a route from the robot's
current position to the best-ranked frontier. Publishes the path for
visualization in rviz2 as a nav_msgs/Path.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
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


class PathToFrontier(Node):
    def __init__(self):
        super().__init__('path_to_frontier')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.get_logger().info('Path-to-frontier planner started.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        frontier_cells = self.find_frontier_cells(grid)
        if not frontier_cells:
            self.get_logger().info('No frontiers found yet.')
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
        self.get_logger().info(
            f"Best frontier: x={best['x']:.2f}, y={best['y']:.2f}, score={best['score']:.2f}")

        self.publish_markers(scored)

        robot_col = int((self.robot_x - origin_x) / resolution)
        robot_row = int((self.robot_y - origin_y) / resolution)
        start = (robot_row, robot_col)
        goal = (best['row'], best['col'])

        path_cells = self.astar(grid, start, goal)

        if path_cells is None:
            self.get_logger().info('No path found to best frontier (blocked or unreachable).')
            return

        self.publish_path(path_cells, origin_x, origin_y, resolution)
        self.get_logger().info(f'Path found: {len(path_cells)} waypoints.')

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
        """Standard grid A*. Only confirmed free (0) cells are treated
        as traversable; occupied (100) and unknown (-1) cells are
        treated as obstacles for safe planning."""
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
    node = PathToFrontier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
