#!/usr/bin/env python3
"""
Frontier Detector with Terrain-Cost Weighting
-----------------------------------------------
Reads the live SLAM map (/map), finds frontier cells (the boundary
between explored free space and unknown space), clusters them into
candidate exploration targets, and ranks them using:

    score = information_gain - (cost_weight * terrain_cost * information_gain)

Terrain cost zones are manually defined based on the known obstacle
positions in my_disaster_world.world. Publishes ranked frontiers as
markers to rviz2 (green = best, red = others).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


# Terrain cost zones: (x, y, radius, cost) - matches obstacle positions
# in my_disaster_world.world
TERRAIN_COST_ZONES = [
    (2.0, 0.0, 1.5, 5.0),    # rubble_block_1 - high cost
    (-2.0, 1.5, 1.3, 4.0),   # rubble_block_2 - high cost
    (0.0, 3.0, 2.5, 3.0),    # wall_segment_1 - medium cost
    (3.0, -2.0, 1.8, 2.0),   # ramp_1 - lower cost (traversable slope)
]

COST_WEIGHT = 0.5  # how strongly terrain cost penalizes the score


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 10)
        self.get_logger().info('Terrain-weighted frontier detector started.')

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

        scored_frontiers = []
        for cluster in clusters:
            avg_row = sum(c[0] for c in cluster) / len(cluster)
            avg_col = sum(c[1] for c in cluster) / len(cluster)
            world_x = origin_x + (avg_col * resolution)
            world_y = origin_y + (avg_row * resolution)
            info_gain = len(cluster)

            terrain_cost = self.compute_terrain_cost(world_x, world_y)
            score = info_gain - (COST_WEIGHT * terrain_cost * info_gain)

            scored_frontiers.append({
                'x': world_x, 'y': world_y,
                'info_gain': info_gain,
                'terrain_cost': terrain_cost,
                'score': score
            })

        scored_frontiers.sort(key=lambda f: f['score'], reverse=True)

        self.get_logger().info(f'--- {len(scored_frontiers)} frontiers ranked ---')
        for i, f in enumerate(scored_frontiers):
            self.get_logger().info(
                f"  #{i+1}: x={f['x']:.2f}, y={f['y']:.2f}, "
                f"info_gain={f['info_gain']}, terrain_cost={f['terrain_cost']:.2f}, "
                f"score={f['score']:.2f}"
            )

        self.publish_markers(scored_frontiers)

    def compute_terrain_cost(self, x, y):
        """Returns a cost value based on proximity to known high-cost
        terrain zones. Closer to a costly obstacle = higher cost."""
        total_cost = 0.0
        for zone_x, zone_y, radius, cost in TERRAIN_COST_ZONES:
            distance = math.hypot(x - zone_x, y - zone_y)
            if distance < radius:
                proximity_factor = 1.0 - (distance / radius)
                total_cost += cost * proximity_factor
        return total_cost

    def find_frontier_cells(self, grid):
        """A frontier cell is a FREE cell (0) with at least one
        neighboring UNKNOWN cell (-1)."""
        height, width = grid.shape
        frontier_cells = []
        for row in range(1, height - 1):
            for col in range(1, width - 1):
                if grid[row, col] != 0:
                    continue
                neighbors = [
                    grid[row - 1, col], grid[row + 1, col],
                    grid[row, col - 1], grid[row, col + 1]
                ]
                if -1 in neighbors:
                    frontier_cells.append((row, col))
        return frontier_cells

    def cluster_frontiers(self, frontier_cells, max_distance=3):
        """Groups nearby frontier cells into clusters using
        distance-based BFS grouping, so we get a few meaningful
        exploration targets instead of hundreds of boundary cells."""
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
                current = queue.pop()
                cluster.append(current)
                r, c = current
                for dr in range(-max_distance, max_distance + 1):
                    for dc in range(-max_distance, max_distance + 1):
                        neighbor = (r + dr, c + dc)
                        if neighbor in cells and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
            if len(cluster) >= 3:  # ignore tiny noise clusters
                clusters.append(cluster)

        return clusters

    def publish_markers(self, scored_frontiers):
        """Best frontier = GREEN sphere. Others = RED."""
        marker_array = MarkerArray()
        for i, f in enumerate(scored_frontiers):
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
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.35
            if i == 0:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
