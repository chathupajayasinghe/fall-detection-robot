#!/usr/bin/env python3
import os
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from geometry_msgs.msg import PointStamped

REFERENCE_SCAN_PATH = os.path.expanduser('~/maps/reference_scan.npy')
DIFF_THRESHOLD = 0.3  # meters closer than reference to count as a new obstacle

MIN_CLUSTER_POINTS = 12  # fewer beams than this is noise/small objects (e.g. a backpack at ~9)
MIN_PERSON_SIZE = 0.3  # meters, smallest plausible physical span of a person-sized cluster
MAX_PERSON_SIZE = 2.0  # meters, largest plausible physical span of a person-sized cluster


class LidarDifferencing(Node):
    def __init__(self):
        super().__init__('lidar_differencing')

        self.reference_ranges = None
        self.latest_scan = None

        self.load_reference_scan()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.save_sub = self.create_subscription(
            Empty, '/save_reference_scan', self.save_reference_callback, 10)
        self.find_sub = self.create_subscription(
            Empty, '/find_person', self.find_person_callback, 10)

        self.person_pub = self.create_publisher(
            PointStamped, '/person_location', 10)

    def load_reference_scan(self):
        if os.path.exists(REFERENCE_SCAN_PATH):
            self.reference_ranges = np.load(REFERENCE_SCAN_PATH)
            self.get_logger().info(
                f'Loaded reference scan from {REFERENCE_SCAN_PATH}')
        else:
            self.get_logger().warn(
                f'No reference scan found at {REFERENCE_SCAN_PATH}. '
                'Publish to /save_reference_scan once the room is empty.')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def save_reference_callback(self, _msg: Empty):
        if self.latest_scan is None:
            self.get_logger().warn('No scan received yet, cannot save reference.')
            return

        ranges = np.array(self.latest_scan.ranges, dtype=np.float64)
        os.makedirs(os.path.dirname(REFERENCE_SCAN_PATH), exist_ok=True)
        np.save(REFERENCE_SCAN_PATH, ranges)
        self.reference_ranges = ranges
        self.get_logger().info(
            f'Saved reference scan ({len(ranges)} beams) to {REFERENCE_SCAN_PATH}')

    def find_person_callback(self, _msg: Empty):
        if self.reference_ranges is None:
            self.get_logger().warn('No reference scan available. Save one first.')
            return
        if self.latest_scan is None:
            self.get_logger().warn('No live scan received yet.')
            return

        scan = self.latest_scan
        live_ranges = np.array(scan.ranges, dtype=np.float64)
        reference_ranges = self.reference_ranges

        if len(live_ranges) != len(reference_ranges):
            self.get_logger().warn(
                'Live scan and reference scan have different beam counts '
                f'({len(live_ranges)} vs {len(reference_ranges)}); cannot compare.')
            return

        n = len(live_ranges)
        valid = (
            np.isfinite(live_ranges) & np.isfinite(reference_ranges)
            & (live_ranges > 0.0) & (reference_ranges > 0.0)
        )

        is_new_obstacle = np.zeros(n, dtype=bool)
        is_new_obstacle[valid] = (
            reference_ranges[valid] - live_ranges[valid]
        ) > DIFF_THRESHOLD

        cluster = self.select_person_cluster(is_new_obstacle, live_ranges, scan)
        if cluster is None:
            self.get_logger().info('No new obstacle found in live scan.')
            return

        centroid = self.cluster_centroid(cluster, live_ranges, scan)
        if centroid is None:
            self.get_logger().info('Cluster found but centroid was invalid.')
            return

        x, y = centroid
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = 'base_link'
        point.point.x = x
        point.point.y = y
        point.point.z = 0.0
        self.person_pub.publish(point)

        self.get_logger().info(
            f'Person candidate detected at base_link ({x:.2f}, {y:.2f}), '
            f'cluster size={len(cluster)}')

    def select_person_cluster(self, is_new_obstacle, live_ranges, scan: LaserScan):
        n = len(is_new_obstacle)
        if not np.any(is_new_obstacle):
            return None

        clusters = []
        current = []
        for i in range(n):
            if is_new_obstacle[i]:
                current.append(i)
            else:
                if current:
                    clusters.append(current)
                    current = []
        if current:
            clusters.append(current)

        # Merge wrap-around cluster (scan indices 0 and n-1 are adjacent for a full sweep)
        if len(clusters) > 1 and clusters[0][0] == 0 and clusters[-1][-1] == n - 1:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

        if not clusters:
            return None

        candidates = []
        for cluster in clusters:
            if len(cluster) < MIN_CLUSTER_POINTS:
                continue

            points = self.cluster_points(cluster, live_ranges, scan)
            span = self.cluster_span(points)
            if span is None or span < MIN_PERSON_SIZE or span > MAX_PERSON_SIZE:
                continue

            candidates.append(cluster)

        if not candidates:
            return None

        return max(candidates, key=len)

    def cluster_points(self, cluster_indices, live_ranges, scan: LaserScan):
        points = []
        for i in cluster_indices:
            r = live_ranges[i]
            if not np.isfinite(r) or r <= 0.0:
                continue
            angle = scan.angle_min + i * scan.angle_increment + math.pi
            points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

    def cluster_span(self, points):
        if len(points) < 2:
            return None

        max_dist = 0.0
        for idx, (x1, y1) in enumerate(points):
            for x2, y2 in points[idx + 1:]:
                dist = math.hypot(x2 - x1, y2 - y1)
                if dist > max_dist:
                    max_dist = dist
        return max_dist

    def cluster_centroid(self, cluster_indices, live_ranges, scan: LaserScan):
        xs = []
        ys = []
        for i in cluster_indices:
            r = live_ranges[i]
            if not np.isfinite(r) or r <= 0.0:
                continue
            angle = scan.angle_min + i * scan.angle_increment + math.pi
            xs.append(r * math.cos(angle))
            ys.append(r * math.sin(angle))

        if not xs:
            return None

        return (sum(xs) / len(xs), sum(ys) / len(ys))


def main():
    rclpy.init()
    node = LidarDifferencing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
