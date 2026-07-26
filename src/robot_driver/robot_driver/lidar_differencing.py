#!/usr/bin/env python3
"""Person detection by LIDAR scan differencing.

Compares the live ``/scan`` against a reference scan captured while the room was
empty. Beams that come back *closer* than the reference indicate something new in
the room; contiguous runs of such beams are clustered and filtered to person-sized
objects, and the centroid of the best candidate is published on ``/person_location``.

Workflow:
    1. Publish to ``/save_reference_scan`` once, with the room empty.
    2. Publish to ``/find_person`` whenever a search is wanted.
"""
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

MIN_CLUSTER_POINTS = 8  # fewer beams than this is noise; span filter still rejects small objects
MIN_PERSON_SIZE = 0.3  # meters, smallest plausible physical span of a person-sized cluster
MAX_PERSON_SIZE = 2.0  # meters, largest plausible physical span of a person-sized cluster


class LidarDifferencing(Node):
    """Detects a person as the difference between a live and a reference scan."""

    def __init__(self):
        super().__init__('lidar_differencing')

        self._setup_scan_state()
        self._load_reference_scan()
        self._setup_interfaces()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _setup_scan_state(self):
        """Initialise the reference and live scan buffers to 'nothing yet'."""
        self.reference_ranges = None
        self.latest_scan = None

    def _setup_interfaces(self):
        """Create the scan/trigger subscriptions and the person location publisher."""
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)
        self.save_sub = self.create_subscription(
            Empty, '/save_reference_scan', self._save_reference_callback, 10)
        self.find_sub = self.create_subscription(
            Empty, '/find_person', self._find_person_callback, 10)

        self.person_pub = self.create_publisher(
            PointStamped, '/person_location', 10)

    # ------------------------------------------------------------------
    # Reference scan persistence
    # ------------------------------------------------------------------

    def _load_reference_scan(self):
        """Load the empty-room reference scan from disk, if one was ever saved.

        Absence is not an error - the node runs fine without one and simply
        refuses to search until a reference is captured.
        """
        if os.path.exists(REFERENCE_SCAN_PATH):
            self.reference_ranges = np.load(REFERENCE_SCAN_PATH)
            self.get_logger().info(
                f'Loaded reference scan from {REFERENCE_SCAN_PATH}')
        else:
            self.get_logger().warn(
                f'No reference scan found at {REFERENCE_SCAN_PATH}. '
                'Publish to /save_reference_scan once the room is empty.')

    def _scan_callback(self, msg: LaserScan):
        """Buffer the most recent scan; differencing happens on demand, not per scan."""
        self.latest_scan = msg

    def _save_reference_callback(self, _msg: Empty):
        """Snapshot the current scan as the empty-room reference, on disk and in memory."""
        if self.latest_scan is None:
            self.get_logger().warn('No scan received yet, cannot save reference.')
            return

        ranges = np.array(self.latest_scan.ranges, dtype=np.float64)
        os.makedirs(os.path.dirname(REFERENCE_SCAN_PATH), exist_ok=True)
        np.save(REFERENCE_SCAN_PATH, ranges)
        self.reference_ranges = ranges
        self.get_logger().info(
            f'Saved reference scan ({len(ranges)} beams) to {REFERENCE_SCAN_PATH}')

    # ------------------------------------------------------------------
    # Person search
    # ------------------------------------------------------------------

    def _find_person_callback(self, _msg: Empty):
        """Diff the live scan against the reference and publish the best person candidate."""
        if not self._detection_inputs_ready():
            return

        scan = self.latest_scan
        live_ranges = np.array(scan.ranges, dtype=np.float64)
        reference_ranges = self.reference_ranges

        if not self._beam_counts_match(live_ranges, reference_ranges):
            return

        is_new_obstacle = self._compute_new_obstacle_mask(live_ranges, reference_ranges)

        cluster = self._select_person_cluster(is_new_obstacle, live_ranges, scan)
        if cluster is None:
            self.get_logger().info('No new obstacle found in live scan.')
            return

        centroid = self._cluster_centroid(cluster, live_ranges, scan)
        if centroid is None:
            self.get_logger().info('Cluster found but centroid was invalid.')
            return

        x, y = centroid
        self._publish_person_location(x, y)

        self.get_logger().info(
            f'Person candidate detected at base_link ({x:.2f}, {y:.2f}), '
            f'cluster size={len(cluster)}')

    def _detection_inputs_ready(self):
        """True when both a reference scan and a live scan are available to compare."""
        if self.reference_ranges is None:
            self.get_logger().warn('No reference scan available. Save one first.')
            return False
        if self.latest_scan is None:
            self.get_logger().warn('No live scan received yet.')
            return False
        return True

    def _beam_counts_match(self, live_ranges, reference_ranges):
        """True when live and reference scans are index-comparable.

        A mismatch means the reference was captured with a different LIDAR
        configuration (resolution or angular range), so beam i in one is not
        beam i in the other and the diff would be meaningless.
        """
        if len(live_ranges) != len(reference_ranges):
            self.get_logger().warn(
                'Live scan and reference scan have different beam counts '
                f'({len(live_ranges)} vs {len(reference_ranges)}); cannot compare.')
            return False
        return True

    def _compute_new_obstacle_mask(self, live_ranges, reference_ranges):
        """Per-beam mask of returns that are DIFF_THRESHOLD closer than the reference.

        Only beams with a finite, positive range in *both* scans are considered;
        infinities and zeros mean 'no return' and carry no information. The
        comparison is one-directional on purpose: a beam reading *further* than
        the reference means something was removed, not that a person arrived.
        """
        n = len(live_ranges)
        valid = (
            np.isfinite(live_ranges) & np.isfinite(reference_ranges)
            & (live_ranges > 0.0) & (reference_ranges > 0.0)
        )

        is_new_obstacle = np.zeros(n, dtype=bool)
        is_new_obstacle[valid] = (
            reference_ranges[valid] - live_ranges[valid]
        ) > DIFF_THRESHOLD
        return is_new_obstacle

    def _publish_person_location(self, x, y):
        """Publish the person candidate centroid in the base_link frame."""
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = 'base_link'
        point.point.x = x
        point.point.y = y
        point.point.z = 0.0
        self.person_pub.publish(point)

    # ------------------------------------------------------------------
    # Clustering
    # ------------------------------------------------------------------

    def _select_person_cluster(self, is_new_obstacle, live_ranges, scan: LaserScan):
        """Pick the largest person-sized cluster of new obstacle beams, or None."""
        n = len(is_new_obstacle)
        if not np.any(is_new_obstacle):
            return None

        clusters = self._group_contiguous_indices(is_new_obstacle)
        clusters = self._merge_wraparound_cluster(clusters, n)

        if not clusters:
            return None

        candidates = [
            cluster for cluster in clusters
            if self._is_person_sized(cluster, live_ranges, scan)
        ]

        if not candidates:
            return None

        return max(candidates, key=len)

    def _group_contiguous_indices(self, is_new_obstacle):
        """Group flagged beam indices into runs of angularly adjacent beams."""
        n = len(is_new_obstacle)
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
        return clusters

    def _merge_wraparound_cluster(self, clusters, n):
        """Join the first and last clusters when an object straddles the 0/360 seam.

        Scan indices 0 and n-1 are angularly adjacent for a full sweep, so an
        object sitting behind the robot appears as two separate runs at either
        end of the index range.
        """
        if len(clusters) > 1 and clusters[0][0] == 0 and clusters[-1][-1] == n - 1:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()
        return clusters

    def _is_person_sized(self, cluster, live_ranges, scan: LaserScan):
        """True when a cluster is big enough to be real and small enough to be a person.

        Two filters, because neither alone is sufficient: the beam count rejects
        sensor noise, while the physical span rejects things that return many
        beams but are the wrong size - a chair leg up close, or a whole wall.
        """
        if len(cluster) < MIN_CLUSTER_POINTS:
            return False

        points = self._cluster_points(cluster, live_ranges, scan)
        span = self._cluster_span(points)
        if span is None or span < MIN_PERSON_SIZE or span > MAX_PERSON_SIZE:
            return False

        return True

    # ------------------------------------------------------------------
    # Cluster geometry
    # ------------------------------------------------------------------

    def _cluster_points(self, cluster_indices, live_ranges, scan: LaserScan):
        """Convert a cluster's beams to Cartesian (x, y), dropping invalid returns.

        The ``+ math.pi`` term rotates the beams 180 degrees because the LIDAR is
        physically mounted facing backwards - the same rotation the
        ``base_link -> laser`` static transform declares as its 3.14159 yaw.

        Known limitation: the static transform also declares x = -0.045 m, which
        this conversion does not apply, so the resulting points carry a constant
        ~4.5 cm offset from true base_link.
        """
        points = []
        for i in cluster_indices:
            r = live_ranges[i]
            if not np.isfinite(r) or r <= 0.0:
                continue
            angle = scan.angle_min + i * scan.angle_increment + math.pi
            points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

    def _cluster_span(self, points):
        """Largest pairwise distance across a cluster - its physical extent in meters.

        Brute-force O(n^2) over the cluster's beams. Person-sized clusters are
        small enough for this to be irrelevant, and it runs only on demand.
        """
        if len(points) < 2:
            return None

        max_dist = 0.0
        for idx, (x1, y1) in enumerate(points):
            for x2, y2 in points[idx + 1:]:
                dist = math.hypot(x2 - x1, y2 - y1)
                if dist > max_dist:
                    max_dist = dist
        return max_dist

    def _cluster_centroid(self, cluster_indices, live_ranges, scan: LaserScan):
        """Mean (x, y) of a cluster's valid beams, or None if none are valid."""
        points = self._cluster_points(cluster_indices, live_ranges, scan)
        if not points:
            return None

        xs = [x for x, _ in points]
        ys = [y for _, y in points]
        return (sum(xs) / len(xs), sum(ys) / len(ys))


def main():
    rclpy.init()
    node = LidarDifferencing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
