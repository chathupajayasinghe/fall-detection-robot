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
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from geometry_msgs.msg import PointStamped

REFERENCE_SCAN_PATH = os.path.expanduser('~/maps/reference_scan.npy')
DIFF_THRESHOLD = 0.3  # meters closer than reference to count as a new obstacle
# Half-width of the reference window each live beam is compared against. Absorbs
# the 1-2 deg heading error between reference capture and arrival, which otherwise
# flips beams at the door recess edges between door and wall.
REFERENCE_WINDOW_DEG = 3.0

LIVE_SCAN_COUNT = 5  # scans median-filtered per /find_person, to suppress single-scan dropouts
REFERENCE_SCAN_COUNT = 10  # scans median-filtered into the saved empty-room reference

MIN_CLUSTER_POINTS = 5  # fewer beams than this is noise; span filter still rejects small objects
MIN_PERSON_SIZE = 0.2  # meters, smallest plausible physical span of a person-sized cluster
MAX_PERSON_SIZE = 2.0  # meters, largest plausible physical span of a person-sized cluster

# Adjacent clusters whose facing endpoints are closer than this are one fragmented object.
MERGE_GAP = 0.15  # meters
# Clusters centred farther than this are ignored: the fall zone is 1.0-1.8 m from the scan
# point, while wall and door ghosts sit at 2.4-3.2 m.
MAX_SEARCH_RANGE = 2.2  # meters from base_link


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
        # Multi-scan captures: None when idle, else the scans gathered so far.
        self._live_scans = None
        self._reference_scans = None

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
        """Buffer the most recent scan and feed any multi-scan capture in progress.

        Differencing happens on demand, once a capture has collected its scans,
        not per scan.
        """
        self.latest_scan = msg

        if self._reference_scans is not None:
            self._reference_scans.append(msg)
            if len(self._reference_scans) >= REFERENCE_SCAN_COUNT:
                scans, self._reference_scans = self._reference_scans, None
                self._save_reference(scans)

        if self._live_scans is not None:
            self._live_scans.append(msg)
            if len(self._live_scans) >= LIVE_SCAN_COUNT:
                scans, self._live_scans = self._live_scans, None
                self._find_person(scans)

    def _save_reference_callback(self, _msg: Empty):
        """Start capturing REFERENCE_SCAN_COUNT scans for the empty-room reference."""
        if self.latest_scan is None:
            self.get_logger().warn('No scan received yet, cannot save reference.')
            return
        if self._reference_scans is not None:
            self.get_logger().warn('Reference capture already in progress; ignoring request.')
            return

        self._reference_scans = []
        self.get_logger().info(f'Capturing {REFERENCE_SCAN_COUNT} scans for the reference')

    def _save_reference(self, scans):
        """Save the per-beam median of the scans as the reference, on disk and in memory."""
        ranges = self._median_ranges(scans)
        os.makedirs(os.path.dirname(REFERENCE_SCAN_PATH), exist_ok=True)
        np.save(REFERENCE_SCAN_PATH, ranges)
        self.reference_ranges = ranges
        self.get_logger().info(
            f'Saved reference scan ({len(ranges)} beams, median of {len(scans)} scans) '
            f'to {REFERENCE_SCAN_PATH}')

    def _median_ranges(self, scans):
        """Per-beam median range across scans, ignoring no-return (inf/0) readings.

        A beam with no valid reading in any scan comes out as inf, the same
        'no return' value a single scan would carry. Scans whose beam count
        differs from the last one are left out, since their beams do not line up.
        """
        n = len(scans[-1].ranges)
        matching = [s for s in scans if len(s.ranges) == n]
        if len(matching) < len(scans):
            self.get_logger().warn(
                f'Left {len(scans) - len(matching)} of {len(scans)} scans out of the median '
                f'(beam count other than {n}).')

        stacked = np.array([s.ranges for s in matching], dtype=np.float64)
        stacked[~np.isfinite(stacked) | (stacked <= 0.0)] = np.nan
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)  # beams with no valid reading
            median = np.nanmedian(stacked, axis=0)
        median[np.isnan(median)] = np.inf
        return median

    # ------------------------------------------------------------------
    # Person search
    # ------------------------------------------------------------------

    def _find_person_callback(self, _msg: Empty):
        """Start capturing LIVE_SCAN_COUNT scans for a person search."""
        if not self._detection_inputs_ready():
            return
        if self._live_scans is not None:
            self.get_logger().info('Person search already in progress; ignoring request.')
            return

        self._live_scans = []

    def _find_person(self, scans):
        """Diff the median live scan against the reference and publish the best candidate."""
        scan = scans[-1]  # angle_min / angle_increment are the same for every scan
        live_ranges = self._median_ranges(scans)
        reference_ranges = self.reference_ranges

        if not self._beam_counts_match(live_ranges, reference_ranges):
            return

        is_new_obstacle = self._compute_new_obstacle_mask(
            live_ranges, reference_ranges, scan.angle_increment)

        cluster = self._select_person_cluster(is_new_obstacle, live_ranges, scan)
        if cluster is None:
            self.get_logger().info('No person-sized cluster found in live scan.')
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

    def _compute_new_obstacle_mask(self, live_ranges, reference_ranges, angle_increment):
        """Per-beam mask of returns DIFF_THRESHOLD closer than the nearby reference.

        Each live beam is compared against the *minimum* reference range within
        +/-REFERENCE_WINDOW_DEG of it, not just the same-index beam. A small
        heading error between reference and arrival shifts depth discontinuities
        (door recesses, furniture edges) by a beam or two; the windowed minimum
        absorbs that, while a genuinely new object on open floor is still closer
        than every reference beam around it.

        Only beams with a finite, positive range in *both* scans are considered;
        infinities and zeros mean 'no return' and carry no information. The
        comparison is one-directional on purpose: a beam reading *further* than
        the reference means something was removed, not that a person arrived.
        """
        n = len(live_ranges)
        reference_valid = np.isfinite(reference_ranges) & (reference_ranges > 0.0)
        valid = np.isfinite(live_ranges) & (live_ranges > 0.0) & reference_valid

        # No-return reference beams must not win the minimum, so treat them as infinitely far.
        reference_for_min = np.where(reference_valid, reference_ranges, np.inf)
        half_width = int(round(math.radians(REFERENCE_WINDOW_DEG) / abs(angle_increment)))
        window_min = reference_for_min.copy()
        for shift in range(1, half_width + 1):
            # np.roll wraps around, matching the full 360 deg sweep
            # (see _merge_wraparound_cluster).
            window_min = np.minimum(window_min, np.roll(reference_for_min, shift))
            window_min = np.minimum(window_min, np.roll(reference_for_min, -shift))

        is_new_obstacle = np.zeros(n, dtype=bool)
        is_new_obstacle[valid] = (window_min[valid] - live_ranges[valid]) > DIFF_THRESHOLD
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
        clusters = self._merge_nearby_clusters(clusters, live_ranges, scan)

        if not clusters:
            return None

        candidates = []
        for cluster in clusters:
            rejection, span = self._cluster_rejection(cluster, live_ranges, scan)
            self._log_cluster(cluster, live_ranges, scan, span, rejection)
            if rejection is None:
                candidates.append(cluster)

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

    def _merge_nearby_clusters(self, clusters, live_ranges, scan: LaserScan):
        """Join angularly adjacent clusters whose facing endpoints are within MERGE_GAP.

        One object can come back as several runs when a few beams across it are
        not flagged. Rejoining them before the size filters stops a fragmented
        box losing to a larger ghost. Clusters are in angular order, so each is
        compared with the next, and the last with the first across the 0/360
        seam. The unflagged gap beams are not added to the merged cluster.
        """
        if len(clusters) < 2:
            return clusters

        groups = [[clusters[0]]]
        for cluster in clusters[1:]:
            if self._endpoint_gap(groups[-1][-1], cluster, live_ranges, scan) <= MERGE_GAP:
                groups[-1].append(cluster)
            else:
                groups.append([cluster])

        if len(groups) > 1 and self._endpoint_gap(
                groups[-1][-1], groups[0][0], live_ranges, scan) <= MERGE_GAP:
            groups[0] = groups.pop() + groups[0]

        merged = []
        for group in groups:
            if len(group) > 1:
                self._log_merge(group, live_ranges, scan)
            merged.append([i for cluster in group for i in cluster])
        return merged

    def _endpoint_gap(self, cluster_a, cluster_b, live_ranges, scan: LaserScan):
        """Distance in meters from the last beam of cluster_a to the first beam of cluster_b."""
        end = self._cluster_points([cluster_a[-1]], live_ranges, scan)
        start = self._cluster_points([cluster_b[0]], live_ranges, scan)
        if not end or not start:
            return math.inf
        return math.hypot(start[0][0] - end[0][0], start[0][1] - end[0][1])

    def _log_merge(self, group, live_ranges, scan: LaserScan):
        """Log which clusters were joined and across what gaps."""
        sizes = ' + '.join(str(len(cluster)) for cluster in group)
        gaps = ', '.join(
            f'{self._endpoint_gap(a, b, live_ranges, scan):.2f}'
            for a, b in zip(group, group[1:]))
        total = sum(len(cluster) for cluster in group)
        self.get_logger().info(
            f'Merged adjacent clusters of {sizes} points (gaps {gaps} m) '
            f'into one of {total} points')

    def _cluster_rejection(self, cluster, live_ranges, scan: LaserScan):
        """Name of the filter rejecting a cluster (None if it passes), and its span.

        The range filter drops wall and door ghosts beyond the fall zone. The two
        size filters are both needed: the beam count rejects sensor noise, while
        the physical span rejects things that return many beams but are the
        wrong size - a chair leg up close, or a whole wall. The span is measured
        even when an earlier filter rejects the cluster, so every cluster can be
        logged in full.
        """
        points = self._cluster_points(cluster, live_ranges, scan)
        span = self._cluster_span(points)

        centroid = self._cluster_centroid(cluster, live_ranges, scan)
        if centroid is not None and math.hypot(*centroid) > MAX_SEARCH_RANGE:
            return 'MAX_SEARCH_RANGE', span
        if len(cluster) < MIN_CLUSTER_POINTS:
            return 'MIN_CLUSTER_POINTS', span
        if span is None or span < MIN_PERSON_SIZE:
            return 'MIN_PERSON_SIZE', span
        if span > MAX_PERSON_SIZE:
            return 'MAX_PERSON_SIZE', span
        return None, span

    def _log_cluster(self, cluster, live_ranges, scan: LaserScan, span, rejection):
        """Log one candidate cluster and its verdict, so a missed detection can be diagnosed."""
        centroid = self._cluster_centroid(cluster, live_ranges, scan)
        centre = f'({centroid[0]:.2f}, {centroid[1]:.2f})' if centroid else '(invalid)'
        span_text = f'{span:.2f} m' if span is not None else 'n/a'
        verdict = f'rejected by {rejection}' if rejection else 'accepted'
        self.get_logger().info(
            f'Cluster at base_link {centre}: {len(cluster)} points, span {span_text}, {verdict}')

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
