import json
import math
import queue
import threading
import time

import firebase_admin
from firebase_admin import credentials, firestore
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty, String
import tf2_ros
from tf2_geometry_msgs import do_transform_point


# Map-frame coordinates of the single room's scan point. Set per environment.
SCAN_POINT = {'x': 1.0, 'y': 0.5}

APPROACH_DISTANCE = 0.8  # meters to stop short of the detected person
PERSON_LOCATION_TIMEOUT = 15.0  # seconds to wait for /person_location after /find_person
WELFARE_RESULT_TIMEOUT = 20.0  # seconds to wait for /welfare_result after /welfare_check
NAV_SERVER_WAIT_TIMEOUT = 5.0  # seconds to wait for the navigate_to_pose action server
TF_LOOKUP_TIMEOUT = 1.0  # seconds to wait for a map->base_link transform
WELFARE_WARMUP_RETRY_DELAY = 5.0  # seconds to wait before re-triggering /welfare_check once
MAX_NAV_FAILURES = 3  # nav/TF failures allowed for a given alert before giving up

STAGE_AWAITING_PERSON = 'awaiting_person'
STAGE_AWAITING_WELFARE = 'awaiting_welfare'

VERDICT_TO_STATUS = {
    'movement': 'confirmed_movement',
    'no_movement': 'confirmed_no_movement',
    'uncertain': 'confirmed_uncertain',
}

CREDENTIAL_PATH = '/home/et2020045/eldercare-firebase-key.json'
PROJECT_ID = 'fall-detector-app-3319d'


class FirestoreDispatcher(Node):

    def __init__(self):
        super().__init__('firestore_dispatcher')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._alert_queue = queue.Queue()
        self._in_progress = set()
        self._lock = threading.Lock()

        # Exactly one alert is worked at a time - the robot can only be in one place.
        self._active = None
        self._nav_failure_counts = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.find_person_pub = self.create_publisher(Empty, '/find_person', 10)
        self.welfare_check_pub = self.create_publisher(Empty, '/welfare_check', 10)

        self.create_subscription(
            PointStamped, '/person_location', self._on_person_location, 10)
        self.create_subscription(
            String, '/welfare_result', self._on_welfare_result, 10)

        cred = credentials.Certificate(CREDENTIAL_PATH)
        try:
            firebase_admin.initialize_app(cred, {'projectId': PROJECT_ID})
        except ValueError:
            pass  # app already initialized; safe to reuse on node restart
        self._db = firestore.client()

        self._setup_listener()
        self.create_timer(1.0, self._process_queue)
        self.create_timer(0.5, self._check_timeouts)
        self.get_logger().info('FirestoreDispatcher ready, listening for fall alerts')

    def _setup_listener(self):
        query = self._db.collection('fall_alerts').where('status', '==', 'pending')
        self._watch = query.on_snapshot(self._on_snapshot)

    def _on_snapshot(self, col_snapshot, changes, read_time):
        for change in changes:
            if change.type.name in ('ADDED', 'MODIFIED'):
                doc = change.document
                data = doc.to_dict()
                if data.get('status') == 'pending':
                    self._alert_queue.put((doc.id, data))

    def _process_queue(self):
        if self._active is not None:
            return  # already working an alert

        try:
            doc_id, data = self._alert_queue.get_nowait()
        except queue.Empty:
            return

        with self._lock:
            if doc_id in self._in_progress:
                return
            self._in_progress.add(doc_id)

        room = data.get('room', '')
        self._active = {
            'doc_id': doc_id,
            'room': room,
            'stage': None,
            'deadline': None,
        }

        self.get_logger().info(
            f'Fall alert in "{room}" (doc={doc_id}), dispatching to scan point {SCAN_POINT}'
        )
        self._navigate(
            doc_id, SCAN_POINT['x'], SCAN_POINT['y'], yaw=0.0,
            on_success=self._on_scan_point_reached,
            on_failure=self._on_nav_failure,
        )

    # ------------------------------------------------------------------
    # Nav2 helpers
    # ------------------------------------------------------------------

    def _navigate(self, doc_id, x, y, yaw, on_success, on_failure):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        if not self._action_client.wait_for_server(timeout_sec=NAV_SERVER_WAIT_TIMEOUT):
            self.get_logger().error('navigate_to_pose action server not available')
            on_failure(doc_id, 'action server unavailable')
            return

        try:
            future = self._action_client.send_goal_async(goal)
            future.add_done_callback(
                lambda f: self._on_goal_response(f, doc_id, on_success, on_failure))
        except Exception as e:
            self.get_logger().error(f'send_goal_async failed: {e}')
            on_failure(doc_id, f'send_goal_async exception: {e}')

    def _on_goal_response(self, future, doc_id, on_success, on_failure):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Goal send failed: {e}')
            on_failure(doc_id, f'goal send exception: {e}')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected goal')
            on_failure(doc_id, 'goal rejected')
            return
        self.get_logger().info('Nav2 accepted goal')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_nav_result(f, doc_id, on_success, on_failure))

    def _on_nav_result(self, future, doc_id, on_success, on_failure):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Nav2 result error: {e}')
            on_failure(doc_id, f'nav result exception: {e}')
            return
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Nav2 failed to reach goal (status={result.status})')
            on_failure(doc_id, f'nav status {result.status}')
            return
        on_success(doc_id)

    def _on_nav_failure(self, doc_id, reason):
        count = self._nav_failure_counts.get(doc_id, 0) + 1
        self._nav_failure_counts[doc_id] = count

        if count >= MAX_NAV_FAILURES:
            self.get_logger().error(
                f'Navigation failed {count} times for alert {doc_id} '
                f'(last reason: {reason}); giving up')
            self._nav_failure_counts.pop(doc_id, None)
            self._finalize_alert(doc_id, 'navigation_failed')
            return

        # Leave the Firestore doc as 'pending' so it can be retried; just stop working it.
        self.get_logger().error(
            f'Navigation failed for alert {doc_id} (attempt {count}/{MAX_NAV_FAILURES}): {reason}')
        self._clear_active(doc_id)

    # ------------------------------------------------------------------
    # Stage 1: scan point -> person search
    # ------------------------------------------------------------------

    def _on_scan_point_reached(self, doc_id):
        active = self._active
        if active is None or active['doc_id'] != doc_id:
            return  # stale callback from an alert we've already dropped

        self.get_logger().info(f'Arrived at scan point for alert {doc_id}, searching for person')
        self.find_person_pub.publish(Empty())
        active['stage'] = STAGE_AWAITING_PERSON
        active['deadline'] = time.time() + PERSON_LOCATION_TIMEOUT

    def _on_person_location(self, msg: PointStamped):
        active = self._active
        if active is None or active['stage'] != STAGE_AWAITING_PERSON:
            return  # not expecting this right now

        doc_id = active['doc_id']
        active['stage'] = None  # guard against re-entry while we compute the approach pose
        active['deadline'] = None

        approach = self._compute_approach_pose(msg)
        if approach is None:
            self.get_logger().error(
                f'Could not transform person location to map frame for alert {doc_id}')
            self._on_nav_failure(doc_id, 'tf transform to map failed')
            return

        approach_x, approach_y, yaw = approach
        self.get_logger().info(
            f'Person found for alert {doc_id}, approaching ({approach_x:.2f}, {approach_y:.2f})')
        self._navigate(
            doc_id, approach_x, approach_y, yaw,
            on_success=self._on_approach_reached,
            on_failure=self._on_nav_failure,
        )

    def _compute_approach_pose(self, person_point: PointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', person_point.header.frame_id, person_point.header.stamp,
                timeout=rclpy.duration.Duration(seconds=TF_LOOKUP_TIMEOUT))
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'TF lookup map<-{person_point.header.frame_id} failed: {e}')
            return None

        person_map = do_transform_point(person_point, transform).point
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        dx = person_map.x - robot_x
        dy = person_map.y - robot_y
        dist = math.hypot(dx, dy)

        if dist < 1e-3:
            # Person is essentially where the robot already is; nowhere to approach from.
            return (robot_x, robot_y, 0.0)

        yaw = math.atan2(dy, dx)
        approach_x = person_map.x - (dx / dist) * APPROACH_DISTANCE
        approach_y = person_map.y - (dy / dist) * APPROACH_DISTANCE
        return (approach_x, approach_y, yaw)

    # ------------------------------------------------------------------
    # Stage 2: approach -> welfare check
    # ------------------------------------------------------------------

    def _on_approach_reached(self, doc_id):
        active = self._active
        if active is None or active['doc_id'] != doc_id:
            return

        self.get_logger().info(f'Arrived at approach pose for alert {doc_id}, checking welfare')
        self.welfare_check_pub.publish(Empty())
        active['stage'] = STAGE_AWAITING_WELFARE
        active['deadline'] = time.time() + WELFARE_RESULT_TIMEOUT

    def _on_welfare_result(self, msg: String):
        active = self._active
        if active is None or active['stage'] != STAGE_AWAITING_WELFARE:
            return

        doc_id = active['doc_id']
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Malformed /welfare_result payload for alert {doc_id}: {e}')
            self._on_nav_failure(doc_id, 'malformed welfare result')
            return

        verdict = payload.get('verdict')

        if verdict == 'warming_up':
            if active.get('welfare_retried'):
                self.get_logger().error(
                    f'Welfare sensor still warming up after retry for alert {doc_id}')
                self._finalize_alert(doc_id, 'welfare_timeout')
                return

            self.get_logger().warn(
                f'Welfare sensor warming up for alert {doc_id}; '
                f'retrying in {WELFARE_WARMUP_RETRY_DELAY:.0f}s')
            active['welfare_retried'] = True
            active['stage'] = None  # pause matching until the retry fires
            active['deadline'] = None
            self._schedule_welfare_retry(doc_id)
            return

        status = VERDICT_TO_STATUS.get(verdict)
        if status is None:
            self.get_logger().warn(
                f'Unrecognized welfare verdict "{verdict}" for alert {doc_id}, '
                'treating as confirmed_uncertain')
            status = 'confirmed_uncertain'

        self._finalize_alert(doc_id, status)

    def _schedule_welfare_retry(self, doc_id):
        holder = {}

        def _retry():
            holder['timer'].cancel()
            active = self._active
            if active is None or active['doc_id'] != doc_id:
                return
            self.get_logger().info(f'Retrying welfare check for alert {doc_id}')
            self.welfare_check_pub.publish(Empty())
            active['stage'] = STAGE_AWAITING_WELFARE
            active['deadline'] = time.time() + WELFARE_RESULT_TIMEOUT

        holder['timer'] = self.create_timer(WELFARE_WARMUP_RETRY_DELAY, _retry)

    # ------------------------------------------------------------------
    # Timeouts
    # ------------------------------------------------------------------

    def _check_timeouts(self):
        active = self._active
        if active is None or active['deadline'] is None:
            return
        if time.time() < active['deadline']:
            return

        doc_id = active['doc_id']
        if active['stage'] == STAGE_AWAITING_PERSON:
            self.get_logger().warn(f'No person found within timeout for alert {doc_id}')
            self._finalize_alert(doc_id, 'no_person_found')
        elif active['stage'] == STAGE_AWAITING_WELFARE:
            self.get_logger().error(f'Welfare result timed out for alert {doc_id}')
            self._finalize_alert(doc_id, 'welfare_timeout')

    # ------------------------------------------------------------------
    # Finalization
    # ------------------------------------------------------------------

    def _finalize_alert(self, doc_id, status):
        try:
            self._db.collection('fall_alerts').document(doc_id).update({'status': status})
            self.get_logger().info(f'Alert {doc_id} marked as "{status}" in Firestore')
        except Exception as e:
            self.get_logger().error(f'Failed to update alert {doc_id}: {e}')
        finally:
            self._nav_failure_counts.pop(doc_id, None)
            self._clear_active(doc_id)

    def _clear_active(self, doc_id):
        with self._lock:
            self._in_progress.discard(doc_id)
        if self._active is not None and self._active['doc_id'] == doc_id:
            self._active = None


def main(args=None):
    rclpy.init(args=args)
    node = FirestoreDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
