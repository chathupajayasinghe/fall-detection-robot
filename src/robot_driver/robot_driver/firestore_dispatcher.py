import queue
import threading

import firebase_admin
from firebase_admin import credentials, firestore
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


ROOM_COORDINATES = {
    'bedroom':     {'x':  1.0, 'y':  0.5, 'z': 0.0},
    'kitchen':     {'x': -1.0, 'y':  1.0, 'z': 0.0},
    'bathroom':    {'x':  0.0, 'y': -1.0, 'z': 0.0},
    'living_room': {'x':  2.0, 'y':  0.0, 'z': 0.0},
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

        cred = credentials.Certificate(CREDENTIAL_PATH)
        firebase_admin.initialize_app(cred, {'projectId': PROJECT_ID})
        self._db = firestore.client()

        self._setup_listener()
        self.create_timer(1.0, self._process_queue)
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
        try:
            doc_id, data = self._alert_queue.get_nowait()
        except queue.Empty:
            return

        with self._lock:
            if doc_id in self._in_progress:
                return
            self._in_progress.add(doc_id)

        room = data.get('room', '')
        coords = ROOM_COORDINATES.get(room)
        if coords is None:
            self.get_logger().warn(f'Unknown room "{room}" in alert {doc_id}, skipping')
            with self._lock:
                self._in_progress.discard(doc_id)
            return

        self.get_logger().info(
            f'Fall alert in "{room}" (doc={doc_id}), dispatching to {coords}'
        )
        self._send_nav_goal(doc_id, room, coords)

    def _send_nav_goal(self, doc_id, room, coords):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(coords['x'])
        pose.pose.position.y = float(coords['y'])
        pose.pose.position.z = float(coords['z'])
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available')
            with self._lock:
                self._in_progress.discard(doc_id)
            return

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, doc_id, room))

    def _on_goal_response(self, future, doc_id, room):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Nav2 rejected goal to "{room}"')
            with self._lock:
                self._in_progress.discard(doc_id)
            return
        self.get_logger().info(f'Nav2 accepted goal to "{room}"')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_result(f, doc_id, room))

    def _on_nav_result(self, future, doc_id, room):
        result = future.result()
        self.get_logger().info(
            f'Arrived at "{room}" (Nav2 status={result.status}), acknowledging alert'
        )
        self._acknowledge_alert(doc_id)

    def _acknowledge_alert(self, doc_id):
        try:
            self._db.collection('fall_alerts').document(doc_id).update(
                {'status': 'acknowledged'}
            )
            self.get_logger().info(f'Alert {doc_id} marked as acknowledged in Firestore')
        except Exception as e:
            self.get_logger().error(f'Failed to acknowledge alert {doc_id}: {e}')
        finally:
            with self._lock:
                self._in_progress.discard(doc_id)


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
