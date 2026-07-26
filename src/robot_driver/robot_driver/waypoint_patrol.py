#!/usr/bin/env python3
"""Standalone Nav2 waypoint patrol.

Drives an endless loop through a fixed list of map-frame waypoints, sending one
NavigateToPose goal at a time and advancing when each completes.

Not started by any launch file - run manually for testing:
    ros2 run robot_driver waypoint_patrol
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# Map-frame patrol route as (x, y, yaw). These coordinates are specific to the
# map this robot was surveyed in and will be meaningless against a different map.
WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (0.608, -0.925, 0.0),
    (0.354, -1.599, 0.0),
    (-1.051, -1.053, 0.0),
]

class WaypointPatrol(Node):
    """Cycles endlessly through WAYPOINTS via the navigate_to_pose action."""

    def __init__(self):
        super().__init__('waypoint_patrol')
        self.waypoints = WAYPOINTS
        self.current_index = 0
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # KNOWN ISSUE: blocks here indefinitely if Nav2 is not running. This is
        # before rclpy.spin, so the node never becomes responsive either.
        self._action_client.wait_for_server()
        self._send_next_goal()

    def _yaw_to_quaternion(self, yaw):
        """Convert a yaw angle to the (z, w) pair of a flat-ground quaternion."""
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _make_pose(self, x, y, yaw):
        """Build a stamped map-frame pose for one waypoint."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qz, qw = self._yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _send_next_goal(self):
        """Send the waypoint at the current index to Nav2."""
        x, y, yaw = self.waypoints[self.current_index]
        self.get_logger().info(
            f'Driving to waypoint {self.current_index}: x={x}, y={y}, yaw={yaw}')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, yaw)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle Nav2 accepting or rejecting the waypoint goal.

        KNOWN ISSUE: on rejection this retries the same index immediately, from
        inside the rejection callback, with no delay and no attempt limit - a
        persistently rejected goal becomes a tight retry loop.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Waypoint {self.current_index} was rejected by the action server')
            self._send_next_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Advance to the next waypoint, wrapping around to repeat the loop.

        KNOWN ISSUE: the result status is discarded, so an aborted or cancelled
        navigation is logged as 'Reached waypoint' and the patrol advances
        anyway. Compare _on_nav_result in firestore_dispatcher.py, which checks
        for STATUS_SUCCEEDED.
        """
        future.result()
        self.get_logger().info(f'Reached waypoint {self.current_index}')
        self.current_index = (self.current_index + 1) % len(self.waypoints)
        self._send_next_goal()

def main():
    rclpy.init()
    node = WaypointPatrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
