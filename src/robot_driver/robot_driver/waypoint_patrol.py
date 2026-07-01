#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (0.608, -0.925, 0.0),
    (0.354, -1.599, 0.0),
    (-1.051, -1.053, 0.0),
]

class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')
        self.waypoints = WAYPOINTS
        self.current_index = 0
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._action_client.wait_for_server()
        self.send_next_goal()

    def yaw_to_quaternion(self, yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_next_goal(self):
        x, y, yaw = self.waypoints[self.current_index]
        self.get_logger().info(
            f'Driving to waypoint {self.current_index}: x={x}, y={y}, yaw={yaw}')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Waypoint {self.current_index} was rejected by the action server')
            self.send_next_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        future.result()
        self.get_logger().info(f'Reached waypoint {self.current_index}')
        self.current_index = (self.current_index + 1) % len(self.waypoints)
        self.send_next_goal()

def main():
    rclpy.init()
    node = WaypointPatrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
