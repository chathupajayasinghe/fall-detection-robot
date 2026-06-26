#!/usr/bin/env python3
import os
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time

os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"
from gpiozero import RotaryEncoder

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Encoder pins (from your hardware_test.py)
        self.left_encoder = RotaryEncoder(17, 27, max_steps=0)
        self.right_encoder = RotaryEncoder(22, 10, max_steps=0)
        
        # Robot parameters
        self.wheel_radius = 0.0325  # meters (adjust to your wheel size)
        self.wheel_base = 0.20      # meters between wheels
        self.ticks_per_revolution = 20  # adjust for your encoder
        
        # Position tracking
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = time.time()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.create_timer(0.05, self.update_odometry)
        
    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Get encoder counts
        left_ticks = self.left_encoder.steps
        right_ticks = self.right_encoder.steps
        
        # Calculate distance traveled
        left_dist = (left_ticks - self.last_left_ticks) / self.ticks_per_revolution * (2 * math.pi * self.wheel_radius)
        right_dist = (right_ticks - self.last_right_ticks) / self.ticks_per_revolution * (2 * math.pi * self.wheel_radius)
        
        # Calculate robot motion
        linear = (right_dist + left_dist) / 2.0
        angular = (right_dist - left_dist) / self.wheel_base
        
        # Update position
        self.x += linear * math.cos(self.theta)
        self.y += linear * math.sin(self.theta)
        self.theta += angular
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        self.odom_pub.publish(odom)
        
        # Save state
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time

def main():
    rclpy.init()
    node = OdometryPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

