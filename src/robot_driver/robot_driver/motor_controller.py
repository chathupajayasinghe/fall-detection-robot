#!/usr/bin/env python3
import os
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"
from gpiozero import Device, PWMOutputDevice, DigitalOutputDevice, RotaryEncoder

# Motor speed correction factors to compensate for hardware imbalance.
# Tune RIGHT_SPEED_SCALE if the robot drifts — values below 1.0 slow that side.
LEFT_SPEED_SCALE  = 1.0
RIGHT_SPEED_SCALE = 0.85

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # --- Motor Pins (Channel A: Left, Channel B: Right) ---
        self.pwm_a = self._create_pwm_device(12)
        self.ain1  = DigitalOutputDevice(5)
        self.ain2  = DigitalOutputDevice(6)

        self.pwm_b = self._create_pwm_device(13)
        self.bin1  = DigitalOutputDevice(23)
        self.bin2  = DigitalOutputDevice(24)

        # --- Encoder Pins ---
        self.left_encoder = RotaryEncoder(17, 27, max_steps=0)
        self.right_encoder = RotaryEncoder(22, 10, max_steps=0)

        # --- Robot Parameters ---
        self.wheel_separation = 0.20   # meters between wheels
        self.wheel_radius = 0.0325     # meters
        self.ticks_per_rev = 20        # adjust for your encoder

        # --- Odometry State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = self.left_encoder.steps
        self.last_right = self.right_encoder.steps
        self.last_time = time.time()

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)

        # --- Timer for odometry ---
        self.create_timer(0.05, self.update_odometry)

        self.get_logger().info('🤖 Motor+Odometry Node Ready — Pins: 12,5,6,13,23,24')

    def _create_pwm_device(self, pin):
        for attempt in range(2):
            try:
                return PWMOutputDevice(pin, frequency=1000, initial_value=0)
            except Exception as exc:
                self.get_logger().warn(
                    f'PWM pin {pin} init failed on attempt {attempt + 1}: {exc}'
                )
                if attempt == 0:
                    Device.close_all()
                    time.sleep(0.1)
                else:
                    raise

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        left_speed  = (linear_x - (angular_z * self.wheel_separation / 2.0)) * LEFT_SPEED_SCALE
        right_speed = (linear_x + (angular_z * self.wheel_separation / 2.0)) * RIGHT_SPEED_SCALE
        self.control_motor(left_speed, self.pwm_a, self.ain1, self.ain2)
        self.control_motor(right_speed, self.pwm_b, self.bin1, self.bin2)

    def control_motor(self, speed, pwm_dev, in1, in2):
        speed = max(min(speed, 1.0), -1.0)
        if speed == 0.0:
            in1.off(); in2.off()
            pwm_dev.value = 0.0
            return
        if speed > 0:
            in1.off(); in2.on()
        else:
            in1.on(); in2.off()
        pwm_dev.value = abs(speed)

    def update_odometry(self):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return

        left_ticks = self.left_encoder.steps
        right_ticks = self.right_encoder.steps

        left_dist = (left_ticks - self.last_left) / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)
        right_dist = (right_ticks - self.last_right) / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)

        linear = (right_dist + left_dist) / 2.0
        angular = (right_dist - left_dist) / self.wheel_separation

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

        # Broadcast odom → base_link transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        self.last_left = left_ticks
        self.last_right = right_ticks
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pwm_a.value = 0
            node.pwm_b.value = 0
            node.ain1.off(); node.ain2.off()
            node.bin1.off(); node.bin2.off()
            node.pwm_a.close()
            node.pwm_b.close()
            node.ain1.close(); node.ain2.close()
            node.bin1.close(); node.bin2.close()
            node.left_encoder.close()
            node.right_encoder.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
