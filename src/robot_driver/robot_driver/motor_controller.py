#!/usr/bin/env python3
"""Motor drive and wheel-encoder odometry for the tracked chassis.

Subscribes ``/cmd_vel``, converts each Twist into per-side motor duty cycles,
and independently integrates the wheel encoders into ``/odom`` plus the
``odom -> base_link`` transform at 20 Hz.

Nearly every constant here was established by physical measurement on the robot
rather than taken from a datasheet, and several deliberately disagree with the
spec sheets - see the individual comments before changing any of them.
"""
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
from gpiozero import PWMOutputDevice, DigitalOutputDevice, RotaryEncoder

# Per-side output scaling. Both 1.0, so currently inert; kept as the hook for
# per-side scaling should one motor ever need it independently of RIGHT_TRIM.
LEFT_SPEED_SCALE  = 1.0
RIGHT_SPEED_SCALE = 1.0

# Motor deadband. Below this duty cycle the motors buzz but do not turn, so any
# non-zero command is rescaled into [MIN_PWM, 1.0] rather than being allowed to
# fall into the dead zone. See _control_motor.
MIN_PWM = 0.35

# Proportional gain for closed-loop straight-line encoder correction.
# Deliberately zeroed: a sign-flip bug in this correction made the robot pivot
# instead of driving straight. The direction clamp in _apply_straight_correction
# was added to guard that failure. At KP = 0 the whole correction is inert, but
# it is disabled code rather than dead code - do not delete it.
STRAIGHT_KP = 0.0

# Fixed open-loop trim: right side runs slightly faster than left per calibration.
# With STRAIGHT_KP at 0.0 this is the ONLY active straightness compensation on
# the robot. Open-loop test measured 2.7 cm of drift over 154 cm.
RIGHT_TRIM = 0.97

# Diagonal 6×6 covariance matrices (row-major, 36 elements).
# Unmeasured axes (z, roll, pitch, lateral velocity) carry 1e9 (effectively unknown).
_POSE_COV = [
    0.001, 0.0,   0.0, 0.0,   0.0,   0.0,
    0.0,   0.001, 0.0, 0.0,   0.0,   0.0,
    0.0,   0.0,   1e9, 0.0,   0.0,   0.0,
    0.0,   0.0,   0.0, 1e9,   0.0,   0.0,
    0.0,   0.0,   0.0, 0.0,   1e9,   0.0,
    0.0,   0.0,   0.0, 0.0,   0.0,   0.001,
]
_TWIST_COV = [
    0.001, 0.0,   0.0, 0.0,   0.0,   0.0,
    0.0,   1e9,   0.0, 0.0,   0.0,   0.0,
    0.0,   0.0,   1e9, 0.0,   0.0,   0.0,
    0.0,   0.0,   0.0, 1e9,   0.0,   0.0,
    0.0,   0.0,   0.0, 0.0,   1e9,   0.0,
    0.0,   0.0,   0.0, 0.0,   0.0,   0.001,
]

class MotorController(Node):
    """Drives both motors from /cmd_vel and publishes encoder odometry."""

    def __init__(self):
        super().__init__('motor_controller')

        self._free_motor_pins()
        self._setup_motor_pins()
        self._setup_encoders()
        self._setup_robot_geometry()
        self._setup_odometry_state()
        self._setup_control_state()
        self._setup_interfaces()
        self._setup_timers()

        self.get_logger().info('🤖 Motor+Odometry Node Ready — Pins: 12,5,6,13,23,24')

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _free_motor_pins(self):
        """Release the motor pins from any previous owner.

        Must run before gpiozero claims them. A reboot or an unclean shutdown can
        leave the pins held by a dead process, and gpiozero then fails to
        construct the devices at all - this is what makes the node reboot-safe.

        The bare excepts are deliberate: every failure here is non-fatal, since
        the pins being un-held is exactly the normal case.
        """
        # Reboot-safe GPIO cleanup: free all motor pins before gpiozero claims them
        import lgpio
        try:
            h = lgpio.gpiochip_open(0)
            for pin in [12, 5, 6, 13, 23, 24]:
                try:
                    lgpio.gpio_free(h, pin)
                except:
                    pass
            lgpio.gpiochip_close(h)
        except:
            pass

    def _setup_motor_pins(self):
        """Claim the PWM and direction pins for both motor channels."""
        # --- Motor Pins (Channel A: Left, Channel B: Right) ---
        self.pwm_a = self._create_pwm_device(12)
        self.ain1  = DigitalOutputDevice(5)
        self.ain2  = DigitalOutputDevice(6)

        self.pwm_b = self._create_pwm_device(13)
        self.bin1  = DigitalOutputDevice(23)
        self.bin2  = DigitalOutputDevice(24)

    def _setup_encoders(self):
        """Claim both quadrature encoders.

        Channel order matters: RotaryEncoder(10, 22) and RotaryEncoder(22, 10)
        count in opposite directions.
        """
        # --- Encoder Pins ---
        # lgpio pin factory registers GPIO edge-detect callbacks for both A/B channels,
        # so .steps is updated in real time by hardware interrupts — not by the odom timer.
        self.left_encoder  = RotaryEncoder(17, 27, max_steps=0, bounce_time=None)
        self.right_encoder = RotaryEncoder(10, 22, max_steps=0, bounce_time=None)

    def _setup_robot_geometry(self):
        """Set the measured chassis constants used by the odometry maths.

        All three were established by physical measurement over repeated trials
        and deliberately disagree with the datasheet figures:

        wheel_separation - the physical track centre-to-centre is 0.135 m, but
            tank tracks scrub sideways when turning, so commanded 90 degree
            rotations were measuring 114-115 degrees. Widening the effective
            separation to 0.17 m compensates.
        wheel_radius - encoder calibration over two 1.0 m tape-measured drives
            gave 1343 and 1389 ticks (~1360 ticks/m).
        ticks_per_rev - verified by hand-rotating one wheel a full turn, giving
            221-235 across trials.
        """
        # --- Robot Parameters ---
        self.wheel_separation = 0.17  # meters (effective, not physical — see docstring)
        self.wheel_radius = 0.027      # meters (from encoder calibration)
        self.ticks_per_rev = 235       # measured by hand-rotating one wheel a full turn

    def _setup_odometry_state(self):
        """Zero the integrated pose and latch the encoder counts to start from."""
        # --- Odometry State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = self.left_encoder.steps
        self.last_right = self.right_encoder.steps
        self.last_time = time.time()

    def _setup_control_state(self):
        """Initialise the straight-line corrector, command watchdog and log throttle."""
        # --- Closed-loop straight-line correction state ---
        self.left_ticks_per_sec  = 0.0
        self.right_ticks_per_sec = 0.0
        self.was_turning = False

        # --- Command timeout ---
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5  # seconds

        # --- Debug tick logging (every 1 s at 20 Hz) ---
        self._log_tick = 0

    def _setup_interfaces(self):
        """Create the odometry publisher, TF broadcaster and /cmd_vel subscription."""
        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

    def _setup_timers(self):
        """Start the 20 Hz odometry timer and the /cmd_vel watchdog."""
        # --- Timer for odometry ---
        self.create_timer(0.05, self._update_odometry)

        # --- Timer for cmd_vel timeout watchdog ---
        self.create_timer(0.05, self._check_cmd_timeout)

    def _create_pwm_device(self, pin):
        """Construct a PWM output, retrying once after forcibly freeing the pin.

        The first attempt can fail if the pin is still held from a previous run;
        the retry drops the gpiozero pin factory (falling back to freeing the pin
        through lgpio directly) and tries again before giving up.
        """
        for attempt in range(2):
            try:
                return PWMOutputDevice(pin, frequency=1000, initial_value=0)
            except Exception as exc:
                self.get_logger().warn(
                    f'PWM pin {pin} init failed on attempt {attempt + 1}: {exc}'
                )
                if attempt == 0:
                    try:
                        from gpiozero import Device
                        Device._default_pin_factory().close()
                    except:
                        import lgpio
                        try:
                            h = lgpio.gpiochip_open(0)
                            lgpio.gpio_free(h, pin)
                            lgpio.gpiochip_close(h)
                        except:
                            pass
                    time.sleep(0.1)
                else:
                    raise

    # ------------------------------------------------------------------
    # Drive control
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg):
        """Convert a Twist into per-side motor commands and drive both motors."""
        self.last_cmd_time = time.time()
        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        left_speed, right_speed = self._twist_to_wheel_speeds(linear_x, angular_z)
        left_speed, right_speed = self._apply_rotation_boost(
            left_speed, right_speed, linear_x, angular_z)
        left_speed, right_speed = self._apply_straight_correction(
            left_speed, right_speed, linear_x, angular_z)
        right_speed = self._apply_right_trim(right_speed)

        self._drive_both_motors(left_speed, right_speed)

    def _twist_to_wheel_speeds(self, linear_x, angular_z):
        """Differential-drive kinematics: split a Twist into left/right speeds."""
        left_speed  = (linear_x - (angular_z * self.wheel_separation / 2.0)) * LEFT_SPEED_SCALE
        right_speed = (linear_x + (angular_z * self.wheel_separation / 2.0)) * RIGHT_SPEED_SCALE
        return left_speed, right_speed

    def _apply_rotation_boost(self, left_speed, right_speed, linear_x, angular_z):
        """Boost pure pivots, where tank tracks must scrub sideways to rotate.

        The x1.5 exists because tank tracks fight stiction when turning on the
        spot; without it the commanded rotation stalls. Only applies to rotation
        with no forward component - a turn while driving already has forward
        motion helping the tracks break loose.
        """
        if angular_z != 0.0 and linear_x == 0.0:
            left_speed  *= 1.5
            right_speed *= 1.5
        return left_speed, right_speed

    def _apply_straight_correction(self, left_speed, right_speed, linear_x, angular_z):
        """Closed-loop straight-line correction, currently inert at STRAIGHT_KP = 0.

        Only runs during pure forward/backward motion. The first cycle after a
        turn is skipped because stale turn-phase tick rates would produce a
        massive error and drive left_speed negative.
        """
        # Closed-loop correction: only during pure forward/backward motion.
        # Skip the first cycle after a turn — stale turn-phase tick rates would
        # produce a massive error and drive left_speed negative.
        if angular_z == 0.0 and linear_x != 0.0:
            if not self.was_turning:
                error = self.left_ticks_per_sec - self.right_ticks_per_sec
                left_speed  -= STRAIGHT_KP * error
                right_speed += STRAIGHT_KP * error
                # Correction must not flip either wheel's direction relative
                # to the commanded linear_x, or the deadband rescale in
                # _control_motor will spin that wheel in reverse and pivot
                # the robot instead of driving straight.
                if linear_x > 0.0:
                    left_speed  = max(left_speed, 0.0)
                    right_speed = max(right_speed, 0.0)
                else:
                    left_speed  = min(left_speed, 0.0)
                    right_speed = min(right_speed, 0.0)
            self.was_turning = False
        else:
            self.was_turning = True
            self.left_ticks_per_sec  = 0.0
            self.right_ticks_per_sec = 0.0
        return left_speed, right_speed

    def _apply_right_trim(self, right_speed):
        """Apply the fixed right-side trim, the only active straightness compensation."""
        return right_speed * RIGHT_TRIM

    def _drive_both_motors(self, left_speed, right_speed):
        """Send the two per-side speeds to their motor channels."""
        self._control_motor(left_speed, self.pwm_a, self.ain1, self.ain2)
        self._control_motor(right_speed, self.pwm_b, self.bin1, self.bin2)

    def _check_cmd_timeout(self):
        """Stop both motors if /cmd_vel has gone quiet.

        Without this the robot keeps executing its last command indefinitely if
        the commanding node dies or the network drops.
        """
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self._control_motor(0.0, self.pwm_a, self.ain1, self.ain2)
            self._control_motor(0.0, self.pwm_b, self.bin1, self.bin2)

    def _control_motor(self, speed, pwm_dev, in1, in2):
        """Set one motor's direction pins and duty cycle.

        ``speed`` is treated directly as a normalised throttle and clamped to
        +/-1. Any non-zero magnitude is rescaled onto [MIN_PWM, 1.0] so that
        small commands still clear the motor deadband instead of merely buzzing;
        exactly zero cuts the drive entirely.
        """
        speed = max(min(speed, 1.0), -1.0)
        if speed == 0.0:
            in1.off(); in2.off()
            pwm_dev.value = 0.0
            return
        if speed > 0:
            in1.off(); in2.on()
        else:
            in1.on(); in2.off()
        duty = MIN_PWM + abs(speed) * (1.0 - MIN_PWM)
        pwm_dev.value = duty

    # ------------------------------------------------------------------
    # Odometry
    # ------------------------------------------------------------------

    def _update_odometry(self):
        """Integrate the encoders into a pose, then publish /odom and the TF."""
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return

        left_ticks, right_ticks, left_delta, right_delta = self._read_encoder_deltas()
        self._update_tick_rates(left_delta, right_delta, dt)

        left_dist, right_dist = self._compute_wheel_distances(left_delta, right_delta)
        vx, wz = self._integrate_pose(left_dist, right_dist, dt)

        odom = self._build_odom_msg(vx, wz)
        self.odom_pub.publish(odom)
        self._broadcast_transform(odom)

        self._log_encoder_steps(left_ticks, right_ticks)

        self.last_left = left_ticks
        self.last_right = right_ticks
        self.last_time = now

    def _read_encoder_deltas(self):
        """Read both encoders, returning absolute counts and deltas since last update."""
        left_ticks = self.left_encoder.steps
        right_ticks = self.right_encoder.steps
        left_delta = left_ticks - self.last_left
        right_delta = right_ticks - self.last_right
        return left_ticks, right_ticks, left_delta, right_delta

    def _update_tick_rates(self, left_delta, right_delta, dt):
        """Record per-side tick rates, the feedback signal for the straight-line corrector."""
        self.left_ticks_per_sec  = left_delta  / dt
        self.right_ticks_per_sec = right_delta / dt

    def _compute_wheel_distances(self, left_delta, right_delta):
        """Convert encoder tick deltas into metres travelled by each track."""
        left_dist  = left_delta  / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)
        right_dist = right_delta / self.ticks_per_rev * (2 * math.pi * self.wheel_radius)
        return left_dist, right_dist

    def _integrate_pose(self, left_dist, right_dist, dt):
        """Advance x/y/theta by this step's motion and return the twist velocities.

        Note the heading is applied before it is updated, so the step is
        integrated about the pose held at the start of the interval.
        """
        linear = (right_dist + left_dist) / 2.0
        angular = (right_dist - left_dist) / self.wheel_separation
        vx = linear / dt
        wz = angular / dt

        self.x += linear * math.cos(self.theta)
        self.y += linear * math.sin(self.theta)
        self.theta += angular

        return vx, wz

    def _build_odom_msg(self, vx, wz):
        """Assemble the Odometry message for the current pose and velocities."""
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom.pose.covariance = _POSE_COV
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = _TWIST_COV
        return odom

    def _broadcast_transform(self, odom):
        """Broadcast odom -> base_link, reusing the odometry message's stamp and rotation."""
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

    def _log_encoder_steps(self, left_ticks, right_ticks):
        """Log raw encoder counts once per second (every 20th call at 20 Hz)."""
        self._log_tick += 1
        if self._log_tick >= 20:
            self.get_logger().info(
                f'Encoder steps — left: {left_ticks}, right: {right_ticks}'
            )
            self._log_tick = 0

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
