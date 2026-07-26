#!/usr/bin/env python3
"""PIR + RCWL movement sensing for the welfare check.

Continuously publishes raw sensor state, and on request runs a timed
settle-then-sample window and publishes a JSON verdict on ``/welfare_result``.

Two sensors are used because neither is trusted alone - see
``_verdict_from_samples`` for how their disagreement is resolved.
"""
import os
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"
from gpiozero import DigitalInputDevice

PIR_PIN = 25
RCWL_PIN = 16

# PIR sensors need to thermally stabilise after power-on; readings before this
# are unreliable, so a check requested during warmup returns the 'warming_up'
# verdict instead of a real answer and the caller is expected to retry.
WARMUP_SECONDS = 60.0

# A check is requested right after the robot has driven to the person, so the
# settle window discards the robot's own arrival motion before the sample window
# starts counting sensor activity.
# TODO(why): the specific 2.0 s / 10.0 s durations are not recorded anywhere.
SETTLE_SECONDS = 2.0
SAMPLE_SECONDS = 10.0

PUBLISH_HZ = 5.0

STATE_IDLE = 'idle'
STATE_SETTLING = 'settling'
STATE_SAMPLING = 'sampling'


class WelfareSensor(Node):
    """Publishes raw PIR/RCWL state and runs on-demand movement checks."""

    def __init__(self):
        super().__init__('welfare_sensor')

        self._free_sensor_pins()
        self._setup_sensors()
        self._setup_check_state()
        self._setup_interfaces()
        self._setup_timers()

        self.get_logger().info(
            f'Welfare Sensor Node Ready — PIR: GPIO{PIR_PIN}, RCWL: GPIO{RCWL_PIN}'
        )

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _free_sensor_pins(self):
        """Release the sensor pins from any previous owner.

        Must run before gpiozero claims them. An unclean shutdown or a reboot can
        leave the pins held by a dead process, and gpiozero then fails to
        construct the devices at all - this is what makes the node reboot-safe.

        The bare excepts are deliberate: every failure here is non-fatal, since
        the pins being un-held is exactly the normal case.
        """
        # Reboot-safe GPIO cleanup: free sensor pins before gpiozero claims them
        import lgpio
        try:
            h = lgpio.gpiochip_open(0)
            for pin in [PIR_PIN, RCWL_PIN]:
                try:
                    lgpio.gpio_free(h, pin)
                except:
                    pass
            lgpio.gpiochip_close(h)
        except:
            pass

    def _setup_sensors(self):
        """Claim the PIR and RCWL inputs and start the warmup clock."""
        self.pir = DigitalInputDevice(PIR_PIN)
        self.rcwl = DigitalInputDevice(RCWL_PIN)

        self.start_time = time.time()

    def _setup_check_state(self):
        """Initialise the settle/sample state machine to idle."""
        # --- Check state machine ---
        self.state = STATE_IDLE
        self.settle_end_time = 0.0
        self.sample_end_time = 0.0
        self.sample_pir_seen = False
        self.sample_rcwl_seen = False

    def _setup_interfaces(self):
        """Create the raw sensor publishers, the result publisher and the trigger."""
        # --- Publishers ---
        self.pir_pub = self.create_publisher(Bool, '/welfare/pir', 10)
        self.rcwl_pub = self.create_publisher(Bool, '/welfare/rcwl', 10)
        self.result_pub = self.create_publisher(String, '/welfare_result', 10)

        # --- Subscriber ---
        self.create_subscription(Empty, '/welfare_check', self._welfare_check_callback, 10)

    def _setup_timers(self):
        """Start the raw publishing timer and the check state machine tick."""
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_readings)
        self.create_timer(0.1, self._update_check_state)

    # ------------------------------------------------------------------
    # Raw sensor stream
    # ------------------------------------------------------------------

    def _in_warmup(self):
        """True while the PIR is still stabilising and its readings cannot be trusted."""
        return (time.time() - self.start_time) < WARMUP_SECONDS

    def _publish_readings(self):
        """Publish the instantaneous state of both sensors."""
        pir_msg = Bool()
        pir_msg.data = bool(self.pir.value)
        self.pir_pub.publish(pir_msg)

        rcwl_msg = Bool()
        rcwl_msg.data = bool(self.rcwl.value)
        self.rcwl_pub.publish(rcwl_msg)

    # ------------------------------------------------------------------
    # Welfare check state machine
    # ------------------------------------------------------------------

    def _welfare_check_callback(self, _msg):
        """Begin a settle-then-sample welfare check, unless one is already running."""
        if self._in_warmup():
            self._publish_result(bool(self.pir.value), bool(self.rcwl.value), 'warming_up')
            return

        if self.state != STATE_IDLE:
            self.get_logger().warn('Welfare check already in progress — ignoring trigger')
            return

        now = time.time()
        self.settle_end_time = now + SETTLE_SECONDS
        self.sample_end_time = self.settle_end_time + SAMPLE_SECONDS
        self.sample_pir_seen = False
        self.sample_rcwl_seen = False
        self.state = STATE_SETTLING
        self.get_logger().info('Welfare check triggered — settling')

    def _update_check_state(self):
        """Advance the check state machine; called on a fixed tick."""
        if self.state == STATE_IDLE:
            return

        now = time.time()

        if self.state == STATE_SETTLING:
            self._advance_settling(now)
            return

        if self.state == STATE_SAMPLING:
            self._advance_sampling(now)

    def _advance_settling(self, now):
        """Move on to sampling once the settle window has elapsed."""
        if now >= self.settle_end_time:
            self.state = STATE_SAMPLING
            self.get_logger().info('Welfare check settled — sampling')

    def _advance_sampling(self, now):
        """Latch any sensor activity, and finish once the sample window has elapsed."""
        self._latch_sensor_activity()

        if now >= self.sample_end_time:
            self._finish_sample_window()

    def _latch_sensor_activity(self):
        """Record that a sensor fired at any point during the sample window.

        Latched rather than sampled instantaneously because both sensors pulse -
        a person can be moving without either line being high at the moment the
        window happens to close.
        """
        if self.pir.value:
            self.sample_pir_seen = True
        if self.rcwl.value:
            self.sample_rcwl_seen = True

    def _finish_sample_window(self):
        """Publish the verdict for the completed sample window and return to idle."""
        pir_result = self.sample_pir_seen
        rcwl_result = self.sample_rcwl_seen

        verdict = self._verdict_from_samples(pir_result, rcwl_result)

        self._publish_result(pir_result, rcwl_result, verdict)
        self.state = STATE_IDLE

    def _verdict_from_samples(self, pir, rcwl):
        """Resolve the two sensors into a single verdict, trusting PIR over RCWL.

        The RCWL modules fitted are clones, with a sub-1 m effective range against
        an advertised ~7 m. RCWL firing alone is therefore not enough to call it
        movement - it yields 'uncertain' so the caller can escalate rather than
        record a false all-clear.
        """
        if pir:
            return 'movement'
        if rcwl:
            return 'uncertain'
        return 'no_movement'

    def _publish_result(self, pir, rcwl, verdict):
        """Publish the check outcome as JSON on /welfare_result."""
        payload = {'pir': pir, 'rcwl': rcwl, 'verdict': verdict}
        msg = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)
        self.get_logger().info(f'Welfare result: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = WelfareSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pir.close()
            node.rcwl.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
