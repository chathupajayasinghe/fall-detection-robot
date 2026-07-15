#!/usr/bin/env python3
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

WARMUP_SECONDS = 60.0
SETTLE_SECONDS = 2.0
SAMPLE_SECONDS = 10.0
PUBLISH_HZ = 5.0

STATE_IDLE = 'idle'
STATE_SETTLING = 'settling'
STATE_SAMPLING = 'sampling'


class WelfareSensor(Node):
    def __init__(self):
        super().__init__('welfare_sensor')

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

        self.pir = DigitalInputDevice(PIR_PIN)
        self.rcwl = DigitalInputDevice(RCWL_PIN)

        self.start_time = time.time()

        # --- Check state machine ---
        self.state = STATE_IDLE
        self.settle_end_time = 0.0
        self.sample_end_time = 0.0
        self.sample_pir_seen = False
        self.sample_rcwl_seen = False

        # --- Publishers ---
        self.pir_pub = self.create_publisher(Bool, '/welfare/pir', 10)
        self.rcwl_pub = self.create_publisher(Bool, '/welfare/rcwl', 10)
        self.result_pub = self.create_publisher(String, '/welfare_result', 10)

        # --- Subscriber ---
        self.create_subscription(Empty, '/welfare_check', self.welfare_check_callback, 10)

        # --- Timers ---
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_readings)
        self.create_timer(0.1, self.update_check_state)

        self.get_logger().info(
            f'Welfare Sensor Node Ready — PIR: GPIO{PIR_PIN}, RCWL: GPIO{RCWL_PIN}'
        )

    def in_warmup(self):
        return (time.time() - self.start_time) < WARMUP_SECONDS

    def publish_readings(self):
        pir_msg = Bool()
        pir_msg.data = bool(self.pir.value)
        self.pir_pub.publish(pir_msg)

        rcwl_msg = Bool()
        rcwl_msg.data = bool(self.rcwl.value)
        self.rcwl_pub.publish(rcwl_msg)

    def welfare_check_callback(self, msg):
        if self.in_warmup():
            self.publish_result(bool(self.pir.value), bool(self.rcwl.value), 'warming_up')
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

    def update_check_state(self):
        if self.state == STATE_IDLE:
            return

        now = time.time()

        if self.state == STATE_SETTLING:
            if now >= self.settle_end_time:
                self.state = STATE_SAMPLING
                self.get_logger().info('Welfare check settled — sampling')
            return

        if self.state == STATE_SAMPLING:
            if self.pir.value:
                self.sample_pir_seen = True
            if self.rcwl.value:
                self.sample_rcwl_seen = True

            if now >= self.sample_end_time:
                pir_result = self.sample_pir_seen
                rcwl_result = self.sample_rcwl_seen

                if pir_result:
                    verdict = 'movement'
                elif rcwl_result:
                    verdict = 'uncertain'
                else:
                    verdict = 'no_movement'

                self.publish_result(pir_result, rcwl_result, verdict)
                self.state = STATE_IDLE

    def publish_result(self, pir, rcwl, verdict):
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
