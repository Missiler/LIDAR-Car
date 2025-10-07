
from autocar_interface.msg import DrivingCommand
import rclpy
from rclpy.node import Node
import pigpio
from time import sleep

PIN_ESC = 18
PIN_SERVO = 12
FREQ = 50  # Hz, standard for ESC/servo PWM

class ESCServoNode(Node):
    def __init__(self):
        super().__init__('esc_servo_node')
        self.subscription = self.create_subscription(
            DrivingCommand,
            'topic',
            self.listener_callback,
            10
        )

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Start it with: sudo pigpiod")

        # Initialize ESC and Servo at neutral positions
        self.neutral_pulse = 1500
        self.pi.set_servo_pulsewidth(PIN_ESC, self.neutral_pulse)
        self.pi.set_servo_pulsewidth(PIN_SERVO, self.neutral_pulse)

        self.last_time = self.get_clock().now()
        self.min_interval = 0.05  # 20 Hz update limit

        self.get_logger().info("ESC + Servo initialized with pigpio (neutral = 1500 µs)")

    def _map_value(self, value, in_min, in_max, out_min, out_max):
        """Linear mapping function."""
        value = max(in_min, min(in_max, value))  # clamp
        return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        # --- ESC control ---
        speed = max(-100.0, min(100.0, msg.speedproc))
        esc_pulse = self._map_value(speed, -100, 100, 1000, 2000)
        self.pi.set_servo_pulsewidth(PIN_ESC, esc_pulse)

        # --- Servo control ---
        angle = max(-45.0, min(45.0, msg.angle))
        servo_pulse = self._map_value(angle, -45, 45, 1100, 1900)
        self.pi.set_servo_pulsewidth(PIN_SERVO, servo_pulse)

        self.get_logger().info(
            f"speed={speed:.1f}% → ESC={esc_pulse:.0f} µs | angle={angle:.1f}° → SERVO={servo_pulse:.0f} µs"
        )

    def destroy_node(self):
        self.get_logger().info("Shutting down ESC + Servo safely...")
        self.pi.set_servo_pulsewidth(PIN_ESC, self.neutral_pulse)
        self.pi.set_servo_pulsewidth(PIN_SERVO, self.neutral_pulse)
        sleep(0.5)
        self.pi.set_servo_pulsewidth(PIN_ESC, 0)
        self.pi.set_servo_pulsewidth(PIN_SERVO, 0)
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESCServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
