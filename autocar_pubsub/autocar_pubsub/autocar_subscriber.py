import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
import pigpio
from time import sleep

ESC_PIN = 17
pi = pigpio.pi()
pi.set_mode(ESC_PIN, pigpio.OUTPUT)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            DrivingCommand,
            'topic',
            self.listener_callback,
            10
        )
        self.last_time = self.get_clock().now()
        self.min_interval = 0.2

        # Arm ESC at neutral (1.5 ms)
        self.get_logger().info("Arming ESC (neutral)...")
        pi.set_servo_pulsewidth(ESC_PIN, 1500)
        sleep(2)
        self.get_logger().info("ESC armed.")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        speed = msg.speedproc  # assume -100..100 or 0..100
        # Map 0..100 → 1500–2000 µs  (only forward)
        pulse = 1500 + (speed * 5)  # 100 → 2000 µs
        # For bidirectional ESCs, use: pulse = 1500 + (speed * 5) if -100..100

        pulse = max(1000, min(2000, pulse))
        pi.set_servo_pulsewidth(ESC_PIN, pulse)
        self.get_logger().info(f"Speed {speed}, PWM {pulse} µs")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    pi.set_servo_pulsewidth(ESC_PIN, 1500)
    pi.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
