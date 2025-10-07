import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
import pigpio
from time import sleep

PIN_SERVO = 12
PIN_ESC = 18

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

        self.last_time = self.get_clock().now()
        self.min_interval = 0.05  # 20 Hz update limit
        
        self.servo_min = 0  # microseconds
        self.servo_max = 2000
        self.servo_center = 1500  # neutral steering
        
        self.pi.set_servo_pulsewidth(PIN_SERVO, self.servo_center)

        self.get_logger().info("")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now
        
        steering = max(-1.0, min(1.0, msg.angle))
        servo_pulse = self.servo_min + (steering + 1) * (self.servo_max - self.servo_min) / 2
        self.pi.set_servo_pulsewidth(PIN_SERVO, servo_pulse)
        
        self.get_logger().info(
            f""
        )

    def destroy_node(self):
        self.get_logger().info("Shutting down ESC + Servo safely...")
        # Stop PWM outputs
        self.pi.set_servo_pulsewidth(PIN_SERVO, 0)
        self.pi.set_servo_pulsewidth(PIN_ESC, 0)
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
