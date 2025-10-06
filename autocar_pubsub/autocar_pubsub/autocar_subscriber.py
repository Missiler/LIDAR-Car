import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, Servo
from time import sleep

# Use lgpio backend
Device.pin_factory = LGPIOFactory()

# Configure servo for ESC (typical 1–2 ms pulse)
esc = Servo(17, min_pulse_width=1.2e-3, max_pulse_width=1.8e-3, frame_width=20e-3)

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
        self.min_interval = 0.2  # seconds

        # Arm ESC
        self.get_logger().info("Arming ESC (neutral)...")
        esc.value = 0.0
        sleep(1.0)
        self.get_logger().info("ESC armed.")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        self.get_logger().info(
            f'I heard: button="{msg.button}", '
            f'speed_proc={msg.speedproc}, '
            f'angle={msg.angle}, '
            f'pressed={msg.pressed}'
        )

        # Map speed_proc (-100..100) to servo value (-1..1)
        esc_val = (msg.speedproc / 100.0) * 0.5  # limit throttle range
        esc.value = esc_val
        self.get_logger().info(f"ESC output set to {esc_val:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
