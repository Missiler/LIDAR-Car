import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, Servo
from time import sleep

# Use lgpio backend for low-level GPIO access
Device.pin_factory = LGPIOFactory()

# === ESC CONFIGURATION ===
# Adjust these as needed for your ESC and PWM range
ESC_PIN = 18              # Change to your ESC signal pin
ESC_MIN = -1.0            # Full reverse (for bidirectional ESC)
ESC_NEUTRAL = 0.0         # Stop / Neutral
ESC_MAX = 1.0             # Full forward
RAMP_STEP = 0.02          # Increment per ramp step
RAMP_DELAY = 0.05         # Delay between steps (seconds)

# Initialize ESC
esc = Servo(ESC_PIN, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000, frame_width=20.0/1000)


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

        self.current_speed = ESC_NEUTRAL

        # Arm ESC
        self.get_logger().info("Arming ESC (neutral)...")
        esc.value = ESC_NEUTRAL
        sleep(1.0)
        self.get_logger().info("ESC armed and ready.")

    def ramp_to_speed(self, target_speed):
        """Gradually move from current to target speed"""
        step = RAMP_STEP if target_speed > self.current_speed else -RAMP_STEP
        while abs(target_speed - self.current_speed) > abs(step):
            self.current_speed += step
            esc.value = self.current_speed
            sleep(RAMP_DELAY)
        # Final adjustment
        esc.value = target_speed
        self.current_speed = target_speed

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

        # Clamp and ramp safely
        target_speed = max(min(msg.speedproc, ESC_MAX), ESC_MIN)
        self.get_logger().info(f"Ramping to {target_speed:.2f}")
        self.ramp_to_speed(target_speed)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping ESC (neutral)...")
        esc.value = ESC_NEUTRAL
        sleep(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
