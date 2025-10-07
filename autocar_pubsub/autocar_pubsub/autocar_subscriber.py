import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import lgpio
from time import sleep

# Use lgpio backend
Device.pin_factory = LGPIOFactory()

PIN_ESC = 17       # PWM output pin (BCM numbering)
FREQ = 1000        # 1 kHz PWM
CHIP = 0           # Usually 0 on Raspberry Pi

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
        self.min_interval = 0.05  # seconds between updates (20 Hz)

        # Initialize PWM pin
        self.gpio_handle = lgpio.gpiochip_open(CHIP)
        lgpio.gpio_claim_output(self.gpio_handle, PIN_ESC)
        sleep(0.1)

        # Start with 0 % duty (off)
        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, 0.0)
        self.get_logger().info("PWM at 1 kHz initialized, output = 0 %")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        # Convert speedproc (expected −100 → 100) to duty 0–100 %
        duty_percent = (max(-100.0, min(100.0, msg.speedproc)))
        if msg.speedproc == 0:
            duty_percent = 0

        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, duty_percent)

        self.get_logger().info(
            f'speedproc={msg.speedproc:.1f} → duty={duty_percent:.1f} %'
        )

    def destroy_node(self):
        self.get_logger().info("Stopping PWM and shutting down...")
        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, 0, 0)
        lgpio.gpiochip_close(self.gpio_handle)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
