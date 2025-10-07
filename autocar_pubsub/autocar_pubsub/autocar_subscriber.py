import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import lgpio
from time import sleep

# Use lgpio backend
Device.pin_factory = LGPIOFactory()

PIN_ESC = 18        # PWM output pin (BCM numbering)
FREQ = 50           # 50 Hz typical for servo/ESC control
CHIP = 0            # Usually 0 on Raspberry Pi

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

        # Initialize GPIO
        self.gpio_handle = lgpio.gpiochip_open(CHIP)
        lgpio.gpio_claim_output(self.gpio_handle, PIN_ESC)
        sleep(0.1)

        # Arm ESC: send neutral pulse for safety
        duty_neutral = self._pulse_to_duty(1.5)
        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, duty_neutral)
        self.get_logger().info(f"ESC armed at neutral (1.5 ms → {duty_neutral:.2f}% duty)")

    def _pulse_to_duty(self, pulse_ms):
        """Convert pulse width (ms) to duty % for 50 Hz PWM."""
        period_ms = 1000 / FREQ  # 20 ms
        return (pulse_ms / period_ms) * 100

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        # Clamp and map [-100, 100] → [1.0, 2.0] ms
        speed = max(-100.0, min(100.0, msg.speedproc))
        pulse_ms = 1.5 + (speed / 100.0) * 0.5  # 1.0–2.0 ms
        duty = self._pulse_to_duty(pulse_ms)

        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, duty)

        self.get_logger().info(
            f"speedproc={speed:.1f} → pulse={pulse_ms:.3f} ms → duty={duty:.2f}%"
        )

    def destroy_node(self):
        self.get_logger().info("Stopping ESC PWM and shutting down...")
        # Send neutral before shutdown
        duty_neutral = self._pulse_to_duty(1.5)
        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, duty_neutral)
        sleep(0.5)
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
