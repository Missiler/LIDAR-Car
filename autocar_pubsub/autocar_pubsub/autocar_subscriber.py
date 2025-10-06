import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import lgpio
from time import sleep

# Use lgpio backend
Device.pin_factory = LGPIOFactory()

PIN_ESC = 17       # ESC signal pin (BCM numbering)
FREQ = 50          # 50 Hz -> 20 ms period
CHIP = 0           # Usually 0 on Raspberry Pi

# Convert pulse width (ms) to duty cycle (0–1)
def pulse_to_duty(pulse_ms):
    period_ms = 1000 / FREQ
    return pulse_ms / period_ms


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

        # Initialize ESC pin
        self.handle = lgpio.gpiochip_open(CHIP)
        lgpio.gpio_claim_output(self.handle, PIN_ESC)

        # Perform ESC calibration once
        self.get_logger().info("Starting ESC calibration...")
        lgpio.tx_pwm(self.handle, PIN_ESC, FREQ, pulse_to_duty(2.0) * 100)  # Full throttle
        sleep(2.0)
        lgpio.tx_pwm(self.handle, PIN_ESC, FREQ, pulse_to_duty(1.0) * 100)  # Full reverse
        sleep(2.0)
        lgpio.tx_pwm(self.handle, PIN_ESC, FREQ, pulse_to_duty(1.5) * 100)  # Neutral
        sleep(2.0)
        self.get_logger().info("ESC calibration done. Arming neutral...")
        sleep(1.0)

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        # Clamp speed to safe range (-50 to 50)
        safe_speed = max(-50.0, min(50.0, msg.speedproc))

        # Map speedproc (-50 to 50) to pulse width 1.3–1.7 ms
        pulse_ms = 1.5 + (safe_speed / 50.0) * 0.2
        duty = pulse_to_duty(pulse_ms)

        # Send PWM signal
        lgpio.tx_pwm(self.handle, PIN_ESC, FREQ, duty * 100)

        self.get_logger().info(
            f'speedproc={msg.speedproc:.1f} (clamped={safe_speed:.1f}) → '
            f'pulse={pulse_ms:.2f} ms (duty={duty*100:.1f}%)'
        )

    def destroy_node(self):
        self.get_logger().info("Stopping ESC PWM and shutting down...")
        lgpio.tx_pwm(self.handle, PIN_ESC, 0, 0)
        lgpio.gpiochip_close(self.handle)
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
