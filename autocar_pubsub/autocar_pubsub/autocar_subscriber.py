import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
import lgpio
from time import sleep, time

ESC_PIN = 17  # GPIO pin connected to ESC signal
CHIP = 0      # Usually 0 for Raspberry Pi

# Typical ESC calibration (adjust if needed)
NEUTRAL_US = 1500
MIN_US = 1000
MAX_US = 2000
FRAME_US = 20000

class ESCController:
    def __init__(self, chip, pin):
        self.chip = chip
        self.pin = pin
        self.handle = lgpio.gpiochip_open(chip)
        lgpio.gpio_claim_output(self.handle, pin)

    def set_pulse_us(self, pulse_width_us):
        # Clamp values
        pulse_width_us = max(MIN_US, min(MAX_US, pulse_width_us))
        lgpio.tx_pwm(self.handle, self.pin, 50, (pulse_width_us - 1000) / 10.0)
        # 50 Hz PWM → 20 ms frame; duty cycle = (pulse_us / 20000) * 100

    def cleanup(self):
        lgpio.gpiochip_close(self.handle)

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
        self.esc = ESCController(CHIP, ESC_PIN)

        # Arm ESC
        self.get_logger().info("Arming ESC at neutral...")
        self.esc.set_pulse_us(NEUTRAL_US)
        sleep(2.0)
        self.get_logger().info("ESC armed and ready.")

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

        # Map speed_proc (-100..100) → pulse width (1000–2000 µs)
        pulse = int(NEUTRAL_US + (msg.speedproc / 100.0) * 500)
        self.esc.set_pulse_us(pulse)
        self.get_logger().info(f"ESC pulse set to {pulse} µs")

    def destroy_node(self):
        self.esc.set_pulse_us(NEUTRAL_US)
        self.esc.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
