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
PIN_DIR = 4
FREQ = 2000        # 1 kHz PWM
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
        lgpio.gpio_claim_output(self.gpio_handle, PIN_DIR)
        sleep(0.1)

        # Start with 0 % duty (off)
        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, 0.0)
        self.get_logger().info("PWM at 1 kHz initialized, output = 0 %")
        
        # Initialize motion control state
        self.current_duty = 0.0
        self.current_dir = 0
        self.ramp_rate = 10.0  # percent per update (controls acceleration smoothness)

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        # Clamp speed command
        speed = max(-100.0, min(100.0, msg.speedproc))
        target_dir = 1 if speed < 0 else 0
        target_duty = abs(speed)

        # Smoothly ramp toward target_duty
        if target_duty > self.current_duty:
            self.current_duty = min(self.current_duty + self.ramp_rate, target_duty)
        else:
            self.current_duty = max(self.current_duty - self.ramp_rate, target_duty)

        # Handle safe direction changes
        if target_dir != self.current_dir and self.current_duty > 0:
            # Stop before reversing
            lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, 0)
            sleep(0.05)
            self.current_duty = 0

        # Apply new direction if needed
        if target_dir != self.current_dir:
            lgpio.gpio_write(self.gpio_handle, PIN_DIR, target_dir)
            self.current_dir = target_dir

        # Output PWM
        lgpio.tx_pwm(self.gpio_handle, PIN_ESC, FREQ, self.current_duty)

        self.get_logger().info(
            f'speedproc={speed:.1f} dir={'REV' if target_dir else 'FWD'} → duty={self.current_duty:.1f}%'
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
