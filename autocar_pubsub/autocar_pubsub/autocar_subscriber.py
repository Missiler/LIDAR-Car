import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from time import sleep
import lgpio

# === GPIO + ESC setup ===
ESC_PIN = 17          # PWM output pin
CHIP = 0               # Usually /dev/gpiochip0
PWM_FREQ = 50          # 50 Hz = 20 ms frame
ESC_NEUTRAL = 1.5e-3   # 1.5 ms = stop
ESC_MIN = 1.0e-3       # 1.0 ms = reverse
ESC_MAX = 2.0e-3       # 2.0 ms = full forward
RAMP_STEP = 0.02       # Speed ramp step (0-1 scale)
RAMP_DELAY = 0.05      # Time between steps

# open chip
chip = lgpio.gpiochip_open(CHIP)

# setup PWM on pin
lgpio.gpio_claim_output(chip, ESC_PIN)

def set_esc_throttle(duty_value):
    """
    duty_value: normalized -1.0 → +1.0
    converts to 1.0–2.0 ms pulse width at 50 Hz
    """
    pulse_width = ESC_NEUTRAL + duty_value * (ESC_MAX - ESC_NEUTRAL)
    duty_cycle = (pulse_width * PWM_FREQ) * 100.0  # percentage for lgpio
    lgpio.tx_pwm(chip, ESC_PIN, PWM_FREQ, duty_cycle)


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
        self.current_speed = 0.0

        # Arm ESC
        self.get_logger().info("Arming ESC (neutral)...")
        set_esc_throttle(0.0)
        sleep(2.0)
        self.get_logger().info("ESC armed and ready.")

    def ramp_to_speed(self, target_speed):
        step = RAMP_STEP if target_speed > self.current_speed else -RAMP_STEP
        while abs(target_speed - self.current_speed) > abs(step):
            self.current_speed += step
            set_esc_throttle(self.current_speed)
            sleep(RAMP_DELAY)
        set_esc_throttle(target_speed)
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

        # Clamp speed range
        target_speed = max(min(msg.speedproc, 1.0), -1.0)
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
        node.get_logger().info("Stopping ESC...")
        set_esc_throttle(0.0)
        sleep(1.0)
        lgpio.gpiochip_close(chip)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
