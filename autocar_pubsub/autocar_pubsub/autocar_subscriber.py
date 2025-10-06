import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, PWMOutputDevice
from time import sleep

# Use lgpio backend
Device.pin_factory = LGPIOFactory()

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

        # Initialize PWM on pin 17
        self.esc = PWMOutputDevice(pin=17, frequency=50, initial_value=0.0)

        # Arm ESC / neutral signal
        self.get_logger().info("Arming ESC (neutral)...")
        self.esc.value = 0.0
        sleep(1.0)
        self.get_logger().info("ESC armed.")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        # Convert msg.speedproc to PWM duty cycle
        # Assuming speedproc is between -1.0 and 1.0
        pwm_value = max(0.0, min(1.0, (msg.speedproc + 1.0) / 2.0))
        self.esc.value = pwm_value

        self.get_logger().info(
            f'I heard: button="{msg.button}", '
            f'speed_proc={msg.speedproc}, '
            f'angle={msg.angle}, '
            f'pressed={msg.pressed}, '
            f'PWM value set to {pwm_value:.2f}'
        )

    def destroy_node(self):
        self.esc.close()
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
