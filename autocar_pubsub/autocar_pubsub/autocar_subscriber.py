import rclpy
from rclpy.node import Node
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, PWMOutputDevice

# Use lgpio backend for performance
Device.pin_factory = LGPIOFactory()

class PWMNode(Node):
    def __init__(self):
        super().__init__('pwm_node')
        self.get_logger().info("Starting PWM output on GPIO17...")

        # Create PWM output on GPIO17 at 50 Hz (typical for servos/ESC)
        self.pwm = PWMOutputDevice(pin=17, frequency=50, initial_value=0.0)

        # Set initial duty cycle (0.0–1.0)
        self.pwm.value = 0.5  # 50% duty (adjust as needed)
        self.get_logger().info("PWM active at 50% duty cycle.")

    def destroy_node(self):
        self.pwm.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
