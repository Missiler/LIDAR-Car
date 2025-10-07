import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
import pigpio
from time import sleep

PIN_SERVO = 12
PIN_ESC = 18

class ESCServoNode(Node):
    def __init__(self):
        super().__init__('esc_servo_node')
        self.subscription = self.create_subscription(
            DrivingCommand,
            'topic',
            self.listener_callback,
            10
        )
        
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Start it with: sudo pigpiod")

        self.last_time = self.get_clock().now()
        self.min_interval = 0.05  # 20 Hz update limit
        
        self.pi_min = 500  # microseconds
        self.pi_max = 2000
        self.pi_center = 500  # neutral steering
        
        self.pi_speed = 1500
        
        self.pi.set_servo_pulsewidth(PIN_SERVO, self.pi_center)
        self.pi.set_servo_pulsewidth(PIN_ESC, self.pi_speed)
        
        self.pi.set_servo_pulsewidth(PIN_ESC, 2000)
        sleep(2)
        self.pi.set_servo_pulsewidth(PIN_ESC, 1000)
        sleep(2)

        self.get_logger().info("")

    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now
        
        
        servo_pulse = map_range(msg.angle,-90, 0, 500, 2000)
        esc_pulse = map_range(msg.speedproc,-100, 100, 1000, 2000)
        
        self.pi.set_servo_pulsewidth(PIN_ESC, esc_pulse)
        self.pi.set_servo_pulsewidth(PIN_SERVO, servo_pulse)
        
        self.get_logger().info(
            f"Speed%={msg.speedproc} angle={msg.angle}"
        )

    def destroy_node(self):
        self.get_logger().info("Shutting down ESC + Servo safely...")
        # Stop PWM outputs
        self.pi.set_servo_pulsewidth(PIN_SERVO, 0)
        self.pi.set_servo_pulsewidth(PIN_ESC, 0)
        self.pi.stop()
        self.pi.stop()

        super().destroy_node()


def map_range(x, in_min, in_max, out_min, out_max):
    """Linearly map a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    node = ESCServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
