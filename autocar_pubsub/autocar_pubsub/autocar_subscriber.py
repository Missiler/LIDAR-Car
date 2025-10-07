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
        self.min_interval = 0.2  # 20 Hz update limit

        # Servo parameters
        self.servo_min = 500
        self.servo_max = 2000
        self.servo_center = (self.servo_min + self.servo_max) / 2

        # ESC (throttle) parameters
        self.esc_min = 1390
        self.esc_max = 1450
        self.esc_neutral = (self.esc_min + self.esc_max) / 2
        
        # Center servo and ESC on startup
        self.pi.set_servo_pulsewidth(PIN_ESC, 1000)
        sleep(2)
        self.pi.set_servo_pulsewidth(PIN_SERVO, self.servo_center)
        self.pi.set_servo_pulsewidth(PIN_ESC, 1440)
        sleep(0.1)

        self.get_logger().info("Initialized, servo and ESC to neutral / center.")
        
    def listener_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return
        self.last_time = now

        # Map steering: assuming msg.angle in [-90, 0] or some range
        servo_pulse = map_range(msg.angle, -90, 0, self.servo_min, self.servo_max)
        # Map speedproc: assuming msg.speedproc in [-100, 100]
        esc_pulse = map_range(msg.speedproc, -100, 100, self.esc_min, self.esc_max)

        self.get_logger().info(f"Mapped pulses: servo={servo_pulse:.1f} µs, esc={esc_pulse:.1f} µs")

        # Set outputs
        self.pi.set_servo_pulsewidth(PIN_SERVO, servo_pulse)
        self.pi.set_servo_pulsewidth(PIN_ESC, esc_pulse)
            

    def destroy_node(self):
        self.get_logger().info("Shutting down ESC + Servo safely...")
        self.pi.set_servo_pulsewidth(PIN_SERVO, 0)
        self.pi.set_servo_pulsewidth(PIN_ESC, 0)
        self.pi.stop()
        super().destroy_node()



def map_range(x, in_min, in_max, out_min, out_max):
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
