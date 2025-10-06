import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
from rclpy.time import Time  # for comparing ROS time
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory()

from gpiozero import Servo
#Used on GPIO 17, look at GPIO-sheet to see.

esc = Servo(17)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            DrivingCommand,
            'topic',
            self.listener_callback,
            10
        )
        self.last_time = self.get_clock().now()  # remember last accepted time
        self.min_interval = 0.2  # seconds

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
        if msg.speedproc == 0:
            esc = 0
        else:
            esc = msg.speedproc/200
        print(esc)


def main(args=None):
    
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
