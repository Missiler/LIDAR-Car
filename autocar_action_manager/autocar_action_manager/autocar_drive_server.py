import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from autocar_action_interface.action import Drive


class DriveActionServer(Node):

    def __init__(self):
        super().__init__('Drive_action_server')
        self._action_server = ActionServer(
            self,
            Drive,
            'Drive',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        """
        Called when the client sends a new goal (keypress).
        """
        key = goal_handle.request.keypress
        self.get_logger().info(f'Executing goal with keypress: {key!r}')

        # Create feedback if needed
        feedback_msg = Drive.Feedback()

        # Process the keypress
        # (Here you decide what 'driving' means for your robot)
        driving_keys = ['w', 'a', 's', 'd', 'up', 'down', 'left', 'right']

        result = Drive.Result()
        if key.lower() in driving_keys:
            result.status = f"Driving command received: {key}"
            feedback_msg.isdriving = True
        else:
            result.status = f"Stopped or unknown command: {key}"
            feedback_msg.isdriving = False

        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DriveActionServer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
