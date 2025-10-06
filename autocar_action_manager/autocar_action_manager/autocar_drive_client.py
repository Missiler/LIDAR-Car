import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import curses

from autocar_action_interface.action import Drive


class DriveActionClient(Node):

    def __init__(self):
        super().__init__('Drive_action_client')
        self._action_client = ActionClient(self, Drive, 'Drive')

    def send_goal(self, keypress: str):
        """
        Send a new goal to the Drive action server with the given keypress string.
        """
        goal_msg = Drive.Goal()
        goal_msg.keypress = keypress

        # Ensure the server is ready
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal:"{keypress}"')

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Server response → status: '{result.status}'"
        )

    def feedback_callback(self, feedback_msg):
        """
        Optional: handle streaming feedback from the server.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')


def curses_main(stdscr):
#Kod tagen från GPT för att setta up hur man kan använda tangentbordet för att styra.
    rclpy.init()
    node = DriveActionClient()

    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.addstr(0, 0, "Press 'q' to quit. Use W/A/S/D or arrow keys to drive.")

    #Vad varje keypress innebär.
    key_map = {
        curses.KEY_UP:    "up",
        curses.KEY_DOWN:  "down",
        curses.KEY_LEFT:  "left",
        curses.KEY_RIGHT: "right",
        ord('w'):         "w",
        ord('a'):         "a",
        ord('s'):         "s",
        ord('d'):         "d",
    }

    try:
        while rclpy.ok():
            #Spinnar rclpy för att kunna köra programmet.
            rclpy.spin_once(node, timeout_sec=0.1)

            key = stdscr.getch()
            if key == -1:
                continue

            #För att stänga ner loopen
            if key in (ord('q'), ord('Q')):
                stdscr.addstr(1, 0, "Quitting...               ")
                break

            #Skicka den mappade knappen till 
            if key in key_map:
                command = key_map[key]
                node.send_goal(command)
                stdscr.addstr(1, 0, f"Sent command: {command}      ")
            else:
                #fallback: send printable character as a string
                if 32 <= key <= 126:
                    command = chr(key)
                    node.send_goal(command)
                    stdscr.addstr(1, 0, f"Sent other key: {command}    ")

    #När loopen stängs av pga: tryckt på q --> Stänger ner noden.
    finally:
        rclpy.shutdown()

#Eftersom att man vill köra i cruses, är main endast curses-metoden.
def main():
    curses.wrapper(curses_main)


if __name__ == '__main__':
    main()
