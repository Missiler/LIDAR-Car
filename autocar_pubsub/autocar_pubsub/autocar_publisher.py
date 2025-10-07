import rclpy
from rclpy.node import Node
from autocar_interface.msg import DrivingCommand
'''
string button
int32 speedproc
int32 angle
bool pressed
'''
import curses
import time


class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(DrivingCommand, 'topic', 10)
        
        #Startvärden.
        self.speedproc = 0
        self.angle = 0

    def publish_state(self, button_name: str, speedproc: int, angle: int, pressed: bool):
        #De olika meddelanden som ska skickas, det är det som skickas i koden.
        msg = DrivingCommand()
        msg.button = button_name
        msg.speedproc = speedproc
        msg.angle = angle
        msg.pressed = pressed
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing button="{button_name}", speedproc={speedproc}, angle={angle}, pressed={pressed}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    
    #Genererad av ChatGPT:
    def curses_loop(stdscr):
        stdscr.nodelay(True)     #non-blocking
        stdscr.keypad(True)      #get KEY_UP/KEY_DOWN etc.
        stdscr.addstr(0, 0,
            "UP/DOWN = speed ±5, LEFT/RIGHT = steer, t/g = extra, q = quit. (samples every 0.5s)")
        stdscr.refresh()

        period = 0.2
        next_time = time.monotonic()

        while rclpy.ok():
            now = time.monotonic()
            if now < next_time:
                rclpy.spin_once(node, timeout_sec=0.05)
                continue
            next_time = now + period

            # ---- Drain ALL pending key events; keep only the most recent ----
            #Endast det nuvarande och inte tidigare knapptryck.
            last = -1
            while True:
                ch = stdscr.getch()
                if ch == -1:
                    break
                last = ch

            button = ""
            pressed = False
            #Olika inputs:
            if last == ord('q'):
                return  # exit cleanly

            if last == curses.KEY_UP:
                node.speedproc = min(node.speedproc + 2, 100)
                button = "UP"; pressed = True
            elif last == curses.KEY_DOWN:
                node.speedproc = max(node.speedproc - 2, -100)
                button = "DOWN"; pressed = True
            elif last == curses.KEY_LEFT:
                node.angle = max(node.angle - 5, -45)
                button = "LEFT"; pressed = True
            elif last == curses.KEY_RIGHT:
                node.angle = min(node.angle + 5, 45)
                button = "RIGHT"; pressed = True
            elif last == ord('t'):
                button = "T"; pressed = True
            elif last == ord('g'):
                button = "G"; pressed = True
            elif last == -1:
                # No new key since last tick → do NOT change speed.
                # Optional: gently recenter steering
                node.angle = int(node.angle * 1)
            else:
                # Unhandled key: ignore (don’t change speed/angle)
                pass

            # Publish once per tick
            node.publish_state(button, node.speedproc, node.angle, pressed)

    curses.wrapper(curses_loop)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
