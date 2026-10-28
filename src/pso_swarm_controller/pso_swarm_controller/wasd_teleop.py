#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

msg = """
Reading from the keyboard and Publishing to /cmd_vel_keyboard!
---------------------------
Moving around:
   w
a  s  d
q  e (rotate)

space : force stop
CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    'q': (0, 0, 0, 1),
    'e': (0, 0, 0, -1),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_keyboard', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 1.0
        self.turn = 1.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.get_logger().info(msg)

    def timer_callback(self):
        key = getKey(self.settings)
        
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.z = moveBindings[key][2]
            self.th = moveBindings[key][3]
        elif key == ' ' or key == 'k':
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.th = 0.0
        else:
            if (key == '\x03'): # CTRL-C
                rclpy.shutdown()
                sys.exit(0)
            
            # Decay the velocity if no key pressed to make it smoother
            self.x *= 0.8
            self.y *= 0.8
            self.th *= 0.8

            if abs(self.x) < 0.01: self.x = 0.0
            if abs(self.y) < 0.01: self.y = 0.0
            if abs(self.th) < 0.01: self.th = 0.0

        twist = Twist()
        twist.linear.x = float(self.x * self.speed)
        twist.linear.y = float(self.y * self.speed)
        twist.linear.z = float(self.z * self.speed)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.th * self.turn)
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    try:
        rclpy.spin(teleop_node)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, teleop_node.settings)
        teleop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
