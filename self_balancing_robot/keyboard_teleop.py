#!/usr/bin/env python3
"""
Keyboard Teleop for the Self-Balancing Robot
----------------------------------------------
Publishes a "driving intent" to /teleop_cmd. It does NOT publish
directly to /cmd_vel -- the balance_controller node is the only thing
allowed to publish there, so it can keep the robot upright while also
following your driving requests.

CONTROLS:
  w : increase forward speed
  s : increase backward speed (or slow down if currently moving forward)
  a : turn left
  d : turn right
  x or SPACE : stop (reset speed and turn to zero)
  q : quit this program

Each key press nudges the target by one small step -- press repeatedly
to build up speed, same feel as the standard ROS2 teleop tool.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


INSTRUCTIONS = """
Self-Balancing Robot -- Keyboard Teleop
----------------------------------------
  w : speed up forward
  s : speed up backward / slow down
  a : turn left
  d : turn right
  x / space : stop (zero speed and turn)
  q : quit
----------------------------------------
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        
        self.linear_step = 0.03    # m/s added per 'w'/'s' press
        self.angular_step = 0.3    # rad/s added per 'a'/'d' press
        self.max_linear = 0.25     # m/s cap -- stay well under balance limits
        self.max_angular = 1.0     # rad/s cap

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.publisher = self.create_publisher(Twist, '/teleop_cmd', 10)
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20 Hz

        self.settings = termios.tcgetattr(sys.stdin)
        print(INSTRUCTIONS)

    def get_key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.publisher.publish(msg)

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'w':
                    self.target_linear = min(
                        self.target_linear + self.linear_step, self.max_linear)
                elif key == 's':
                    self.target_linear = max(
                        self.target_linear - self.linear_step, -self.max_linear)
                elif key == 'a':
                    self.target_angular = min(
                        self.target_angular + self.angular_step, self.max_angular)
                elif key == 'd':
                    self.target_angular = max(
                        self.target_angular - self.angular_step, -self.max_angular)
                elif key in ('x', ' '):
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                elif key == 'q':
                    break

                print(
                    f"\rtarget speed: {self.target_linear:+.2f} m/s   "
                    f"target turn: {self.target_angular:+.2f} rad/s   ",
                    end=''
                )

                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.publish_cmd()  # send one final "stop" before exiting
            print("\nStopped. Exiting.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
