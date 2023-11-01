#!/usr/bin/env python

import rospy
import signal
from geometry_msgs.msg import Twist
import sys

KEYCODE_RIGHT = 0x43
KEYCODE_LEFT = 0x44
KEYCODE_UP = 0x41
KEYCODE_DOWN = 0x42
KEYCODE_W = 0x77
KEYCODE_S = 0x73
KEYCODE_Q = 0x71

class KeyboardReader:
    def __init__(self):
        self.cooked = None

    def read_one(self):
        try:
            c = sys.stdin.read(1)
            return ord(c)
        except IOError:
            pass

    def shutdown(self):
        if self.cooked:
            sys.stdin = self.cooked

input = KeyboardReader()

class TeleopTurtle:
    def __init__(self):
        self.linear = 0
        self.angular = 0
        self.l_scale = 2.0
        self.a_scale = 2.0
        self.twist_pub = rospy.Publisher("/turtle/cmd_vel", Twist, queue_size=1)

    def quit(self, sig, frame):
        input.shutdown()
        rospy.signal_shutdown("Keyboard interrupt")

    def get_key_press(self):
        c = input.read_one()
        self.linear = self.angular = 0
        return c

    def key_loop(self):
        print("Reading from keyboard")
        print("---------------------------")
        print("Use arrow keys to move the turtle. 'q' to quit.")

        while not rospy.is_shutdown():
            c = self.get_key_press()

            if c == KEYCODE_UP:
                print("UP")
                self.linear = 1.0
            elif c == KEYCODE_DOWN:
                print("DOWN")
                self.linear = -1.0
            elif c == KEYCODE_W:
                print("W")
                self.linear = 1.0
            elif c == KEYCODE_S:
                print("S")
                self.linear = -1.0
            elif c == KEYCODE_Q:
                print("quit")
                return

            twist = Twist()
            twist.angular.z = self.a_scale * self.angular
            twist.linear.x = self.l_scale * self.linear
            self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node("teleop_turtle")
    teleop_turtle = TeleopTurtle()
    signal.signal(signal.SIGINT, teleop_turtle.quit)
    teleop_turtle.key_loop()
