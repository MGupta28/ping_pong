#!/usr/bin/env python

import rospy
import signal
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SpawnRequest
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys
import threading

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

class TeleopPingPong:
    def __init__(self):
        self.linear_left = 0
        self.linear_right = 0
        self.l_scale = 2.0
        self.twist_pub_left = rospy.Publisher("/left_paddle/cmd_vel", Twist, queue_size=1)
        self.twist_pub_right = rospy.Publisher("/right_paddle/cmd_vel", Twist, queue_size=1)
        self.ball_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.ball_x = 1.0
        self.ball_y = 1.0
        self.ball_speed_x = 1.0
        self.ball_speed_y = 0.6

        self.left_paddle_x = 1.0
        self.left_paddle_y = 5.0
        self.right_paddle_x = 10.0
        self.right_paddle_y = 5.0

        self.ball_pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.ball_pose_callback)
        self.left_paddle_pose_sub = rospy.Subscriber("/left_paddle/pose", Pose, self.left_paddle_pose_callback)
        self.right_paddle_pose_sub = rospy.Subscriber("/right_paddle/pose", Pose, self.right_paddle_pose_callback)

    def ball_pose_callback(self, data):
        # Update the ball's position based on the subscriber
        self.ball_x = data.x
        self.ball_y = data.y

    def left_paddle_pose_callback(self, data):
        self.left_paddle_x = data.x
        self.left_paddle_y = data.y

    def right_paddle_pose_callback(self, data):
        self.right_paddle_x = data.x
        self.right_paddle_y = data.y

    def spawn_paddle_turtles(self):
        # Spawn the left paddle turtle
        spawn_left_paddle = rospy.ServiceProxy('/spawn', Spawn)
        left_paddle_request = SpawnRequest()
        left_paddle_request.x = 1.0  # Set the initial x-coordinate of the left paddle
        left_paddle_request.y = 5.0  # Set the initial y-coordinate of the left paddle
        left_paddle_request.theta = 0.0
        left_paddle_request.name = 'left_paddle'
        left_paddle_response = spawn_left_paddle(left_paddle_request)

        # Spawn the right paddle turtle
        spawn_right_paddle = rospy.ServiceProxy('/spawn', Spawn)
        right_paddle_request = SpawnRequest()
        right_paddle_request.x = 10.0  # Set the initial x-coordinate of the right paddle
        right_paddle_request.y = 5.0  # Set the initial y-coordinate of the right paddle
        right_paddle_request.theta = 3.14
        right_paddle_request.name = 'right_paddle'
        right_paddle_response = spawn_right_paddle(right_paddle_request)

    def quit(self, sig, frame):
        input.shutdown()
        rospy.signal_shutdown("Keyboard interrupt")

    def get_key_press(self):
        c = input.read_one()
        self.linear_left = self.linear_right = 0
        return c

    def key_loop(self):
        print("Reading from keyboard")
        print("---------------------------")
        print("Use 'w' to move the left paddle up, 's' to move it down. Use 'i' to move the right paddle up, 'k' to move it down. 'q' to quit.")

        while not rospy.is_shutdown():
            c = self.get_key_press()

            if c == KEYCODE_W:
                print("Move left paddle up (W)")
                self.linear_left = 1.0
                self.twist_pub_left.publish(self.create_twist(self.linear_left))
            elif c == KEYCODE_S:
                print("Move left paddle down (S)")
                self.linear_left = -1.0
                self.twist_pub_left.publish(self.create_twist(self.linear_left))
            elif c == KEYCODE_UP:
                print("Move right paddle up (I)")
                self.linear_right = 1.0
                self.twist_pub_right.publish(self.create_twist(self.linear_right))
            elif c == KEYCODE_DOWN:
                print("Move right paddle down (K)")
                self.linear_right = -1.0
                self.twist_pub_right.publish(self.create_twist(self.linear_right))
            elif c == KEYCODE_Q:
                print("Quit")
                return

    def create_twist(self, linear):
        twist = Twist()
        twist.linear.y = self.l_scale * linear
        return twist

    def move_ball(self):
        # Update the ball's position
        self.ball_x += self.ball_speed_x
        self.ball_y += self.ball_speed_y

        # Bounce off the top and bottom walls
        if self.ball_y < 0.1 or self.ball_y > 10.8:
            self.ball_speed_y *= -1

        # Check for collisions with the left paddle
        if (
            self.ball_x < self.left_paddle_x 
            and self.ball_y > self.left_paddle_y - 0.5
            and self.ball_y < self.left_paddle_y + 0.5
        ):
            self.ball_speed_x *= -1

        # Check for collisions with the right paddle
        if (
            self.ball_x > self.right_paddle_x
            and self.ball_y > self.right_paddle_y - 0.5
            and self.ball_y < self.right_paddle_y + 0.5
        ):
            self.ball_speed_x *= -1

        # Publish the updated ball position
        ball_twist = Twist()
        ball_twist.linear.x = self.ball_speed_x
        ball_twist.linear.y = self.ball_speed_y
        self.ball_pub.publish(ball_twist)

    def move_ball_continuous(self):
        rate = rospy.Rate(10)  # Adjust the rate as needed (e.g., 10 Hz)

        while not rospy.is_shutdown():
            self.move_ball()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("teleop_ping_pong")
    teleop_ping_pong = TeleopPingPong()
    signal.signal(signal.SIGINT, teleop_ping_pong.quit)                 
    teleop_ping_pong.spawn_paddle_turtles()  # Spawn the paddle turtles

    # Create threads for continuous ball movement and key loop for paddle control
    ball_thread = threading.Thread(target=teleop_ping_pong.move_ball_continuous)
    key_thread = threading.Thread(target=teleop_ping_pong.key_loop)

    # Start the threads
    ball_thread.start()
    key_thread.start()

    # Wait for threads to finish
    ball_thread.join()
    key_thread.join()
