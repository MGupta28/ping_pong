#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute, SetPen
from random import random
import math

class Ball:
    def __init__(self):
        rospy.init_node('ping_pong_ball')
        self.my_X = 0
        self.my_Y = 0
        self.direction = self.STOP
        self.pose = None
        self.pose_left = None
        self.pose_right = None
        self.cmd_vel_pub = rospy.Publisher('/ball/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/turtle1/pose", Pose, self.poseCallback)
        self.teleport_abs_client = rospy.ServiceProxy('/ball/teleport_absolute', TeleportAbsolute)
        self.penOff(True)
        self.spawnBall()

    def penOff(self, off):
        set_pen = SetPen()
        set_pen.off = off
        rospy.ServiceProxy('/ball/set_pen', SetPen)
    
    def spawnBall(self):
        spawn = Spawn()
        spawn.name = "ball"
        spawn.x = 2.0
        spawn.y = 2.0
        spawn.theta = 0.0
        rospy.ServiceProxy('/spawn',Spawn)
    
    def setPoseAbs(self, x, y, theta):
        pose_abs = TeleportAbsolute()
        pose_abs.x = x
        pose_abs.y = y
        pose_abs.theta = theta
        self.teleport_abs_client(pose_abs)

    def setVel(self, x, theta):
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = theta
        self.cmd_vel_pub.publish(twist)

    def randomAngle(self):
        angle = random()
        r = random()
        if r > 0.5:
            return angle * math.pi
        else:
            return angle * -math.pi

    def poseCallback(self, pose):
        self.pose = pose

    def poseLeftCallback(self, pose):
        self.pose_left = pose

    def poseRightCallback(self, pose):
        self.pose_right = pose

    def move(self):
        if self.pose is None or self.pose_left is None or self.pose_right is None:
            rospy.loginfo("Pose not yet received")
            self.setPoseAbs(3.0, 3.0, -0.5)
            return

        self.setVel(2.0, 0.0)

        # get current pose
        x = self.pose.x
        y = self.pose.y

        self.checkPlayerCollision()

        self.updateDirection()

        # Update pose if ball hits the wall
        new_theta = 0.0

        if self.pose.y > 11.0: # hit top wall
            if self.direction == self.UP_LEFT:
                new_theta = 2.0*math.pi - self.pose.theta
                self.setPoseAbs(x, y, new_theta)
            if self.direction == self.UP_RIGHT:
                new_theta = 2.0*math.pi - self.pose.theta
                self.setPoseAbs(x, y, new_theta)

        if self.pose.y < 0.001: # hit bottom wall
            if self.direction == self.DOWN_LEFT:
                new_theta = -self.pose.theta
                self.setPoseAbs(x, y, new_theta)
            if self.direction == self.DOWN_RIGHT:
                new_theta = -self.pose.theta
                self.setPoseAbs(x, y, new_theta)

        if self.pose.x > 11.0: # hit right wall
            if self.direction == self.UP_RIGHT:
                new_theta = math.pi - self.pose.theta
                self.setPoseAbs(x, y, new_theta)
            if self.direction == self.DOWN_RIGHT:
                new_theta = -math.pi - self.pose.theta
                self.setPoseAbs(x, y, new_theta)

        if self.pose.x < 0.001: # hit left wall
            if self.direction == self.UP_LEFT:
                new_theta = math.pi - self.pose.theta
                self.setPoseAbs(x, y, new_theta)
            if self.direction == self.DOWN_LEFT:
                new_theta = -(math.pi + self.pose.theta)
                self.setPoseAbs(x, y, new_theta)

    def checkPlayerCollision(self):
        ball_x = self.pose.x
        ball_y = self.pose.y

        left_x = self.pose_left.x
        left_y = self.pose_left.y

        right_x = self.pose_right.x
        right_y = self.pose_right.y

        delta_x = 0.2
        delta_y = 0.5
        paddle_size = 2.0 * delta_y
        max_bounce_angle = 5 * math.pi / 12

        if left_x - delta_x < ball_x and ball_x < left_x + delta_x:
            if left_y - delta_y < ball_y and ball_y < left_y + delta_y:
                rel_intersect_y = (left_y + delta_y) - ball_y
                norm_rel_intersect_y = rel_intersect_y / paddle_size
                bounce_angle = norm_rel_intersect_y * max_bounce_angle
                rospy.loginfo("turtle bounced")
                self.setPoseAbs(ball_x, ball_y, bounce_angle)

        if right_x - delta_x < ball_x and ball_x < right_x + delta_x:
            if right_y - delta_y < ball_y and ball_y < right_y + delta_y:
                rel_intersect_y = (right_y + delta_y) - ball_y
                norm_rel_intersect_y = rel_intersect_y / paddle_size
                bounce_angle = norm_rel_intersect_y * max_bounce_angle
                if bounce_angle > 0:
                    bounce_angle += math.pi / 2
                else:
                    bounce_angle -= math.pi / 2
                rospy.loginfo("turtle bounced")
                self.setPoseAbs(ball_x, ball_y, bounce_angle)

    def updateDirection(self):
        theta = self.pose.theta
        if math.isclose(theta, 0.0):
            self.direction = self.RIGHT
        elif 0.0 < theta < math.pi / 2:
            self.direction = self.UP_RIGHT
        elif math.pi / 2 < theta < math.pi:
            self.direction = self.UP_LEFT
        elif math.isclose(theta, math.pi):
            self.direction = self.LEFT
        elif -math.pi < theta < -math.pi / 2:
            self.direction = self.DOWN_LEFT
        elif -math.pi / 2 < theta < 0.0:
            self.direction = self.DOWN_RIGHT
        else:
            self.direction = self.STOP

    STOP = 0
    RIGHT = 1
    UP_RIGHT = 2
    UP_LEFT = 3
    LEFT = 4
    DOWN_LEFT = 5
    DOWN_RIGHT = 6

if __name__ == '__main__':
    ball = Ball()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        ball.move()
        rate.sleep()
