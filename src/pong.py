#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Color
import math

def color_sensor_callback(color):
    # ROS_INFO_THROTTLE(0.1, "Color received (r,g,b) = (%i,%i,%i)", color.r, color.g, color.b)
    pass

def spawn_player_turtle(name, x, y, theta):
    rospy.loginfo(f"Spawn {name}")
    spawn = rospy.ServiceProxy('/spawn', Spawn)
    set_pen = rospy.ServiceProxy(f'/{name}/set_pen', SetPen)

    try:
        spawn(name, x, y, theta)
        set_pen(True)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('pong')
    nh = rospy.NodeHandle()

    rospy.loginfo("Reset and remove default turtle1 from the game")
    rospy.wait_for_service('/reset')
    empty = rospy.ServiceProxy('/reset', Empty)
    empty()

    kill = rospy.ServiceProxy('/kill', Kill)
    kill('turtle1')

    spawn_player_turtle("turtle_left", 1.0, 5.0, math.pi/2)
    spawn_player_turtle("turtle_right", 10.0, 5.0, math.pi/2)

    ball_color_sub = rospy.Subscriber('/ball/color_sensor', Color, color_sensor_callback)

    # pub = nh.advertise('/turtle1/cmd_vel', Twist, queue_size=1)
    # twist = Twist()

    loop_rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        loop_rate.sleep()
