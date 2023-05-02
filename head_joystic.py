#!/usr/bin/env python3
import actionlib
from actionlib_msgs.msg import GoalStatus
import controller_manager_msgs.srv
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from std_msgs.msg import Float32MultiArray

import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations

import math
import numpy as np
import pygame
from pygame.locals import *
import os

from simple_suction import Suction
os.environ["SDL_VIDEODRIVER"] = "dummy"


# pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
cut_threshold = 0.05

rospy.init_node('test')

# initialize ROS publisher
pub_head_radian = rospy.Publisher('/head_direction', Float32MultiArray, queue_size=1)


# # wait to establish connection between the controller
# while pub_head_radian.get_num_connections() == 0:
#     rospy.sleep(0.1)

rate = 60
rospy_rate = rospy.Rate(rate)


# axis
x_axis = 1
y_axis = 0
yaw_axis = 2
z_axis = 3

# rad limits
max_tilt = 0.5
min_tilt = -0.5
max_pan = 0.5
min_pan = -0.5


# main loop
direction_tilt = 0
direction_pan = 0
print('start main loop')
tilt_degree = 0
pan_degree = 0

while not rospy.is_shutdown():
    pygame.event.pump()
    now = rospy.Time.now()

    tilt = -1 * joystick.get_axis(x_axis) if abs(joystick.get_axis(x_axis)) > cut_threshold else 0.0
    pan = joystick.get_axis(y_axis) if abs(joystick.get_axis(y_axis)) > cut_threshold else 0.0
    # pan = joystick.get_axis(yaw_axis) if abs(joystick.get_axis(yaw_axis)) > cut_threshold else 0.0
    # z_input = joystick.get_axis(z_axis) if abs(joystick.get_axis(z_axis)) > cut_threshold else 0.0

    tilt_degree += tilt*0.5
    pan_degree += pan*0.5

    tilt_rad = np.clip(np.deg2rad(tilt_degree), min_tilt, max_tilt)
    pan_rad = np.clip(np.deg2rad(pan_degree), min_pan, max_pan)
    rads = [pan_rad, tilt_rad]
    arr = Float32MultiArray()
    arr.data = rads
    
    pub_head_radian.publish(arr)

    rospy_rate.sleep()
