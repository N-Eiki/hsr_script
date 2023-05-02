#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

from copy import deepcopy
import math
import sys
import actionlib
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import shape_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_multiply
import trajectory_msgs.msg
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from move_whole_body import treat_cube
import tf.transformations as tft


class MoveBase(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)
        self.Kp = 50
        self.Kp_velocity = 25
        self.angular_z = 0.
        self.current_position = None
        self.reference_frame = "odom"
        self.base = moveit_commander.MoveGroupCommander("base",wait_for_servers=0.0)
        self.base.set_goal_joint_tolerance(0.05)
        self.base.set_max_acceleration_scaling_factor(.1)# max_vel=10
        self.base.set_max_velocity_scaling_factor(.01)
        self.pub = rospy.Publisher('/hsrb/command_velocity',geometry_msgs.msg.Twist, queue_size=1)
        # rospy.Subscriber("/hsrb/wheel_odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/hsrb/odom_ground_truth", Odometry, self.odometry_callback)
        

    def odometry_callback(self, msg):
        q = msg.pose.pose.orientation
        euler_angles = tft.euler_from_quaternion((q.x, q.y, q.z, q.w))
        # print("Angular velocity: ", self.angular_z)
        self.angular_z = euler_angles[-1]
        self.current_position = msg.pose.pose.position


    def goto_point(self, x, y):
        curr = self.base.get_current_joint_values()
        curr[0] = x
        curr[1] = y
        self.base.set_joint_value_target(curr)
        self.base.go()

    def anguler_p_controller(self, target_angle):
        def wrap_to_pi(r):
            wrapped = math.fmod(r, 2.0 * math.pi)
            if wrapped > math.pi:
                wrapped -= 2.0 * math.pi
            elif wrapped <= -math.pi:
                wrapped += 2.0 * math.pi
            return wrapped
        angular_error = wrap_to_pi(target_angle - self.angular_z)
        angular = self.Kp * angular_error
        return angular

    def velocity_p_controller(self, target, target_axis="x"):
        try:
            pos_error = target - getattr(self.current_position, target_axis)
            target_velocity = self.Kp_velocity * pos_error
            return target_velocity
        except:
            pass
        return 0.


    
    
if __name__ == "__main__":
    robot = MoveBase()
    treat_cube("cube0", 0.4, 0.6, 0.4)
    treat_cube("cube1", 0.4, 0.6, 0.3)
    rate = rospy.Rate(10)  # Publish rate: 10 Hz

    linear_y = 1.0

    angular_z = 0.
    while not rospy.is_shutdown():
        tw = geometry_msgs.msg.Twist()
        tw.linear.y = linear_y
        tw.linear.x = robot.velocity_p_controller(0, "x")
        tw.angular.z = robot.anguler_p_controller(angular_z)
        robot.pub.publish(tw)
        rate.sleep()

