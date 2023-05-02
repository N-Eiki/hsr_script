#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import rospy
import tf.transformations

import time


def move_to(goal_x, goal_y, goal_yaw):
    # initialize ROS publisher
    pub = rospy.Publisher('goal', PoseStamped, queue_size=10)

    # wait to establish connection between the navigation interface
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)
    # fill ROS message
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position = Point(goal_x, goal_y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    goal.pose.orientation = Quaternion(*quat)

    # publish ROS message
    pub.publish(goal)


if __name__=="__main__":
    rospy.init_node('test')
    print('go')
    move_to(0, 1, 0)
    print("sleep")
    time.sleep(10)
    print('back')
    move_to(0, 0, 0)

   