#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

from std_msgs.msg import Float32MultiArray

import numpy as np

class Head:
    def __init__(self):
        # pan   : horizontal ← or →
        # tilt  : vertical ↑ or ↓
        self.max_tilt = 0.5
        self.min_tilt = -0.5
        self.max_pan = 0.5
        self.min_pan = -0.5

        sub = rospy.Subscriber("/head_direction", Float32MultiArray, self.move_head_callback)
        self.head_angle_pub = rospy.Publisher(
            '/hsrb/head_trajectory_controller/command',
            trajectory_msgs.msg.JointTrajectory, queue_size=10)

        # wait to establish connection between the controller
        while self.head_angle_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)

        for c in list_controllers().controller:
            if c.name == 'head_trajectory_controller' and c.state == 'running':
                running = True

        rospy.spin()
                
    def move_head_callback(self, data):
        # data must be radian
        pan = np.clip(data.data[0], self.min_pan, self.max_pan)
        tilt = np.clip(data.data[1], self.min_tilt, self.max_tilt)
        print(pan, tilt)
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [pan, tilt]
        p.velocities = [0, 0]
        p.time_from_start = rospy.Duration(3)
        traj.points = [p]
        # publish ROS message
        self.head_angle_pub.publish(traj)

if __name__=="__main__":
    rospy.init_node('head_test')
    print('start')
    h = Head()
