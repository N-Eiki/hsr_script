#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import numpy as np
import math

import controller_manager_msgs.srv
import rospy
import rospkg 
import trajectory_msgs.msg

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from move_whole_body import treat_cube


if __name__=="__main__":
    treat_cube("cube0", 0.4, 0.8, 0.4)
    treat_cube("cube1", 0.4, 0.8, 0.3)

    rospy.init_node('arm_test')
    # initialize ROS publisher
    pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                        trajectory_msgs.msg.JointTrajectory, queue_size=10)

    # wait to establish connection between the controller
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers = (
        rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                        controller_manager_msgs.srv.ListControllers))
    running = False
    while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'arm_trajectory_controller' and c.state == 'running':
                running = True

    # fill ROS message
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                        "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    before_joint = [0.20007442706359227, -1.1561440927869766, -0.0006512175585218216, -1.75018630860815, 0.0023625875692188103+math.pi/2, 0.0]
    after_joint = [0.1980021531111825, -2.2807609067479184, -0.0007123678763729657, -0.792575025845303, 0.0028319549774264985+math.pi/2, 0.0]

    p.positions = before_joint
    p.velocities = [0, 0, 0, 0, 0]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]

    # publish ROS message
    pub.publish(traj)
    