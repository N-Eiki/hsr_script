#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

from copy import deepcopy
import math
import sys
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3, Quaternion
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import shape_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_multiply
import trajectory_msgs.msg

from move_action import move_to as action_move
from move_topic import move_to
import time
import threading

import controller_manager_msgs.srv

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import numpy as np

class Arm(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        self.reference_frame = "odom"
        self.group = moveit_commander.MoveGroupCommander("arm",
                                                  wait_for_servers=0.0)
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)
    
    def move_manager(self, pose_requested, joints_array_requested, movement_type_requested):
        success = False
        rospy.sleep(1)
        if movement_type_requested == "TCP":
            # print(pose_requested)
            success = self.ee_traj(pose_requested)
        elif movement_type_requested == "JOINTS":
            success = self.joint_traj(joints_array_requested)
        elif movement_type_requested == "TORSO":
            torso_height = joints_array_requested[0]
            success = self.move_torso(torso_height)
        elif movement_type_requested == "HEAD":
            XYZ = [joints_array_requested[0],
                   joints_array_requested[1],
                   joints_array_requested[2]]
            success = self.move_head_point(XYZ)
        elif movement_type_requested == "GRIPPER":
            gripper_x = joints_array_requested[0]
            max_effort = joints_array_requested[1]
            success = self.move_gripper(gripper_x, max_effort)
        else:
            rospy.logerr("Asked for non supported movement type==>" +
                         str(movement_type_requested))
        return success
    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        rospy.logdebug("Group Vars:")
        rospy.logdebug(self.group_variable_values)
        rospy.logdebug("Point:")
        rospy.logdebug(positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        # self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory(self.group)
        return result

    def ee_traj(self, pose):
        # pose_frame = self.group.get_pose_reference_frame()
        self.group.set_pose_target(pose)
        result = self.execute_trajectory(self.group)
        return result

    def execute_trajectory(self, group):
        result = self.group.go()


def get_position(position):
    ps = geometry_msgs.msg.Pose()
    
    quaternion = [0.993036598592378, -0.02454754192500744, -0.03405677016698293, 0.0007427215106143473]
    ps.position = Vector3(*position)
    ps.orientation = Quaternion(*quaternion)
    return ps

def move_thread():
    curr = 0
    steps = 10
    target_pos = 5.
    action_move(0, 0, 0)
    # time.sleep(3)
    move_to(0, target_pos, 0)
    for step in range(steps):
        target = step/steps
        # print(target)
        move_to(0, target, 0)
        time.sleep(0.5)
    action_move(0, target, 0)
    move_to(0, 0, 0)

class CubePoseControll():
    def __init__(self):
        pass

    def pose_controll(self, model_name, x, y, z):
        rand = np.random.rand(2)
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except (rospy.ServiceException, e):
            print("Service call failed: %s" % e)



def treat_cube(model_name, x, y, z):
    rand = np.random.rand(2)
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except (rospy.ServiceException, e):
        print("Service call failed: %s" % e)

class gripper_controller:
    def __init__(self):
        self.gripper = moveit_commander.MoveGroupCommander("gripper",
                                                      wait_for_servers=0.0)
        self.gripper.set_goal_joint_tolerance(0.05)
        self.gripper.set_max_acceleration_scaling_factor(1)
        self.gripper.set_max_velocity_scaling_factor(1)
        self.open()

    def close(self):
        self.gripper.set_joint_value_target("hand_motor_joint", 0.0)
        self.gripper.go()

    def open(self):
        self.gripper.set_joint_value_target("hand_motor_joint", 1.2)
        self.gripper.go()
    
if __name__ == "__main__":
    init_joint = [0.009200126094910216, -6.0276045254603616e-05, -1.568029726037926, -1.5700488771307501, 4.292498591951244e-05, 0.0]
    before_joint = [0.20007442706359227, -1.1561440927869766, -0.0006512175585218216, -1.75018630860815, 0.0023625875692188103, 0.0]
    after_joint = [0.1980021531111825, -2.2807609067479184, -0.0007123678763729657, -0.792575025845303, 0.0028319549774264985, 0.0]

    before_position = [0.45467554101050756, 0.002603384451447619, 0.35190323568909975]
    grasp_position = [0.39345565711676, 0.10954432070249302, 0.17]
    arm = Arm()

    # import ipdb;ipdb.set_trace()
    gripper = gripper_controller()

    print('before')
    ps = get_position(before_position)
    result = arm.move_manager(pose_requested=None ,joints_array_requested=before_joint, movement_type_requested="JOINTS")
    print('done')

    treat_cube("cube0", 0.4, 0.1, 0.4)
    treat_cube("cube1", 0.4, 0.1, 0.3)
    # thread = threading.Thread(target=move_thread)
    # thread.start()
    print('grasp')
    ps = get_position(grasp_position)
    result = arm.move_manager(pose_requested=None, joints_array_requested=after_joint, movement_type_requested="JOINTS")
    print(arm.group.get_current_pose())
    gripper.close()

    result = arm.move_manager(pose_requested=None, joints_array_requested=init_joint, movement_type_requested="JOINTS")
    
    print('done')
    