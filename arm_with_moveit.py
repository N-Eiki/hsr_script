#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
import sys
from copy import deepcopy
import math

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

from move_whole_body import treat_cube

class Arm(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        self.reference_frame = "odom"
        self.group = moveit_commander.MoveGroupCommander("arm",
                                                  wait_for_servers=0.0)
        
    # def go_target_position(self, best_pix_x, best_pix_y, height, quaternion):
    #     primitive_position = [best_pix_x * self.config.heightmap_resolution + self.config.workspace_limits[0][0] , #-self.config.x_offset/2,
    #                              best_pix_y * self.config.heightmap_resolution + self.config.workspace_limits[1][0] - self.config.y_offset,
    #                               height + self.config.workspace_limits[2][0]]
    #     ps = geometry_msgs.msg.Pose()
    #     ps.position = Vector3(*primitive_position)
    #     ps.orientation = Quaternion(*quaternion)
    #     result = False
    #     # while not result:
    #     result = self.move_manager(pose_requested=ps, joints_array_requested=None, movement_type_requested="TCP")
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
        
    def move_manager(self, pose_requested, joints_array_requested, movement_type_requested):
        success = False
        rospy.sleep(1)
        if movement_type_requested == "TCP":
            print(pose_requested)
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

    def ee_traj(self, pose):
        pose_frame = self.group.get_pose_reference_frame()
        # if pose_frame != "base_link":
        #     new_reference_frame = "base_link"
        #     self.group.set_pose_reference_frame(new_reference_frame)
        #     pose_frame = self.group.get_pose_reference_frame()
        # else:
        #     pass
        self.group.set_pose_target(pose)
        result = self.execute_trajectory(self.group)
        return result


    def execute_trajectory(self, group):
        # plan = group.plan()
        # if plan.joint_trajectory.header.frame_id=='':
        #     return False
        result = self.group.go()
        # result = group.execute(plan, wait=True)
    
    def move_thread(self,):
        rate = rospy.Rate(10)  # Publish rate: 10 Hz
        linear_y = 1.0
        angular_z = 0.
        while self.move_flg:
            tw = geometry_msgs.msg.Twist()
            tw.linear.y = linear_y
            tw.linear.x = robot.velocity_p_controller(0, "x")
            tw.angular.z = robot.anguler_p_controller(angular_z)
            robot.pub.publish(tw)
            rate.sleep()
        
        move_to(0, 0, 0)

def get_position(position):
    ps = geometry_msgs.msg.Pose()
    quaternion = [0.993036598592378, -0.02454754192500744, -0.03405677016698293, 0.0007427215106143473]
    ps.position = Vector3(*position)
    ps.orientation = Quaternion(*quaternion)
    return ps




if __name__ == "__main__":
    treat_cube("cube0", 0.4, 0.8, 0.4)
    treat_cube("cube1", 0.4, 0.8, 0.3)
    
    before_joint = [0.20007442706359227, -1.1561440927869766, -0.0006512175585218216, -1.75018630860815, 0.0023625875692188103+math.pi/2, 0.0]
    after_joint = [0.1980021531111825, -2.2807609067479184, -0.0007123678763729657, -0.792575025845303, 0.0028319549774264985+math.pi/2, 0.0]

    before_position = [0.45467554101050756, 0.002603384451447619, 0.35190323568909975]
    grasp_position = [0.39345565711676, 0.10954432070249302, 0.17]
    arm = Arm()
    gripper = gripper_controller()

    print('before')
    ps = get_position(before_position)
    result = arm.move_manager(pose_requested=None ,joints_array_requested=before_joint, movement_type_requested="JOINTS")
    print('done')

    self.move_flg = True
    thread = threading.Thread(target=move_thread)
    thread.start()
    print('grasp')
    ps = get_position(grasp_position)
    result = arm.move_manager(pose_requested=None, joints_array_requested=after_joint, movement_type_requested="JOINTS")
    print(arm.group.get_current_pose())
    gripper.close()
    print('done')
    self.move_flg = False
    