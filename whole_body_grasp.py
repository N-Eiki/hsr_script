#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
import sys
import threading
import time
from copy import deepcopy
import matplotlib.pyplot as plt

import math
import numpy as np

import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3, Quaternion
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import trajectory_msgs.msg

import moveit_commander
import moveit_msgs.msg

import rospy
import shape_msgs.msg

from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf.transformations as tft


from move_whole_body import treat_cube
# from move_topic import move_to
from move_action import move_to

class Body:
    def __init__(self, wait=0.0, y_velocity=1):
        self.arm = Arm(wait)
        self.gripper = Gripper(wait)
        self.base = Base(wait, y_velocity)

        self.move_flg = False
        self.y_velocity = y_velocity
        # self.go_home_base_position()

    
    def start_base(self):
        self.move_flg = True
        thread = threading.Thread(target=self.move_thread)
        thread.start()

    def stop_base(self):
        self.move_flg = False

    def move_thread(self,):
        rate = rospy.Rate(10)  # Publish rate: 10 Hz
        angular_z = 0.
        while self.move_flg:
            tw = geometry_msgs.msg.Twist()
            tw.linear.y = self.y_velocity
            tw.linear.x = self.base.velocity_p_controller(0, "x")
            tw.angular.z = self.base.anguler_p_controller(angular_z)
            self.base.pub.publish(tw)
            rate.sleep()

    def go_home_base_position(self):
        rate = rospy.Rate(10)
        while 1:
            tw = geometry_msgs.msg.Twist()
            tw.linear.y = self.base.velocity_p_controller(0, "y")
            tw.linear.x = self.base.velocity_p_controller(0, "x")
            tw.angular.z = self.base.anguler_p_controller(0)
            self.base.pub.publish(tw)
            rate.sleep()
            error = np.sqrt(self.base.current_position.x**2 + self.base.current_position.y**2)
            if error<0.5:
                break

class Base(object):
    def __init__(self, wait=0.0, velocity=1.):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)
        self.Kp = 25*velocity
        self.Kp_velocity = 25*velocity
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

class Arm(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        self.reference_frame = "odom"
        self.group = moveit_commander.MoveGroupCommander("arm",
                                                  wait_for_servers=0.0)
        self.group.set_max_acceleration_scaling_factor(1)
        self.group.set_max_velocity_scaling_factor(1)

    def get_position(self, position):
        ps = geometry_msgs.msg.Pose()
        quaternion = [0.993036598592378, -0.02454754192500744, -0.03405677016698293, 0.0007427215106143473]
        ps.position = Vector3(*position)
        ps.orientation = Quaternion(*quaternion)
        return ps

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

class Gripper:
    def __init__(self, wait):
        self.gripper = moveit_commander.MoveGroupCommander("gripper",
                                                      wait_for_servers=wait)
        self.gripper.set_goal_joint_tolerance(0.01)
        self.gripper.set_max_acceleration_scaling_factor(1)
        self.gripper.set_max_velocity_scaling_factor(1)

        # rospy.Subscriber("/hsrb/odom_ground_truth", Odometry, self.autograsp_odometry_callback)
        self.open()

    def close(self):
        self.gripper.set_joint_value_target("hand_motor_joint", 0.0)
        self.gripper.go()

    def open(self):
        self.gripper.set_joint_value_target("hand_motor_joint", 1.0)
        self.gripper.go()

    
    # def autograsp_odometry_callback(self, msg):
    #     q = msg.pose.pose.orientation
    #     euler_angles = tft.euler_from_quaternion((q.x, q.y, q.z, q.w))
    #     # print("Angular velocity: ", self.angular_z)
    #     self.current_position = msg.pose.pose.position
    #     if self.current_position.y>=0.5:
    #         self.close()
def check_moving_time():
    treat_cube("cube0", 0.4, 2, 0.4)
    treat_cube("cube1", 0.4, 2, 0.3)
    
    robot = Body(y_velocity=0.2)
    init_joint = [0.009200126094910216, -6.0276045254603616e-05, -1.568029726037926, -1.5700488771307501, 4.292498591951244e-05, 0.0]
    before_joint = [0.20007442706359227, -1.1561440927869766, -0.0006512175585218216, -1.75018630860815, 0.0023625875692188103+math.pi/2, 0.0]
    after_joint = [0.1980021531111825, -2.2807609067479184, -0.0007123678763729657, -0.792575025845303, 0.0028319549774264985+math.pi/2, 0.0]

    before_position = [0.45467554101050756, 0.002603384451447619, 0.35190323568909975]
    grasp_position = [0.39345565711676, 0.10954432070249302, 0.17]
   
    times = {
        "init2before":[],
        "before2grasp":[],
        "close_gripper":[],
    }

    for _ in range(10):
        print('before')
        tmp = time.time()
        ps = robot.arm.get_position(before_position)
        result = robot.arm.move_manager(pose_requested=None ,joints_array_requested=before_joint, movement_type_requested="JOINTS")
        times["init2before"].append(time.time()-tmp)
        print('done')
        # robot.start_base()
    
        print('grasp')
        tmp = time.time()
        ps = robot.arm.get_position(grasp_position)
        times["before2grasp"].append(time.time()-tmp)
        result = robot.arm.move_manager(pose_requested=None, joints_array_requested=after_joint, movement_type_requested="JOINTS")
        tmp = time.time()
        robot.gripper.close()
        times["close_gripper"].append(time.time()-tmp)
        print('done')
        
        robot.stop_base()
        robot.base.goto_point(0, 0)
        rospy.loginfo('go initial')

        result = robot.arm.move_manager(pose_requested=None, joints_array_requested=init_joint, movement_type_requested="JOINTS")
        print('done')
    print('finish')
    for k in times.keys():
        plt.plot(times[k], label=k)
    plt.legend()
    plt.show()

    np.save("moveing_time.npy", times)

if __name__=="__main__":
    # check_moving_time()

    treat_cube("cube0", 0.45, .85, 0.4)
    # treat_cube("cube1", 0.45, .7, 0.3)
    
    robot = Body(y_velocity=0.1)
    # import ipdb;ipdb.set_trace()

    init_joint = [0.009200126094910216, -6.0276045254603616e-05, -1.568029726037926, -1.5700488771307501, 4.292498591951244e-05, 0.0]
    
    # before_joint = [0.20007442706359227, -1.1561440927869766, -0.0006512175585218216, -1.75018630860815, 0.0023625875692188103, 0.0]
    # after_joint = [0.1980021531111825, -2.2807609067479184, -0.0007123678763729657, -0.792575025845303, 0.0028319549774264985, 0.0]

    # before_joint = [0.20007442706359227, -1.1561440927869766, -0.0006512175585218216, -1.75018630860815, 0.0023625875692188103+math.pi/2, 0.0]
    # after_joint = [0.1980021531111825, -2.2807609067479184, -0.0007123678763729657, -0.792575025845303, 0.0028319549774264985+math.pi/2, 0.0]

    before_joint = after_joint = [-2.3110134843744925e-06, -2.1266098113995957, 1.0422966361807458, -1.218875769749812, 1.203239417920721, 0.0]
    before_joint = [0.2015549209192433, -2.129589331428237, 1.0418513366843518, -1.2178296877378418, 1.2032654240475367, 0.0]

    print('before')
    # ps = robot.arm.get_position(before_position)
    result = robot.arm.move_manager(pose_requested=None ,joints_array_requested=before_joint, movement_type_requested="JOINTS")
    print('done')
    robot.start_base()

    print('grasp')
    # ps = robot.arm.get_position(grasp_position)
    result = robot.arm.move_manager(pose_requested=None, joints_array_requested=after_joint, movement_type_requested="JOINTS")
    
    robot.gripper.close()
    print(robot.arm.group.get_current_pose())
    print('done')
    time.sleep(3)
    robot.stop_base()
    print('go home')


    
    # robot.go_home_base_position()
    rospy.loginfo('go initial')
    result = robot.arm.move_manager(pose_requested=None, joints_array_requested=init_joint, movement_type_requested="JOINTS")
    print('done')
    time.sleep(10)
    robot.gripper.open()

