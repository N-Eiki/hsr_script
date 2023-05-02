#!/usr/bin/env python3
import actionlib
from actionlib_msgs.msg import GoalStatus
import controller_manager_msgs.srv
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
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

def clamp(value, minimum, maximum):
    return min(max(value, minimum), maximum)

def make_goal(goal_pose):
    pose = PoseStamped()
    pose.header.stamp = now
    pose.header.frame_id = "map"
    pose.pose.position = Point(*goal_pose[:3])
    pose.pose.orientation = Quaternion(*goal_pose[3:])
    goal = MoveBaseGoal()
    goal.target_pose = pose
    return goal

class Controller:
    def __init__(self, cycle):
        # robot param
        self.__max_velocity = 0.22
        self.__max_accel_to_velocity = 0.7
        self.__max_velangular = 2.0
        self.__max_accel_to_velangular = 3.0

        self.__velocity = np.array([0.0, 0.0])
        self.__velangular = 0.0
        self.__cycle = cycle

    def update(self, target_velocity, target_velangular):
        # velocity
        diff = target_velocity - self.__velocity
        if np.linalg.norm(diff) > 0.0:
            self.__velocity += min(np.linalg.norm(diff), self.__cycle * self.__max_accel_to_velocity) * diff / np.linalg.norm(diff)
        if self.__max_velocity < np.linalg.norm(self.__velocity):
            self.__velocity = np.linalg.norm(self.__max_velocity) * self.__velocity / np.linalg.norm(self.__velocity)
        # velangular
        cycle_acc = self.__cycle * self.__max_accel_to_velangular
        self.__velangular = clamp(target_velangular, self.__velangular - cycle_acc, self.__velangular + cycle_acc)
        self.__velangular = clamp(self.__velangular, -self.__max_velangular, self.__max_velangular)
        return self.__velocity, self.__velangular
    
    def max_velocity(self):
        return self.__max_velocity

    def max_velangular(self):
        return self.__max_velangular

# pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
cut_threshold = 0.05

rospy.init_node('test')

# initialize ROS publisher
pub_omni = rospy.Publisher(
    '/hsrb/command_velocity',
    Twist, queue_size=1)

pub_goal = rospy.Publisher(
    '/goal',
    PoseStamped, queue_size=10)

pub_correct_pose = rospy.Publisher(
    '/laser_2d_correct_pose',
    PoseWithCovarianceStamped, queue_size=10)

act_goal = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)

# wait to establish connection between the controller
while pub_omni.get_num_connections() == 0:
    rospy.sleep(0.1)
while pub_goal.get_num_connections() < 2:
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
        if c.name == 'omni_base_controller' and c.state == 'running':
            running = True

# Python API
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
whole_body.linear_weight = 100.0
whole_body.angular_wight = 100.0
gripper = robot.get('gripper')
tts = robot.get('default_tts')

# axis
x_axis = 1
y_axis = 0
yaw_axis = 2
z_axis = 3


rate = 60
rospy_rate = rospy.Rate(rate)

controller = Controller(1.0 / rate)

mode_list = [
    "omni_vel",
    "omni_goal",
    "arm",
    "auto",
]
mode_index = 0

gripper_command = 1.0

# 0: 原点, 1: 棚の前, 2: 机の前, 3: 机の左
pose_list = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    [0.50, 0.1, 0.0, 0.0, 0.0, -0.70, 0.70],
    [0.59, 0.70, 0.0, 0.0, 0.0, 0.0, 1.0],
    [1.25, 1.37, 0.0, 0.0, 0.0, 0.70, 0.71],
]
goal_index = 0
goal_is_canceled = True

auto_state = 0
prev_auto_state = 3

# main loop
while not rospy.is_shutdown():
    pygame.event.pump()
    now = rospy.Time.now()

    x_input = joystick.get_axis(x_axis) if abs(joystick.get_axis(x_axis)) > cut_threshold else 0.0
    y_input = joystick.get_axis(y_axis) if abs(joystick.get_axis(y_axis)) > cut_threshold else 0.0
    yaw_input = joystick.get_axis(yaw_axis) if abs(joystick.get_axis(yaw_axis)) > cut_threshold else 0.0
    z_input = joystick.get_axis(z_axis) if abs(joystick.get_axis(z_axis)) > cut_threshold else 0.0

    goal_button_pressed = False

    prev_mode_index = mode_index

    for e in pygame.event.get():
        if e.type == pygame.locals.JOYBUTTONDOWN:
            if e.button == 0:
                whole_body.move_to_neutral()
            elif e.button == 1:
                whole_body.move_to_go()
            elif e.button == 2:
                sentence = u'こんにちは'
                print(sentence)
                # tts.say(sentence)
            elif e.button == 3:
                max_command = 1.0
                min_command = -0.3
                gripper_command = max_command if gripper_command == min_command else min_command
                gripper.command(gripper_command, sync=False)
            elif e.button == 4:
                mode_index += 1
                if len(mode_list) <= mode_index:
                    mode_index = 0
            elif e.button == 9:
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = now
                pose.header.frame_id = 'map'
                pose.pose.pose.position = Point(0.0, 0.0, 0.0)
                pose.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                pub_correct_pose.publish(pose)
            elif e.button == 10:
                goal_button_pressed = True
            elif e.button == 11:
                goal_index += 1
                if len(pose_list) <= goal_index:
                    goal_index = 0

    mode = mode_list[mode_index]
    mode_changed = prev_mode_index != mode_index
    if mode_changed:
        print("mode:", mode)
        goal_is_canceled = True
        act_goal.cancel_all_goals()
        act_goal.stop_tracking_goal()

    if mode == "omni_vel":
        target_vx = -controller.max_velocity() * x_input
        target_vy = -controller.max_velocity() * y_input
        target_velangular = -controller.max_velangular() * yaw_input
        velocity, velangular = controller.update(np.array([target_vx, target_vy]), target_velangular)
        tw = Twist()
        tw.linear.x = velocity[0]
        tw.linear.y = velocity[1]
        tw.angular.z = velangular
        pub_omni.publish(tw)
    elif mode == "omni_goal":
        goal_pose = pose_list[goal_index]
        if goal_button_pressed:
            if goal_is_canceled:
                act_goal.send_goal(make_goal(goal_pose))
            else:
                act_goal.cancel_all_goals()
                act_goal.stop_tracking_goal()
            goal_is_canceled = not goal_is_canceled
    elif mode == "arm":
        norm = math.sqrt(x_input**2 + y_input**2 + z_input**2)
        if norm > 0.0:
            try:
                whole_body.impedance_config = 'compliance_soft'
                whole_body.move_end_effector_by_line((-z_input / norm, y_input / norm, -x_input / norm), 0.1 * norm)
                whole_body.impedance_config = None
            except:
                pass
    elif mode == "auto":
        state_changed = prev_auto_state != auto_state
        prev_auto_state = auto_state
        if auto_state == 0:
            # 原点に移動
            if act_goal.get_state() == GoalStatus.SUCCEEDED:
                auto_state = 1
                act_goal.cancel_all_goals()
                act_goal.stop_tracking_goal()
            elif state_changed or mode_changed:
                whole_body.move_to_go()
                goal_index = 0
                act_goal.send_goal(make_goal(pose_list[goal_index]))
        elif auto_state == 1:
            # 棚の前に移動，コップを把持
            if act_goal.get_state() == GoalStatus.SUCCEEDED:
                auto_state = 2
                act_goal.cancel_all_goals()
                act_goal.stop_tracking_goal()
                #　把持
                gripper.command(1.0, sync=False)
                whole_body.move_to_neutral()
                pan_list = [0.0, -0.5, 0.5]
                for pan in pan_list:
                    whole_body.impedance_config = None
                    whole_body.move_to_joint_positions({'head_tilt_joint': 0.2, 'head_pan_joint': pan})
                    whole_body.impedance_config = 'compliance_soft'
                    try:
                        whole_body.move_end_effector_pose([geometry.pose(y=0.25, ek=-1.57)], ref_frame_id='ar_marker/202')
                        break
                    except:
                        continue
                whole_body.impedance_config = 'compliance_soft'
                gripper.command(-0.3, sync=True)
                whole_body.move_end_effector_pose([geometry.pose(x=0.05, z=-0.05)], ref_frame_id='hand_palm_link')
                whole_body.impedance_config = None
                whole_body.move_to_go()
            elif state_changed or mode_changed:
                whole_body.move_to_go()
                goal_index = 1
                act_goal.send_goal(make_goal(pose_list[goal_index]))
        elif auto_state == 2:
            # 机の前に移動，コップを置く
            if act_goal.get_state() == GoalStatus.SUCCEEDED:
                auto_state = 0
                act_goal.cancel_all_goals()
                act_goal.stop_tracking_goal()
                # 解放
                whole_body.move_to_neutral()
                whole_body.impedance_config = 'compliance_soft'
                whole_body.move_end_effector_pose([geometry.pose(x=-0.35, z=0.2)], ref_frame_id='hand_palm_link')
                gripper.command(1.0, sync=True)
                whole_body.move_end_effector_pose([geometry.pose(x=0.1, z=-0.2)], ref_frame_id='hand_palm_link')
                whole_body.impedance_config = None
                whole_body.move_to_go()
            elif state_changed or mode_changed:
                whole_body.move_to_go()
                gripper.command(-0.3, sync=False)
                goal_index = 2
                act_goal.send_goal(make_goal(pose_list[goal_index]))

        if state_changed:
            print("goal_index:", goal_index)

    rospy_rate.sleep()
