#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2017 Toyota Motor Corporation
"""Suction Controller Sample"""

import sys
import threading

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from tmc_suction.msg import (
    SuctionControlAction,
    SuctionControlGoal
)

import numpy as np
import math

_CONNECTION_TIMEOUT = 10.0

# Wait until pressure sensor is True
# If it is negative number, goal will be rejected

# continuous action time
_SUCTION_TIMEOUT = rospy.Duration(60.)

class Suction:
    def __init__(self,):
        suction_action = '/hsrb/suction_control'
        self.suction_control_client = actionlib.SimpleActionClient(
            suction_action, SuctionControlAction)

        # Wait for connection
        try:
            if not self.suction_control_client.wait_for_server(
                    rospy.Duration(_CONNECTION_TIMEOUT)):
                raise Exception(suction_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        # Send a goal to start suction
        rospy.loginfo('Suction will start')
        self.on_suction = False
        
    def execute_thread(self):
        suction_goal = SuctionControlGoal()
        suction_goal.timeout = _SUCTION_TIMEOUT
        suction_goal.suction_on.data = self.on_suction
        self.suction_control_client.send_goal_and_wait(suction_goal)

    def execute(self,):
        self.on_suction = not self.on_suction
        thr = threading.Thread(target=self.execute_thread)
        thr.start()



if __name__ == '__main__':
    rospy.init_node('hsrb_suction_controller')
    s = Suction()
    while True:
        do = input("input q for quit, other inputs control suction>>>")
        if do=="q":
            print("quit")
            break
        s.execute()        