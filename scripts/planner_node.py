#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Author: LI Jinjie
File: planner_node.py
Date: 2023/3/30 下午9:50
Description:
"""
import sys
import os
import time

current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

import rospy
from transitions import Machine
import random
import actionlib

from oop_qd_onbd.msg import TrackTrajAction, TrackTrajGoal, TrackTrajResult, TrackTrajFeedback


class PlannerNode(object):
    # Define some states. Most of the time, narcoleptic superheroes are just like
    # everyone else. Except for...
    states = ["GROUND", "TAKEOFF", "PLANNING", "EXECUTION", "LAND"]
    # states = ["asleep", "hanging out", "hungry", "sweaty", "saving the world"]

    def __init__(self):
        rospy.init_node("planner_node", anonymous=False)

        # Action Client
        self.tracking_client = actionlib.SimpleActionClient("tracking_controller/track_traj", TrackTrajAction)
        self.tracking_client.wait_for_server()
        rospy.loginfo("Action Service exist: tracking_controller/track_traj")

        # Finite State machine
        self.machine = Machine(model=self, states=PlannerNode.states, initial="PLANNING")

        # self.machine.add_transition(trigger="take_off", source="GROUND", dest="EXECUTION", after="set_takeoff_traj")
        self.machine.add_transition(trigger="finish_planning", source="PLANNING", dest="EXECUTION", after="set_traj")
        self.machine.add_transition(trigger="reach_target", source="EXECUTION", dest="PLANNING", after="planning_traj")
        # self.machine.add_transition(trigger="land", source="*", dest="LAND", after="set_landing")
        # self.machine.add_transition(trigger="touch_ground", source="LAND", dest="GROUND")

        # start
        self.goal = TrackTrajGoal()
        self.planning_traj()

    def planning_traj(self):
        rospy.loginfo("Start planning trajectory......")

        # TODO: planning trajectory
        time.sleep(1)
        self.goal = TrackTrajGoal()

        rospy.loginfo("Finish planning trajectory.")
        self.finish_planning()  # trigger the FMS to change state

    def set_traj(self):
        self.tracking_client.send_goal(self.goal, feedback_cb=self.track_traj_feedback_cb)

        self.tracking_client.wait_for_result()

        result: TrackTrajResult = self.tracking_client.get_result()

        rospy.loginfo("Reach target.")
        rospy.loginfo(f"Trajectory tracking RMSE: {result.tracking_error_rmse} m")
        self.reach_target()  # trigger the FMS to change state

    def track_traj_feedback_cb(self, feedback: TrackTrajFeedback):
        rospy.loginfo(f"Trajectory tracking percent complete: {feedback.percent_complete}")

    # def set_takeoff_traj(self):
    #     pass
    #
    # def set_landing(self):
    #     pass


if __name__ == "__main__":
    try:
        node = PlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
