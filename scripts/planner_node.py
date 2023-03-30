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

current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

import rospy
from transitions import Machine
import random


class PlannerNode(object):
    # Define some states. Most of the time, narcoleptic superheroes are just like
    # everyone else. Except for...
    states = ["GROUND", "TAKEOFF", "PLANNING", "EXECUTION", "LAND"]
    # states = ["asleep", "hanging out", "hungry", "sweaty", "saving the world"]

    def __init__(self, name):
        # No anonymous superheroes on my watch! Every narcoleptic superhero gets
        # a name. Any name at all. SleepyMan. SlumberGirl. You get the idea.
        self.name = name

        # What have we accomplished today?
        self.kittens_rescued = 0

        # Initialize the state machine
        self.machine = Machine(model=self, states=PlannerNode.states, initial="GROUND")

        # Add some transitions. We could also define these using a static list of
        # dictionaries, as we did with states above, and then pass the list to
        # the Machine initializer as the transitions= argument.

        # At some point, every superhero must rise and shine.
        self.machine.add_transition(trigger="take_off", source="GROUND", dest="EXECUTION", after="pub_takeoff_traj")
        self.machine.add_transition(trigger="finish_planning", source="PLANNING", dest="EXECUTION", after="pub_traj")
        self.machine.add_transition(trigger="reach_target", source="EXECUTION", dest="PLANNING", after="planning_traj")
        self.machine.add_transition(trigger="land", source="*", dest="LAND", after="pub_landing")
        self.machine.add_transition(trigger="touch_ground", source="LAND", dest="GROUND")

    def pub_takeoff_traj(self):
        pass

    def pub_traj(self):
        pass

    def planning_traj(self):
        pass

    def pub_landing(self):
        pass
