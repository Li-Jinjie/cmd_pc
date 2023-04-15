#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Author: LI Jinjie
File: inner_msgs.py
Date: 2023/4/15 上午9:26
Description:
"""

import numpy as np


class MsgWaypoints:
    def __init__(self):
        self.num_waypoints = 0
        self.xyz_list = np.array([[], [], []])
        self.yaw_list = np.array([])
        self.speed_list = np.array([])

    def add(self, xyz=np.array([[0, 0, 0]]).T, yaw=0, speed=0):
        self.num_waypoints = self.num_waypoints + 1
        self.xyz_list = np.append(self.xyz_list, xyz, axis=1)
        self.yaw_list = np.append(self.yaw_list, yaw)
        self.speed_list = np.append(self.speed_list, speed)


class MsgTrajCoeff:
    def __init__(self):
        self.coeff_x = None
        self.coeff_y = None
        self.coeff_z = None
        self.coeff_yaw = None

        self.traj_time_seg = None
        self.traj_time_cum = None

        self.final_pos = None
