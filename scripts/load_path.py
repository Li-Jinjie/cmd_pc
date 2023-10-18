#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Author: LI Jinjie
Description:
"""

import rospy
import numpy as np
from traj_gen import MsgWaypoints


def ros_param_2_wpts(ros_name: str):
    wpts = MsgWaypoints()
    for pt in rospy.get_param(ros_name + "/path"):
        wpts.add(
            xyz=np.array([[pt["pos"][0], pt["pos"][1], pt["pos"][2]]]).T,
            yaw=np.radians(pt["yaw"]),
            speed=pt["vel"],
        )

    return wpts


def dict_2_wpts(dict_list: list):
    v_mean = 0.5
    wpts = MsgWaypoints()
    for d in dict_list:
        wpts.add(
            xyz=np.array([[d["x"], d["y"], d["z"]]]).T,
            yaw=np.radians(d["yaw"]) if "yaw" in d else 0,
            speed=d["speed"] if "speed" in d else v_mean,
        )
    # wpts.speed_list[0] = 0.0
    # wpts.speed_list[-1] = 0.0
    return wpts


_dict_lab_area_wpts = [
    dict(x=6.0, y=3.0, z=0.5),
    dict(x=6.0, y=-2.5, z=0.5),
    dict(x=2.0, y=-2.5, z=0.5),
    dict(x=0.0, y=-4.0, z=0.5),
    dict(x=-3.0, y=-4.0, z=0.5),
    dict(x=-4.0, y=-3.0, z=0.5),
    dict(x=-4.0, y=2.0, z=0.5),
    dict(x=0.0, y=3.0, z=0.5),
    dict(x=6.0, y=3.0, z=0.5),
]

lab_area_wpts = dict_2_wpts(_dict_lab_area_wpts)
