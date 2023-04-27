#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Author: LI Jinjie
File: waypoints.py
Date: 2023/4/15 上午9:21
Description:
"""
import numpy as np
from traj_gen import MsgWaypoints

_dict_eight_wpt_1 = [
    dict(x=1.0, y=1.0, z=0.5, yaw=np.radians(0)),
    dict(x=3.0, y=2.0, z=0.5, yaw=np.radians(0)),
    dict(x=5.0, y=1.0, z=0.5, yaw=np.radians(0)),
    dict(x=3.0, y=0.0, z=0.5, yaw=np.radians(0)),
    dict(x=1.0, y=1.0, z=0.5, yaw=np.radians(0)),
    dict(x=-1.0, y=2.0, z=0.5, yaw=np.radians(0)),
    dict(x=-3.0, y=1.0, z=0.5, yaw=np.radians(0)),
    dict(x=-1.0, y=0.0, z=0.5, yaw=np.radians(0)),
    dict(x=1.0, y=1.0, z=0.5, yaw=np.radians(0)),
]

_dict_eight_wpt_1_diff_h = [
    dict(x=1.0, y=1.0, z=0.5, yaw=np.radians(0)),
    dict(x=3.0, y=2.0, z=1.0, yaw=np.radians(0)),
    dict(x=5.0, y=1.0, z=1.0, yaw=np.radians(0)),
    dict(x=3.0, y=0.0, z=1.0, yaw=np.radians(0)),
    dict(x=1.0, y=1.0, z=0.5, yaw=np.radians(0)),
    dict(x=-1.0, y=2.0, z=0.0, yaw=np.radians(0)),
    dict(x=-3.0, y=1.0, z=0.0, yaw=np.radians(0)),
    dict(x=-1.0, y=0.0, z=0.0, yaw=np.radians(0)),
    dict(x=1.0, y=1.0, z=0.5, yaw=np.radians(0)),
]

_dict_eight_wpt_2 = [
    dict(x=1.0, y=1.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=3.0, y=0.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=5.0, y=1.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=3.0, y=2.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=1.0, y=1.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=-1.0, y=0.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=-3.0, y=1.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=-1.0, y=2.0 - 1.5, z=1.5, yaw=np.radians(0)),
    dict(x=1.0, y=1.0 - 1.5, z=1.5, yaw=np.radians(0)),
]

_dict_eight_wpt_3 = [
    dict(x=1.0, y=1.0, z=5, yaw=np.radians(0), speed=5),
    dict(x=6.0, y=6.0, z=6.5, yaw=np.radians(0.1), speed=5),
    dict(x=11.0, y=1.0, z=8, yaw=np.radians(0), speed=5),
    dict(x=6.0, y=-4.0, z=6.5, yaw=np.radians(-0.1), speed=5),
    dict(x=1.0, y=1.0, z=5, yaw=np.radians(0), speed=5),
    dict(x=-4.0, y=6.0, z=3.5, yaw=np.radians(0), speed=5),
    dict(x=-9.0, y=1.0, z=2, yaw=np.radians(0), speed=5),
    dict(x=-4.0, y=-4.0, z=3.5, yaw=np.radians(0), speed=5),
    dict(x=1.0, y=1.0, z=5, yaw=np.radians(0), speed=5),
    dict(x=6.0, y=6.0, z=6.5, yaw=np.radians(0.1), speed=5),
    dict(x=11.0, y=1.0, z=8, yaw=np.radians(0), speed=5),
    dict(x=6.0, y=-4.0, z=6.5, yaw=np.radians(-0.1), speed=5),
    dict(x=1.0, y=1.0, z=5, yaw=np.radians(0), speed=5),
    dict(x=-4.0, y=6.0, z=3.5, yaw=np.radians(0), speed=5),
    dict(x=-9.0, y=1.0, z=2, yaw=np.radians(0), speed=5),
    dict(x=-4.0, y=-4.0, z=3.5, yaw=np.radians(0), speed=5),
    dict(x=1.0, y=1.0, z=5, yaw=np.radians(0), speed=5),
]

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

v_mean = 0.5


def dict_2_waypoints(dict_list: list):
    wpts = MsgWaypoints()
    for d in dict_list:
        wpts.add(
            xyz=np.array([[d["x"], d["y"], d["z"]]]).T,
            yaw=d["yaw"] if "yaw" in d else 0,
            speed=d["speed"] if "speed" in d else v_mean,
        )
    return wpts


eight_wpt_1 = dict_2_waypoints(_dict_eight_wpt_1)
eight_wpt_1_diff_h = dict_2_waypoints(_dict_eight_wpt_1_diff_h)
eight_wpt_2 = dict_2_waypoints(_dict_eight_wpt_2)
eight_wpt_3 = dict_2_waypoints(_dict_eight_wpt_3)
lab_area_wpts = dict_2_waypoints(_dict_lab_area_wpts)
