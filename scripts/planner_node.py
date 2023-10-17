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
import actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ndp_nmpc.msg import TrackTrajAction, TrackTrajGoal, TrackTrajResult, TrackTrajFeedback

from traj_gen import TrajGenerator, MsgWaypoints
from waypoints import eight_wpt_1, lab_area_wpts, eight_wpt_3, eight_wpt_1_diff_h


class PlannerNode(object):
    # Define some states. Most of the time, narcoleptic superheroes are just like
    # everyone else. Except for...
    states = ["GROUND", "TAKEOFF", "PLANNING", "EXECUTION", "LAND"]
    # states = ["asleep", "hanging out", "hungry", "sweaty", "saving the world"]

    def __init__(self):
        rospy.init_node("planner_node", anonymous=False)

        # Trajectory generator
        self.traj_generator = TrajGenerator()

        # Action Client
        self.tracking_client = actionlib.SimpleActionClient(
            "/fhnp/traj_tracker/pt_pub_action_server", TrackTrajAction  # TODO: change name
        )
        self.tracking_client.wait_for_server()
        rospy.loginfo("Action Service exist: /fhnp/traj_tracker/pt_pub_action_server")

        # Finite State machine
        self.machine = Machine(model=self, states=PlannerNode.states, initial="PLANNING")

        # self.machine.add_transition(trigger="take_off", source="GROUND", dest="EXECUTION", after="set_takeoff_traj")
        self.machine.add_transition(trigger="finish_planning", source="PLANNING", dest="EXECUTION", after="set_traj")
        self.machine.add_transition(trigger="reach_target", source="EXECUTION", dest="PLANNING", after="planning_traj")
        # self.machine.add_transition(trigger="land", source="*", dest="LAND", after="set_landing")
        # self.machine.add_transition(trigger="touch_ground", source="LAND", dest="GROUND")

        # viz
        self.lab_area_pub = rospy.Publisher("path_planner/viz_lab_area/", Path, queue_size=10)
        self.path_pub = rospy.Publisher("path_planner/viz_path/", Path, queue_size=10)
        self.traj_pub = rospy.Publisher("path_planner/viz_traj/", Path, queue_size=10)

        # start
        self.goal = TrackTrajGoal()
        self.planning_traj()

    def planning_traj(self):
        rospy.loginfo("Start planning trajectory......")

        # -------- path planner ----------
        waypoints = eight_wpt_1  # change here!
        self.viz_path(waypoints, self.path_pub)
        rospy.loginfo("Get path points!")

        self.viz_path(lab_area_wpts, self.lab_area_pub)
        rospy.loginfo("Pub lab area!")

        # -------- trajectory generator ----------
        rospy.loginfo("Generating trajectory......")
        traj_coeff, all_pose_list = self.traj_generator.generate_traj(waypoints)

        self.goal = TrackTrajGoal()
        self.goal.traj_coeff = traj_coeff

        wpts_traj = MsgWaypoints()
        for i in range(len(all_pose_list[0])):
            wpts_traj.add(xyz=all_pose_list[:, i : i + 1])
        self.viz_path(wpts_traj, self.traj_pub)

        rospy.loginfo("Finish planning trajectory!")
        self.finish_planning()  # trigger the FMS to change state

    def set_traj(self):
        self.tracking_client.send_goal(self.goal, feedback_cb=self.track_traj_feedback_cb)

        self.tracking_client.wait_for_result()

        result: TrackTrajResult = self.tracking_client.get_result()

        rospy.loginfo("Reach target.")
        rospy.loginfo(f"Trajectory tracking RMSE: {result.pos_error_rmse} [m]")
        rospy.loginfo(f"Yaw angle tracking RMSE: {result.yaw_error_rmse} [deg]")
        self.reach_target()  # trigger the FMS to change state

    def track_traj_feedback_cb(self, feedback: TrackTrajFeedback) -> None:
        rospy.loginfo_throttle(1, f"Trajectory tracking percent complete: {100 * feedback.percent_complete:.2f}%")

    @staticmethod
    def viz_path(waypoints: MsgWaypoints, pub: rospy.Publisher):
        wpts_path = Path()
        for i in range(waypoints.num_waypoints):
            pt = PoseStamped()
            pt.pose.position.x = waypoints.xyz_list[0, i]
            pt.pose.position.y = waypoints.xyz_list[1, i]
            pt.pose.position.z = waypoints.xyz_list[2, i]
            wpts_path.poses.append(pt)

        wpts_path.header.stamp = rospy.Time.now()
        wpts_path.header.frame_id = "map"

        pub.publish(wpts_path)

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
