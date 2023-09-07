#!/usr/bin/env python
# -*- encoding: ascii -*-
"""
Author: LI Jinjie
File: minimum_snap.py
Date: 2022/5/9 17:36
LastEditors: LI Jinjie
LastEditTime: 2022/5/9 17:36
Description: trajectory generator, using closed-form Minimum Snap method.
"""
import rospy
import numpy as np

from .polym_optimizer import PolymOptimizer, MinMethod
from .inner_msgs import MsgWaypoints, MsgTrajCoeff
from ndp_nmpc_qd.msg import TrajCoefficients, TrajPt


class TrajGenerator:
    def __init__(self, xyz_method=MinMethod.SNAP, yaw_method=MinMethod.ACCEL) -> None:
        self.optim_x = PolymOptimizer(xyz_method)
        self.optim_y = PolymOptimizer(xyz_method)
        self.optim_z = PolymOptimizer(xyz_method)
        self.optim_yaw = PolymOptimizer(yaw_method)

        self.traj_coeff = MsgTrajCoeff()

        # state
        self.tracking_pos_err = 0  # m
        self.tracking_yaw_err = 0  # deg
        self.tracking_pt_num = 0

        self.is_activated = True  # if finish the whole trajectory, False.

    def reset(self) -> None:
        self.tracking_pos_err = 0  # m
        self.tracking_yaw_err = 0  # deg
        self.tracking_pt_num = 0

        self.is_activated = True

    def generate_traj(self, waypoints: MsgWaypoints) -> (TrajCoefficients, list):
        """generate trajectory coefficients from waypoints

        Args:
            waypoints: MsgWaypoints
        """
        traj_coeff = MsgTrajCoeff()

        """ Time Allocation
        % this code: t = d / v_mean
        % Other method: Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments
        % transfer time allocation into another optimization problem
        """
        distance = waypoints.xyz_list[:, 1:] - waypoints.xyz_list[:, 0:-1]

        v_mean = (waypoints.speed_list[:-1] + waypoints.speed_list[1:]) / 2

        traj_time_sep = (
            np.sqrt(np.square(distance[0, :]) + np.square(distance[1, :]) + np.square(distance[2, :])) / v_mean
        )

        traj_coeff.traj_time_seg = traj_time_sep
        traj_coeff.traj_time_cum = np.insert(np.cumsum(traj_time_sep), 0, 0.0)

        """ Minimum Snap
        % According to the slides from Fei Gao, the method here is "Closed-form Solution to Minimum Snap"
        % but the implementation details are different from the article below:
        % "Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments"

        % Kumar Coursera Method. The mathematical formula is the same with the above article.
        % Just make the matrix A full rank by adding constraints, then C=A/b.
        % Kumar tells how to optimize using calculus of variations. He is more insightful about why choose 7-order Polynomial.
        """
        x_wpts = waypoints.xyz_list[0, :]
        traj_coeff.coeff_x = self.optim_x.get_coeff(x_wpts)
        y_wpts = waypoints.xyz_list[1, :]
        traj_coeff.coeff_y = self.optim_y.get_coeff(y_wpts)
        z_wpts = waypoints.xyz_list[2, :]
        traj_coeff.coeff_z = self.optim_z.get_coeff(z_wpts)
        yaw_wpts = waypoints.yaw_list
        traj_coeff.coeff_yaw = self.optim_yaw.get_coeff(yaw_wpts)

        traj_coeff.final_pos = waypoints.xyz_list[:, -1:]

        self.traj_coeff = traj_coeff

        all_pos_list = self._check_continuity()

        ros_traj_coeff = TrajCoefficients()
        ros_traj_coeff.coeff_x = np.squeeze(self.traj_coeff.coeff_x).tolist()
        ros_traj_coeff.coeff_y = np.squeeze(self.traj_coeff.coeff_y).tolist()
        ros_traj_coeff.coeff_z = np.squeeze(self.traj_coeff.coeff_z).tolist()
        ros_traj_coeff.coeff_yaw = np.squeeze(self.traj_coeff.coeff_yaw).tolist()
        ros_traj_coeff.traj_time_cum = np.squeeze(self.traj_coeff.traj_time_cum).tolist()
        ros_traj_coeff.traj_time_seg = np.squeeze(self.traj_coeff.traj_time_seg).tolist()
        ros_traj_coeff.final_pt.x = self.traj_coeff.final_pos.item(0)
        ros_traj_coeff.final_pt.y = self.traj_coeff.final_pos.item(1)
        ros_traj_coeff.final_pt.z = self.traj_coeff.final_pos.item(2)

        return ros_traj_coeff, all_pos_list

    def get_traj_pt(self, t: float) -> TrajPt:
        traj_pt = TrajPt()

        if t >= self.traj_coeff.traj_time_cum[-1]:  # change to "hover" after finished
            traj_pt.position.x = self.traj_coeff.final_pos.item(0)
            traj_pt.position.y = self.traj_coeff.final_pos.item(1)
            traj_pt.position.z = self.traj_coeff.final_pos.item(2)

            self.is_activated = False
            return traj_pt

        time_cum = self.traj_coeff.traj_time_cum
        time_seg = self.traj_coeff.traj_time_seg
        t_index = np.argwhere(time_cum > t)[0].item() - 1  # t_index ? S_i

        t_segment = time_seg[t_index]
        t_scaled = (t - time_cum[t_index]) / t_segment

        # x,y,z
        c_x = _get_specific_coeff(t_index, self.optim_x, self.traj_coeff.coeff_x)
        c_y = _get_specific_coeff(t_index, self.optim_y, self.traj_coeff.coeff_y)
        c_z = _get_specific_coeff(t_index, self.optim_z, self.traj_coeff.coeff_z)

        traj_pt.position.x = _get_output_value(self.optim_x, 0, t_scaled, t_segment, c_x)
        traj_pt.position.y = _get_output_value(self.optim_y, 0, t_scaled, t_segment, c_y)
        traj_pt.position.z = _get_output_value(self.optim_z, 0, t_scaled, t_segment, c_z)

        traj_pt.velocity.x = _get_output_value(self.optim_x, 1, t_scaled, t_segment, c_x)
        traj_pt.velocity.y = _get_output_value(self.optim_y, 1, t_scaled, t_segment, c_y)
        traj_pt.velocity.z = _get_output_value(self.optim_z, 1, t_scaled, t_segment, c_z)

        traj_pt.accel.x = _get_output_value(self.optim_x, 2, t_scaled, t_segment, c_x)
        traj_pt.accel.y = _get_output_value(self.optim_y, 2, t_scaled, t_segment, c_y)
        traj_pt.accel.z = _get_output_value(self.optim_z, 2, t_scaled, t_segment, c_z)

        traj_pt.jerk.x = _get_output_value(self.optim_x, 3, t_scaled, t_segment, c_x)
        traj_pt.jerk.y = _get_output_value(self.optim_y, 3, t_scaled, t_segment, c_y)
        traj_pt.jerk.z = _get_output_value(self.optim_z, 3, t_scaled, t_segment, c_z)

        # yaw
        c_yaw = _get_specific_coeff(t_index, self.optim_yaw, self.traj_coeff.coeff_yaw)
        traj_pt.yaw = _get_output_value(self.optim_yaw, 0, t_scaled, t_segment, c_yaw)
        traj_pt.yaw_dot = _get_output_value(self.optim_yaw, 1, t_scaled, t_segment, c_yaw)

        return traj_pt

    def _check_continuity(self) -> list:
        rospy.loginfo("Checking the trajectory......")
        t_sim = 0
        dt = 0.1
        all_pos = []
        all_vel = []
        all_acc = []
        all_jerk = []
        all_yaw = []
        all_yaw_dot = []
        while True:
            traj_pt = self.get_traj_pt(t=t_sim)
            t_sim += dt

            all_pos.append([traj_pt.position.x, traj_pt.position.y, traj_pt.position.z])
            all_vel.append([traj_pt.velocity.x, traj_pt.velocity.y, traj_pt.velocity.z])
            all_acc.append([traj_pt.accel.x, traj_pt.accel.y, traj_pt.accel.z])
            all_jerk.append([traj_pt.jerk.x, traj_pt.jerk.y, traj_pt.jerk.z])
            all_yaw.append([traj_pt.yaw])
            all_yaw_dot.append([traj_pt.yaw_dot])

            if self.is_activated is False:
                all_pos = np.array(all_pos).T
                all_vel = np.array(all_vel).T
                all_acc = np.array(all_acc).T
                all_jerk = np.array(all_jerk).T
                all_yaw = np.array(all_yaw)
                all_yaw_dot = np.array(all_yaw_dot)

                numeric_vel = np.gradient(all_pos, axis=1) / dt
                numeric_acc = np.gradient(all_vel, axis=1) / dt
                numeric_jerk = np.gradient(all_acc, axis=1) / dt
                numeric_yaw_dot = np.gradient(all_yaw, axis=0) / dt

                # refer to "data_driven mpc"
                for i in range(self.tracking_pt_num):
                    # 1) check if velocity is consistent with position
                    if not np.allclose(all_vel[:, i], numeric_vel[:, i], atol=1e-2, rtol=1e-2):
                        print(f"time: {i * dt}; analytic_vel: {all_vel[:, i]:f}; numeric_vel: {numeric_vel[:, i]:f}")
                        raise ValueError("inconsistent velocity")

                    # 2) check if acceleration is consistent with velocity
                    if not np.allclose(all_acc[:, i], numeric_acc[:, i], atol=1e-2, rtol=1e-2):
                        print(f"time: {i * dt}; analytic_acc: {all_acc[:, i]:f}; numeric_acc: {numeric_acc[:, i]:f}")
                        raise ValueError("inconsistent acceleration")

                    # 3) check if jerk is consistent with acceleration
                    if not np.allclose(all_jerk[:, i], numeric_jerk[:, i], atol=1e-2, rtol=1e-2):
                        print(
                            f"time: {i * dt}; analytic_jerk: {all_jerk[:, i]:f}; numeric_jerk: {numeric_jerk[:, i]:f}"
                        )
                        raise ValueError("inconsistent jerk")

                    # 4) check if yaw_dot is consistent with yaw
                    if not np.allclose(all_yaw_dot[i], numeric_yaw_dot[i], atol=1e-2, rtol=1e-2):
                        print(
                            f"time: {i * dt}; analytic_yaw_dot: {all_yaw_dot[i]:f}; numeric_yaw_dot: {numeric_yaw_dot[i]:f}"
                        )
                        raise ValueError("inconsistent yaw_dot")

                rospy.loginfo("The Trajectory is consistent!")
                message = (
                    f"\n=============== trajectory info ===============\n"
                    f"X_max = {np.max(all_pos[0, :]):.4f} [m]; X_min = {np.min(all_pos[0, :]):.4f} [m]\n"
                    f"Y_max = {np.max(all_pos[1, :]):.4f} [m]; Y_min = {np.min(all_pos[1, :]):.4f} [m]\n"
                    f"Z_max = {np.max(all_pos[2, :]):.4f} [m]; Z_min = {np.min(all_pos[2, :]):.4f} [m]\n"
                    f"vel_max = {np.max(np.linalg.norm(all_vel, ord=2, axis=0)):.4f} [m/s]\n"
                    f"accel_max = {np.max(np.linalg.norm(all_acc, ord=2, axis=0)):.4f} [m/s^2]\n"
                    f"yaw_max = {np.degrees(np.max(all_yaw)):.4f} [deg]; yaw_min = {np.degrees(np.min(all_yaw)):.4f} [deg]\n"
                    f"yaw_dot_max = {np.degrees(np.max(all_yaw_dot)):.4f} [deg/s]\n"
                    f"================================================\n"
                )
                print(message)

                self.reset()  # mainly err counter
                break
        return all_pos


def _get_output_value(
    optim: PolymOptimizer, deriv_num: int, t_scaled: float, t_segment: float, coeff: np.array
) -> float:
    """get the output value under the specific deriv order, time, and coefficient. scale the time here

    Returns:
        real output value: float
    """
    poly_time_d = optim.get_poly_params(deriv_num, t_scaled) / (np.power(t_segment, deriv_num))
    return (poly_time_d @ coeff).item()


def _get_specific_coeff(t_index: int, optimizer: PolymOptimizer, coeff_all: np.array) -> np.array:
    """get polynomial parameters in this segment from all coefficients

    Args:
        t_index: which segment in waypoints
        optimizer: polynomial optimizer
        coeff_all: all parameters computed before

    Returns:
        polynomial parameters in this segment

    """
    n = optimizer.ord_polym
    row_shift = t_index * (n + 1)
    return coeff_all[row_shift : row_shift + (n + 1), :]
