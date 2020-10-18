#! /usr/bin/env python
from typing import List

import numpy as np
from more_itertools import pairwise

import rospy
from control_msgs.msg import JointTolerance, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

# These may be different because by making path tolerance larger,
# you might get smoother execution but still keep goal precision
DEFAULT_PATH_TOLERANCE_POSITION = 0.01
DEFAULT_GOAL_TOLERANCE_POSITION = 0.1


def get_ordered_tolerance_list(joint_names, tolerance: List[JointTolerance], is_goal: bool = False):
    tolerance_list = []
    for name in joint_names:
        default_tolerance_position = 0.01 if is_goal else 0.1
        tolerance_position = None
        for tolerance_for_name in tolerance:
            if tolerance_for_name.name == name:
                tolerance_position = tolerance_for_name.position
                break
        if tolerance_position is None:
            tolerance_position = default_tolerance_position
            rospy.logwarn_throttle(1, f"using default path tolerance {default_tolerance_position}")
        tolerance_list.append(tolerance_position)
    return tolerance_list


def make_joint_tolerance(pos, name):
    j = JointTolerance()
    j.position = pos
    j.velocity = 1
    j.acceleration = 100  # this is high because if we bump the arm, the controller will abort unnecessarily
    j.name = name
    return j


def waypoint_error(actual: JointTrajectoryPoint, desired: JointTrajectoryPoint):
    actual = np.array(actual.positions)
    desired = np.array(desired.positions)
    return np.abs(actual - desired)


def waypoint_reached(actual: JointTrajectoryPoint, desired: JointTrajectoryPoint, tolerance: List[float]):
    error = waypoint_error(actual, desired)
    tolerance = np.array(tolerance)
    return np.all(error < tolerance)


def interpolate_joint_trajectory_points(points: List[JointTrajectoryPoint], max_step_size: float):
    # TODO: support arbitrarily weighted norms, probably on a fixed-per-robot basis. So this could become a method
    # of the victor/val class, which has that weight as a class field
    if len(points) == 1:
        return points
    elif len(points) == 2:
        p1 = np.array(points[0].positions)
        p2 = np.array(points[1].positions)
        t0 = points[0].time_from_start.to_sec()
        t1 = points[1].time_from_start.to_sec()
        total_distance = np.linalg.norm(p1 - p2)
        n_steps = int(np.ceil(total_distance / max_step_size))
        interpolated_points_np = np.linspace(p1, p2, n_steps)
        interpolated_times = np.linspace(t0, t1, n_steps)
        interpolated_points = []
        for p_t, t in zip(interpolated_points_np, interpolated_times):
            p_msg = JointTrajectoryPoint()
            p_msg.positions = p_t
            p_msg.time_from_start = rospy.Duration(t)
            interpolated_points.append(p_msg)
        return interpolated_points
    else:
        interpolated_points = []
        for p1, p2 in pairwise(points):
            interpolated_points.extend(interpolate_joint_trajectory_points([p1, p2], max_step_size))
        return interpolated_points


def make_follow_joint_trajectory_goal(trajectory: JointTrajectory):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory = trajectory

    goal.goal_tolerance = [make_joint_tolerance(0.02, n) for n in trajectory.joint_names]
    goal.goal_time_tolerance = rospy.Duration(nsecs=500_000_000)
    return goal
