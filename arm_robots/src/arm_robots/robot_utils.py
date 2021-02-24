#! /usr/bin/env python
import itertools
from collections import defaultdict
from dataclasses import dataclass
from typing import List, Sequence, Optional, Tuple

import numpy as np
import rospy
from control_msgs.msg import JointTolerance, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from more_itertools import pairwise
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes
from rospy.logger_level_service_caller import LoggerLevelServiceCaller
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

# These may be different because by making path tolerance larger,
# you might get smoother execution but still keep goal precision
DEFAULT_PATH_TOLERANCE_POSITION = 0.01
DEFAULT_GOAL_TOLERANCE_POSITION = 0.1

# TODO Remove hardcoded parameter
LAG_IN_S = 0.07


def get_ordered_tolerance_list(joint_names, tolerance: Sequence[JointTolerance], is_goal: bool = False):
    def default_tolerance():
        tol = 0.01 if is_goal else 0.1
        rospy.logwarn_throttle(1, f"using default path tolerance {tol}")
        return tol

    tolerance_of = defaultdict(default_tolerance)
    tolerance_of.update({t.name: t.position for t in tolerance})
    return [tolerance_of[name] for name in joint_names]


def make_joint_tolerance(position_tolerance, name):
    return JointTolerance(position=position_tolerance,
                          velocity=2,
                          acceleration=100,  # high because if we bump the arm, the controller will abort unnecessarily
                          name=name)


def waypoint_error(actual: JointTrajectoryPoint, desired: JointTrajectoryPoint) -> float:
    actual = np.array(actual.positions)
    desired = np.array(desired.positions)
    return np.abs(actual - desired)


def is_waypoint_reached(actual: JointTrajectoryPoint, desired: JointTrajectoryPoint, tolerance: Sequence[float]):
    error = waypoint_error(actual, desired)
    tolerance = np.array(tolerance)
    if np.all(error < tolerance):
        return True

    anticipated_position = LAG_IN_S * np.array(desired.velocities) + np.array(actual.positions)
    anticipated_error = np.abs(anticipated_position - np.array(desired.positions))
    if np.all(anticipated_error < tolerance):
        return True
    return False


def _interpolate_joint_trajectory_points_positions(points: List[JointTrajectoryPoint], max_step_size: float) \
        -> List[JointTrajectoryPoint]:
    """
    Interpolates the positions of a list of joint trajectory points
    """
    # TODO: support arbitrarily weighted norms, probably on a fixed-per-robot basis. So this could become a method
    # of the victor/val class, which has that weight as a class field
    if len(points) <= 1:
        return points
    if len(points) == 2:
        p1 = np.array(points[0].positions)
        p2 = np.array(points[1].positions)
        t0 = points[0].time_from_start.to_sec()
        t1 = points[1].time_from_start.to_sec()
        total_distance = np.linalg.norm(p1 - p2)
        n_steps = int(np.ceil(total_distance / max_step_size)) + 1
        interpolated_points_np = np.linspace(p1, p2, n_steps)
        interpolated_times = np.linspace(t0, t1, n_steps)
        interpolated_points = [JointTrajectoryPoint(positions=p_t, time_from_start=rospy.Duration(t))
                               for p_t, t in zip(interpolated_points_np, interpolated_times)]
        return interpolated_points

    interpolated_points = []
    for p1, p2 in pairwise(points):
        interpolated_points.extend(_interpolate_joint_trajectory_points_positions([p1, p2], max_step_size)[:-1])
    interpolated_points.append(points[-1])
    return interpolated_points


def interpolate_joint_trajectory_points(points: Sequence[JointTrajectoryPoint], max_step_size: float) \
        -> List[JointTrajectoryPoint]:
    """
    Interpolates between positions in points and recalculates velocities using point times,
    assuming 0-velocity endpoints.

    All original velocity values (if present) are discarded. Acceleration limits are not enforced.
    """
    interpolated_points = _interpolate_joint_trajectory_points_positions(points, max_step_size)
    interpolated_points[0].velocities = [0.0] * len(interpolated_points[0].positions)
    interpolated_points[-1].velocities = [0.0] * len(interpolated_points[-1].positions)
    lower, mid, upper = itertools.tee(interpolated_points, 3)
    next(mid, None)
    next(upper, None)
    next(upper, None)
    for l, m, u in zip(lower, mid, upper):
        m.velocities = (u.positions - l.positions) / (u.time_from_start - l.time_from_start).to_sec()
    return interpolated_points


def make_follow_joint_trajectory_goal(trajectory: JointTrajectory) -> FollowJointTrajectoryGoal:
    goal = FollowJointTrajectoryGoal(trajectory=trajectory)
    goal.path_tolerance = [make_joint_tolerance(0.1, n) for n in trajectory.joint_names]
    goal.goal_tolerance = [make_joint_tolerance(0.05, n) for n in trajectory.joint_names]
    goal.goal_time_tolerance = rospy.Duration(nsecs=500_000_000)
    return goal


@dataclass
class ExecutionResult:
    trajectory: Optional[JointTrajectory]
    execution_result: Optional[FollowJointTrajectoryResult]
    action_client_state: int
    success: bool


class PlanningResult:

    def __init__(self,
                 move_group_plan_tuple: Optional[Tuple[bool, RobotTrajectory, float, MoveItErrorCodes]] = None,
                 success: Optional[bool] = None,
                 plan: Optional[RobotTrajectory] = None,
                 planning_time: Optional[float] = None,
                 planning_error_code: Optional[int] = None,
                 ):
        if move_group_plan_tuple is not None:
            self.success = move_group_plan_tuple[0]
            self.plan = move_group_plan_tuple[1]
            self.planning_time = move_group_plan_tuple[2]
            self.planning_error_code = move_group_plan_tuple[3]
        else:
            self.success = success
            self.plan = plan
            self.planning_time = planning_time
            self.planning_error_code = planning_error_code


class PlanningAndExecutionResult:
    def __init__(self, planning_result: PlanningResult, execution_result: ExecutionResult):
        self.planning_result = planning_result
        self.execution_result = execution_result
        self.success = planning_result.success or execution_result.success


def set_move_group_log_level(event):
    # Up the logging level for MoveGroupInterface because it's annoying
    log_level = LoggerLevelServiceCaller()
    node_name = "cpp_" + rospy.get_name().strip("/")
    logger_name = "ros.moveit_ros_planning_interface.move_group_interface"
    node_names = log_level.get_node_names()
    if node_name in node_names:
        loggers = log_level.get_loggers(node_name)
        if logger_name in loggers:
            try:
                success = log_level.send_logger_change_message(node_name, logger_name, "WARN")
                rospy.logdebug_once(f"status of changing move_group_interface logger level: {success}")
            except Exception:
                pass


def is_empty_trajectory(trajectory: JointTrajectory):
    if len(trajectory.points) == 0:
        return True
    elif len(trajectory.points) == 1 and trajectory.points[0].time_from_start == rospy.Duration(0):
        return True
    return False
