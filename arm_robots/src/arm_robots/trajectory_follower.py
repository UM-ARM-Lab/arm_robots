#! /usr/bin/env python
import pathlib
from typing import List, Callable, Optional

import actionlib
import rospy
from arm_robots.base_robot import DualArmRobot
from arm_robots.robot_utils import get_ordered_tolerance_list, interpolate_joint_trajectory_points, is_waypoint_reached, \
    waypoint_error
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, \
    FollowJointTrajectoryResult
from rosgraph.names import ns_join
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryFollower:
    def __init__(self, robot: DualArmRobot, controller_name: str):
        self.robot = robot
        self.action_name = ns_join(robot.robot_namespace, ns_join(controller_name, "follow_joint_trajectory"))
        self.server = actionlib.SimpleActionServer(self.action_name, FollowJointTrajectoryAction,
                                                   execute_cb=self.follow_trajectory_cb,
                                                   auto_start=False)

        self.client = actionlib.SimpleActionClient(self.action_name, FollowJointTrajectoryAction)

    def start_server(self):
        self.server.start()

    def follow_trajectory_goal(self,
                               traj_msg: FollowJointTrajectoryGoal,
                               feedback_cb: Optional[Callable] = None,
                               stop_cb: Optional[Callable] = lambda: (False, "")):

        # Interpolate the trajectory to a fine resolution
        # if you set max_step_size to be large and position tolerance to be small, then things will be jerky
        if len(traj_msg.trajectory.points) == 0:
            rospy.loginfo("Ignoring empty trajectory")
            return FollowJointTrajectoryResult(error_code=FollowJointTrajectoryResult.SUCCESSFUL,
                                               error_string="empty trajectory")

        # construct a list of the tolerances in order of the joint names
        trajectory_joint_names = traj_msg.trajectory.joint_names
        tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.path_tolerance)
        goal_tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.goal_tolerance, is_goal=True)
        interpolated_points = interpolate_joint_trajectory_points(traj_msg.trajectory.points, max_step_size=0.01)

        if len(interpolated_points) == 0:
            rospy.loginfo("Trajectory was empty after interpolation")
            return FollowJointTrajectoryResult(error_code=FollowJointTrajectoryResult.SUCCESSFUL,
                                               error_string="empty trajectory")

        rospy.logdebug(f"Interpolated points: {interpolated_points}")

        trajectory_point_idx = 0
        t0 = rospy.Time.now()
        while True:
            # tiny sleep lets the listeners process messages better, results in smoother following
            rospy.sleep(1e-6)

            # get feedback
            new_waypoint = False
            actual_point = self.get_actual_trajectory_point(trajectory_joint_names)
            while trajectory_point_idx < len(interpolated_points) - 1 and \
                    is_waypoint_reached(interpolated_points[trajectory_point_idx], actual_point, tolerance):
                trajectory_point_idx += 1
                new_waypoint = True

            desired_point = interpolated_points[trajectory_point_idx]

            if trajectory_point_idx >= len(interpolated_points) - 1 and \
                    is_waypoint_reached(desired_point, actual_point, goal_tolerance):
                return FollowJointTrajectoryResult(error_code=FollowJointTrajectoryResult.SUCCESSFUL)

            if new_waypoint:
                command_failed, command_failed_msg = self.robot.send_joint_command(trajectory_joint_names,
                                                                                   desired_point)
                if command_failed:
                    rospy.logwarn(f"Command failed with message: {command_failed}")
                    return FollowJointTrajectoryResult(error_code=-10, error_string=command_failed_msg)

            # let the caller stop
            stop, stop_msg = stop_cb(actual_point)

            dt = rospy.Time.now() - t0
            error = waypoint_error(actual_point, desired_point)
            rospy.logdebug_throttle(1, f"{error} {desired_point.time_from_start.to_sec()} {dt.to_sec()}")
            if desired_point.time_from_start.to_sec() > 0 and dt > desired_point.time_from_start * 5.0:
                stop = True
                if trajectory_point_idx == len(interpolated_points) - 1:
                    stop_msg = f"timeout. expected t={desired_point.time_from_start.to_sec()} but t={dt.to_sec()}." \
                               + f" error to waypoint is {error}, goal tolerance is {goal_tolerance}"
                else:
                    stop_msg = f"timeout. expected t={desired_point.time_from_start.to_sec()} but t={dt.to_sec()}." \
                               + f" error to waypoint is {error}, tolerance is {tolerance}"

            if stop:
                # command the current configuration
                actual_point.velocities = [0] * len(actual_point.positions)
                self.robot.send_joint_command(trajectory_joint_names, actual_point)
                rospy.loginfo("Preempt requested, aborting.")
                rospy.logwarn(f"Stopped with message: {stop_msg}")
                return FollowJointTrajectoryResult(error_code=-10, error_string=stop_msg)

            if feedback_cb is not None:
                feedback = FollowJointTrajectoryFeedback(desired=desired_point,
                                                         actual=actual_point,
                                                         joint_names=trajectory_joint_names)
                feedback.header.stamp = rospy.Time.now()
                feedback_cb(feedback)

    def follow_trajectory_cb(self, traj_msg: FollowJointTrajectoryGoal):
        def _feedback_cb(feedback):
            self.server.publish_feedback(feedback)

        def _stop_cb(*args, **kwargs):
            stop = self.server.is_preempt_requested()
            return stop, "Goal cancelled" if stop else ""

        result = self.follow_trajectory_goal(traj_msg, _feedback_cb, _stop_cb)
        # TODO: crappy api here
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def get_actual_trajectory_point(self, trajectory_joint_names: List[str]) -> JointTrajectoryPoint:
        return JointTrajectoryPoint(positions=self.robot.get_joint_positions(trajectory_joint_names))

