#! /usr/bin/env python
import pathlib
from typing import List, Callable, Optional

import actionlib
import rospy
from arm_robots.base_robot import DualArmRobot
from arm_robots.robot_utils import get_ordered_tolerance_list, interpolate_joint_trajectory_points, waypoint_reached, \
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
                               stop_cb: Optional[Callable] = None):
        result = FollowJointTrajectoryResult()

        # Interpolate the trajectory to a fine resolution
        # if you set max_step_size to be large and position tolerance to be small, then things will be jerky
        if len(traj_msg.trajectory.points) == 0:
            rospy.loginfo("Ignoring empty trajectory")
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            result.error_string = "empty trajectory"
            return result

        # construct a list of the tolerances in order of the joint names
        trajectory_joint_names = traj_msg.trajectory.joint_names
        tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.path_tolerance)
        goal_tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.goal_tolerance, is_goal=True)

        interpolated_points = interpolate_joint_trajectory_points(traj_msg.trajectory.points, max_step_size=0.1)
        if len(interpolated_points) == 0:
            rospy.loginfo("Trajectory was empty after interpolation")
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            result.error_string = "empty trajectory"
            return result

        rospy.logdebug(interpolated_points)

        trajectory_point_idx = 0
        t0 = rospy.Time.now()
        while True:
            # tiny sleep lets the listeners process messages better, results in smoother following
            rospy.sleep(1e-6)

            desired_point = interpolated_points[trajectory_point_idx]

            command_failed, command_failed_msg = self.robot.send_joint_command(trajectory_joint_names, desired_point)

            # get feedback
            actual_point = self.get_actual_trajectory_point(trajectory_joint_names)

            # let the caller stop
            if stop_cb is not None:
                stop, stop_msg = stop_cb(actual_point)
            else:
                stop = None
                stop_msg = ""

            dt = rospy.Time.now() - t0
            error = waypoint_error(actual_point, desired_point)
            rospy.logdebug_throttle(1, f"{error} {desired_point.time_from_start.to_sec()} {dt.to_sec()}")
            if desired_point.time_from_start.to_sec() > 0 and dt > desired_point.time_from_start * 5.0:
                stop = True
                if trajectory_point_idx == len(interpolated_points) - 1:
                    stop_msg = f"timeout. expected t={desired_point.time_from_start.to_sec()} but t={dt.to_sec()}." \
                               + f" error to waypoint is {error}, tolerance is {goal_tolerance}"
                else:
                    stop_msg = f"timeout. expected t={desired_point.time_from_start.to_sec()} but t={dt.to_sec()}." \
                               + f" error to waypoint is {error}, tolerance is {tolerance}"

            if command_failed or stop:
                # command the current configuration
                self.robot.send_joint_command(trajectory_joint_names, actual_point)
                rospy.loginfo("Preempt requested, aborting.")
                if command_failed_msg:
                    rospy.loginfo(command_failed_msg)
                if stop_msg:
                    rospy.logwarn(stop_msg)
                result.error_code = -10
                result.error_string = stop_msg
                break

            # If we're close enough, advance
            if trajectory_point_idx >= len(interpolated_points) - 1:
                if waypoint_reached(desired_point, actual_point, goal_tolerance):
                    # we're done!
                    result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    break
            else:
                if waypoint_reached(desired_point, actual_point, tolerance):
                    trajectory_point_idx += 1

            if feedback_cb is not None:
                feedback = FollowJointTrajectoryFeedback()
                feedback.header.stamp = rospy.Time.now()
                feedback.joint_names = trajectory_joint_names
                feedback.desired = desired_point
                feedback.actual = actual_point
                feedback_cb(feedback)

        return result

    def follow_trajectory_cb(self, traj_msg: FollowJointTrajectoryGoal):
        def _feedback_cb(feedback):
            self.server.publish_feedback(feedback)

        def _stop_cb(*args, **kwargs):
            stop = self.server.is_preempt_requested()
            stop_msg = "Goal cancelled"
            return stop, stop_msg

        result = self.follow_trajectory_goal(traj_msg, _feedback_cb, _stop_cb)
        # TODO: crappy api here
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

    def get_actual_trajectory_point(self, trajectory_joint_names: List[str]):
        current_joint_positions = self.robot.get_joint_positions(trajectory_joint_names)
        actual_point = JointTrajectoryPoint()
        actual_point.positions = current_joint_positions
        return actual_point
