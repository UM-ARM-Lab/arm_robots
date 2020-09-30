#! /usr/bin/env python
from typing import List

import numpy as np

import actionlib
import rospy
from arm_robots.hdt_michigan import Val
from arm_robots.robot import interpolate_joint_trajectory_points, waypoint_reached, get_ordered_tolerance_list, ARMRobot
from arm_robots.victor import Victor
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, \
    FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint


# # TODO: make this work with val too


class TrajectoryFollower:
    def __init__(self, robot: ARMRobot):
        self.robot = robot
        self._action_name = self.robot.robot_namespace + "/both_arms_trajectory_controller/follow_joint_trajectory"
        self.action_server = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction,
                                                          execute_cb=self.execute_cb,
                                                          auto_start=False)
        self.action_server.start()

    def execute_cb(self, traj_msg: FollowJointTrajectoryGoal):
        # Interpolate the trajectory to a fine resolution
        # if you set max_step_size to be large and position tolerance to be small, then things will be jerky
        if len(traj_msg.trajectory.points) == 0:
            rospy.loginfo("Ignoring empty trajectory")
            return

        # construct a list of the tolerances in order of the joint names
        trajectory_joint_names = traj_msg.trajectory.joint_names
        tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.path_tolerance)
        goal_tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.goal_tolerance, is_goal=True)

        interpolated_points = interpolate_joint_trajectory_points(traj_msg.trajectory.points, max_step_size=0.1)

        trajectory_point_idx = 0
        while True:
            # tiny sleep lets the listeners process messages better, results in smoother following
            rospy.sleep(1e-6)

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Preempt requested, aborting.")
                break

            now = rospy.Time.now()
            desired_point = interpolated_points[trajectory_point_idx]

            # still too specific, only works for joint trajectories
            self.robot.send_setpoint_to_controller(self.action_server, now, trajectory_joint_names, desired_point)

            # get feedback
            actual_point = self.get_actual_trajectory_point(trajectory_joint_names)

            # If we're close enough, advance
            if trajectory_point_idx == len(interpolated_points) - 1:
                if waypoint_reached(desired_point, actual_point, goal_tolerance):
                    # we're done!
                    break
            else:
                if waypoint_reached(desired_point, actual_point, tolerance):
                    trajectory_point_idx += 1

            feedback = FollowJointTrajectoryFeedback()
            feedback.header.stamp = now
            feedback.joint_names = trajectory_joint_names
            feedback.desired = desired_point
            feedback.actual = actual_point
            self.action_server.publish_feedback(feedback)

        success_result = FollowJointTrajectoryResult()
        success_result.error_code = actionlib.GoalStatus.SUCCEEDED
        self.action_server.set_succeeded(success_result)

    def get_actual_trajectory_point(self, trajectory_joint_names: List[str]):
        current_joint_positions = self.robot.get_joint_positions(trajectory_joint_names)
        actual_point = JointTrajectoryPoint()
        actual_point.positions = current_joint_positions
        return actual_point


def main():
    np.set_printoptions(linewidth=200, suppress=True, precision=4)
    rospy.init_node('trajectory_follower')

    robot_name_rosparam_name = "robot_name"
    if robot_name := rospy.get_param(robot_name_rosparam_name, None):
        if robot_name == "victor":
            robot = Victor(execute_by_default=True)
        elif robot_name == "victor":
            robot = Val(execute_by_default=True)
        else:
            raise NotImplementedError(f"Invalid ros param {robot_name_rosparam_name} {robot_name}")
    else:
        rospy.loginfo(f"rosparam {robot_name_rosparam_name} not set, Defaulting to Victor")
        robot = Victor(execute_by_default=True)

    fwd = TrajectoryFollower(robot)

    rospy.spin()


if __name__ == "__main__":
    main()
