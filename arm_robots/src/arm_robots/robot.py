#! /usr/bin/env python
from typing import List
from typing import Optional

import numpy as np
import pyjacobian_follower
import ros_numpy

import actionlib
import moveit_commander
import rospy
from control_msgs.msg import FollowJointTrajectoryFeedback
from geometry_msgs.msg import Point


def make_joint_tolerance(pos, name):
    j = JointTolerance()
    j.position = pos
    j.name = name
    return j


left_gripper_joint_names = [
    "victor_left_gripper_fingerA_joint_2",
    "victor_left_gripper_fingerA_joint_3",
    "victor_left_gripper_fingerA_joint_4",
    "victor_left_gripper_fingerB_joint_2",
    "victor_left_gripper_fingerB_joint_3",
    "victor_left_gripper_fingerB_joint_4",
    "victor_left_gripper_fingerB_knuckle",
    "victor_left_gripper_fingerC_joint_2",
    "victor_left_gripper_fingerC_joint_3",
    "victor_left_gripper_fingerC_joint_4",
    "victor_left_gripper_fingerC_knuckle",
]

right_gripper_joint_names = [
    "victor_right_gripper_fingerA_joint_2",
    "victor_right_gripper_fingerA_joint_3",
    "victor_right_gripper_fingerA_joint_4",
    "victor_right_gripper_fingerB_joint_2",
    "victor_right_gripper_fingerB_joint_3",
    "victor_right_gripper_fingerB_joint_4",
    "victor_right_gripper_fingerB_knuckle",
    "victor_right_gripper_fingerC_joint_2",
    "victor_right_gripper_fingerC_joint_3",
    "victor_right_gripper_fingerC_joint_4",
    "victor_right_gripper_fingerC_knuckle",
]

from arc_utilities.algorithms import consecutive_pairs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint


def waypoint_reached(actual: JointTrajectoryPoint, desired: JointTrajectoryPoint, tolerance: List[float]):
    actual = np.array(actual.positions)
    desired = np.array(desired.positions)
    tolerance = np.array(tolerance)
    return np.all((actual - desired) < tolerance)


def interpolate_joint_trajectory_points(points: List[JointTrajectoryPoint], max_step_size: float):
    # TODO: support arbitrarily weighted norms, probably on a fixed-per-robot basis. So this could become a method
    # of the victor/val class, which has that weight as a class field
    if len(points) == 1:
        raise ValueError("cannot interpolate a single point")
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
        for p1, p2 in consecutive_pairs(points):
            interpolated_points.extend(interpolate_joint_trajectory_points([p1, p2], max_step_size))
        return interpolated_points


class ARMRobot:
    def __init__(self, execute_by_default: bool = False, wait_for_action_servers=True):
        self.execute_by_default = execute_by_default
        self.robot = moveit_commander.RobotCommander()
        self.jacobian_follower = pyjacobian_follower.JacobianFollower(translation_step_size=0.002,
                                                                      minimize_rotation=True)
        self.right_arm_client = actionlib.SimpleActionClient(
            '/victor/right_arm_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        self.right_hand_client = actionlib.SimpleActionClient('/victor/right_hand_controller/follow_joint_trajectory',
                                                              FollowJointTrajectoryAction)
        if wait_for_action_servers:
            self.right_hand_client.wait_for_server()

        self.right_hand_client.feedback_cb = self.right_hand_follow_joint_trajectory_feedback

        rospy.loginfo("MotionEnabledRobot ready")

    def right_hand_follow_joint_trajectory_feedback(feedback: FollowJointTrajectoryFeedback):
        print(feedback.error)

    def traj_from_path_msg(self, path_msg):
        raise NotImplementedError()

    def execute_trajectory(self, trajectory, blocking=True):
        raise NotImplementedError()

    def move_to_home(self, blocking=True):
        raise NotImplementedError()

    def plan_to_configuration(self, target_config, execute=False, steplength=0.01, blocking=True, **kwargs):
        if execute is None:
            execute = self.execute_by_default
        raise NotImplementedError()

    def plan_to_configuration_both_arms(self, right_config, left_config, execute=False, steplength=0.01, blocking=True):
        if execute is None:
            execute = self.execute_by_default
        raise NotImplementedError()

    def plan_to_configuration_whole_body(self, right_config, left_config, other_config=None, execute=False,
                                         steplength=0.01, blocking=True):
        if execute is None:
            execute = self.execute_by_default
        raise NotImplementedError()

    def plan_to_relative_pose(self, relative_pose, execute=False, **kwargs):
        if execute is None:
            execute = self.execute_by_default
        raise NotImplementedError()

    def plan_to_position(self, group_name: str, ee_link_name: str, target_position, execute=None, blocking=True):
        if execute is None:
            execute = self.execute_by_default
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        move_group.set_position_target(list(target_position))
        if execute:
            return move_group.go(wait=blocking)
        else:
            return move_group.plan()

    def plan_to_position_cartesian(self,
                                   group_name: str,
                                   ee_link_name: str,
                                   target_position,
                                   execute=None,
                                   blocking=True):
        if execute is None:
            execute = self.execute_by_default
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)

        # by starting with the current pose, we will be preserving the orientation
        waypoint_pose = move_group.get_current_pose().pose
        waypoint_pose.position.x = 0.8
        waypoint_pose.position.y = -0.2
        waypoint_pose.position.z = 0.65
        waypoints = [waypoint_pose]
        plan, fraction = move_group.compute_cartesian_path(waypoints=waypoints, eef_step=0.002, jump_threshold=1.0)
        if fraction != 1.0:
            raise RuntimeError("Failed to find a cartesian path that is complete!")

        if execute:
            return move_group.execute(plan, wait=blocking)
        else:
            return plan

    def plan_to_pose(self, group_name, ee_link_name, target_pose, execute=None, blocking=True):
        if execute is None:
            execute = self.execute_by_default
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        move_group.set_pose_target(target_pose)
        move_group.set_goal_position_tolerance(0.002)
        move_group.set_goal_orientation_tolerance(0.02)
        if execute:
            return move_group.go(wait=blocking)
        else:
            return move_group.plan()

    def plan_to_pose_both_arms(self, right_pose, left_pose, execute=None, blocking=True, **kwargs):
        if execute is None:
            execute = self.execute_by_default
        raise NotImplementedError()

    def wait_gripper(self, gripper):
        raise NotImplementedError()

    def get_group_end_effector_pose(self, group_name: str, ee_link_name: str):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        left_end_effector_pose_stamped = move_group.get_current_pose()
        return left_end_effector_pose_stamped.pose

    def set_right_gripper(self, positions, blocking=True):
        point = JointTrajectoryPoint()
        point.time_from_start.secs = 1
        point.positions = positions
        points = [point]
        goal = self.make_follow_joint_trajectory_goal(right_gripper_joint_names, points)

        # TODO: debug code -5, goal_tolerance not met
        self.right_hand_client.send_goal(goal)
        if blocking:
            self.right_hand_client.wait_for_result()
            result = self.right_hand_client.get_result()
            print(result.error_code, result.error_string)

    def make_follow_joint_trajectory_goal(self, joint_names, points):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.header.stamp = rospy.Time.now()

        goal.trajectory.points = points

        goal.goal_tolerance = [make_joint_tolerance(0.05, n) for n in joint_names]
        goal.goal_time_tolerance = rospy.Duration(nsecs=500_000_000)
        return goal

    def plan_to_joint_config(self, group_name: str, joint_config, execute: bool = None, blocking=True):
        if execute is None:
            execute = self.execute_by_default
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_joint_value_target(list(joint_config))
        if execute:
            return move_group.go(wait=blocking)
        else:
            return move_group.plan()

    def open_right_gripper(self):
        pass

    def distance(self,
                 group_name: str,
                 ee_link_name: str,
                 target_position):
        current_pose = self.get_group_end_effector_pose(group_name, ee_link_name)
        error = np.linalg.norm(ros_numpy.numpify(current_pose.position) - target_position)
        return error

    def follow_jacobian_to_position(self,
                                    group_name: str,
                                    ee_link_name: str,
                                    target_position,
                                    minimize_rotation: bool = True,
                                    blocking: Optional[bool] = True
                                    ):
        if isinstance(target_position, Point):
            waypoint = ros_numpy.numpify(target_position)
        else:
            waypoint = target_position

        gripper_points = [waypoint]

        group_name = group_name
        speed = 0.2
        tool_names = [ee_link_name]
        grippers = [gripper_points]
        robot_trajectory_msg = self.jacobian_follower.plan(group_name, tool_names, grippers, speed)
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # TODO: optional execution
        move_group.execute(robot_trajectory_msg, blocking)

    def close_left_gripper(self, blocking=True):
        raise NotImplementedError()

    def open_gripper(self, gripper_name, blocking=True):
        raise NotImplementedError()

    def set_gripper(self, gripper_name, pos, blocking=True):
        raise NotImplementedError("set_gripper is a abstract method. It should never be called.")

    def move_both_hands_straight(self,
                                 right_moving_direction=None,
                                 left_moving_direction=None,
                                 right_moving_distance=0.0,
                                 left_moving_distance=0.0,
                                 right_step_size=0.005,
                                 left_step_size=0.005,
                                 execute=None,
                                 blocking=True):
        if execute is None:
            execute = self.execute_by_default
        raise NotImplementedError()

    def rotate_both_hands_straight(self,
                                   right_moving_axis=None,
                                   left_moving_axis=None,
                                   right_moving_radians=0.0,
                                   left_moving_radians=0.0,
                                   right_step_size=0.005,
                                   left_step_size=0.005,
                                   execute=None,
                                   blocking=True):
        if execute is None:
            execute = self.execute_by_default
        raise NotImplementedError()

    def object_grasped(self, gripper):
        raise NotImplementedError()
