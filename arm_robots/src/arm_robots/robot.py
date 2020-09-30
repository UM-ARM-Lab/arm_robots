#! /usr/bin/env python
from typing import List, Union, Callable
from typing import Optional

import numpy as np
import pyjacobian_follower
import ros_numpy
from more_itertools import pairwise

import moveit_commander
import rospy
from arc_utilities import ros_helpers
from arc_utilities.ros_helpers import Listener, prepend_namespace
from control_msgs.msg import FollowJointTrajectoryGoal, JointTolerance
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
# These may be different because by making path tolerance larger,
# you might get smoother execution but still keep goal precision
from victor_hardware_interface_msgs.msg import MotionStatus

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


def waypoint_reached(actual: JointTrajectoryPoint, desired: JointTrajectoryPoint, tolerance: List[float]):
    actual = np.array(actual.positions)
    desired = np.array(desired.positions)
    tolerance = np.array(tolerance)
    return np.all((actual - desired) < tolerance)


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


class ARMRobot:
    def __init__(self, execute_by_default: bool = False):
        self.robot_namespace = ''
        self.execute_by_default = execute_by_default
        self.robot_commander = moveit_commander.RobotCommander()
        self.jacobian_follower = pyjacobian_follower.JacobianFollower(translation_step_size=0.002,
                                                                      minimize_rotation=True)
        joint_states_topic = prepend_namespace(self.robot_namespace, 'joint_states')
        self.joint_state_listener = Listener(joint_states_topic, JointState)

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
                                   target_position: Union[Point, List, np.array],
                                   execute=None,
                                   blocking=True,
                                   step_size: float = 0.02,
                                   ):
        if execute is None:
            execute = self.execute_by_default
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)

        # by starting with the current pose, we will be preserving the orientation
        waypoint_pose = move_group.get_current_pose().pose
        if isinstance(target_position, Point):
            waypoint_pose.position = target_position
        else:
            waypoint_pose.position.x = target_position[0]
            waypoint_pose.position.y = target_position[1]
            waypoint_pose.position.z = target_position[2]
        waypoints = [waypoint_pose]
        plan, fraction = move_group.compute_cartesian_path(waypoints=waypoints, eef_step=step_size, jump_threshold=0.0)
        if fraction != 1.0:
            raise RuntimeError(f"Cartesian path is only {fraction * 100}% complete")

        if execute:
            return move_group.execute(plan, wait=blocking)
        else:
            return plan

    def plan_to_pose(self, group_name, ee_link_name, target_pose, execute=None, blocking=True):
        self.check_inputs(group_name, ee_link_name)
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

    def wait_gripper(self, gripper):
        raise NotImplementedError()

    def get_group_end_effector_pose(self, group_name: str, ee_link_name: str):
        self.check_inputs(group_name, ee_link_name)
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        left_end_effector_pose_stamped = move_group.get_current_pose()
        return left_end_effector_pose_stamped.pose

    def make_follow_joint_trajectory_goal(self, joint_names, points):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.header.stamp = rospy.Time.now()

        goal.trajectory.points = points

        goal.goal_tolerance = [make_joint_tolerance(0.05, n) for n in joint_names]
        goal.goal_time_tolerance = rospy.Duration(nsecs=500_000_000)
        return goal

    def plan_to_joint_config(self,
                             group_name: str,
                             joint_config,
                             execute: bool = None,
                             blocking=True,
                             stop_condition: Optional[Callable] = None):
        if execute is None:
            execute = self.execute_by_default
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_joint_value_target(list(joint_config))
        if stop_condition and execute:
            _, plan, _, _ = move_group.plan()
            # TODO: how could we know which controller/trajectory follower client to use? moveit should know this...
            #  or we could just not use moveit execution at all. The major down side is then you can't use the RViz plugin to
            #  move the robot.
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = plan.joint_trajectory
            self.client.send_goal(goal)
            self.client.feedback_cb = lambda feedback: stop_condition(self.client, feedback)
            self.client.wait_for_result()
            return plan
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

    def get_joint_positions(self, joint_names: Optional[List[str]] = None):
        """
        :args joint_names an optional list of names if you want to have a specific order or a subset
        """
        joint_state: JointState = self.joint_state_listener.get()
        current_joint_positions = []
        for name in joint_names:
            if name not in joint_state.name:
                ros_helpers.logfatal(ValueError, f"Joint {name} not found in joint states")
            idx = joint_state.name.index(name)
            pos = joint_state.position[idx]
            current_joint_positions.append(pos)
        return current_joint_positions

    def get_joint_position_from_status_messages(self, left_status: MotionStatus, right_status: MotionStatus, name: str):
        if name == 'victor_left_arm_joint_1':
            pos = left_status.measured_joint_position.joint_1
        elif name == 'victor_left_arm_joint_2':
            pos = left_status.measured_joint_position.joint_2
        elif name == 'victor_left_arm_joint_3':
            pos = left_status.measured_joint_position.joint_3
        elif name == 'victor_left_arm_joint_4':
            pos = left_status.measured_joint_position.joint_4
        elif name == 'victor_left_arm_joint_5':
            pos = left_status.measured_joint_position.joint_5
        elif name == 'victor_left_arm_joint_6':
            pos = left_status.measured_joint_position.joint_6
        elif name == 'victor_left_arm_joint_7':
            pos = left_status.measured_joint_position.joint_7
        elif name == 'victor_right_arm_joint_1':
            pos = right_status.measured_joint_position.joint_1
        elif name == 'victor_right_arm_joint_2':
            pos = right_status.measured_joint_position.joint_2
        elif name == 'victor_right_arm_joint_3':
            pos = right_status.measured_joint_position.joint_3
        elif name == 'victor_right_arm_joint_4':
            pos = right_status.measured_joint_position.joint_4
        elif name == 'victor_right_arm_joint_5':
            pos = right_status.measured_joint_position.joint_5
        elif name == 'victor_right_arm_joint_6':
            pos = right_status.measured_joint_position.joint_6
        elif name == 'victor_right_arm_joint_7':
            pos = right_status.measured_joint_position.joint_7
        else:
            raise NotImplementedError()
        return pos

    def check_inputs(self, group_name: str, ee_link_name: str):
        links = self.robot_commander.get_link_names()
        if ee_link_name not in links:
            rospy.logwarn_throttle(1, f"Link [{ee_link_name}] does not exist. Existing links are:")
            rospy.logwarn_throttle(1, links)

        groups = self.robot_commander.get_group_names()
        if group_name not in groups:
            rospy.logwarn_throttle(1, f"Group [{group_name}] does not exist. Existing groups are:")
            rospy.logwarn_throttle(1, groups)
