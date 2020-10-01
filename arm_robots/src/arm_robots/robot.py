#! /usr/bin/env python
import pathlib
from typing import List, Union

import numpy as np
import pyjacobian_follower

import actionlib
import moveit_commander
import ros_numpy
import rospy
from arc_utilities.conversions import convert_to_pose_msg
from arm_robots.base_robot import BaseRobot
from arm_robots.robot_utils import make_follow_joint_trajectory_goal
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory
from victor_hardware_interface_msgs.msg import MotionStatus


class MoveitEnabledRobot:
    def __init__(self, base_robot: BaseRobot, execute: bool = True, block: bool = True, force_trigger: float = 9.0):
        self.execute = execute
        self.block = block
        self.force_trigger = 9.0

        controller_name = self.get_joint_trajectory_controller_name()

        follow_joint_trajectory_action_name = pathlib.Path(
            base_robot.robot_namespace) / controller_name / "follow_joint_trajectory"
        # self.joint_trajectory_follower_client = None
        self.joint_trajectory_follower_client = actionlib.SimpleActionClient(follow_joint_trajectory_action_name.as_posix(),
                                                                             FollowJointTrajectoryAction)
        rospy.loginfo(f"Waiting for joint trajectory follower server {follow_joint_trajectory_action_name}...")
        self.joint_trajectory_follower_client.wait_for_server()
        rospy.loginfo(f"Connected.")

        self.robot_commander = moveit_commander.RobotCommander()
        self.jacobian_follower = pyjacobian_follower.JacobianFollower(translation_step_size=0.002,
                                                                      minimize_rotation=True)

    def set_execute(self, execute: bool):
        self.execute = execute

    def set_block(self, block: bool):
        self.block = block

    def plan_to_position(self,
                         group_name: str,
                         ee_link_name: str,
                         target_position):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        move_group.set_position_target(list(target_position))
        plan = move_group.plan()[1]
        return self.follow_joint_trajectory(plan.joint_trajectory)

    def plan_to_position_cartesian(self,
                                   group_name: str,
                                   ee_link_name: str,
                                   target_position: Union[Point, List, np.array],
                                   step_size: float = 0.02,
                                   ):
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
        return self.follow_joint_trajectory(plan.joint_trajectory)

    def plan_to_pose(self, group_name, ee_link_name, target_pose):
        self.check_inputs(group_name, ee_link_name)
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        target_pose_stamped = convert_to_pose_msg(target_pose)
        move_group.set_pose_target(target_pose_stamped)
        move_group.set_goal_position_tolerance(0.002)
        move_group.set_goal_orientation_tolerance(0.02)

        plan = move_group.plan()[1]
        return self.follow_joint_trajectory(plan.joint_trajectory)

    def get_group_end_effector_pose(self, group_name: str, ee_link_name: str):
        self.check_inputs(group_name, ee_link_name)
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        left_end_effector_pose_stamped = move_group.get_current_pose()
        return left_end_effector_pose_stamped.pose

    def plan_to_joint_config(self, group_name: str, joint_config):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_joint_value_target(list(joint_config))
        plan = move_group.plan()[1]
        return self.follow_joint_trajectory(plan.joint_trajectory)

    def follow_joint_trajectory(self, trajectory: JointTrajectory):
        result = None
        if self.execute:
            goal = make_follow_joint_trajectory_goal(trajectory)
            self.joint_trajectory_follower_client.send_goal(goal)
            if self.block:
                self.joint_trajectory_follower_client.wait_for_result()
                result = self.joint_trajectory_follower_client.get_result()
        return trajectory, result

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
        robot_trajectory_msg: moveit_commander.RobotTrajectory = self.jacobian_follower.plan(group_name,
                                                                                             tool_names,
                                                                                             grippers,
                                                                                             speed)
        return self.follow_joint_trajectory(robot_trajectory_msg.joint_trajectory)

    def close_left_gripper(self):
        raise NotImplementedError()

    def open_gripper(self, gripper_name):
        raise NotImplementedError()

    def set_gripper(self, gripper_name, pos):
        raise NotImplementedError("set_gripper is a abstract method. It should never be called.")

    def object_grasped(self, gripper):
        raise NotImplementedError()

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

    def get_joint_trajectory_controller_name(self):
        """ This should match the *.yaml file, and you can also run rqt_controller_manager to check the names """
        raise NotImplementedError
        pass
