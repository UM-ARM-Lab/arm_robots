#! /usr/bin/env python
from typing import List, Union

import numpy as np
import pyjacobian_follower

import moveit_commander
import ros_numpy
import rospy
from actionlib import SimpleActionClient
from arc_utilities.conversions import convert_to_pose_msg
from arm_robots.base_robot import BaseRobot
from arm_robots.robot_utils import make_follow_joint_trajectory_goal
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from victor_hardware_interface_msgs.msg import MotionStatus


class MoveitEnabledRobot:

    def __init__(self,
                 base_robot: BaseRobot,
                 arms_controller_name: str,
                 left_gripper_controller_name: str,
                 right_gripper_controller_name: str,
                 execute: bool = True,
                 block: bool = True,
                 force_trigger: float = 9.0):
        self.base_robot = base_robot
        self.execute = execute
        self.block = block

        # TODO: implement stop-on-force
        self.force_trigger = force_trigger

        self.arms_client = self.setup_joint_trajectory_controller_client(arms_controller_name)
        self.left_gripper_client = self.setup_joint_trajectory_controller_client(left_gripper_controller_name)
        self.right_gripper_client = self.setup_joint_trajectory_controller_client(right_gripper_controller_name)

        self.robot_commander = moveit_commander.RobotCommander()
        self.jacobian_follower = pyjacobian_follower.JacobianFollower(translation_step_size=0.01,
                                                                      minimize_rotation=True)

    def setup_joint_trajectory_controller_client(self, controller_name):
        action_name = controller_name + "/follow_joint_trajectory"
        client = SimpleActionClient(action_name, FollowJointTrajectoryAction)
        resolved_action_name = rospy.resolve_name(action_name)
        wait_msg = f"Waiting for joint trajectory follower server {resolved_action_name}..."
        rospy.loginfo(wait_msg)
        client.wait_for_server()
        rospy.loginfo(f"Connected.")
        return client

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
        return self.follow_arms_joint_trajectory(plan.joint_trajectory)

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
        return self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def plan_to_pose(self, group_name, ee_link_name, target_pose, frame_id: str = 'robot_root'):
        self.check_inputs(group_name, ee_link_name)
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        target_pose_stamped = convert_to_pose_msg(target_pose)
        target_pose_stamped.header.frame_id = frame_id
        move_group.set_pose_target(target_pose_stamped)
        move_group.set_goal_position_tolerance(0.002)
        move_group.set_goal_orientation_tolerance(0.02)

        plan = move_group.plan()[1]
        result = self.follow_arms_joint_trajectory(plan.joint_trajectory)
        return result

    def get_link_pose(self, group_name: str, link_name: str):
        self.check_inputs(group_name, link_name)
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link(link_name)
        left_end_effector_pose_stamped = move_group.get_current_pose()
        return left_end_effector_pose_stamped.pose

    def plan_to_joint_config(self, group_name: str, joint_config):
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_joint_value_target(list(joint_config))
        plan = move_group.plan()[1]
        return self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def follow_joint_trajectory(self, trajectory: JointTrajectory, client: SimpleActionClient):
        rospy.logdebug(f"sending trajectory goal with f{len(trajectory.points)} points")
        result = None
        if self.execute:
            goal = make_follow_joint_trajectory_goal(trajectory)
            client.send_goal(goal)
            if self.block:
                client.wait_for_result()
                result = client.get_result()
        return trajectory, result

    def follow_joint_config(self, joint_names: List[str], joint_positions, client: SimpleActionClient):
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.1)
        point.positions = joint_positions
        trajectory.points.append(point)
        return self.follow_joint_trajectory(trajectory, client)

    def follow_left_gripper_joint_trajectory(self, trajectory: JointTrajectory):
        return self.follow_joint_trajectory(trajectory, self.left_gripper_client)

    def follow_right_gripper_joint_trajectory(self, trajectory: JointTrajectory):
        return self.follow_joint_trajectory(trajectory, self.right_gripper_client)

    def follow_arms_joint_trajectory(self, trajectory: JointTrajectory):
        return self.follow_joint_trajectory(trajectory, self.arms_client)

    def distance(self,
                 group_name: str,
                 ee_link_name: str,
                 target_position):
        current_pose = self.get_link_pose(group_name, ee_link_name)
        error = np.linalg.norm(ros_numpy.numpify(current_pose.position) - target_position)
        return error

    def follow_jacobian_to_position(self,
                                    group_name: str,
                                    tool_names: List[str],
                                    points: List[List],
                                    ):
        robot_trajectory_msg: moveit_commander.RobotTrajectory = self.jacobian_follower.plan(group_name,
                                                                                             tool_names,
                                                                                             points,
                                                                                             max_velocity_scaling_factor=0.1,
                                                                                             max_acceleration_scaling_factor=0.1,
                                                                                             )
        return self.follow_arms_joint_trajectory(robot_trajectory_msg.joint_trajectory)

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

    def set_left_gripper(self, joint_names: List[str], joint_positions):
        return self.follow_joint_config(joint_names, joint_positions, self.left_gripper_client)

    def set_right_gripper(self, joint_names: List[str], joint_positions):
        return self.follow_joint_config(joint_names, joint_positions, self.right_gripper_client)

    def get_right_gripper_links(self):
        return self.robot_commander.get_link_names("right_gripper")

    def get_left_gripper_links(self):
        return self.robot_commander.get_link_names("left_gripper")

    def get_right_gripper_joints(self):
        raise NotImplementedError()

    def get_left_gripper_joints(self):
        raise NotImplementedError()

    def get_gripper_closed_positions(self):
        raise NotImplementedError()

    def get_gripper_open_positions(self):
        raise NotImplementedError()

    def open_left_gripper(self):
        return self.set_left_gripper(self.get_left_gripper_joints(), self.get_gripper_open_positions())

    def open_right_gripper(self):
        return self.set_right_gripper(self.get_right_gripper_joints(), self.get_gripper_open_positions())

    def close_left_gripper(self):
        return self.set_left_gripper(self.get_left_gripper_joints(), self.get_gripper_closed_positions())

    def close_right_gripper(self):
        return self.set_right_gripper(self.get_right_gripper_joints(), self.get_gripper_closed_positions())
