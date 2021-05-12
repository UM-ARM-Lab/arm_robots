#! /usr/bin/env python
from typing import List, Union, Tuple, Callable, Optional, Dict

import numpy as np
import pyjacobian_follower

import moveit_commander
import ros_numpy
import rospy
from actionlib import SimpleActionClient
from arc_utilities.conversions import convert_to_pose_msg
from arm_robots.base_robot import BaseRobot
from arm_robots.robot_utils import make_follow_joint_trajectory_goal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult, \
    FollowJointTrajectoryGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from rosgraph.names import ns_join
from rospy import logfatal
from rospy.logger_level_service_caller import LoggerLevelServiceCaller
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pdb
from arm_robots.config.med_config import KUKA_MED_MAX_JOINT_VELOCITIES, KUKA_MED_MAX_JOINT_ACCEL

STORED_ORIENTATION = None

store_error_msg = ("No stored tool orientations! "
                   "You have to call store_tool_orientations or store_current_tool_orientations first")


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


class FMoveitEnabledRobot(BaseRobot):
    max_velocity_scale_factor = 0.1

    def __init__(self,
                 robot_namespace: str,
                 arms_controller_name: str,
                 execute: bool = True,
                 block: bool = True,
                 force_trigger: float = 9.0,
                 manual_execute: bool = True):
        super().__init__(robot_namespace)
        self.stored_tool_orientations = None
        self.execute = execute
        self.block = block
        self.manual_execute = manual_execute
        self.force_trigger = force_trigger

        self.arms_controller_name = arms_controller_name

        # Override these in base classes!
        self.tool_name = None

        self.arms_client = None
        self.jacobian_follower = None

        self.feedback_callbacks = []

        # Up the logging level for MoveGroupInterface because it's annoying
        # rospy.Timer(rospy.Duration(2), set_move_group_log_level)

    def connect(self):
        # TODO: bad api? raii? this class isn't fully usable by the time it's constructor finishes, that's bad.
        self.arms_client = self.setup_joint_trajectory_controller_client(self.arms_controller_name)

        # Removing for now - add back in later.
        # self.jacobian_follower = pyjacobian_follower.JacobianFollower(robot_namespace=self.robot_namespace,
        #                                                               translation_step_size=0.005,
        #                                                               minimize_rotation=True)

    def setup_joint_trajectory_controller_client(self, controller_name):
        action_name = ns_join(self.robot_namespace, ns_join(controller_name, "follow_joint_trajectory"))
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

    def get_move_group_commander(self, group_name) -> moveit_commander.MoveGroupCommander:
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=self.robot_namespace)
        # TODO Make this a settable param or at least make the hardcoded param more obvious
        # The purpose of this safety factor is to make sure we never send victor a velocity
        # faster than the kuka controller's max velocity, otherwise the kuka controllers will error out.
        safety_factor = 0.9
        move_group.set_max_velocity_scaling_factor(self.max_velocity_scale_factor * safety_factor)
        return move_group

    def plan_to_position(self,
                         group_name: str,
                         ee_link_name: str,
                         target_position):
        move_group = self.get_move_group_commander(group_name)
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
        move_group = self.get_move_group_commander(group_name)
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
        move_group = self.get_move_group_commander(group_name)
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
        move_group = self.get_move_group_commander(group_name)
        move_group.set_end_effector_link(link_name)
        left_end_effector_pose_stamped = move_group.get_current_pose()
        return left_end_effector_pose_stamped.pose

    def plan_to_joint_config(self, group_name: str, joint_config):
        move_group = self.get_move_group_commander(group_name)
        move_group.set_joint_value_target(list(joint_config))
        plan = move_group.plan()[1]
        return self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def make_follow_joint_trajectory_goal(self, trajectory) -> FollowJointTrajectoryGoal:
        return make_follow_joint_trajectory_goal(trajectory)

    def check_trajectory_limits(self, trajectory: FollowJointTrajectoryGoal):
        for pt in trajectory.trajectory.points:
            for j_idx in range(len(pt.velocities)):
                if not abs(pt.velocities[j_idx]) < KUKA_MED_MAX_JOINT_VELOCITIES[j_idx] * self.max_velocity_scale_factor:
                    print("Joint %d velocity %f is above limit %f" % (j_idx, pt.velocities[j_idx],
                                                                      KUKA_MED_MAX_JOINT_VELOCITIES[j_idx]))
                    assert(False)
            for j_idx in range(len(pt.velocities)):
                if not abs(pt.accelerations[j_idx]) < KUKA_MED_MAX_JOINT_ACCEL[j_idx]:
                    print("Joint %d acceleration %f is above limit %f" % (j_idx, pt.accelerations[j_idx],
                                                                          KUKA_MED_MAX_JOINT_ACCEL[j_idx]))
                    assert(False)

    def follow_joint_trajectory(self,
                                trajectory: JointTrajectory,
                                client: SimpleActionClient,
                                stop_condition: Optional[Callable] = None):
        if self.manual_execute:
            exec_path = input("Execute path? [y/n]")
            if exec_path != "y":
                rospy.logdebug(f"cancelling trajectory")
                result = FollowJointTrajectoryResult()
                result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
                return trajectory, result, client.get_state()

        if len(trajectory.points) == 0:
            rospy.logdebug(f"ignoring empty trajectory")
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            return trajectory, result, client.get_state()

        rospy.logdebug(f"sending trajectory goal with f{len(trajectory.points)} points")
        result: Optional[FollowJointTrajectoryResult] = None
        if self.execute:
            goal = self.make_follow_joint_trajectory_goal(trajectory)
            self.check_trajectory_limits(goal)

            def _feedback_cb(feedback: FollowJointTrajectoryFeedback):
                for feedback_callback in self.feedback_callbacks:
                    feedback_callback(client, goal, feedback)
                if stop_condition is not None and stop_condition(feedback):
                    client.cancel_all_goals()

            time_start = rospy.get_time()
            client.send_goal(goal, feedback_cb=_feedback_cb)
            if self.block:
                client.wait_for_result()
                result = client.get_result()
            time_end = rospy.get_time()
            print("Time expected: " + str(goal.trajectory.points[-1].time_from_start / 1e9))
            print("Time real: " + str(time_end - time_start))
        return trajectory, result, client.get_state()

    def follow_joint_config(self, joint_names: List[str], joint_positions, client: SimpleActionClient):
        trajectory = JointTrajectory(joint_names=joint_names)
        point = JointTrajectoryPoint(time_from_start=rospy.Duration(1.0), positions=joint_positions)
        trajectory.points.append(point)
        return self.follow_joint_trajectory(trajectory, client)

    def follow_arms_joint_trajectory(self, trajectory: JointTrajectory, stop_condition: Optional[Callable] = None):
        return self.follow_joint_trajectory(trajectory, self.arms_client, stop_condition=stop_condition)

    def distance(self,
                 group_name: str,
                 ee_link_name: str,
                 target_position):
        current_pose = self.get_link_pose(group_name, ee_link_name)
        error = np.linalg.norm(ros_numpy.numpify(current_pose.position) - target_position)
        return error

    def get_orientation(self, name: str):
        link = self.robot_commander.get_link(name)
        return ros_numpy.numpify(link.pose().pose.orientation)

    def get_arm_joints(self):
        raise NotImplementedError()

    def get_n_joints(self):
        return len(self.robot_commander.get_joint_names())
