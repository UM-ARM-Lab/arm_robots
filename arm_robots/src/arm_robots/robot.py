#! /usr/bin/env python
from typing import List, Union, Tuple, Callable, Optional, Dict

import numpy as np
import pyjacobian_follower

import moveit_commander
import ros_numpy
import rospy
from actionlib import SimpleActionClient
from arc_utilities.conversions import convert_to_pose_msg
from arm_robots.base_robot import DualArmRobot
from arm_robots.robot_utils import make_follow_joint_trajectory_goal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from geometry_msgs.msg import Point, Pose, Quaternion
from rosgraph.names import ns_join
from rospy import logfatal
from rospy.logger_level_service_caller import LoggerLevelServiceCaller
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from victor_hardware_interface_msgs.msg import MotionStatus

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


class MoveitEnabledRobot(DualArmRobot):

    def __init__(self,
                 robot_namespace: str,
                 arms_controller_name: str,
                 execute: bool = True,
                 block: bool = True,
                 force_trigger: float = 9.0):
        super().__init__(robot_namespace)
        self.stored_tool_orientations = None
        self.execute = execute
        self.block = block
        self.force_trigger = force_trigger

        self.arms_controller_name = arms_controller_name

        # Override these in base classes!
        self.left_tool_name = None
        self.right_tool_name = None

        self.arms_client = None
        self.jacobian_follower = None

        self.feedback_callbacks = []

        # Up the logging level for MoveGroupInterface because it's annoying
        # rospy.Timer(rospy.Duration(2), set_move_group_log_level)

    def connect(self):
        # TODO: bad api? raii? this class isn't fully usable by the time it's constructor finishes, that's bad.
        self.arms_client = self.setup_joint_trajectory_controller_client(self.arms_controller_name)

        self.jacobian_follower = pyjacobian_follower.JacobianFollower(robot_namespace=self.robot_namespace,
                                                                      translation_step_size=0.005,
                                                                      minimize_rotation=True)

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

    def plan_to_position(self,
                         group_name: str,
                         ee_link_name: str,
                         target_position):
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=self.robot_namespace)
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
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=self.robot_namespace)
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
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=self.robot_namespace)
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
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=self.robot_namespace)
        move_group.set_end_effector_link(link_name)
        left_end_effector_pose_stamped = move_group.get_current_pose()
        return left_end_effector_pose_stamped.pose

    def plan_to_joint_config(self, group_name: str, joint_config):
        move_group = moveit_commander.MoveGroupCommander(group_name, ns=self.robot_namespace)
        move_group.set_joint_value_target(list(joint_config))
        plan = move_group.plan()[1]
        return self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def follow_joint_trajectory(self,
                                trajectory: JointTrajectory,
                                client: SimpleActionClient,
                                stop_condition: Optional[Callable] = None):
        if len(trajectory.points) == 0:
            rospy.logdebug(f"ignoring empty trajectory")
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            return trajectory, result, client.get_state()

        rospy.logdebug(f"sending trajectory goal with f{len(trajectory.points)} points")
        result: Optional[FollowJointTrajectoryResult] = None
        if self.execute:
            goal = make_follow_joint_trajectory_goal(trajectory)

            def _feedback_cb(feedback: FollowJointTrajectoryFeedback):
                for feedback_callback in self.feedback_callbacks:
                    feedback_callback(client, goal, feedback)
                if stop_condition is not None:
                    stop = stop_condition(feedback)
                    if stop:
                        client.cancel_all_goals()

            client.send_goal(goal, feedback_cb=_feedback_cb)
            if self.block:
                client.wait_for_result()
                result = client.get_result()
        return trajectory, result, client.get_state()

    def follow_joint_config(self, joint_names: List[str], joint_positions, client: SimpleActionClient):
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(1.0)
        point.positions = joint_positions
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

    def store_tool_orientations(self, preferred_tool_orientations: Optional[Dict]):
        """ dict values can be Pose, Quaternion, or just a numpy array/list """
        self.stored_tool_orientations = {}
        for k, v in preferred_tool_orientations.items():
            if isinstance(v, Pose):
                q = ros_numpy.numpify(v.orientation)
                self.stored_tool_orientations[k] = q
            elif isinstance(v, Quaternion):
                q = ros_numpy.numpify(v)
                self.stored_tool_orientations[k] = q
            else:
                self.stored_tool_orientations[k] = v

    def store_current_tool_orientations(self, tool_names: Optional[List]):
        self.stored_tool_orientations = {n: self.get_orientation(n) for n in tool_names}

    def follow_jacobian_to_position(self,
                                    group_name: str,
                                    tool_names: List[str],
                                    points: List[List],
                                    preferred_tool_orientations: Optional[List] = STORED_ORIENTATION,
                                    vel_scaling=0.1,
                                    stop_condition: Optional[Callable] = None,
                                    ):
        """ If preferred_tool_orientations is None, we use the stored ones as a fallback """
        if preferred_tool_orientations == STORED_ORIENTATION:
            preferred_tool_orientations = []
            if self.stored_tool_orientations is None:
                logfatal(store_error_msg)
            for k in tool_names:
                if k not in self.stored_tool_orientations:
                    rospy.logerr(f"tool {k} has no stored orientation. aborting.")
                    return
                preferred_tool_orientations.append(self.stored_tool_orientations[k])
        robot_trajectory_msg: moveit_commander.RobotTrajectory = self.jacobian_follower.plan(
            group_name=group_name,
            tool_names=tool_names,
            preferred_tool_orientations=preferred_tool_orientations,
            grippers=points,
            max_velocity_scaling_factor=vel_scaling,
            max_acceleration_scaling_factor=0.1,
        )
        return self.follow_arms_joint_trajectory(robot_trajectory_msg.joint_trajectory, stop_condition=stop_condition)

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

    def get_both_arm_joints(self):
        return self.get_left_arm_joints() + self.get_right_arm_joints()

    def get_n_joints(self):
        return len(self.robot_commander.get_joint_names())

    def get_right_arm_joints(self):
        raise NotImplementedError()

    def get_left_arm_joints(self):
        raise NotImplementedError()

    def get_right_gripper_joints(self):
        raise NotImplementedError()

    def get_left_gripper_joints(self):
        raise NotImplementedError()

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        raise NotImplementedError()

    def get_gripper_positions(self):
        left_gripper = self.robot_commander.get_link(self.left_tool_name)
        right_gripper = self.robot_commander.get_link(self.right_tool_name)
        return left_gripper.pose().pose.position, right_gripper.pose().pose.position
