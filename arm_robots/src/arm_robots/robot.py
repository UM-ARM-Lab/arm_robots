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
from arm_robots.robot_utils import make_follow_joint_trajectory_goal, PlanningResult, PlanningAndExecutionResult, \
    ExecutionResult, is_empty_trajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult, \
    FollowJointTrajectoryGoal
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import RobotTrajectory
from rosgraph.names import ns_join
from rospy import logfatal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

STORED_ORIENTATION = None

store_error_msg = ("No stored tool orientations! "
                   "You have to call store_tool_orientations or store_current_tool_orientations first")


class MoveitEnabledRobot(DualArmRobot):

    def __init__(self,
                 robot_namespace: str,
                 arms_controller_name: str,
                 execute: bool = True,
                 block: bool = True,
                 raise_on_failure: bool = False,
                 force_trigger: float = 9.0):
        super().__init__(robot_namespace)
        self.max_velocity_scale_factor = 0.1
        self.stored_tool_orientations = None
        self.raise_on_failure = raise_on_failure
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

        planning_result = PlanningResult(move_group.plan())
        if self.raise_on_failure and not planning_result.success:
            raise RuntimeError(f"Plan to position failed {planning_result.planning_error_code}")

        execution_result = self.follow_arms_joint_trajectory(planning_result.plan.joint_trajectory)
        return PlanningAndExecutionResult(planning_result, execution_result)

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
        planning_result = PlanningResult(success=(fraction == 1.0), plan=plan)
        if self.raise_on_failure and not planning_result.success:
            raise RuntimeError(f"Cartesian path is only {fraction * 100}% complete")

        execution_result = self.follow_arms_joint_trajectory(plan.joint_trajectory)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def plan_to_pose(self, group_name, ee_link_name, target_pose, frame_id: str = 'robot_root'):
        self.check_inputs(group_name, ee_link_name)
        move_group = self.get_move_group_commander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        target_pose_stamped = convert_to_pose_msg(target_pose)
        target_pose_stamped.header.frame_id = frame_id
        move_group.set_pose_target(target_pose_stamped)
        move_group.set_goal_position_tolerance(0.002)
        move_group.set_goal_orientation_tolerance(0.02)

        planning_result = PlanningResult(move_group.plan())
        if self.raise_on_failure and not planning_result.success:
            raise RuntimeError(f"Plan to pose failed {planning_result.planning_error_code}")

        execution_result = self.follow_arms_joint_trajectory(planning_result.plan.joint_trajectory)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def get_link_pose(self, group_name: str, link_name: str):
        self.check_inputs(group_name, link_name)
        move_group = self.get_move_group_commander(group_name)
        move_group.set_end_effector_link(link_name)
        left_end_effector_pose_stamped = move_group.get_current_pose()
        return left_end_effector_pose_stamped.pose

    def plan_to_joint_config(self, group_name: str, joint_config: Union[List, Dict, str]):
        """
        Args:
            group_name: group name, defined in the SRDF
            joint_config: Either a dictionary of joint_name: joint value,
             or a string for the group_state defined in the SRDF. You can technically use a list
             here instead of a dict but you need to provide all joints in the right order,
             which is hard to get right and gives bad error messages

        Returns:
            The result message of following the trajectory
        """
        move_group = self.get_move_group_commander(group_name)
        if isinstance(joint_config, str):
            joint_config_name = joint_config
            joint_config = move_group.get_named_target_values(joint_config_name)
            if len(joint_config) == 0:
                raise ValueError(f"No group state named {joint_config_name}")
            move_group.set_joint_value_target(joint_config)
        else:
            move_group.set_joint_value_target(joint_config)
        planning_result = PlanningResult(move_group.plan())
        if self.raise_on_failure and not planning_result.success:
            raise RuntimeError(f"Plan to position failed {planning_result.planning_error_code}")
        execution_result = self.follow_arms_joint_trajectory(planning_result.plan.joint_trajectory)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def make_follow_joint_trajectory_goal(self, trajectory) -> FollowJointTrajectoryGoal:
        return make_follow_joint_trajectory_goal(trajectory)

    def follow_joint_trajectory(self,
                                trajectory: JointTrajectory,
                                client: SimpleActionClient,
                                stop_condition: Optional[Callable] = None):
        if is_empty_trajectory(trajectory):
            rospy.logdebug(f"ignoring empty trajectory")
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            return ExecutionResult(trajectory=trajectory,
                                   execution_result=result,
                                   action_client_state=client.get_state(),
                                   success=True)

        rospy.logdebug(f"sending trajectory goal with f{len(trajectory.points)} points")
        result: Optional[FollowJointTrajectoryResult] = None
        if self.execute:
            goal = self.make_follow_joint_trajectory_goal(trajectory)

            def _feedback_cb(feedback: FollowJointTrajectoryFeedback):
                for feedback_callback in self.feedback_callbacks:
                    feedback_callback(client, goal, feedback)
                if stop_condition is not None and stop_condition(feedback):
                    client.cancel_all_goals()

            # NOTE: this is where execution is actually requested in the form of a joint trajectory
            client.send_goal(goal, feedback_cb=_feedback_cb)
            if self.block:
                client.wait_for_result()
                result = client.get_result()
            if self.raise_on_failure and result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                raise RuntimeError(f"Follow Joint Trajectory Failed: ({result.error_code}) {result.error_string}")

        return ExecutionResult(trajectory=trajectory,
                               execution_result=result,
                               action_client_state=client.get_state(),
                               success=(result.error_code == FollowJointTrajectoryResult.SUCCESSFUL))

    def follow_joint_config(self, joint_names: List[str], joint_positions, client: SimpleActionClient):
        trajectory = JointTrajectory(joint_names=joint_names)
        point = JointTrajectoryPoint(time_from_start=rospy.Duration(secs=1), positions=joint_positions)
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
                    return ExecutionResult(trajectory=None,
                                           execution_result=None,
                                           action_client_state=self.arms_client.get_state(),
                                           success=False)
                preferred_tool_orientations.append(self.stored_tool_orientations[k])
        robot_trajectory_msg: RobotTrajectory = self.jacobian_follower.plan(
            group_name=group_name,
            tool_names=tool_names,
            preferred_tool_orientations=preferred_tool_orientations,
            grippers=points,
            max_velocity_scaling_factor=vel_scaling,
            max_acceleration_scaling_factor=0.1,
        )
        planning_success = True
        planning_result = PlanningResult(success=planning_success, plan=robot_trajectory_msg)
        if self.raise_on_failure and not planning_success:
            raise RuntimeError(f"Jacobian planning failed")

        execution_result = self.follow_arms_joint_trajectory(robot_trajectory_msg.joint_trajectory,
                                                             stop_condition=stop_condition)
        return PlanningAndExecutionResult(planning_result, execution_result)

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
        # NOTE: this function requires that gazebo be playing
        left_gripper = self.robot_commander.get_link(self.left_tool_name)
        right_gripper = self.robot_commander.get_link(self.right_tool_name)
        return left_gripper.pose().pose.position, right_gripper.pose().pose.position
