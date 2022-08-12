#! /usr/bin/env python
import warnings
from typing import List, Union, Tuple, Callable, Optional, Dict

import numpy as np
import pyjacobian_follower
from matplotlib import colors

from moveit_msgs.srv import GetPlanningSceneRequest, GetPlanningScene

with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=RuntimeWarning)
    import moveit_commander

import ros_numpy
import rospy
import urdf_parser_py.xml_reflection.core
from actionlib import SimpleActionClient
from arc_utilities.conversions import convert_to_pose_msg
from arc_utilities.ros_helpers import try_to_connect
from arm_robots.base_robot import BaseRobot
from arm_robots.robot_utils import make_follow_joint_trajectory_goal, PlanningResult, PlanningAndExecutionResult, \
    ExecutionResult, is_empty_trajectory, merge_joint_state_and_scene_msg
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult, \
    FollowJointTrajectoryGoal
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PointStamped
from moveit_msgs.msg import RobotTrajectory, DisplayRobotState, ObjectColor, RobotState, PlanningScene, \
    DisplayTrajectory, MoveItErrorCodes
from rosgraph.names import ns_join
from rospy import logfatal
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF as urdf
from visualization_msgs.msg import Marker

STORED_ORIENTATION = None

store_error_msg = ("No stored tool orientations! "
                   "You have to call store_tool_orientations or store_current_tool_orientations first")


class RobotPlanningError(Exception):
    pass


class FollowJointTrajectoryError(Exception):
    pass


def on_error(message):
    rospy.logdebug(message, logger_name='urdf_parser_py')


urdf_parser_py.xml_reflection.core.on_error = on_error


class MoveitEnabledRobot(BaseRobot):

    def __init__(self,
                 robot_namespace: str,
                 arms_controller_name: str,
                 robot_description: str = 'robot_description',
                 execute: bool = True,
                 block: bool = True,
                 raise_on_failure: bool = False,
                 display_goals: bool = True,
                 force_trigger: float = 9.0,
                 jacobian_follower: Optional[pyjacobian_follower.JacobianFollower] = None,
                 jacobian_target_not_reached_is_failure: Optional[bool] = True):
        super().__init__(robot_namespace, robot_description)
        self.jacobian_target_not_reached_is_failure = jacobian_target_not_reached_is_failure
        self._max_velocity_scale_factor = 0.1
        self.stored_tool_orientations = None
        self.raise_on_failure = raise_on_failure
        self.execute = execute
        self.block = block
        self.display_goals = display_goals
        self.force_trigger = force_trigger
        self.jacobian_follower = jacobian_follower

        self.arms_controller_name = arms_controller_name

        self.display_robot_state_pub = rospy.Publisher('display_robot_state', DisplayRobotState, queue_size=10)
        self.display_goal_position_pub = rospy.Publisher('goal_position', Marker, queue_size=10)
        self.display_robot_state_pubs = {}
        self.display_robot_traj_pubs = {}

        self.arms_client = None

        if jacobian_follower is None:
            self.jacobian_follower = pyjacobian_follower.JacobianFollower(robot_namespace=self.robot_namespace,
                                                                          robot_description=robot_description,
                                                                          translation_step_size=0.005,
                                                                          minimize_rotation=True,
                                                                          collision_check=True,
                                                                          visualize=True)

        self.feedback_callbacks = []
        self._move_groups = {}
        self.get_planning_scene_req = GetPlanningSceneRequest()
        self.get_planning_scene_srv = None
        self.planning_scene_service_name = ns_join(self.robot_namespace, 'get_planning_scene')

    def connect(self, preload_move_groups=True):
        """
        Args:
            preload_move_groups: Load the move_group_commander objects on connect (if False, loaded lazily)

        Returns:
        """
        super().connect()

        self.jacobian_follower.connect_to_psm()

        # connect to planning scene service
        # all 1's in binary, see http://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/PlanningSceneComponents.html
        self.get_planning_scene_req.components.components = 2 ** 10 - 1
        self.get_planning_scene_srv = rospy.ServiceProxy(self.planning_scene_service_name, GetPlanningScene)

        # TODO: bad api? raii? this class isn't fully usable by the time it's constructor finishes, that's bad.
        self.arms_client = self.setup_joint_trajectory_controller_client(self.arms_controller_name)
        if preload_move_groups:
            for group_name in self.robot_commander.get_group_names():
                self.get_move_group_commander(group_name)

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

    def get_move_group_commander(self, group_name: str) -> moveit_commander.MoveGroupCommander:
        if group_name not in self._move_groups:
            self._move_groups[group_name] = moveit_commander.MoveGroupCommander(group_name, ns=self.robot_namespace,
                                                                                robot_description=self.robot_description)
        move_group: moveit_commander.MoveGroupCommander = self._move_groups[group_name]
        move_group.clear_pose_targets()
        move_group.set_planning_time(30.0)

        # TODO Make this a settable param or at least make the hardcoded param more obvious
        # The purpose of this safety factor is to make sure we never send victor a velocity
        # faster than the kuka controller's max velocity, otherwise the kuka controllers will error out.
        safety_factor = 0.9
        move_group.set_max_velocity_scaling_factor(self._max_velocity_scale_factor * safety_factor)
        return move_group

    def plan_to_position(self,
                         group_name: str,
                         ee_link_name: str,
                         target_position,
                         stop_condition: Optional[Callable] = None):
        move_group = self.get_move_group_commander(group_name)
        move_group.set_end_effector_link(ee_link_name)
        move_group.set_position_target(list(target_position))

        planning_result = PlanningResult(move_group.plan())
        if self.raise_on_failure and not planning_result.success:
            raise RobotPlanningError(f"Plan to position failed {planning_result.planning_error_code}")

        execution_result = self.follow_arms_joint_trajectory(planning_result.plan.joint_trajectory, stop_condition)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def plan_to_position_cartesian(self,
                                   group_name: str,
                                   ee_link_name: str,
                                   target_position: Union[Point, List, np.array],
                                   step_size: float = 0.02,
                                   stop_condition: Optional[Callable] = None,
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
            raise RobotPlanningError(f"Cartesian path is only {fraction * 100}% complete")

        execution_result = self.follow_arms_joint_trajectory(plan.joint_trajectory, stop_condition)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def plan_to_poses(self, group_name, ee_links, target_poses: List[Pose]):
        for ee_link, target_pose_i in zip(ee_links, target_poses):
            self.display_goal_pose(target_pose_i, ee_link)

        nearest_ik_solution, _ = self.nearby_ik(ee_links, group_name, target_poses)

        group_joint_names = self.get_joint_names(group_name)
        target_joint_config = {}
        for n in group_joint_names:
            i = nearest_ik_solution.joint_state.name.index(n)
            p = nearest_ik_solution.joint_state.position[i]
            target_joint_config[n] = p
        return self.plan_to_joint_config(group_name, target_joint_config)

    def nearby_ik(self, ee_links, group_name, target_poses):
        joint_limits_param = rospy.get_param(self.robot_description + '_planning/joint_limits')
        distance_weights = {k: v['distance_weight'] for k, v in joint_limits_param.items()}
        current_scene_response = self.get_planning_scene_srv(self.get_planning_scene_req)
        current_scene = current_scene_response.scene
        current_positions = np.array(current_scene.robot_state.joint_state.position)
        exp_joint_weights = [distance_weights.get(n, 1) for n in current_scene.robot_state.joint_state.name]
        min_d = 1e9
        nearest_ik_solution = None  # TODO: seed the IK solver with the best solution so far? or decay rng_dist?
        min_ds = []
        for i in range(500):
            ik_solution = self.jacobian_follower.compute_collision_free_pose_ik(current_scene.robot_state, target_poses,
                                                                                group_name, ee_links, current_scene)
            if ik_solution is None:
                continue
            ik_positions = np.array(ik_solution.joint_state.position)
            joint_distance = abs(ik_positions - current_positions)
            weighted_joint_distance = (exp_joint_weights * joint_distance).sum()
            if weighted_joint_distance < min_d:
                self.display_robot_state(ik_solution, 'ik_solution')
                min_d = weighted_joint_distance
                nearest_ik_solution = ik_solution
            min_ds.append(min_d)
        if nearest_ik_solution is None:
            raise RobotPlanningError("No IK Solution found!")
        self.display_robot_state(nearest_ik_solution, 'nearest_ik_solution')
        return nearest_ik_solution, min_ds

    def plan_to_pose(self, group_name, ee_link_name, target_pose, frame_id: str = 'robot_root',
                     start_state: Optional[RobotState] = None, stop_condition: Optional[Callable] = None,
                     position_tol=0.002, orientation_tol=0.02, timeout=None):
        self.check_inputs(group_name, ee_link_name)
        move_group = self.get_move_group_commander(group_name)

        if start_state is not None:
            move_group.set_start_state(start_state)

        if timeout is not None:
            move_group.set_planning_time(timeout)

        move_group.set_end_effector_link(ee_link_name)
        target_pose_stamped = convert_to_pose_msg(target_pose)
        target_pose_stamped.header.frame_id = frame_id
        move_group.set_pose_target(target_pose_stamped)
        move_group.set_goal_position_tolerance(position_tol)
        move_group.set_goal_orientation_tolerance(orientation_tol)

        if self.display_goals:
            self.display_goal_pose(target_pose_stamped.pose)

        planning_result = PlanningResult(move_group.plan())
        if self.raise_on_failure and not planning_result.success:
            raise RobotPlanningError(f"Plan to pose failed {planning_result.planning_error_code}")

        execution_result = self.follow_arms_joint_trajectory(planning_result.plan.joint_trajectory, stop_condition)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def get_link_pose(self, link_name: str):
        # FIXME: there's a bug where link.pose() returns the wrong joint state and prints a warning.
        #  and I don't know how to wait for the right state
        # link: moveit_commander.RobotCommander.Link = self.robot_commander.get_link(link_name)
        # pose_stamped: PoseStamped = link.pose()
        # return pose_stamped.pose
        transform = self.tf_wrapper.get_transform(self.robot_commander.get_planning_frame(), link_name)
        pose = ros_numpy.msgify(Pose, transform)
        return pose

    def plan_to_joint_config(self, group_name: str, joint_config: Union[List, Dict, str],
                             start_state: Optional[RobotState] = None,
                             stop_condition: Optional[Callable] = None, timeout=None):
        """
        Args:
            group_name: group name, defined in the SRDF
            joint_config: Either a dictionary of joint_name: joint value,
             or a string for the group_state defined in the SRDF. You can technically use a list
             here instead of a dict but you need to provide all joints in the right order,
             which is hard to get right and gives bad error messages
            start_state: robot start state. If None, uses the current robot state
            stop_condition: optional stop condition function to halt execution early.

        Returns:
            The result message of following the trajectory
        """
        move_group = self.get_move_group_commander(group_name)

        if start_state is not None:
            move_group.set_start_state(start_state)

        if timeout is not None:
            move_group.set_planning_time(timeout)

        if isinstance(joint_config, str):
            joint_config_name = joint_config
            joint_config = move_group.get_named_target_values(joint_config_name)
            if len(joint_config) == 0:
                raise ValueError(f"No group state named {joint_config_name}")
        if isinstance(joint_config, List):
            joint_config = {name: val for name, val in zip(move_group.get_active_joints(), joint_config)}

        move_group.set_joint_value_target(joint_config)

        if self.display_goals:
            robot_state = RobotState(joint_state=JointState(name=list(joint_config.keys()),
                                                            position=list(joint_config.values())))
            self.display_robot_state(robot_state, label='joint_config_goal')

        planning_result = PlanningResult(move_group.plan())
        if planning_result.planning_error_code.val == MoveItErrorCodes.INVALID_MOTION_PLAN:
            print("Invalid plan, trying to replan.")
            planning_result = PlanningResult(move_group.plan())

        if self.raise_on_failure and not planning_result.success:
            raise RobotPlanningError(f"Plan to joint config failed {planning_result.planning_error_code}")
        execution_result = self.follow_arms_joint_trajectory(planning_result.plan.joint_trajectory, stop_condition)
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

            if client is None:
                action_client_state = None
            else:
                action_client_state = client.get_state()

            return ExecutionResult(trajectory=trajectory,
                                   execution_result=result,
                                   action_client_state=action_client_state,
                                   success=True)

        rospy.logdebug(f"sending trajectory goal with f{len(trajectory.points)} points")
        result: Optional[FollowJointTrajectoryResult] = None
        if self.execute:
            if client is None:
                raise ConnectionError("You asked to execute an action without calling connect() first!")

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
            failure = (result is None or result.error_code != FollowJointTrajectoryResult.SUCCESSFUL)
            if self.raise_on_failure and failure:
                if result is not None:
                    raise FollowJointTrajectoryError(
                        f"Follow Joint Trajectory Failed: ({result.error_code}) {result.error_string}")
                raise FollowJointTrajectoryError(f"Follow Joint Trajectory Failed: (???)")

        success = result is not None and result.error_code == FollowJointTrajectoryResult.SUCCESSFUL

        if client is None:
            action_client_state = None
        else:
            action_client_state = client.get_state()

        return ExecutionResult(trajectory=trajectory,
                               execution_result=result,
                               action_client_state=action_client_state,
                               success=success)

    def follow_joint_config(self, joint_names: List[str], joint_positions, client: SimpleActionClient):
        trajectory = JointTrajectory(joint_names=joint_names)
        point = JointTrajectoryPoint(time_from_start=rospy.Duration(secs=1), positions=joint_positions)
        trajectory.points.append(point)
        return self.follow_joint_trajectory(trajectory, client)

    def follow_arms_joint_trajectory(self, trajectory: JointTrajectory, stop_condition: Optional[Callable] = None):
        return self.follow_joint_trajectory(trajectory, self.arms_client, stop_condition=stop_condition)

    def distance(self, ee_link_name: str, target_position):
        current_pose = self.get_link_pose(ee_link_name)
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

    def merge_tool_orientations_with_defaults(self, tool_names: List[str],
                                              preferred_tool_orientations: Optional[List] = STORED_ORIENTATION):
        # If preferred_tool_orientations is None, we use the stored ones as a fallback
        if preferred_tool_orientations == STORED_ORIENTATION:
            preferred_tool_orientations = []
            if self.stored_tool_orientations is None:
                logfatal(store_error_msg)
            for k in tool_names:
                if k not in self.stored_tool_orientations:
                    rospy.logerr(f"tool {k} has no stored orientation. aborting.")
                    return []
                preferred_tool_orientations.append(self.stored_tool_orientations[k])
        return preferred_tool_orientations

    def follow_jacobian_to_position_from_scene_and_state(self,
                                                         scene_msg: PlanningScene,
                                                         joint_state: JointState,
                                                         group_name: str,
                                                         tool_names: List[str],
                                                         points: List[List],
                                                         preferred_tool_orientations: Optional = STORED_ORIENTATION,
                                                         vel_scaling=0.1,
                                                         stop_condition: Optional[Callable] = None,
                                                         ):
        if isinstance(tool_names, str):
            err_msg = "you need to pass in a list of strings, not a single string."
            rospy.logerr(err_msg)
            raise ValueError(err_msg)

        preferred_tool_orientations = self.merge_tool_orientations_with_defaults(tool_names,
                                                                                 preferred_tool_orientations)
        if len(preferred_tool_orientations) == 0:
            return ExecutionResult(trajectory=None,
                                   execution_result=None,
                                   action_client_state=self.arms_client.get_state(),
                                   success=False)

        robot_trajectory_msg: RobotTrajectory
        reached: bool
        scene_msg, robot_state = merge_joint_state_and_scene_msg(scene_msg, joint_state)
        robot_trajectory_msg, reached = self.jacobian_follower.plan(
            group_name=group_name,
            tool_names=tool_names,
            preferred_tool_orientations=preferred_tool_orientations,
            start_state=robot_state,
            scene=scene_msg,
            grippers=points,
            max_velocity_scaling_factor=vel_scaling,
            max_acceleration_scaling_factor=0.1,
        )

        if self.jacobian_target_not_reached_is_failure:
            planning_success = reached
        else:
            planning_success = True

        planning_result = PlanningResult(success=planning_success, plan=robot_trajectory_msg)
        if self.raise_on_failure and not planning_success:
            raise RobotPlanningError(f"Tried to execute a jacobian action which could not be reached")

        execution_result = self.follow_arms_joint_trajectory(robot_trajectory_msg.joint_trajectory,
                                                             stop_condition=stop_condition)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def follow_jacobian_to_position(self,
                                    group_name: str,
                                    tool_names: List[str],
                                    points: List[List],
                                    preferred_tool_orientations: Optional[List] = STORED_ORIENTATION,
                                    vel_scaling=0.1,
                                    stop_condition: Optional[Callable] = None,
                                    ):
        if isinstance(tool_names, str):
            err_msg = "you need to pass in a list of strings, not a single string."
            rospy.logerr(err_msg)
            raise ValueError(err_msg)

        preferred_tool_orientations = self.merge_tool_orientations_with_defaults(tool_names,
                                                                                 preferred_tool_orientations)
        if len(preferred_tool_orientations) == 0:
            return ExecutionResult(trajectory=None,
                                   execution_result=None,
                                   action_client_state=self.arms_client.get_state(),
                                   success=False)

        robot_trajectory_msg: RobotTrajectory
        reached: bool
        robot_trajectory_msg, reached = self.jacobian_follower.plan(
            group_name=group_name,
            tool_names=tool_names,
            preferred_tool_orientations=preferred_tool_orientations,
            grippers=points,
            max_velocity_scaling_factor=vel_scaling,
            max_acceleration_scaling_factor=0.1,
        )
        if self.jacobian_target_not_reached_is_failure:
            planning_success = reached
        else:
            planning_success = True

        planning_result = PlanningResult(success=planning_success, plan=robot_trajectory_msg)
        if self.raise_on_failure and not planning_success:
            raise RobotPlanningError(f"Jacobian planning failed")

        execution_result = self.follow_arms_joint_trajectory(robot_trajectory_msg.joint_trajectory,
                                                             stop_condition=stop_condition)
        return PlanningAndExecutionResult(planning_result, execution_result)

    def get_joint_names(self, group_name: Optional[str] = None):
        # NOTE: Getting a list of joint names should not require anything besides a lookup on the ros parameter server.
        #   However, MoveIt does not have this implemented that way, it requires connecting to the move group node.
        #   This causes problems sometimes, so to simplify things we do the lookup manually if no group name is given.
        if group_name is None:
            robot = urdf.from_parameter_server(key=self.robot_description)
            active_joint_names = [j.name for j in robot.joints if j.type != 'fixed']
            return active_joint_names
        if hasattr(self.robot_commander, 'get_active_joint_names'):
            return self.robot_commander.get_active_joint_names(group_name)

        return self.get_move_group_commander(group_name).get_active_joints()

    def get_num_joints(self, group_name: Optional[str] = None):
        return len(self.get_joint_names(group_name))

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        raise NotImplementedError()

    def display_robot_traj(self, trajectory: RobotTrajectory, label: str, color: Optional = None):
        """

        Args:
            trajectory:
            label:
            color: any kind of matplotlib color, e.g "blue", [0,0.6,1], "#ff0044", etc...

        Returns:

        """
        topic_name = rospy.names.ns_join('display_robot_trajectory', label)
        topic_name = topic_name.rstrip('/')

        display_robot_traj_pub = self.display_robot_traj_pubs.get(label, None)
        if display_robot_traj_pub is None:
            display_robot_traj_pub = rospy.Publisher(topic_name, DisplayTrajectory, queue_size=10)
            try_to_connect(display_robot_traj_pub)
            self.display_robot_traj_pubs[label] = display_robot_traj_pub  # save a handle to the publisher

        display_robot_traj_msg = DisplayTrajectory(model_id=self.robot_namespace,
                                                   trajectory=[trajectory])

        display_robot_traj_pub.publish(display_robot_traj_msg)

    def display_robot_state(self, robot_state: RobotState, label: str, color: Optional = None):
        """

        Args:
            robot_state:
            label:
            color: any kind of matplotlib color, e.g "blue", [0,0.6,1], "#ff0044", etc...

        Returns:

        """
        topic_name = rospy.names.ns_join('display_robot_state', label)
        topic_name = topic_name.rstrip('/')

        display_robot_state_pub = self.display_robot_state_pubs.get(label, None)
        if display_robot_state_pub is None:
            display_robot_state_pub = rospy.Publisher(topic_name, DisplayRobotState, queue_size=10)
            try_to_connect(display_robot_state_pub)
            self.display_robot_state_pubs[label] = display_robot_state_pub  # save a handle to the publisher

        display_robot_state_msg = DisplayRobotState()
        display_robot_state_msg.state = robot_state
        display_robot_state_msg.state.joint_state.header.stamp = rospy.Time.now()
        display_robot_state_msg.state.is_diff = False

        if color is not None:
            color = ColorRGBA(*colors.to_rgba(color))
            for link_name in self.robot_commander.get_link_names():
                object_color = ObjectColor(id=link_name, color=color)
                display_robot_state_msg.highlight_links.append(object_color)

        display_robot_state_pub.publish(display_robot_state_msg)

    def display_goal_position(self, point: PointStamped):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = point.header.frame_id
        m.id = 0
        m.action = Marker.ADD
        m.action = Marker.SPHERE
        m.color = ColorRGBA(r=0, g=1, b=0, a=1)
        s = 0.02
        m.scale = Vector3(x=s, y=s, z=s)
        m.pose.position = point.point
        m.pose.orientation.w = 1
        self.display_goal_position_pub.publish(m)

    def display_goal_pose(self, pose: Pose, label: Optional[str] = None):
        name = 'arm_robots_goal'
        if label is not None and label != '':
            name = f'{name}-{label}'
        self.tf_wrapper.send_transform_from_pose_msg(pose, 'robot_root', name)

    def get_state(self, group_name: str = None):
        robot_state = RobotState()
        if group_name is None:
            joint_names = self.get_joint_names()
        else:
            joint_names = self.get_move_group_commander(group_name).get_active_joints()

        robot_state.joint_state.name = joint_names
        robot_state.joint_state.position = self.get_joint_positions(joint_names)
        robot_state.joint_state.velocity = self.get_joint_velocities(joint_names)
        return robot_state

    def estimated_torques(self, robot_state: RobotState, group, wrenches=None):
        return self.jacobian_follower.estimated_torques(robot_state, group, wrenches)

    def get_current_jacobian(self, group_name: str, link_name: str):
        current_joint_positions = self.get_joint_positions(self.get_joint_names(group_name))
        return self.jacobian_follower.get_jacobian(group_name, link_name, current_joint_positions)

    def get_jacobian(self, group_name: str, joint_positions):
        return self.jacobian_follower.get_jacobian(group_name, joint_positions)

    def get_base_link(self, group_name: str):
        return self.jacobian_follower.get_base_link(group_name)
