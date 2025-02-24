#! /usr/bin/env python
from typing import Tuple, Sequence

import numpy as np
from colorama import Fore
from typing import List, Union, Dict, Tuple, Sequence, Callable, Optional
import pyjacobian_follower
import rospy
from arm_robots.robot import FollowJointTrajectoryError
from arm_robots.trajectory_follower import TrajectoryFollower
from rosgraph.names import ns_join
from mik_tools import matrix_to_pose, pose_to_matrix
from arc_utilities.conversions import convert_to_pose_msg
from arc_utilities.listener import Listener
from arm_robots.base_robot import BaseRobot
from arm_robots.config.med_config import ARM_JOINT_NAMES, COMBINED_ARM_JOINT_NAMES
from arm_robots.robot_utils import make_follow_joint_trajectory_goal, PlanningResult, PlanningAndExecutionResult, \
    ExecutionResult, is_empty_trajectory, merge_joint_state_and_scene_msg
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult, \
    FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from victor_hardware_interface.victor_utils import get_control_mode_params, list_to_jvq, jvq_to_list
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus, MotionCommand
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode, GetControlModeRequest, \
    GetControlModeResponse, SetControlModeResponse
import time
import os
from trajectory_msgs.msg import JointTrajectoryPoint
import pdb
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, PositionIKRequest, PlanningScene
from arm_robots.robot_utils import PlanningResult
from moveit_msgs.srv import GetPositionIK
from arc_utilities.transformation_helper import PoseFromComponents
import copy
from pyjacobian_follower import IkParams
from arm_robots.robot_utils import get_ordered_tolerance_list, interpolate_joint_trajectory_points, is_waypoint_reached, \
    waypoint_error


def delegate_to_arms(positions: List, joint_names: Sequence[str]) -> Tuple[Dict[str, List], bool, str]:
    """
    Given a list (e.g. of positions) and a corresponding list of joint names,
    assign and order by victor's joint groups.

    Args:
        positions: values to delegate
        joint_names: list of joint_names

    Returns:
        object: (map from joint_names to values, abort?, abort_msg)
    """
    assert len(positions) == len(joint_names), "positions and joint_names must be same length"

    # TODO: Why can't joint_names be a combination of arm and gripper joints?
    ok = set(joint_names) in [set(COMBINED_ARM_JOINT_NAMES)]

    if not ok:
        blank_positions = {n: None for n in ['arm']}
        return blank_positions, True, f"Invalid joint_names {joint_names}"

    joint_position_of = dict(zip(joint_names, positions))

    def fill_using(joint_ordering: Sequence[str]):
        if not all(j in joint_position_of for j in joint_ordering):
            return None
        return [joint_position_of[name] for name in joint_ordering]

    positions_by_interface = {
        'thanos': fill_using(COMBINED_ARM_JOINT_NAMES[:7]),
        'medusa': fill_using(COMBINED_ARM_JOINT_NAMES[7:]),
    }
    # set equality ignores order

    return positions_by_interface, False, ""


def follow_trajectory_goal(traj_msg: FollowJointTrajectoryGoal,
                           robot: BaseRobot,
                           stop_cb: Optional[Callable] = lambda: (False, "")):
    if len(traj_msg.trajectory.points) == 0:
        rospy.loginfo('Trajectory provided is empty -- ignoring it')
        return
    # construct a list of the tolerances in order of the joint names
    trajectory_joint_names = traj_msg.trajectory.joint_names
    tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.path_tolerance)
    goal_tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.goal_tolerance, is_goal=True)
    interpolated_points = interpolate_joint_trajectory_points(traj_msg.trajectory.points, max_step_size=0.01)

    if len(interpolated_points) == 0:
        rospy.loginfo('Trajectory is empty after interpolating -- ignoring it')
        return

    trajectory_point_idx = 0
    t_0 = rospy.Time.now()
    while True:
        # tiny sleep lets the listeners process messages better, results in smoother following
        rospy.sleep(1e-3)
        dt = rospy.Time.now() - t_0
        # get feedback
        new_waypoint = False
        actual_point = JointTrajectoryPoint(positions=robot.get_joint_positions(trajectory_joint_names), time_from_start=dt)
        while trajectory_point_idx < len(interpolated_points) - 1 and is_waypoint_reached(actual_point, interpolated_points[trajectory_point_idx], tolerance):
            trajectory_point_idx += 1
            new_waypoint = True

        desired_point = interpolated_points[trajectory_point_idx]

        if trajectory_point_idx >= len(interpolated_points) - 1 and is_waypoint_reached(actual_point, desired_point, goal_tolerance):
            return

        if new_waypoint:
            command_failed, command_failed_msg = robot.send_joint_command(trajectory_joint_names, desired_point)
            if command_failed:
                rospy.logwarn(f"Command failed with message: {command_failed}")
                return FollowJointTrajectoryResult(error_code=-10, error_string=command_failed_msg)

        # let the caller stop
        stop, stop_msg = False, ""
        # stop, stop_msg = stop_cb(actual_point)

        error = waypoint_error(actual_point, desired_point)
        rospy.logdebug_throttle(1, f"{error} {desired_point.time_from_start.to_sec()} {dt.to_sec()}")
        if desired_point.time_from_start.to_sec() > 0 and dt > desired_point.time_from_start * 5.0:
            stop = True
            if trajectory_point_idx == len(interpolated_points) - 1:
                stop_msg = f"timeout. expected t={desired_point.time_from_start.to_sec()} but t={dt.to_sec()}." \
                           + f" error to waypoint is {error}, goal tolerance is {goal_tolerance}"
            else:
                stop_msg = f"timeout. expected t={desired_point.time_from_start.to_sec()} but t={dt.to_sec()}." \
                           + f" error to waypoint is {error}, tolerance is {tolerance}"

        if stop:
            # command the current configuration
            actual_point.velocities = [0.0] * len(actual_point.positions)
            robot.send_joint_command(trajectory_joint_names, actual_point)
            rospy.loginfo("Preempt requested, aborting.")
            rospy.logwarn(f"Stopped with message: {stop_msg}")
            return FollowJointTrajectoryResult(error_code=-10, error_string=stop_msg)


class DualMed(BaseRobot):
    def __init__(self, robot_namespace: str = 'combined_med', thanos_prefix='thanos', medusa_prefix='medusa', block=True, force_trigger: float = -0.0, base_kwargs=None, **kwargs):
        self._init_ros_node()
        self.block = block
        BaseRobot.__init__(self, robot_namespace=robot_namespace, **kwargs)

        # self.trajectory_follower_client = None
        # start the trajectory follower client --
        self.controller_name = 'combined_med_trajectory_controller'
        self.trajectory_follower_server = TrajectoryFollower(self, controller_name=self.controller_name)
        self.trajectory_follower_server.start_server()
        self.trajectory_follower_client = self._setup_trajectory_follower_client()

        self.thanos_prefix = thanos_prefix
        self.medusa_prefix = medusa_prefix

        self.thanos_arm_command_pub = rospy.Publisher( f'/{self.thanos_prefix}/motion_command', MotionCommand, queue_size=10)
        self.medusa_arm_command_pub = rospy.Publisher(f'/{self.medusa_prefix}/motion_command', MotionCommand, queue_size=10)
        
        self.thanos_arm_status_listener = Listener(f'/{self.thanos_prefix}/motion_status', MotionStatus)
        self.medusa_arm_status_listener = Listener(f'/{self.medusa_prefix}/motion_status', MotionStatus)
        
        self.thanos_set_control_mode_srv = rospy.ServiceProxy(f'/{self.thanos_prefix}/set_control_mode_service', SetControlMode)
        self.medusa_set_control_mode_srv = rospy.ServiceProxy(f'/{self.medusa_prefix}/set_control_mode_service', SetControlMode)

        self.thanos_get_control_mode_srv = rospy.ServiceProxy(f'/{self.thanos_prefix}/get_control_mode_service', GetControlMode)
        self.medusa_get_control_mode_srv = rospy.ServiceProxy(f'/{self.medusa_prefix}/get_control_mode_service', GetControlMode)

        self.ik_proxy = self._init_ik_client()

        self.scene_listener = Listener(f'/{self.robot_namespace}/move_group/monitored_planning_scene', PlanningScene)

    def _init_ros_node(self):
        try:
            rospy.init_node('dual_med')
        except (rospy.exceptions.ROSInitException, rospy.exceptions.ROSException):
            pass

    def _init_ik_client(self):
        service_name = 'combined_med/compute_ik'
        rospy.wait_for_service(service_name)
        try:
            ik_proxy = rospy.ServiceProxy(service_name, GetPositionIK)
            return ik_proxy
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _setup_trajectory_follower_client(self):
        controller_name = self.controller_name
        action_name = ns_join(self.robot_namespace, ns_join(controller_name, "follow_joint_trajectory"))
        client = SimpleActionClient(action_name, FollowJointTrajectoryAction)
        resolved_action_name = rospy.resolve_name(action_name)
        wait_msg = f"Waiting for joint trajectory follower server {resolved_action_name}..."
        rospy.loginfo(wait_msg)
        client.wait_for_server()
        rospy.loginfo(f"Joint trajectory follower server connected.")
        return client

    def _call_ik_solver(self, srv_input):
        try:
            ik_resp = self.ik_proxy(srv_input)
            return ik_resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _get_move_group_commander(self, group_name=None) -> moveit_commander.MoveGroupCommander:
        group_name = group_name or 'combined_med'
        move_group = moveit_commander.MoveGroupCommander(group_name, ns='combined_med',
                                                         robot_description=rospy.resolve_name('robot_description'))
        return move_group

    def _unpack_joints(self, joints, joint_names, reference_joint_names=None):
        if reference_joint_names is None:
            reference_joint_names = self.get_arm_joints()
        joints_out = []
        for joint_name_i in reference_joint_names:
            joint_indx = joint_names.index(joint_name_i)
            joints_out.append(joints[joint_indx])
        return joints_out

    def follow_arms_joint_trajectory(self, trajectory: JointTrajectory,
                                stop_condition: Optional[Callable] = lambda: (False, "")):
        # PACK THE GOAL -
        goal = self.make_follow_joint_trajectory_goal(trajectory)
        follow_trajectory_goal(traj_msg=goal, robot=self, stop_cb=stop_condition)

    def follow_arms_joint_trajectory_server(self,
                                trajectory: JointTrajectory,
                                stop_condition: Optional[Callable] = None):
        client = self.trajectory_follower_client
        if is_empty_trajectory(trajectory):
            rospy.logdebug(f"ignoring empty trajectory")
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            success = True
        else:
            rospy.logdebug(f"sending trajectory goal with f{len(trajectory.points)} points")
            result: Optional[FollowJointTrajectoryResult] = None
            if client is None:
                raise ConnectionError("You asked to execute an action without calling connect() first!")

            # PACK THE GOAL -
            goal = self.make_follow_joint_trajectory_goal(trajectory)

            def _feedback_cb(feedback: FollowJointTrajectoryFeedback):
                if stop_condition is not None and stop_condition(feedback):
                    client.cancel_all_goals()

            # SEND THE GOAL -
            client.send_goal(goal, feedback_cb=_feedback_cb)

            if self.block:
                client.wait_for_result()
                result = client.get_result()

            # Process the result ---
            failure = (result is None or result.error_code != FollowJointTrajectoryResult.SUCCESSFUL)
            if failure:
                raise FollowJointTrajectoryError(f"Follow Joint Trajectory Failed: (???)")
            success = result is not None and result.error_code == FollowJointTrajectoryResult.SUCCESSFUL

       # return an execution result packed
        if client is None:
            action_client_state = None
        else:
            action_client_state = client.get_state()
        return ExecutionResult(trajectory=trajectory,
                               execution_result=result,
                               action_client_state=action_client_state,
                               success=success)


    def make_follow_joint_trajectory_goal(self, trajectory) -> FollowJointTrajectoryGoal:
        return make_follow_joint_trajectory_goal(trajectory)

    def compute_ik_combined_med(self,
                                thanos_target_pose=None,
                                medusa_target_pose=None,
                                thanos_ik_link_name:str='thanos_grasp_frame',
                                medusa_ik_link_name:str='medusa_grasp_frame',
                                ref_frame='bimanual_base',
                                thanos_ref_frame=None,
                                medusa_ref_frame=None,
                                init_state=None, joint_names=None):
        # call the ik service:
        # move_group = self.get_move_group_commander(group_name=group_name)
        group_name = 'combined_med'
        move_group = self._get_move_group_commander()
        ik_request = PositionIKRequest()
        ik_request.group_name = group_name  # string
        if init_state is None:
            ik_request.robot_state = move_group.get_current_state()
        else:
            ik_request.robot_state = init_state

        # solve the poses
        if thanos_ref_frame is None:
            thanos_ref_frame = ref_frame
        if medusa_ref_frame is None:
            medusa_ref_frame = ref_frame

        if thanos_target_pose is None:
            thanos_target_pose = self.get_current_pose_thanos(frame_id=thanos_ik_link_name, ref_frame=thanos_ref_frame, as_matrix=False)
        if medusa_target_pose is None:
            medusa_target_pose = self.get_current_pose_medusa(frame_id=medusa_ik_link_name, ref_frame=medusa_ref_frame, as_matrix=False)

        if isinstance(thanos_target_pose, np.ndarray):
            thanos_target_pose = thanos_target_pose.tolist()
        if isinstance(medusa_target_pose, np.ndarray):
            medusa_target_pose = medusa_target_pose.tolist()
        thanos_target_pose_stamped = convert_to_pose_msg(thanos_target_pose)
        medusa_target_pose_stamped = convert_to_pose_msg(medusa_target_pose)
        thanos_target_pose_stamped.header.frame_id = thanos_ref_frame
        medusa_target_pose_stamped.header.frame_id = medusa_ref_frame

        ik_request.avoid_collisions = True
        # single arm:
        # ik_request.pose_stamped = target_pose_stamped
        # ik_request.ik_link_name = ee_link_name
        # multiple arms:
        # NOTE: There is 'ik_link_names' which is a list of strings when there are multiple end effectors
        # same for pose_stamped_vector which is a list of PoseStamped, one per ik_link_name
        ik_request.ik_link_name = '' # empty so we let the node know that we will provide multiple frames
        ik_request.ik_link_names = [medusa_ik_link_name, thanos_ik_link_name] # THE ORDER IS BACKWARDS FROM THE SRDF
        ik_request.pose_stamped_vector = [medusa_target_pose_stamped, thanos_target_pose_stamped]
        ik_out = self._call_ik_solver(ik_request)
        ik_solution = ik_out.solution
        error_code = ik_out.error_code
        # reorder the joints to match the order of the combined_med group
        joint_names = joint_names or self.get_arm_joints()
        # if error_code.val != 1:
        #     return None, error_code
        try:
            robot_joints = self._unpack_joints(ik_solution.joint_state.position, joint_names=ik_solution.joint_state.name, reference_joint_names=joint_names)
        except Exception as e:
            import pdb; pdb.set_trace()
            print(e)
        return robot_joints, error_code

    def get_current_pose(self, frame_id, ref_frame='bimanual_base', as_matrix=False):
        """
        Get the current pose of the frame frame_id in reference to ref_frame as [x,y,z, qx, qy, qz, qw]
        """
        rf_X_fid = self.tf_wrapper.get_transform(ref_frame, frame_id)  # current pose as a 4x4 matrix homogenous transformation
        if as_matrix:
            return rf_X_fid
        rf_pose_fid = matrix_to_pose(rf_X_fid)  # current pose as [x,y,z, qx, qy, qz, qw]
        return rf_pose_fid

    def get_current_pose_thanos(self, frame_id='thanos_grasp_frame', ref_frame='bimanual_base', as_matrix=False):
        """
        Get the current pose of the frame frame_id in reference to ref_frame as [x,y,z, qx, qy, qz, qw]
        """
        return self.get_current_pose(frame_id=frame_id, ref_frame=ref_frame, as_matrix=as_matrix)

    def get_current_pose_medusa(self, frame_id='medusa_grasp_frame', ref_frame='bimanual_base', as_matrix=False):
        """
        Get the current pose of the frame frame_id in reference to ref_frame as [x,y,z, qx, qy, qz, qw]
        """
        return self.get_current_pose(frame_id=frame_id, ref_frame=ref_frame, as_matrix=as_matrix)

    def get_arm_joints(self):
        return COMBINED_ARM_JOINT_NAMES
    
    def get_thanos_arm_control_mode(self):
        thanos_control_mode_res: GetControlModeResponse = self.thanos_get_control_mode_srv(GetControlModeRequest())
        return thanos_control_mode_res.active_control_mode.control_mode

    def get_medusa_arm_control_mode(self):
        medusa_control_mode_res: GetControlModeResponse = self.medusa_get_control_mode_srv(GetControlModeRequest())
        return medusa_control_mode_res.active_control_mode.control_mode

    def set_medusa_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        res: SetControlModeResponse = self.medusa_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch medusa arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)
        return res

    def set_thanos_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_control_mode_params(control_mode, **kwargs)
        res: SetControlModeResponse = self.thanos_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch thanos arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)
        return res

    def set_thanos_joint_position_control(self, vel=0.1, **kwargs):
        self.set_thanos_arm_control_mode(ControlMode.JOINT_POSITION, vel=vel, **kwargs)

    def set_medusa_joint_position_control(self, vel=0.1, **kwargs):
        self.set_medusa_arm_control_mode(ControlMode.JOINT_POSITION, vel=vel, **kwargs)

    def set_joint_position_control(self, vel=0.1, **kwargs):
        self.set_thanos_joint_position_control(vel=vel, **kwargs)
        self.set_medusa_joint_position_control(vel=vel, **kwargs)

    def get_control_modes(self):
        return {'thanos': self.get_thanos_arm_control_mode(), 'medusa': self.get_medusa_arm_control_mode()}
    
    def get_joint_positions_map(self) -> Dict[str, float]:
        all_joint_vals = jvq_to_list(self.thanos_arm_status_listener.get().measured_joint_position)
        all_joint_vals += jvq_to_list(self.medusa_arm_status_listener.get().measured_joint_position)
        return {n: val for n, val in zip(COMBINED_ARM_JOINT_NAMES, all_joint_vals)}
    
    def reached_endpoint(self, target, joint_names=COMBINED_ARM_JOINT_NAMES, tol=1e-2):
        target_joints = dict(zip(joint_names, target))
        current_joints = self.get_joint_positions_map()
        
        error = np.array([abs(target_joints[j] - current_joints[j]) for j in joint_names])
        
        return np.all(error < tol)
    
    def send_arm_command(self, command_pub: rospy.Publisher, control_mode: ControlMode,
                         positions, velocities=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        if positions is None:
            return

        def trunc(values, decs=0):
            return np.trunc(values * 10 ** decs) / (10 ** decs)

        velocities = trunc(np.array(velocities), 3)  # Kuka does not like sending small but non-zero velocity commands

        # TODO: use enforce bounds? https://github.com/ros-planning/moveit/pull/2356
        low, high = self.get_joint_limits(COMBINED_ARM_JOINT_NAMES[:7], safety_margin=1e-2)
        limit_enforced_positions = np.clip(positions, low, high)

        # TODO: enforce velocity limits
        cmd = MotionCommand(joint_position=list_to_jvq(limit_enforced_positions),
                            joint_velocity=list_to_jvq(velocities),
                            control_mode=control_mode)
        cmd.header.stamp = rospy.Time.now()
        
        command_pub.publish(cmd)
        
    def send_joint_command(self, joint_names: Sequence[str], trajectory_point: JointTrajectoryPoint) -> Tuple[
        bool, str]:
        # TODO: in victor's impedance mode, we want to modify the setpoint so that there is a limit
        #  on the force we will apply
        positions, abort, msg = delegate_to_arms(trajectory_point.positions, joint_names)
        if abort:
            return True, msg

        velocities, _, _ = delegate_to_arms([0.0] * len(COMBINED_ARM_JOINT_NAMES), COMBINED_ARM_JOINT_NAMES)
        if len(trajectory_point.velocities) != 0:
            velocities, abort, msg = delegate_to_arms(trajectory_point.velocities, joint_names)
        if abort:
            return True, msg

        # Get the current control mode
        control_mode = self.get_control_modes()
        thanos_arm_control_mode = control_mode['thanos']
        medusa_arm_control_mode = control_mode['medusa']

        self.send_arm_command(self.thanos_arm_command_pub, thanos_arm_control_mode,
                              positions['thanos'], velocities['thanos'])
        self.send_arm_command(self.medusa_arm_command_pub, medusa_arm_control_mode,
                              positions['medusa'], velocities['medusa'])
        return False, ""
    
    def goto_config(self, joint_positions, joint_names=COMBINED_ARM_JOINT_NAMES, control_mode=ControlMode.JOINT_POSITION, tol=1e-3, **kwargs):
        '''
        NOTE: trajectory_point takes in thanos_joint0 -> thanos_joint7 + medusa_joint0 -> medusa_joint7
        '''
        traj_point = JointTrajectoryPoint()
        traj_point.positions = joint_positions
        traj_point.velocities = np.zeros(len(joint_positions))
        
        self.send_joint_command(joint_names, traj_point)
        while not self.reached_endpoint(joint_positions, joint_names=joint_names, tol=tol):
            time.sleep(1e-5)
        return False, ""

    def joints_to_jointstate_msg(self, joints):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.get_arm_joints()
        msg.position = joints
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        return msg

    def jointstate_to_robotstate_msg(self, jointstate_msg):
        robotstate_msg = RobotState()
        robotstate_msg.joint_state = jointstate_msg
        return robotstate_msg

    def get_plan_from_goal_config(self, goal_config, joint_names = COMBINED_ARM_JOINT_NAMES):
        # NOTE: this is really for collision checking, get planning_result.success to check
        commander = self._get_move_group_commander()
        
        start_config = self.get_joint_positions_map()
        start_config = np.array([start_config[joint] for joint in joint_names])
        
        jointstate_msg = self.joints_to_jointstate_msg(start_config)
        robotstate_msg = self.jointstate_to_robotstate_msg(jointstate_msg)
        commander.set_start_state(robotstate_msg)
        
        joint_config = dict(zip(self.get_arm_joints(), goal_config))
        commander.set_joint_value_target(joint_config)
        
        planning_result = PlanningResult(commander.plan())
        return planning_result

    def follow_plan(self, planning_result: PlanningResult):
        traj = planning_result.plan.joint_trajectory
        # joint_names = traj.joint_names
        # for point in traj.points:
        #     joints_i = point.positions
        #     self.goto_config(joints_i, joint_names)
        # TODO: Interpolate the velocities as well, i.e. do a trajectory_follower

        # execution_result = self.follow_arms_joint_trajectory_server(traj)
        execution_result = self.follow_arms_joint_trajectory(traj)

        # joint_names = traj.joint_names
        # joints_interpolated = interpolate_trajectory_joints(traj, num_steps=5)
        # for joint_i in joints_interpolated:
        #     self.goto_config(joint_i, joint_names)
        return False, ""

    def get_joints(self, joint_names=None):
        """
        :param joint_names: if provided, it will determine the order of the joints
        :return joint_values: np.array of shape (num_joints,)
        """
        if joint_names is None:
            joint_names = self.get_arm_joints()
        joint_values = np.asarray(self.get_joint_positions(joint_names=joint_names))
        return joint_values

    def set_joints(self, desried_joints, joint_names=None):
        """
        Plans to the desired joints and executes the plan
        :param desried_joints: (14,) list or np.array of desired joint positions for the robot. If joint_names is None, the order is given by COMBINED_ARM_JOINT_NAMES
        :return:
        """
        desried_joints = np.asarray(desried_joints)
        if joint_names is None:
            joint_names = self.get_arm_joints()
        result = self.get_plan_from_goal_config(desried_joints, joint_names)
        if result.success:
            self.follow_plan(result)
        return result

    def set_raw_joints(self, desried_joints, joint_names=None):
        """
        Sets the joints directly without planning
        ---------------------------------------------------------------------------------
        WARNING: This method does not check for collisions!!!! Use with caution!!!!!!!!!!!
        ---------------------------------------------------------------------------------
        :param desried_joints: (14,) list or np.array of desired joint positions for the robot. If joint_names is None, the order is given by COMBINED_ARM_JOINT_NAMES
        :return:
        """
        desried_joints = np.asarray(desried_joints)
        if joint_names is None:
            joint_names = self.get_arm_joints()

        positions, abort, msg = delegate_to_arms(positions=desried_joints.tolist(), joint_names=joint_names)
        if abort:
            return True, msg
        thanos_joints = positions['thanos']
        medusa_joints = positions['medusa']

        # Get the current control mode
        control_mode = self.get_control_modes()
        thanos_arm_control_mode = control_mode['thanos']
        medusa_arm_control_mode = control_mode['medusa']

        self.send_arm_command(self.thanos_arm_command_pub, thanos_arm_control_mode, thanos_joints)
        self.send_arm_command(self.medusa_arm_command_pub, medusa_arm_control_mode, medusa_joints)

    def set_poses(self, thanos_pose=None, medusa_pose=None, thanos_frame_id=None, medusa_frame_id=None, thanos_ref_frame=None, medusa_ref_frame=None):
        # solve the None parameters with default values
        if thanos_frame_id is None:
            thanos_frame_id = 'thanos_grasp_frame'
        if medusa_frame_id is None:
            medusa_frame_id = 'medusa_grasp_frame'
        if thanos_ref_frame is None:
            # thanos_ref_frame = f'{self.robot_namespace}_base'
            thanos_ref_frame = 'bimanual_base'
        if medusa_ref_frame is None:
            # medusa_ref_frame = f'{self.robot_namespace}_base'
            medusa_ref_frame = 'bimanual_base'
        # TODO: Fix for frames that are not ref_frame i.e. binamual_base
        # Solve IK:
        target_joints, error_code = self.compute_ik_combined_med(thanos_target_pose=thanos_pose,
                                                     medusa_target_pose=medusa_pose,
                                                     thanos_ik_link_name=thanos_frame_id,
                                                     medusa_ik_link_name=medusa_frame_id,
                                                     thanos_ref_frame=thanos_ref_frame,
                                                     medusa_ref_frame=medusa_ref_frame,
                                                     )
        # set the joints
        self.set_joints(target_joints) # This involves planning

    def set_poses_raw(self, thanos_pose=None, medusa_pose=None, thanos_frame_id=None, medusa_frame_id=None, thanos_ref_frame=None, medusa_ref_frame=None):
        # solve the None parameters with default values
        if thanos_frame_id is None:
            thanos_frame_id = 'thanos_grasp_frame'
        if medusa_frame_id is None:
            medusa_frame_id = 'medusa_grasp_frame'
        if thanos_ref_frame is None:
            # thanos_ref_frame = f'{self.robot_namespace}_base'
            thanos_ref_frame = 'bimanual_base'
        if medusa_ref_frame is None:
            # medusa_ref_frame = f'{self.robot_namespace}_base'
            medusa_ref_frame = 'bimanual_base'
        # Solve IK:
        target_joints, error_code = self.compute_ik_combined_med(thanos_target_pose=thanos_pose,
                                                     medusa_target_pose=medusa_pose,
                                                     thanos_ik_link_name=thanos_frame_id,
                                                     medusa_ik_link_name=medusa_frame_id,
                                                     thanos_ref_frame=thanos_ref_frame,
                                                     medusa_ref_frame=medusa_ref_frame,
                                                     )
        # set the joints
        self.set_raw_joints(target_joints)


def interpolate_trajectory_joints(traj, num_steps=1):
    all_points = np.array([point.positions for point in traj.points]) # (T, num_joints)
    # interpolate the trajectory
    interpolated_joints = []
    for i in range(len(all_points) - 1):
        start = all_points[i]
        end = all_points[i + 1]
        diff = end - start
        for j in range(num_steps):
            interpolated_joints.append(start + j * diff / num_steps)
    interpolated_joints.append(all_points[-1])
    interpolated_joints = np.stack(interpolated_joints, axis=0)
    return interpolated_joints

