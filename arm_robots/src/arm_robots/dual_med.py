#! /usr/bin/env python
from typing import Tuple, Sequence

import numpy as np
from colorama import Fore
from typing import List, Dict, Tuple, Sequence
import rospy
from arc_utilities.conversions import convert_to_pose_msg
from arc_utilities.listener import Listener
from arm_robots.base_robot import BaseRobot
from arm_robots.config.med_config import ARM_JOINT_NAMES, COMBINED_ARM_JOINT_NAMES
from trajectory_msgs.msg import JointTrajectoryPoint
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
from moveit_msgs.msg import RobotState, PositionIKRequest
from arm_robots.robot_utils import PlanningResult
from moveit_msgs.srv import GetPositionIK
from arc_utilities.transformation_helper import PoseFromComponents
import copy
from pyjacobian_follower import IkParams

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


class DualMed(BaseRobot):
    def __init__(self, robot_namespace: str = 'combined_med', force_trigger: float = -0.0, base_kwargs=None, **kwargs):
        BaseRobot.__init__(self, robot_namespace=robot_namespace)
        
        thanos_prefix = os.path.join('/', 'med')
        medusa_prefix = os.path.join('/', 'medusa')
        self.thanos_arm_command_pub = rospy.Publisher(os.path.join(thanos_prefix, 'motion_command'), MotionCommand, queue_size=10)
        self.medusa_arm_command_pub = rospy.Publisher(os.path.join(medusa_prefix, 'motion_command'), MotionCommand, queue_size=10)
        
        self.thanos_arm_status_listener = Listener(os.path.join(thanos_prefix, 'motion_status'), MotionStatus)
        self.medusa_arm_status_listener = Listener(os.path.join(medusa_prefix, 'motion_status'), MotionStatus)
        
        self.thanos_set_control_mode_srv = rospy.ServiceProxy(os.path.join(thanos_prefix, 'set_control_mode_service'),
                                                            SetControlMode)
        self.medusa_set_control_mode_srv = rospy.ServiceProxy(os.path.join(medusa_prefix, 'set_control_mode_service'),
                                                            SetControlMode)
        
        self.thanos_get_control_mode_srv = rospy.ServiceProxy(os.path.join(thanos_prefix, 'get_control_mode_service'),
                                                            GetControlMode)
        self.medusa_get_control_mode_srv = rospy.ServiceProxy(os.path.join(medusa_prefix, 'get_control_mode_service'),
                                                            GetControlMode)
        self.ik_proxy = self._init_ik_client()
        
        
    def _init_ik_client(self):
        service_name = 'combined_med/compute_ik'
        rospy.wait_for_service(service_name)
        try:
            ik_proxy = rospy.ServiceProxy(service_name, GetPositionIK)
            return ik_proxy
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    def _call_ik_solver(self, srv_input):
        try:
            ik_resp = self.ik_proxy(srv_input)
            return ik_resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    def compute_ik(self, target_pose, group_name=None, ee_link_name='grasp_frame', ref_frame='bimanual_base', init_state=None):
        if group_name is None:
            group_name = self.arm_group
        # call the ik service:
        move_group = self.get_move_group_commander(group_name=group_name)
        ik_request = PositionIKRequest()
        ik_request.group_name = group_name  # string
        if init_state is None:
            ik_request.robot_state = move_group.get_current_state()
        else:
            ik_request.robot_state = init_state
        target_pose_stamped = convert_to_pose_msg(target_pose)
        target_pose_stamped.header.frame_id = ref_frame
        ik_request.pose_stamped = target_pose_stamped
        ik_request.avoid_collisions = True
        ik_request.ik_link_name = ee_link_name
        ik_out = self._call_ik_solver(ik_request)
        ik_solution = ik_out.solution
        error_code = ik_out.error_code
        robot_joints = np.asarray(ik_solution.joint_state.position)[:7] # TODO: make this more general to get rid of the non-robot joints
        return robot_joints, error_code
    def compute_ik_fast(self, target_pose, group_name=None, ee_link_name='grasp_frame', ref_frame='bimanual_base', init_joints=None):
        """
        This method uses collision free ik. It is faster than compute_ik (by orders of mangitude).
        :param target_pose:
        :param group_name:
        :param ee_link_name:
        :param ref_frame:
        :return:
        """
        # TESTING MODE ---------------------- Do not use in general
        if group_name is None:
            group_name = self.arm_group
        move_group = self.get_move_group_commander(group_name=group_name)
        scene_msg = self.scene_listener.get()
        # transform the pose to be with respect to the end effector.
        target_pose_msg = PoseFromComponents(target_pose[:3], target_pose[3:])
        target_pose_msg.header.frame_id = ref_frame
        robot_state = move_group.get_current_state()
        current_joints = copy.deepcopy(robot_state.joint_state.position[:7])
        if init_joints is not None:
            assert len(init_joints) == 7, 'must be the 7 values of the 7 DoF KUKA MED'
            robot_state.joint_state.position = tuple(init_joints) + robot_state.joint_state.position[7:]
        ik_params = IkParams(rng_dist=0.001, max_collision_check_attempts=20)
        robot_state_ik = self.jacobian_follower.compute_collision_free_pose_ik(robot_state, [target_pose_msg], group_name, tip_names=[ee_link_name], scene_msg=scene_msg, ik_params=ik_params)
        # IkParams: {rng_dist=0.1, max_collision_check_attempts=100}
        joint_solution = robot_state_ik.joint_state.position[:7] # TODO: make this more general to get rid of the non-robot joints
        joint_solution = np.asarray(joint_solution)
        return joint_solution
    def get_current_pose_thanos(self, frame_id='grasp_frame', ref_frame='bimanual_base'):
        pass
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
        commander = moveit_commander.MoveGroupCommander('combined_med', ns='combined_med', robot_description=rospy.resolve_name('robot_description'))
        
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
        joint_names = traj.joint_names
        for point in traj.points:
            joints_i = point.positions
            self.goto_config(joints_i, joint_names)
        return False, ""