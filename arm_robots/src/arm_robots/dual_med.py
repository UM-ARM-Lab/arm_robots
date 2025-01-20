#! /usr/bin/env python
from typing import Tuple, Sequence

import numpy as np
from colorama import Fore
from typing import List, Dict, Tuple, Sequence
import rospy
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
        # self.create_cartesian_impedance_controller([self.thanos_arm_status_listener, self.medusa_arm_status_listener],
        #                                                 [self.thanos_arm_command_pub, self.medusa_arm_command_pub],
        #                                                 COMBINED_ARM_JOINT_NAMES, "bimanual_base", **kwargs)
        
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
    
    def reached_endpoint(self, target, joint_names=COMBINED_ARM_JOINT_NAMES, tol=1e-3):
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
    
    def goto_config(self, joint_positions, joint_names=COMBINED_ARM_JOINT_NAMES, control_mode=ControlMode.JOINT_POSITION, **kwargs):
        traj_point = JointTrajectoryPoint()
        traj_point.positions = joint_positions
        traj_point.velocities = np.zeros(len(joint_positions))
        
        
        positions, abort, msg = delegate_to_arms(traj_point.positions, joint_names)
        # if abort:
        #     return True, msg
        
        # self.set_thanos_arm_control_mode(control_mode, **kwargs)
        # self.set_medusa_arm_control_mode(control_mode, **kwargs)
        self.send_joint_command(joint_names, traj_point)
        while not self.reached_endpoint(joint_positions):
            time.sleep(1e-5)
        
        return False, ""