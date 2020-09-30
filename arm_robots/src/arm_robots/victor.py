#! /usr/bin/env python
from enum import Enum

import actionlib
import rospy
from arm_robots.robot import ARMRobot
from control_msgs.msg import FollowJointTrajectoryAction
from victor_hardware_interface.victor_utils import get_joint_impedance_params, get_joint_position_params
from victor_hardware_interface_msgs.msg import ControlModeParameters, ControlMode
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode


class Stiffness(Enum):
    STIFF = 1
    MEDIUM = 3
    SOFT = 2


class Victor(ARMRobot):

    def __init__(self, execute_by_default: bool = False):
        super().__init__(execute_by_default=execute_by_default)
        self.robot_namespace = 'victor'
        rospy.loginfo("Victor ready!")
        self.left_set_control_mode_srv = rospy.ServiceProxy("/left_arm/set_control_mode_service", SetControlMode)
        self.right_set_control_mode_srv = rospy.ServiceProxy("/right_arm/set_control_mode_service", SetControlMode)
        self.left_get_control_mode_srv = rospy.ServiceProxy("/left_arm/get_control_mode_service", GetControlMode)
        self.right_get_control_mode_srv = rospy.ServiceProxy("/right_arm/get_control_mode_service", GetControlMode)

        name = self.robot_namespace + '/both_arms_trajectory_controller/follow_joint_trajectory'
        self.client = actionlib.SimpleActionClient(name, FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def set_control_mode(self, control_mode: ControlMode):
        self.set_left_arm_control_mode(control_mode)
        self.set_right_arm_control_mode(control_mode)

    def set_right_arm_control_mode(self, control_mode: ControlMode, stiffness=Stiffness.MEDIUM, vel=0.1, accel=0.1):
        new_control_mode = ControlModeParameters()

        if control_mode == ControlMode.JOINT_IMPEDANCE:
            new_control_mode = get_joint_position_params(vel, accel)
        elif control_mode == ControlMode.JOINT_IMPEDANCE:
            new_control_mode = get_joint_impedance_params(stiffness, vel, accel)

        # only change the contol mode itself
        new_control_mode.control_mode.mode = control_mode
        res = self.right_set_control_mode_srv(new_control_mode)
        if not res.success:
            rospy.logerr("Failed to switch right arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)

    def set_left_arm_control_mode(self, control_mode: ControlMode, stiffness=Stiffness.MEDIUM, vel=0.1, accel=0.1):
        new_control_mode = ControlModeParameters()

        if control_mode == ControlMode.JOINT_IMPEDANCE:
            new_control_mode = get_joint_position_params(vel, accel)
        elif control_mode == ControlMode.JOINT_IMPEDANCE:
            new_control_mode = get_joint_impedance_params(stiffness, vel, accel)

        new_control_mode.control_mode.mode = control_mode
        res = self.left_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch left arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)


if __name__ == "__main__":
    rospy.init_node("motion_enabled_victor")
    mev = Victor()
    rospy.spin()
