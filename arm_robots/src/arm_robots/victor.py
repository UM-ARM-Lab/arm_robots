#! /usr/bin/env python

import rospy
from arm_robots.robot import ARMRobot
from victor_hardware_interface_msgs.msg import ControlModeParameters, ControlMode
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode


class Victor(ARMRobot):

    def __init__(self, execute_by_default: bool = False):
        super().__init__(execute_by_default=execute_by_default)
        self.robot_namespace = 'victor'
        rospy.loginfo("Victor ready!")
        self.left_set_control_mode_srv = rospy.ServiceProxy("/left_arm/set_control_mode_service", SetControlMode)
        self.right_set_control_mode_srv = rospy.ServiceProxy("/right_arm/set_control_mode_service", SetControlMode)
        self.left_get_control_mode_srv = rospy.ServiceProxy("/left_arm/get_control_mode_service", GetControlMode)
        self.right_get_control_mode_srv = rospy.ServiceProxy("/right_arm/get_control_mode_service", GetControlMode)

    def set_control_mode(self, control_mode: ControlMode):
        self.set_left_arm_control_mode(control_mode)
        self.set_right_arm_control_mode(control_mode)

    def set_right_arm_control_mode(self, control_mode: ControlMode):
        new_control_mode = ControlModeParameters()
        new_control_mode.control_mode.mode = control_mode
        res = self.right_set_control_mode_srv(new_control_mode)
        if not res.success:
            rospy.logerr("Failed to switch right arm to control mode: " + str(control_mode))

    def set_left_arm_control_mode(self, control_mode: ControlMode):
        new_control_mode = ControlModeParameters()
        new_control_mode.control_mode.mode = control_mode
        res = self.left_set_control_mode_srv(new_control_mode)
        if not res.success:
            rospy.logerr("Failed to switch left arm to control mode: " + str(control_mode))


if __name__ == "__main__":
    rospy.init_node("motion_enabled_victor")
    mev = Victor()
    rospy.spin()
