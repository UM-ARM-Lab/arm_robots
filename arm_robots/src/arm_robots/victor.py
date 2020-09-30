#! /usr/bin/env python

import actionlib
import rospy
from arc_utilities.ros_helpers import Listener, prepend_namespace
from arm_robots.robot import ARMRobot
from control_msgs.msg import FollowJointTrajectoryAction
from victor_hardware_interface.victor_utils import get_new_control_mode
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus
from victor_hardware_interface_msgs.srv import SetControlMode, GetControlMode, GetControlModeRequest, \
    GetControlModeResponse

left_impedance_switch_config = [-0.694, 0.14, -0.229, -1.11, -0.512, 1.272, 0.077]
right_impedance_switch_config = [0.724, 0.451, 0.94, -1.425, 0.472, 0.777, -0.809]


class Victor(ARMRobot):

    def __init__(self, execute_by_default: bool = False):
        super().__init__(execute_by_default=execute_by_default)
        self.robot_namespace = '/victor'

        self.left_set_control_mode_srv = rospy.ServiceProxy("/left_arm/set_control_mode_service", SetControlMode)
        self.right_set_control_mode_srv = rospy.ServiceProxy("/right_arm/set_control_mode_service", SetControlMode)

        self.left_get_control_mode_srv = rospy.ServiceProxy("/left_arm/get_control_mode_service", GetControlMode)
        self.right_get_control_mode_srv = rospy.ServiceProxy("/right_arm/get_control_mode_service", GetControlMode)

        self.left_motion_status_listener = Listener("/left_arm/motion_status", MotionStatus)
        self.right_motion_status_listener = Listener("/right_arm/motion_status", MotionStatus)

        # TODO: don't hard-code this
        name = prepend_namespace(self.robot_namespace, 'both_arms_trajectory_controller/follow_joint_trajectory')
        self.client = actionlib.SimpleActionClient(name, FollowJointTrajectoryAction)
        rospy.loginfo("Victor ready!")

    def get_motion_status(self):
        return {'left': self.left_motion_status_listener.get(), 'right': self.right_motion_status_listener.get()}

    def get_right_motion_status(self):
        right_status = self.right_motion_status_listener.get()
        return right_status

    def get_left_motion_status(self):
        left_status = self.left_motion_status_listener.get()
        return left_status

    def get_control_mode(self):
        return {'left': self.get_left_arm_control_mode(), 'right': self.get_right_arm_control_mode()}

    def set_control_mode(self, control_mode: ControlMode, **kwargs):
        self.set_left_arm_control_mode(control_mode, **kwargs)
        self.set_right_arm_control_mode(control_mode, **kwargs)

    def get_left_arm_control_mode(self):
        left_control_mode_res: GetControlModeResponse = self.left_get_control_mode_srv(GetControlModeRequest())
        return left_control_mode_res.active_control_mode.control_mode

    def get_right_arm_control_mode(self):
        right_control_mode_res: GetControlModeResponse = self.right_get_control_mode_srv(GetControlModeRequest())
        return right_control_mode_res.active_control_mode.control_mode

    def set_right_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_new_control_mode(control_mode, **kwargs)
        res = self.right_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch right arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)

    def set_left_arm_control_mode(self, control_mode: ControlMode, **kwargs):
        new_control_mode = get_new_control_mode(control_mode, **kwargs)
        res = self.left_set_control_mode_srv(new_control_mode)

        if not res.success:
            rospy.logerr("Failed to switch left arm to control mode: " + str(control_mode))
            rospy.logerr(res.message)

    def move_to_impedance_switch(self, actually_switch: bool = True):
        self.plan_to_joint_config("left_arm", left_impedance_switch_config)
        self.plan_to_joint_config("right_arm", right_impedance_switch_config)
        if actually_switch:
            self.set_control_mode(ControlMode.JOINT_IMPEDANCE)


if __name__ == "__main__":
    rospy.init_node("motion_enabled_victor")
    mev = Victor()
    rospy.spin()
