#! /usr/bin/env python
from typing import List, Tuple

from colorama import Fore

import rospy
from arm_robots.base_robot import BaseRobot
from arm_robots.robot import MoveitEnabledRobot
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseVal(BaseRobot):

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        pass

    def __init__(self, robot_namespace: str):
        super().__init__(robot_namespace=robot_namespace)
        rospy.loginfo("Val ready !")


def make_joint_group_command(command):
    msg = Float64MultiArray()
    msg.data = command
    return msg


class Val(MoveitEnabledRobot):
    def __init__(self, robot_namespace: str = 'val'):
        self.base_val = BaseVal(robot_namespace)
        super().__init__(self.base_val,
                         arms_controller_name='both_arms_trajectory_controller',
                         left_gripper_controller_name='left_gripper_controller',
                         right_gripper_controller_name='right_gripper_controller',
                         )
        rospy.loginfo(Fore.GREEN + "Val ready!")

    def get_joint_trajectory_controller_name(self):
        """ This should match the *.yaml file, and you can also run rqt_controller_manager to check the names """
        return "both_arms_trajectory_controller"

    def get_right_gripper_joints(self):
        return ['rightgripper', 'rightgripper2']

    def get_left_gripper_joints(self):
        return ['leftgripper', 'leftgripper2']

    def get_gripper_closed_positions(self):
        return [0, 0]

    def get_gripper_open_positions(self):
        return [0.5, 0.5]
