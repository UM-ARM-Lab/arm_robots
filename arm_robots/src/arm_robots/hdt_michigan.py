#! /usr/bin/env python
from typing import List, Tuple

from colorama import Fore

import rospy
from arm_robots.base_robot import DualArmRobot
from arm_robots.robot import MoveitEnabledRobot
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseVal(DualArmRobot):

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        pass

    def __init__(self, robot_namespace: str):
        DualArmRobot.__init__(self, robot_namespace=robot_namespace)


def make_joint_group_command(command):
    msg = Float64MultiArray()
    msg.data = command
    return msg

left_arm_joints = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
    'joint_7',
]

right_arm_joints = [
    'joint_41',
    'joint_42',
    'joint_43',
    'joint_44',
    'joint_45',
    'joint_46',
    'joint_47',
]


class Val(BaseVal, MoveitEnabledRobot):
    def __init__(self, robot_namespace: str = 'hdt_michigan'):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    arms_controller_name='both_arms_trajectory_controller')
        BaseVal.__init__(self, robot_namespace=robot_namespace)
        self.left_arm_group = 'left_arm'
        self.right_arm_group = 'right_arm'
        self.left_tool_name = 'left_tool'
        self.right_tool_name = 'right_tool'

    def get_right_gripper_joints(self):
        return ['rightgripper', 'rightgripper2']

    def get_left_gripper_joints(self):
        return ['leftgripper', 'leftgripper2']

    def get_gripper_closed_positions(self):
        return [0, 0]

    def get_gripper_open_positions(self):
        return [0.5, 0.5]

    def get_right_arm_joints(self):
        return right_arm_joints

    def get_left_arm_joints(self):
        return left_arm_joints

    def connect(self):
        super().connect()
        rospy.loginfo(Fore.GREEN + "Val ready!")
