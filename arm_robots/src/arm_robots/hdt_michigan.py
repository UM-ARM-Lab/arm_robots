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
    gripper_open_position = 0.5
    gripper_closed_position = 0.2

    def __init__(self, robot_namespace: str = 'hdt_michigan', **kwargs):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    arms_controller_name='both_arms_trajectory_controller',
                                    **kwargs)
        BaseVal.__init__(self, robot_namespace=robot_namespace)
        self.max_velocity_scale_factor = 1.0
        self.left_arm_group = 'left_arm'
        self.right_arm_group = 'right_arm'
        self.left_tool_name = 'left_tool'
        self.right_tool_name = 'right_tool'

    def get_right_gripper_joints(self):
        return ['rightgripper', 'rightgripper2']

    def get_left_gripper_joints(self):
        return ['leftgripper', 'leftgripper2']

    def set_left_gripper(self, position):
        move_group = self.get_move_group_commander('left_gripper')
        move_group.set_joint_value_target({'leftgripper_joint':  position,
                                           'leftgripper2_joint': position, })
        plan = move_group.plan()[1]
        self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def set_right_gripper(self, position):
        move_group = self.get_move_group_commander('right_gripper')
        move_group.set_joint_value_target({'rightgripper_joint':  position,
                                           'rightgripper2_joint': position, })
        plan = move_group.plan()[1]
        return self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def open_left_gripper(self):
        return self.set_left_gripper(self.gripper_open_position)

    def open_right_gripper(self):
        return self.set_right_gripper(self.gripper_open_position)

    def close_left_gripper(self):
        return self.set_left_gripper(self.gripper_closed_position)

    def close_right_gripper(self):
        return self.set_right_gripper(self.gripper_closed_position)

    def get_right_arm_joints(self):
        return right_arm_joints

    def get_left_arm_joints(self):
        return left_arm_joints

    def connect(self):
        super().connect()
        rospy.loginfo(Fore.GREEN + "Val ready!")
