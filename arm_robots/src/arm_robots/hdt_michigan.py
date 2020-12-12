#! /usr/bin/env python
from threading import Thread
from typing import List, Tuple

from colorama import Fore

import rospy
from sensor_msgs.msg import JointState

from arm_robots.base_robot import DualArmRobot
from arm_robots.robot import MoveitEnabledRobot
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseVal(DualArmRobot):

    def __init__(self, robot_namespace: str):
        DualArmRobot.__init__(self, robot_namespace=robot_namespace)
        rospy.loginfo("Val ready !")

        self.command_pub = rospy.Publisher("hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
        self.latest_command = JointState()

        self.thread = Thread(target=self.publish_commands_constantly)
        self.thread.start()

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        command_msg = JointState(name=joint_names, position=trajectory_point.positions)
        command_msg.header.stamp = rospy.Time.now()
        self.latest_command = command_msg
        err_msg = ""
        abort = False
        return abort, err_msg

    def publish_commands_constantly(self):
        while not rospy.is_shutdown():
            latest_command_isnt_too_old = rospy.Time.now() - self.latest_command.header.stamp < rospy.Duration(10)
            if self.latest_command is not None and latest_command_isnt_too_old:
                self.command_pub.publish(self.latest_command)


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
    def __init__(self, robot_namespace: str = 'val'):
        self.base_val = BaseVal(robot_namespace)
        super().__init__(robot_namespace=robot_namespace,
                         arms_controller_name='both_arms_trajectory_controller',
                         left_gripper_controller_name='left_gripper_controller',
                         right_gripper_controller_name='right_gripper_controller',
                         )
        rospy.loginfo(Fore.GREEN + "Val ready!")

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
