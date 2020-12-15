#! /usr/bin/env python
import threading
from threading import Thread
from typing import List, Tuple

import rospy
from colorama import Fore
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_robots.base_robot import DualArmRobot
from arm_robots.robot import MoveitEnabledRobot

left_arm_joints = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
    'joint7',
]

right_arm_joints = [
    'joint41',
    'joint42',
    'joint43',
    'joint44',
    'joint45',
    'joint46',
    'joint47',
]

torso_joints = [
    'joint56',
    'joint57',
]

left_gripper_joints = [
    'leftgripper',
    'leftgripper2',
]

right_gripper_joints = [
    'rightgripper',
    'rightgripper2',
]

all_joints = left_arm_joints + left_gripper_joints + right_arm_joints + right_gripper_joints + torso_joints

n_joints = len(all_joints)


class BaseVal(DualArmRobot):
    min_joint_velocity = 0.1

    def __init__(self, robot_namespace: str):
        DualArmRobot.__init__(self, robot_namespace=robot_namespace)

        self.command_pub = rospy.Publisher("hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
        self.latest_command = JointState(name=all_joints, position=[0] * n_joints)

        self.command_lock = threading.RLock()
        self.thread = Thread(target=self.publish_commands_constantly)
        self.thread.start()

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint) -> Tuple[bool, str]:
        current_joint_positions = self.get_joint_positions(all_joints)

        all_positions = []
        all_velocities = []
        for joint_name, current_joint_position in zip(all_joints, current_joint_positions):
            if joint_name in joint_names:
                i = joint_names.index(joint_name)
                all_positions.append(trajectory_point.positions[i])
                all_velocities.append(trajectory_point.velocities[i])
            else:
                all_positions.append(current_joint_position)
                # any joints not in the current command should not move
                all_velocities.append(0)

        self.latest_command.position = all_positions
        self.latest_command.velocity = all_velocities
        self.latest_command.header.stamp = rospy.Time.now()
        err_msg = ""
        abort = False
        return abort, err_msg

    def publish_commands_constantly(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            latest_command_not_too_old = rospy.Time.now() - self.latest_command.header.stamp < rospy.Duration(10)
            if self.latest_command is not None and latest_command_not_too_old:
                with self.command_lock:
                    self.latest_command = self.fix_velocity_command(self.latest_command)
                    self.command_pub.publish(self.latest_command)
                rate.sleep()

    @staticmethod
    def fix_velocity_command(command: JointState):
        for i, v in enumerate(command.velocity):
            if v > 0:
                command.velocity[i] = max(v, BaseVal.min_joint_velocity)
        return command


class Val(BaseVal, MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'val'):
        MoveitEnabledRobot.__init__(self,
                                    robot_namespace=robot_namespace,
                                    arms_controller_name='both_arms_trajectory_controller')
        BaseVal.__init__(self, robot_namespace=robot_namespace)
        rospy.loginfo(Fore.GREEN + "Val ready!")

    def get_right_gripper_joints(self):
        return right_gripper_joints

    def get_left_gripper_joints(self):
        return left_gripper_joints

    def get_gripper_closed_positions(self):
        return [0, 0]

    def get_gripper_open_positions(self):
        return [0.5, 0.5]

    def get_right_arm_joints(self):
        return right_arm_joints

    def get_left_arm_joints(self):
        return left_arm_joints
