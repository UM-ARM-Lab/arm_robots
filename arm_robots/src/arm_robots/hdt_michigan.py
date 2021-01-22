#! /usr/bin/env python
from threading import Thread
from typing import List, Dict

import numpy as np
import rospy
from colorama import Fore
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_robots.base_robot import DualArmRobot
from arm_robots.robot import MoveitEnabledRobot


class BaseVal(DualArmRobot):

    def __init__(self, robot_namespace: str):
        DualArmRobot.__init__(self, robot_namespace=robot_namespace)
        self.first_valid_command = False
        self.min_velocities = {
            'joint1':  0.05,
            'joint2':  0.05,
            'joint3':  0.05,
            'joint4':  0.05,
            'joint41': 0.05,
            'joint42': 0.05,
            'joint43': 0.05,
            'joint44': 0.05,
            'joint45': 0.05,
            'joint46': 0.05,
            'joint47': 0.05,
            'joint5':  0.05,
            'joint56': 0.05,
            'joint57': 0.05,
            'joint6':  0.05,
            'joint7':  0.05,
        }
        self.command_thread = Thread(target=self.command_thread_func)

        self.command_pub = rospy.Publisher("/hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
        # the initialized velocities will be zero, so we need not worry about it accidentally moving on startup
        self.latest_cmd = JointState()
        self.command_rate = rospy.Rate(100)
        self.ready = 0

        self.command_thread.start()

    def command_thread_func(self):
        while not self.first_valid_command:
            rospy.sleep(0.1)
        while True:
            # actually send commands periodically
            self.latest_cmd.header.stamp = rospy.Time.now()
            rospy.logdebug_throttle(1, self.latest_cmd)
            self.command_pub.publish(self.latest_cmd)
            self.command_rate.sleep()

    def threshold_velocities(self, joint_names, velocities):
        fixed_velocities = []
        for j, v in zip(joint_names, velocities):
            if v > 0:
                fixed_velocity = max(self.min_velocities[j], v)
            else:
                # the HDT robot only uses positive velocities (absolute value)
                # because the HDT driver code determines direction
                fixed_velocity = -min(-self.min_velocities[j], v)

            fixed_velocities.append(fixed_velocity)

        return fixed_velocities

    def send_joint_command(self, joint_names: List[str], trajectory_point: JointTrajectoryPoint):
        current_joint_command = self.get_joint_positions(joint_names)
        distance = np.linalg.norm(np.array(current_joint_command) - np.array(trajectory_point.positions))
        if distance > 1:
            rospy.logerr("Sending a huge joint command! I'm forcing velocities to zero to prevent this.")
            trajectory_point.velocities = [0] * len(joint_names)
        elif not self.first_valid_command:
            self.first_valid_command = True

        self.latest_cmd.name = joint_names
        self.latest_cmd.position = trajectory_point.positions
        self.latest_cmd.velocity = self.threshold_velocities(joint_names, trajectory_point.velocities)
        # self.latest_cmd.velocity = trajectory_point.velocities
        self.latest_cmd.effort = [0] * len(joint_names)

        # TODO: check the status of the robot and report errors here
        failed = False
        error_msg = ""
        return failed, error_msg


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
        move_group.set_joint_value_target({'leftgripper':  position,
                                           'leftgripper2': position, })
        plan = move_group.plan()[1]
        self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def set_right_gripper(self, position):
        move_group = self.get_move_group_commander('right_gripper')
        move_group.set_joint_value_target({'rightgripper':  position,
                                           'rightgripper2': position, })
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
