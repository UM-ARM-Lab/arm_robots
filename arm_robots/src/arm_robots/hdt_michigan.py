#! /usr/bin/env python
import weakref
from threading import Thread
from time import sleep
from typing import List

import numpy as np
from colorama import Fore

import rospy
from arm_robots.base_robot import DualArmRobot
from arm_robots.robot import MoveitEnabledRobot
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseVal(DualArmRobot):

    def __init__(self, robot_namespace: str):
        DualArmRobot.__init__(self, robot_namespace=robot_namespace)
        self.first_valid_command = False
        self.should_disconnect = False
        self.min_velocity = 0.05
        self.command_thread = Thread(target=BaseVal.command_thread_func, args=(weakref.proxy(self),))

        self.command_pub = rospy.Publisher("/hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
        # the initialized velocities will be zero, so we need not worry about it accidentally moving on startup
        self.latest_cmd = JointState()
        self.command_rate = rospy.Rate(100)
        self.ready = 0

    def __del__(self):
        self.disconnect()

    def connect(self):
        super().connect()
        self.command_thread.start()

    def disconnect(self):
        self.should_disconnect = True
        if self.command_thread.is_alive():
            self.command_thread.join()

    def command_thread_func(self):
        try:
            while not self.first_valid_command:
                if self.should_disconnect:
                    break
                sleep(0.1)

            while True:
                if self.should_disconnect:
                    break
                # actually send commands periodically
                now = rospy.Time.now()
                if (now - self.latest_cmd.header.stamp) < rospy.Duration(secs=1):
                    self.latest_cmd.header.stamp = now
                    self.command_pub.publish(self.latest_cmd)
                else:
                    rospy.logdebug_throttle(1, "latest command is too old, ignoring")
                self.command_rate.sleep()
        except ReferenceError:
            pass

    def threshold_velocities(self, joint_names, velocities):
        fixed_velocities = []
        for j, v in zip(joint_names, velocities):
            if v > 0:
                fixed_velocity = max(self.min_velocity, v)
            else:
                # the HDT robot only uses positive velocities (absolute value)
                # because the HDT driver code determines direction
                fixed_velocity = -min(-self.min_velocity, v)

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
        self.latest_cmd.effort = [0] * len(joint_names)
        self.latest_cmd.header.stamp = rospy.Time.now()

        # TODO: check the status of the robot and report errors here
        failed = False
        error_msg = ""
        return failed, error_msg


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
        move_group = self.move_groups['left_gripper']
        move_group.set_joint_value_target({'leftgripper':  position,
                                           'leftgripper2': position, })
        plan = move_group.plan()[1]
        self.follow_arms_joint_trajectory(plan.joint_trajectory)

    def set_right_gripper(self, position):
        move_group = self.move_groups['right_gripper']
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
        self.command_thread.start()
        rospy.loginfo(Fore.GREEN + "Val ready!")

    def is_gripper_closed(self, gripper: str):
        if gripper == 'left':
            move_group = self.move_groups['left_gripper']
        elif gripper == 'right':
            move_group = self.move_groups['right_gripper']
        else:
            raise NotImplementedError(f"invalid gripper {gripper}")
        current_joint_values = move_group.get_current_joint_values()
        return np.allclose(current_joint_values, self.gripper_closed_position, atol=0.01)
