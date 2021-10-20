#! /usr/bin/env python
import weakref
from copy import deepcopy
from threading import Thread
from time import sleep
from typing import List

import numpy as np
from colorama import Fore

import rospy
from arm_robots.base_robot import BaseRobot
from arm_robots.robot import MoveitEnabledRobot
from arm_robots.robot_utils import robot_state_from_joint_state_and_joint_names
from geometry_msgs.msg import Wrench
from moveit_msgs.msg import RobotState
from rosgraph.names import ns_join
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class BaseVal(BaseRobot):

    def __init__(self, robot_namespace: str, robot_description: str = None):
        if robot_description is None:
            robot_description = ns_join(robot_namespace, 'robot_description')
        BaseRobot.__init__(self, robot_namespace=robot_namespace, robot_description=robot_description)
        self.first_valid_command = False
        self.should_disconnect = False
        self.min_velocity = 0.05
        self.command_thread = Thread(target=BaseVal.command_thread_func, args=(weakref.proxy(self),))

        self.command_pub = rospy.Publisher("/hdt_adroit_coms/joint_cmd", JointState, queue_size=10)
        # the initialized velocities will be zero, so we need not worry about it accidentally moving on startup
        self.latest_cmd = JointState()
        self.command_rate = rospy.Rate(100)
        self.ready = 0
        self.has_started_command_thread = False
        self._max_velocity_scale_factor = 1.0

    def __del__(self):
        self.disconnect()

    def connect(self):
        super().connect()
        if rospy.get_param("use_sim_time", False):
            rospy.loginfo("Simulation detected, no command thread will be started.")
        elif not self.has_started_command_thread:
            rospy.loginfo('Starting val command thread')
            self.has_started_command_thread = True
            self.command_thread.start()

    def disconnect(self):
        super().disconnect()

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
                time_since_last_command = now - self.latest_cmd.header.stamp
                if time_since_last_command < rospy.Duration(secs=1):
                    command_to_send = deepcopy(self.latest_cmd)
                    command_to_send.header.stamp = now
                    self.command_pub.publish(command_to_send)
                else:
                    rospy.logdebug_throttle(1, "latest command is too old, ignoring", logger_name="hdt_michigan")
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

    def get_right_gripper_links(self):
        return self.robot_commander.get_link_names("right_gripper")

    def get_left_gripper_links(self):
        return self.robot_commander.get_link_names("left_gripper")


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
                                    robot_description=ns_join(robot_namespace, 'robot_description'),
                                    arms_controller_name='both_arms_trajectory_controller',
                                    **kwargs)
        BaseVal.__init__(self, robot_namespace=robot_namespace,
                         robot_description=ns_join(robot_namespace, 'robot_description'))
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

    def get_both_arm_joints(self):
        return self.get_left_arm_joints() + self.get_right_arm_joints()

    def get_right_arm_joints(self):
        return right_arm_joints

    def get_left_arm_joints(self):
        return left_arm_joints

    def get_gripper_positions(self):
        # NOTE: this function requires that gazebo be playing
        return self.get_link_pose(self.left_tool_name).position, self.get_link_pose(self.right_tool_name).position

    def connect(self, **kwargs):
        super().connect(**kwargs)
        rospy.loginfo(Fore.GREEN + "Val ready!")

    def is_gripper_closed(self, gripper: str):
        if gripper == 'left':
            move_group = self.get_move_group_commander('left_gripper')
        elif gripper == 'right':
            move_group = self.get_move_group_commander('right_gripper')
        else:
            raise NotImplementedError(f"invalid gripper {gripper}")
        current_joint_values = move_group.get_current_joint_values()
        return np.allclose(current_joint_values, self.gripper_closed_position, atol=0.01)

    def is_left_gripper_closed(self):
        return self.is_gripper_closed('left')

    def is_right_gripper_closed(self):
        return self.is_gripper_closed('right')


def estimated_torques(joint_state: JointState, robot: Val):
    group_name1 = "left_side"
    group_name2 = "right_side"
    n_links = 14
    wrenches = [Wrench()] * n_links

    joint_names1 = robot.robot_commander.get_active_joint_names(group_name1)
    robot_state1 = robot_state_from_joint_state_and_joint_names(joint_names1, joint_state)
    torques1 = robot.estimated_torques(robot_state1, group_name1, wrenches)

    joint_names2 = robot.robot_commander.get_active_joint_names(group_name2)
    robot_state2 = robot_state_from_joint_state_and_joint_names(joint_names2, joint_state)
    torques2 = robot.estimated_torques(robot_state2, group_name2, wrenches)

    if torques1 is None or torques2 is None:
        return

    robot_state = RobotState()
    names = sorted(set(robot_state1.joint_state.name + robot_state2.joint_state.name))
    for name in names:
        p = None
        e = None
        try:
            i = robot_state1.joint_state.name.index(name)
            p = robot_state1.joint_state.position[i]
            e = torques1[i]
        except ValueError:
            pass
        try:
            i = robot_state2.joint_state.name.index(name)
            p = robot_state2.joint_state.position[i]
            e = torques2[i]
        except ValueError:
            pass
        robot_state.joint_state.name.append(name)
        robot_state.joint_state.position.append(p)
        robot_state.joint_state.effort.append(e)

    return robot_state
