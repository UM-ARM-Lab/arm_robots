#! /usr/bin/env python
from typing import List

import numpy as np

import actionlib
import rospy
from arc_utilities.ros_helpers import Listener
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, \
    FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import list_to_jvq
from victor_hardware_interface_msgs.msg import MotionCommand, MotionStatus, ControlMode


def waypoint_reached(actual: JointTrajectoryPoint, desired: JointTrajectoryPoint, tolerance: List[float]):
    actual = np.array(actual.positions)
    desired = np.array(desired.positions)
    tolerance = np.array(tolerance)
    return np.all((actual - desired) < tolerance)


joint_map = {
    'victor_left_arm_joint_1': 'joint_1',
    'victor_left_arm_joint_2': 'joint_2',
    'victor_left_arm_joint_3': 'joint_3',
    'victor_left_arm_joint_4': 'joint_4',
    'victor_left_arm_joint_5': 'joint_5',
    'victor_left_arm_joint_6': 'joint_6',
    'victor_left_arm_joint_7': 'joint_7',
    'victor_right_arm_joint_1': 'joint_1',
    'victor_right_arm_joint_2': 'joint_2',
    'victor_right_arm_joint_3': 'joint_3',
    'victor_right_arm_joint_4': 'joint_4',
    'victor_right_arm_joint_5': 'joint_5',
    'victor_right_arm_joint_6': 'joint_6',
    'victor_right_arm_joint_7': 'joint_7',
}


# TODO: make this work with val too
class TrajectoryForwarder(object):
    def __init__(self):
        self._action_name = "follow_joint_trajectory"
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self.left_arm_motion_command_pub = rospy.Publisher("left_arm/motion_command", MotionCommand, queue_size=10)
        self.right_arm_motion_command_pub = rospy.Publisher("right_arm/motion_command", MotionCommand, queue_size=10)
        self.left_arm_motion_command_listener = Listener("left_arm/motion_status", MotionStatus)
        self.right_arm_motion_command_listener = Listener("right_arm/motion_status", MotionStatus)
        self._as.start()

    def execute_cb(self, traj_msg: FollowJointTrajectoryGoal):
        trajectory_point_idx = 0

        tolerance = []
        for name in traj_msg.trajectory.joint_names:
            tolerance_position = 0.01
            for tolerance_for_name in traj_msg.path_tolerance:
                if tolerance_for_name.name == name:
                    tolerance_position = tolerance_for_name.position
                    break
            tolerance.append(tolerance_position)
        print(tolerance)

        while True:
            desired_point = traj_msg.trajectory.points[trajectory_point_idx]

            # command waypoint
            left_arm_command = MotionCommand()
            left_arm_command.header.stamp = rospy.Time.now()
            left_arm_command.joint_position = list_to_jvq(desired_point.positions)
            left_arm_command.control_mode.mode = ControlMode.JOINT_POSITION
            self.left_arm_motion_command_pub.publish(left_arm_command)

            # get feedback
            actual_point = self.get_actual_point(traj_msg)

            # check if we're close enough to advance
            if waypoint_reached(desired_point, actual_point, tolerance):
                trajectory_point_idx += 1
                if trajectory_point_idx == len(traj_msg.trajectory.points):
                    break

            feedback = FollowJointTrajectoryFeedback()
            feedback.header.stamp = rospy.Time.now()
            feedback.joint_names = traj_msg.trajectory.joint_names
            feedback.desired = desired_point
            feedback.actual = actual_point
            self._as.publish_feedback(feedback)

        success_result = FollowJointTrajectoryResult()
        success_result.error_code = actionlib.GoalStatus.SUCCEEDED
        self._as.set_succeeded(success_result)

    def get_actual_point(self, traj_msg):
        left_status = self.left_arm_motion_command_listener.get()
        right_status = self.right_arm_motion_command_listener.get()
        # This method of converting status messages to a list ensure the order matches in the trajectory
        current_joint_positions = []
        for name in traj_msg.trajectory.joint_names:
            if name == 'victor_left_arm_joint_1':
                pos = left_status.measured_joint_position.joint_1
            elif name == 'victor_left_arm_joint_2':
                pos = left_status.measured_joint_position.joint_2
            elif name == 'victor_left_arm_joint_3':
                pos = left_status.measured_joint_position.joint_3
            elif name == 'victor_left_arm_joint_4':
                pos = left_status.measured_joint_position.joint_4
            elif name == 'victor_left_arm_joint_5':
                pos = left_status.measured_joint_position.joint_5
            elif name == 'victor_left_arm_joint_6':
                pos = left_status.measured_joint_position.joint_6
            elif name == 'victor_left_arm_joint_7':
                pos = left_status.measured_joint_position.joint_7
            elif name == 'victor_right_arm_joint_1':
                pos = right_status.measured_joint_position.joint_1
            elif name == 'victor_right_arm_joint_2':
                pos = right_status.measured_joint_position.joint_2
            elif name == 'victor_right_arm_joint_3':
                pos = right_status.measured_joint_position.joint_3
            elif name == 'victor_right_arm_joint_4':
                pos = right_status.measured_joint_position.joint_4
            elif name == 'victor_right_arm_joint_5':
                pos = right_status.measured_joint_position.joint_5
            elif name == 'victor_right_arm_joint_6':
                pos = right_status.measured_joint_position.joint_6
            elif name == 'victor_right_arm_joint_7':
                pos = right_status.measured_joint_position.joint_7
            else:
                raise NotImplementedError()
            current_joint_positions.append(pos)
        actual_point = JointTrajectoryPoint()
        actual_point.positions = current_joint_positions
        return actual_point


def main():
    rospy.init_node('victor_ros_trajectory_forwarder')

    fwd = TrajectoryForwarder()

    rospy.spin()


if __name__ == "__main__":
    main()
