#! /usr/bin/env python
from typing import List

import numpy as np

import actionlib
import rospy
from arc_utilities.ros_helpers import Listener
from arm_robots.robot import interpolate_joint_trajectory_points, waypoint_reached
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, \
    FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import list_to_jvq
from victor_hardware_interface_msgs.msg import MotionCommand, MotionStatus, ControlMode
from victor_hardware_interface_msgs.srv import GetControlMode, SetControlMode, GetControlModeRequest, \
    SetControlModeRequest

left_arm_joints = {
    'victor_left_arm_joint_1',
    'victor_left_arm_joint_2',
    'victor_left_arm_joint_3',
    'victor_left_arm_joint_4',
    'victor_left_arm_joint_5',
    'victor_left_arm_joint_6',
    'victor_left_arm_joint_7',
}

right_arm_joints = {
    'victor_right_arm_joint_1',
    'victor_right_arm_joint_2',
    'victor_right_arm_joint_3',
    'victor_right_arm_joint_4',
    'victor_right_arm_joint_5',
    'victor_right_arm_joint_6',
    'victor_right_arm_joint_7',
}

both_arm_joints = left_arm_joints.union(right_arm_joints)


def delegate_positions_to_arms(positions, joint_names: List[str]):
    """
    Given the list of positions, assumed to be in the same order as the list of joint names,
    determine wether it's meant for the left arm, right arm, or both arms, and error if it doesn't perfectly match any
    """
    abort_msg = None
    left_arm_positions = None
    right_arm_positions = None
    if set(joint_names) == left_arm_joints:
        left_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in left_arm_joints:
                left_arm_positions.append(joint_value)
    elif set(joint_names) == right_arm_joints:
        right_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in right_arm_joints:
                right_arm_positions.append(joint_value)
    elif set(joint_names) == both_arm_joints:
        left_arm_positions = []
        right_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in left_arm_joints:
                left_arm_positions.append(joint_value)
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in right_arm_joints:
                right_arm_positions.append(joint_value)
    else:
        abort_msg = "Invalid joint_names, didn't match the left arm joints, right arm joints, or all joints"
    return right_arm_positions, left_arm_positions, abort_msg


# TODO: make this work with val too
class TrajectoryForwarder(object):
    def __init__(self):
        self._action_name = "follow_joint_trajectory"
        self.action_server = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction,
                                                          execute_cb=self.execute_cb,
                                                          auto_start=False)
        self.left_arm_motion_command_pub = rospy.Publisher("left_arm/motion_command", MotionCommand, queue_size=10)
        self.right_arm_motion_command_pub = rospy.Publisher("right_arm/motion_command", MotionCommand, queue_size=10)
        self.left_arm_motion_command_listener = Listener("left_arm/motion_status", MotionStatus)
        self.right_arm_motion_command_listener = Listener("right_arm/motion_status", MotionStatus)
        self.left_arm_get_control_mode_srv = rospy.ServiceProxy("/left_arm/get_control_mode_service", GetControlMode)
        self.right_arm_get_control_mode_srv = rospy.ServiceProxy("/right_arm/get_control_mode_service", GetControlMode)
        self.left_arm_set_control_mode_srv = rospy.ServiceProxy("/left_arm/set_control_mode_service", SetControlMode)
        self.right_arm_set_control_mode_srv = rospy.ServiceProxy("/right_arm/set_control_mode_service", SetControlMode)
        self.action_server.start()

    def execute_cb(self, traj_msg: FollowJointTrajectoryGoal):
        # TODO: set the control mode speed so that we move the expected distance at the expected speed?
        left_control_mode = self.left_arm_get_control_mode_srv(GetControlModeRequest())
        right_control_mode = self.right_arm_get_control_mode_srv(GetControlModeRequest())

        set_left_control_mode = SetControlModeRequest()
        if left_control_mode.has_active_control_mode:
            set_left_control_mode.new_control_mode = left_control_mode.active_control_mode
        else:
            rospy.logerr("Left arm has no active control mode. Refusing to set control mode.")
        set_left_control_mode.new_control_mode.joint_path_execution_params.joint_relative_velocity = 0.1
        set_left_control_mode.new_control_mode.joint_path_execution_params.joint_relative_acceleration = 0.5

        set_right_control_mode = SetControlModeRequest()
        if right_control_mode.has_active_control_mode:
            set_right_control_mode.new_control_mode = right_control_mode.active_control_mode
        else:
            rospy.logerr("Left arm has no active control mode. Refusing to set control mode.")

        self.left_arm_set_control_mode_srv(set_left_control_mode)
        self.right_arm_set_control_mode_srv(set_right_control_mode)

        # construct a list of the tolerances in order of the joint names
        tolerance = []
        for name in traj_msg.trajectory.joint_names:
            default_tolerance_position = 0.05
            tolerance_position = None
            for tolerance_for_name in traj_msg.path_tolerance:
                if tolerance_for_name.name == name:
                    tolerance_position = tolerance_for_name.position
                    break
            if tolerance_position is None:
                tolerance_position = default_tolerance_position
                rospy.logwarn_throttle(1, f"using default path tolerance {default_tolerance_position}")
            tolerance.append(tolerance_position)

        # Interpolate the trajectory to a fine resolution
        # interpolated_points = interpolate_joint_trajectory_points(traj_msg.trajectory.points, max_step_size=0.001)
        interpolated_points = traj_msg.trajectory.points

        trajectory_point_idx = 0
        while True:
            desired_point = interpolated_points[trajectory_point_idx]

            right_arm_positions, left_arm_positions, abort_msg = delegate_positions_to_arms(desired_point.positions,
                                                                                            traj_msg.trajectory.joint_names)

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Preempt requested, aborting.")
                break

            if abort_msg is not None:
                self.action_server.set_aborted(text=abort_msg)

            # command waypoint
            if left_arm_positions is not None:
                left_arm_command = MotionCommand()
                left_arm_command.header.stamp = rospy.Time.now()
                left_arm_command.joint_position = list_to_jvq(left_arm_positions)
                left_arm_command.control_mode.mode = ControlMode.JOINT_POSITION
                self.left_arm_motion_command_pub.publish(left_arm_command)
            if right_arm_positions is not None:
                right_arm_command = MotionCommand()
                right_arm_command.header.stamp = rospy.Time.now()
                right_arm_command.joint_position = list_to_jvq(right_arm_positions)
                right_arm_command.control_mode.mode = ControlMode.JOINT_POSITION
                self.right_arm_motion_command_pub.publish(right_arm_command)

            # get feedback
            actual_point = self.get_actual_trajectory_point(traj_msg)

            # If we're close enough, advance
            if waypoint_reached(desired_point, actual_point, tolerance):
                trajectory_point_idx += 1
                if trajectory_point_idx == len(interpolated_points):
                    break

            feedback = FollowJointTrajectoryFeedback()
            feedback.header.stamp = rospy.Time.now()
            feedback.joint_names = traj_msg.trajectory.joint_names
            feedback.desired = desired_point
            feedback.actual = actual_point
            self.action_server.publish_feedback(feedback)

        success_result = FollowJointTrajectoryResult()
        success_result.error_code = actionlib.GoalStatus.SUCCEEDED
        self.action_server.set_succeeded(success_result)

    def get_actual_trajectory_point(self, traj_msg):
        left_status = self.left_arm_motion_command_listener.get(block_until_data=True)
        right_status = self.right_arm_motion_command_listener.get(block_until_data=True)
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
    np.set_printoptions(linewidth=200, suppress=True, precision=4)
    rospy.init_node('victor_ros_trajectory_forwarder')

    fwd = TrajectoryForwarder()

    rospy.spin()


if __name__ == "__main__":
    main()
