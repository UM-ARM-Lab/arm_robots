#! /usr/bin/env python
from typing import List

import numpy as np

import actionlib
import rospy
from arc_utilities.extra_functions_to_be_put_in_the_right_place import make_color
from arc_utilities.ros_helpers import Listener
from arm_robots.hdt_michigan import Val
from arm_robots.robot import interpolate_joint_trajectory_points, waypoint_reached, get_ordered_tolerance_list, ARMRobot
from arm_robots.victor import Victor
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, \
    FollowJointTrajectoryResult
from moveit_msgs.msg import DisplayRobotState, ObjectColor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import list_to_jvq
from victor_hardware_interface_msgs.msg import MotionCommand, MotionStatus, ControlMode
from victor_hardware_interface_msgs.srv import GetControlMode, SetControlMode, GetControlModeRequest, \
    SetControlModeRequest

left_arm_joints = [
    'victor_left_arm_joint_1',
    'victor_left_arm_joint_2',
    'victor_left_arm_joint_3',
    'victor_left_arm_joint_4',
    'victor_left_arm_joint_5',
    'victor_left_arm_joint_6',
    'victor_left_arm_joint_7',
]

right_arm_joints = [
    'victor_right_arm_joint_1',
    'victor_right_arm_joint_2',
    'victor_right_arm_joint_3',
    'victor_right_arm_joint_4',
    'victor_right_arm_joint_5',
    'victor_right_arm_joint_6',
    'victor_right_arm_joint_7',
]

both_arm_joints = left_arm_joints + right_arm_joints


def delegate_positions_to_arms(positions, joint_names: List[str]):
    """
    Given the list of positions, assumed to be in the same order as the list of joint names,
    determine wether it's meant for the left arm, right arm, or both arms, and error if it doesn't perfectly match any
    """
    abort_msg = None
    left_arm_positions = None
    right_arm_positions = None
    # set equality ignores order
    if set(joint_names) == set(left_arm_joints):
        left_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in left_arm_joints:
                left_arm_positions.append(joint_value)
    elif set(joint_names) == set(right_arm_joints):
        right_arm_positions = []
        for joint_name, joint_value in zip(joint_names, positions):
            if joint_name in right_arm_joints:
                right_arm_positions.append(joint_value)
    elif set(joint_names) == set(both_arm_joints):
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
    def __init__(self, arm_robot: ARMRobot):
        self._action_name = arm_robot.robot_namespace + "/both_arms_trajectory_controller/follow_joint_trajectory"
        self.action_server = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction,
                                                          execute_cb=self.execute_cb,
                                                          auto_start=False)
        self.left_arm_motion_command_pub = rospy.Publisher("left_arm/motion_command", MotionCommand, queue_size=10)
        self.right_arm_motion_command_pub = rospy.Publisher("right_arm/motion_command", MotionCommand, queue_size=10)
        self.waypoint_state_pub = rospy.Publisher("waypoint_robot_state", DisplayRobotState, queue_size=10)
        self.left_arm_motion_command_listener = Listener("left_arm/motion_status", MotionStatus)
        self.right_arm_motion_command_listener = Listener("right_arm/motion_status", MotionStatus)
        self.left_arm_get_control_mode_srv = rospy.ServiceProxy("/left_arm/get_control_mode_service", GetControlMode)
        self.right_arm_get_control_mode_srv = rospy.ServiceProxy("/right_arm/get_control_mode_service", GetControlMode)
        self.left_arm_set_control_mode_srv = rospy.ServiceProxy("/left_arm/set_control_mode_service", SetControlMode)
        self.right_arm_set_control_mode_srv = rospy.ServiceProxy("/right_arm/set_control_mode_service", SetControlMode)
        self.action_server.start()
        self.arm_robot = arm_robot

    def execute_cb(self, traj_msg: FollowJointTrajectoryGoal):
        # TODO: set the control mode speed so that we move the expected distance at the expected speed?
        left_control_mode_res = self.left_arm_get_control_mode_srv(GetControlModeRequest())
        right_control_mode_res = self.right_arm_get_control_mode_srv(GetControlModeRequest())

        set_left_control_mode = SetControlModeRequest()
        if left_control_mode_res.has_active_control_mode:
            left_arm_active_control_mode = left_control_mode_res.active_control_mode.control_mode
            set_left_control_mode.new_control_mode = left_control_mode_res.active_control_mode
        else:
            left_arm_active_control_mode = ControlMode.JOINT_POSITION
            rospy.logerr("Left arm has no active control mode. Refusing to set control mode.")
        set_left_control_mode.new_control_mode.joint_path_execution_params.joint_relative_velocity = 0.5
        set_left_control_mode.new_control_mode.joint_path_execution_params.joint_relative_acceleration = 0.5

        set_right_control_mode = SetControlModeRequest()
        if right_control_mode_res.has_active_control_mode:
            right_arm_active_control_mode = right_control_mode_res.active_control_mode.control_mode
            set_right_control_mode.new_control_mode = right_control_mode_res.active_control_mode
        else:
            right_arm_active_control_mode = ControlMode.JOINT_POSITION
            rospy.logerr("Left arm has no active control mode. Refusing to set control mode.")

        self.left_arm_set_control_mode_srv(set_left_control_mode)
        self.right_arm_set_control_mode_srv(set_right_control_mode)

        # construct a list of the tolerances in order of the joint names
        trajectory_joint_names = traj_msg.trajectory.joint_names
        tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.path_tolerance)
        goal_tolerance = get_ordered_tolerance_list(trajectory_joint_names, traj_msg.goal_tolerance, is_goal=True)

        # Interpolate the trajectory to a fine resolution
        # if you set max_step_size to be large and position tolerance to be small, then things will be jerky
        if len(traj_msg.trajectory.points) == 0:
            rospy.loginfo("Ignoring empty trajectory")
            return

        interpolated_points = interpolate_joint_trajectory_points(traj_msg.trajectory.points, max_step_size=0.1)

        trajectory_point_idx = 0
        while True:
            now = rospy.Time.now()
            desired_point = interpolated_points[trajectory_point_idx]

            right_arm_positions, left_arm_positions, abort_msg = delegate_positions_to_arms(desired_point.positions,
                                                                                            trajectory_joint_names)

            if self.action_server.is_preempt_requested():
                rospy.loginfo("Preempt requested, aborting.")
                break

            if abort_msg is not None:
                self.action_server.set_aborted(text=abort_msg)

            # command waypoint
            waypoint_joint_state = JointState()
            waypoint_joint_state.header.stamp = now
            if left_arm_positions is not None:
                left_arm_command = MotionCommand()
                left_arm_command.header.stamp = now
                left_arm_command.joint_position = list_to_jvq(left_arm_positions)
                left_arm_command.control_mode = left_arm_active_control_mode
                self.left_arm_motion_command_pub.publish(left_arm_command)
                waypoint_joint_state.name.extend(left_arm_joints)
                waypoint_joint_state.position.extend(left_arm_positions)

            if right_arm_positions is not None:
                right_arm_command = MotionCommand()
                right_arm_command.header.stamp = now
                right_arm_command.joint_position = list_to_jvq(right_arm_positions)
                right_arm_command.control_mode = right_arm_active_control_mode
                self.right_arm_motion_command_pub.publish(right_arm_command)
                waypoint_joint_state.name.extend(right_arm_joints)
                waypoint_joint_state.position.extend(right_arm_positions)

            waypoint_state = DisplayRobotState()
            waypoint_state.state.joint_state = waypoint_joint_state
            for link_name in self.arm_robot.robot_commander.get_link_names():
                object_color = ObjectColor()
                object_color.id = link_name
                object_color.color = make_color(0, 1, 0, 1)
                waypoint_state.highlight_links.append(object_color)

            self.waypoint_state_pub.publish(waypoint_state)

            # get feedback
            actual_point = self.get_actual_trajectory_point(trajectory_joint_names)

            # If we're close enough, advance
            if trajectory_point_idx == len(interpolated_points) - 1:
                if waypoint_reached(desired_point, actual_point, goal_tolerance):
                    # we're done!
                    break
            else:
                if waypoint_reached(desired_point, actual_point, tolerance):
                    trajectory_point_idx += 1

            feedback = FollowJointTrajectoryFeedback()
            feedback.header.stamp = now
            feedback.joint_names = trajectory_joint_names
            feedback.desired = desired_point
            feedback.actual = actual_point
            self.action_server.publish_feedback(feedback)

        success_result = FollowJointTrajectoryResult()
        success_result.error_code = actionlib.GoalStatus.SUCCEEDED
        self.action_server.set_succeeded(success_result)

    def get_actual_trajectory_point(self, trajectory_joint_names: List[str]):
        rospy.sleep(1e-6)  # lets the listeners process messages better, results in smoother following
        left_status = self.left_arm_motion_command_listener.get()
        right_status = self.right_arm_motion_command_listener.get()
        current_joint_positions = self.arm_robot.get_joint_positions(trajectory_joint_names, left_status, right_status)
        actual_point = JointTrajectoryPoint()
        actual_point.positions = current_joint_positions
        return actual_point


def main():
    np.set_printoptions(linewidth=200, suppress=True, precision=4)
    rospy.init_node('trajectory_follower')

    robot_name_rosparam_name = "robot_name"
    if robot_name := rospy.get_param(robot_name_rosparam_name, None):
        if robot_name == "victor":
            robot = Victor(execute_by_default=True)
        elif robot_name == "victor":
            robot = Val(execute_by_default=True)
        else:
            raise NotImplementedError(f"Invalid ros param {robot_name_rosparam_name} {robot_name}")
    else:
        rospy.loginfo(f"rosparam {robot_name_rosparam_name} not set, Defaulting to Victor")
        robot = Victor(execute_by_default=True)

    fwd = TrajectoryForwarder(robot)

    rospy.spin()


if __name__ == "__main__":
    main()
