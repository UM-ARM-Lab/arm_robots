#! /usr/bin/env python


# Run this script to be able to manually push the robot to any desired position
#
# Puts the robot in impedance mode, but updates the setpoint position to be the measured position
# Lowpass filter and thresholding prevent drift
# 


import time
from threading import Lock

import numpy as np

import rospy
from arm_robots.victor import Victor
from victor_hardware_interface_msgs.msg import MotionCommand, MotionStatus, ControlMode

joint_names = ['joint_' + str(i) for i in range(1, 8)]

hard_limits_deg = np.array([170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0])
safety_deg = 30.0  # Stay far away from joint hard limits

joint_lower = -(hard_limits_deg - safety_deg) * np.pi / 180
joint_upper = (hard_limits_deg - safety_deg) * np.pi / 180


class ManualMotion:

    def __init__(self, arm_name):
        self.sub = rospy.Subscriber(arm_name + "/motion_status", MotionStatus, self.callback_update)
        self.pub = rospy.Publisher(arm_name + "/motion_command", MotionCommand, queue_size=10)

        self.threshold = 0.05
        self.arrive_threshold = 0.02

        self.low_pass_tau = 0.01  # seconds
        self.prev_time = None
        self.lock = Lock()
        self.follow = False
        self.last_pos = None

    def callback_update(self, msg):
        with self.lock:
            self.run_lowpass(msg)

    def run_lowpass(self, msg):

        if self.prev_time is None:
            self.prev_time = time.time()

        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        if dt > 1:
            msg = f"Incredibly long time {dt}s between messages. Something is weird"
            rospy.logerr(msg)
            raise RuntimeError(msg)

        # Update our internal copy and calculate error for later thresholding
        meas = np.array([getattr(msg.measured_joint_position, joint) for joint in joint_names])
        cmd = np.array([getattr(msg.commanded_joint_position, joint) for joint in joint_names])
        err = np.linalg.norm(meas - cmd)
        self.last_pos = meas

        if err > self.threshold:
            # Robot is far enough away from setpoint that we assume someone is pushing it
            self.follow = True

        if err < self.arrive_threshold:
            # Robot is close enough to setpoint that we go into "hold position" mode
            self.follow = False

        if not self.follow:
            # Just hold position
            return

        # Stay away from hard limits
        meas = meas.clip(joint_lower, joint_upper)

        # Low pass filter to avoid jerky motions
        alpha = dt / (self.low_pass_tau + dt)
        new_cmd = (1.0 - alpha) * cmd + alpha * meas

        # Send the actual command
        cmd_msg = MotionCommand()
        cmd_msg.control_mode.mode = ControlMode.JOINT_IMPEDANCE
        for idx in range(len(joint_names)):
            setattr(cmd_msg.joint_position, joint_names[idx], new_cmd[idx])
        self.pub.publish(cmd_msg)


def print_joints(left, right):
    """Print nicely to the terminal so joint values can be copied"""
    rospy.loginfo("Joint angles are: ")

    def _vec_to_rad_str(vec):
        return '[' + ', '.join([str(np.round(rad, 3)) for rad in vec]) + ']'

    if (left is not None) and (left.last_pos is not None):
        rospy.loginfo("Left: " + _vec_to_rad_str(left.last_pos))
    if (right is not None) and (right.last_pos is not None):
        rospy.loginfo("Right: " + _vec_to_rad_str(right.last_pos))


def main():
    rospy.init_node("manual_motion")

    use_left_arm = rospy.get_param("~use_left_arm", True)
    use_right_arm = rospy.get_param("~use_right_arm", True)

    victor = Victor()

    if use_left_arm:
        rospy.loginfo("initializing left arm ...")
        victor.set_left_arm_control_mode(ControlMode.JOINT_IMPEDANCE)
        rospy.loginfo("done")
    else:
        rospy.loginfo("not using left arm")

    if use_right_arm:
        rospy.loginfo("initializing right arm ...", )
        victor.set_right_arm_control_mode(ControlMode.JOINT_IMPEDANCE)
        rospy.loginfo("done")
    else:
        rospy.loginfo("not using right arm")

    left = None
    right = None
    if use_left_arm:
        left = ManualMotion("left_arm")
    if use_right_arm:
        right = ManualMotion("right_arm")

    rospy.on_shutdown(lambda: print_joints(left, right))

    rospy.loginfo("Running")
    rospy.spin()


if __name__ == "__main__":
    main()
