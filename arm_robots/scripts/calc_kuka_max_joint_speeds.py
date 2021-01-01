#! /usr/bin/env python
import math
import colorama
import numpy as np
import rospy
from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.victor import Victor, RIGHT_ARM_JOINT_NAMES, LEFT_ARM_JOINT_NAMES
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import ControlMode
import time

"""
This scipt calculates the Kuka arms max joint speeds.
These values should be set in the Kuka "machine data" (as per the kuka manual), however I (Brad) cannot find it
The results of this script for our Kuka R800 LWR are approximately:
[100.0, 100.0, 100.0, 130.0, 140.0, 180.0, 180.0] deg/s
These results agrees with some numbers I have seen on forums.

Online these values cannot be set directly. Instead, a "relative velocity" can be set to scale all speeds.
"""


arm_to_use = "right_arm"
joints_to_use = {"right_arm": RIGHT_ARM_JOINT_NAMES,
                 "left_arm": LEFT_ARM_JOINT_NAMES}[arm_to_use]


def calc_kuka_speed_for_joint(victor, joint_number: int, relative_vel=0.1, motion_distance=1.0):
    victor.speak("Moving to zero position")

    def set_setup_control_mode():
        victor.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=.1, accel=0.1)

    def move(config):
        victor.plan_to_joint_config(arm_to_use, config)

    def reached_endpoint():
        return abs(victor.get_joint_positions_map()[joints_to_use[joint_number]] - motion_distance) < 0.01

    p_start = (0, 0, 0, 0, 0, 0, 0)
    p = list(p_start)
    set_setup_control_mode()
    move(p)
    p[joint_number] = -motion_distance
    move(p)
    p[joint_number] = motion_distance
    victor.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=relative_vel, accel=1.0)
    time.sleep(1.0)
    t0 = time.time()
    victor.send_joint_command(joints_to_use, JointTrajectoryPoint(positions=p))
    while not reached_endpoint():
        time.sleep(1e-5)
    dt = time.time() - t0

    speed = motion_distance*2 / dt / relative_vel
    print(f"Kuka joint {joint_number} max speed is {speed} rad/s = {speed*180 / math.pi} deg/s")
    set_setup_control_mode()
    move(p_start)
    return speed * 180 / math.pi


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("basic_motion")

    victor = Victor()
    victor.connect()

    # Visually verify victor is connected
    rospy.sleep(1)
    victor.open_left_gripper()
    victor.open_right_gripper()
    rospy.sleep(0.5)
    victor.close_left_gripper()
    victor.close_right_gripper()
    rospy.sleep(0.5)

    joint_speeds = {}
    for joint_num in range(7):
        joint_speeds[joint_num] = calc_kuka_speed_for_joint(victor, joint_num, relative_vel=0.05)

    print(f"Kuka max joint speeds (deg/s): {[joint_speeds[i] for i in range(7)]}")



if __name__ == "__main__":
    main()
