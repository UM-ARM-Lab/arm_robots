#! /usr/bin/env python
import math
import colorama
import numpy as np
import rospy
from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.victor import Victor
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import ControlMode
import time
from arm_robots.config.victor_config import KUKA_MAX_JOINT_VELOCITIES, LEFT_ARM_JOINT_NAMES, RIGHT_ARM_JOINT_NAMES

"""
This scipt calculates the Kuka arms max joint speeds and max joint accels.
These values should be set in the Kuka "machine data" (as per the kuka manual), however I (Brad) cannot find it

Joint velocities: The results of this script for our Kuka R800 LWR are approximately:
[100.0, 100.0, 100.0, 130.0, 140.0, 180.0, 180.0] deg/s
These results agrees with some numbers I have seen on forums.

Online these values cannot be set directly. Instead, a "relative velocity" can be set to scale all speeds.

Joint Accels: Setting control_mode.joint_relative_acceleration does not seem to do anything.
I can still test these values at full acceleration (and full vel for better accuracy)

"""


# arm_to_use = "right_arm"
arm_to_use = "left_arm"
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


def calc_kuka_accel_for_joint(victor, joint_number: int, relative_accel=0.1, motion_distance=0.2):
    victor.speak("Moving to zero position")

    def set_setup_control_mode():
        victor.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=0.2, accel=0.1)

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
    # victor.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=1.0, accel=relative_accel)
    victor.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=1, accel=0.01)
    time.sleep(1.0)
    t0 = time.time()
    victor.send_joint_command(joints_to_use, JointTrajectoryPoint(positions=p))
    while not reached_endpoint():
        time.sleep(1e-5)
    dt = time.time() - t0
    print(dt)
    average_speed = motion_distance*2 / dt
    if average_speed > KUKA_MAX_JOINT_VELOCITIES[joint_number]/2:
        print(f"{colorama.Fore.RED}Average speed of {average_speed} is more than half of max speed of "
              f"{KUKA_MAX_JOINT_VELOCITIES[joint_number]}\n"
              f"Estimated acceleration is not accurate{colorama.Style.RESET_ALL}")
    accel = average_speed * 4 / dt
    print(f"Estimated accel is {accel} rad/s^2 or {accel * 180 / np.pi} deg/s^2")

    # print(f"Kuka joint {joint_number} max speed is {speed} rad/s = {speed*180 / math.pi} deg/s")
    set_setup_control_mode()
    move(p_start)
    return accel * 180 / np.pi


def calc_max_joint_speeds(victor):
    joint_speeds = {}
    for joint_num in range(7):
        joint_speeds[joint_num] = calc_kuka_speed_for_joint(victor, joint_num, relative_vel=0.05)

    print(f"Kuka max joint speeds (deg/s): {[joint_speeds[i] for i in range(7)]}")


def calc_max_joint_accel(victor):
    # for motion_distance in [0.05, 0.1, 0.2, 0.3, 0.4]:
    #     calc_kuka_accel_for_joint(victor, 0, relative_accel=0.00001, motion_distance=motion_distance)
    # calc_kuka_accel_for_joint(victor, 0, relative_accel=0.1, motion_distance=1.3)
    joint_motion_distances = {
        0: 1.2,
        1: 0.5,
        2: 0.1,
        3: 0.2,
        4: 0.1,
        5: 0.1,
        6: 0.1
    }

    joint_accels = {}
    for joint_num, motion_distance in joint_motion_distances.items():
        print(f"Testing joint {joint_num} with distance {motion_distance}")
        joint_accels[joint_num] = calc_kuka_accel_for_joint(victor, joint_num, motion_distance=motion_distance)

    print(f"Kuka max joint accels (deg/s^2): {[joint_accels[i] for i in range(7)]}")


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

    calc_max_joint_speeds(victor)
    # calc_max_joint_accel(victor)




if __name__ == "__main__":
    main()
