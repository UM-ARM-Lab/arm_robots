#! /usr/bin/env python
import argparse
import sys

import colorama
import numpy as np
import roscpp_initializer
import rospy
from victor_hardware_interface_msgs.msg import ControlMode

from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.victor import Victor


def wrap_angle(angle_rad):
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def mirror_right_to_left(q):
    q_new = q.copy()
    q_new[0] = -wrap_angle(q[0] + np.pi)
    q_new[1] = -q[1]
    q_new[2] = -(q[3] + np.pi / 2)
    q_new[3] = -q[3]
    q_new[4] = -q[4]
    q_new[5] = -q[5]
    q_new[6] = -q[6]
    return q_new


def main():
    np.set_printoptions(suppress=True, precision=2, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("hardcore_test")

    parser = argparse.ArgumentParser()
    parser.add_argument('arm', choices=['left', 'right'])
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    victor = Victor()
    victor.connect()

    # victor.open_gripper(args.arm)
    # rospy.sleep(1)
    # victor.close_gripper(args.arm)
    # rospy.sleep(1)

    # test joint trajectories
    for control_mode in [ControlMode.JOINT_POSITION, ControlMode.JOINT_IMPEDANCE]:
        victor.move_to_impedance_switch(actually_switch=False)
        victor.set_control_mode(control_mode)
        test_joint_trajectories(args, victor)

    # TODO:
    # - check the force torque sensors
    # - test cartesian impedance

    roscpp_initializer.shutdown()


def test_joint_trajectories(args, victor):
    group = args.arm + '_arm'
    victor.plan_to_joint_config(group, 'zero')
    q = np.deg2rad([-66, 60, 47, -38, 74, 53, 90.0])
    if args.arm == 'right':
        q = mirror_right_to_left(q)
    victor.plan_to_joint_config(group, q)
    for i in range(7):
        q_lower = q.copy()
        q_lower[i] -= 0.5

        q_upper = q.copy()
        q_upper[i] += 0.5

        victor.plan_to_joint_config(group, q_lower)

        victor.plan_to_joint_config(group, q_upper)
    victor.plan_to_joint_config(group, 'zero')


if __name__ == "__main__":
    main()
