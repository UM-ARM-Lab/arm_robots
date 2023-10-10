#! /usr/bin/env python
from victor_hardware_interface_msgs.msg import ControlMode
import rospy
from arm_robots.victor import Victor
import sys
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", action="store_true")
    parser.add_argument("--right", action="store_true")
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    rospy.init_node("switch_to_impedance")

    victor = Victor(robot_namespace='victor')
    victor.connect()

    if args.left:
        print("left")
        victor.set_left_arm_control_mode(ControlMode.JOINT_IMPEDANCE)
    if args.right:
        print("right")
        victor.set_right_arm_control_mode(ControlMode.JOINT_IMPEDANCE)
