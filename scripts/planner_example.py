#! /usr/bin/env python

import arm_or_robots.motion_victor
import rospy
import time
from tf.transformations import compose_matrix
from math import pi
from victor_hardware_interface.msg import *


config1 = [1.60963234611, 0.533623140261, -0.742047506794, -1.16109858085, -0.323169964304, 1.09790965774, 0.219634110827]

config3 = [1.3614709264, -1.17878472014, -1.00180420345, -1.20442928365, -0.917654439591, 1.39904650482, -1.04936635634]


left_hand_start = compose_matrix(angles=[-pi/2, -pi/4, -pi],
                                 translate=[.63, .33, .72])

right_hand_start = compose_matrix(angles=[pi/2, pi/4, pi],
                                  translate=[.63, -.33, .72])


def run_mev():
    rospy.init_node("motion_enabled_victor")

    mev = arm_or_robots.motion_victor.MotionEnabledVictor()

    mev.open_gripper("left", blocking=False)
    mev.open_gripper("right", blocking=False)

    print("Using joint position mode")
    mev.set_manipulator("left_arm")


    mev.change_control_mode(ControlMode.JOINT_POSITION)
    mev.plan_to_configuration(config1, execute=True, blocking=True)
    mev.plan_to_pose(left_hand_start, execute=True, blocking=True)

    mev.set_manipulator("right_arm")
    mev.change_control_mode(ControlMode.JOINT_POSITION)
    mev.plan_to_configuration(config3, execute=True, blocking=True)
    mev.plan_to_pose(right_hand_start, execute=True, blocking=True)

    mev.close_gripper("left", blocking=False)
    mev.close_gripper("right")
    time.sleep(1)

    mev.set_manipulator("left_arm")
    mev.plan_to_relative_pose(compose_matrix(translate=[0,-.1,0]), execute=True, blocking=True)
    mev.plan_to_pose(left_hand_start, execute=True, blocking=True)

    mev.set_manipulator("right_arm")
    mev.plan_to_relative_pose(compose_matrix(translate=[-.1,0,0]), execute=True, blocking=True)
    mev.plan_to_pose(right_hand_start, execute=True, blocking=True)


    print("Switching to Impedance mode")
    mev.set_manipulator("left_arm")
    mev.change_control_mode(ControlMode.JOINT_IMPEDANCE)
    mev.plan_to_relative_pose(compose_matrix(translate=[0,-.1,0]), execute=True, blocking=True)
    mev.plan_to_pose(left_hand_start, execute=True, blocking=True)

    mev.set_manipulator("right_arm")
    mev.change_control_mode(ControlMode.JOINT_IMPEDANCE)
    mev.plan_to_relative_pose(compose_matrix(translate=[-.1,0,0]), execute=True, blocking=True)
    mev.plan_to_pose(right_hand_start, execute=True, blocking=True)

    print("plan complete")


if __name__ == "__main__":
    run_mev()
