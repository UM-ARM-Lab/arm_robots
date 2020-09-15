#! /usr/bin/env python
import time
from math import pi

import rospy
from arm_robots.victor import Victor
from tf.transformations import compose_matrix
from victor_hardware_interface_msgs.msg import ControlMode

config1 = [1.60963234611, 0.533623140261, -0.742047506794, -1.16109858085, -0.323169964304, 1.0979096577, 0.21963411082]

config3 = [1.3614709264, -1.17878472014, -1.00180420345, -1.20442928365, -0.917654439591, 1.39904650482, -1.04936635634]

left_hand_start = compose_matrix(angles=[-pi / 2, -pi / 4, -pi],
                                 translate=[.63, .33, .72])

right_hand_start = compose_matrix(angles=[pi / 2, pi / 4, pi],
                                  translate=[.63, -.33, .72])


def run_mev():
    global config1, config2, log_one_data

    rospy.init_node("motion_enabled_victor")

    mev = Victor()
    # TODO: replace with moveit call
    # mev.or_model.env.GetCollisionChecker().SetCollisionOptions(rave.CollisionOptions.ActiveDOFs)

    mev.open_gripper("left", blocking=False)
    mev.open_gripper("right", blocking=False)
    time.sleep(1)

    print("Using joint position mode")
    mev.set_manipulator("left_arm")
    mev.change_control_mode(ControlMode.JOINT_POSITION)
    mev.plan_to_configuration(config1, execute=True)
    mev.plan_to_pose(left_hand_start, execute=True)

    mev.set_manipulator("right_arm")
    mev.change_control_mode(ControlMode.JOINT_POSITION)
    mev.plan_to_configuration(config3, execute=True)
    mev.plan_to_pose(right_hand_start, execute=True)

    mev.close_gripper("left", blocking=False)
    mev.close_gripper("right", blocking=False)
    time.sleep(1)

    mev.set_manipulator("left_arm")
    mev.plan_to_relative_pose(compose_matrix(translate=[0, -.1, 0]), execute=True)
    mev.plan_to_pose(left_hand_start, execute=True)

    mev.set_manipulator("right_arm")
    mev.plan_to_relative_pose(compose_matrix(translate=[-.1, 0, 0]), execute=True)
    mev.plan_to_pose(right_hand_start, execute=True)

    print("Switching to Impedance mode")

    mev.set_manipulator("left_arm")
    mev.change_control_mode(ControlMode.JOINT_IMPEDANCE)
    mev.plan_to_relative_pose(compose_matrix(translate=[0, -.1, 0]), execute=True)
    mev.plan_to_pose(left_hand_start, execute=True)

    mev.set_manipulator("right_arm")
    mev.change_control_mode(ControlMode.JOINT_IMPEDANCE)
    mev.plan_to_relative_pose(compose_matrix(translate=[-.1, 0, 0]), execute=True)
    mev.plan_to_pose(right_hand_start, execute=True)

    print("plan complete")


if __name__ == "__main__":
    run_mev()
