#! /usr/bin/env python

# Victor repeatedly grabs a cloth from a table
#
# This is intended as an example script
#
# Note: you need to have a table set up for the gripper to hit

import time

import rospy
from victor_hardware_interface_msgs.msg import ControlMode

config_start = [0.977085421269, 0.682855921949, 2.07134929804, 1.50146884222, -0.98459285517, -1.24244563762,
                0.323371274152]


def grab_cloth(mev):
    """Pickup and drop a cloth from the table"""
    mev.set_gripper("right", [0.3, 0.3, 0.3], blocking=False)
    mev.set_manipulator("right_arm")
    mev.change_control_mode(ControlMode.JOINT_IMPEDANCE)

    # Move over table
    mev.plan_to_configuration(config_start, execute=True, blocking=True)

    time.sleep(1)
    # mev.guarded_move_hand_straight([0,0,-1], 0.2, force_trigger=15, step_size=0.01)

    # Move down to table and keep pushing
    # mev.push_with_force([0,0,-1], 40, planning_distance=.2)
    mev.push_with_force([0, 0, -1], 40, planning_distance=.2, step_size=0.02)

    # Grab cloth
    mev.close_gripper("right", blocking=True, continuous=True)

    # Move back up
    mev.plan_to_configuration(config_start, execute=True, blocking=True)

    time.sleep(5)

    # Drop cloth
    mev.set_gripper("right", [0.3, 0.3, 0.3], blocking=True)


def run_mev():
    global config1, config2, log_one_data

    rospy.init_node("motion")
    mev = Victor(world_frame="victor_root")

    while True:
        grab_cloth(mev)

    print("plan complete")


if __name__ == "__main__":
    run_mev()
