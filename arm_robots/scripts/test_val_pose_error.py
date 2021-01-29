#! /usr/bin/env python
import colorama
import numpy as np

from arc_utilities import ros_init
from arm_robots.hdt_michigan import Val


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    ros_init.rospy_and_cpp_init("basic_motion")

    val = Val(raise_on_failure=True)
    val.connect()
    val.set_execute(False)

    move_group = val.get_move_group_commander('both_arms')
    print(move_group.get_end_effector_link())

    val.plan_to_pose('both_arms', 'drive10', [0.3, 0.8, 0.5, -np.pi / 2, np.pi / 2, 0])

    val.disconnect()
    ros_init.shutdown()


if __name__ == "__main__":
    main()
