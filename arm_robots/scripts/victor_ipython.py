#! /usr/bin/env python
import colorama
import ipdb
import numpy as np
import roscpp_initializer

import rospy
from arm_robots.victor import Victor


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("victor_ipython")

    victor = Victor()
    victor.connect()
    victor.close_left_gripper()
    victor.open_left_gripper()
    victor.close_right_gripper()
    victor.open_right_gripper()

    ipdb.set_trace()

    roscpp_initializer.shutdown()


def rospy_and_cpp_init(name):
    roscpp_initializer.init_node("cpp_" + name, [], disable_signals=True)
    rospy.init_node(name)


if __name__ == "__main__":
    main()
