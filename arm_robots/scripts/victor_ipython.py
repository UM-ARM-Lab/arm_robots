#! /usr/bin/env python
import colorama
import ipdb
import numpy as np
import roscpp_initializer

from arc_utilities.ros_init import rospy_and_cpp_init
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


if __name__ == "__main__":
    main()
