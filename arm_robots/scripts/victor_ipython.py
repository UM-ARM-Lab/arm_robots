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

    roscpp_initializer.init_node("cpp_basic_motion", [], disable_signals=True)
    rospy.init_node('basic_motion')

    victor = Victor()

    ipdb.set_trace()

    roscpp_initializer.shutdown()


if __name__ == "__main__":
    main()
