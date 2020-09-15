#! /usr/bin/env python


import time
from math import pi

import rospy
from arm_robots.victor import Victor
from tf.transformations import compose_matrix

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

    mev.set_force_limit(5.2)

    time.sleep(2)

    print("plan complete")
    # mev.start()
    # rospy.spin()
    # config1 =


if __name__ == "__main__":
    # TODO Add this test
    print("This test is not implemented yet")
    run_mev()
