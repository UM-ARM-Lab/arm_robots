#! /usr/bin/env python

import rospy
from arm_robots.robot import ARMRobot


class Val(ARMRobot):

    def __init__(self, execute_by_default: bool = False):
        super().__init__(execute_by_default=execute_by_default)
        rospy.loginfo("Val ready !")


if __name__ == "__main__":
    rospy.init_node("val")
    mev = Val()
    rospy.spin()
