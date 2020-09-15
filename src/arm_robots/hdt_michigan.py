#! /usr/bin/env python

import rospy
from arm_robots.robot import ARMRobot


class Val(ARMRobot):
    def __init__(self):
        super().__init__()
        rospy.loginfo("Val ready!")


if __name__ == "__main__":
    rospy.init_node("val")
    mev = Val()
    rospy.spin()
