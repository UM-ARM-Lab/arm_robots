#! /usr/bin/env python

import rospy
from arm_robots.robot import ARMRobot


class Victor(ARMRobot):

    def __init__(self, execute_by_default: bool = False):
        super().__init__(execute_by_default=execute_by_default)
        rospy.loginfo("Victor ready!")


if __name__ == "__main__":
    rospy.init_node("motion_enabled_victor")
    mev = Victor()
    rospy.spin()
