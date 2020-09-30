#! /usr/bin/env python

import rospy
from arm_robots.base_robot import BaseRobot


class BaseVal(BaseRobot):

    def __init__(self, robot_namespace: str = 'val'):
        super().__init__(robot_namespace=robot_namespace)
        rospy.loginfo("Val ready !")
