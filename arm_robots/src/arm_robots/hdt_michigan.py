#! /usr/bin/env python
from colorama import Fore

import rospy
from arm_robots.base_robot import BaseRobot
from arm_robots.robot import MoveitEnabledRobot


class BaseVal(BaseRobot):

    def __init__(self, robot_namespace: str):
        super().__init__(robot_namespace=robot_namespace)
        rospy.loginfo("Val ready !")


class Val(MoveitEnabledRobot):

    def __init__(self, robot_namespace: str = 'val'):
        self.base_val = BaseVal(robot_namespace)
        super().__init__(self.base_val)
        rospy.loginfo(Fore.GREEN + "Val ready!")

    def get_joint_trajectory_controller_name(self):
        """ This should match the *.yaml file, and you can also run rqt_controller_manager to check the names """
        return "both_arms_trajectory_controller"
