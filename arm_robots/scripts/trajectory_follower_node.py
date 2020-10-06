#! /usr/bin/env python

import numpy as np

import rospy
from arm_robots.hdt_michigan import BaseVal
from arm_robots.trajectory_follower import TrajectoryFollower
from arm_robots.victor import BaseVictor


def main():
    np.set_printoptions(linewidth=200, suppress=True, precision=4)
    rospy.init_node('trajectory_follower')

    robot_name_rosparam_name = "~robot_name"
    if robot_name := rospy.get_param(robot_name_rosparam_name, None):
        if robot_name == "victor":
            base_robot = BaseVictor(robot_name)
        elif robot_name == "victor":
            base_robot = BaseVal(robot_name)
        else:
            raise NotImplementedError(f"Invalid ros param {robot_name_rosparam_name} {robot_name}")
    else:
        rospy.loginfo(f"rosparam {robot_name_rosparam_name} not set, Defaulting to Victor")
        base_robot = BaseVictor(robot_name)
    controller_name = rospy.get_param('~controller')
    fwd = TrajectoryFollower(base_robot, controller_name)
    fwd.start_server()

    rospy.spin()


if __name__ == "__main__":
    main()
