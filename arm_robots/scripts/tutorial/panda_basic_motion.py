#! /usr/bin/env python
import numpy as np

import rospy
from arm_robots.panda import Panda

if __name__ == '__main__':
    rospy.init_node('panda_motion')

    panda_1 = Panda(arms_controller_name="/combined_panda/effort_joint_trajectory_controller_panda_1",
                    controller_name="effort_joint_trajectory_controller_panda_1",
                    robot_namespace='combined_panda',
                    panda_name='panda_1',
                    has_gripper=True)
    panda_1.connect()

    panda_2 = Panda(arms_controller_name="/combined_panda/effort_joint_trajectory_controller_panda_2",
                    controller_name="effort_joint_trajectory_controller_panda_2",
                    robot_namespace='combined_panda',
                    panda_name='panda_2',
                    has_gripper=True)
    panda_2.connect()

    # panda_1.plan_to_joint_config("panda_1", [0.0, -1.3, 0.0, -2.6, 0, 1.3, np.pi / 4])
    panda_2.plan_to_joint_config("panda_2", [0.0, 0.0, -np.pi/4, -np.pi/2., 0, np.pi/2, 0.0])
