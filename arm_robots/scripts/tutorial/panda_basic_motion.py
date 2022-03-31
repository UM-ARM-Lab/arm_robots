#! /usr/bin/env python
import pdb

import rospy
from arm_robots.panda import Panda

if __name__ == '__main__':
    rospy.init_node('panda_motion')
    panda = Panda()
    panda.connect()
    panda.set_execute(False)  # this will stop the robot from actually executing a path, good for testing
    panda.plan_to_pose(panda.panda_1, panda.panda_1_EE, [0.2, 0.25, 1.2, 0.0, 3.14, 1.57], frame_id="world")

    # joint_config = [-0.007018571913499291, -1.3080290837538866, -0.0070561850128466625, -2.598759190173999, -0.008907794188422758, 1.295876371807555, 1.25]
    #
    # panda.plan_to_joint_config('panda_2', joint_config)
