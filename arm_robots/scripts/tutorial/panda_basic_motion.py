#! /usr/bin/env python
import pdb

import rospy
import numpy as np
from arm_robots.panda import Panda

if __name__ == '__main__':
    rospy.init_node('panda_motion')
    panda = Panda()
    panda.connect()
    print("Panda connected! Planning now...")
    # pdb.set_trace()
    panda.set_execute(False)  # this will stop the robot from actually executing a path, good for testing
    if not panda.execute:
        print("Real execution disabled.")
    # pose_1_start = [0.07, 0.24, 1.3, -0.7071068, 0.7071068, 0, 0]
    # pose_2_start = [0.07, 0.24, 1.1, np.radians(-90), np.radians(-90), 0]
    # panda.plan_to_pose(panda.panda_1, panda.panda_1_EE, pose_1_start, frame_id="world")
    # panda.plan_to_pose(panda.panda_2, panda.panda_2_EE, pose_2_start, frame_id="world")

    pose_1_end = [0.07, 0.24, 1.2, -0.7071068, 0.7071068, 0, 0]
    panda.plan_to_position_cartesian(panda.panda_1, panda.panda_1_EE, pose_1_end[:3])
    pdb.set_trace()
    # joint_config = [-0.007018571913499291, -1.3080290837538866, -0.0070561850128466625, -2.598759190173999, -0.008907794188422758, 1.295876371807555, 1.25]
    #
    # panda.plan_to_joint_config('panda_2', joint_config)
