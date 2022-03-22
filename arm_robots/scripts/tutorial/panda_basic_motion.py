#! /usr/bin/env python

import rospy
from arm_robots.panda import Panda

if __name__ == '__main__':
    rospy.init_node('panda_motion')
    panda = Panda()
    panda.connect()
    panda.set_execute(False)  # this will stop the robot from actually executing a path, good for testing
    # panda.plan_to_position_cartesian(panda.nebula_arm, panda.nebula_wrist, target_position=[0.04, 0.2867, 1.17206])
    # panda.plan_to_position_cartesian(panda.rocket_arm, panda.rocket_wrist, target_position=[-0.013, -0.04, 1.16967])
    # poking_approach_pose_1 = [0.0, 0.0, 1.07, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_target_pose_1 = [-0.039, 0.0, 1.07, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_approach_pose_2 = [0.0, 0.015, 1.085, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_target_pose_2 = [-0.039, 0.015, 1.085, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_approach_pose_3 = [0.0, 0.015, 1.055, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_target_pose_3 = [-0.039, 0.015, 1.055, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_approach_pose_4 = [0.0, -0.015, 1.055, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_target_pose_4 = [-0.039, -0.015, 1.055, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_approach_pose_5 = [0.0, -0.015, 1.085, np.radians(135), np.radians(-90), np.radians(225)]
    # poking_target_pose_5 = [-0.039, -0.015, 1.085, np.radians(135), np.radians(-90), np.radians(225)]
    # pdb.set_trace()
    # poking_approach_pose_1 = [0.29320, 0.00387, 0.75, 1.0, 0.0, 0.0, 0.0]
    # poking_target_pose_1 = [0.29320, 0.00387, 0.723] #, 1.0, 0.0, 0.0, 0.0]
    # time.sleep(1)
    # panda.plan_to_pose(panda.rocket_arm, "rocket_EE", poking_approach_pose_1, "world")
    # time.sleep(3)
    # panda.plan_to_position_cartesian(panda.rocket_arm, "rocket_EE", poking_target_pose_1, 0.001)
    # time.sleep(3)
    # panda.plan_to_pose(panda.rocket_arm, "rocket_EE", poking_approach_pose_1, "world")
    # time.sleep(3)
    # TODO: Reference med_motion.py for examples.
