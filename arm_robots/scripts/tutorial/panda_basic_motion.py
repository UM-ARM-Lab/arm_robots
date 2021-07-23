#! /usr/bin/env python

import rospy
from arm_robots.panda import Panda

if __name__ == '__main__':
    rospy.init_node('panda_motion')
    panda = Panda()
    panda.connect()
    panda.set_execute(False) # this will stop the robot from actually executing a path, good for testing
    # panda.plan_to_position_cartesian(panda.nebula_arm, panda.nebula_wrist, target_position=[0.04, 0.2867, 1.17206])
    # panda.plan_to_position_cartesian(panda.rocket_arm, panda.rocket_wrist, target_position=[-0.013, -0.04, 1.16967])

    # TODO: Reference med_motion.py for examples.