#! /usr/bin/env python

import rospy
from arm_robots.panda import Panda

if __name__ == '__main__':
    rospy.init_node('panda_motion')
    panda = Panda()
    panda.connect()

    # TODO: Reference med_motion.py for examples.