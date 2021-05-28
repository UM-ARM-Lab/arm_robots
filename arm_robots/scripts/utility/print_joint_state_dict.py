#!/usr/bin/env python
import argparse
import sys
from pprint import pprint

import rospy
from arc_utilities.listener import Listener
from sensor_msgs.msg import JointState


def main():
    rospy.init_node('print_joint_state_dict')

    parser = argparse.ArgumentParser()
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    l = Listener("joint_states", JointState)
    joint_state: JointState = l.get()
    config = {}
    for name, position in zip(joint_state.name, joint_state.position):
        config[name] = position

    pprint(config)


if __name__ == '__main__':
    main()
