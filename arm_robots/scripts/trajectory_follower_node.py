#! /usr/bin/env python

import argparse
import sys

import numpy as np

import rospy
from arm_robots.get_robot import get_base_robot
from arm_robots.trajectory_follower import TrajectoryFollower


def main():
    np.set_printoptions(linewidth=200, suppress=True, precision=4)
    rospy.init_node('trajectory_follower')

    parser = argparse.ArgumentParser()
    parser.add_argument("robot_name", type=str)
    parser.add_argument("controller_name", type=str)
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    base_robot = get_base_robot(args.robot_name)
    base_robot.connect()

    fwd = TrajectoryFollower(base_robot, args.controller_name)
    fwd.start_server()

    rospy.spin()


if __name__ == "__main__":
    main()
