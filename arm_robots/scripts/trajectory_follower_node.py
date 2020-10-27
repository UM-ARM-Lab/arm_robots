#! /usr/bin/env python

import numpy as np

import sys
import rospy
import argparse
from arm_robots.hdt_michigan import BaseVal
from arm_robots.trajectory_follower import TrajectoryFollower
from arm_robots.victor import BaseVictor




def main():
    np.set_printoptions(linewidth=200, suppress=True, precision=4)
    rospy.init_node('trajectory_follower')

    parser = argparse.ArgumentParser()
    parser.add_argument("robot_name", type=str)
    parser.add_argument("controller_name", type=str)
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    if args.robot_name == "victor":
        base_robot = BaseVictor(args.robot_name)
    elif args.robot_name == "val":
        base_robot = BaseVal(args.robot_name)
    else:
        raise NotImplementedError(f"Invalid robot name {args.robot_name}")
    fwd = TrajectoryFollower(base_robot, args.controller_name)
    fwd.start_server()

    rospy.spin()


if __name__ == "__main__":
    main()
