#! /usr/bin/env python
import rospy
from argparse import ArgumentParser
from arc_utilities import ros_init
from colorama import Fore
from arm_robots.get_robot import get_moveit_robot


def move_to(robot, group_name, position_name):
    robot.speak(f"Moving to {position_name}")
    rospy.sleep(3.0)
    robot.plan_to_joint_config(group_name, position_name)


@ros_init.with_ros("move_to")
def main():
    p = ArgumentParser()
    p.add_argument("robot", help="Moveit name of a joint group")
    # TODO: Look up group name automatically from position name
    p.add_argument("group_name", help="The name of the moveit joint group")
    p.add_argument("position_name", help="Name of the stored group state")

    args = p.parse_args(rospy.myargv()[1:])
    robot = get_moveit_robot(args.robot)
    robot.connect()

    move_to(robot, args.group_name, args.position_name)


if __name__ == "__main__":
    main()
