#! /usr/bin/env python
import rospy
from argparse import ArgumentParser
from arc_utilities import ros_init
from colorama import Fore
from arm_robots.get_robot import get_moveit_robot


def print_current_position_xml(robot, group_name, position_name):
    joint_names = robot.get_joint_names("both_arms")
    positions = robot.get_joint_positions(joint_names)

    print(f"\n{Fore.GREEN}Copy this code to the moveit srdf file "
          f"(e.g. victor_moveit_config/config/victor.srdf){Fore.RESET}\n")

    print(f'<group_state name="{position_name}" group="{group_name}">')
    for name, pos in zip(joint_names, positions):
        print(f'    <joint name="{name}" value="{pos:.3f}"/>')
    print(f'</group_state>')


@ros_init.with_ros("save_named_position")
def main():
    p = ArgumentParser()
    p.add_argument("robot", help="Moveit name of a joint group")
    p.add_argument("joint_group", help="Moveit name of a joint group")
    p.add_argument("position_name", help="Name of the new position")

    args = p.parse_args(rospy.myargv()[1:])


    robot = get_moveit_robot(args.robot)

    print_current_position_xml(robot, args.joint_group, args.position_name)


if __name__ == "__main__":
    main()
