#!/usr/bin/env python
import argparse
import sys
from threading import Thread, Lock
from typing import List

import numpy as np
from inputs import get_gamepad

import ros_numpy
import rospy
from arc_utilities import ros_init
from arm_robots.get_robot import get_moveit_robot
from arm_robots.logitech import Logitech
from tf.transformations import quaternion_from_euler


class CartesianTeleop:
    """
    use the roslaunch file to start this and the joy node. Then use the D-pad and left joystick to move in XYZ
    The "x" button (on logitech controller) will switch which tool name is being controller

    """
    def __init__(self, robot_name: str, group_name: str, tool_names: List[str]):
        self.gamepad = Logitech()
        self.group_name = group_name
        self.tool_names = tool_names

        self.robot = get_moveit_robot(robot_name)
        self.robot.connect()

        self.robot.store_current_tool_orientations(self.tool_names)

        self.current_tool_idx = 0
        self.delta_scale_factor = 0.01

        self.should_disconnect = False
        self.buttons_thread = Thread(target=self.buttons_thread_func)
        self.buttons_thread.start()

        self.current_tool_idx_mutex = Lock()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.send_commands()
            r.sleep()

        self.should_disconnect = True

    def get_current_tool_positions(self):
        target_positions = []
        for tool_name in self.tool_names:
            current_tool_pose = self.robot.get_link_pose(tool_name)
            current_tool_position = ros_numpy.numpify(current_tool_pose.position)
            target_positions.append(current_tool_position)
        return target_positions

    def buttons_thread_func(self):
        while not self.should_disconnect:
            events = get_gamepad()
            for event in events:
                if self.gamepad.x_clicked(event):
                    with self.current_tool_idx_mutex:
                        self.current_tool_idx += 1
                        if self.current_tool_idx == len(self.tool_names):
                            self.current_tool_idx = 0

    def send_commands(self):
        delta_position = self.gamepad.get_3d_delta()
        target_positions = self.get_current_tool_positions()

        with self.current_tool_idx_mutex:
            target_positions[self.current_tool_idx] += delta_position * self.delta_scale_factor

            tool_name = self.tool_names[self.current_tool_idx]

            target_position = target_positions[self.current_tool_idx]

        rospy.logdebug(f"Controlling tool {tool_name}, target position: {target_position}")
        target_positions = np.expand_dims(target_positions, axis=1)
        self.robot.follow_jacobian_to_position(self.group_name, self.tool_names, target_positions, vel_scaling=1.0)


@ros_init.with_ros("cartesian_teleop")
def main():
    np.set_printoptions(precision=3, suppress=True)

    parser = argparse.ArgumentParser("use a joystick to move the arms in x/y/z")
    parser.add_argument("robot_name", type=str, help="robot name, e.g. 'victor' or 'val'")
    parser.add_argument("group_name", type=str, help="moveit group name, e.g. 'both_arms'")
    parser.add_argument("tool_names", type=str, help="the links you want to move, e.g. 'left_tool'", nargs='+')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    ct = CartesianTeleop(args.robot_name, args.group_name, args.tool_names)
    rospy.spin()


if __name__ == '__main__':
    main()
