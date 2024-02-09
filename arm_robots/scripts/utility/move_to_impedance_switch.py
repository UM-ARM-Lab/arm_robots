#! /usr/bin/env python

"""
This moves Victor's arms to a configuration where impedance validations tends to work well.
if you want to actually change the control mode, not just move to the position, add the argument --actually-switch
"""

import argparse

import rclpy
from rclpy.node import Node

from arm_robots.victor import Victor
from arm_robots.victor_planning import move_to_group_state
from victor_hardware_interfaces.msg import ControlMode


class MoveToImpedanceSwitch(Node):

    def __init__(self, actually_switch=False):
        super().__init__('move_to_impedance_switch')
        victor = Victor(self)

        move_to_group_state(victor, 'both_arms', 'impedance_switch')

        if actually_switch:
            victor.set_control_mode(ControlMode.JOINT_IMPEDANCE)


def main():
    rclpy.init()

    parser = argparse.ArgumentParser(description='Move Victor to a configuration where impedance control works well')
    parser.add_argument('--actually-switch', action='store_true', help='Actually switch to impedance control')

    args = parser.parse_args()

    node = MoveToImpedanceSwitch(args.actually_switch)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
