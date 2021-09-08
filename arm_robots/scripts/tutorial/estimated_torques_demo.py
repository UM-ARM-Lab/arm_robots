#!/usr/bin/env python
import argparse
import sys

import rospy
from arc_utilities import ros_init
from arm_robots.get_robot import get_moveit_robot
from arm_robots.hdt_michigan import estimated_torques
from arm_robots.robot_utils import robot_state_from_joint_state_and_joint_names
from geometry_msgs.msg import Wrench
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

torque_viz_scales = {
    "joint56": 500.0,
    "joint57": 50.0,
    "joint1":  10.0,
    "joint2":  40.0,
    "joint3":  100.0,
    "joint4":  100.0,
    "joint5":  100.0,
    "joint6":  1000.0,
    "joint7":  1000.0,
    "joint41": 10.0,
    "joint42": 40.0,
    "joint43": 100.0,
    "joint44": 100.0,
    "joint45": 100.0,
    "joint46": 1000.0,
    "joint47": 1000.0,
}


@ros_init.with_ros("estimated_torques_demo")
def main():
    """
    Run `rosrun joint_state_publisher_gui joint_state_publisher_gui`
    Then run this script
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-namespace', default='hdt_michigan')

    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    robot = get_moveit_robot(args.robot_namespace)

    def _joint_state_cb(joint_state: JointState):
        robot_state = estimated_torques(joint_state, robot)

        for name, scale_factor in torque_viz_scales.items():
            i = robot_state.joint_state.name.index(name)
            robot_state.joint_state.effort[i] *= scale_factor

        pub.publish(robot_state.joint_state)

    pub = rospy.Publisher("estimated_torques", JointState, queue_size=10)
    sub = rospy.Subscriber("joint_states", JointState, _joint_state_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
