#!/usr/bin/env python
import argparse
import sys

import rospy
from arc_utilities import ros_init
from arm_robots.get_robot import get_moveit_robot
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


def robot_state_from_joint_state_and_joint_names(joint_names1, joint_state):
    robot_state1 = RobotState()
    for name in joint_names1:
        i = joint_state.name.index(name)
        p = joint_state.position[i]
        robot_state1.joint_state.name.append(name)
        robot_state1.joint_state.position.append(p)
        robot_state1.joint_state.velocity.append(0)
    return robot_state1


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
    group_name1 = "left_side"
    group_name2 = "right_side"
    n_links = 14
    wrenches = [Wrench()] * n_links

    def _joint_state_cb(joint_state: JointState):
        joint_names1 = robot.robot_commander.get_active_joint_names(group_name1)
        robot_state1 = robot_state_from_joint_state_and_joint_names(joint_names1, joint_state)
        torques1 = robot.estimated_torques(robot_state1, group_name1, wrenches)

        joint_names2 = robot.robot_commander.get_active_joint_names(group_name2)
        robot_state2 = robot_state_from_joint_state_and_joint_names(joint_names2, joint_state)
        torques2 = robot.estimated_torques(robot_state2, group_name2, wrenches)

        if torques1 is None or torques2 is None:
            return

        robot_state = RobotState()
        names = sorted(set(robot_state1.joint_state.name + robot_state2.joint_state.name))
        for name in names:
            p = None
            e = None
            try:
                i = robot_state1.joint_state.name.index(name)
                p = robot_state1.joint_state.position[i]
                e = torques1[i]
            except ValueError:
                pass
            try:
                i = robot_state2.joint_state.name.index(name)
                p = robot_state2.joint_state.position[i]
                e = torques2[i]
            except ValueError:
                pass
            robot_state.joint_state.name.append(name)
            robot_state.joint_state.position.append(p)
            robot_state.joint_state.effort.append(e)

        for name, scale_factor in torque_viz_scales.items():
            i = robot_state.joint_state.name.index(name)
            robot_state.joint_state.effort[i] *= scale_factor

        pub.publish(robot_state.joint_state)

    pub = rospy.Publisher("estimated_torques", JointState, queue_size=10)
    sub = rospy.Subscriber("joint_states", JointState, _joint_state_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
