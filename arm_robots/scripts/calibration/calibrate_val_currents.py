#!/usr/bin/env python
import argparse
import sys

import numpy as np

import rospy
from arc_utilities import ros_init
from arm_robots.get_robot import get_moveit_robot
from arm_robots.hdt_michigan import Val
from geometry_msgs.msg import Wrench
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


def robot_state_from_joint_state_and_joint_names(joint_names, joint_state):
    robot_state = RobotState()
    for name in joint_names:
        i = joint_state.name.index(name)
        p = joint_state.position[i]
        robot_state.joint_state.name.append(name)
        robot_state.joint_state.position.append(p)
        robot_state.joint_state.velocity.append(0)
    return robot_state


@ros_init.with_ros("calibrate_val_currents")
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-namespace', default='hdt_michigan')

    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    robot = get_moveit_robot(args.robot_namespace)
    group_name1 = "left_side"
    group_name2 = "right_side"
    n_links = 14
    wrenches = [Wrench()] * n_links

    data = []

    names = None

    def _joint_state_cb(joint_state: JointState):
        nonlocal names
        joint_names1 = robot.robot_commander.get_active_joint_names(group_name1)
        robot_state1 = robot_state_from_joint_state_and_joint_names(joint_names1, joint_state)
        torques1 = robot.estimated_torques(robot_state1, group_name1, wrenches)

        joint_names2 = robot.robot_commander.get_active_joint_names(group_name2)
        robot_state2 = robot_state_from_joint_state_and_joint_names(joint_names2, joint_state)
        torques2 = robot.estimated_torques(robot_state2, group_name2, wrenches)

        if torques1 is None or torques2 is None:
            return

        names = sorted(set(robot_state1.joint_state.name + robot_state2.joint_state.name))

        datum = []
        for name in names:
            j = joint_state.name.index(name)
            measured_current = joint_state.effort[j]
            try:
                i = robot_state1.joint_state.name.index(name)
                predicted_torque = torques1[i]
            except ValueError:
                pass
            try:
                i = robot_state2.joint_state.name.index(name)
                predicted_torque = torques2[i]
            except ValueError:
                pass

            datum.append([predicted_torque, measured_current])
            del predicted_torque
            del measured_current
        data.append(datum)

    sub = rospy.Subscriber("joint_states", JointState, _joint_state_cb)

    print("press enter to make the robot move around")
    val = Val(raise_on_failure=True)
    val.connect()
    val.plan_to_joint_config('both_arms', 'home')

    rospy.signal_shutdown('done collecting data')
    while not rospy.is_shutdown():
        pass

    data = np.array(data)
    np.save('calibrate_val_currents.npy', data)
    np.save('names.npy', names)


def analyze():
    import matplotlib.pyplot as plt
    import scipy.stats

    data = np.load('calibrate_val_currents.npy')
    names = np.load('names.npy')

    for i, name in enumerate(names):
        data_for_name = data[:, i]
        plt.figure()
        torques = data_for_name[:, 0]
        currents = data_for_name[:, 1]
        plt.scatter(torques, currents)
        plt.xlabel("torque")
        plt.ylabel("current")
        plt.title(f"Joint {name}")

        # compute linear relationship
        result = scipy.stats.linregress(torques, currents)
        print(name, result.slope, result.intercept, result.rvalue)

    plt.show()


if __name__ == '__main__':
    main()
    analyze()
