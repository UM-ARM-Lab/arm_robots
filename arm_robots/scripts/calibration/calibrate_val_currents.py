#!/usr/bin/env python
import argparse
import sys

import numpy as np

import rospy
from arc_utilities import ros_init
from arm_robots import hdt_michigan
from arm_robots.get_robot import get_moveit_robot
from arm_robots.hdt_michigan import Val
from arm_robots.robot_utils import robot_state_from_joint_state_and_joint_names
from geometry_msgs.msg import Wrench
from hdt_robot.hdt_michigan_control.scripts.joint_state_filter import torque_to_current
from sensor_msgs.msg import JointState

configs = [
    {
        'joint1':  -5.396254512529697,
        'joint2':  -2.0917040017045907,
        'joint3':  1.1320173830316378,
        'joint4':  0.6948121410710071,
        'joint41': 1.4709404538175175,
        'joint42': 0.17354700209999763,
        'joint43': 1.242620003074972,
        'joint44': 0.32588109590641956,
        'joint45': -4.437416829789349,
        'joint46': 1.4157287964298142,
        'joint47': -0.3404874152922428,
        'joint5':  1.1644405919722072,
        'joint56': 3.3896842525749094e-05,
        'joint57': 0.8685931170422307,
        'joint6':  0.9165488097014061,
        'joint7':  -2.9943833783015243,
    },
    {
        'joint1':  -2.6612617797877256,
        'joint2':  -2.876976196373282,
        'joint3':  -5.050329412489545,
        'joint4':  0.5412567824670225,
        'joint41': -1.4354763202901268,
        'joint42': 2.099184977028287,
        'joint43': -3.229694064876136,
        'joint44': 0.5410216888899573,
        'joint45': -2.9959781788114843,
        'joint46': 1.570662356844907,
        'joint47': 2.4587953415870603,
        'joint5':  5.583720377954102,
        'joint56': 8.89590759758363e-05,
        'joint57': 3.0600670708480493e-07,
        'joint6':  -1.269048864797909,
        'joint7':  4.340814183733032,
    },
    {
        'joint1':  -5.23858171079653,
        'joint2':  -0.48260437669564693,
        'joint3':  -2.4225177461557514,
        'joint4':  -2.6699136059242528,
        'joint41': -4.764212018583838,
        'joint42': 0.6067729367952497,
        'joint43': -6.283091732044968,
        'joint44': 0.305652502762503,
        'joint45': -3.016978314280272,
        'joint46': -1.5235572798586663,
        'joint47': -5.934199295853484,
        'joint5':  -0.6250939361853338,
        'joint56': -6.1164234761434955e-06,
        'joint57': -2.1604079892156847e-05,
        'joint6':  0.8042605670638387,
        'joint7':  -3.7645542794274234,
    },
    {
        'joint1':  -4.1818931972850795,
        'joint2':  0.14171963053558478,
        'joint3':  2.086062490131032,
        'joint4':  -0.3857336085975751,
        'joint41': 0.04535083273771434,
        'joint42': -0.1499266943636357,
        'joint43': 0.5073477061444933,
        'joint44': 0.6549128673986102,
        'joint45': -5.396934402873609,
        'joint46': 0.5203856383024918,
        'joint47': -2.564925856904623,
        'joint5':  0.5837782829440457,
        'joint56': -0.7450522877159238,
        'joint57': -0.4525343606186265,
        'joint6':  -1.4249931960926188,
        'joint7':  -3.4529867974533848,
    }
]


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
    done = False

    def _joint_state_cb(joint_state: JointState):
        nonlocal names

        if done:
            return

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

    # input("press enter to make the robot move around")
    val = Val(raise_on_failure=True)
    val.connect()

    sub = rospy.Subscriber(val.joint_states_topic, JointState, _joint_state_cb)

    val.plan_to_joint_config('both_arms', 'home')
    for i, config in enumerate(configs):
        print(i)
        val.plan_to_joint_config('both_arms', config)

    done = True

    if names is None:
        print("No joint states received? Aborting.")
        return

    data = np.array(data)
    np.save('calibrate_val_currents.npy', data)
    np.save('names.npy', names)

    import matplotlib.pyplot as plt
    import scipy.stats

    data = np.load('calibrate_val_currents.npy')
    names = np.load('names.npy')

    motor_constants = {}
    for i, name in enumerate(names):
        data_for_name = data[:, i]
        plt.figure()
        torques = data_for_name[:, 0]
        currents = data_for_name[:, 1]
        plt.scatter(torques, currents)
        plt.xlabel("torque")
        plt.ylabel("current")
        plt.title(f"Joint {name}")
        plt.savefig(f"{name}.png")

        # compute linear relationship
        result = scipy.stats.linregress(torques, currents)
        motor_constants[name] = (result.slope, result.intercept)

    motor_constants['joint46'] = (10.0, 0.0)
    motor_constants['joint47'] = (10.0, 0.0)
    motor_constants['joint7'] = (10.0, 0.0)

    print(motor_constants)

    plt.show()

    max_currents = {
        "joint1":        0.0,
        "joint2":        0.0,
        "joint3":        0.0,
        "joint4":        0.0,
        "joint41":       0.0,
        "joint42":       0.0,
        "joint43":       0.0,
        "joint44":       0.0,
        "joint45":       0.0,
        "joint46":       0.0,
        "joint47":       0.0,
        "joint5":        0.0,
        "joint56":       0.0,
        "joint57":       0.0,
        "joint6":        0.0,
        "joint7":        0.0,
        "leftgripper":   0.0,
        "leftgripper2":  0.0,
        "rightgripper":  0.0,
        "rightgripper2": 0.0,
    }

    def _joint_state_cb(joint_state):
        torque_state = hdt_michigan.estimated_torques(joint_state, val)
        estimated_torques = torque_state.joint_state.effort

        for joint_name, measured_current in zip(joint_state.name, joint_state.effort):
            if joint_name == 'joint46':
                # NOTE: joint46 is giving bogus current data, so just ignore it and let it do whatever it wants
                # this means if you strain ONLY joint46 it might just break itself.
                continue
            if joint_name not in torque_state.joint_state.name:
                # ignore grippers or other joints we don't have data on
                continue
            joint_idx = torque_state.joint_state.name.index(joint_name)
            estimated_torque = estimated_torques[joint_idx]
            estimated_current = torque_to_current(joint_name, estimated_torque)
            external_current = measured_current - estimated_current
            external_current = abs(external_current)
            if external_current > max_currents[joint_name]:
                max_currents[joint_name] = external_current

    sub = rospy.Subscriber(val.joint_states_topic, JointState, _joint_state_cb)

    val.plan_to_joint_config('both_arms', 'home')
    for i, config in enumerate(configs):
        print(i)
        val.plan_to_joint_config('both_arms', config)

    max_currents['joint46'] = 999
    print({k: np.ceil(v) for k, v in max_currents.items()})


if __name__ == '__main__':
    main()
