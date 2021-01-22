#! /usr/bin/env python
import time

import colorama
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.hdt_michigan import BaseVal

"""
This scipt calculates the Kuka arms max joint speeds and max joint accels.
These values should be set in the Kuka "machine data" (as per the kuka manual), however I (Brad) cannot find it
"""

joints_to_use = ['joint45']


def calc_kuka_speed_for_joint(val: BaseVal, joint_name: str, motion_distance=0.5):
    def reached_endpoint(target_position):
        current_position = val.get_joint_positions([joint_name])[0]
        return abs(current_position - target_position) < 0.01

    def send(position):
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [100]  # hdt will just move as fast as it can here
        val.send_joint_command(joint_names=[joint_name], trajectory_point=point)

    def move(position):
        send(position)
        while not reached_endpoint(position):
            time.sleep(0.1)

    # go to start
    start_position = 0
    move(start_position)

    # start moving
    send(start_position + motion_distance)

    # measure the speed once we've moved a bit
    measured_speeds = []
    while not reached_endpoint(start_position + motion_distance):
        state: JointState = val._joint_state_listener.get()
        idx = state.name.index(joint_name)
        measured_velocity = state.velocity[idx]
        measured_speeds.append(np.abs(measured_velocity))
        time.sleep(0.1)

    measured_velocity = np.max(measured_speeds)
    print(f'{joint_name}: {measured_velocity}')
    return measured_velocity


def calc_kuka_accel_for_joint(val, joint_number: int, relative_accel=0.1, motion_distance=0.2):
    val.speak("Moving to zero position")

    def set_setup_control_mode():
        val.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=0.2, accel=0.1)

    def move(config):
        val.plan_to_joint_config(arm_to_use, config)

    def reached_endpoint():
        return abs(val.get_joint_positions_map()[joints_to_use[joint_number]] - motion_distance) < 0.01

    p_start = (0, 0, 0, 0, 0, 0, 0)
    p = list(p_start)
    set_setup_control_mode()
    move(p)
    p[joint_number] = -motion_distance
    move(p)
    p[joint_number] = motion_distance
    # val.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=1.0, accel=relative_accel)
    val.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=1, accel=0.01)
    time.sleep(1.0)
    t0 = time.time()
    val.send_joint_command(joints_to_use, JointTrajectoryPoint(positions=p))
    while not reached_endpoint():
        time.sleep(1e-5)
    dt = time.time() - t0
    print(dt)
    average_speed = motion_distance * 2 / dt
    if average_speed > KUKA_MAX_JOINT_VELOCITIES[joint_number] / 2:
        print(f"{colorama.Fore.RED}Average speed of {average_speed} is more than half of max speed of "
              f"{KUKA_MAX_JOINT_VELOCITIES[joint_number]}\n"
              f"Estimated acceleration is not accurate{colorama.Style.RESET_ALL}")
    accel = average_speed * 4 / dt
    print(f"Estimated accel is {accel} rad/s^2 or {accel * 180 / np.pi} deg/s^2")

    # print(f"Kuka joint {joint_number} max speed is {speed} rad/s = {speed*180 / math.pi} deg/s")
    set_setup_control_mode()
    move(p_start)
    return accel * 180 / np.pi


def calc_max_joint_speeds(val):
    joint_speeds = {}
    for joint_num in range(7):
        joint_speeds[joint_num] = calc_kuka_speed_for_joint(val, joint_num, relative_vel=0.05)

    print(f"Kuka max joint speeds (deg/s): {[joint_speeds[i] for i in range(7)]}")


def calc_max_joint_accel(val):
    # for motion_distance in [0.05, 0.1, 0.2, 0.3, 0.4]:
    #     calc_kuka_accel_for_joint(val, 0, relative_accel=0.00001, motion_distance=motion_distance)
    # calc_kuka_accel_for_joint(val, 0, relative_accel=0.1, motion_distance=1.3)
    joint_motion_distances = {
        0: 1.2,
        1: 0.5,
        2: 0.1,
        3: 0.2,
        4: 0.1,
        5: 0.1,
        6: 0.1
    }

    joint_accels = {}
    for joint_num, motion_distance in joint_motion_distances.items():
        print(f"Testing joint {joint_num} with distance {motion_distance}")
        joint_accels[joint_num] = calc_kuka_accel_for_joint(val, joint_num, motion_distance=motion_distance)

    print(f"Kuka max joint accels (deg/s^2): {[joint_accels[i] for i in range(7)]}")


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("calc_hdt_max_joint_speeds")

    val = BaseVal('hdt_michigan')

    joints = [
        # 'joint41',
        # 'joint42',
        # 'joint43',
        # 'joint44',
        # 'joint45',
        # 'joint46',
        # 'joint47',
        # 'leftgripper'
    ]
    for joint in joints:
        calc_kuka_speed_for_joint(val, joint)


if __name__ == "__main__":
    main()
