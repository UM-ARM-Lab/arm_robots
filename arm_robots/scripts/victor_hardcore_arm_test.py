#! /usr/bin/env python
import argparse
import sys
from time import sleep

import colorama
import numpy as np
import ros_numpy
import roscpp_initializer
import rospy
from geometry_msgs.msg import Quaternion, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from victor_hardware_interface_msgs.msg import ControlMode, MotionCommand

from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.victor import Victor


def wrap_angle(angle_rad):
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def quaternion_msg_from_euler(roll, pitch, yaw):
    return ros_numpy.msgify(Quaternion, np.array(quaternion_from_euler(0, 0, 0)))


def mirror_right_to_left(q):
    q_new = q.copy()
    q_new[0] = -wrap_angle(q[0] + np.pi)
    q_new[1] = -q[1]
    q_new[2] = -(q[3] + np.pi / 2)
    q_new[3] = -q[3]
    q_new[4] = -q[4]
    q_new[5] = -q[5]
    q_new[6] = -q[6]
    return q_new


def main():
    np.set_printoptions(suppress=True, precision=2, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("hardcore_test")

    parser = argparse.ArgumentParser()
    parser.add_argument('arm', choices=['left', 'right'])
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    victor = Victor()
    victor.connect()

    victor.open_gripper(args.arm)
    rospy.sleep(1)
    victor.close_gripper(args.arm)
    rospy.sleep(1)

    test joint trajectories
    for control_mode in [ControlMode.JOINT_POSITION, ControlMode.JOINT_IMPEDANCE]:
        victor.move_to_impedance_switch(actually_switch=False)
        victor.set_control_mode(control_mode)
        test_joint_trajectories(victor, args.arm)

    group = args.arm + '_arm'
    victor.set_control_mode(ControlMode.JOINT_POSITION, vel=0.1)
    pose = Pose()
    pose.position.x = 0.7
    pose.position.y = -0.1
    pose.position.z = 1.0
    q = quaternion_from_euler(np.pi, 0, 0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    victor.plan_to_pose(group, 'right_tool', pose)
    victor.set_control_mode(ControlMode.CARTESIAN_IMPEDANCE, vel=40)

    current_pose: Pose = victor.right_arm_status_listener.get().measured_cartesian_pose
    current_pose.position.x += 0.1

    cmd = MotionCommand()
    cmd.control_mode.mode = ControlMode.CARTESIAN_IMPEDANCE
    cmd.header.frame_id = 'victor_right_arm_world_frame_kuka'
    cmd.cartesian_pose = current_pose
    euler_from_quaternion(ros_numpy.numpify(current_pose.orientation))

    victor.right_arm_command_pub.publish(cmd)
    sleep(2.0)


    victor.set_control_mode(ControlMode.JOINT_POSITION, vel=0.1)
    victor.plan_to_joint_config(group, 'zero')

    roscpp_initializer.shutdown()


def check_torque_sensors(victor: Victor, arm: str):
    force = victor.get_total_joint_torque(arm)
    if force < 10:
        rospy.logerr(f"Measured force {force} is very low!")


def test_joint_trajectories(victor: Victor, arm: str):
    group = arm + '_arm'
    victor.plan_to_joint_config(group, 'zero')
    q = np.deg2rad([-66, 60, 47, -38, 74, 53, 90.0])
    if arm == 'right':
        q = mirror_right_to_left(q)
    victor.plan_to_joint_config(group, q)
    for i in range(7):
        q_lower = q.copy()
        q_lower[i] -= 0.5

        q_upper = q.copy()
        q_upper[i] += 0.5

        victor.plan_to_joint_config(group, q_lower)
        check_torque_sensors(victor, arm)
        victor.plan_to_joint_config(group, q_upper)
        check_torque_sensors(victor, arm)

    victor.plan_to_joint_config(group, 'zero')


if __name__ == "__main__":
    main()
