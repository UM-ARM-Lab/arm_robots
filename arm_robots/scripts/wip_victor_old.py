#! /usr/bin/env python
import math

import colorama
import numpy as np
import roscpp_initializer

import rospy
import sensor_msgs
from arc_utilities.listener import Listener
from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.victor import Victor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import ControlMode, MotionStatus


ask_before_moving = False


def myinput(msg):
    global ask_before_moving
    if ask_before_moving:
        input(msg)


NAMED_POSITIONS = {
    "impedance switch": {"right_arm": [0.724, 0.451, 0.94, -1.425, 0.472, 0.777, -0.809],
                         "left_arm": [-0.694, 0.14, -0.229, -1.11, -0.512, 1.272, 0.077]},
    "impedance wrist flick": {"right_arm": [0.724, 0.451, 0.94, -1.425, -0.472, -0.777, +0.809],
                              "left_arm": [-0.694, 0.14, -0.229, -1.11, 0.512, -1.272, -0.077]},
    "arms up": {"left_arm": [-np.pi / 2, np.pi / 2, 0, 0, 0, 0, 0],
                "right_arm": [np.pi / 2, np.pi / 2, 0, 0, 0, 0, 0]},
    "hug": {"right_arm": [0.311, 1.561, 0.282, -1.296, 0.137, 0.493, 0.112],
            "left_arm": [0.604, 1.568, -0.021, -1.164, 0.355, 0.173, 0.297]},
    "arms out": {"left_arm": [0, 0, 0, 0, 0, 0, 0],
                 "right_arm": [0, 0, 0, 0, 0, 0, 0]},
    "handshake": {"right_arm": [-0.336, 0.105, 0.061, -1.151, -0.01, 0.577, -0.675]},
    "flex": {"left_arm": [-0.229, -0.122, -1.215, -1.562, -0.227, 1.572, -0.119],
             "right_arm": [1.247, -0.751, 1.119, 1.401, 0.281, -1.382, -1.069]},
    "arms at side": {"left_arm": [-1.27, -1.227, -0.309, 0.36, -0.585, 0.496, 0.105],
                     "right_arm": [1.144, -1.189, 0.59, 0.292, 0.296, 0.265, -0.809]}
}



def main():
    # np.set_printoptions(suppress=True, precision=2, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("basic_motion")

    victor = Victor()
    victor.connect()

    rospy.sleep(1)
    victor.open_left_gripper()
    victor.open_right_gripper()
    rospy.sleep(0.5)
    victor.close_left_gripper()
    victor.close_right_gripper()


    rospy.sleep(0.5)
    # victor.move_to("impedance switch")
    vel_scale = 1.0
    # victor.set_control_mode(control_mode=ControlMode.JOINT_IMPEDANCE, vel=vel_scale, accel=1)
    victor.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=vel_scale, accel=1.0)

    def both_arm_config(left_arm_config, right_arm_config):
        assert len(left_arm_config) == 7, "Left arm config must be length 7"
        assert len(right_arm_config) == 7, "Right arm config must be length 7"
        return left_arm_config + right_arm_config

    def move_to(position_name):
        if position_name not in NAMED_POSITIONS:
            raise ValueError(f'{position_name} is not a known position for victor')

        position = NAMED_POSITIONS[position_name]
        if "left_arm" in position and "right_arm" in position:
            victor.plan_to_joint_config("both_arms", both_arm_config(left_arm_config=position["left_arm"],
                                                                     right_arm_config=position["right_arm"]))

        else:
            for joint_group, configs in position.items():
                victor.plan_to_joint_config(joint_group, configs)

    # victor.move_to_impedance_switch(new_relative_velocity=0.5)
    move_to("impedance switch")
    move_to("handshake")
    move_to("flex")
    move_to("impedance switch")
    move_to("arms at side")
    move_to("impedance switch")
    move_to("impedance wrist flick")
    move_to("impedance switch")
    # p1 = (0.724, 0.451, 0.94, -1.425, 0.472, -0.777, -1.0)
    # p2 = (0.724, 0.451, 0.94, -1.425, 0.472, -0.777, +1.0)
    p1 = (-0.5, 0.451, 0.94, -1.425, 0.472, -0.777, -1.0)
    p2 = (0.5, 0.451, 0.94, -1.425, 0.472, -0.777, -1.0)

    p1_jtp = JointTrajectoryPoint(positions=p1, velocities=[0.0] * 7)
    p2_jtp = JointTrajectoryPoint(positions=p2, velocities=[0.0] * 7)
    p1_vel_jtp = JointTrajectoryPoint(positions=list(p1), velocities=[0.0] * 7)
    p1_vel_jtp.positions[6] = p1_vel_jtp.positions[6] + 0.5
    # p1_vel_jtp.velocities[6] = 0.4

    listener = Listener("/victor/right_arm/motion_status", MotionStatus)
    js_listener = Listener("/victor/joint_states", JointState)

    # victor.send_joint_command(RIGHT_ARM_JOINT_NAMES, JointTrajectoryPoint(positions=p1))
    # while not np.all(np.array(victor.get_joint_positions(RIGHT_ARM_JOINT_NAMES)) - np.array(p1) < 0.01):
    #     time.sleep(1e-5)
    # # rospy.sleep(3)
    # total_t0 = time.time()
    # step_size = 0.1
    # # for i in range(21):
    # end = 1.0
    # interp_vals = list(np.arange(0.0, end + step_size, step_size))
    # joint = 0
    # for i, val in enumerate(interp_vals):
    #     # print(val)
    #     px = list(p1)
    #     px[0] += val
    #     velocities = [0.0] * 7
    #     # velocities = [0, 0, 0, 0, 0, 0, .78]
    #     if 1 <= i <= len(interp_vals) - 2:
    #         velocities[0] = vel_scale * victor_config.KUKA_MAX_JOINT_VELOCITIES[joint] * .95
    #         # velocities[6] = 1.5
    #         # velocities[6] = 3
    #
    #     victor.send_joint_command(RIGHT_ARM_JOINT_NAMES, JointTrajectoryPoint(positions=px, velocities=velocities))
    #     # while listener.get()
    #     # rospy.sleep(0.127)
    #     # rospy.sleep(0.15)
    #     t0 = time.time()
    #     # while listener.get().measured_joint_position.joint_7 < px[6] - .1:
    #     while victor.get_joint_positions([RIGHT_ARM_JOINT_NAMES[joint]])[0] < px[joint] - step_size:
    #     # while js_listener.get().position[13] < px[6] - .1:
    #         time.sleep(1e-5)
    #     time.sleep(0.05)
    #     print(f"waited {time.time() - t0}s")
    #     # time.sleep(0.01)
    #
    # print(f"Total time {time.time() - total_t0}")
    # victor.send_joint_command(RIGHT_ARM_JOINT_NAMES, p1_jtp)
    # rospy.sleep(5)

    # victor.send_joint_command(right_arm_joints, p1_vel_jtp)
    # victor.send_joint_command(right_arm_joints, p1_jtp)


    # Loopback time test
    # victor.send_joint_command(right_arm_joints, p1_vel_jtp)
    # rospy.sleep(1.0)
    # print("Sending joint command")
    # victor.send_joint_command(right_arm_joints, p1_jtp)
    # t0 = time.time()
    # while not math.isclose(listener.get().commanded_joint_position.joint_7, p1[6], abs_tol=1e-5):
    #     rospy.sleep(1e-6)
    # print(f"waiting took {time.time() - t0}")


    # victor.send_joint_command(right_arm_joints, p1_jtp)
    # rospy.sleep(10)
    # victor.send_joint_command(right_arm_joints, p2_jtp)
    # rospy.sleep(10)
    # victor.send_joint_command(right_arm_joints, p1_jtp)
    # rospy.sleep(10)
    # victor.send_joint_command(right_arm_joints, p2_jtp)
    # rospy.sleep(10)

    # victor.plan_to_joint_config("right_arm", p1)
    # victor.plan_to_joint_config("right_arm", p2)
    # victor.plan_to_joint_config("right_arm", p1)
    # victor.plan_to_joint_config("right_arm", p2)
    # rospy.sleep(0.5)
    # victor.move_to("arms out")

    # victor.move_to("impedance switch")

    # print("press enter if prompted")
    #
    # # # Plan to joint config
    # myinput("Plan to joint config?")
    # victor.plan_to_joint_config(victor.right_arm_group, [0.35, 1, 0.2, -1, 0.2, -1, 0])
    # #
    # # # Plan to pose
    # myinput("Plan to pose 1?")
    # victor.plan_to_pose(victor.right_arm_group, victor.right_tool_name, [0.6, -0.2, 1.0, 4, 1, 0])
    #
    # # Or you can use a geometry msgs Pose
    # myinput("Plan to pose 2?")
    # pose = Pose()
    # pose.position.x = 0.7
    # pose.position.y = -0.2
    # pose.position.z = 1.0
    # q = quaternion_from_euler(np.pi, 0, 0)
    # pose.orientation.x = q[0]
    # pose.orientation.y = q[1]
    # pose.orientation.z = q[2]
    # pose.orientation.w = q[3]
    # victor.plan_to_pose(victor.right_arm_group, victor.right_tool_name, pose)
    #
    # # # Or with cartesian planning
    # myinput("Cartersian motion back to pose 3?")
    # victor.plan_to_position_cartesian(victor.right_arm_group, victor.right_tool_name, [0.9, -0.4, 0.9], step_size=0.01)
    # victor.plan_to_position_cartesian(victor.right_arm_group, victor.right_tool_name, [0.7, -0.4, 0.9], step_size=0.01)
    #
    # # Move hand straight works either with jacobian following
    # myinput("Follow jacobian to pose 2?")
    # victor.store_current_tool_orientations([victor.right_tool_name])
    # victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[0.7, -0.4, 0.6]]])
    # victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[0.8, -0.4, 1.0]]])
    # victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[1.1, -0.4, 0.9]]])
    # victor.follow_jacobian_to_position(group_name=victor.right_arm_group,
    #                                    tool_names=[victor.right_tool_name],
    #                                    preferred_tool_orientations=[quaternion_from_euler(np.pi, 0, 0)],
    #                                    points=[[[1.1, -0.2, 0.8]]])

    # roscpp_initializer.shutdown()
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
