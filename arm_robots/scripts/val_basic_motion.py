#! /usr/bin/env python
import colorama
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

from arc_utilities import ros_init
from arm_robots.hdt_michigan import Val

ask_before_moving = True


def myinput(msg):
    global ask_before_moving
    if ask_before_moving:
        input(msg)


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    ros_init.rospy_and_cpp_init("basic_motion")

    val = Val(raise_on_failure=True)
    val.connect()

    val.open_left_gripper()
    val.close_left_gripper()
    val.open_right_gripper()
    val.close_right_gripper()

    print("press enter if prompted")

    # Plan to joint config
    myinput("Plan to home joint config?")
    val.plan_to_joint_config('both_arms', 'home')

    # Plan to pose
    myinput("Plan to pose 1?")
    val.plan_to_pose(val.right_arm_group, val.right_tool_name, [0.3, 0.8, 0.5, -np.pi / 2, np.pi / 2, 0])

    # Or you can use a geometry msgs Pose
    myinput("Plan to pose 2?")
    pose = Pose()
    pose.position.x = 0.2
    pose.position.y = 0.5
    pose.position.z = 0.1
    q = quaternion_from_euler(-np.pi, 0, 0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    val.plan_to_pose(val.right_arm_group, val.right_tool_name, pose)

    # or with cartesian planning
    myinput("cartesian motion to pose 3?")
    val.plan_to_position_cartesian(val.right_arm_group, val.right_tool_name, [0.4, 0.3, 0.1], step_size=0.01)

    myinput("follow jacobian to pose 4?")
    val.store_current_tool_orientations([val.right_tool_name])
    val.follow_jacobian_to_position('both_arms', [val.right_tool_name], [[[0.3, 0.5, 0.3]]], vel_scaling=1.0)

    val.disconnect()
    ros_init.shutdown()


 __name__ == "__main__":
  main()
