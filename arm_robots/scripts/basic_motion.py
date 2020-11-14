#! /usr/bin/env python
import colorama
import numpy as np
import roscpp_initializer

import rospy
from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.victor import Victor
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

ask_before_moving = True


def myinput(msg):
    global ask_before_moving
    if not ask_before_moving:
        input(msg)


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("basic_motion")

    victor = Victor()
    victor.connect()

    victor.open_left_gripper()
    rospy.sleep(2)
    victor.close_left_gripper()
    rospy.sleep(2)
    victor.open_right_gripper()
    rospy.sleep(2)
    victor.close_right_gripper()
    rospy.sleep(2)

    print("press enter if prompted")

    # # Plan to joint config
    myinput("Plan to joint config?")
    victor.plan_to_joint_config(victor.right_arm_group, [0.35, 1, 0.2, -1, 0.2, -1, 0])
    #
    # # Plan to pose
    myinput("Plan to pose 1?")
    victor.plan_to_pose(victor.right_arm_group, victor.right_tool_name, [0.6, -0.2, 1.0, 4, 1, 0])

    # Or you can use a geometry msgs Pose
    myinput("Plan to pose 2?")
    pose = Pose()
    pose.position.x = 0.7
    pose.position.y = -0.2
    pose.position.z = 1.0
    q = quaternion_from_euler(np.pi, 0, 0)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    victor.plan_to_pose(victor.right_arm_group, victor.right_tool_name, pose)

    # # Or with cartesian planning
    myinput("Cartersian motion back to pose 3?")
    victor.plan_to_position_cartesian(victor.right_arm_group, victor.right_tool_name, [0.9, -0.4, 0.9], step_size=0.01)
    victor.plan_to_position_cartesian(victor.right_arm_group, victor.right_tool_name, [0.7, -0.4, 0.9], step_size=0.01)

    # Move hand straight works either with jacobian following
    myinput("Follow jacobian to pose 2?")
    victor.store_current_tool_orientations([victor.right_tool_name])
    victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[0.7, -0.4, 0.6]]])
    victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[0.8, -0.4, 1.0]]])
    victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[1.1, -0.4, 0.9]]])
    victor.follow_jacobian_to_position(group_name=victor.right_arm_group,
                                       tool_names=[victor.right_tool_name],
                                       preferred_tool_orientations=[quaternion_from_euler(np.pi, 0, 0)],
                                       points=[[[1.1, -0.2, 0.8]]])

    roscpp_initializer.shutdown()


if __name__ == "__main__":
    main()
