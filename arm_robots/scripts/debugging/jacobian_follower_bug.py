"""
Created to debug: https://github.com/UM-ARM-Lab/arm_robots/issues/70
This file can be deleted when the issue is resolved
"""


#! /usr/bin/env python
import colorama
import numpy as np
import roscpp_initializer

import rospy
from arc_utilities.ros_init import rospy_and_cpp_init

from arc_utilities import ros_init
from arm_robots.victor import Victor
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from victor_hardware_interface_msgs.msg import ControlMode

ask_before_moving = False


def myinput(msg):
    global ask_before_moving
    if ask_before_moving:
        input(msg)


@ros_init.with_ros("victor_basic_motion")
def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    victor = Victor(display_goals=False)
    victor.set_control_mode(control_mode=ControlMode.JOINT_POSITION, vel=0.1)
    victor.connect()

    victor.plan_to_position_cartesian(victor.right_arm_group, victor.right_tool_name, [0.7, -0.4, 0.9], step_size=0.01)

    # Move hand straight works either with jacobian following
    myinput("Follow jacobian to pose 2?")
    victor.store_current_tool_orientations([victor.right_tool_name])
    victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[0.7, -0.4, 0.6]]])
    # rospy.sleep(5)
    victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[0.8, -0.4, 1.0]]])
    # victor.follow_jacobian_to_position(victor.right_arm_group, [victor.right_tool_name], [[[1.1, -0.4, 0.9]]])
    # result = victor.follow_jacobian_to_position(group_name=victor.right_arm_group,
    #                                             tool_names=[victor.right_tool_name],
    #                                             preferred_tool_orientations=[quaternion_from_euler(np.pi, 0, 0)],
    #                                             points=[[[1.1, -0.2, 0.8]]])



if __name__ == "__main__":
    main()
