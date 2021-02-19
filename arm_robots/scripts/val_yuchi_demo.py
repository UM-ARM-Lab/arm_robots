#! /usr/bin/env python
import colorama
import numpy as np

from arc_utilities import ros_init
from arm_robots.hdt_michigan import Val
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

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

    config = {
        'joint56': -1.6,
        'joint57': 0.2,
        'joint41': 0.77,
        'joint42': -0.15,
        'joint43': -0.5,
        'joint44': -0.55,
        'joint45': 0.06,
        'joint46': -0.37,
        'joint47': 0.05,
        'joint1':  0.75,
        'joint2':  0.0,
        'joint3':  0.0,
        'joint4':  -0.6,
        'joint5':  0.35,
        'joint6':  0.0,
        'joint7':  -0.3,
    }
    # myinput('press enter')
    val.plan_to_joint_config('both_arms', config)
    val.store_current_tool_orientations([val.right_tool_name, val.left_tool_name])

    left_pos_1 = [0.72, 0.2, 0.25]
    left_pos_2 = [0.6, 0.1, 0.25]
    left_pos_3 = [0.72, 0.0, 0.25]

    right_pos_1 = [0.72, 0.1, 0.25]
    right_pos_2 = [0.6, 0.0, 0.25]
    right_pos_3 = [0.72, -0.1, 0.25]
    for i in range(3):
        myinput('press enter')
        val.follow_jacobian_to_position('both_arms', [val.right_tool_name, val.left_tool_name],
                                        [[right_pos_3], [left_pos_3]], vel_scaling=1.0)
        myinput('press enter')
        val.follow_jacobian_to_position('both_arms', [val.right_tool_name, val.left_tool_name],
                                        [[right_pos_1], [left_pos_1]], vel_scaling=1.0)
        myinput('press enter')
        val.follow_jacobian_to_position('both_arms', [val.right_tool_name, val.left_tool_name],
                                        [[right_pos_2], [left_pos_2]], vel_scaling=1.0)

    val.disconnect()
    ros_init.shutdown()


if __name__ == "__main__":
    main()
