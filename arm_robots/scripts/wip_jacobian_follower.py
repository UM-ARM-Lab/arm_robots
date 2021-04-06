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
    if ask_before_moving:
        input(msg)


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    rospy_and_cpp_init("basic_motion")

    victor = Victor()
    victor.connect()

    pose = Pose()
    pose.position.x = 1.9
    pose.position.y = 2.0
    pose.position.z = 1.0
    pose.orientation.x = -0.05594805960241513
    pose.orientation.y = -0.7682472566147173
    pose.orientation.z = -0.6317937464624142
    pose.orientation.w = 0.08661771909760922

    config = [-0.28, 0.9, 0, -0.5, 0, 1.0, 0]
    fk_pose = victor.jacobian_follower.fk(config, "right_arm")
    iks = victor.jacobian_follower.compute_IK_solutions(pose, "right_arm")

    roscpp_initializer.shutdown()


if __name__ == "__main__":
    main()
