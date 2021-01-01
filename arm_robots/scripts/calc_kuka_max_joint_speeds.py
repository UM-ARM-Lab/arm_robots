#! /usr/bin/env python
import math

import colorama
import numpy as np
import roscpp_initializer

import rospy
import sensor_msgs
from arc_utilities.listener import Listener
from arc_utilities.ros_init import rospy_and_cpp_init
from arm_robots.victor import Victor, RIGHT_ARM_JOINT_NAMES
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from victor_hardware_interface.victor_utils import ControlMode, MotionStatus
import time
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler