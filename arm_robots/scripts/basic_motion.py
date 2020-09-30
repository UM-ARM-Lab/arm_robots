#! /usr/bin/env python
import colorama
import numpy as np

import actionlib
import moveit_commander
import rospy
from arc_utilities.ros_helpers import Listener
from arm_robots.victor import Victor
from control_msgs.msg import FollowJointTrajectoryFeedback
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from victor_hardware_interface.victor_utils import Stiffness
from victor_hardware_interface_msgs.msg import ControlMode

effort_thresholds = np.array([
    150,  # victor_left_arm_joint_1
    100,  # victor_left_arm_joint_2
    100,  # victor_left_arm_joint_3
    80,  # victor_left_arm_joint_4
    80,  # victor_left_arm_joint_5
    40,  # victor_left_arm_joint_6
    10,  # victor_left_arm_joint_7
    1,  # victor_left_gripper_fingerA_joint_2
    1,  # victor_left_gripper_fingerA_joint_3
    1,  # victor_left_gripper_fingerA_joint_4
    1,  # victor_left_gripper_fingerB_joint_2
    1,  # victor_left_gripper_fingerB_joint_3
    1,  # victor_left_gripper_fingerB_joint_4
    1,  # victor_left_gripper_fingerB_knuckle
    1,  # victor_left_gripper_fingerC_joint_2
    1,  # victor_left_gripper_fingerC_joint_3
    1,  # victor_left_gripper_fingerC_joint_4
    1,  # victor_left_gripper_fingerC_knuckle
    150,  # victor_right_arm_joint_1
    100,  # victor_right_arm_joint_2
    100,  # victor_right_arm_joint_3
    80,  # victor_right_arm_joint_4
    80,  # victor_right_arm_joint_1
    40,  # victor_right_arm_joint_6
    10,  # victor_right_arm_joint_7
    1,  # victor_right_gripper_fingerA_joint_2
    1,  # victor_right_gripper_fingerA_joint_3
    1,  # victor_right_gripper_fingerA_joint_4
    1,  # victor_right_gripper_fingerB_joint_2
    1,  # victor_right_gripper_fingerB_joint_3
    1,  # victor_right_gripper_fingerB_joint_4
    1,  # victor_right_gripper_fingerB_knuckle
    1,  # victor_right_gripper_fingerC_joint_2
    1,  # victor_right_gripper_fingerC_joint_3
    1,  # victor_right_gripper_fingerC_joint_4
    1,  # victor_right_gripper_fingerC_knuckle
])

debug = True


def myinput(msg):
    global debug
    if not debug:
        input(msg)


def main():
    np.set_printoptions(suppress=True, precision=0, linewidth=200)
    colorama.init(autoreset=True)

    joint_state_topic = ['joint_states:=/victor/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('basic_motion', anonymous=True)

    victor = Victor(execute_by_default=True)

    # victor.move_to_impedance_switch(actually_switch=True)
    victor.set_right_arm_control_mode(ControlMode.JOINT_IMPEDANCE, stiffness=Stiffness.STIFF, vel=0.5)

    # Plan to joint config
    print("press enter when prompted...")
    myinput("Plan to joint config?")
    victor.plan_to_joint_config("right_arm", [0.35, 1, 0.2, -1, 0.2, -1, 0])

    # Plan to joint config with a stop condition
    myinput("Plan to joint config, with max force?")

    ext_wrench = victor.get_right_motion_status().estimated_external_wrench
    start_force = np.array([ext_wrench.x, ext_wrench.y, ext_wrench.z])
    force_trigger = 8.0

    def _stop_condition(feedback: FollowJointTrajectoryFeedback):
        wrench = victor.get_right_motion_status().estimated_external_wrench
        cur_force = np.array([wrench.x, wrench.y, wrench.z])
        new_force = np.linalg.norm(start_force - cur_force)
        msg = f"New force {new_force} exceeded threshold {force_trigger}! Canceling all goals."
        stop = new_force > force_trigger
        return stop, msg

    victor.plan_to_joint_config("right_arm", [-0.27, 1, 0.2, -1, 0.2, -1, 0], stop_condition=_stop_condition)
    return

    # Plan to pose
    myinput("Plan to pose 1?")
    victor.plan_to_pose("right_arm", "right_tool_placeholder", [0.6, -0.2, 1.0, 4, 1, 0])

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
    victor.plan_to_pose("right_arm", "right_tool_placeholder", pose)

    # # Or with cartesian planning
    myinput("Cartersian motion back to pose 3?")
    victor.plan_to_position_cartesian("right_arm", "right_tool_placeholder", [0.9, -0.4, 0.9], step_size=0.04)

    # Move hand straight works either with jacobian following
    myinput("Follow jacobian to pose 2?")
    victor.follow_jacobian_to_position("right_arm", "right_tool_placeholder", [1.05, 0.15, 1.0])


if __name__ == "__main__":
    main()
