import numpy as np
from victor_hardware_interface_msgs.msg import Robotiq3FingerCommand
import rospy


############################################################
#                     JOINT NAMES
############################################################
# NOTE: moveit has all of this stuff specified, can we use moveit here? without relying on its execution capabilities?
LEFT_GRIPPER_JOINT_NAMES = (
    "left_finger_a",
    "left_finger_b",
    "left_finger_c",
    "left_scissor",
)
RIGHT_GRIPPER_JOINT_NAMES = (
    "right_finger_a",
    "right_finger_b",
    "right_finger_c",
    "right_scissor",
)
LEFT_ARM_JOINT_NAMES = (
    'victor_left_arm_joint_1',
    'victor_left_arm_joint_2',
    'victor_left_arm_joint_3',
    'victor_left_arm_joint_4',
    'victor_left_arm_joint_5',
    'victor_left_arm_joint_6',
    'victor_left_arm_joint_7',
)
RIGHT_ARM_JOINT_NAMES = (
    'victor_right_arm_joint_1',
    'victor_right_arm_joint_2',
    'victor_right_arm_joint_3',
    'victor_right_arm_joint_4',
    'victor_right_arm_joint_5',
    'victor_right_arm_joint_6',
    'victor_right_arm_joint_7',
)
BOTH_ARM_JOINT_NAMES = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
ALL_JOINT_NAMES = BOTH_ARM_JOINT_NAMES + LEFT_GRIPPER_JOINT_NAMES + RIGHT_GRIPPER_JOINT_NAMES


############################################################
#                     LIMITS
############################################################

# Calculated from scipt and copied to kuka_iiwa_interface joint_limits.yaml
KUKA_MAX_JOINT_VELOCITIES_DEG_S = (100.0, 100.0, 100.0, 130.0, 140.0, 180.0, 180.0)
KUKA_MAX_JOINT_VELOCITIES = (s * np.pi / 180 for s in KUKA_MAX_JOINT_VELOCITIES_DEG_S)
KUKA_MAX_JOINT_ACCEL_DEG_S2 = (65, 70, 500, 500, 1500, 1200, 2000)
KUKA_MAX_JOINT_ACCEL = (s * np.pi / 180 for s in KUKA_MAX_JOINT_ACCEL_DEG_S2)

# Set manually to give decent behavior with our controller. May need updating
KUKA_MIN_PATH_JOINT_POSITION_TOLERANCE = (0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)
KUKA_FULL_SPEED_PATH_JOINT_POSITION_TOLERANCE = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3)
KUKA_GOAL_JOINT_POSITION_TOLERANCE = (0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)

KUKA_MIN_PATH_JOINT_IMPEDANCE_TOLERANCE = (0.1, 0.1, 0.1, 0.07, 0.05, 0.05, 0.05)
KUKA_FULL_SPEED_PATH_JOINT_IMPEDANCE_TOLERANCE = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3)
KUKA_GOAL_JOINT_IMPEDANCE_TOLERANCE = (0.1, 0.1, 0.1, 0.07, 0.05, 0.05, 0.05)



############################################################
#                     DEFAULTS
############################################################

def default_robotiq_command(speed=1.0, force=1.0):
    cmd = Robotiq3FingerCommand()
    cmd.header.stamp = rospy.Time.now()
    cmd.finger_a_command.speed = speed
    cmd.finger_a_command.force = force
    cmd.finger_b_command.speed = speed
    cmd.finger_b_command.force = force
    cmd.finger_c_command.speed = speed
    cmd.finger_c_command.force = force
    cmd.scissor_command.speed = speed
    cmd.scissor_command.force = force
    return cmd
