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
#                     NAMED POSITIONS
############################################################

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
