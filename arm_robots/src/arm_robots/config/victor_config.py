import numpy as np
from victor_hardware_interface_msgs.msg import Robotiq3FingerCommand
import rospy

KUKA_MAX_JOINT_VELOCITIES_DEG_S = [100.0, 100.0, 100.0, 130.0, 140.0, 180.0, 180.0]  # Calculated from scipt
KUKA_MAX_JOINT_VELOCITIES = [s * np.pi / 180 for s in KUKA_MAX_JOINT_VELOCITIES_DEG_S]
NAMED_POSITIONS = {
    "impedance switch": {"right_arm": [0.724, 0.451, 0.94, -1.425, 0.472, 0.777, -0.809],
                         "left_arm": [-0.694, 0.14, -0.229, -1.11, -0.512, 1.272, 0.077]},
    "arms up": {"left_arm": [-np.pi / 2, np.pi / 2, 0, 0, 0, 0, 0],
                "right_arm": [np.pi / 2, np.pi / 2, 0, 0, 0, 0, 0]},
    "hug": {"right_arm": [0.311, 1.561, 0.282, -1.296, 0.137, 0.493, 0.112],
            "left_arm": [0.604, 1.568, -0.021, -1.164, 0.355, 0.173, 0.297]},
    "arms out": {"left_arm": [0, 0, 0, 0, 0, 0, 0],
                 "right_arm": [0, 0, 0, 0, 0, 0, 0]},
    "handshake": {"right_arm": [-0.336, 0.105, 0.061, -1.151, -0.01, 0.577, -0.675]},
}


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